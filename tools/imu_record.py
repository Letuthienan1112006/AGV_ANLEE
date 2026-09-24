#!/usr/bin/env python3
"""
imu_record.py - Ghi /imu/data cua MTi-630R ra CSV phang, chay tren laptop
trong moi truong ROS 2 da source.

Ly do khong dung rosbag: file sinh ra phai doc duoc bang python3 thuong
de so sanh voi wheel_telem.csv cua xe, va laptop khong co numpy ngoai moi
truong ROS. Mot CSV hai cot thoi gian giai quyet xong chuyen do.

Cach dung:

    source /opt/ros/humble/setup.bash
    source <ws>/install/setup.bash
    ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py   # terminal 1
    python3 tools/imu_record.py imu_log.csv                     # terminal 2

Quy trinh do:
  1. Bat driver, bat file nay, roi bat run.sh tren xe. Bat file nay TRUOC
     khi xe bat dau chay.
  2. **Trong luc xe con dung yen, xoay than xe MOT cu ro rang va khong
     doi xung** - vi du quay +40 do, giu 3 giay, ve -15 do, giu, roi ve 0.
     Day la moc dong bo: ca hai cam bien deu thay chuyen dong nay, va
     sync_imu.py dung no de tim do lech thoi gian giua hai dong du lieu.

     **Nen xoay mot cu, dung lac theo nhip.** Da do tren du lieu tong
     hop: cu xoay khong doi xung cho do noi dinh 0.24, cu lac hinh sin chu
     ky 4s cho 0.18, nguong can la 0.15. Cu lac VAN dung duoc - diem bat
     dau va ket thuc cua no pha tinh tuan hoan - nhung bien an toan mong
     hon, va dinh tuong quan cua tin hieu tuan hoan se bien mat neu cu lac
     keo dai het cua so.

     **Phai lam luc xe dung yen.** Khi xe da chay, lenh lai lam yaw tang
     thanh mot duong doc dai; hai duong doc khop gan nhu nhau o moi do
     lech va dinh tuong quan bien mat. sync_imu.py vi vay chi dung doan
     truoc luc banh lan.
  3. Cho xe chay binh thuong, voi lenh lai doi buoc nhieu lan.
  4. Ctrl+C ca hai ben, roi chay sync_imu.py.

Hai cot thoi gian:
  t_wall  thoi diem nhan goi, dong ho he thong laptop
  t_msg   header.stamp cua cam bien - chinh xac hon cho do trong cung mot
          dong du lieu, nhung khong cung goc voi dong ho cua xe
"""

import math
import os
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu
except ImportError as error:
    sys.stderr.write(
        "Khong import duoc rclpy/sensor_msgs: {}\n"
        "Chay 'source /opt/ros/humble/setup.bash' truoc.\n".format(error)
    )
    raise SystemExit(1)


TOPIC = "/imu/data"
HEADER = "t_wall,t_msg,wx,wy,wz,ax,ay,az,roll,pitch,yaw\n"

# Neu sau khoang nay chua co goi nao thi bao cho nguoi dung biet, thay vi
# im lang sinh ra mot file rong.
FIRST_MSG_WARN_SEC = 3.0


def quat_to_rpy(x, y, z, w):
    """Quaternion -> roll, pitch, yaw (do). Yaw trong [0, 360)."""
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.degrees(math.atan2(sinr, cosr))

    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.degrees(math.asin(sinp))

    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.degrees(math.atan2(siny, cosy)) % 360.0

    return roll, pitch, yaw


class ImuLogger(Node):
    def __init__(self, path):
        super().__init__("imu_record")

        self.path = path
        self.handle = open(path, "w")
        self.handle.write(HEADER)
        self.count = 0
        self.first_t = None
        self.last_t = None
        self.warned = False
        self.started = time.time()

        # Driver publish best-effort; khop QoS neu khong se khong nhan goi
        # nao ma cung khong bao loi.
        qos = QoSProfile(depth=200)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.create_subscription(Imu, TOPIC, self.on_imu, qos)
        self.create_timer(1.0, self.on_tick)

        print("[IMU-REC] Ghi {} <- {}".format(path, TOPIC), flush=True)

    def on_imu(self, msg):
        t_wall = time.time()
        stamp = msg.header.stamp
        t_msg = stamp.sec + stamp.nanosec * 1e-9

        roll, pitch, yaw = quat_to_rpy(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )

        self.handle.write(
            "{:.6f},{:.6f},"
            "{:+.6f},{:+.6f},{:+.6f},"
            "{:+.4f},{:+.4f},{:+.4f},"
            "{:+.4f},{:+.4f},{:.4f}\n".format(
                t_wall, t_msg,
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
                roll, pitch, yaw,
            )
        )

        self.count += 1
        if self.first_t is None:
            self.first_t = t_wall
        self.last_t = t_wall

        # Flush thua thot: mat vai goi cuoi khi Ctrl+C khong sao, nhung
        # flush moi goi o 95 Hz thi phi.
        if self.count % 50 == 0:
            self.handle.flush()

    def on_tick(self):
        if self.count == 0:
            if (not self.warned
                    and time.time() - self.started > FIRST_MSG_WARN_SEC):
                self.warned = True
                print(
                    "[IMU-REC] Chua nhan goi nao tren {}.\n"
                    "          Kiem tra: ros2 topic hz {}\n"
                    "          Driver da Activated chua?".format(
                        TOPIC, TOPIC
                    ),
                    flush=True,
                )
            return

        span = self.last_t - self.first_t
        rate = self.count / span if span > 0 else 0.0
        print(
            "\r[IMU-REC] {} goi  {:.1f}s  {:.1f} Hz".format(
                self.count, span, rate
            ),
            end="",
            flush=True,
        )

    def finish(self):
        self.handle.flush()
        self.handle.close()

        span = 0.0 if self.first_t is None else self.last_t - self.first_t
        rate = self.count / span if span > 0 else 0.0
        print(
            "\n[IMU-REC] Xong: {} goi, {:.1f}s, {:.1f} Hz -> {} ({} bytes)".format(
                self.count, span, rate, self.path,
                os.path.getsize(self.path),
            ),
            flush=True,
        )

        if self.count == 0:
            print(
                "[IMU-REC] File rong. Khong co du lieu de dong bo.",
                flush=True,
            )
        elif rate < 50.0:
            print(
                "[IMU-REC] CANH BAO: {:.1f} Hz, thap hon nhieu so voi 95 Hz "
                "mong doi. Mat goi thi phep do do tre se lech.".format(rate),
                flush=True,
            )


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else "imu_log.csv"

    rclpy.init()
    node = ImuLogger(path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
