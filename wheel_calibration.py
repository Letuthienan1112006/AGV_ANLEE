"""
wheel_calibration.py - do dap ung PWM/DELTA thuc te cua tung banh qua
nhieu muc STEER/SPEED, thay vi doan 1 he so nhan RIGHT_WHEEL_PWM_TRIM
duy nhat (hien =1.30, chi kiem chung o dung 1 diem van hanh hom qua).

CHAY TREN LAB, KHONG CHAY DUOC O NHA (can ket noi bridge that qua
127.0.0.1:54321 tren chinh Jetson). Copy file nay vao Jetson, chay TRUC
TIEP tren Jetson (khong phai tu laptop):
    scp wheel_calibration.py agv@<jetson-ip>:~/agv_git_clean/
    ssh agv@<jetson-ip>
    cd ~/agv_git_clean
    # 1. Xe ban banh len khoi mat dat (KHONG dat xuong dat cho lan dau -
    #    day la do dap ung banh don le, chua can test lan duong that)
    # 2. Chay bridge o 1 terminal: python3 real_car_socket.py
    # 3. Chay calib o terminal khac: python3 wheel_calibration.py
    # 4. Doc lai output - la 1 bang PWM vs DELTA cho tung banh o tung
    #    muc STEER da test.

Ket qua mong doi: neu banh phai van "yeu" hon banh trai theo dung ty le
o moi muc SPEED (khong chi rieng luc SPEED~25 nhu hom qua do duoc), thi
RIGHT_WHEEL_PWM_TRIM=1.30 (1 he so co dinh) la du dai dien. Neu ty le
lech thay doi theo SPEED (vd yeu hon nhieu luc SPEED thap, gan nhu bang
nhau luc SPEED cao), thi can 1 ham trim(speed) thay vi hang so - se sua
firmware dua tren bang so lieu nay.
"""
import socket
import time

BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 54321

# Cac muc (steer, speed, giu_trong_giay) can do - speed dung thang lenh
# 0-25 nhu firmware nhan (SAU clamp cua bridge, KHONG phai thang raw
# 0-106 cua AI). steer=0 truoc de co baseline ca 2 banh thang deu.
TEST_POINTS = [
    (0, 10, 4),
    (0, 15, 4),
    (0, 20, 4),
    (0, 25, 4),
    (-20, 15, 4),   # steer am = re trai toi da (theo quy uoc da xac nhan
    (-20, 25, 4),   # hom qua: TARGET phai > TARGET trai khi steer am)
    (20, 15, 4),    # steer duong = re phai toi da
    (20, 25, 4),
]

SETTLE_SEC = 1.5   # bo qua transient dau moi diem, chi lay so lieu on dinh


def main():
    print("Dang ket noi bridge...")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.settimeout(5)
    s.connect((BRIDGE_HOST, BRIDGE_PORT))
    s.settimeout(1.0)   # sau khi connect, giam timeout cho vong lap send/recv
    print("Da ket noi. Bat dau do - THEO DOI dong thoi log cua bridge de "
          "doi chieu DELTA/PWM (bridge in ra [STM32 RX] ...).\n")

    def send_and_drain(cmd_bytes):
        # Bridge dung protocol 2 chieu: sau moi lenh nhan duoc, no DOC
        # 1 frame camera roi gui tra ve tren CUNG socket. Neu client chi
        # gui ma khong doc lai, buffer gui phia bridge day dan, sendall()
        # ben bridge timeout (SOCKET_POLL_TIMEOUT) -> bridge tuong client
        # rot, tu shutdown (da gap that 2026-09-08: "[NET] Gui frame loi:
        # timed out"). Phai recv() drain sau moi lan send.
        s.sendall(cmd_bytes)
        try:
            s.recv(65536)
        except socket.timeout:
            pass

    try:
        for steer, speed, hold_sec in TEST_POINTS:
            cmd = f"{steer} {speed}\n".encode()
            t_start = time.time()
            print(f"=== STEER={steer} SPEED={speed} "
                  f"(bat dau {time.strftime('%H:%M:%S')}, "
                  f"bo qua {SETTLE_SEC}s dau, lay {hold_sec - SETTLE_SEC:.1f}s sau) ===")
            while time.time() - t_start < hold_sec:
                send_and_drain(cmd)
                time.sleep(0.1)   # gui deu, tuong tu tan so patrol_robot that

        print("\n=== XONG - ve STEER=0 SPEED=0 de dung an toan ===")
        for _ in range(10):
            send_and_drain(b"0 0\n")
            time.sleep(0.1)
    finally:
        s.close()
        print("Da dong ket noi. Doc lai log bridge (vd grep 'STM32 RX' "
              "trong khung gio vua roi) de lay DELTA/PWM cho tung diem.")


if __name__ == "__main__":
    main()
