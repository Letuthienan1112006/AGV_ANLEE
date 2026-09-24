"""
BUOC 1 - Test co khi thuan tuy, KHONG dung camera/AI de tinh lenh lai.

Gui thang lenh "0 <speed>" (steer=0 co dinh) qua bridge (real_car_socket.py)
trong 1 khoang thoi gian co dinh, roi tu dong dung. Van di qua bridge nen
LiDAR safety + watchdog STM32 van hoat dong binh thuong - chi bo qua
phan bam vach AI/segmentation de cach ly xem CO KHI/FIRMWARE co tu gay
lech khong, khong lien quan gi toi nhan dien.
Bridge VAN chup/gui anh theo giao thuc; bai test chi doc het, khong phan tich anh.

CACH DUNG:
  1. Chay real_car_socket.py (bridge) truoc, o cua so/terminal khac:
       cd ~/agv_git_clean && python3 real_car_socket.py
  2. Dat xe tren mat dat that (KHONG ke banh), danh dau diem xuat phat
     bang bang keo/phan.
  3. Chay file nay: python3 test_straight_mechanical.py
  4. Do khoang cach xe lech khoi duong thang (cm) o cuoi doan chay.
  5. Lap lai it nhat 3 lan, ghi lai tung lan de lay trung binh
     (khong ket luan tu 1 lan chay duy nhat).

Moi lenh phai nhan het mot frame JSON truoc khi gui lenh tiep theo.
AN TOAN: Ctrl+C yeu cau dung va dong ket noi; khong thay the nut dung
khan cap vat ly. Phan hoi socket KHONG xac nhan banh xe da dung.
"""
import json
import socket
import time
import sys

SOCKET_IP = "127.0.0.1"
SOCKET_PORT = 54321

# Chinh 2 gia tri nay truoc khi chay:
# 2026-09-15: nang tu 8 len 15. Bai chay o speed=8 (target=12 tick) cho
# PWM leo toi 89% tran cau truc (37/41) ma CA HAI banh gan nhu khong quay
# (tong ca bai: trai +4 xung, phai +1 xung tren 2.4s) - khong phai lech
# trai/phai, ma la ca xe khong thang duoc ma sat tinh khi chiu trong luong
# that tren dat. feedForwardPWM di qua goc toa do (sua 2026-09-10) nen o
# target thap, feedforward chi ~27 PWM - co the duoi nguong khoi dong that
# cua dong co khi co tai. speed=15 khop voi toc do co ban cac run road/
# dang dung, va da thay tao ra chuyen dong that (yaw doi ~40 do trong vai
# giay) trong cac run do - phep so sanh truc tiep xem 8 co qua thap hay
# khong.
TEST_SPEED = 15          # toc do co dinh (0-20)

# 2026-09-12: rut tu 5.0 xuong 2.5 cho LAN CHAY DAU TREN MAT DAT. Xe chua
# bao gio duoc thu ha banh sau khi banh phai chi dat 25% muc tieu; neu no
# quet mot vong thi 5 giay la qua dai de voi tay tat.
#
# Co gia phai tra, va can biet truoc: quang duong ngan thi do lech ngang
# cung ngan, nen sai so tuong doi cua phep do bang thuoc lon hon. Neu sau
# 3 lan chay do lech nho den muc khong doc duoc chac chan, hay nang lai
# len 5.0 - luc do da biet xe khong bo chay. DUNG ket luan "xe di thang"
# tu mot do lech nho hon do chinh xac cua thuoc.
TEST_DURATION_SEC = 2.5

FRAME_TIMEOUT_SEC = 1.0  # gioi han TOAN BO send + frame, khong reset moi chunk
STOP_TIMEOUT_SEC = 0.5
MAX_FRAME_BYTES = 2_000_000
COMMAND_INTERVAL_SEC = 0.1


class FrameProtocolError(RuntimeError):
    """Bridge response is not exactly one bounded JSON image frame."""


def _remaining(deadline, clock):
    remaining = deadline - clock()
    if remaining <= 0:
        raise socket.timeout("qua han nhan tron frame bridge")
    return remaining


def recv_frame(sock, deadline, max_bytes=MAX_FRAME_BYTES, clock=time.monotonic):
    """Read one JSON object, respecting escaped strings and TCP fragmentation.

    The bridge has no length prefix or delimiter. Only one request may be in
    flight. A closing brace in a string is not the end of a frame; any extra
    non-whitespace data in the same receive is a protocol error.
    """
    data = bytearray()
    depth = 0
    in_string = False
    escaped = False
    started = False
    while True:
        sock.settimeout(_remaining(deadline, clock))
        chunk = sock.recv(min(65536, max_bytes - len(data) + 1))
        _remaining(deadline, clock)
        if not chunk:
            raise ConnectionError("bridge dong ket noi khi frame chua day du")
        data.extend(chunk)
        if len(data) > max_bytes:
            raise FrameProtocolError("frame bridge vuot gioi han kich thuoc")
        for i, byte in enumerate(chunk):
            if not started:
                if byte in b" \t\r\n":
                    continue
                if byte != ord("{"):
                    raise FrameProtocolError("frame bridge phai la JSON object")
                started = True
            if in_string:
                if escaped:
                    escaped = False
                elif byte == ord("\\"):
                    escaped = True
                elif byte == ord('"'):
                    in_string = False
            elif byte == ord('"'):
                in_string = True
            elif byte in b"{[":
                depth += 1
            elif byte in b"}]":
                depth -= 1
                if depth == 0:
                    if chunk[i + 1:].strip():
                        raise FrameProtocolError("du lieu du sau frame bridge")
                    try:
                        payload = json.loads(data.decode("utf-8"))
                    except (UnicodeError, ValueError, RecursionError) as error:
                        raise FrameProtocolError("frame JSON bridge khong hop le") from error
                    if not isinstance(payload.get("Img"), str) or not payload["Img"]:
                        raise FrameProtocolError("frame bridge thieu anh Img")
                    _remaining(deadline, clock)
                    return payload


class BridgeClient:
    def __init__(self, sock, clock=time.monotonic):
        self.sock = sock
        self.clock = clock
        self.synchronized = True

    def exchange(self, steer, speed, timeout=FRAME_TIMEOUT_SEC):
        if not self.synchronized:
            raise FrameProtocolError("ket noi da mat dong bo, khong gui them lenh")
        deadline = self.clock() + timeout
        # Even a partial send invalidates synchronization. Do not enqueue STOP
        # behind an unfinished request or an unread image after an exception.
        self.synchronized = False
        self.sock.settimeout(_remaining(deadline, self.clock))
        self.sock.sendall(("%d %d" % (steer, speed)).encode("ascii"))
        payload = recv_frame(self.sock, deadline, clock=self.clock)
        self.synchronized = True
        return payload

    def stop_and_close(self):
        """Best-effort STOP on a synchronized stream; otherwise disconnect now.

        True means the bridge replied to STOP, not that motors were measured
        stationary. On a broken stream, closing lets the bridge invalidate AI.
        """
        try:
            if self.synchronized:
                self.exchange(0, 0, STOP_TIMEOUT_SEC)
                return True
            return False
        except (OSError, FrameProtocolError):
            return False
        finally:
            self.sock.close()


def check_safety(payload):
    if payload.get("blocked") or payload.get("lifted"):
        raise FrameProtocolError("bridge bao blocked/lifted; huy phep do co khi")


def drive(client, speed, duration, clock=time.monotonic, sleep=time.sleep):
    deadline = clock() + duration
    while clock() < deadline:
        cycle_start = clock()
        payload = client.exchange(0, speed, min(FRAME_TIMEOUT_SEC, deadline - cycle_start))
        check_safety(payload)
        delay = min(COMMAND_INTERVAL_SEC - (clock() - cycle_start), deadline - clock())
        if delay > 0:
            sleep(delay)


def main():
    client = None
    completed = False
    stop_replied = False
    status = 1
    print(f"=== Test co khi thuan tuy: steer=0, speed={TEST_SPEED}, "
          f"trong {TEST_DURATION_SEC}s ===")
    print("Dam bao: xe tren mat dat that, khong ke, da danh dau diem xuat phat.")
    try:
        input("Nhan Enter de bat dau (Ctrl+C de huy)...")
        sock = socket.create_connection((SOCKET_IP, SOCKET_PORT), timeout=FRAME_TIMEOUT_SEC)
        client = BridgeClient(sock)
        print("[OK] Da ket noi bridge; kiem tra phan hoi bang lenh dung.")
        check_safety(client.exchange(0, 0))
        drive(client, TEST_SPEED, TEST_DURATION_SEC)
        completed = True
        print("[TEST] Da gui du thoi gian, dang yeu cau dung...")
    except KeyboardInterrupt:
        print("\n[HUY] Nguoi dung ngat test - dang dong ket noi.")
        status = 130
    except (OSError, EOFError, FrameProtocolError) as error:
        print("[LOI] %s: %s" % (type(error).__name__, error))
    finally:
        if client is not None:
            try:
                stop_replied = client.stop_and_close()
            except KeyboardInterrupt:
                status = 130
            if stop_replied:
                print("[STOP] Bridge da tra loi lenh dung; chua xac nhan banh da dung.")
            else:
                print("[STOP] Da dong ket noi, khong xac nhan duoc lenh dung. "
                      "Kiem tra xe/nut dung khan cap.")

    if completed and stop_replied and status != 130:
        print("[OK] Hoan tat chuoi lenh; khong tu dong ket luan xe chay thang.")
        print(">>> Khi xe da dung, do do lech (cm); ghi nhan neu xe bi chan/dung giua chung. <<<")
        return 0
    print("[KHONG HOAN TAT] Khong dung lan nay lam phep do du thoi gian.")
    return status


if __name__ == "__main__":
    sys.exit(main())
