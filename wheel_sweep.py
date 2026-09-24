"""wheel_sweep.py - quet qua cac muc steer de do DAP UNG BANH XE.

    XE PHAI KE TREN GIA. Banh se quay that.

Chay qua bridge (khong qua patrol_robot), nen bridge ghi wheel_telem.csv
va `wheel_check.py` doc duoc.

Vi sao khong dung 'run bench': o do lenh steer den tu bo bam vach dang nhin
san lab, nen no thuong dinh o mot gia tri duy nhat. Muon biet "chenh lech
banh tren moi don vi steer" thi phai co NHIEU muc steer khac nhau - mot
diem khong do duoc do doc cua duong nao ca.

Khac voi trim_test.py: file do do STEER_TRIM tren DUONG THANG THAT va chi
quet [-4..2]. File nay quet ca dai +-8 (tran bo dieu khien) tren GIA, va
khong quan tam yaw - tren gia xe khong quay duoc.

    python3 wheel_sweep.py
"""
import socket
import sys
import time

# Ca hai chieu, va co lap lai de moi muc co du mau. Xen ke 0 de thay banh
# co ve deu nhau khi thoi lai hay khong.
LEVELS = [0, 2, 0, -2, 0, 4, 0, -4, 0, 6, 0, -6, 0, 8, 0, -8, 0]
SPEED = 15
HOLD_SEC = 2.0

HOST, PORT = "127.0.0.1", 54321


def recv_frame(sock):
    data = bytearray()
    while len(data) < 2_000_000:
        chunk = sock.recv(65536)
        if not chunk:
            raise ConnectionError("bridge dong ket noi")
        data.extend(chunk)
        if data.rstrip().endswith(b"}"):
            return


def drive(sock, steer, speed, secs):
    end = time.time() + secs
    while time.time() < end:
        sock.sendall(("%d %d" % (steer, speed)).encode("ascii"))
        recv_frame(sock)


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)
    sock.connect((HOST, PORT))
    try:
        print("[SWEEP] Cho bridge on dinh...")
        drive(sock, 0, 0, 2.0)
        for i, st in enumerate(LEVELS):
            print("[SWEEP] %2d/%d  steer=%+d speed=%d  %.1fs"
                  % (i + 1, len(LEVELS), st, SPEED, HOLD_SEC), flush=True)
            drive(sock, st, SPEED, HOLD_SEC)
        print("[SWEEP] Xong, dang dung banh.")
    finally:
        try:
            drive(sock, 0, 0, 1.5)
        except Exception:
            pass
        sock.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[SWEEP] Nguoi dung dung.")
        sys.exit(0)
