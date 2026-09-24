#!/bin/bash
# run.sh - Chay mot lan test AGV bang MOT lenh duy nhat.
#
#   run          duong that (production: LiDAR 1.20/1.60, ne vat BAT)
#   run control  lai bang tay WASD tren terminal; AI + LiDAR van chay
#                (khong co lui - firmware chi nhan speed >= 0)
#   run lane     thu bam lane tren doan duong trong, LiDAR TAM BO QUA.
#                Xe chay tren mat dat: nguoi van hanh phai san sang STOP.
#   run bench    xe ke tren gia (LiDAR 0.20/0.35, ne vat TAT, banh QUAY)
#   run dry      xe ke tren gia, UART khong mo -> motor KHONG nhan gi
#                (dung che do NAY de tai GPU khi do dien - bench cho banh QUAY)
#   run straight chay thang co khi vong ho, steer=0, KHONG bam vach.
#                XE TREN MAT DAT, banh QUAY. Luu wheel_telem.csv.
#   run lidar    kiem tra LiDAR CHI DOC, khong mo UART -> banh khong the quay
#   run snap     chi chup 3 khung + segmentation roi thoat, KHONG lai xe
#                them --session TEN --confirm-placement de ghi hieu chinh
#   run trim      lai HO VONG o vai muc steer co dinh de do STEER_TRIM dung.
#                 CAN DOAN THANG TRONG ~10m. Xe KHONG tu be lai.
#
# `run` va `off` la wrapper trong ~/.local/bin, go duoc tu bat cu thu muc nao.
#
# Dung: Ctrl+C mot lan. Script tu dung ca bridge, roi gom video + CSV +
# log cua ca hai tien trinh vao mot thu muc rieng cho lan chay do.
#
# Vi sao gom vao thu muc: truoc day video/CSV/log nam lan lon trong repo,
# moi lan test xong phai tu tim file nao thuoc lan nao. Gio moi lan chay
# la mot thu muc tu chua du.

set -u
cd "$(dirname "$0")" || exit 1

MODE="${1:-road}"
BRIDGE_ARGS=""
unset AGV_LANE_TEST_NO_LIDAR
unset AGV_MANUAL_CONTROL

# YOLO26 la pipeline production hien tai. Gan mac dinh ngay tai wrapper
# de ca run_mode.json lan tien trinh con deu ghi/nhan cung mot lua chon.
# Van cho phep AGV_USE_YOLO26_UNIFIED=0 khi can doi chieu co chu y.
export AGV_USE_YOLO26_UNIFIED="${AGV_USE_YOLO26_UNIFIED:-1}"

case "$MODE" in
    road)
        unset AGV_BENCH
        ;;
    control)
        # Nguoi lai bang WASD tren terminal. TOAN BO phan AI van chay:
        # YOLO26, canh bao nguoi/xe, telemetry, video, va LiDAR VAN BAT
        # (khac voi 'lane'). Chi lane PID la khong lai xe nua.
        # Khong co lui: firmware clamp speed >= 0 o ba cho, nen 's' la
        # phanh. Muon lui thi phai sua firmware + nap lai.
        unset AGV_BENCH
        export AGV_MANUAL_CONTROL=1
        ;;
    lane)
        # Thu rieng bo dieu khien bam lane tren doan duong da don trong.
        # Day KHONG phai production: LiDAR va ne vat deu bi bo qua.
        unset AGV_BENCH
        export AGV_LANE_TEST_NO_LIDAR=1
        BRIDGE_ARGS="--no-lidar"
        ;;
    bench)
        export AGV_BENCH=1
        ;;
    dry)
        export AGV_BENCH=1
        BRIDGE_ARGS="--dry-run"
        ;;
    trim)
        # Do STEER_TRIM: lai ho vong, giu steer co dinh, doc yaw IMU.
        # Giu nguyen nguong LiDAR production vi xe chay tren duong that.
        unset AGV_BENCH
        ;;
    sweep)
        # Quet cac muc steer de do dap ung banh xe. XE PHAI KE TREN GIA:
        # banh QUAY THAT, va LiDAR bi BO QUA hoan toan.
        #
        # Bo qua LiDAR vi lab qua chat: do duoc non truoc 237mm, ket giua
        # nguong STOP 200mm va RESUME 350mm cua bench, nen bridge dung va
        # khong bao gio nha - khong test duoc gi du banh dang treo lo lung.
        # AGV_BENCH chi la khai bao cua nguoi van hanh, KHONG phai cam bien
        # xac nhan xe tren gia. PHAI tu kiem tra banh khong cham dat.
        export AGV_BENCH=1
        BRIDGE_ARGS="--no-lidar"
        ;;
    wcal)
        # wheel_calibration.py: do dap ung PWM/DELTA tung banh qua 8 diem
        # (steer=0 o 4 muc speed, roi steer=+-20 o 2 muc speed).
        #
        # Tot hon 'run sweep' o mot diem quan trong: no co SETTLE_SEC=1.5
        # nen BO QUA qua do, chi lay so lieu on dinh. 'sweep' gom ca qua do
        # vao trung vi, lam sai so encoder-target nhin te hon thuc te.
        # Va quet steer=0 qua nhieu muc SPEED nen tach duoc mat can bang
        # TRUYEN DONG khoi chuyen LAI.
        #
        # XE PHAI KE TREN GIA - banh quay that, LiDAR bi bo qua.
        export AGV_BENCH=1
        BRIDGE_ARGS="--no-lidar"
        ;;
    snap)
        # Kiem tra camera truoc khi chay that: chup anh + segmentation,
        # khong bat vong lap lai xe. UART khong mo nen motor khong nhan gi.
        export AGV_BENCH=1
        BRIDGE_ARGS="--dry-run"
        ;;
    straight)
        # Chay thang co khi vong ho: steer=0 co dinh, KHONG bam vach.
        #
        # Ton tai vi truoc day ke hoach bao chay tay hai cua so:
        #     python3 real_car_socket.py
        #     python3 test_straight_mechanical.py
        # Khong co AGV_RUN_DIR thi stm32_reader_fn KHONG mo wheel_telem.csv
        # (real_car_socket.py: `if run_dir:`), nen bai test se khong luu
        # DELTA L/R va PWM L/R - dung nhung con so no ton tai de lay.
        #
        # Nguong LiDAR production vi xe chay tren mat dat that.
        unset AGV_BENCH
        ;;
    lidar)
        # Kiem tra LiDAR CHI DOC. Khong mo bridge, khong mo UART, khong gui
        # lenh nao cho xe - nen banh khong the quay.
        #
        # KHONG chay dong thoi voi bridge: ca hai deu mo /dev/ttyUSB0.
        unset AGV_BENCH
        ;;
    *)
        echo "Che do khong hop le: $MODE"
        echo "Dung: run [road|lane|control|bench|dry|snap|trim|straight|sweep|wcal|lidar]"
        echo "      (mac dinh: road)"
        exit 1
        ;;
esac

STAMP=$(date +%Y%m%d_%H%M%S)

# Ghi vao the nho, KHONG ghi vao /mnt/ssd.
#
# /mnt/ssd khong phai SSD: no la o cung co 2.5" Seagate ST500LT0 gan ngoai
# qua USB, dung chung bus voi camera C270, LiDAR va STM32. Ngay 2026-09-10
# no rung khoi bus giua luc chay, ext4 bao loi va remount read-only, keo
# theo ca file swap 8GB nam tren do. Khong ghi gi vao no trong luc lai.
#
# Nghen fps hoa ra la YOLO tren CPU (850ms), khong phai dia - nen the nho
# hoan toan du, mien la con cho: video ~15MB moi 2 phut.
RUN_BASE="$HOME/agv_runs"

FREE_MB=$(df -m "$HOME" | tail -1 | tr -s ' ' | cut -d' ' -f4)
if [ "${FREE_MB:-0}" -lt 500 ]; then
    echo "[RUN] CANH BAO: the nho chi con ${FREE_MB}MB trong."
    echo "[RUN] Ghi video len the gan day lam cham I/O. Don bot truoc khi test dai."
fi

RUN_DIR="$RUN_BASE/${STAMP}_${MODE}"
mkdir -p "$RUN_DIR" || exit 1

# patrol_robot.py doc bien nay va ghi video + CSV THANG vao day, nen khong
# con phai di chuyen file sau khi chay (lan dau glob patrol_record_*.avi da
# hot ca 68 video lich su vao mot thu muc run).
export AGV_RUN_DIR="$RUN_DIR"

# Ghi dieu kien test DUOC KHAI BAO, khong suy "tren gia" tu yaw dung yen.
# Khong import Confg/bridge: khoi nay chi ghi metadata, chua mo phan cung.
python3 - "$MODE" "$RUN_DIR" <<'PY'
import datetime
import json
import os
import subprocess
import sys

mode, run_dir = sys.argv[1:]
try:
    commit = subprocess.check_output(
        ["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL
    ).decode().strip()
    dirty = subprocess.call(["git", "diff", "--quiet", "HEAD"],
                            stderr=subprocess.DEVNULL) != 0
except (OSError, subprocess.CalledProcessError):
    commit, dirty = None, None
with open(os.path.join(run_dir, "run_mode.json"), "w") as handle:
    json.dump({
        "schema_version": 1, "mode": mode,
        "vision_backend": ("yolo26_unified"
                           if os.environ.get("AGV_USE_YOLO26_UNIFIED", "1")
                           not in ("", "0") else "legacy_two_model"),
        "declared_on_stand": (None if mode in ("dry", "snap") else
                              mode in ("bench", "sweep", "wcal")),
        "dry_run": mode in ("dry", "snap"),
        "lidar_bypassed": mode in ("lane", "sweep", "wcal"),
        "git_head": commit, "git_dirty": dirty,
        "started_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
    }, handle, indent=2)
PY
if [ "$?" -ne 0 ]; then
    echo "[RUN] LOI: khong luu duoc run_mode.json; chua khoi dong phan cung."
    exit 1
fi

echo "=============================================="
echo " AGV run  |  che do: $MODE"
echo " ket qua : $RUN_DIR"
echo " dung lai: Ctrl+C (mot lan)"
echo "=============================================="

# Don sach tien trinh cu, neu khong bridge cu se giu cong 54321 va
# camera, lan chay moi that bai voi loi kho hieu.
pkill -INT -f patrol_robot.py 2>/dev/null
pkill -INT -f real_car_socket.py 2>/dev/null
sleep 2
pkill -KILL -f patrol_robot.py 2>/dev/null
pkill -KILL -f real_car_socket.py 2>/dev/null

# Luu lai chinh ban config da chay. Khong co no thi cham diem lai mot lan
# chay cu se dung config HIEN TAI - vd verdict.py doc tran lai +-8 de cham
# mot lan chay von chay o +-20, ra ket luan sai ma khong he bao loi.
cp Confg.py "$RUN_DIR/Confg_snapshot.py" 2>/dev/null

# Ghi xung nhip + nhiet do trong suot lan chay.
#
# 2026-09-11: cung mot code, cung mot ngay, seg do duoc 67ms khi ke tren gia
# nhung 210ms khi chay duong that 20 phut sau - GPU cham 3 lan. Jetson tu co
# gian xung nhip (do luc ranh: GPU 76.8MHz / dinh 921MHz), va con ha xung khi
# nguon khong cap du dong. Khong ghi lai thi moi lan chay cham la lai ngoi
# doan. Doc sysfs, khong can sudo.
GPU_FREQ=/sys/devices/gpu.0/devfreq/57000000.gpu/cur_freq
CPU_FREQ=/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq
(
    echo "t,gpu_mhz,cpu_mhz,gpu_c,cpu_c"
    T0=$(date +%s.%N)
    while :; do
        G=$(cat $GPU_FREQ 2>/dev/null || echo 0)
        C=$(cat $CPU_FREQ 2>/dev/null || echo 0)
        GT=$(cat /sys/devices/virtual/thermal/thermal_zone2/temp 2>/dev/null || echo 0)
        CT=$(cat /sys/devices/virtual/thermal/thermal_zone1/temp 2>/dev/null || echo 0)
        printf '%.1f,%d,%d,%d,%d\n' \
            "$(echo "$(date +%s.%N) - $T0" | bc)" \
            "$((G/1000000))" "$((C/1000))" "$((GT/1000))" "$((CT/1000))"
        sleep 0.5
    done
) > "$RUN_DIR/sysmon.csv" 2>/dev/null &
SYSMON_PID=$!

BRIDGE_LOG="$RUN_DIR/bridge.log"
PATROL_LOG="$RUN_DIR/patrol.log"


cleanup() {
    echo ""
    echo "[RUN] Dang dung..."
    kill $SYSMON_PID 2>/dev/null
    pkill -INT -f patrol_robot.py 2>/dev/null
    sleep 1
    pkill -INT -f real_car_socket.py 2>/dev/null
    sleep 2
    pkill -KILL -f patrol_robot.py 2>/dev/null
    pkill -KILL -f real_car_socket.py 2>/dev/null

    # Khong can di chuyen gi: patrol_robot.py da ghi thang vao $RUN_DIR
    # qua AGV_RUN_DIR.
    sync

    echo ""
    echo "[RUN] Xong. Ket qua trong: $RUN_DIR"
    ls -lh "$RUN_DIR" 2>/dev/null | tail -n +2
    exit 0
}
# --- Che do lidar khong dung bridge: thoat ngay tai day ---
#
# Dat TRUOC 'trap cleanup' de Ctrl+C di thang vao python, vi lidar_safety.py
# tu bat KeyboardInterrupt va in phan tong ket. Neu trap chay truoc thi
# tong ket bi mat.
if [ "$MODE" = "lidar" ]; then
    if pgrep -f real_car_socket.py > /dev/null; then
        echo "[RUN] LOI: bridge dang chay va dang giu /dev/ttyUSB0."
        echo "[RUN] Dung bridge truoc (./off.sh), roi chay lai."
        exit 1
    fi

    echo "[RUN] Kiem tra LiDAR CHI DOC - khong mo UART, banh KHONG THE quay."
    echo "[RUN] Dat xe dung yen. Ctrl+C de dung va xem tong ket."
    echo ""
    python3 -u lidar_safety.py 2>&1 | tee "$PATROL_LOG"

    echo ""
    echo "[RUN] Quet 360 do theo tung cung 15 do (kiem FRONT_CENTER_DEG)..."
    python3 -u lidar_check.py 5 2>&1 | tee -a "$PATROL_LOG"

    sync
    echo ""
    echo "[RUN] Xong. Ket qua trong: $RUN_DIR"
    ls -lh "$RUN_DIR" 2>/dev/null | tail -n +2
    exit 0
fi

trap cleanup INT TERM

# --- Bridge truoc, cho no san sang moi cho AI vao ---
echo "[RUN] Khoi dong bridge..."
python3 -u real_car_socket.py $BRIDGE_ARGS > "$BRIDGE_LOG" 2>&1 &

for i in $(seq 1 40); do
    if grep -q "Cho patrol_robot.py ket noi" "$BRIDGE_LOG" 2>/dev/null; then
        echo "[RUN] Bridge san sang."
        break
    fi
    if ! pgrep -f real_car_socket.py > /dev/null; then
        echo "[RUN] LOI: bridge chet khi khoi dong."
        tail -20 "$BRIDGE_LOG"
        exit 1
    fi
    sleep 0.5
done

if ! grep -q "Cho patrol_robot.py ket noi" "$BRIDGE_LOG" 2>/dev/null; then
    echo "[RUN] LOI: bridge khong san sang sau 20s."
    tail -20 "$BRIDGE_LOG"
    cleanup
fi

sed -n '1,8p' "$BRIDGE_LOG"

# --- Tien trinh chinh o foreground: Ctrl+C vao thang day, trap lo phan
# con lai ---
if [ "$MODE" = "trim" ]; then
    echo "[RUN] Do STEER_TRIM - xe se chay HO VONG, KHONG tu be lai."
    echo "[RUN] Can doan thang trong. Ctrl+C hoac ./off.sh de dung."
    python3 -u trim_test.py "$BRIDGE_LOG" 2>&1 | tee "$PATROL_LOG"
elif [ "$MODE" = "wcal" ]; then
    echo "[RUN] Hieu chinh banh xe - XE PHAI KE TREN GIA, banh se QUAY."
    python3 -u wheel_calibration.py 2>&1 | tee "$PATROL_LOG"
elif [ "$MODE" = "sweep" ]; then
    echo "[RUN] Quet muc steer - XE PHAI KE TREN GIA, banh se QUAY."
    python3 -u wheel_sweep.py 2>&1 | tee "$PATROL_LOG"
elif [ "$MODE" = "straight" ]; then
    echo "[RUN] Chay thang co khi - XE TREN MAT DAT, banh SE QUAY."
    echo "[RUN] steer=0 co dinh, KHONG bam vach. Danh dau diem xuat phat."
    echo "[RUN] Sau khi chay: do lech ngang bang thuoc, lap lai 3 lan."
    python3 -u test_straight_mechanical.py 2>&1 | tee "$PATROL_LOG"
elif [ "$MODE" = "snap" ]; then
    echo "[RUN] Chup anh kiem tra camera..."
    python3 -u snapshot_check.py 3 "${@:2}" 2>&1 | tee "$PATROL_LOG"
    # snapshot_check.py luu vao ./snapshots, dua sang thu muc lan chay
    for f in snapshots/*_raw.jpg snapshots/*_overlay.jpg; do
        [ -e "$f" ] || continue
        # chi lay anh moi hon luc bat dau lan chay nay
        [ "$f" -nt "$RUN_DIR" ] && cp "$f" "$RUN_DIR/" 2>/dev/null
    done
else
    if [ "$MODE" = "lane" ]; then
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
        echo " CANH BAO: THU BAM LANE, LIDAR DANG BI BO QUA"
        echo " Chi chay tren doan duong trong; san sang Ctrl+C/off."
        echo "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    fi
    echo "[RUN] Khoi dong patrol_robot.py..."
    python3 -u patrol_robot.py 2>&1 | tee "$PATROL_LOG"
fi

cleanup
