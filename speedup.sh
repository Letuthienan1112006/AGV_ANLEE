#!/bin/bash
# speedup.sh - phai chay bang: sudo bash speedup.sh
#
# Sau moi lan khoi dong lai, Jetson mat hai thu va vong lap tut tu 7.2 fps
# xuong 3.4 fps:
#   1. swapfile 4GB tren /mnt/ssd khong duoc bat lai (khong co trong fstab),
#      chi con zram - loai swap nen trong RAM, ton CPU. Do duoc: trong mot
#      lan chay 45s, MemAvailable tut 3.1GB -> 308MB va 41.505 trang
#      (~162MB) bi day ra zram, CPU ban nen bo nho thay vi nuoi GPU.
#   2. jetson_clocks khong duoc chay lai, nen CPU/GPU/EMC deu tha noi cho
#      bo dieu khien tu co gian.
#
# Chay lai script nay sau MOI lan khoi dong lai Jetson.

if [ "$(id -u)" -ne 0 ]; then
    echo "Phai chay bang: sudo bash $0"
    exit 1
fi

echo "===================== TRUOC ====================="
free -m | sed -n '1p;2p;3p'

# (khong con hard-code duong dan swapfile - dung swapon -a theo fstab)
# 2026-09-11, SUA LOI CUA CHINH SCRIPT NAY: ban dau no bat
# /mnt/ssd/swapfile (4GB, file rac tu 18/6 khong ai dung). File that su
# duoc dung nam trong /etc/fstab:
#     /mnt/ssd/agv_swapfile none swap sw,nofail,pri=-1 0 0
# tuc 8GB. Hau qua do duoc: sau khi chay script cu, swap ra 6077MB
# (4GB + zram) chu khong phai 10173MB (8GB + zram) nhu truoc do - con so
# do da hien ra ma toi khong truy.
#
# Nen dung 'swapon -a': no doc fstab, bat dung file ma he thong dinh dung,
# va tu bo qua cai da bat. Khong hard-code duong dan nao ca.
#
# VI SAO SWAP "MAT" SAU MOT SO LAN BOOT: /mnt/ssd mount theo UUID voi
# 'nofail'. O USB nay lau len, nen co lan swapon luc boot chay TRUOC khi o
# san sang va that bai IM LANG (nofail nghia la khong chan boot, cung khong
# bao loi). Lan khoi dong nao o len kip thi swap co, lan nao khong kip thi
# mat - dung kieu "co luc duoc co luc khong" da thay.
echo "[SWAP] swapon -a (theo /etc/fstab):"
swapon -a 2>&1 | sed "s/^/         /"
if swapon -s | tail -n +2 | grep -q .; then
    swapon -s | tail -n +2 | awk '{printf "         dang bat: %s  %.1f GB\n", $1, $3/1048576}'
else
    echo "         KHONG co swap nao dang bat."
    echo "         Kiem tra /mnt/ssd con mount khong: mount | grep ssd"
    echo "         KHONG tu chay mkswap - no ghi de header cua file."
fi

if [ -x /usr/bin/jetson_clocks ]; then
    /usr/bin/jetson_clocks
    echo "[CLOCK] Da ghim CPU/GPU/EMC o muc toi da."
else
    echo "[CLOCK] Khong thay jetson_clocks."
fi

echo "===================== SAU ======================="
free -m | sed -n '2p;3p'
echo -n "GPU: "; awk '{printf "%d MHz\n", $1/1000000}' \
    /sys/devices/gpu.0/devfreq/57000000.gpu/cur_freq 2>/dev/null
echo -n "CPU: "; awk '{printf "%d MHz\n", $1/1000}' \
    /sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq 2>/dev/null
echo "================================================"
echo
echo "KHONG nen them swapfile nay vao /etc/fstab."
echo "O USB nay tung roi khoi bus giua luc dang chay (2026-09-10). Mat mount"
echo "thi con cuu duoc; mat swap dang song thi treo may. Bat tay moi lan nhu"
echo "script nay it ra con biet luc nao no dang bat."
echo
echo "Luu y: chua chung minh duoc swap anh huong toc do vong lap. Do ngay"
echo "2026-09-11: bat swap + ghim xung nhip xong van 3.4 fps, sau do khong"
echo "sua gi lai len 7.4 fps. Script nay chi dua may ve trang thai da biet,"
echo "khong phai mot ban va toc do."
