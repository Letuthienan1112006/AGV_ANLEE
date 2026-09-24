"""
Confg.py — Cấu hình tập trung cho hệ thống AGV Patrol Robot
============================================================
Tất cả các file Python khác sẽ import từ file này:
    from Confg import *

Khi cần đổi 1 thông số → CHỈ sửa ở đây, không sửa các file khác.

Tổ chức theo 7 nhóm:
  1. NETWORK    — socket xe + MQTT
  2. TELEGRAM   — bot token, chat id
  3. CAMERA     — kích thước, FPS
  4. MODEL      — đường dẫn AI models
  5. VISION     — ngưỡng detection
  6. CONTROL    — PID, tốc độ, steering
  7. NAVIGATION — patrol locations, ArUco, intersection
  8. ALERT      — cooldown, lưu ảnh
============================================================
"""

import os as _os

# ============================================================
#  0. CHẾ ĐỘ BENCH — xe kê trên giá trong phòng lab
# ============================================================
# Bật bằng biến môi trường: AGV_BENCH=1
#
# Vì sao dùng biến môi trường chứ không sửa số trong file: phòng lab chật,
# ngưỡng LiDAR production 1.20 m coi cả bức tường cách 46 cm là vật cản
# nên không test được gì. Trước đây phải sửa tay 2 con số này rồi nhớ
# phục hồi — và đã từng để sai. Với biến môi trường thì mở terminal mới
# là tự về production, không thể để quên.
BENCH_MODE = _os.environ.get("AGV_BENCH", "") not in ("", "0")

# ============================================================
#  1. NETWORK — Socket xe (Jetson ↔ simulator/STM32) + MQTT
# ============================================================

# Socket xe (Python ↔ STM32 hoặc fake_car_socket)
SOCKET_IP   = "127.0.0.1"
SOCKET_PORT = 54321

# MQTT broker (server.py chạy trên PC)
MQTT_BROKER_IP    = "172.20.10.6"   # IP máy chủ chạy Mosquitto (laptop, mang hotspot - co the doi neu doi mang/IP DHCP)
MQTT_PORT         = 1883
MQTT_TOPIC_ALERT  = "patrol/alert"     # khi phát hiện xâm nhập
MQTT_TOPIC_STATUS = "patrol/status"    # heartbeat mỗi 10 giây
MQTT_CLIENT_ID    = "jetson_patrol_01"


# ============================================================
#  2. TELEGRAM — Bot thông báo cảnh báo
# ============================================================

TELEGRAM_BOT_TOKEN = "YOUR_BOT_TOKEN_HERE"   # lấy từ @BotFather
TELEGRAM_CHAT_ID   = "YOUR_CHAT_ID_HERE"     # ID nhóm bảo vệ (số âm)


# ============================================================
#  3. CAMERA
# ============================================================

CAMERA_INDEX  = 0       # 0 = C270 mặc định
CAMERA_WIDTH  = 640
CAMERA_HEIGHT = 360
CAMERA_FPS    = 30


# ============================================================
#  4. MODEL PATHS — đường dẫn các file model AI
# ============================================================

# Nam tren the nho, KHONG tren /mnt/ssd.
#
# 2026-09-10: /mnt/ssd khong phai SSD ma la o cung co gan ngoai qua USB,
# dung chung bus voi camera, LiDAR va STM32. No rung khoi bus giua phien
# lam viec: ext4 remount read-only va model tro thanh khong doc duoc, tuc
# xe khong khoi dong noi. Model chi 20MB nen dat thang len the nho, xe
# khong con phu thuoc o ngoai de chay.
#
# Ban du phong nam cung cho: segmentation_bridged_v2.tar (tot thu 2) va
# segmentation_bridged.tar. Bo goc van o /mnt/ssd/agv_models khi o song.
SEGMENTATION_MODEL_PATH = "/home/agv/agv_models/segmentation_v2data.tar"
# 2026-09-09: model MOI NHAT, train tren data nguoi dung tu tay label lai
# toan bo "line" thanh polygon lien tuc (khong con convex-hull tu dong)
# + anh moi thu thap + trong so "line" x4. So voi segmentation_bridged_v2
# (ban truoc): +83% pixel line, +64% do phu (test tren 30 anh, 27/30 anh
# tot hon). CHUA TEST TREN XE THAT LAN NAO - day la lan dau. Cac ban cu
# van con nguyen trong /mnt/ssd/agv_models/ neu can quay lai:
#   segmentation_bridged_v2.tar (ban truoc, +12% so voi mobilenetv2 goc)
#   segmentation_mobilenetv2.tar (ban goc, dang deploy truoc 2026-09-08)
SIGN_MODEL_PATH         = "/mnt/big/detectsign.pt"
PERSON_MODEL_PATH       = "yolov8n.pt"   # Jetson compatible YOLOv8n
PERSON_INFERENCE_SIZE = 320    # Toi uu YOLO Jetson Nano

# Thiết bị chạy YOLO. 2026-09-10: từng bị ép "cpu" vì torchvision 0.11.1
# trên JetPack 4.6 không có kernel CUDA cho NMS. Đã vá bằng cách cho NMS
# chạy trên CPU còn forward trên GPU (xem _nms_cpu_fallback trong
# patrol_robot.py). Đo được: CPU 518 ms/khung → GPU 51.5 ms/khung, nhanh
# hơn 10 lần, kết quả y hệt, và GPU chỉ chiếm thêm 25 MB.
#
# Dùng "cuda:0", KHÔNG dùng device=0: ultralytics 8.0.20 xử lý dạng số
# bằng cách set CUDA_VISIBLE_DEVICES rồi assert, và nó văng
# "No CUDA GPUs are available" nếu CUDA đã khởi tạo trước đó.
PERSON_DEVICE = "cuda:0" if _os.environ.get("AGV_YOLO_CPU", "") in ("", "0") else "cpu"

# Tham số model
SEG_INPUT_SIZE = (256, 448)    # Toi uu Jetson Nano
SEG_NORMALIZE_MEAN = [0.485, 0.456, 0.406]
SEG_NORMALIZE_STD  = [0.229, 0.224, 0.225]


# ============================================================
#  5. VISION — Ngưỡng detection
# ============================================================

# --- Person detection (YOLO) ---
PERSON_CONF_THRESHOLD = 0.55    # ngưỡng confidence (0-1)
PERSON_MIN_AREA       = 1500    # diện tích bbox tối thiểu (pixel²)
PERSON_CONFIRM_FRAMES = 8       # số frame liên tiếp để xác nhận
PERSON_CONFIRM_RATIO  = 0.75    # tỉ lệ frame phải detect được

# --- Toi uu toc do xu ly (giam do tre vong dieu khien lai) ---
# YOLO dang chay CPU (torchvision NMS CUDA khong san co tren Jetson nay),
# nen cho no chay it khung hon thay vi moi khung, chay song song thread
# rieng voi segmentation (GPU) de khong cong don do tre.
# YOLO giờ chạy xuyên khung, không chặn vòng điều khiển nữa, nhưng nó vẫn
# ăn CPU mà vòng lặp cần cho giải nén JPEG / resize. Một lượt YOLO mất
# ~850ms còn khung giờ ~250ms, nên 1/3 khung nghĩa là YOLO chạy liên tục
# không nghỉ. 1/8 khung cho ra khoảng 2 giây một lượt — quá đủ để phát
# hiện người xâm nhập.
# 20 cho buoi test do lech: uu tien toc do vong dieu khien. Mot luot YOLO
# ~850ms van lam segmentation cham tu 233ms len ~399ms vi giành CPU, nen
# cang thua YOLO thi vong lai cang nhanh. 1/20 khung o ~3fps la khoang 7
# giay mot luot - van bat duoc nguoi di vao khu tuan tra. Ha xuong 8 lai
# khi khong con do toc do vong lap.
# 2026-09-10. YOLO giờ chạy GPU (51.5 ms thay vì 518 ms trên CPU), nên con
# số này không còn bị chi phí CPU ràng buộc — nó thành lựa chọn giữa tốc độ
# vòng điều khiển và độ dày phát hiện người. Đo được cả hai phía:
#     1/5  khung -> vòng lặp 274 ms, phát hiện người mỗi ≈1.4 giây
#     1/20 khung -> vòng lặp ≈260 ms, phát hiện người mỗi ≈5.5 giây
# Chọn 20 theo yêu cầu: ưu tiên tốc độ vòng điều khiển, vì đó là thứ quyết
# định chất lượng bám vạch, còn phát hiện người mỗi 5.5 giây là đủ dùng.
# Muốn dày hơn thì hạ số này — GPU còn thừa sức, không phải nút thắt nữa.
YOLO_EVERY_N_FRAMES = 20

# So khung bo di luc khoi dong, truoc khi bat dau lai. Camera C270 mat vai
# chuc khung moi tu chinh xong phoi sang; nhung khung dau la anh den va
# model doan bua tren do. 30 khung o ~3.5fps la khoang 8 giay.
CAMERA_WARMUP_FRAMES = 30
PERSON_CLASS_ID       = 0       # class ID 0 = person trong COCO
VEHICLE_CLASS_ID      = 2       # class ID 2 = car trong COCO

# --- Segmentation (DeepLabV3) ---
SEG_NUM_CLASSES = 6
SEG_BACKBONE    = "mobilenetv2"
# Thu tu PHAI khop dung luc train (xem train_segmentation.py: CLASS_NAMES)
SEG_CLASS_NAMES = ["background", "road", "line", "car", "motobike", "person"]
CLASS_COLORS    = {
    0: [0, 0, 0],        # background
    1: [128, 0, 128],    # road - tim
    2: [0, 255, 255],    # line - xanh ngoc
    3: [0, 255, 0],      # car - xanh la
    4: [255, 165, 0],    # motobike - cam
    5: [255, 0, 0],      # person - do
}


# ============================================================
#  6. CONTROL — Điều khiển xe
# ============================================================

# --- Tốc độ ---
# QUAN TRỌNG: bridge kẹp tốc độ ở SPEED_MAX = 25 (khớp firmware STM32).
# Mọi giá trị > 25 đều bị kẹp về 25, nên phải giữ CẢ HAI số dưới 25 —
# nếu không thì adaptive_speed_control chạy nhưng vô nghĩa.
#
# 2026-09-10: hai số này từng là 90 và 70 (giá trị thời chạy mô phỏng).
# Đo trên xe thật 230 khung: adaptive_speed_control trả về 77-106 và
# LUÔN bị kẹp về 25, dù lệch 0 hay 280 px. Cơ chế "vào cua thì chậm lại"
# chưa từng hoạt động trên xe thật — cùng loại code chết với lưới an toàn
# lane_lost. Xe vào cua ở tốc độ tối đa trong khi steer đã bão hòa ±20
# suốt 38.7% số khung, khớp đúng chuyện giữ steer max mà độ lệch không giảm.
#
# Thang tốc độ giờ đơn điệu, chậm dần theo mức nghiêm trọng:
#   22 đường thẳng  →  18 khi cua (MIN_SPEED)
#   →  16 khi phục hồi (RECOVERY_SPEED, lệch > 150 px)
#   →  0 khi mất vạch 8 khung (LANE_LOST_SPEED)
# MIN_SPEED phải LỚN HƠN RECOVERY_SPEED, vì phục hồi ghi đè lên adaptive:
# nếu ngược lại thì lúc phục hồi xe sẽ TĂNG tốc, sai hẳn ý định.
# 2026-09-10 (sau khi nạp firmware SPEED_REFERENCE=25): hạ xuống cho lần
# chạy đường đầu tiên. Cùng con số speed nhưng xe nhanh hơn hẳn trước, vì
# bộ PI giờ đẩy PWM lên để đạt mục tiêu mà trước đây nó không với tới —
# đo trên giá: PWM đi thẳng ở speed 21 đã từ 54 lên 72.
#
# CẢNH BÁO khi tune: lực lái TỈ LỆ với tốc độ. Đo bán kính quay khi bẻ hết
# cỡ, tính từ (ngoài+trong)/(ngoài−trong):
#     speed 12 -> 4.89     speed 16 -> 4.16
#     speed 21 -> 3.65     speed 25 -> 3.66 (bão hòa, PWM chạm trần 90)
# Tức chạy CHẬM làm xe quay RỘNG hơn. Giảm tốc khi lệch nhiều là nới bán
# kính đúng lúc cần quay gấp nhất. Nếu xe vẫn không kéo lại kịp thì NÂNG
# BASE_SPEED lên 21 chứ đừng hạ.
BASE_SPEED = 16    # tốc độ mặc định trên thẳng
MIN_SPEED  = 14    # tốc độ tối thiểu khi cua

# --- Steering ---
MAX_STEER  = 20    # góc lái tối đa (độ)
# 2026-09-11: KHONG phai 640/2. Tam hinh hoc cua anh khong trung voi tam
# xe - camera gan lech. Bay lan do lien tiep, khi nguoi dat xe len vach,
# camera thay tam vach o 357, 392, 339, 363, 348, 365 va 344 - KHONG lan
# nao duoi 339. Bo dieu khien lai xe cho tam vach ve dung STEER_CENTER_X,
# nen de 320 la no keo xe sang PHAI khoi vach dung bang khoang lech do -
# chinh la "xe di sat vach chu tam xe chua nam tren line" va "moi vao da
# quet phai".
# 344 la lan do co kiem soat (run snap, 3 khung deu 344, tan mat 0px).
# Neu chay roi van thay xe rieu sang PHAI vach thi tang dan len ~357
# (trung vi gop ca bay lan do).
STEER_CENTER_X = 344
STEER_TRIM = -1   # bu lech co khi trai/phai, cong vao steer truoc khi gui. Am = xe tu lech PHAI -> bu trai. Duong = xe tu lech TRAI -> bu phai. Tune tung buoc +-1/+-2.

# --- Che do phuc hoi khi lech qua nhieu khoi tam lane ---
# Firmware KHONG ho tro lui (moi lenh luon FORWARD, target min=0), nen
# "xoay tai cho" thuc te la xoay quanh 1 banh (banh kia ve 0) o toc do thap,
# khong phai pivot 2 chieu quanh tam xe.
RECOVERY_ERROR_THRESHOLD = 150   # |lane_error| px vuot nguong nay -> vao che do phuc hoi
RECOVERY_SPEED           = 12    # phai NHO HON MIN_SPEED, vi phuc hoi ghi de len adaptive

# 2026-09-04: neu mat dau lane (lane_points=None) qua nhieu khung lien tiep,
# dung lai an toan thay vi cu lai theo last_midpoint cu (da thay that: xe
# di lac vao "Khu B - Bai xe", Lane count:0, camera nhin vao cay/tuong ma
# van cu chay theo gia tri lane cu).
# 2026-09-11: 8 khung nghe it, nhung do la 1.9 GIAY xe chay mu. Do tren lan
# chay 085347: 8 lan mat vach trong 30s, dai nhat 10 khung - chi 1 lan cham
# nguong nay, 7 lan con lai xe cu chay tiep voi lai khoa het co.
# Chon 4 chu khong phai 3: thong ke do dai cac doan mat vach qua 3 lan chay
# cho thay hau het chi 1-2 khung (nhap nhay binh thuong), con cac doan benh
# ly la 6-10 khung. Nguong 3 se bat nham ca nhap nhay 3 khung. 4 khung o
# 7.25 fps = 0.55s, van con xa muc 1.9s cu.
LANE_LOST_MAX_FRAMES = 4    # so khung lien tiep khong thay lane truoc khi dung
LANE_LOST_SPEED      = 0    # toc do khi da mat dau qua lau (0 = dung han)

# --- PID lane following ---
# 2026-09-08: gia tri o day la KET QUA SAU KHI DA TEST TREN XE THAT toi
# hom do (khong phai gia tri mo phong ban dau nua):
# - KP=0.22 (mo phong) test that GAY DAO DONG (18 lan doi dau huong/20s) ->
#   revert ve KP=0.16, xac nhan on dinh lai.
# - INTEGRAL_LIMIT=200 (mo phong) test that GAY WINDUP NANG (ghim steer
#   o +-max, 28 mau lien tuc mot chieu, frame co raw=-1 nhung final=24
#   - loi that gan bang 0 nhung gia tri da loc bi thoi phong do tich luy
#   integral qua lon). Giam xuong 20 -> het dau hieu windup ro ret, nhung
#   CHI moi test 1 lan ~9.5s (ngan) - CHUA xac nhan on dinh tren 1 lan
#   chay dai/sach. KI giu nguyen 0.01 (khong doi trong lan retest nay).
# 2026-09-10, sau khi sua feedForwardPWM: luc lai TANG 4.6 LAN (chenh lech
# banh khi be het co, do tren duong: 2.8 -> 13.0 ticks). Cung mot lenh steer
# gio sinh ra toc do xoay lon hon nhieu, nen he so vong kin cu 0.16 la qua
# nong - xe vot qua vach.
#
# Chon lai theo mot quy tac ro rang: BE HET CO chi nen danh cho luc xe da
# lech RAT xa (vach gan ra khoi khung), tuc |loi| >= 200 px. Voi
# PID_MAX_OUTPUT = 20 thi KP = 20/200 = 0.10.
#     KP=0.16: loi 125 px da bao hoa       (ban kinh quay 1.26 vet banh)
#     KP=0.10: loi 200 px moi bao hoa
PID_KP = 0.05
# 2026-09-10: 0.01 voi PID_INTEGRAL_LIMIT=20 nghia la khau I dong gop
# toi da 20 x 0.01 = 0.2 tren thang steer +-20 — tuc gan nhu KHONG TON TAI.
# Bo dieu khien chay thuan khau P, ma khau P thi KHONG THE khu duoc sai so
# xac lap: no dung lai dung cho KP x loi vua du can bang luc nhieu.
#
# Do duoc tren duong (403 khung, 2 phut): xe on dinh o HAI trang thai lech
# khac nhau tuy huong chay —
#     chay huong A: raw_err dung yen o +55, steer +8   (0.16 x 62 = 9.9 OK)
#     chay huong B: raw_err dung yen o -78, steer -11  (0.16 x 78 = 12.5 OK)
# Ca hai deu on dinh (do lech chuan tut con 10 px) va deu KHAC 0. Anh chup
# xac nhan model bam dung vach son, nen day khong phai loi nhan dien — xe
# that su chay song song canh vach chu khong dam len vach.
#
# Nhieu loan doi dau theo huong chay (do nghieng mat duong + lech co khi),
# nen moi huong xe dung o mot muc lech khac nhau. Khau I la thu duy nhat
# keo sai so xac lap ve 0 bat ke nhieu loan la bao nhieu.
#
# Chon so: can khau I cap duoc ~10 don vi steer de bu luc nhieu do duoc.
# 0.03 x 60px x 4s ~ 7 don vi — hoi tu trong khoang 4-5 giay, du nhanh de
# bam vach ma khong gay dao dong o 3.3 fps.
PID_KI = 0.03
# 2026-09-11: 0.05 duoc chinh khi PID_MAX_OUTPUT=20. Do tren lan chay
# 085347: khau D rieng no da co median 6.2 / p90 16.7 tren thang chi con 8,
# tuc mot minh khau D du suc bao hoa dau ra. Giu dung ty le quyen luc cu:
# 0.05 x (8/20) = 0.02.
PID_KD = 0.02
# 2026-09-10 (lan 2). Ha tu 20 xuong 8.
#
# Sau khi sua feedForwardPWM, be het co cho BAN KINH QUAY 1.26 VET BANH -
# gan nhu quay tai cho. Do la kha nang tot cho ne vat, nhung dung cho bam
# vach thi xe quet qua quet lai het chieu ngang khung hinh: do duoc vach di
# tu mid=12 (mep trai) sang mid=618 (mep phai) va nguoc lai, chu ky 4-5
# giay, steer nam o bao hoa 50% so khung.
#
# Tinh ban kinh quay theo tung muc steer (speed 16, base 25 ticks,
# turnTarget = base x steer x 70/2000):
#     steer 20 -> banh 7/60  -> ban kinh 1.26 vet banh  (quay tai cho)
#     steer 10 -> banh 16/34 -> ban kinh 2.8
#     steer  8 -> banh 18/32 -> ban kinh 3.6            <- chon muc nay
#     steer  5 -> banh 21/29 -> ban kinh 5.7
# Voi KP=0.05: loi 100 px -> steer 5 (ban kinh 5.7, nhe nhang)
#              loi 160 px -> steer 8 (bao hoa, ban kinh 3.6, dut khoat)
#
# Day la tran cua BO DIEU KHIEN BAM VACH. Firmware van nhan toi +-20, nen
# ne vat cua bridge khong bi anh huong.
PID_MAX_OUTPUT  = 8
# Tran dong gop cua khau I = 150 x 0.03 = 4.5 tren thang 20.
# Ha tu 400 (=12 don vi steer): khau I chi can du de khu do lech xac lap
# ~5 don vi, ma luc lai gio manh 4.6 lan nen 12 don vi la qua nhieu.
PID_INTEGRAL_LIMIT = 100   # dong gop khau I = 100 x 0.03 = 3 tren thang 8     # anti-windup (200 gay windup nang, xem ghi chu tren)
# 2026-09-11: 16 duoc chon khi PID_MAX_OUTPUT=20, tuc gioi han moi buoc
# bang 80% toan thang. Khi ha tran xuong 8, bien do thay doi toi da cua dau
# ra chi con 16 (tu -8 den +8), nen nguong 16 KHONG BAO GIO cham duoc - bo
# han che toc do doi lai bi CHET. Giu dung y do cu (80% toan thang) tren
# thang moi: 0.8 x 8 = 6.4 -> 6.
PID_STEP_LIMIT  = 6        # max delta giữa 2 lần compute

# --- Loc mem lane_error (giam rung giat do nhieu segmentation frame-to-frame) ---
# EMA: smoothed = ALPHA*moi + (1-ALPHA)*cu. ALPHA=1.0 -> khong loc (nhu cu).
# ALPHA cang nho -> cang muot nhung cang tre phan ung. Tune tung buoc +-0.1.
LANE_ERROR_SMOOTHING_ALPHA = 0.5

# --- Tat dan lenh lai khi MAT VACH ---
# 2026-09-11: truoc day khi mat vach, loi duoc giu nguyen o (last_midpoint -
# center_x) roi van bom vao EMA moi khung, nen loi DA CU cu hoi tu len gia
# tri cu va steer con TANG DAN trong luc xe khong nhin thay gi (do duoc o
# lan chay 085347, khung 25-34: raw_err dong bang 163 ma steer bo tu 14 len
# 18 suot 10 khung). Ma lenh lai cuoi cung chinh la lenh vua day vach ra
# khoi khung hinh - no la lenh DANG NGO NHAT de giu lai.
# Gio moi khung mat vach nhan loi voi he so nay: 0.6 -> 0.36 -> 0.22...
# Nhap nhay 1-2 khung van giu phan lon lenh lai; mat lau thi lai tu ve 0.
LANE_LOST_STEER_DECAY = 0.6

# --- Adaptive speed ---
ADAPTIVE_ERROR_NORM      = 280     # chuẩn hóa error → factor
ADAPTIVE_POWER           = 1.8     # mũ giảm tốc theo error
ADAPTIVE_HEAVY_THRESHOLD = 140     # error > 140 → giảm tốc thêm 0.8x
ADAPTIVE_SEVERE_THRESHOLD = 180    # error > 180 → giảm tốc thêm 0.7x

# --- Slope detection ---
# ĐANG TẮT (= 1.0). 2026-09-10.
#
# detect_steep_slope() nhân các số này vào kết quả adaptive_speed_control().
# Nhưng nó suy ra "đang lên dốc" từ |lane_error|, mà lệch lớn nghĩa là xe
# lệch tâm, KHÔNG phải đang lên dốc. Hậu quả: nó TĂNG tốc đúng lúc
# adaptive_speed_control muốn GIẢM tốc — hai cơ chế triệt tiêu nhau.
# Đo được: BASE_SPEED=22 mà tốc độ vẫn ra 25 vì 22 × 1.15 = 25.3.
#
# Nhánh HEAVY còn tệ hơn: nó tăng tốc khi vạch nhảy nhiều giữa 2 khung,
# tức tăng tốc đúng lúc nhận diện đang chập chờn nhất.
#
# Bù dốc thật cần góc pitch của IMU. Bridge ĐÃ đọc roll/pitch từ BNO055
# nhưng chỉ chuyển sang AI cờ "lifted", chưa chuyển pitch. Khi nào chuyển
# thì viết lại detect_steep_slope() theo pitch rồi mới bật lại mấy số này.
SLOPE_BOOST_LIGHT      = 1.0
SLOPE_BOOST_MEDIUM     = 1.0
SLOPE_BOOST_HEAVY      = 1.0


# ============================================================
#  7. NAVIGATION — Tuần tra, ArUco, giao lộ
# ============================================================

# --- Vị trí tuần tra ---
PATROL_LOCATIONS = [
    "Cong chinh",
    "Khu A - Hanh lang",
    "Khu B - Bai xe",
    "Khu C - San sau",
]
LOCATION_INTERVAL = 60   # giây — đổi vị trí mỗi 60s

# --- ArUco markers ---
ARUCO_DICT_NAME      = "DICT_4X4_50"     # tên dict trong cv2.aruco
ARUCO_MIN_AREA       = 800                # pixel² — bỏ marker quá nhỏ
ARUCO_CONFIRM_FRAMES = 5                  # số frame liên tiếp để nhận lệnh

# Marker generation (cho generate_markers.py)
ARUCO_MARKER_SIZE  = 600     # pixel — kích thước ảnh marker
ARUCO_BORDER_SIZE  = 80      # pixel — viền trắng xung quanh
ARUCO_OUTPUT_DIR   = "./markers"

# Ánh xạ marker ID → lệnh
ARUCO_COMMANDS = {
    0: "turn_left",    # marker ID 0 → rẽ trái
    1: "turn_right",   # marker ID 1 → rẽ phải
    2: "straight",     # marker ID 2 → đi thẳng
    3: "stop",         # marker ID 3 → dừng lại
}

# Hiển thị nhãn marker khi in ra
ARUCO_MARKER_INFO = {
    0: {"cmd": "RE TRAI",  "color": (220, 100, 50),  "symbol": "<-"},
    1: {"cmd": "RE PHAI",  "color": (50,  100, 220), "symbol": "->"},
    2: {"cmd": "DI THANG", "color": (50,  180, 50),  "symbol": "^"},
    3: {"cmd": "DUNG LAI", "color": (50,  50,  220), "symbol": "[]"},
}

# --- Logic dừng khi gặp marker STOP ---
STOP_DURATION = 3.0   # giây dừng khi gặp marker STOP

# --- Intersection detection ---
INTERSECTION_SCAN_Y           = 270   # đường scan để detect lane center
INTERSECTION_DISPLAY_SCAN_Y   = 360   # đường scan để vẽ display
INTERSECTION_SHIFT_THRESHOLD  = 80    # pixel — ngưỡng dịch chuyển detect rẽ

# --- Line scan band (2026-09-04) ---
# Vach ke duong la net dut: neu chi quet dung 1 hang y, co luc rot vao
# khoang trong giua 2 net dut -> mat tin hieu line, fallback ve road
# (khong nhay theo lech that cua xe) -> troi tich luy roi moi keo gap.
# Quet them +-LINE_SCAN_BAND hang quanh y, uu tien hang GAN y nhat co
# vach, de "nhin" qua khoang trong ma van bam theo vach lien tuc.
LINE_SCAN_BAND = 12    # pixel, +- quanh hang scan chinh
# 2026-09-09: giam 35 -> 12. Do tren 60 anh test voi model moi (line da
# label lien tuc, khong con net dut): 100% anh co vach NGAY TAI hang quet
# y=270, dai quet chua bao gio phai lay hang khac (lech median=0, max=0).
# Tuc la ly do sinh ra tham so nay (nhay qua khoang trong) khong con nua.
# Van giu 1 chut lam luoi du phong cho luc model rot khung, nhung thu hep
# lai vi khi dai quet PHAI kich hoat, no lay toa do x cua hang cach y toi
# 35px roi dung nhu the la toa do tai y - voi vach nghieng 30 do thi sai
# so ~20px; o 12px chi con ~7px.

# --- Line min pixels (2026-09-08, phat hien qua phan tich lai video that) ---
# get_lane_midpoint() truoc gio chi can >=2 pixel class "line" trong 1 hang
# la TIN TUYET DOI (bo qua road, dung lam tam lane ngay) - gan nhu khong co
# loc nhieu. Phan tich lai 1 video test that (giai ma dung lai pred_mask tu
# video) cho thay class "line" trong du lieu that RAT THUA (~2-6% so voi
# class road) va hay xuat hien duoi dang cac dom nho rai rac (nhieu/false-
# positive cua model, vd 1 dom ~6x6px khong trung voi vach son nao thay
# duoc trong anh goc) - khong phai 2 vach lien tuc nhu thuat toan gia dinh.
# Nhung dom nay vAn duoc get_lane_midpoint tin tuong 100% vi thoa >=2px,
# keo tam lane lech xa vi tri that (co truong hop dom nam ngay canh 1 xe
# dau gan do, ~img "XE 65%", keo raw_lane_error len ~38-107px du xe khong
# he lien quan lane that). Them nguong toi thieu de loc bot cac dom qua
# nho truoc khi tin - CHUA test tren xe that, 8px la uoc luong ban dau
# (nho hon FIT_MIN_PIXELS=20 cua duong Stanley vi day la 1 HANG don, con
# FIT_MIN_PIXELS dem ca 1 DAI ROI nhieu hang).
LINE_MIN_PIXELS = 20   # pixel toi thieu trong 1 hang de tin la line that
# 2026-09-09: tang 8 -> 20 sau khi do thuc te tren model moi (line lien
# tuc). Vach THAT rong: tai hang gan y=270 min=58 / median=100 px; tai
# hang xa y=180 min=30 / median=60 px. Nguong 8 cu qua de dai so voi tin
# hieu that (thap hon ca muc thap nhat 30 tai hang xa), nghia la dom nhieu
# 8-30px van lot qua va duoc tin nhu vach. Lay 20: chat hon 2.5 lan nhung
# van duoi muc thap nhat do duoc (30) mot khoang an toan ~33%, du cho luc
# troi toi/anh xau lam vach mong di. Neu thay nhieu khung bao HOLD tren
# HUD luc chay that (nhat la luc chieu toi) thi ha lai ve 12-15.

# --- Heading error (2026-09-04) ---
# Quet them 1 hang o xa hon (Y nho hon = xa hon trong anh), noi voi diem
# gan (INTERSECTION_SCAN_Y) thanh 1 duong thang, lay GOC lech cua duong
# do lam tin hieu bo sung cho lane_error (chi lech ngang tai 1 hang).
# Cong THEM vao lane_error hien co (khong thay the) - neu 1 trong 2 diem
# khong thay thi bo qua, dung nguyen lane_error cu nhu truoc gio.
HEADING_SCAN_Y_FAR    = 180   # hang quet "xa" (Y nho hon INTERSECTION_SCAN_Y)
HEADING_ERROR_WEIGHT  = 2.0   # so pixel-tuong-duong cho moi 1 do lech goc

# --- Stanley Controller (2026-09-05/08) ---
# Duong an toan MOI, TAT MAC DINH - chua duoc test tren xe that lan nao.
# Bat len de A/B so voi PID cu (khong xoa PID cu, chi chuyen doi qua lai).
# Xem algorithm_v2_proposal/lane_algorithm_v2.py de biet ly do/test offline.
USE_STANLEY_CONTROLLER = False

FIT_ROI_MARGIN  = 30   # px, +- quanh near_y/far_y de fit duong qua vach
FIT_MIN_PIXELS  = 1500 # so pixel line toi thieu trong ROI de tin fit
# 2026-09-09: tang 20 -> 1500. Do that tren 60 anh test voi model moi: dai
# ROI nay (y=150..300, ~150 hang) chua 4208-12638 pixel vach that (median
# 7838). Nguong 20 cu chi bang 0.3% tin hieu that -> gan nhu MOI dom nhieu
# deu lot qua roi duoc np.polyfit ve thanh "duong line", cho ra ca do lech
# lan GOC sai bay (nguy hiem hon duong PID vi Stanley dung ca goc de lai).
# 1500 = chat hon 75 lan, van thap hon muc thap nhat do duoc (4208) gan 3
# lan de chiu duoc anh xau/troi toi.

# CHUA CO CO SO DO DAC THUC TE (khong co camera calibration/homography).
# k=0.8, gain=0.3 la diem khoi dau tu mo phong offline - CAN TUNE THUC TE.
STANLEY_K           = 0.8
STANLEY_MIN_SPEED   = 30.0
STANLEY_STEER_GAIN  = 0.3

INTERSECTION_STABLE_FRAMES    = 3     # số frame ổn định để kích hoạt
INTERSECTION_MAX_FRAMES       = 25    # frame tối đa ở chế độ intersection
INTERSECTION_TIMEOUT_FRAMES   = 40    # frame timeout reset sign

# Khoảng cách an toàn sau khi đi qua marker
SAFE_DIST_FRAMES_AFTER_SIGN = 5

# Lane post-intersection stability
LANE_STABLE_OFFSET = 40   # |midpoint - center_x| < 40 → coi như đã thẳng
LANE_STABLE_WIDTH  = 70   # right - left > 70 → lane đủ rộng

# Stability filter (chống nhiễu giao lộ)
STABILITY_REQUIRED_FRAMES    = 3
STABILITY_CONFIDENCE_THR     = 0.6
STABILITY_AGREEMENT_RATIO    = 0.7


# ============================================================
#  8. ALERT & LOGGING
# ============================================================

ALERT_COOLDOWN_SEC = 30                     # giây chờ giữa 2 lần alert
SAVE_ALERT_IMAGE   = True
ALERT_IMAGE_DIR    = "./alert_images"
STATUS_INTERVAL_SEC = 10                    # gửi heartbeat mỗi 10s

# --- Server side (Sever.py) ---
ALERT_SERVER_SAVE_DIR = "./server_alerts"
PATROL_DB_PATH        = "./patrol_log.db"


# ============================================================
#  9. DEBUG
# ============================================================

SHOW_DEBUG_WINDOW = False    # Headless test qua SSH
DEBUG_HUD_FONT_SCALE = 0.52
DEBUG_HUD_LINE_HEIGHT = 21


# ============================================================
#  10. LIDAR (chưa dùng — để dành cho v2)
# ============================================================

# Đơn vị: mét
if BENCH_MODE:
    # Chỉ dùng khi xe kê trên giá trong phòng chật. KHÔNG dùng trên đường.
    LIDAR_STOP_DIST_M       = 0.20
    LIDAR_RESUME_DIST_M     = 0.35
else:
    LIDAR_STOP_DIST_M       = 1.20
    LIDAR_RESUME_DIST_M     = 1.60
LIDAR_MAX_VALID_DIST_M      = 12.0
LIDAR_STOP_CONFIRM_FRAMES   = 3
LIDAR_CLEAR_CONFIRM_FRAMES  = 8


# ============================================================
#  HELPER — Tạo các giá trị dẫn xuất (không sửa)
# ============================================================
import os
import cv2

# Lấy ArUco dict object từ tên string
def _get_aruco_dict():
    return cv2.aruco.getPredefinedDictionary(
        getattr(cv2.aruco, ARUCO_DICT_NAME)
    )

# Đảm bảo các thư mục tồn tại
def _ensure_dirs():
    os.makedirs(ALERT_IMAGE_DIR, exist_ok=True)
    os.makedirs(ARUCO_OUTPUT_DIR, exist_ok=True)
    os.makedirs(ALERT_SERVER_SAVE_DIR, exist_ok=True)

# Auto-call khi import
_ensure_dirs()
