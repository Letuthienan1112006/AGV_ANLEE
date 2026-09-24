# Known Issues — bài học từ các bug đã fix

Mục đích: mỗi khi Coder/Planner trong pipeline `/ship` sắp đụng vào vùng code liên quan,
đọc file này trước để không lặp lại lỗi đã tốn công tìm ra. Format mỗi mục:

```
## <ngày> — <tên ngắn>
**Vùng ảnh hưởng:** file/module
**Triệu chứng:** quan sát được gì (sai lệch, hành vi lạ)
**Nguyên nhân thật:** root cause sau khi điều tra
**Cách tránh lặp lại:** quy tắc cụ thể cho lần sau
**Tham chiếu:** commit hash / memory file (nếu có)
```

Khi fix xong một bug đáng nhớ (đặc biệt loại "tưởng là A nhưng thật ra là B"),
thêm một mục mới vào CUỐI file này.

---

## 2026-09-16 — Backend selection dùng try/except ImportError sai chỗ
**Vùng ảnh hưởng:** `patrol_robot.py` (chọn backend YOLO26: TensorRT vs ultralytics)
**Triệu chứng:** Trên Jetson, code luôn rơi vào nhánh ultralytics rồi FileNotFoundError vì
`yolo26_unified.pt` chưa từng được deploy lên đó (đúng ra không nên deploy).
**Nguyên nhân thật:** Jetson đã có sẵn `ultralytics==8.0.20` (dùng cho model YOLOv8n người cũ) —
đủ cũ để không đọc được checkpoint YOLO26, nhưng vẫn `import` thành công, nên nhánh
`except ImportError` không bao giờ chạy.
**Cách tránh lặp lại:** Không dùng try/except ImportError để chọn backend khi thư viện có thể
tồn tại nhưng không tương thích phiên bản checkpoint. Chọn theo file THẬT SỰ có trên máy
(`os.path.exists(UNIFIED_ENGINE_PATH)` trước, rồi `UNIFIED_MODEL_PATH`) thay vì suy luận qua
import.
**Tham chiếu:** commit `3c7c2963`

---

## 2026-09-18 — Steering quantization dead zone (zigzag)
**Vùng ảnh hưởng:** `motor_test_bts7960/src/main.cpp` (tính `turnTarget`, ~dòng 417)
**Triệu chứng:** Xe lắc zigzag trái-phải thay vì bám làn mượt; lệnh PID nhỏ (1-3) không tạo
ra chênh lệch tốc độ bánh nào — dead zone chiếm 4-36% số lệnh.
**Nguyên nhân thật:** Ép kiểu số nguyên (integer cast) tại chỗ tính `turnTarget` làm tròn về 0,
nên các lệnh lái nhỏ bị nuốt mất hoàn toàn, không phải do PID gain sai.
**Cách tránh lặp lại:** Khi thêm/sửa logic lượng tử hoá lệnh động cơ trong firmware, luôn kiểm
tra cả hai nhánh dương/âm riêng biệt bằng test hồi quy (xem `tests/test_steer_trim_coupling.py`) —
grep source không đủ để bắt lỗi truncation một phía.
**Tham chiếu:** observation log S49 (2026-09-18)

---

## 2026-09-09 — PWM ceiling không phải PWM_MAX
**Vùng ảnh hưởng:** phân tích log động cơ (`wheel_check.py`, telemetry PWM/encoder)
**Triệu chứng:** Nhìn log tưởng bánh phải bị "bão hoà" (saturated) ở target hiện tại.
**Nguyên nhân thật:** PWM thực sự đạt được là `feedforward + KP·err + 9`, không phải hằng số
`PWM_MAX` (140) — và giá trị PWM_R ghi log là post-trim, không có integral, nên so trực tiếp
với PWM_MAX là kết luận sai.
**Cách tránh lặp lại:** Không kết luận "bão hoà PWM" chỉ từ so sánh với PWM_MAX. Phải tính lại
ceiling thực tế theo công thức feedforward+KP·err+trim tại đúng target đang xét, và đối chiếu
tỉ lệ encoder/PWM để có bằng chứng.
**Tham chiếu:** memory `agv-pwm-ceiling-not-pwm-max`

---

## 2026-09-09 — Heading-error road-fallback che mất lỗi lane thật
**Vùng ảnh hưởng:** logic điều khiển lái khi mất line (`lane_control.py` / fallback point)
**Triệu chứng:** Xe trôi lệch nhưng không bao giờ tự sửa hướng ("drifts but never corrects").
**Nguyên nhân thật:** Khi lane bị mất, code tạo ra một "far point" giả để fallback, nhưng điểm
giả này lại triệt tiêu đúng phần sai số lane thật đang tồn tại, khiến bộ điều khiển nghĩ là
đang đi đúng hướng.
**Cách tránh lặp lại:** Fallback point khi mất lane không được phép ghi đè/triệt tiêu sai số đã
tích luỹ từ trước — chỉ nên giữ nguyên hướng lái cuối cùng đã biết (hold last heading) thay vì
tự bịa điểm mới.
**Tham chiếu:** memory `agv-heading-error-road-fallback-bug`

---

## 2026-09-10 — Dead config family: hằng số simulator vượt giới hạn firmware
**Vùng ảnh hưởng:** các hằng số điều khiển được set ở tầng Python nhưng có trần cứng ở firmware
**Triệu chứng:** Bốn cơ chế điều khiển "chạy" nhưng không có tác dụng gì (hoặc ngược tác dụng).
**Nguyên nhân thật:** Giá trị set trong simulator/Python vượt quá giới hạn cứng đã đặt trong
firmware, nên firmware âm thầm clamp về một giá trị khác — không có lỗi/crash nào báo hiệu.
**Cách tránh lặp lại:** Mỗi khi thêm một hằng số điều khiển mới ở tầng Python, kiểm tra ngay xem
firmware có clamp/limit riêng cho nó không, và log giá trị SAU KHI clamp, không chỉ giá trị set.
**Tham chiếu:** memory `agv-dead-config-family`

---

## 2026-09-22 — Benchmark model đơn lẻ không đại diện cho model chạy trong vòng lặp thật
**Vùng ảnh hưởng:** `benchmark_yolo26.py`, mọi số liệu tốc độ model dùng để quyết định đổi engine
**Triệu chứng:** Một báo cáo trước đó dùng số benchmark model-only (82.0ms) làm căn cứ ước tính
FPS vòng lặp đầy đủ, kết luận "model chiếm ~50% frame period".
**Nguyên nhân thật:** Đối chiếu với log chạy thật trên xe (`runs_pulled/20260916_131525_lane/patrol.log`),
đúng `execute` (suy luận GPU) chậm hơn ~60% khi chạy trong vòng lặp thật (96-108ms) so với benchmark
đơn lẻ (61.4ms) trên CÙNG engine — vì phải tranh CPU/GPU/băng thông bộ nhớ với bridge (camera + JPEG
encode + base64), LiDAR, và ghi video. Phần tiền/hậu xử lý numpy thì gần như không đổi. Model vẫn
chiếm ~73% frame period trên xe, không phải ~50% như benchmark đơn lẻ ngụ ý.
**Cách tránh lặp lại:** Không dùng benchmark model-only làm bằng chứng cho cải thiện frame period.
Tiêu chí quyết định PHẢI là median `interval` từ log `[STAGE]` của vòng lặp thật (`patrol_robot.py`
chạy qua `run.sh dry`/`road`), đo xen kẽ A,B,A,B, không phải benchmark cô lập. Benchmark model-only
chỉ dùng để so sánh TƯƠNG ĐỐI giữa hai engine (cùng phương pháp, cùng ảnh), không dùng để ước tính
FPS tuyệt đối của xe.
**Tham chiếu:** `.bangiao/ke-hoach.md` mục 2.3 (branch `feature/tang-fps`), `.bangiao/ket-qua-do.md`
mục 3 và 5 (đối chiếu số đo thật 2026-09-22: model-only 72.2ms vs `interval` vòng lặp thật 130-135ms
cho cùng engine 640 FP16)

---

## 2026-09-22 — Tên file khi build engine TensorRT phải khớp đúng biến Confg dùng để nạp
**Vùng ảnh hưởng:** lệnh `trtexec --saveEngine=...` khi build engine mới, `Confg.UNIFIED_ENGINE_PATH`
**Triệu chứng:** Một báo cáo trước đó đề xuất lệnh build ghi ra
`/home/agv/agv_models/yolo26_unified.engine` (tên file FP32 640 cũ, không ai đọc), trong khi
`Confg.py:115` (`UNIFIED_ENGINE_PATH`) trỏ tới `yolo26_unified_fp16.engine`. Chạy đúng lệnh đó sẽ
ghi đè một file không ai dùng, tốn ~12 phút build trên Tegra X1, rồi đo lại thấy tốc độ y hệt vì xe
vẫn nạp file FP16 cũ như trước — không có lỗi/crash nào báo hiệu chỗ sai.
**Nguyên nhân thật:** Không có bước nào đối chiếu tên file đích trong lệnh `--saveEngine` với giá trị
THẬT SỰ mà `Confg.py`/`AGV_UNIFIED_ENGINE_PATH` sẽ nạp. Đây là biến thể của "dead config family": thay
đổi có chạy (build thành công) nhưng không có tác dụng.
**Cách tránh lặp lại:** Khi build engine TensorRT mới, luôn (1) đặt TÊN FILE MỚI HOÀN TOÀN, không
trùng bất kỳ file nào code đang đọc, (2) dùng biến môi trường A/B có sẵn
(`AGV_UNIFIED_ENGINE_PATH`) để thử trước khi sửa `Confg.py`, (3) bắt buộc log shape/đường dẫn engine
THỰC TẾ đã nạp lúc khởi tạo (`TensorRTUnifiedYOLO26.__init__` in dòng `[TRT] engine=... input=(H, W)`)
và xác nhận dòng đó trong log trước khi tin bất kỳ số đo nào.
**Tham chiếu:** `.bangiao/ke-hoach.md` mục 2.2 (branch `feature/tang-fps`), commit thêm dòng log
trong `yolo26_tensorrt_runtime.py` (Bước 4.3)
