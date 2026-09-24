# Phân tích và bản sửa bám vạch — 15/09/2026

## Dữ liệu đã kiểm tra

- Lượt mới nhất tìm thấy trên Jetson: `20260915_015830_road`, bắt đầu
  01:58:30 UTC, tức 08:58:30 giờ Việt Nam. Đã tải bản sao nguyên log/video
  về `runs_pulled/20260915_015830_road/`.
- Video: `patrol_record_20260915_015844.avi`, 91 khung. File gắn 10 fps,
  nhưng telemetry ghi 24,418 giây thực; không dùng thời gian phát video để
  tính độ trễ. Đối chiếu theo thứ tự khung với `patrol_telem_*.csv`.
- Bản sao `patrol_robot.py`, `real_car_socket.py`, `Confg.py` trên Jetson
  khớp bản local trước khi sửa; snapshot run khớp cấu hình local trước sửa.
- Run mới đã dùng `PID_MAX_OUTPUT=14`, `PID_INTEGRAL_LIMIT=175`,
  `PID_STEP_LIMIT=11`. Tốc độ road vẫn 12–15 trong CSV; tăng tốc bài
  `straight` và tăng giới hạn lái là hai thay đổi khác nhau.
- Các run local trước đó lúc 07:03/07:26 còn dùng giới hạn ±8. Không dùng
  chúng để kết luận rằng bản ±14 chưa từng kéo được xe trở lại.

## Những gì thấy trong video và log mới

Vạch đi từ giữa sang trái, trở lại giữa rồi sang phải, sau đó quét về trái
và ra khỏi khung. Như vậy giới hạn ±14 đã có khả năng kéo lại, nhưng vòng
điều khiển còn dao động; tăng tiếp giới hạn không phải bản sửa có bằng chứng.

1. Ở khung 35, sai số điều khiển thô đã đổi sang +4,1 nhưng lệnh vẫn −4;
   khung 36, sai số +38,9 nhưng lệnh vẫn −2. Ở chiều ngược lại, khung 51
   sai số −34,6 nhưng lệnh vẫn +2. Sai số thô bao gồm cả vị trí và heading,
   không đồng nghĩa riêng midpoint đã vượt tâm ảnh.
2. PID cũ giữ tích phân từ phía trước đó trong khi EMA còn trễ. Tích phân
   không được ghi trong run cũ: đây là nhận định từ mã và phát lại số đo,
   không phải giá trị integral đo trực tiếp từ xe.
3. Khung 56 mất vạch: steer giảm −14 → −3 nhưng speed vẫn 12. Khung 57
   một mảnh nhận diện ở scanline bị tính là LINE, reset bộ đếm mất vạch.
   Khung 58–60 vẫn HOLD nhưng speed 14–15; khung 61 mới về 0.
4. LiDAR có một đoạn chặn giữa lúc chạy, sau scan 17 điểm không hợp lệ.
   Bản sửa này không thay đổi quy tắc LiDAR. Khi bị chặn, trạng thái PID
   mới được xóa và phải xác nhận lại vạch trước khi xin chạy tiếp.
5. Chênh đáp ứng hai bánh dưới tải vẫn hiện diện: trước lúc bù mạnh,
   t=56,498 s của wheel CSV có target 19/29, encoder 21/19, PWM 46/73.
   Khi bù −14, t=61,606 s target 10/28, encoder 12/17 và yaw giảm.
   Không kết luận driver, nguồn, cơ khí hoặc encoder đã được loại trừ.

## Đã sửa

| File | Nội dung |
|---|---|
| `lane_control.py` (mới) | PID độc lập phần cứng; xóa tích phân cũ khi sai số thô đổi dấu, không tích phân ngược dấu số đo mới trong lúc EMA trễ; không tạo derivative kick ở mẫu đầu; dùng đồng hồ monotonic. Thêm trạng thái xác nhận vạch. |
| `patrol_robot.py` | Dùng bộ điều khiển mới trong hai nhánh PID; chốt lệnh cuối sau adaptive/recovery/navigation; reset PID và EMA khi chưa được chạy; ghi thêm `tracking_state`, `pid_integral` và trạng thái trên video. |
| `Confg.py` | Mất vạch 1 khung → lệnh dừng; khởi động/chạy lại cần 3 khung LINE liên tiếp. Giữ nguyên tốc độ, tâm 344, trim −1, gains và giới hạn lái ±14. |
| `tests/test_lane_control.py` (mới) | 10 kiểm tra regression, gồm đổi dấu trước EMA, giới hạn lệnh, reset, khung LINE lẻ sau mất vạch, bridge chặn và thực thi đoạn chốt lệnh trong mã production. |
| `tools/replay_lane_control.py` (mới) | Phát lại số đo CSV offline để đối chiếu lệnh PID và chốt mất vạch; không mở model/socket/UART. |

## Kiểm chứng

`python3 -B -m unittest discover -s tests -v`: **153 tests passed**.
Mã sửa được kiểm tra cú pháp và `git diff --check`.

Phát lại:

```bash
python3 -B tools/replay_lane_control.py runs_pulled/20260915_015830_road
```

| Khung | raw error | Lệnh PID trong log | PID mới trên cùng số đo |
|---|---:|---:|---:|
| 35 | +4,1 | −4 | 0 |
| 36 | +38,9 | −2 | +2 |
| 51 | −34,6 | +2 | −3 |
| 52 | −93,4 | −1 | −6 |

Thử riêng chốt mất vạch trên chuỗi LINE/HOLD cũ: speed về 0 từ khung 56
(15,280 s), thay vì khung 61 (16,674 s); khung LINE lẻ 57 không cho chạy
lại. Chênh 1,394 s là thời điểm tính lệnh trên dữ liệu cũ, không phải
quãng đường/phanh hoặc thời gian dừng vật lý đã đo.

## Giới hạn và bước tiếp theo

- Replay giữ cố định quan sát cũ. Lệnh mới sẽ tạo ảnh và chuyển động mới;
  chưa thể suy ra quỹ đạo hoặc khẳng định xe thật đã hết dao động.
- Dừng ngay khi mất vạch có thể làm xe dừng thường hơn khi segmentation
  chập chờn, tại vạch đứt đoạn hoặc giao lộ. Ba khung liên tiếp chỉ loại
  nhận diện lẻ, không chứng minh một vùng nhận diện ổn định là đúng vạch.
- Xóa integral ở điểm đổi dấu cần được kiểm chứng dưới tải; không thay
  cho phép kiểm tra cân bánh/nguồn. Không tăng thêm tốc độ để bù lỗi.
- Bản sửa hiện ở workspace laptop, **chưa đồng bộ code sang Jetson**.
  Ba file runtime phải đi cùng nhau: `Confg.py`, `patrol_robot.py`,
  `lane_control.py`. Không cần nạp STM32 cho bản sửa Python này.
- Chưa chạy xe, chưa chạy model hay upload firmware trong lượt làm việc.
  SSH chỉ đọc và tải log/code để đối chiếu.
- Những thay đổi có sẵn của người dùng trong `snapshot_check.py`,
  `test_static_camera_calib.py`, `verdict.py` được giữ nguyên.
