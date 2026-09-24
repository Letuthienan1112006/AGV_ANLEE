# Tổng kết sửa AGV — 11/09/2026

## Phạm vi

Sửa trong workspace `/mnt/big/AGV_ANLEE`, dựa trên commit `3399df7c`.
Đã chạy kiểm thử offline và phân tích lại log có sẵn. Chưa SSH/deploy lên
Jetson, chưa mở camera/serial thật, chưa cho bánh xe quay, chưa nạp firmware.
Các thay đổi vẫn nằm trong working tree, chưa commit.

`speedup.sh` đã có thay đổi của người dùng từ trước buổi làm việc; giữ nguyên
toàn bộ. SHA-256 trước/sau đều là
`8c1216f4a0ec5f77281bcf71a38384782a18dabfb418e85a1ab1eb33335802af`.

## Từng file đã sửa/thêm

| File | Thay đổi trong buổi này |
| --- | --- |
| `lidar_safety.py` | Khi ép STOP do lỗi, vô hiệu hóa trạng thái scan và khoảng cách cũ. Snapshot bổ sung tuổi scan, `scan_fresh`, `bypassed`. Làm rõ không có phản xạ bên hông là chưa biết, không phải chắc chắn thông thoáng. |
| `real_car_socket.py` | Hủy né và gửi tốc độ 0 khi LiDAR lỗi/cũ, thiếu điểm phía trước, mất khoảng trống bên đang rẽ hoặc AI yêu cầu dừng. Không bắt đầu né khi AI yêu cầu tốc độ 0. Không xem bên hông không có dữ liệu là khoảng trống vô hạn. Giữ giới hạn tốc độ/góc, bảo vệ AI/UART, chế độ bench và các lệnh STOP lúc kết thúc. |
| `run.sh` | Lưu `run_mode.json` trước khi khởi động phần cứng: chế độ, khai báo kê trên giá, dry-run, bypass LiDAR, Git HEAD và trạng thái dirty. Đây là khai báo, không phải cảm biến xác nhận tư thế. Truyền tùy chọn hiệu chuẩn từ `run snap` xuống `snapshot_check.py`. |
| `camera_calibration.py` — mới | Logic nhẹ không phụ thuộc camera/GPU: tìm LINE theo đúng dải quét/ngưỡng pixel, tính heading, kiểm tra mẫu và lịch sử hiệu chuẩn. Tối thiểu 3 ảnh hợp lệ/lần đặt; từ chối góc thiếu/không hữu hạn/vượt 3 độ và tâm dao động quá 25 px. Cần ít nhất 3 lần đặt lại hợp lệ, cùng phiên/cấu hình mới đề xuất tâm. |
| `snapshot_check.py` | Mặc định chỉ quan sát. Chỉ ghi lịch sử khi có `--session` và `--confirm-placement`. Xác nhận kích thước ảnh; lịch sử JSONL phiên bản mới chứa cấu hình và SHA-256 model. Không trộn CSV cũ hay mẫu khác phiên/cấu hình. Chỉ in tâm ứng viên, không tự sửa cấu hình. Import model/camera trễ để `--help` và unit test không khởi tạo phần cứng. |
| `test_static_camera_calib.py` | Dùng chung luồng snapshot qua bridge, không còn camera mặc định khác độ phân giải, hàng quét 85%, fallback ROAD hay phép đo thiếu heading. Đây là thay đổi cách dùng: cần bridge đang chạy; cách ưu tiên là `run snap`. |
| `wheel_check.py` | Đọc điều kiện test khai báo từ metadata hoặc banner log cũ; không suy kê trên giá từ yaw ít đổi/thiếu. Phân biệt lệnh chạy và encoder hoạt động; giữ các mẫu có lệnh nhưng bánh đứng. Không bỏ qua sai số tốc độ khi không tải. Chênh lệch AI/firmware cùng dòng chỉ là quan sát chưa đồng bộ, không phải kết luận mất lệnh. Sửa diễn giải dấu hệ số lái và giảm kết luận quá chắc từ tương quan/độ trễ. |
| `verdict.py` | Ưu tiên `wheel_telem.csv` thay vì chỉ log bị giới hạn tần suất. Phân biệt lệnh, encoder, điều kiện bench/dry và bằng chứng chuyển động thân xe. Không kết luận xe đã chạy/bám tâm chỉ vì có lệnh; không khẳng định vật cản tồn tại suốt lần chạy từ vài dòng STOP; không quy lỗi cơ khí từ các thống kê chưa đủ chứng cứ. |
| `tests/test_bridge_safety.py` — mới | 15 bài kiểm tra an toàn với driver, UART và đồng hồ giả; tái hiện lỗi trước sửa, kiểm tra dừng khi cảm biến lỗi/AI dừng, khoảng trống bên hông, chế độ bench và STOP cuối. |
| `tests/test_run_metadata.py` — mới | 2 bài kiểm tra metadata cho đủ 6 chế độ và việc chuyển tiếp tùy chọn snapshot. Chỉ chạy đoạn ghi metadata trong thư mục tạm, không chạy `run.sh`. |
| `tests/test_camera_calibration.py` — mới | 27 bài kiểm tra mask giả, heading, ngưỡng mẫu, phiên/lịch sử, xác nhận tư thế và hai entrypoint dùng cùng phép đo. |
| `tests/test_straight_protocol.py` — mới | 23 bài kiểm tra socket/clock giả: JSON phân mảnh, escape, timeout, giới hạn frame, ngắt kết nối, Ctrl+C, STOP và kết quả hoàn tất/chưa hoàn tất. |
| `tests/test_wheel_analysis.py` — mới | 20 bài kiểm tra dữ liệu tổng hợp: metadata, yaw thiếu/đứng số, lệnh nhưng encoder đứng, sai số tốc độ trên giá, dấu encoder và cách diễn giải báo cáo. |
| `test_straight_mechanical.py` | Mỗi lệnh nhận hết đúng một JSON trước khi gửi tiếp. Giới hạn frame 2 MB và thời hạn tổng gửi–nhận; dùng monotonic. Kiểm tra bằng STOP trước khi thử; hủy khi bridge báo blocked/lifted. Khi mất đồng bộ thì đóng kết nối, không nối thêm lệnh vào luồng lỗi. Chỉ báo hoàn thành chuỗi lệnh khi đủ thời gian và nhận phản hồi STOP; không coi đó là xác nhận bánh đã dừng hoặc xe chạy thẳng. |
| `reports/AGV_FIX_SUMMARY_20260911.md` — mới | Bản tổng kết này. |

## Kiểm thử đã chạy

```text
python3 -B -m unittest discover -s tests -v
87 tests — OK

bash -n run.sh speedup.sh
PASS — chỉ kiểm tra cú pháp, không chạy script

git diff --check
PASS
```

Các file Python mới/sửa được kiểm tra cú pháp tương thích grammar Python 3.6;
đây không phải xác nhận runtime trên Jetson. Hai lệnh `--help` của snapshot và
static calibration hoạt động offline, không import model/driver.

Phân tích lại log bằng công cụ đã sửa:

- `20260911_053434_sweep`: 172 mẫu firmware có lệnh chạy, 171 mẫu encoder hoạt
  động; điều kiện khai báo là sweep/bench. Sai số encoder–target trung vị vẫn
  +6/+7 tick; không che sai số này với lý do không tải.
- `20260911_050117_road`: 398 khung, nhịp trung vị 7,41 FPS; không có lệnh chạy
  hoặc encoder hoạt động trong mẫu ghi được. Không dùng lần này để kết luận
  khả năng bám tâm trên đường.
- `20260911_073551_bench`: 459 mẫu, 34 mẫu có lệnh chạy và 30 mẫu encoder hoạt
  động; nhận đúng khai báo bench. Không thay đổi log gốc.

## Cách dùng hiệu chuẩn đã thay đổi

Chưa thực hiện các lệnh có camera dưới đây trong buổi sửa này:

```text
run snap
run snap --session mount-a --confirm-placement
```

Lệnh đầu chỉ quan sát. Lệnh sau chỉ dùng sau khi đã đặt lại **tâm xe** lên vạch
thẳng bằng đo đạc thực tế, thân xe song song với vạch, camera cố định. Mỗi lần
ghi là một lần đặt lại, không phải chạy lệnh nhiều lần khi giữ nguyên xe. Sau
khi thay gá/vị trí camera phải dùng tên phiên mới. Lịch sử mới sẽ ghi vào
`center_calib_v2.jsonl`; `center_calib.csv` cũ không bị xóa/sửa. Không có thao
tác tự động đổi `STEER_CENTER_X`.

## Những gì chưa thay đổi/chưa kiểm chứng

- Không sửa `Confg.py`, tâm 344, PID, trim, firmware, `camera.py`, `daydu.py`
  hoặc `platformio.ini`. Không can thiệp swap/clocks/hệ điều hành.
- Sai số bám tốc độ thực tế vẫn cần đo và đánh giá dưới tải; chưa tự chỉnh PI
  hay feed-forward từ dữ liệu trên giá.
- Chưa kiểm tra camera/GPU và vận hành thật trên Jetson; bản sửa chưa được
  triển khai. Các helper mới phải đi cùng source khi triển khai, không chỉ
  chép riêng một file gọi chúng.
- Guard mới bảo vệ nhánh né; chưa thay chính sách cũ khi đang chạy bình thường
  với scan toàn cục hợp lệ nhưng không có phản xạ trong cone phía trước.
- Chế độ bench/sweep là người vận hành khai báo, không tự phát hiện xe đã kê.
  Bypass LiDAR chỉ dành cho bánh không chạm đất. Phản hồi socket không thay
  thế xác nhận dừng vật lý hoặc nút dừng khẩn cấp.
- Các kết quả offline không chứng minh xe đã hết quẹo phải hay bám tâm tốt.
