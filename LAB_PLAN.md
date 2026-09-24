# Kế hoạch buổi lab — thứ Hai 14/09/2026

Mục tiêu: **một buổi lab, sáu số đo, không đoán thêm.** Thứ tự dưới đây
xếp theo nguyên tắc: đo xong những gì không cần xe chạy, rồi mới cho xe chạy.

Chuẩn bị trước khi đi: **sạc pin đầy** (12.6V), **mang đồng hồ đo**, mang thước dây.

---

## ĐÃ ĐO XONG — 13–14/09/2026 (đọc trước, đừng làm lại)

| Bước | Kết quả | Kết luận |
|---|---|---|
| **0** pin lúc nghỉ | 12.60 V; cell 4.19 / 4.19 / 4.23, lệch **0.04 V** | ✅ pin đầy, cell cân — **loại** "pin cạn" và "cell chai" |
| **1** 5V hạ áp dưới tải GPU | 5.23 V idle → **5.19 V** khi seg chạy (sụt 0.04 V) | ✅ **loại** mạch hạ áp dưới tải GPU. *Tải động cơ chưa thử* |
| **2** LiDAR | 244 điểm trung vị, **cone 12/12**, 0 timeout | ✅ **loại** "LiDAR hỏng" và **loại** "ngưỡng 20 điểm là lỗi" |
| **2b** quay tay cảm nhận | hai bánh đều nhau, không nặng/cạ/kịt | ✅ **loại** cơ khí thô ở tải ~0 |
| **xung/vòng** (encoder_cal.py) | phải **330.5** xung/vòng (lần sạch, 0 đổi chiều); trái 361–368 | ✅ lệch thật **~10%**, không phải 53% → **encoder KHÔNG phải nguyên nhân chính** |

### Vì sao encoder bị loại

Hai lần đo đầu cho bánh phải ra 5614 rồi 3654 xung — lệch 53%, trông như encoder hỏng.
Lần thứ ba **đếm to từng vòng** ra **3305 xung / 10 vòng = 330.5/vòng, sạch hoàn toàn**
(rong = đường đi, 0 lần đổi chiều). Vậy hai lần đầu là **đếm nhầm số vòng tay**, không
phải lỗi encoder. Lệch ~10% so với bánh trái nằm trong sai số quay tay — **không thể
giải thích việc bánh phải chỉ đạt 25% target** (lệch 4 lần, không phải 1.1 lần).

### Còn lại ĐÚNG MỘT nhánh, và nó cần xe chạy có tải

| Nhánh | Cách đo |
|---|---|
| Nội trở pin khi tải động cơ | đồng hồ ở đầu ra pin **lúc bánh đang quay** |
| Mạch hạ áp khi tải động cơ | đồng hồ ở chân nguồn Jetson **lúc bánh đang quay** |
| Điện kênh PHẢI (BTS7960 / dây / đầu nối) | so PWM vs encoder dưới tải; phép thử đổi chỗ 2 động cơ |

**Cả ba đều mở ra bằng MỘT lần `./run.sh straight` ở chỗ trống.**

### Chặn hiện tại: xe bị vây

```
Vật cản cone trước:  237 mm   (14/09, quét 360°)
Ngưỡng STOP:        1200 mm
Ngưỡng RESUME:      1600 mm
```

Trong lab chật xe **không bao giờ** lên GO được. **Việc đầu tiên sáng mai: đưa xe ra
chỗ phía trước thoáng ≥ 1.6 m.** Không phải sửa gì, chỉ là chỗ đứng.

### Firmware: đã build sẵn, chờ nạp

`motor_test_bts7960/.pio/build/nucleo_f411re/firmware.bin` — build OK 14/09, 36596 bytes.
Chứa: 3 trục gyro, `,SEQ,<n>`, `PRINT_INTERVAL_MS` 200 → 100.

Nạp mất ~1 phút, nhưng cần **đổi cáp USB của Nucleo từ Jetson sang laptop** (Jetson không
có `pio`/`st-flash`). **Kê bánh hoặc ngắt nguồn công suất motor trước khi nạp.**

---

## Bước 0a — Nạp firmware mới (5 phút, làm trước tiên)

Firmware đã thêm **tốc độ quay thô từ con quay hồi chuyển** vào dòng telemetry
(3 trục, độ/giây, độ phân giải 0.0625 °/s). Chỉ ghi thêm số liệu, **không thay
đổi điều khiển động cơ**.

> **Cẩn thận: trong repo có hai cây firmware.** Cây đang chạy thật là
> `motor_test_bts7960/` — nhận ra bằng chuỗi `,IMU,NA,NA,NA,NA` khớp đúng log
> run 030448. Cây `AGV_STM32_MOTOR_BNO055_V2/` in định dạng khác
> (`IMU,1,HEAD,...,YAW,...`) mà bridge **không** parse được. Nạp đúng cây
> `motor_test_bts7960/`, nếu nạp cây kia thì toàn bộ telemetry IMU sẽ mất.

```
cd /mnt/big/AGV_ANLEE/motor_test_bts7960 && pio run -t upload
```

Kiểm tra ngay sau khi nạp, trên dòng `[STM32 RX] ENC,...`:

- **7** số sau `IMU,` thay vì 4 (thêm 3 trục gyro)
- có trường **`,SEQ,<n>`** và `n` **tăng đều 1 mỗi dòng**
- dòng ra **nhanh gấp đôi** trước (`PRINT_INTERVAL_MS` 200 → 100)

Số `SEQ` bắt đầu từ 0 mỗi lần MCU khởi động, nên nó là bằng chứng trực tiếp cho
cả hai câu: **mất bản tin** (có lỗ hổng) và **MCU đã reset** (nhảy về gần 0).
Trước đây phía phân tích phải *suy* "đã reset" từ việc bộ đếm encoder giảm — suy
luận sai, vì bộ đếm có dấu (`++`/`--` trong ISR) nên quay bánh ngược chiều làm nó
giảm là bình thường. Nếu vẫn 4 số thì chưa nạp được, và `wheel_telem.csv` sẽ để trống ba
cột `gyro_*` — nó ghi nhận **thiếu số liệu**, không ghi 0.

## Bước 0 — Pin lúc nghỉ (2 phút, xe chưa cấp điện)

Đo ở đầu XT60 và ở giắc balance. Ghi vào bảng:

| Đo | Số | Đạt? |
|---|---|---|
| Tổng lúc nghỉ | ____ V | đầy = 12.60 |
| Cell 1 | ____ V | 4.15–4.20 |
| Cell 2 | ____ V | 4.15–4.20 |
| Cell 3 | ____ V | 4.15–4.20 |
| Lệch cell cao – thấp | ____ V | **< 0.05** |

> Lệch > 0.10 V → một cell đã chai. Đó là câu trả lời, dừng ở đây, không cần
> chạy tiếp: cell yếu sụt trước, kéo cả pack xuống khi có tải. Đúng triệu
> chứng "trên giá đỡ bình thường, xuống đất bánh chỉ đạt 25%".

## Bước 1 — 5V ở Jetson khi GPU tải nặng (3 phút)

```
sudo bash /home/agv/agv_git_clean/speedup.sh     # bắt buộc sau mỗi lần reboot
cd /home/agv/agv_git_clean && ./run.sh dry
```

> **`dry`, không phải `bench`.** `run.sh` dòng 5 ghi rõ `bench` vẫn mở UART và
> **bánh QUAY**, với ngưỡng LiDAR bench 0.20/0.35 rất dễ dãi. Bạn sẽ đang cầm que
> đo vào chân nguồn Jetson — không được để bánh có thể quay. Chế độ `dry` vẫn chạy
> camera + segmentation để tải GPU đúng như thật, nhưng `open_stm32()` trả `None`
> nên **motor không nhận được gì**.
>
> Tương tự ở bước 0a: khi nạp firmware, **kê bánh lên hoặc ngắt nguồn công suất
> motor** — reset MCU có thể sinh ra xung ở đầu ra.

Kẹp đồng hồ vào **chân nguồn vào Jetson**, đọc trong lúc segmentation đang chạy:

| Đo | Số | Đạt? |
|---|---|---|
| 5V lúc idle | ____ V | 5.00–5.10 |
| 5V khi seg đang chạy | ____ V | **> 4.75** |

> Dưới 4.70 V → kết tội mạch hạ áp. Giải thích luôn đèn chớp chớp lúc khởi
> động, LiDAR chết, và ổ USB rụng giữa run.

## Bước 2 — LiDAR: khỏe, bệnh, hay đường trống (2 phút, xe đứng yên)

**Chạy trước tiên, trước cả khi cho bánh quay.** Chế độ này không mở bridge,
không mở UART — bánh không thể quay.

```
./run.sh lidar        # Ctrl+C khi đã xem đủ, nó in tổng kết
```

Nó in mỗi scan một dòng có **`reason`** — trước đây thiếu đúng chỗ này — kèm
số điểm và độ phủ cone trước, rồi tổng kết trả lời ba câu:

| Cột | Nghĩa |
|---|---|
| `points` | số điểm hợp lệ trên **cả 360°** |
| `cone` | số ô 5° trong cone trước **có điểm**, tối đa 12 |
| `reason` | vì sao chưa GO |

| `points` | `cone` | Kết luận |
|---|---|---|
| vài trăm | 11–12 | **Scan khỏe.** Ngưỡng 20 không hề bị chạm tới, nên lần chạy này **không nói gì** về nó — cứ để nguyên |
| 10–19 | 0–2 | **Scan yếu** — đừng nới ngưỡng, kiểm cáp/nguồn USB, xem LiDAR có quay đều |
| vài trăm | **0** | Cone trước rỗng. **Chưa kết luận được** — đọc ba cột lý do bị loại ở dưới |

Ngưỡng `MIN_VALID_SCAN_POINTS = 20` chỉ được bàn tới khi số điểm **thực sự** xuống
gần nó. Một scan khỏe chứng minh nó chưa bao giờ là vấn đề trong lần chạy đó, chứ
không chứng minh nó sai.

### Lỗi mới tìm được — quan trọng

Trong `update()` của `lidar_safety.py`:

```python
path_clear = distance is not None and distance >= RESUME_DIST_MM
```

Khi scan **hợp lệ nhưng cone trước không có điểm phản xạ nào**, `analyze_scan`
trả `distance = None` (chính docstring của nó nói rõ trường hợp này). Lúc đó cả
`obstacle_close` và `path_clear` đều False → rơi vào nhánh else → reset
`clear_confirm` → **máy trạng thái không bao giờ ra khỏi STOP**, và log báo
`vung hysteresis` tức "có vật ở giữa 1.20–1.60 m", trong khi thực ra
**không có điểm nào trong cone** — hai chuyện khác nhau.

### `distance = None` nghĩa là gì — và không nghĩa là gì

Nó **không** nghĩa là "không có gì trong 12 m phía trước". Nó chỉ nghĩa là
**không điểm nào trong cone đạt đồng thời** `quality >= 5` và
`100 mm <= d <= 12000 mm`. Bốn nguyên nhân khác nhau:

1. Thật sự không có gì ở đó.
2. **Có vật nhưng phản hồi bị loại vì `quality < 5`** — vật tối màu, bề mặt chéo.
3. Có vật **gần hơn 100 mm**.
4. Chỉ có vật **xa hơn 12 m**.

Trường hợp 2 và 3 **là vật cản**. Một cái chân người mặc quần tối ở 0.5 m có thể
trả về quality thấp và bị loại, làm cone "rỗng". Nên **giữ STOP ở đây có thể là
đúng**, và "sửa" bằng cách coi cone rỗng là thoáng có thể lái xe vào nó.

Vì vậy `describe_scan` giờ đếm luôn số điểm trong cone **bị loại và vì sao**
(`front_raw`, `front_low_quality`, `front_too_near`, `front_too_far`), và
`run lidar` in ba cột đó. Đó mới là số phân biệt được bốn nguyên nhân trên.

### Còn số lần hysteresis trong log cũ thì sao

4 / 16 / 11 ở ba run 11/09 — **không kết luận được gì từ chúng.** Trước khi đổi
nhãn, cả "có vật ở 1.2–1.6 m" và "cone rỗng" đều ghi cùng một chuỗi. Tôi từng đọc
nó theo cả hai hướng; cả hai đều không có trong dữ liệu.

**Tôi chỉ sửa cái nhãn**, thành `cone truoc khong co diem phan xa`. Máy trạng
thái không đổi một dòng. Cổng đúng phải là "cone trước có được lấy mẫu hay
không", tức chính cột `cone` mà bước này đi đo. Nới một điều kiện STOP dựa trên
test tổng hợp là làm ngược thứ tự.

Nên: **đọc cột `cone` ở bước này, rồi mới quyết định sửa máy trạng thái.**

Sau đó nó tự chạy `lidar_check.py` quét 360° theo cung 15° để kiểm
`FRONT_CENTER_DEG = 130°` có đúng là hướng đi tới. File đó vừa được sửa: trước
đây nó vẽ nón **rộng 30°** trong khi lớp an toàn dùng **60°**, nên một vật ở
105° sẽ chặn xe mà không hiện trong "nón trước".

## Bước 2b — Quay bánh bằng tay, TẮT NGUỒN (2 phút)

Phép phân biệt **rẻ nhất và sạch nhất**, và nó đứng trước mọi thứ có động cơ.

**Cách cấp nguồn cho bài này: TẮT HẲN.** Không cần đọc số gì, chỉ cần cảm nhận
bằng tay, nên tắt hết là an toàn nhất.

Quay từng bánh bằng tay, so hai bên:

| Thấy gì | Nghĩa |
|---|---|
| Hai bánh quay nhẹ như nhau, trôi tự do đều | Truyền động sạch → nhìn sang điện: đầu nối, dây, kênh BTS7960 phải |
| Bánh phải nặng hơn, ráp, hoặc cạ | **Đã tìm ra.** Cơ khí, không cần đo điện gì thêm |
| Bánh phải kẹt từng đoạn | Ổ trục hoặc hộp giảm tốc |

Sờ cả hai lốp: khác đường kính hay khác độ căng cũng làm xe cong dù mọi ốc đều chặt,
vì PI của STM32 cân **số vòng**, không cân **quãng đường**.

> Vì sao bước này quan trọng: `wheel_check.py [2b]` cho biết **trong cùng lần
> chạy, bánh phải được yêu cầu PWM cao hơn nhưng encoder trả về ít hơn đáng kể**
> (tỷ số đáp ứng encoder/PWM 0.126 so với 0.048), trong khi trên giá đỡ hai bánh
> cân trong 4%.
>
> Đây **không** phải hiệu suất vật lý: vòng kín tự tăng PWM *vì* encoder chậm, nên
> hai đại lượng có quan hệ phản hồi, và hai bánh còn chịu target/tải khác nhau khi
> cua. Kết luận chắc nhất là **lỗi nằm sau tầng tạo lệnh** — cơ khí, dây/đầu nối,
> BTS7960, nguồn tại kênh phải, hoặc encoder khi có tải.
>
> Bàn tay bạn phân biệt "nặng" với "thiếu điện" trong 30 giây, còn đồng hồ thì không.

## Bước 3 — Chốt tâm camera (5 phút, xe không chạy)

Cần **3 lần đặt xe khác nhau**, mỗi lần đo thước cho thân xe song song vạch.
Không phải 3 ảnh cùng một chỗ.

```
./run.sh snap        # lần 1, đặt lại xe
./run.sh snap        # lần 2, đặt lại xe
./run.sh snap        # lần 3
```

Kết quả cộng dồn vào `center_calib.csv`. Nó tự từ chối kết luận nếu 3 lần
lệch nhau > 25 px. Hiện `STEER_CENTER_X = 344` chỉ dựa trên **một** lần đặt.

## Bước 4 — Chạy thẳng cơ khí, KHÔNG bám vạch (5 phút)

**Không** `run road` trước. Lần đặt xuống đất đầu tiên phải là chạy thẳng
vòng hở, để tách cơ khí/firmware khỏi nhận diện:

```
./run.sh straight
```

> Dùng `run straight`, **không** chạy tay hai cửa sổ. Chạy tay thì không có
> `AGV_RUN_DIR`, và `stm32_reader_fn` chỉ mở `wheel_telem.csv` khi biến đó được
> đặt (`real_car_socket.py:803`, `if run_dir:`) — nên bài test sẽ **không lưu
> DELTA L/R và PWM L/R**, đúng những con số nó tồn tại để lấy. Chế độ mới lo
> phần đó, và vẫn giữ ngưỡng LiDAR production vì xe chạy trên mặt đất thật.

`steer = 0` cố định, `speed = 8`, **2.5 giây** (đã rút từ 5.0 — xe chưa bao giờ
được thử hạ bánh sau khi bánh phải chỉ đạt 25%). Vẫn đi qua bridge nên LiDAR
safety và watchdog STM32 vẫn hoạt động.

**Lặp 3 lần**, mỗi lần ghi: quãng đường, lệch ngang (thước), góc quay,
DELTA L/R, PWM L/R. Đừng kết luận từ một lần.

### Kèm luôn phép kiểm encoder trực tiếp

Dán một mảnh băng dính lên mỗi bánh. **Quay phim** lúc chạy. Rồi đếm **số vòng
thực tế** từng bánh trong đoạn video và so với **tổng xung encoder** trong cùng
khoảng thời gian.

> **Lấy tổng xung từ `count_l`/`count_r`, đừng cộng dồn `enc_l`/`enc_r`.**
> `enc_l`/`enc_r` là **DELTA** từng chu kỳ; hai bộ đếm **tích luỹ** nằm ngay sau
> `ENC,` thì trước 12/09 bị bridge bỏ qua hoàn toàn. Và cộng dồn DELTA không cho
> tổng xung: firmware tính delta mỗi `CONTROL_INTERVAL_MS = 100 ms` nhưng trước
> 12/09 chỉ in mỗi 200 ms, nên **một nửa các khoảng đếm chưa bao giờ được xuất
> ra**. Một dòng telemetry bị mất cũng mất hẳn một khoảng.
>
> Hai cột `count_l`/`count_r` giờ đã được lưu. Lấy **`count_cuối − count_đầu`**
> trong đúng khoảng video — phép trừ đó miễn nhiễm với cả hai vấn đề. Và
> **xác nhận MCU không reset giữa khoảng**: bộ đếm phải tăng đơn điệu, nếu nó tụt
> về 0 thì phép trừ ra số vô nghĩa. `wheel_check.py` mục `[2c]` kiểm cả hai việc
> này và in độ lệch giữa hai cách tính. **Lệch một mình không chứng minh mất bản
> tin** — sai lệch ở hai đầu khoảng, khác nhịp lấy mẫu và quay đổi chiều cũng gây
> chênh. Chỉ cột `seq` khẳng định được.

| Thấy gì | Nghĩa |
|---|---|
| Bánh phải quay **ít vòng** hơn, encoder cũng ít xung tương ứng | Bánh thật sự chậm → cơ khí / điện / BTS7960 phải |
| Bánh phải quay **đủ vòng** nhưng encoder ít xung | **Encoder đo sai** — và PI đang bị lừa |

> **Đừng giả định hai bên cùng số xung mỗi vòng.** Điều đó chỉ đúng nếu encoder,
> tỉ số truyền và cách đếm (x1/x2/x4) giống nhau ở hai bên. **Xác định chuẩn
> xung/vòng của TỪNG bên trước**, rồi mới so mỗi bên với chuẩn của chính nó.

### Bài đo xung/vòng — cách cấp nguồn KHÁC bước 2b

Bước 2b tắt nguồn hẳn vì chỉ cần cảm nhận bằng tay. Bài này **phải đọc được
encoder**, nên:

| | |
|---|---|
| MCU + encoder | **phải có điện** |
| Nguồn công suất motor | **ngắt riêng** |

Ngắt thế nào thì tuỳ cách đấu thực tế của xe — thường là rút nguồn vào hai con
BTS7960 mà vẫn giữ 5V cho Nucleo. **Nếu hai đường không tách được** thì đừng làm
bài này theo cách quay tay: bánh có thể bị driver kéo bất ngờ.

Cách làm:

1. Quay **một chiều đã quy ước** (ví dụ chiều xe đi tới), đủ **10 vòng** đếm bằng
   dấu băng dính. Đừng quay qua lại — bộ đếm có dấu nên qua lại sẽ triệt tiêu.
2. Đọc `count_l`/`count_r` **trước và sau**, lấy hiệu, chia 10.
3. Làm riêng từng bánh, ghi hai con số riêng.

Khi đối chiếu với video ở trên, dùng **đúng đại lượng**: nếu đoạn video có đổi
chiều thì so với *đường đi* (tổng trị tuyệt đối từng bước), không so với *thay
đổi ròng*. Mục `[2c]` in cả hai và đếm số bước đổi chiều.

> Nếu độ lệch nhỏ đến mức không đọc chắc được bằng thước, nâng
> `TEST_DURATION_SEC` lên 5.0 cho lần 2–3 — lúc đó đã biết xe không bỏ chạy.
> **Đừng** kết luận "xe đi thẳng" từ một độ lệch nhỏ hơn độ chính xác của thước.

Chỉ khi hai bánh cân dưới tải mới sang bám vạch:

```
./run.sh road
```

Giữ đồng hồ ở đầu ra pin, đọc **trong lúc bánh đang quay có tải**:

| Đo | Số | Kết luận |
|---|---|---|
| Pin lúc tải | ____ V | sụt < 0.3V so với nghỉ = pin tốt |
| | | sụt > 1.0V = pin chai, đó là nguyên nhân |

Sau run, chạy trên laptop:

```
python3 wheel_check.py runs_pulled/<run moi nhat>
python3 verdict.py    runs_pulled/<run moi nhat>
```

Xem **mục `[2b]`** mới trong `wheel_check.py`: nó in dư địa còn lại của vòng PI
từng bánh. Trần thật là `feedforward + KP·err + KI·INTEGRAL_LIMIT`, và
`KI·INTEGRAL_LIMIT` chỉ đáng **9 nấc PWM**.

`PWM_MAX = 140` **không với tới trong dải target của run 080230** (target 14–28 →
trần 40–81). Ở target lớn thì feed-forward một mình vẫn chạm được: `ff(62) = 139.5`,
và `MAX_WHEEL_TARGET_TICKS = 70` cho `ff(70) = 157.5` bị kẹp về 140.

Trên run 080230, **sau khi gỡ `RIGHT_WHEEL_PWM_TRIM = 1.03`** khỏi `PWM_R` (log ghi
giá trị *sau* trim): bánh trái còn **+4.8** nấc (~46% tích phân đã dùng), bánh phải
còn **+2.5** nấc (~72%). **Chưa bão hoà** — và integral không bao giờ được ghi vào
telemetry, nên hai con số đó là **suy ra**, không phải đo được.

**Mốc so sánh** (`đạt %` = encoder thực / target của firmware):

| Run | đạt % |
|---|---|
| 20260910_030448_road (pin tốt) | **126%** |
| 20260911_080230_road (pin yếu?) | **25%** |

Nếu lần này về lại ~100% → pin đã là nguyên nhân, đóng hồ sơ.
Nếu vẫn 25% với pin đầy 12.6V và sụt < 0.3V → **không phải pin**, chuyển
sang đo điện trở mối nối dây động cơ.

## Bước 5 — `STEER_TRIM` vòng hở (2 phút, cần một đoạn thẳng trống)

```
./run.sh trim
```

Chưa bao giờ chạy trọn. IMU BNO055 **không** chặn việc này (đã kiểm chứng).

## Bước 6 — Lần đầu kiểm chứng bản sửa zigzag

Bản vá lane-lost decay **chưa bao giờ được chạy trên một cái xe đủ sức đi**.
Run 080230 chạy ở 25% tốc độ lệnh nên bộ điều khiển không hề bị thử ở tốc
độ thật. Nếu bước 5 cho xe đi đúng tốc độ thì đây là lần đầu câu hỏi "hết
zigzag chưa" có câu trả lời.

Nếu xe đi thẳng nhưng phản ứng chậm với khúc quanh, cân nhắc nâng
`PID_MAX_OUTPUT` từ 8 — **nhưng chỉ sau khi có lại số ticks/đơn vị lái ở
tốc độ thật**. Con số 0.30 hiện tại lấy từ run 25% nên không dùng được.

---

## Đừng làm trong buổi này

- Đừng nâng `STEER_CENTER_X` lên 357 trước khi bước 3 xong.
- Đừng chỉnh PID trước khi bước 5 xác nhận bánh đạt được target.
- Đừng nới `MIN_VALID_SCAN_POINTS` trước khi bước 2 nói LiDAR khỏe.
- Đừng hạ độ phân giải hay đổi model vì "GPU hết khả năng".

## Sau mỗi run

Video + log tự kéo về laptop, không cần yêu cầu.

---

## Bước 7 (tuỳ chọn) — Đo độ trễ và độ mạnh lái bằng MTi-630R

Chỉ làm nếu còn thời gian và mang được laptop ra chỗ xe. Hai con số này hiện
đang bị nhiễm: độ trễ ~600ms lấy từ **3 mẫu**, độ mạnh lái 0.30 lấy từ run
bánh chỉ đạt 25%.

Kẹp MTi lên khung xe, cắm USB vào laptop:

```
# terminal 1
source /opt/ros/humble/setup.bash
source /mnt/72be588c-.../ros2_xsens_ws/install/setup.bash
export PYTHONNOUSERSITE=1
ros2 launch xsens_mti_ros2_driver xsens_mti_node.launch.py

# terminal 2
python3 tools/imu_record.py imu_log.csv
```

Rồi — **trước khi cho xe chạy** — xoay thân xe **một cú rõ ràng, không đối
xứng**: quay +40°, giữ 3 giây, về -15°, giữ, rồi về 0. Đó là mốc đồng bộ.
Đừng lắc qua lại theo nhịp (biên an toàn mỏng hơn: 0.18 so với 0.24, ngưỡng 0.15).
Và phải làm lúc xe còn đứng yên — khi xe đã chạy thì lệnh lái làm yaw thành
một đường dốc dài, và hai đường dốc khớp ở mọi độ lệch nên đỉnh tương quan
biến mất.

Cho xe chạy với lệnh lái đổi bước nhiều lần, rồi:

```
python3 tools/sync_imu.py imu_log.csv runs_pulled/<run>/wheel_telem.csv
```

Nó tự từ chối nếu không đủ dữ liệu và nói rõ thiếu gì. Đã kiểm chứng trên dữ
liệu tổng hợp: khôi phục đúng độ lệch đã gieo (-7.350s), độ mạnh lái
(1.800 °/s/đơn vị, r=1.000), và bao được độ trễ thật 420ms trong khoảng
376–476ms.

**Lưu ý về từ kế:** driver đang ở profile `NorthReference`, dùng từ kế cho
hướng tuyệt đối. Trên xe thì từ kế nằm cạnh hai con BTS7960. Yaw tương đối
trong ~10s là do con quay hồi chuyển chi phối nên vẫn dùng được, nhưng nếu số
liệu kỳ lạ thì đổi sang profile VRU (unreferenced yaw).
