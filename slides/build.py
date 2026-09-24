#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from gen_slides import wrap, write, foot, svg_icon

# ============================================================ 01. BÌA
body = f"""
  <div style="flex:1;display:flex;flex-direction:column;justify-content:center;max-width:980px;">
    <div class="eyebrow">Báo cáo tiến độ &middot; ISLAB &mdash; HCMUTE</div>
    <div style="font-size:58px;font-weight:700;line-height:1.05;letter-spacing:-0.02em;margin:0 0 18px 0;">
      AGV Patrol Robot
    </div>
    <div style="font-size:21px;line-height:1.5;color:var(--text-dim);max-width:850px;margin:0 0 34px 0;">
      Xe tuần tra tự hành bám làn &mdash; quá trình tìm nguyên nhân gốc
      và khắc phục lỗi <span class="accent" style="font-weight:600;">&ldquo;xe lệch phải, không tự kéo lại&rdquo;</span>
    </div>
    <div style="display:flex;gap:36px;align-items:center;">
      <div class="mono" style="font-size:14px;color:var(--text);">Lê Tự Thiện An</div>
      <div style="width:1px;height:16px;background:var(--border);"></div>
      <div class="mono" style="font-size:14px;color:var(--text-dim);">ISLAB &mdash; HCMUTE</div>
      <div style="width:1px;height:16px;background:var(--border);"></div>
      <div class="mono" style="font-size:14px;color:var(--text-dim);">10&#8202;/&#8202;09&#8202;/&#8202;2026</div>
    </div>
  </div>

  <div style="position:absolute;right:64px;bottom:120px;">
    <div class="card" style="width:210px;padding:16px 18px;display:flex;flex-direction:column;gap:8px;">
      <div class="tile-label mono" style="letter-spacing:0.1em;">3 TẦNG LỖI CHỒNG NHAU</div>
      <div style="display:flex;justify-content:space-between;align-items:center;"><span style="font-size:15px;font-weight:600;">Nhận diện</span><span class="pill pill-good">OK</span></div>
      <div style="display:flex;justify-content:space-between;align-items:center;"><span style="font-size:15px;font-weight:600;">Điều khiển</span><span class="pill pill-good">OK</span></div>
      <div style="display:flex;justify-content:space-between;align-items:center;"><span style="font-size:15px;font-weight:600;">Cơ khí / lực lái</span><span class="pill pill-accent">ĐANG ĐO</span></div>
    </div>
  </div>
"""
write("Main", wrap("Main", "Bìa", body + foot(1, "AGV Patrol Robot")))

# ============================================================ 02. HỆ THỐNG & NHIỆM VỤ
def row(icon, title, sub):
    return f"""
      <div style="display:flex;gap:14px;align-items:flex-start;">
        <div style="flex:none;width:34px;height:34px;border-radius:6px;background:var(--accent-soft);display:flex;align-items:center;justify-content:center;">{svg_icon(icon, 18)}</div>
        <div>
          <div style="font-size:15px;font-weight:600;">{title}</div>
          <div style="font-size:13px;color:var(--text-dim);line-height:1.4;">{sub}</div>
        </div>
      </div>
"""

hw_rows = "".join([
    row("cpu", "Jetson", "Chạy AI: phân đoạn làn + phát hiện người lạ, vòng lặp điều khiển chính"),
    row("wheel", "STM32 Nucleo&#8209;F411RE + 2&times; BTS7960", "Nhận lệnh steer/speed, điều khiển PI cho từng bánh qua encoder"),
    row("radar", "RPLidar", "Quét 360&deg;, dừng &le;&#8202;1.2m &middot; đi tiếp &ge;&#8202;1.6m"),
    row("gauge", "BNO055 IMU", "Góc nghiêng &amp; yaw &mdash; phát hiện bị nhấc lên, đo lệch cơ khí"),
    row("camera", "Logitech C270", "640&times;360, đầu vào duy nhất cho khối phân đoạn làn"),
])

mission_rows = "".join([
    row("route", "Tuần tra tự động", "Bám làn theo vạch sơn dọc hành lang / đường nội bộ"),
    row("shield", "Né vật cản", "Dừng khẩn khi LiDAR thấy vật &lt;&#8202;1.2m, né khi bị chặn quá lâu"),
    row("person", "Cảnh báo người lạ", "YOLOv8n riêng, phát hiện &amp; gửi cảnh báo qua MQTT / Discord"),
    row("target", "Tự bảo vệ", "IMU phát hiện bị nhấc lên, tự dừng khi mất vạch &ge;&#8202;8 khung"),
])

body = f"""
  <div class="eyebrow">Hệ thống</div>
  <div class="h1">Nhiệm vụ &amp; kiến trúc phần cứng</div>
  <div class="rule"></div>
  <div style="flex:1;display:grid;grid-template-columns:1fr 1fr;gap:40px;">
    <div>
      <div class="h2" style="font-size:18px;margin-bottom:16px;">Nhiệm vụ</div>
      <div style="display:flex;flex-direction:column;gap:20px;">{mission_rows}</div>
    </div>
    <div>
      <div class="h2" style="font-size:18px;margin-bottom:16px;">Phần cứng</div>
      <div style="display:flex;flex-direction:column;gap:20px;">{hw_rows}</div>
    </div>
  </div>
"""
write("Overview", wrap("Overview", "Hệ thống", body + foot(2, "Kiến trúc phần cứng")))

# ============================================================ 03. KIẾN TRÚC AI
def pbox(label, sub, w=172):
    return f"""<div style="flex:none;width:{w}px;background:var(--surface);border:1px solid var(--border);border-radius:6px;padding:14px 14px;text-align:center;">
      <div style="font-size:14px;font-weight:600;line-height:1.25;">{label}</div>
      <div style="font-size:11.5px;color:var(--text-dim);margin-top:4px;">{sub}</div>
    </div>"""

def parrow():
    return '<div style="flex:none;color:var(--text-faint);display:flex;align-items:center;">' + svg_icon("arrow", 20, "var(--text-faint)") + '</div>'

pipeline = f"""
  <div style="display:flex;align-items:center;gap:8px;">
    {pbox("Camera", "640&times;360")}
    {parrow()}
    {pbox("DeepLabV3 +<br>MobileNetV2", "phân đoạn 6 lớp", 196)}
    {parrow()}
    {pbox("scan_lane()", "dò tâm vạch")}
    {parrow()}
    {pbox("PID", "KP&nbsp;&middot;&nbsp;KI&nbsp;&middot;&nbsp;KD")}
    {parrow()}
    {pbox("STM32", "steer / speed")}
  </div>
"""

classes = ["background","road","line","car","motobike","person"]
class_chips = "".join([f'<span class="pill pill-accent" style="margin:3px 6px 3px 0;">{c}</span>' for c in classes])

body = f"""
  <div class="eyebrow">Kiến trúc AI</div>
  <div class="h1">Đường xử lý nhận thức &amp; điều khiển</div>
  <div class="rule"></div>

  <div class="card" style="padding:26px 22px;">
    {pipeline}
  </div>

  <div style="display:grid;grid-template-columns:1.3fr 1fr;gap:24px;margin-top:24px;">
    <div class="card">
      <div class="h2" style="font-size:16px;">Phân đoạn ngữ nghĩa &mdash; 6 lớp</div>
      <div style="margin-top:10px;">{class_chips}</div>
      <div style="font-size:13px;color:var(--text-dim);margin-top:14px;line-height:1.5;">
        Đầu vào 256&times;448, chạy fp16 trên Jetson &mdash; <span class="mono accent">~72&#8202;ms/khung</span> suy luận thuần.
        Lớp <span class="mono">road</span> phủ gần hết khung hình (không có lề đường trong khung),
        nên chỉ <span class="mono">line</span> mang thông tin vị trí thật sự.
      </div>
    </div>
    <div class="card">
      <div class="h2" style="font-size:16px;">Phát hiện người lạ</div>
      <div style="font-size:13px;color:var(--text-dim);margin-top:10px;line-height:1.6;">
        YOLOv8n riêng, chạy <span class="accent" style="font-weight:600;">song song</span>,
        <span style="font-weight:600;color:var(--text);">không tham gia lái xe</span>.
        Chỉ phục vụ cảnh báo xâm nhập qua MQTT &amp; Discord.
      </div>
    </div>
  </div>
"""
write("Perception", wrap("Perception", "Kiến trúc AI", body + foot(3, "Kiến trúc xử lý AI")))

print("Da tao 3 slide dau")

# ============================================================ 04. VAN DE
def layerchip(n, name, status_pill, active=False):
    bg = "var(--accent-soft)" if active else "var(--surface)"
    bd = "var(--accent-line)" if active else "var(--border)"
    return f"""<div style="flex:1;background:{bg};border:1px solid {bd};border-radius:6px;padding:20px 18px;">
      <div class="mono" style="font-size:12px;color:var(--text-faint);">TẦNG {n}</div>
      <div style="font-size:17px;font-weight:600;margin:6px 0 10px 0;">{name}</div>
      {status_pill}
    </div>"""

body = f"""
  <div class="eyebrow">Vấn đề</div>
  <div class="h1">&ldquo;Xe lệch phải mà không tự kéo lại&rdquo;</div>
  <div class="lede">
    Triệu chứng lặp lại suốt nhiều tuần thử nghiệm: xe trôi dần sang phải, bộ điều khiển
    có phản ứng &mdash; nhưng không đủ để xe quay về vạch. Mỗi lần sửa xong một chỗ,
    triệu chứng vẫn y như cũ.
  </div>
  <div class="rule"></div>

  <div style="flex:1;display:flex;flex-direction:column;justify-content:center;gap:22px;">
    <div style="font-size:15px;color:var(--text-dim);">
      Hoá ra đây không phải một lỗi &mdash; mà là
      <span class="accent" style="font-weight:700;">ba lỗi chồng lên nhau ở ba tầng khác nhau</span>.
      Sửa đúng một tầng thì hai tầng kia vẫn che mất kết quả, nên nhìn từ ngoài triệu chứng không đổi.
    </div>
    <div style="display:flex;gap:18px;">
      {layerchip(1, "Nhận diện", '<span class="pill pill-good">' + svg_icon("check", 12, "var(--good)") + ' ĐÃ SỬA</span>')}
      {layerchip(2, "Điều khiển", '<span class="pill pill-good">' + svg_icon("check", 12, "var(--good)") + ' ĐÃ SỬA</span>')}
      {layerchip(3, "Cơ khí &mdash; lực lái", '<span class="pill pill-accent">GỐC RỄ THẬT SỰ</span>', active=True)}
    </div>
  </div>
"""
write("Problem", wrap("Problem", "Van de", body + foot(4, "Ba tầng lỗi chồng nhau")))

# ============================================================ 05. TANG 1 - DU LIEU
stat_tiles_5 = "".join([f"""
  <div class="tile">
    <div class="tile-label">{lbl}</div>
    <div class="tile-num">{num}</div>
    <div class="tile-sub">{sub}</div>
  </div>""" for lbl, num, sub in [
    ("Bộ dữ liệu", "3&#8202;052 ảnh", "2&#8202;673 train &middot; 252 valid &middot; 127 test"),
    ("Nhãn &lsquo;line&rsquo; / ảnh", "2.33&nbsp;&rarr;&nbsp;1.01", "vạch đứt &rarr; một nét liền qua khoảng trống"),
    ("Pixel vạch nhận được", "<span class=\"good\">+83%</span>", "so với model cũ, trên ảnh test chưa từng train"),
    ("Độ phủ theo hàng quét", "<span class=\"good\">+64%</span>", "27/30 ảnh test tốt hơn rõ rệt"),
]])

body = f"""
  <div class="eyebrow">Tầng 1 &middot; Nhận diện</div>
  <div class="h1">Nhãn vạch liền mạch thay vì đứt nét</div>
  <div class="rule"></div>

  <div style="display:flex;gap:24px;flex:1;">
    <div style="flex:none;width:560px;">
      <img src="compare_line.jpg" style="width:100%;border-radius:6px;border:1px solid var(--border);display:block;">
      <div style="display:flex;justify-content:space-between;margin-top:8px;font-size:12px;color:var(--text-faint);">
        <span>&larr; Model cũ &mdash; vạch đứt đoạn</span>
        <span>Model mới &mdash; vạch liền mạch &rarr;</span>
      </div>
      <div style="font-size:13px;color:var(--text-dim);margin-top:14px;line-height:1.55;">
        Vạch sơn ngoài đời là đứt nét. Việc dán nhãn lại thành một đường liền
        qua các khoảng trống giúp <span class="mono">scan_lane()</span> không phải đoán
        mò giữa hai đoạn vạch &mdash; đây là cải thiện có tác động lớn nhất trong toàn bộ dự án.
      </div>
    </div>
    <div style="flex:1;display:grid;grid-template-columns:1fr 1fr;gap:14px;align-content:start;">
      {stat_tiles_5}
    </div>
  </div>
"""
write("Data", wrap("Data", "Tang 1", body + foot(5, "Tầng 1 — Dữ liệu & nhận diện")))

# ============================================================ 06. TANG 2 - DIEU KHIEN THUAN P
body = f"""
  <div class="eyebrow">Tầng 2 &middot; Điều khiển</div>
  <div class="h1">Bộ điều khiển chỉ chạy khâu P</div>
  <div class="rule"></div>

  <div style="display:grid;grid-template-columns:1fr 1fr;gap:28px;flex:1;">
    <div>
      <div class="card" style="margin-bottom:16px;">
        <div class="h2" style="font-size:16px;">Vì sao xe dừng lại&nbsp;&mdash; không phải ở 0</div>
        <div style="font-size:13.5px;color:var(--text-dim);line-height:1.6;margin-top:8px;">
          <span class="mono">KI&nbsp;=&nbsp;0.01</span> với tích phân bị chặn ở <span class="mono">&plusmn;20</span>
          &rarr; đóng góp tối đa <span class="mono accent">0.2&nbsp;/&nbsp;20&nbsp;&asymp;&nbsp;1%</span>.
          Bộ điều khiển chạy gần như thuần khâu&nbsp;P &mdash; mà khâu P
          <span style="color:var(--text);font-weight:600;">không thể khử sai số xác lập</span>:
          nó dừng đúng chỗ mà <span class="mono">KP&nbsp;&times;&nbsp;lỗi</span> vừa đủ cân bằng nhiễu.
        </div>
      </div>
      <div class="card">
        <div class="h2" style="font-size:16px;">Sửa</div>
        <div style="font-size:13.5px;color:var(--text-dim);line-height:1.6;margin-top:8px;">
          <span class="mono">KI&nbsp;0.01&nbsp;&rarr;&nbsp;0.03</span>, giới hạn tích phân
          <span class="mono">&plusmn;20&nbsp;&rarr;&nbsp;&plusmn;400</span>, kèm
          <span style="color:var(--text);font-weight:600;">chống windup có điều kiện</span>
          &mdash; ngừng tích luỹ khi đầu ra đã bão hoà.
        </div>
      </div>
    </div>
    <div class="card" style="display:flex;flex-direction:column;">
      <div class="h2" style="font-size:16px;">Đo trên đường &mdash; 403 khung, 2 phút</div>
      <div style="flex:1;display:flex;flex-direction:column;justify-content:center;gap:18px;margin-top:8px;">
        <div style="display:flex;justify-content:space-between;align-items:baseline;">
          <span style="font-size:13px;color:var(--text-dim);">Chạy hướng&nbsp;A</span>
          <span class="mono" style="font-size:22px;font-weight:600;">lỗi đứng ở <span class="bad">+55px</span></span>
        </div>
        <div style="display:flex;justify-content:space-between;align-items:baseline;">
          <span style="font-size:13px;color:var(--text-dim);">Chạy hướng&nbsp;B</span>
          <span class="mono" style="font-size:22px;font-weight:600;">lỗi đứng ở <span class="bad">&minus;78px</span></span>
        </div>
        <div class="rule" style="margin:4px 0;"></div>
        <div style="font-size:12.5px;color:var(--text-faint);line-height:1.5;">
          Cả hai đều <span style="color:var(--text-dim);">ổn định</span> (độ lệch chuẩn tụt còn 10px)
          nhưng đều khác 0 &mdash; đúng con số mà <span class="mono">KP&nbsp;&times;&nbsp;lỗi</span> dự đoán.
          Ảnh chụp xác nhận model bám đúng vạch sơn: đây không phải lỗi nhận diện.
        </div>
      </div>
    </div>
  </div>
"""
write("Control", wrap("Control", "Tang 2", body + foot(6, "Tầng 2 — Bộ điều khiển")))

# ============================================================ 07. TANG 3 - GOC RE (FIRMWARE)
body = f"""
  <div class="eyebrow">Tầng 3 &middot; Cơ khí / firmware</div>
  <div class="h1">Gốc rễ thật sự: lực lái quá yếu</div>
  <div class="rule"></div>

  <div style="display:grid;grid-template-columns:1fr 1fr;gap:28px;flex:1;">
    <div style="display:flex;flex-direction:column;gap:16px;">
      <div class="card">
        <div class="h2" style="font-size:16px;">Hai hằng số mâu thuẫn nhau &mdash; trong cùng 1 file firmware</div>
        <div class="mono" style="font-size:13px;margin-top:10px;line-height:1.9;">
          <div><span class="bad">SPEED_REFERENCE&nbsp;=&nbsp;90</span></div>
          <div><span class="text-dim">SPEED_COMMAND_MAX&nbsp;=&nbsp;25</span> &nbsp;<span style="color:var(--text-faint);">(giới hạn thật)</span></div>
        </div>
        <div style="font-size:13px;color:var(--text-dim);line-height:1.55;margin-top:10px;">
          Đầu vào không bao giờ vượt 25, nhưng thang quy đổi lấy mốc 90
          &rarr; lệnh cho bánh xe luôn kẹt trong <span class="mono">0&ndash;11</span> của thang <span class="mono">70</span>.
        </div>
      </div>
      <div class="card">
        <div class="h2" style="font-size:16px;">Đo trực tiếp trên đường, có tải</div>
        <div style="display:flex;justify-content:space-between;align-items:center;margin-top:10px;">
          <span style="font-size:13px;color:var(--text-dim);">Bẻ lái <span style="color:var(--text);font-weight:600;">HẾT CỠ</span></span>
          <span class="mono" style="font-size:24px;font-weight:700;">20&nbsp;<span style="color:var(--text-faint);font-weight:400;">/</span>&nbsp;20</span>
        </div>
        <div style="font-size:12.5px;color:var(--bad);margin-top:2px;">hai bánh quay giống hệt nhau &mdash; chênh lệch&nbsp;=&nbsp;0</div>
      </div>
    </div>

    <div style="display:flex;flex-direction:column;gap:16px;">
      <img src="hud_drift.jpg" style="width:100%;border-radius:6px;border:1px solid var(--border);display:block;">
      <div class="card" style="flex:1;">
        <div class="h2" style="font-size:16px;">Sau khi sửa firmware</div>
        <div style="display:flex;gap:22px;margin-top:12px;">
          <div>
            <div class="tile-label">SPEED_REFERENCE</div>
            <div class="mono" style="font-size:18px;font-weight:600;">90&nbsp;&rarr;&nbsp;25</div>
          </div>
          <div>
            <div class="tile-label">PWM_MAX</div>
            <div class="mono" style="font-size:18px;font-weight:600;">90&nbsp;&rarr;&nbsp;140</div>
          </div>
          <div>
            <div class="tile-label">Lực lái (giá thử)</div>
            <div class="mono" style="font-size:18px;font-weight:600;">&plusmn;6&nbsp;&rarr;&nbsp;<span class="good">&plusmn;20</span></div>
          </div>
        </div>
        <div style="font-size:12.5px;color:var(--text-faint);margin-top:12px;">
          Đã nạp &amp; kiểm chứng trên giá &mdash; đang chờ đo lại có tải trên đường.
        </div>
      </div>
    </div>
  </div>
"""
write("RootCause", wrap("RootCause", "Tang 3", body + foot(7, "Tầng 3 — Gốc rễ: lực lái")))

# ============================================================ 08. AN TOAN
def safe_row(icon, title, sub):
    return f"""
    <div style="display:flex;gap:16px;align-items:flex-start;background:var(--surface);border:1px solid var(--border);border-radius:6px;padding:16px 18px;">
      <div style="flex:none;width:36px;height:36px;border-radius:6px;background:var(--good-soft);display:flex;align-items:center;justify-content:center;">{svg_icon(icon, 18, "var(--good)")}</div>
      <div>
        <div style="font-size:15px;font-weight:600;">{title}</div>
        <div style="font-size:13px;color:var(--text-dim);line-height:1.45;margin-top:2px;">{sub}</div>
      </div>
    </div>"""

body = f"""
  <div class="eyebrow">An toàn vận hành</div>
  <div class="h1">Nhiều lớp bảo vệ &mdash; độc lập với AI</div>
  <div class="rule"></div>

  <div style="display:grid;grid-template-columns:1fr 1fr;gap:16px;flex:1;align-content:start;">
    {safe_row("radar", "Dừng khẩn theo LiDAR", "&le;&#8202;1.2m dừng ngay &middot; &ge;&#8202;1.6m mới cho đi tiếp (chống rung theo ngưỡng)")}
    {safe_row("clock", "Watchdog 500&nbsp;ms", "Mất tín hiệu điều khiển từ Jetson &gt; 500ms &rarr; STM32 tự dừng động cơ")}
    {safe_row("layers", "Tự dừng khi mất vạch", "&ge;&#8202;8 khung liên tiếp không thấy vạch thật &rarr; giảm tốc về 0")}
    {safe_row("shield", "Né vật đòi hỏi AI còn sống", "Lỗi vừa tìm thấy: bridge từng tự lái khi CHƯA có AI kết nối &mdash; đã vá")}
  </div>

  <div class="card" style="margin-top:6px;">
    <div style="font-size:13px;color:var(--text-dim);line-height:1.55;">
      <span class="bad" style="font-weight:600;">Phát hiện trong buổi kiểm thử:</span>
      chạy bridge một mình, phòng thí nghiệm chật &mdash; LiDAR coi tường cách 46&#8202;cm là vật cản,
      sau 10 giây máy trạng thái né vật tự ra lệnh quay bánh dù <span style="color:var(--text);">chưa hề có AI nào kết nối</span>.
      Đã sửa: né vật giờ đòi hỏi lệnh AI còn sống trong 1.4s gần nhất.
    </div>
  </div>
"""
write("Safety", wrap("Safety", "An toan", body + foot(8, "Hệ thống an toàn")))

# ============================================================ 09. KET QUA
dash = "".join([f"""
  <div class="tile">
    <div class="tile-label">{lbl}</div>
    <div style="display:flex;align-items:baseline;gap:8px;">
      <span class="tile-num" style="font-size:24px;color:var(--text-faint);">{before}</span>
      <span class="arrow">&rarr;</span>
      <span class="tile-num" style="font-size:24px;">{after}</span>
    </div>
    <div class="tile-sub">{sub}</div>
  </div>""" for lbl, before, after, sub in [
    ("Tốc độ vòng lặp", "1.85", "<span class=\"good\">3.3</span> fps", "tách YOLO khỏi đường điều khiển"),
    ("Tỉ lệ nhận vạch", "&mdash;", "<span class=\"good\">97&ndash;100%</span>", "đo trên 3 lần chạy đường thật"),
    ("Lực lái (giá thử)", "&plusmn;6", "<span class=\"good\">&plusmn;20</span> ticks", "sau khi sửa firmware"),
    ("Tốc độ tới động cơ", "2 mức", "<span class=\"good\">5 bậc</span>", "thang đơn điệu, hết bị kẹp"),
    ("Cơ chế cấu hình &lsquo;chết&rsquo;", "&mdash;", "<span class=\"good\">4</span> đã sửa", "chạy nhưng không có / sai tác dụng"),
    ("Pixel vạch nhận diện", "nền", "<span class=\"good\">+83%</span>", "sau khi nhãn lại dữ liệu"),
]])

body = f"""
  <div class="eyebrow">Kết quả</div>
  <div class="h1">Số liệu đo được &mdash; trước / sau</div>
  <div class="rule"></div>
  <div style="flex:1;display:grid;grid-template-columns:repeat(3,1fr);gap:16px;align-content:start;">
    {dash}
  </div>
"""
write("Results", wrap("Results", "Ket qua", body + foot(9, "Kết quả đo được")))

# ============================================================ 10. TON DONG & KE HOACH
def plan_row(status, title, sub):
    if status == "done":
        pill = '<span class="pill pill-good">' + svg_icon("check", 12, "var(--good)") + ' XONG</span>'
    else:
        pill = '<span class="pill pill-accent">' + svg_icon("clock", 12, "var(--accent)") + ' TIẾP THEO</span>'
    return f"""
    <div style="display:flex;justify-content:space-between;align-items:flex-start;gap:16px;padding:13px 0;border-bottom:1px solid var(--border);">
      <div>
        <div style="font-size:14.5px;font-weight:600;">{title}</div>
        <div style="font-size:12.5px;color:var(--text-dim);margin-top:2px;line-height:1.4;">{sub}</div>
      </div>
      <div style="flex:none;">{pill}</div>
    </div>"""

done_rows = "".join([
    plan_row("done", "Lỗi nhận diện heading", "get_heading_error() bịa điểm xa từ nền road &mdash; đã triệt tiêu sai lệch"),
    plan_row("done", "Nhãn dữ liệu liền mạch", "+83% pixel vạch, +64% độ phủ &mdash; đã train &amp; triển khai model mới"),
    plan_row("done", "Khâu tích phân PID", "Thêm KI thật sự &plusmn; chống windup có điều kiện"),
    plan_row("done", "Firmware lực lái", "SPEED_REFERENCE &amp; PWM_MAX &mdash; đã nạp, xác nhận trên giá"),
])
next_rows = "".join([
    plan_row("next", "Đo lực lái CÓ TẢI trên đường", "PWM_MAX chỉ bị siết khi có tải &mdash; giá thử không kiểm chứng được"),
    plan_row("next", "Hiệu chỉnh STEER_CENTER_X", "Điểm ngắm camera lệch ~40px so với tâm vạch thật &mdash; công cụ đo đã sẵn (run snap)"),
    plan_row("next", "Đo STEER_TRIM (hở vòng)", "Xe tự trôi phải dù 2 bánh cân bằng &mdash; do lệch cơ khí, đo bằng yaw IMU (run trim)"),
    plan_row("next", "Tinh chỉnh lại PID", "Chỉ sau khi lực lái đủ &mdash; khâu I mới có vùng tuyến tính để làm việc"),
])

body = f"""
  <div class="eyebrow">Kế hoạch</div>
  <div class="h1">Đã xong &amp; bước tiếp theo</div>
  <div class="rule"></div>
  <div style="display:grid;grid-template-columns:1fr 1fr;gap:36px;flex:1;">
    <div>
      <div class="h2" style="font-size:16px;margin-bottom:6px;">Đã hoàn thành</div>
      <div>{done_rows}</div>
    </div>
    <div>
      <div class="h2" style="font-size:16px;margin-bottom:6px;">Tiếp theo</div>
      <div>{next_rows}</div>
    </div>
  </div>
  <div class="card" style="margin-top:18px;">
    <div style="font-size:13px;color:var(--text-dim);">
      <span style="color:var(--text);font-weight:600;">Công cụ mới:</span>
      <span class="mono accent">run</span> &mdash; một lệnh cho mọi chế độ thử (đường thật / giá / khô / hiệu chỉnh) &middot;
      <span class="mono accent">verdict.py</span> &mdash; tự chấm điểm mỗi lần chạy theo đúng 3 tầng lỗi ở trên.
    </div>
  </div>
"""
write("NextSteps", wrap("NextSteps", "Ke hoach", body + foot(10, "Còn tồn đọng & kế hoạch")))

print("Da tao du 10 slide")
