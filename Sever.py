"""
Sever.py — Server nhận MQTT alert + gửi Telegram + lưu database
============================================================
Đã refactor để dùng Confg.py
Chạy trên PC (cùng máy với Mosquitto broker):
    python3 Sever.py
"""

import json
import base64
import sqlite3
import os
import requests
import logging
from datetime import datetime
from pathlib import Path

import paho.mqtt.client as mqtt

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
#  IMPORT CẤU HÌNH TỪ Confg.py
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
from Confg import (
    MQTT_PORT, MQTT_TOPIC_ALERT, MQTT_TOPIC_STATUS,
    TELEGRAM_BOT_TOKEN, TELEGRAM_CHAT_ID,
    ALERT_SERVER_SAVE_DIR, PATROL_DB_PATH,
)

# Server chạy local, override IP về localhost (không dùng IP Jetson)
MQTT_BROKER_IP = "localhost"

# ============================================================

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
log = logging.getLogger("patrol-server")

ALERT_SAVE_DIR = Path(ALERT_SERVER_SAVE_DIR)
ALERT_SAVE_DIR.mkdir(exist_ok=True)


# ── Database ─────────────────────────────────────────────────

def init_db():
    con = sqlite3.connect(PATROL_DB_PATH)
    con.execute("""
        CREATE TABLE IF NOT EXISTS alerts (
            id         INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp  TEXT,
            location   TEXT,
            count      INTEGER,
            confidence REAL,
            image_path TEXT,
            sent_ok    INTEGER DEFAULT 0
        )
    """)
    con.execute("""
        CREATE TABLE IF NOT EXISTS status_log (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            timestamp    TEXT,
            location     TEXT,
            speed        INTEGER,
            steer        INTEGER,
            intruder     INTEGER,
            aruco_cmd    TEXT,
            total_alerts INTEGER
        )
    """)
    con.commit()
    con.close()
    log.info(f"Database: {PATROL_DB_PATH}")


# ── Telegram ─────────────────────────────────────────────────

def send_telegram_photo(img_bytes: bytes, caption: str) -> bool:
    url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendPhoto"
    try:
        r = requests.post(
            url,
            data={"chat_id": TELEGRAM_CHAT_ID,
                  "caption": caption, "parse_mode": "HTML"},
            files={"photo": ("alert.jpg", img_bytes, "image/jpeg")},
            timeout=15,
        )
        ok = r.status_code == 200
        if not ok:
            log.warning(f"Telegram lỗi {r.status_code}: {r.text[:150]}")
        return ok
    except Exception as e:
        log.error(f"Telegram exception: {e}")
        return False


def send_telegram_text(text: str) -> bool:
    url = f"https://api.telegram.org/bot{TELEGRAM_BOT_TOKEN}/sendMessage"
    try:
        r = requests.post(
            url,
            data={"chat_id": TELEGRAM_CHAT_ID,
                  "text": text, "parse_mode": "HTML"},
            timeout=10,
        )
        return r.status_code == 200
    except Exception as e:
        log.error(f"Telegram text exception: {e}")
        return False


# ── Xử lý message ────────────────────────────────────────────

def handle_alert(data: dict, con: sqlite3.Connection):
    loc  = data.get("location", "N/A")
    cnt  = data.get("count", 0)
    conf = data.get("confidence", 0.0)
    ts   = data.get("timestamp", "N/A")
    log.warning(f"[ALERT] {loc} | {cnt} người | conf={conf:.0%}")

    img_b64    = data.get("image", "")
    image_path = None
    sent_ok    = False

    if img_b64:
        img_bytes  = base64.b64decode(img_b64)
        ts_safe    = datetime.now().strftime("%Y%m%d_%H%M%S")
        image_path = str(ALERT_SAVE_DIR / f"alert_{ts_safe}.jpg")
        with open(image_path, "wb") as f:
            f.write(img_bytes)

        caption = (
            f"🚨 <b>PHÁT HIỆN XÂM NHẬP!</b>\n"
            f"📍 Vị trí: <b>{loc}</b>\n"
            f"🕐 Thời gian: {ts}\n"
            f"👤 Số người: <b>{cnt}</b>\n"
            f"📊 Độ tin cậy: {conf:.0%}"
        )
        sent_ok = send_telegram_photo(img_bytes, caption)
    else:
        sent_ok = send_telegram_text(
            f"🚨 Xâm nhập tại {loc} lúc {ts} — {cnt} người"
        )

    con.execute(
        "INSERT INTO alerts (timestamp,location,count,confidence,image_path,sent_ok)"
        " VALUES (?,?,?,?,?,?)",
        (ts, loc, cnt, conf, image_path, int(sent_ok)),
    )
    con.commit()


def handle_status(data: dict, con: sqlite3.Connection):
    log.info(
        f"[STATUS] {data.get('location','?')} | "
        f"speed={data.get('speed',0)} steer={data.get('steer',0)} | "
        f"intruder={'⚠️' if data.get('intruder_active') else 'clear'} | "
        f"cmd={data.get('aruco_command','none')} | "
        f"alerts={data.get('total_alerts',0)}"
    )
    con.execute(
        "INSERT INTO status_log"
        " (timestamp,location,speed,steer,intruder,aruco_cmd,total_alerts)"
        " VALUES (?,?,?,?,?,?,?)",
        (
            data.get("timestamp"), data.get("location"),
            data.get("speed"), data.get("steer"),
            int(data.get("intruder_active", False)),
            data.get("aruco_command", "none"),
            data.get("total_alerts", 0),
        ),
    )
    con.commit()


def make_on_message(con: sqlite3.Connection):
    def on_message(client, userdata, msg):
        try:
            data  = json.loads(msg.payload.decode())
            event = data.get("event", "")
            if event == "intruder_detected":
                handle_alert(data, con)
            elif event == "status":
                handle_status(data, con)
        except Exception as e:
            log.error(f"Lỗi xử lý message: {e}")
    return on_message


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        log.info("MQTT: kết nối broker thành công")
        client.subscribe(MQTT_TOPIC_ALERT)
        client.subscribe(MQTT_TOPIC_STATUS)
        send_telegram_text(
            "🟢 <b>Server tuần tra HCMUTE đã khởi động</b>\n"
            f"🕐 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
        )
    else:
        log.error(f"MQTT: lỗi kết nối rc={rc}")


def on_disconnect(client, userdata, rc):
    log.warning(f"MQTT: mất kết nối rc={rc}")


# ── Main ─────────────────────────────────────────────────────

if __name__ == "__main__":
    log.info("Patrol Server — HCMUTE")

    if "YOUR_BOT_TOKEN" in TELEGRAM_BOT_TOKEN:
        log.warning("Chưa cấu hình TELEGRAM_BOT_TOKEN trong Confg.py!")
    if "YOUR_CHAT_ID" in TELEGRAM_CHAT_ID:
        log.warning("Chưa cấu hình TELEGRAM_CHAT_ID trong Confg.py!")

    init_db()
    con = sqlite3.connect(PATROL_DB_PATH, check_same_thread=False)

    client = mqtt.Client(client_id="patrol_server")
    client.on_connect    = on_connect
    client.on_disconnect = on_disconnect
    client.on_message    = make_on_message(con)

    try:
        client.connect(MQTT_BROKER_IP, MQTT_PORT, keepalive=60)
    except Exception as e:
        log.error(f"Không kết nối được MQTT: {e}")
        exit(1)

    log.info("Server đang chạy. Ctrl+C để dừng.")
    try:
        client.loop_forever()
    except KeyboardInterrupt:
        log.info("Dừng server...")
    finally:
        send_telegram_text(
            "🔴 <b>Server tuần tra HCMUTE đã tắt</b>\n"
            f"🕐 {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}"
        )
        client.disconnect()
        con.close()