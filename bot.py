#!/usr/bin/env python3
"""
Pajero Node -> Telegram alert bot.

Runs 24/7 on Railway. Subscribes to the car node's MQTT topics on HiveMQ and
pushes a Telegram message when something happens: the car starts moving, leaves
or arrives home, the gate auto-opens, or the node stops reporting entirely.

This replaces the old yad2 bot code in the same Railway project. All secrets come
from Railway environment variables -- nothing sensitive is committed to GitHub.

Test build: HiveMQ credentials are hardcoded below. Fill in the two Telegram
values (TG_TOKEN, TG_CHAT) and it runs with no configuration at all.
Any value can still be overridden by a Railway environment variable of the
same name if you later want them out of the code.
"""

import json
import os
import threading
import time

import paho.mqtt.client as mqtt
import requests

# ---------------------------------------------------------------- config
MQTT_HOST = os.getenv("MQTT_HOST", "7e8b557fbc1e49c6b158e020ea18b265.s1.eu.hivemq.cloud")
MQTT_PORT = int(os.getenv("MQTT_PORT", "8883"))          # 8883 = MQTT over TLS
MQTT_USER = os.getenv("MQTT_USER", "pajero2")
MQTT_PASS = os.getenv("MQTT_PASS", "pajeropassword123")

DEVICE_ID = os.getenv("DEVICE_ID", "pajero1")
T_EVENT = f"pajero/{DEVICE_ID}/event"
T_STATUS = f"pajero/{DEVICE_ID}/status"

# --- Telegram: bot @yad2_pajero_alerts_bot -> chat 7979565535 (Gidi) ---
TG_TOKEN = os.getenv("TELEGRAM_TOKEN", "8768067779:AAF67VdHLw61SamevCP2OIy2k9zQJUQLV_Q")
TG_CHAT = os.getenv("TELEGRAM_CHAT_ID", "7979565535")
TG_API = f"https://api.telegram.org/bot{TG_TOKEN}"

OFFLINE_AFTER = int(os.getenv("OFFLINE_AFTER_MIN", "10")) * 60   # silence = alarm
DEDUPE_SEC = int(os.getenv("DEDUPE_SEC", "30"))

# ---------------------------------------------------------------- state
state = {
    "last_status": None,      # most recent parsed status dict
    "last_status_at": 0.0,    # epoch seconds
    "offline_alerted": False,
    "muted_until": 0.0,
    "recent": {},             # event type -> last seen epoch (de-duplication)
    "started_at": time.time(),
}
lock = threading.Lock()


# ---------------------------------------------------------------- telegram
def tg_send(text: str, force: bool = False) -> None:
    """Send a Telegram message. Respects mute unless force=True."""
    if not TG_TOKEN or not TG_CHAT:
        print("[tg] not configured, would send:", text)
        return
    if not force and time.time() < state["muted_until"]:
        print("[tg] muted, skipped:", text.splitlines()[0])
        return
    try:
        r = requests.post(
            f"{TG_API}/sendMessage",
            json={
                "chat_id": TG_CHAT,
                "text": text,
                "parse_mode": "HTML",
                "disable_web_page_preview": True,
            },
            timeout=15,
        )
        if r.status_code != 200:
            print("[tg] error", r.status_code, r.text[:200])
    except Exception as exc:                                   # noqa: BLE001
        print("[tg] send failed:", exc)


def maplink(lat, lon) -> str:
    if lat in (None, 0) and lon in (None, 0):
        return ""
    return f'\n<a href="https://maps.google.com/?q={lat},{lon}">{lat:.5f}, {lon:.5f}</a>'


# ---------------------------------------------------------------- formatting
def describe(ev: dict) -> str | None:
    """Turn one event payload into a message. Returns None to stay silent."""
    kind = ev.get("ev", "")
    lat, lon = ev.get("lat"), ev.get("lon")
    spd = ev.get("spd")
    where = maplink(lat, lon)

    if kind == "drive_start":
        s = f" at {spd:.0f} km/h" if isinstance(spd, (int, float)) else ""
        return f"🚗 <b>Pajero started moving</b>{s}{where}"
    if kind == "drive_stop":
        return f"🅿️ <b>Pajero stopped</b>{where}"
    if kind == "geofence_exit":
        return f"🚨 <b>Left the home area</b>{where}"
    if kind == "geofence_enter":
        return f"🏠 <b>Arrived home</b>{where}"
    if kind == "gate_auto":
        return "🚪 <b>Gate opened automatically</b> on arrival"
    if kind == "home_set":
        return f"⚙️ Geofence centre set{where}"
    if kind == "geo_set":
        return f"⚙️ Geofence updated{where}"
    return f"ℹ️ Event: <code>{kind}</code>{where}"


def dedupe(kind: str) -> bool:
    """True if this event type fired too recently (suppress it)."""
    now = time.time()
    with lock:
        last = state["recent"].get(kind, 0)
        state["recent"][kind] = now
    return (now - last) < DEDUPE_SEC


# ---------------------------------------------------------------- mqtt
def on_connect(client, userdata, flags, reason_code, properties=None):
    if reason_code == 0:
        client.subscribe([(T_EVENT, 1), (T_STATUS, 0)])
        print(f"[mqtt] connected, subscribed to {T_EVENT} and {T_STATUS}")
    else:
        print("[mqtt] connect failed:", reason_code)


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload.decode("utf-8", "replace"))
    except Exception:                                          # noqa: BLE001
        print("[mqtt] unparsable payload on", msg.topic)
        return

    if msg.topic == T_STATUS:
        with lock:
            state["last_status"] = payload
            state["last_status_at"] = time.time()
            was_offline = state["offline_alerted"]
            state["offline_alerted"] = False
        if was_offline:
            tg_send("✅ <b>Node back online</b>", force=True)
        return

    if msg.topic == T_EVENT:
        kind = payload.get("ev", "")
        print("[event]", payload)
        if dedupe(kind):
            print("[event] deduped:", kind)
            return
        text = describe(payload)
        if text:
            # Leaving home unexpectedly is the one alert that ignores mute.
            tg_send(text, force=(kind == "geofence_exit"))


# ---------------------------------------------------------------- watchdog
def watchdog():
    """Alert if the node goes quiet -- covers power loss, tow, no coverage."""
    while True:
        time.sleep(60)
        with lock:
            last = state["last_status_at"]
            alerted = state["offline_alerted"]
            running_for = time.time() - state["started_at"]
        if last == 0:
            # never heard from it; only complain once we've been up a while
            if running_for > OFFLINE_AFTER and not alerted:
                with lock:
                    state["offline_alerted"] = True
                tg_send("⚠️ <b>No contact with the node</b> since the bot started.",
                        force=True)
            continue
        gap = time.time() - last
        if gap > OFFLINE_AFTER and not alerted:
            with lock:
                state["offline_alerted"] = True
            tg_send(f"⚠️ <b>Node offline</b> — no report for {int(gap // 60)} min.",
                    force=True)


# ---------------------------------------------------------------- commands
def status_text() -> str:
    with lock:
        st = state["last_status"]
        at = state["last_status_at"]
    if not st:
        return "No status received yet."
    age = int(time.time() - at)
    lat, lon = st.get("lat"), st.get("lon")
    lines = [
        f"<b>Pajero</b> — reported {age}s ago",
        f"Fix: {st.get('fix', '?')}   Sats: {st.get('sats', '-')}",
        f"Signal: {st.get('csq', '-')}   Battery: {st.get('bat', '-')}%",
        f"Driving: {'yes' if st.get('drive') else 'no'}"
        f"   Home: {'inside' if st.get('inside') else 'away'}",
    ]
    if lat and lon:
        lines.append(maplink(lat, lon).strip())
    return "\n".join(lines)


def poll_commands():
    """Minimal Telegram long-poll. Read-only commands, locked to your chat id."""
    if not TG_TOKEN:
        return
    offset = None
    while True:
        try:
            r = requests.get(
                f"{TG_API}/getUpdates",
                params={"timeout": 50, "offset": offset},
                timeout=60,
            )
            for upd in r.json().get("result", []):
                offset = upd["update_id"] + 1
                msg = upd.get("message") or {}
                chat_id = str((msg.get("chat") or {}).get("id", ""))
                text = (msg.get("text") or "").strip().lower()
                if not text:
                    continue
                if TG_CHAT and chat_id != str(TG_CHAT):
                    print("[tg] ignoring message from", chat_id)
                    continue

                if text.startswith("/status") or text.startswith("/where"):
                    tg_send(status_text(), force=True)
                elif text.startswith("/mute"):
                    parts = text.split()
                    mins = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else 60
                    with lock:
                        state["muted_until"] = time.time() + mins * 60
                    tg_send(f"🔕 Muted for {mins} min (theft alerts still come through).",
                            force=True)
                elif text.startswith("/unmute"):
                    with lock:
                        state["muted_until"] = 0
                    tg_send("🔔 Alerts back on.", force=True)
                elif text.startswith("/start") or text.startswith("/help"):
                    tg_send(
                        "Pajero node bot.\n"
                        "/status — latest telemetry and position\n"
                        "/mute 60 — pause routine alerts\n"
                        "/unmute — resume",
                        force=True,
                    )
        except Exception as exc:                               # noqa: BLE001
            print("[tg] poll error:", exc)
            time.sleep(5)


# ---------------------------------------------------------------- main
def main():
    if "PASTE_" in TG_TOKEN or "PASTE_" in TG_CHAT:
        print("!! TG_TOKEN / TG_CHAT still contain placeholders — "
              "alerts will print to the log instead of Telegram.")

    threading.Thread(target=watchdog, daemon=True).start()
    threading.Thread(target=poll_commands, daemon=True).start()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                         client_id=f"pajero-bot-{int(time.time())}")
    client.username_pw_set(MQTT_USER, MQTT_PASS)
    client.tls_set()                        # system CA store; HiveMQ uses Let's Encrypt
    client.reconnect_delay_set(min_delay=1, max_delay=60)
    client.on_connect = on_connect
    client.on_message = on_message

    print(f"[mqtt] connecting to {MQTT_HOST}:{MQTT_PORT} …")
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_forever(retry_first_connection=True)


if __name__ == "__main__":
    main()
