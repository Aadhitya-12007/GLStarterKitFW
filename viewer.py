#!/usr/bin/env python3
"""BlackBox-EDR live MQTT crash plotter and remote controller."""

import queue
import re
import sys
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import paho.mqtt.client as mqtt

BROKER, PORT = "test.mosquitto.org", 1883
DT, TRIG_IDX = 0.04, 50  # 25 Hz (75 downsampled pts = 3.0 s): 50 pre + 25 post
dev_id = sys.argv[1] if len(sys.argv) > 1 else None
inbox = queue.Queue()


def parse_payload(text: str):
    text = text.strip()
    if text.startswith("EMPTY"):
        return {"empty": True}
    head, g_str = re.split(r",G[:=]", text, maxsplit=1)
    meta = dict(re.split(r"[:=]", kv, maxsplit=1) for kv in head.split(","))
    return {
        "empty": False,
        "id": int(meta["ID"]),
        "peak": float(meta["PEAK"]),
        "crc_ok": meta["CRC"].upper() == "OK",
        "g": [float(v) for v in g_str.split(",") if v],
    }


client = (
    mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    if hasattr(mqtt, "CallbackAPIVersion")
    else mqtt.Client()
)


def on_connect(cli, *_):
    topic = f"SBC/{dev_id}/edr/data" if dev_id else "SBC/+/edr/data"
    cli.subscribe(topic)
    print(f"[MQTT] Connected to {BROKER}:{PORT} | Subscribed: {topic}")


def on_message(_cli, _ud, msg):
    global dev_id
    parts = msg.topic.split("/")
    if not dev_id and len(parts) >= 2:
        dev_id = parts[1]
        print(f"[MQTT] Auto-detected Device ID: {dev_id}")
    try:
        inbox.put(parse_payload(msg.payload.decode("utf-8", "ignore")))
    except Exception as exc:
        print(f"[WARN] Ignored malformed payload: {exc}")


client.on_connect, client.on_message = on_connect, on_message
client.connect(BROKER, PORT, keepalive=30)
client.loop_start()

fig, ax = plt.subplots(figsize=(9, 5))
plt.subplots_adjust(bottom=0.20)


def render(data=None):
    ax.clear()
    ax.set(xlim=(-2.0, 1.0), xlabel="Time relative to shock (s)", ylabel="Acceleration (g)")
    ax.grid(True, linestyle=":", alpha=0.6)
    if not data or data["empty"]:
        msg = "Flash Sector 0 is EMPTY" if data else "Waiting for crash (ARM & shake, or click READ)"
        ax.set_ylim(0, 5.0)
        ax.set_title(msg, fontweight="bold")
    else:
        g = data["g"]
        t = [(i - TRIG_IDX) * DT for i in range(len(g))]
        ok = data["crc_ok"]
        col = "#1b8a2f" if ok else "#cc1f1a"
        status = "FLASH CRC: OK" if ok else "TAMPER DETECTED (CRC FAIL)"
        ax.axvspan(-2.0, 0.0, color="#e8eef5", alpha=0.5, label="Pre-trigger (2.0 s)")
        ax.axvspan(0.0, 1.0, color="#fff3e0" if ok else "#fde8e8", alpha=0.5, label="Post-trigger (1.0 s)")
        ax.axvline(0.0, color="black", ls="--", lw=1.2, label="Impact (t = 0 s)")
        ax.axhline(2.5, color="#d97706", ls=":", lw=1.2, label="2.5 g Threshold")
        ax.plot(t, g, color=col, lw=2.0, label="Resultant G-force")
        ax.set_ylim(0, max(4.5, max(g, default=3.0) + 0.8))
        ax.set_title(
            f"Event #{data['id']}  |  Peak: {data['peak']:.2f} g  |  {status}",
            color="white", backgroundcolor=col, fontweight="bold", pad=10,
        )
        ax.legend(loc="upper left", fontsize=8)
    fig.canvas.draw_idle()


def send(cmd: str):
    if not dev_id:
        print("[WARN] Device ID unknown — pass it on the command line or wait for a board publish.")
        return
    topic = f"SBC/{dev_id}/edr/cmd"
    client.publish(topic, cmd)
    print(f"[CMD] '{cmd}' -> {topic}")


buttons = [
    (Button(plt.axes([0.16, 0.04, 0.18, 0.07]), "ARM",    color="#d1fae5", hovercolor="#a7f3d0"), "ARM"),
    (Button(plt.axes([0.41, 0.04, 0.18, 0.07]), "DISARM", color="#f3f4f6", hovercolor="#e5e7eb"), "DISARM"),
    (Button(plt.axes([0.66, 0.04, 0.18, 0.07]), "READ",   color="#dbeafe", hovercolor="#bfdbfe"), "READ"),
]
for btn, cmd in buttons:
    btn.on_clicked(lambda _e, c=cmd: send(c))


def poll():
    latest = None
    while not inbox.empty():
        latest = inbox.get_nowait()
    if latest is not None:
        render(latest)


timer = fig.canvas.new_timer(interval=150)
timer.add_callback(poll)
timer.start()

render()
try:
    plt.show()
finally:
    client.loop_stop()
    client.disconnect()