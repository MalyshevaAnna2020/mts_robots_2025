#!/usr/bin/env python3

import math
import os
import socket
import struct
import time

print("hellow, world!")

CMD_HOST  = str(os.getenv("CMD_HOST", "127.0.0.1"))
CMD_PORT  = int(os.getenv("CMD_PORT", "5555"))
TEL_HOST  = str(os.getenv("TEL_HOST", "0.0.0.0"))
TEL_PORT  = int(os.getenv("TEL_PORT", "5600"))
PROTO     = str(os.getenv("PROTO", "tcp"))

# === Параметры ===
SAFE_DIST = 0.5
CRASH_DIST = 0.3
RECOVER_TIME = 1               # время отката назад
TURN_TIME = 0.6                # базовое время для поворота
FORWARD_SPEED = 0.6            # м/с

# === Машина состояний ===
state = "FORWARD"
state_until = 0
last_pos = None
start_time = time.time()
last_move_time = start_time

sock_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# if PROTO == "udp":
#     sock_tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#     sock_tel.bind((TEL_HOST, TEL_PORT))
#     print("hello")
# else:
#     sock_tel = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#     sock_tel.bind((TEL_HOST, TEL_PORT))
#     sock_tel.listen(1)
#     print(f"[client] waiting for telemetry TCP on {TEL_HOST}:{TEL_PORT}...")
#     conn, _ = sock_tel.accept()
#     sock_tel = conn
#     print("[client] connected to udp_diff telemetry")