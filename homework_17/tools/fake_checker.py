#!/usr/bin/env python3
"""Мінімальний «псевдочекер» для локальної перевірки потоку MAVLink 2.

Не замінює справжній чекер курсу — це лише інструмент для швидкої самоперевірки
темпів, одиниць вимірювання і логіки повторів COMMAND_LONG.

  python3 fake_checker.py --ack-on 2   # відповісти ACK на 2-й COMMAND_LONG
  python3 fake_checker.py --ack-on 0   # ніколи не відповідати (перевірка 5 спроб)
"""
import argparse
import socket
import struct
import sys
import time

MAGIC_V2 = 0xFD
MSG_HEARTBEAT = 0
MSG_ATTITUDE = 30
MSG_GLOBAL_POSITION_INT = 33
MSG_COMMAND_LONG = 76
MSG_COMMAND_ACK = 77
CRC_EXTRA = {0: 50, 30: 39, 33: 104, 76: 152, 77: 143}
MAV_CMD_USER_1 = 31010
MAV_RESULT_ACCEPTED = 0


def crc_accumulate(byte, crc):
    tmp = byte ^ (crc & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    return ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF


def crc16(data, extra):
    crc = 0xFFFF
    for b in data:
        crc = crc_accumulate(b, crc)
    return crc_accumulate(extra, crc)


def parse_frame(buf):
    if len(buf) < 12 or buf[0] != MAGIC_V2:
        return None
    payload_len = buf[1]
    msgid = buf[7] | (buf[8] << 8) | (buf[9] << 16)
    payload = buf[10:10 + payload_len]
    crc_rx = struct.unpack_from("<H", buf, 10 + payload_len)[0]
    if msgid in CRC_EXTRA and crc16(buf[1:10 + payload_len], CRC_EXTRA[msgid]) != crc_rx:
        return {"msgid": msgid, "crc_ok": False}
    return {"msgid": msgid, "crc_ok": True, "sysid": buf[5], "compid": buf[6],
            "payload": payload, "seq": buf[4]}


def unpack(payload, fmt, size):
    return struct.unpack(fmt, bytes(payload).ljust(size, b"\x00")[:size])


def build_ack(command, seq):
    payload = struct.pack("<HB", command, MAV_RESULT_ACCEPTED)
    header = bytes([len(payload), 0, 0, seq, 255, 190,
                    MSG_COMMAND_ACK & 0xFF, (MSG_COMMAND_ACK >> 8) & 0xFF, 0])
    crc = crc16(header + payload, CRC_EXTRA[MSG_COMMAND_ACK])
    return bytes([MAGIC_V2]) + header + payload + struct.pack("<H", crc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=14550)
    ap.add_argument("--ack-on", type=int, default=2,
                    help="номер COMMAND_LONG, на який відповісти ACK (0 = ніколи)")
    ap.add_argument("--timeout", type=float, default=600.0,
                    help="запобіжник: максимальний час роботи, с")
    ap.add_argument("--idle", type=float, default=3.0,
                    help="скільки секунд тиші вважати кінцем польоту, с")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", args.port))
    sock.settimeout(1.0)

    counts = {}
    hb_times, telem_times = [], []
    boot_times = []
    cmd_count = 0
    ack_seq = 0
    last_pos = None
    unit_errors = []
    started = time.time()
    last_rx = None

    # Звіт друкуємо, коли дрон замовк (як робить справжній чекер), а не за
    # фіксованим лімітом: тривалість польоту залежить від --target і --speed.
    while time.time() - started < args.timeout:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            if last_rx and time.time() - last_rx > args.idle:
                break
            continue
        now = time.time()
        last_rx = now
        frame = parse_frame(data)
        if not frame or not frame.get("crc_ok"):
            print("!! невалідний кадр MAVLink 2")
            continue
        mid = frame["msgid"]
        counts[mid] = counts.get(mid, 0) + 1

        if mid == MSG_HEARTBEAT:
            hb_times.append(now)
        elif mid == MSG_GLOBAL_POSITION_INT:
            telem_times.append(now)
            t, lat, lon, alt, rel, vx, vy, vz, hdg = unpack(
                frame["payload"], "<IiiiihhhH", 28)
            boot_times.append(t)
            if last_pos:
                dt = (t - last_pos[0]) / 1000.0
                if dt > 0:
                    d_lat_m = (lat - last_pos[1]) / 1e7 * 111320.0
                    exp_m = last_pos[2] / 100.0 * dt
                    if abs(d_lat_m - exp_m) > max(0.5, abs(exp_m) * 0.15):
                        unit_errors.append(f"позиція/швидкість: {d_lat_m:.2f} vs {exp_m:.2f}")
            last_pos = (t, lat, vx)
            if len(telem_times) == 1:
                print(f"   перший GLOBAL_POSITION_INT: lat={lat/1e7:.6f} lon={lon/1e7:.6f} "
                      f"alt={alt/1000.0:.1f}м vx={vx/100.0:.1f} vy={vy/100.0:.1f} "
                      f"hdg={hdg/100.0:.1f}°")
        elif mid == MSG_COMMAND_LONG:
            fields = unpack(frame["payload"], "<fffffffHBBB", 33)
            command = fields[7]
            cmd_count += 1
            print(f"   COMMAND_LONG #{cmd_count}: command={command} "
                  f"lat={fields[4]:.6f} lon={fields[5]:.6f} alt={fields[6]:.1f} "
                  f"confirmation={fields[10]}")
            if command == MAV_CMD_USER_1 and args.ack_on and cmd_count == args.ack_on:
                sock.sendto(build_ack(command, ack_seq), addr)
                ack_seq += 1
                print("   -> надіслано COMMAND_ACK(ACCEPTED)")

    def rate(times):
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.0

    print("\n--- підсумок ---")
    print(f"HEARTBEAT: {counts.get(MSG_HEARTBEAT,0)} кадрів, {rate(hb_times):.2f} Гц")
    print(f"GLOBAL_POSITION_INT: {counts.get(MSG_GLOBAL_POSITION_INT,0)}, {rate(telem_times):.2f} Гц")
    print(f"ATTITUDE: {counts.get(MSG_ATTITUDE,0)}")
    print(f"COMMAND_LONG: {cmd_count}")
    print(f"time_boot_ms монотонний: {all(b <= a for b, a in zip(boot_times, boot_times[1:]))}")
    print(f"узгодженість позиція/швидкість: {'OK' if not unit_errors else unit_errors[:3]}")
    return 0 if not unit_errors else 1


if __name__ == "__main__":
    sys.exit(main())
