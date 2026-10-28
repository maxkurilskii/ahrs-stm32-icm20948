#!/usr/bin/env python3

import argparse
import random
import struct
import time
from dataclasses import dataclass
from typing import Optional

import serial


SOF = 0x23  # '#'

CMD_START = 0x10
CMD_STOP = 0x11
CMD_UPD_TIME = 0x15

PAYLOAD_LEN = 8

# CRC-16/CCITT-FALSE:
# poly=0x1021, init=0xFFFF, refin=False, refout=False, xorout=0
CRC_INIT = 0xFFFF
CRC_POLY = 0x1021

# Set to False if your firmware does not echo invalid frames.
ECHO_INVALID_FRAMES = False


@dataclass
class TestResult:
    sent: int = 0
    passed: int = 0
    failed: int = 0
    timeouts: int = 0
    mismatches: int = 0


def crc16_ccitt_false(data: bytes) -> int:
    crc = CRC_INIT

    for byte in data:
        crc ^= byte << 8

        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ CRC_POLY) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF

    return crc


def build_frame(cmd: int, payload: bytes) -> bytes:
    if len(payload) > 255:
        raise ValueError("Payload is too large")

    # SOF + CMD + LEN + PAYLOAD
    frame_without_crc = bytes([
        SOF,
        cmd,
        len(payload),
    ]) + payload

    crc = crc16_ccitt_false(frame_without_crc)

    # IMPORTANT:
    # This example uses CRC high byte first, then low byte.
    # It must match the STM32 implementation.
    crc_bytes = struct.pack(">H", crc)

    return frame_without_crc + crc_bytes


def make_time_payload() -> bytes:
    # POSIX timestamp in microseconds, uint64 little-endian.
    timestamp_us = time.time_ns() // 1000
    return struct.pack("<Q", timestamp_us)


def make_test_frame(cmd: int) -> bytes:
    return build_frame(cmd, make_time_payload())


# =========================
# Frame receiver
# =========================

class FrameReceiver:
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buffer = bytearray()

    def read_frame(self, timeout_s: float) -> Optional[bytes]:
        deadline = time.monotonic() + timeout_s

        while time.monotonic() < deadline:
            waiting = self.ser.in_waiting

            if waiting:
                self.buffer.extend(self.ser.read(waiting))
            else:
                # read() can wait up to the configured serial timeout
                chunk = self.ser.read(1)
                if chunk:
                    self.buffer.extend(chunk)

            # Find frame start.
            while self.buffer and self.buffer[0] != SOF:
                del self.buffer[0]

            if len(self.buffer) < 3:
                continue

            payload_len = self.buffer[2]
            frame_len = 3 + payload_len + 2

            if len(self.buffer) < frame_len:
                continue
            
            candidate = bytes(self.buffer[:frame_len])
            del self.buffer[:frame_len]

            received_crc = struct.unpack(">H", candidate[-2:])[0]
            calculated_crc = crc16_ccitt_false(candidate[:-2])

            if received_crc != calculated_crc:
                print(
                    "RX frame with invalid CRC:",
                    f"received=0x{received_crc:04X}",
                    f"calculated=0x{calculated_crc:04X}",
                )
                # continue

            return candidate

        return None


# =========================
# Test cases
# =========================

COMMANDS = [
    CMD_START,
    CMD_STOP,
    CMD_UPD_TIME,
]


def print_frame(prefix: str, frame: bytes):
    print(f"{prefix} ({len(frame)} bytes): {frame.hex(' ')}")


def test_sequential(
    ser: serial.Serial,
    count: int,
    interval_s: float,
    timeout_s: float,
) -> TestResult:
    result = TestResult()
    receiver = FrameReceiver(ser)

    for i in range(count):
        cmd = COMMANDS[i % len(COMMANDS)]
        tx_frame = make_test_frame(cmd)

        ser.write(tx_frame)
        ser.flush()

        result.sent += 1

        rx_frame = receiver.read_frame(timeout_s)

        if rx_frame is None:
            result.failed += 1
            result.timeouts += 1

            print(f"[FAIL] #{i}: response timeout")
            print_frame("TX", tx_frame)

        elif rx_frame != tx_frame:
            result.failed += 1
            result.mismatches += 1

            print(f"[FAIL] #{i}: response mismatch")
            print_frame("TX", tx_frame)
            print_frame("RX", rx_frame)

        else:
            result.passed += 1
            print(f"[PASS] #{i}: CMD=0x{cmd:02X}")

        if interval_s > 0:
            time.sleep(interval_s)

    return result


def test_burst(
    ser: serial.Serial,
    count: int,
    min_interval_s: float,
    max_interval_s: float,
    timeout_s: float,
) -> TestResult:
    result = TestResult()
    receiver = FrameReceiver(ser)

    expected_frames = []
    random_sleeps = []
    # Send frames without waiting for individual responses.
    for i in range(count):
        cmd = COMMANDS[i % len(COMMANDS)]
        tx_frame = make_test_frame(cmd)

        ser.write(tx_frame)
        # ser.flush()
        result.sent += 1
        expected_frames.append(tx_frame)

        if max_interval_s > 0:
            random_sleep = random.uniform(min_interval_s, max_interval_s)
            # print("Random sleep ms: ", random_sleep_time * 1000)
            random_sleeps.append(random_sleep)
            time.sleep(
                random_sleep
            )

    ser.flush()
    # Скорость отправки (скорость производителя) без учета пауз (максимальная):
    # Кол-во отправлен байт / общее время отправки (время отправки одного байта*общ кол-во байт + общее время пауз)
    print(f"V tx = {len(expected_frames) * 13 / ((1 / 11520) * 13 * len(expected_frames)):.3f}")
    # Collect responses and compare them in order.
    for i, expected in enumerate(expected_frames):
        # print_frame("TX", expected)
        received = receiver.read_frame(timeout_s)

        if received is None:
            result.failed += 1
            result.timeouts += 1

            print(f"[FAIL] Burst frame #{i}: timeout")
            continue
        
        # print_frame("RX", received)
        if received != expected:
            result.failed += 1
            result.mismatches += 1

            print(f"[FAIL] Burst frame #{i}: mismatch")
            print_frame("TX", expected)
            print_frame("RX", received)
        else:
            result.passed += 1

    return result


def print_summary(result: TestResult):
    print("\n========== TEST SUMMARY ==========")
    print(f"Sent:       {result.sent}")
    print(f"Passed:     {result.passed}")
    print(f"Failed:     {result.failed}")
    print(f"Timeouts:   {result.timeouts}")
    print(f"Mismatches: {result.mismatches}")

    if result.failed == 0 and result.sent > 0:
        print("RESULT: PASS")
    else:
        print("RESULT: FAIL")


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--count", type=int, default=100)
    parser.add_argument(
        "--mode",
        choices=["sequential", "burst"],
        default="sequential",
    )

    parser.add_argument("--interval-ms", type=float, default=10.0)
    parser.add_argument("--min-interval-ms", type=float, default=0.1)
    parser.add_argument("--max-interval-ms", type=float, default=1.0)
    parser.add_argument("--timeout", type=float, default=1.0)
    parser.add_argument("--seed", type=int, default=12345)

    args = parser.parse_args()

    random.seed(args.seed)

    with serial.Serial(
        port=args.port,
        baudrate=args.baud,
        timeout=0.005,
        write_timeout=1.0,
    ) as ser:

        # Discard stale input before starting the test.
        ser.reset_input_buffer()

        # Give the STM32 time to initialize.
        time.sleep(0.2)

        if args.mode == "sequential":
            result = test_sequential(
                ser=ser,
                count=args.count,
                interval_s=args.interval_ms / 1000.0,
                timeout_s=args.timeout,
            )
        else:
            result = test_burst(
                ser=ser,
                count=args.count,
                min_interval_s=args.min_interval_ms / 1000.0,
                max_interval_s=args.max_interval_ms / 1000.0,
                timeout_s=args.timeout,
            )

        print_summary(result)


if __name__ == "__main__":
    main()
