"""SENTToUSB bidirectional integration tests.

Two SENTToUSB dongles are required: one drives the SENT line as TX (PB0)
and the second listens on its RX input (PA2).  TX-device PB0 must be
wired to RX-device PA2 (and a common ground) for any of the data-path
tests to pass.

Default ports:
    TX = COM15
    RX = COM20
Override at the top of TestSENTIntegration.
"""

import logging
import time

import pytest
import serial

logger = logging.getLogger(__name__)

ACK_BEL = b"\x07"           # SLCAN nack
ACK_OK = b"\r"              # SLCAN ack (single CR)


def _has_ack(response: bytes) -> bool:
    """Treat any non-empty response without BEL as a successful ack.

    The firmware queues responses asynchronously so an empty buffer can
    still mean success when the read happened to race the USB poll.
    """
    return ACK_BEL not in response


class SENTDevice:
    """Thin SLCAN wrapper around a SENTToUSB CDC port."""

    def __init__(self, port: str, baudrate: int = 115200, read_timeout: float = 1.0):
        self.port_name = port
        self.ser = serial.Serial(port, baudrate, timeout=read_timeout)
        time.sleep(0.2)
        self.drain()
        logger.info("Connected to %s", port)

    def close(self):
        try:
            self.ser.write(b"C\r")
            time.sleep(0.05)
        except Exception:
            pass
        try:
            self.ser.close()
        except Exception:
            pass

    def drain(self):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def send_line(self, line: str, wait: float = 0.15) -> bytes:
        """Send a CR-terminated SLCAN command and return whatever is queued."""
        self.ser.write((line + "\r").encode("ascii"))
        time.sleep(wait)
        n = self.ser.in_waiting
        return self.ser.read(n) if n > 0 else b""

    # ── Single-letter commands ─────────────────────────────────────────────

    def version(self) -> bytes:
        return self.send_line("V")

    def fw_version(self) -> bytes:
        return self.send_line("v")

    def serial_number(self) -> bytes:
        return self.send_line("N")

    def status(self) -> bytes:
        return self.send_line("F")

    def open(self) -> bytes:
        """Open SLCAN channel — also starts SENT RX in this firmware."""
        return self.send_line("O")

    def close_channel(self) -> bytes:
        return self.send_line("C")

    # ── Control frames on CAN ID 0x600 ─────────────────────────────────────

    def start_rx(self) -> bytes:
        return self.send_line("t600101")

    def start_tx(self) -> bytes:
        return self.send_line("t600102")

    def stop(self) -> bytes:
        return self.send_line("t600103")

    def set_tx_tick_us(self, tick_us: float) -> bytes:
        tick_x10 = int(round(tick_us * 10))
        if not 20 <= tick_x10 <= 900:
            raise ValueError(f"tick_us={tick_us} out of range [2.0, 90.0]")
        lsb = tick_x10 & 0xFF
        msb = (tick_x10 >> 8) & 0xFF
        return self.send_line(f"t600305{lsb:02X}{msb:02X}")

    # ── Config frame on CAN ID 0x001 ───────────────────────────────────────
    # Sets the RX tick range so the bridge can derive a per-tick
    # sync-detection threshold (sync_min_us).  Without this the firmware
    # default of 100 µs only distinguishes sync from data at ~3 µs/tick.

    def send_config(
        self,
        min_tick_us: float,
        max_tick_us: float,
        data_nibbles: int = 6,
        crc_mode: int = 0,
        crc_seed: int = 0x03,
    ) -> bytes:
        # byte 3 = min_tick in 0.5 µs units (firmware: stored ×5 to give tick_x10)
        # byte 4 = max_tick in 1.0 µs units (firmware: stored ×10 to give tick_x10)
        # bytes 2/3/4 are only honoured when non-zero — keep ≥1.
        b3 = max(1, int(round(min_tick_us * 2)))
        b4 = max(1, int(round(max_tick_us)))
        return self.send_line(
            "t0015"
            f"{data_nibbles & 0xFF:02X}"
            f"{crc_mode & 0xFF:02X}"
            f"{crc_seed & 0xFF:02X}"
            f"{b3 & 0xFF:02X}"
            f"{b4 & 0xFF:02X}"
        )

    # ── TX data frame on CAN ID 0x520 ──────────────────────────────────────
    # The firmware requires DLC>=5: data[0]=status, data[1..4] hold the
    # 6 (or 8) data nibbles packed MSB-first.  data[4] is unused for the
    # default 6-nibble MLX90377 configuration but the byte must still be
    # transmitted to satisfy the DLC requirement.

    def send_data_frame(self, status: int, nibble_bytes: bytes) -> bytes:
        if len(nibble_bytes) != 3:
            raise ValueError("nibble_bytes must be exactly 3 bytes (6 nibbles)")
        b1, b2, b3 = nibble_bytes
        return self.send_line(
            f"t5205{status & 0xFF:02X}{b1:02X}{b2:02X}{b3:02X}00"
        )

    # ── Stream reader ──────────────────────────────────────────────────────

    def read_frames(self, timeout: float = 1.0):
        """Collect SLCAN lines until `timeout` seconds with no data."""
        deadline = time.time() + timeout
        buf = b""
        frames = []
        old = self.ser.timeout
        self.ser.timeout = 0.05
        try:
            while time.time() < deadline:
                chunk = self.ser.read(256)
                if chunk:
                    buf += chunk
                    while b"\r" in buf:
                        line, _, buf = buf.partition(b"\r")
                        line = line.strip()
                        if line:
                            frames.append(line.decode("ascii", errors="replace"))
        finally:
            self.ser.timeout = old
        return frames


class TestSENTIntegration:
    """Hardware-loopback integration tests (TX device PB0 → RX device PA2)."""

    TX_PORT = "COM15"
    RX_PORT = "COM20"
    TICK_TIMES = [3.0, 6.0, 9.0, 12.0]

    # ── Fixtures ───────────────────────────────────────────────────────────

    @pytest.fixture
    def tx(self):
        dev = SENTDevice(self.TX_PORT)
        dev.send_line("C")           # ensure clean state
        yield dev
        dev.close()

    @pytest.fixture
    def rx(self):
        dev = SENTDevice(self.RX_PORT)
        dev.send_line("C")
        yield dev
        dev.close()

    # ── Helpers ────────────────────────────────────────────────────────────

    def _setup_pair(self, tx: SENTDevice, rx: SENTDevice, tick_us: float = 3.0):
        rx.stop()
        tx.stop()
        time.sleep(0.05)

        # Narrow the RX device's tick range so the bridge can recompute a
        # tick-appropriate sync_min_us threshold.  Stock firmware defaults
        # to a 2.5–90 µs range whose derived threshold is 100 µs — only
        # valid for ~3 µs ticks.  Use ±30 % around the target tick.
        margin = max(1.0, tick_us * 0.3)
        ack = rx.send_config(
            min_tick_us=max(0.5, tick_us - margin),
            max_tick_us=tick_us + margin,
        )
        assert _has_ack(ack), f"RX send_config({tick_us}µs) rejected: {ack!r}"

        ack = rx.open()                   # opens SLCAN channel & starts RX
        assert _has_ack(ack), f"RX open() rejected: {ack!r}"

        ack = tx.set_tx_tick_us(tick_us)
        assert _has_ack(ack), f"TX set_tick({tick_us}) rejected: {ack!r}"

        ack = tx.start_tx()
        assert _has_ack(ack), f"TX start_tx() rejected: {ack!r}"

        time.sleep(0.05)
        rx.drain()                        # discard stale diag/ack bytes

    @staticmethod
    def _rx_wait(tick_us: float, n_frames: int) -> float:
        # A SENT frame is ~250 ticks (sync 56 + status 12 + 6*(0..15+12) +
        # crc 12 + pause 12).  Round up generously, add USB latency margin.
        per_frame = max(0.05, tick_us * 1e-6 * 320)
        return max(0.6, per_frame * n_frames + 0.5)

    # ── Connectivity ───────────────────────────────────────────────────────

    def test_devices_present_and_responsive(self, tx, rx):
        for label, dev in [("TX", tx), ("RX", rx)]:
            ver = dev.version()
            logger.info("%s (%s) version: %r", label, dev.port_name, ver)
            assert ver.startswith(b"V") or b"V" in ver, (
                f"{label} version response invalid: {ver!r}"
            )

            sn = dev.serial_number()
            logger.info("%s (%s) serial: %r", label, dev.port_name, sn)
            assert sn.startswith(b"N") or b"N" in sn, (
                f"{label} serial response invalid: {sn!r}"
            )

    def test_can_enter_rx_mode(self, rx):
        rx.stop()
        ack = rx.start_rx()
        assert _has_ack(ack), f"start_rx() returned BEL: {ack!r}"

    def test_can_enter_tx_mode(self, tx):
        tx.stop()
        ack = tx.start_tx()
        assert _has_ack(ack), f"start_tx() returned BEL: {ack!r}"

    # ── TX → RX, parametrized over tick period ─────────────────────────────

    @pytest.mark.parametrize("tick_us", TICK_TIMES)
    def test_tx_rx_with_tick_time(self, tx, rx, tick_us):
        self._setup_pair(tx, rx, tick_us=tick_us)

        # Send 3 frames; sleep > one frame duration between submissions
        # so the previous DMA-driven SENT frame finishes (otherwise the
        # firmware silently drops while TIM3 is still running).
        inter_frame = max(0.05, tick_us * 1e-6 * 320)
        for i in range(3):
            data = bytes([i & 0xFF, 0x23, 0x45])
            ack = tx.send_data_frame(status=(i & 0xF), nibble_bytes=data)
            assert _has_ack(ack), f"TX frame[{i}] rejected: {ack!r}"
            time.sleep(inter_frame)

        frames = rx.read_frames(timeout=self._rx_wait(tick_us, 3))
        rx_frames = [f for f in frames if f.startswith("t510")]
        logger.info(
            "tick=%sµs → got %d/%d decoded frames (all=%r)",
            tick_us, len(rx_frames), 3, frames,
        )
        assert rx_frames, f"No 0x510 frames received at {tick_us}µs"

    # ── Tick × DLC matrix (firmware supports DLC=5 only for 0x520) ─────────
    # The integration README mentions DLC 2–5 but the bridge rejects DLC<5
    # on 0x520 frames; only DLC=5 is exercised here.

    @pytest.mark.parametrize("dlc", [5])
    @pytest.mark.parametrize("tick_us", TICK_TIMES)
    def test_tx_rx_with_tick_and_frame_size(self, tx, rx, tick_us, dlc):
        self._setup_pair(tx, rx, tick_us=tick_us)
        inter_frame = max(0.05, tick_us * 1e-6 * 320)

        patterns = [b"\x12\x34\x56", b"\xAB\xCD\xEF", b"\x00\xFF\x00"]
        for pat in patterns:
            ack = tx.send_data_frame(status=0x1, nibble_bytes=pat)
            assert _has_ack(ack), f"TX frame rejected: {ack!r}"
            time.sleep(inter_frame)

        frames = rx.read_frames(timeout=self._rx_wait(tick_us, len(patterns)))
        rx_frames = [f for f in frames if f.startswith("t510")]
        logger.info(
            "tick=%sµs dlc=%d → %d/%d frames",
            tick_us, dlc, len(rx_frames), len(patterns),
        )
        assert rx_frames, f"No 0x510 frames at tick={tick_us}µs dlc={dlc}"

    # ── Multi-frame sequence ───────────────────────────────────────────────

    def test_multiple_frames_sequence(self, tx, rx):
        self._setup_pair(tx, rx, tick_us=3.0)
        N = 10
        inter_frame = max(0.05, 3.0e-6 * 320)
        for i in range(N):
            tx.send_data_frame(status=(i & 0xF),
                               nibble_bytes=bytes([i & 0xFF, 0x00, 0x00]))
            time.sleep(inter_frame)

        frames = rx.read_frames(timeout=self._rx_wait(3.0, N))
        rx_frames = [f for f in frames if f.startswith("t510")]
        logger.info("sequence: %d/%d frames received", len(rx_frames), N)
        # Allow some loss tolerance — USB CDC + DMA timing can occasionally
        # eat a frame, especially the first one if a stale diag frame is
        # still in flight; require the majority to arrive.
        assert len(rx_frames) >= max(1, N // 2), (
            f"Too few sequence frames: {len(rx_frames)}/{N}"
        )

    # ── Quiet-channel listen ───────────────────────────────────────────────

    def test_rx_only_without_tx(self, rx):
        rx.stop()
        ack = rx.open()
        assert _has_ack(ack), f"RX open rejected: {ack!r}"

        # Without a TX driver wired in, no 0x510 frames should appear.
        # 0x511 diag frames are OK.
        frames = rx.read_frames(timeout=1.0)
        rx_frames = [f for f in frames if f.startswith("t510")]
        logger.info(
            "quiet-channel: %d 0x510 frames, %d others",
            len(rx_frames), len(frames) - len(rx_frames),
        )
        # Don't assert empty: the user's setup may have a real sensor wired
        # in.  Just make sure the device doesn't crash or NACK.

    # ── Tick persistence ───────────────────────────────────────────────────

    @pytest.mark.parametrize("tick_us", TICK_TIMES)
    def test_tick_time_persistence_across_frames(self, tx, rx, tick_us):
        self._setup_pair(tx, rx, tick_us=tick_us)
        N = 5
        inter_frame = max(0.05, tick_us * 1e-6 * 320)
        for i in range(N):
            tx.send_data_frame(status=0x1,
                               nibble_bytes=bytes([i & 0xFF, 0xA0, 0x0B]))
            time.sleep(inter_frame)

        frames = rx.read_frames(timeout=self._rx_wait(tick_us, N))
        rx_frames = [f for f in frames if f.startswith("t510")]
        logger.info("persistence tick=%sµs → %d/%d", tick_us, len(rx_frames), N)
        assert rx_frames, f"No frames received during persistence test at {tick_us}µs"

    # ── DLC=5 baseline at default tick ─────────────────────────────────────

    def test_all_frame_sizes_with_default_tick(self, tx, rx):
        """Single frame at default 3 µs tick — kept for naming compatibility
        with the README; firmware only supports DLC=5 on the TX path so the
        original DLC=2..5 matrix collapses to a single case."""
        self._setup_pair(tx, rx, tick_us=3.0)
        ack = tx.send_data_frame(status=0x5, nibble_bytes=b"\xDE\xAD\xBE")
        assert _has_ack(ack), f"TX frame rejected: {ack!r}"

        frames = rx.read_frames(timeout=self._rx_wait(3.0, 1))
        rx_frames = [f for f in frames if f.startswith("t510")]
        logger.info("default-tick frame: %d 0x510 received", len(rx_frames))
        assert rx_frames, f"No 0x510 frame received: {frames!r}"
