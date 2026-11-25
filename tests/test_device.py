"""SentUSB against the built-in emulator (no hardware)."""
import time

import pytest

from sentusb import NackError, NotSupportedError, SentFormat, SentUSB, SentUSBError, run_selftest
from sentusb import protocol as P


@pytest.fixture
def dev():
    d = SentUSB("emu://4ch")
    yield d
    d.close()


def test_detects_4_channel_board(dev):
    assert dev.channel_count == 4 and not dev.info.legacy
    assert dev.info.has_termination and dev.info.has_supply
    assert dev.refresh_info().modes == ["rx"] * 4
    assert dev.refresh_info().termination_on == [True] * 4          # AUTO in RX


def test_legacy_firmware_is_one_channel_with_v11_bytes():
    with SentUSB("emu://1ch") as d:
        emu = d._tr
        assert d.channel_count == 1 and d.info.legacy and d.version == "0102"
        assert "t600106" in emu.commands                             # info asked, NACKed
        assert "O" in emu.commands and "L" not in emu.commands
        with pytest.raises(NotSupportedError):
            d.set_termination(1, "off")
        with pytest.raises(NotSupportedError):
            d.supply()
        d.learn(1)
        assert "t600104" in emu.commands                             # v1.1 learn byte
        assert d.read(1, timeout=1.0) is not None


def test_listen_only_keeps_termination_off():
    with SentUSB("emu://4ch", listen_only=True) as d:
        assert "L" in d._tr.commands
        assert d.refresh_info().termination_on == [False] * 4
        d.set_termination(2, "on")
        assert d.refresh_info().termination_on == [False, True, False, False]


def test_learn_applies_format_and_restarts_rx(dev):
    r = dev.learn(2)
    assert r.nibbles == 6 and r.crc_mode is P.CrcMode.STATUS_AND_DATA and r.crc_seed == 5
    assert dev.channels[1].fmt == r.to_format() and dev.channels[1].mode == "rx"
    f = dev.read(2, timeout=1.0)
    assert f is not None and f.channel == 2 and len(f.nibbles) == 6


def test_learn_refused_while_another_channel_learns(dev):
    dev._command(P.control_frame(3, P.CMD_LEARN))      # CH4 has no sensor: keeps learning
    with pytest.raises(NackError, match="CH4 is learning"):
        dev.learn(1)
    with pytest.raises(TimeoutError, match="no SENT format"):
        dev.learn(4, timeout=0.3)


def test_capture_statistics_and_hints(dev, tmp_path):
    csv = tmp_path / "cap.csv"
    res = dev.capture(0.5, decoder={1: "mlx90377", 2: "raw"}, csv_path=str(csv))
    st = {c["channel"]: c for c in res.to_dict()["channels"]}
    assert st[1]["frames"] > 100 and st[1]["counter_jumps"] == 0
    assert st[1]["values"]["inverted_ok"]["false"] == 0
    assert st[1]["slow_messages"]["short:0x03"]["last_value"] == 0x42
    assert st[2]["frames"] == 0 and "CRC" in st[2]["hints"][0]        # default format, sensor uses seed 5
    assert st[4]["frames"] == 0 and "no SENT edges" in st[4]["hints"][0]
    lines = csv.read_text().splitlines()
    assert lines[0].startswith("time,channel,can_id,status,nibbles") and len(lines) > 200


def test_wrong_nibble_count_shows_sync_errors(dev):
    dev.configure(1, nibbles=4)
    st = dev.capture(0.4, [1]).to_dict()["channels"][0]
    assert st["frames"] == 0 and st["device_counters_delta"]["sync_errors"] > 0
    assert "learn" in st["hints"][0]


def test_tx_guard_and_pacing(dev):
    with pytest.raises(SentUSBError, match="receiving SENT frames"):
        dev.transmit(1, "123456", count=3)
    with pytest.raises(ValueError, match="shorter than a CH4 frame"):
        dev.transmit(4, "123456", period_ms=0.2)
    job = dev.transmit(4, ["000000", "FFFFFF"], count=20)
    assert job.wait(5) and job.sent == 20 and job.rejected == 0
    assert dev.channels[3].mode == "tx"
    dev.stop_tx(4)
    assert dev.channels[3].mode == "rx"


def test_tx_on_busy_line_is_rejected_by_firmware(dev):
    dev.configure(4, tick_us=90.0)
    dev.send(4, "FFFFFF")                                   # ~38 ms on the line
    with pytest.raises(NackError):
        dev._command(P.tx_frame(3, 0, (0,) * 6))
    dev.send(4, "000000")                                   # send() waits and retries


def test_link_resync_after_lost_answer(dev):
    dev._tr.drop_next_answers = 1
    with pytest.raises(SentUSBError, match="no answer"):
        dev.start_rx(1)
    dev.start_rx(1)                                         # link usable again
    assert dev.refresh_info().modes[0] == "rx"


def test_selftest_loopback_passes_on_wired_pairs():
    with SentUSB("emu://4ch?loop=12,34") as d:
        rep = run_selftest(d, quick=True, frames=10)
        assert rep.passed, rep.summary()
        names = [c.name for c in rep.checks]
        assert any("CH2->CH1" in n for n in names) and any("termination CH4" in n for n in names)


def test_selftest_detects_unwired_pair():
    with SentUSB("emu://4ch?loop=12") as d:                 # CH3-CH4 not wired
        rep = run_selftest(d, pairs=[(3, 4)], quick=True, frames=5)
        assert not rep.passed
        assert "received 0/5" in next(c.detail for c in rep.checks if c.name.startswith("loopback"))


def test_selftest_two_1_channel_boards_full():
    """Two v1.1 (V0102) boards with their SENT lines wired: every format, both directions."""
    with SentUSB("emu://1ch?wire=t2a") as a, SentUSB("emu://1ch?wire=t2a") as b:
        rep = run_selftest(a, peer=b, frames=8)
        assert rep.passed, rep.summary()
        names = [c.name for c in rep.checks]
        assert "A: board answers" in names and "B: board answers" in names
        loops = [n for n in names if n.startswith("loopback")]
        assert len(loops) == 12                                  # 6 formats x 2 directions
        assert any("A CH1->B CH1" in n for n in loops) and any("B CH1->A CH1" in n for n in loops)
        assert a.refresh_info().modes == ["rx"] and b.refresh_info().modes == ["rx"]


def test_selftest_two_boards_not_wired():
    with SentUSB("emu://1ch?wire=t2b") as a, SentUSB("emu://1ch?wire=t2c") as b:
        rep = run_selftest(a, peer=b, quick=True, frames=4)
        assert not rep.passed
        assert all("received 0/4" in c.detail for c in rep.checks if c.name.startswith("loopback"))


def test_selftest_two_boards_one_direction():
    """One wire (B TX -> A RX): direction='b-to-a' runs only that leg."""
    with SentUSB("emu://1ch?wire=t2e") as a, SentUSB("emu://1ch?wire=t2e") as b:
        rep = run_selftest(a, peer=b, quick=True, frames=4, direction="b-to-a")
        assert rep.passed, rep.summary()
        loops = [c.name for c in rep.checks if c.name.startswith("loopback")]
        assert len(loops) == 2 and all("B CH1->A CH1" in n for n in loops)
        with pytest.raises(ValueError):
            run_selftest(a, peer=b, direction="sideways")


def test_selftest_1_channel_and_4_channel_board():
    """Mixed pair: 1-channel board wired to CH3 of a 4-channel board."""
    with SentUSB("emu://1ch?wire=t2d") as a, SentUSB("emu://4ch?wire=t2d:3") as b:
        rep = run_selftest(a, peer=b, quick=True, frames=6, peer_channels=(1, 3))
        assert rep.passed, rep.summary()
        assert any(c.name == "B: termination CH4" for c in rep.checks)


def test_single_1_channel_board_needs_peer_for_loopback():
    with SentUSB("emu://1ch") as d:
        rep = run_selftest(d, quick=True)
        assert not rep.passed and "--peer" in rep.checks[-1].detail


def test_formats_on_connect_and_close_stops():
    d = SentUSB("emu://4ch", formats={2: SentFormat(3.0, 6, "status_and_data", 5)})
    assert d.read(2, timeout=1.0) is not None
    emu = d._tr
    d.close()
    assert emu.commands[-1] == "C"
