"""Wire format, checked against the firmware sources and the v1.1 viewer's bytes."""
import pytest

from sentusb import protocol as P
from sentusb.decoders import DECODERS
from sentusb.protocol import CrcMode, SentFormat


def test_config_frame_matches_viewer_v11():
    # sent_viewer v1.1 sent exactly this for MLX90377 defaults on 0x510
    assert P.config_frame(0, SentFormat(), 0x510) == b"t001706000305050510\r"
    f = SentFormat(3.0, 6, CrcMode.STATUS_AND_DATA, 5)
    assert P.config_frame(2, f, 0x590) == b"t003706010505050590\r"
    assert P.config_frame(0, SentFormat(3.0, 6, "data", 5, legacy_crc=True), 0x510)[9:11] == b"85"


def test_control_and_tick_frames():
    assert P.control_frame(0, P.CMD_RX) == b"t600101\r"          # v1.1 'start RX'
    assert P.control_frame(2, P.CMD_LEARN) == b"t600124\r"
    assert P.tx_tick_frame(1, 3.0) == b"t6003151E00\r"            # 30 = 0x001E, LE
    assert P.tx_tick_frame(0, 500) == b"t60030584" + b"03\r"      # clamped to 90.0 µs


def test_tx_frame_is_right_aligned_msb_first():
    # firmware sent_unpack_nibbles(MSB_FIRST): low n nibbles of the BE word
    assert P.tx_frame(0, 0, (1, 2, 3, 4, 5, 6)) == b"t52050000123456\r"
    assert P.tx_frame(3, 0xA, (0xF,) * 8) == b"t5E050AFFFFFFFF\r"
    assert P.tx_frame(1, 0, (1, 2, 3), pause_ticks=300) == b"t5607000000012" + b"32C01\r"


def test_parse_nibbles():
    assert P.parse_nibbles("0x1a3") == (1, 10, 3)
    assert P.parse_nibbles([1, 2], 2) == (1, 2)
    with pytest.raises(ValueError):
        P.parse_nibbles("12", 6)
    with pytest.raises(ValueError):
        P.parse_nibbles("12G")


def test_decode_rx_frame_layout():
    # pack_rx_can_frame: 5 nibbles -> 3 bytes, info at index 3, slow message after
    data = bytes([0x12, 0x34, 0x50, 0x71, 0x03, 0x42, 0x00])
    f = P.decode_rx_frame(1, 0x510, data, 5, timestamp=1.0)
    assert f.nibbles == (1, 2, 3, 4, 5) and f.status == 7
    assert f.slow.kind == "short" and f.slow.id == 3 and f.slow.value == 0x42
    # 8 nibbles -> info at index 4
    f = P.decode_rx_frame(2, 0x550, bytes([0xAB, 0xCD, 0xEF, 0x01, 0x20]), 8, 1.0)
    assert f.hex == "ABCDEF01" and f.status == 2 and f.slow is None


def test_info_and_supply_frames():
    info = P.DeviceInfo.from_frame(bytes([4, 0x02, 0x00, 0b11_10_01_00, 3, 0b0011_0101, 7, 0]), "0200")
    assert info.channels == 4 and info.modes == ["off", "rx", "tx", "learn"]
    assert info.learning_channel == 4
    assert info.termination_on == [True, False, True, False]
    assert info.termination_auto == [True, True, False, False]
    s = P.SupplyStatus.from_frame(bytes([0x8A, 0x13, 0x03, 0xFF, 0xFF, 1]))
    assert s.vs_v == 5.002 and s.current_ma is None and s.fault and s.source == "ext"


def test_format_validation():
    with pytest.raises(ValueError):
        SentFormat(nibbles=9)
    with pytest.raises(ValueError):
        SentFormat(crc_seed=0)
    with pytest.raises(ValueError):
        SentFormat(crc_mode="weird")
    assert SentFormat.from_dict(SentFormat(5, 4).to_dict()) == SentFormat(5, 4)


def test_decoders_follow_j2716_appendix_h():
    f = P.RxFrame(1, 0x510, 0.0, 0, (0xA, 0xB, 0xC, 0x3, 0x2, 0x1))
    h1 = DECODERS["h1_dual12"].decode(f)
    assert h1["signal1"] == 0xABC and h1["signal2"] == 0x123          # channel 2 sent LSN first
    f = P.RxFrame(1, 0x510, 0.0, 1, (0x8, 0x0, 0x0, 0x2, 0xA, 0x7))
    h4 = DECODERS["mlx90377"].decode(f)
    assert h4["signal1"] == 0x800 and h4["counter"] == 0x2A and h4["inverted_ok"] and h4["error1"]
    assert h4["angle_deg"] == 180.0
    assert "decode_error" in DECODERS["h4_secure"].decode(P.RxFrame(1, 0, 0, 0, (1, 2, 3)))
