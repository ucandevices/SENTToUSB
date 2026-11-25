import json

from sentusb.cli import main


def run(capsys, *argv):
    rc = main(list(argv))
    return rc, capsys.readouterr()


def test_info_json(capsys):
    rc, out = run(capsys, "--port", "emu://4ch", "--json", "info")
    d = json.loads(out.out)
    assert rc == 0 and d["channels"] == 4 and d["supply"]["source"] == "usb"


def test_capture_with_learn_and_decoder(capsys):
    rc, out = run(capsys, "--port", "emu://4ch", "--json", "capture", "-c", "2", "-s", "0.3",
                  "--learn", "--decoder", "h1_dual12")
    d = json.loads(out.out)["channels"][0]
    assert rc == 0 and d["frames"] > 50 and d["values"]["signal1"]["min"] >= 1500


def test_capture_preset_human(capsys):
    rc, out = run(capsys, "--port", "emu://4ch", "capture", "-c", "1", "-s", "0.3", "--preset", "MLX90377")
    assert rc == 0 and "CH1 rx" in out.out and "angle_deg" in out.out


def test_learn_prints_reusable_flags(capsys):
    rc, out = run(capsys, "--port", "emu://4ch", "learn", "2")
    assert rc == 0 and "--crc status_and_data --seed 0x5" in out.out


def test_tx_refused_then_forced(capsys):
    rc, out = run(capsys, "--port", "emu://4ch", "tx", "1", "123456")
    assert rc == 1 and "receiving SENT frames" in out.err
    rc, out = run(capsys, "--port", "emu://4ch", "--json", "tx", "4", "123456", "654321", "--count", "10")
    d = json.loads(out.out)
    assert rc == 0 and d["sent"] == 10 and d["rejected"] == 0


def test_selftest_exit_codes(capsys):
    rc, out = run(capsys, "--port", "emu://4ch?loop=12,34", "selftest", "--quick", "--frames", "5")
    assert rc == 0 and "ALL PASSED" in out.out
    rc, out = run(capsys, "--port", "emu://4ch", "selftest", "--quick", "--frames", "3")
    assert rc == 2 and "FAIL" in out.out


def test_selftest_peer(capsys):
    rc, out = run(capsys, "--port", "emu://1ch?wire=cli", "--json", "selftest", "--quick", "--frames", "4",
                  "--peer", "emu://1ch?wire=cli")
    d = json.loads(out.out)
    assert rc == 0 and d["passed"] and any(c["name"].startswith("loopback B CH1->A CH1") for c in d["checks"])


def test_bad_channel_on_1_channel_board(capsys):
    rc, out = run(capsys, "--port", "emu://1ch", "capture", "-c", "3")
    assert rc == 1 and "channel must be 1..1" in out.err
