"""MCP server: in-memory client for the tool logic, plus one real stdio session."""
import json
import os
import sys
from pathlib import Path

import anyio
import pytest

pytest.importorskip("mcp")
from mcp.client.client import Client, StdioServerParameters  # noqa: E402

from sentusb import mcp_server  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def _data(result):
    assert not result.is_error, result.content[0].text
    return result.structured_content or json.loads(result.content[0].text)


async def _session():
    async with Client(mcp_server.mcp) as c:
        tools = {t.name for t in (await c.list_tools()).tools}
        assert {"connect", "capture", "learn_format", "start_transmit", "run_self_test"} <= tools

        r = await c.call_tool("capture", {})
        assert r.is_error and "not connected" in r.content[0].text

        d = _data(await c.call_tool("connect", {"port": "emu://4ch"}))
        assert d["channels"] == 4 and d["listen_only"] is True
        assert d["termination_on"] == [False] * 4

        d = _data(await c.call_tool("set_listen_only", {"enabled": False}))
        d = _data(await c.call_tool("learn_format", {"channel": 1}))
        assert d["format"]["nibbles"] == 6

        d = _data(await c.call_tool("capture", {"channels": [1, 4], "duration_s": 0.3, "decoder": "mlx90377"}))
        ch1, ch4 = d["channels"]
        assert ch1["frames"] > 50 and "angle_deg" in ch1["values"]
        assert ch4["frames"] == 0 and ch4["hints"]

        d = _data(await c.call_tool("read_frames", {"channel": 1, "count": 3, "decoder": "h4_secure"}))
        assert len(d["frames"]) == 3 and "counter" in d["frames"][0]["values"]

        r = await c.call_tool("start_transmit", {"channel": 1, "values": ["123456"], "count": 2})
        assert r.is_error and "receiving SENT frames" in r.content[0].text

        d = _data(await c.call_tool("start_transmit", {"channel": 4, "values": ["123456", "ABCDEF"],
                                                       "count": 5}))
        assert d["channel"] == 4
        await anyio.sleep(0.2)
        d = _data(await c.call_tool("stop_transmit", {"channel": 4}))
        assert d["sent"] == 5

        r = await c.call_tool("set_termination", {"channel": 9, "mode": "on"})
        assert r.is_error                                            # schema: channel 1..4
        d = _data(await c.call_tool("set_termination", {"channel": 3, "mode": "off"}))
        assert d["termination_on"] is False

        d = _data(await c.call_tool("supply_status", {}))
        assert 4.9 < d["vs_v"] < 5.1
        d = _data(await c.call_tool("run_self_test", {"loopback": False}))
        assert d["passed"], d["failed"]
        d = _data(await c.call_tool("device_status", {}))
        assert len(d["channels_state"]) == 4
        _data(await c.call_tool("disconnect", {}))


def test_mcp_tools_in_memory():
    anyio.run(_session)


def test_mcp_over_stdio():
    async def go():
        env = dict(os.environ, SENTUSB_PORT="emu://4ch?loop=12,34", PYTHONPATH=str(ROOT))
        params = StdioServerParameters(command=sys.executable, args=["-m", "sentusb.mcp_server"],
                                       env=env, cwd=str(ROOT))
        async with Client(params) as c:
            d = _data(await c.call_tool("connect", {"listen_only": False}))
            assert d["port"].startswith("emu://")
            d = _data(await c.call_tool("run_self_test", {"quick": True, "frames": 5}))
            assert d["passed"], d["failed"]
            _data(await c.call_tool("disconnect", {}))
    anyio.run(go)
