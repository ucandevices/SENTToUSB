"""sentusb - Python library, CLI and MCP server for the ucandevices SENT-to-USB converters.

    from sentusb import SentUSB
    with SentUSB() as dev:              # or SentUSB("COM7"), SentUSB("emu://4ch")
        dev.learn(1)
        print(dev.capture(1.0, decoder="mlx90377").summary())

Channels are numbered 1..N as on the board (SENT1..SENT4).
"""
from .protocol import (ChannelCounters, CrcMode, DeviceInfo, LearnResult, RxFrame, SentFormat,
                       SlowMessage, SupplyStatus)
from .device import NackError, NotSupportedError, SentUSB, SentUSBError, TxJob, find_ports, default_port
from .decoders import DECODERS, SENSOR_PRESETS, get_decoder
from .capture import CaptureResult
from .selftest import run_selftest, SelfTestReport

__version__ = "0.1.0"

__all__ = ["SentUSB", "SentFormat", "CrcMode", "RxFrame", "SlowMessage", "LearnResult", "DeviceInfo",
           "SupplyStatus", "ChannelCounters", "TxJob", "CaptureResult", "SelfTestReport",
           "SentUSBError", "NackError", "NotSupportedError", "find_ports", "default_port",
           "DECODERS", "SENSOR_PRESETS", "get_decoder", "run_selftest", "__version__"]
