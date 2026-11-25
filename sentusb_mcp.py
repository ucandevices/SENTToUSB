"""Start the sentusb MCP server without installing the package.

    claude mcp add sentusb -- python C:\\Users\\LJ\\Desktop\\ucan\\sent\\sent_tools\\sentusb_mcp.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from sentusb.mcp_server import main  # noqa: E402

main()
