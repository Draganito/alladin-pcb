#!/usr/bin/env python3
"""Minimal MCP streamable-HTTP client for the alladin-pcb GUI server.

Usage: mcp_call.py TOOL_NAME [JSON_ARGS]
Prints the tool's JSON reply on stdout.
"""
import json
import sys
import urllib.request

URL = "http://127.0.0.1:8642/mcp"


def post(payload, session=None):
    headers = {
        "Content-Type": "application/json",
        "Accept": "application/json, text/event-stream",
    }
    if session:
        headers["Mcp-Session-Id"] = session
    req = urllib.request.Request(URL, json.dumps(payload).encode(), headers)
    with urllib.request.urlopen(req, timeout=60) as resp:
        sid = resp.headers.get("Mcp-Session-Id")
        body = resp.read().decode()
    # streamable http may answer as SSE ("data: {...}") or plain JSON
    for line in body.splitlines():
        if line.startswith("data:"):
            data = line[5:].strip()
            if data:
                return json.loads(data), sid
    return (json.loads(body) if body.strip() else None), sid


def main():
    tool = sys.argv[1]
    args = json.loads(sys.argv[2]) if len(sys.argv) > 2 else {}
    init, session = post({
        "jsonrpc": "2.0", "id": 1, "method": "initialize",
        "params": {
            "protocolVersion": "2025-03-26",
            "capabilities": {},
            "clientInfo": {"name": "cursor-agent-script", "version": "0"},
        },
    })
    post({"jsonrpc": "2.0", "method": "notifications/initialized"}, session)
    reply, _ = post({
        "jsonrpc": "2.0", "id": 2, "method": "tools/call",
        "params": {"name": tool, "arguments": args},
    }, session)
    result = reply.get("result", reply)
    for block in result.get("content", []):
        if block.get("type") == "text":
            print(block["text"])
            return
    print(json.dumps(result))


if __name__ == "__main__":
    main()
