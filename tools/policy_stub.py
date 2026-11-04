#!/usr/bin/env python3
"""Minimal OpenAI-compatible policy server for hardware-free RoboAPI dry runs.

Implements just enough of the RoboAPI local-runtime protocol (model list, session, one
execute_actions tool call, session close) so the console can run a full task loop without the
cloud. The returned action chunk is a small constant-velocity move in the task frame, built from
the state the client just sent - it proves the wiring, it is not a learned policy.
"""

from __future__ import annotations

import argparse
import json
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MODELS = ["stub-vla"]
SESSIONS: dict[str, int] = {}
LOCK = threading.Lock()


def _last_state(body):
    for message in reversed(body.get("messages", [])):
        if message.get("role") != "user":
            continue
        content = message.get("content")
        parts = content if isinstance(content, list) else [{"type": "text", "text": content}]
        for part in parts:
            text = part.get("text") if isinstance(part, dict) else None
            if not isinstance(text, str):
                continue
            try:
                payload = json.loads(text)
            except json.JSONDecodeError:
                continue
            if isinstance(payload, dict) and "state" in payload:
                return payload
    return {"instruction": "", "state": [0, 0, 0, 0, 0, 0, 1, 1]}


def _chunk(state):
    values = [float(v) for v in state[:8]]
    x, y, z, qx, qy, qz, qw, grip = values
    steps = 15
    return [[x, round(y + 0.0015 * (i + 1), 5), z, qx, qy, qz, qw, grip] for i in range(steps)]


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):  # keep the console quiet
        pass

    def _send(self, code, payload, extra_headers=None):
        data = json.dumps(payload).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        for key, value in (extra_headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path.rstrip("/").endswith("/models"):
            return self._send(200, {"object": "list", "data": [{"id": m, "object": "model"} for m in MODELS]})
        self._send(404, {"error": "not found"})

    def do_POST(self):
        length = int(self.headers.get("Content-Length") or 0)
        raw = self.rfile.read(length) if length else b"{}"
        try:
            body = json.loads(raw or b"{}")
        except json.JSONDecodeError:
            return self._send(400, {"error": "invalid json"})
        path = self.path.rstrip("/")
        if path.endswith("/sessions"):
            with LOCK:
                session_id = "stub_" + secrets.token_hex(6)
                SESSIONS[session_id] = 1
            return self._send(200, {"id": session_id, "model": body.get("model", MODELS[0]),
                                    "status": "open", "next_turn": 1, "last_request_id": None,
                                    "history": [], "messages": []})
        if "/sessions/" in path and path.endswith("/close"):
            session_id = path.split("/sessions/")[-1].split("/")[0]
            with LOCK:
                turn = SESSIONS.get(session_id, 1)
            return self._send(200, {"id": session_id, "status": "closed", "next_turn": turn,
                                    "last_request_id": None, "history": [], "messages": []})
        if path.endswith("/chat/completions"):
            return self._completion(body)
        self._send(404, {"error": "not found"})

    def _completion(self, body):
        metadata = body.get("metadata") or {}
        session_id = metadata.get("session_id")
        turn = metadata.get("turn", "1")
        state = _last_state(body).get("state") or [0, 0, 0, 0, 0, 0, 1, 1]
        request_id = secrets.token_hex(16)
        payload = {
            "id": "chatcmpl-" + request_id,
            "object": "chat.completion",
            "model": body.get("model", MODELS[0]),
            "choices": [{
                "index": 0,
                "finish_reason": "tool_calls",
                "message": {
                    "role": "assistant",
                    "content": None,
                    "tool_calls": [{
                        "id": "call_" + request_id,
                        "type": "function",
                        "function": {"name": "execute_actions",
                                     "arguments": json.dumps({"output": _chunk(state)})},
                    }],
                },
            }],
            "metadata": {"session_id": session_id, "turn": str(turn),
                         "next_turn": str(int(turn) + 1)},
            "usage": {"prompt_tokens": 0, "completion_tokens": 0, "total_tokens": 0},
        }
        with LOCK:
            if session_id in SESSIONS:
                SESSIONS[session_id] = int(turn) + 1
        self._send(200, payload, {"X-Request-ID": request_id})


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9010)
    args = parser.parse_args()
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"stub policy listening on http://{args.host}:{args.port}/v1", flush=True)
    server.serve_forever()


if __name__ == "__main__":
    main()
