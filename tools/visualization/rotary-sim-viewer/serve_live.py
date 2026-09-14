#!/usr/bin/env python3
"""Serve the Rotary viewer and stream fresh SITL runs over Server-Sent Events.

The browser remains a read-only evidence consumer. This server launches the
existing `rip-sitl` binary, normalizes its completed JSONL evidence with the
existing adapter, and streams display frames to the viewer at wall-clock pace.
No Furuta dynamics or controller logic are implemented here.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import subprocess
import tempfile
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from adapt_sitl_trace import load_json, load_jsonl, normalize

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

SCENARIOS = {
    "balance": ROOT / "tools" / "sitl" / "scenarios" / "rotary_balance.toml",
    "swingup": ROOT / "tools" / "sitl" / "scenarios" / "rotary_swingup.toml",
}

# Closing a browser EventSource is a normal end-of-stream condition. Python and
# Windows do not always surface that socket close through the same exception
# subclass, so classify both portable errno values and Winsock-specific codes.
CLIENT_DISCONNECT_ERRORS = (
    BrokenPipeError,
    ConnectionResetError,
    ConnectionAbortedError,
)
CLIENT_DISCONNECT_ERRNOS = {
    errno.EPIPE,
    errno.ECONNRESET,
    errno.ECONNABORTED,
}
CLIENT_DISCONNECT_WINERRORS = {
    10053,  # WSAECONNABORTED
    10054,  # WSAECONNRESET
    10058,  # WSAESHUTDOWN
}


def is_client_disconnect(error: BaseException) -> bool:
    if isinstance(error, CLIENT_DISCONNECT_ERRORS):
        return True
    if isinstance(error, OSError):
        if error.errno in CLIENT_DISCONNECT_ERRNOS:
            return True
        if getattr(error, "winerror", None) in CLIENT_DISCONNECT_WINERRORS:
            return True
    return False


def sse_payload(event: str, payload: object) -> bytes:
    data = json.dumps(payload, separators=(",", ":"), ensure_ascii=False)
    return f"event: {event}\ndata: {data}\n\n".encode("utf-8")


def display_frames(samples: list[dict], fps: float) -> list[dict]:
    if len(samples) <= 2:
        return samples
    period = 1.0 / fps
    selected = [samples[0]]
    next_t = float(samples[0]["t_s"]) + period
    for sample in samples[1:-1]:
        t_s = float(sample["t_s"])
        if t_s + 1e-12 >= next_t:
            selected.append(sample)
            while next_t <= t_s + 1e-12:
                next_t += period
    if selected[-1] is not samples[-1]:
        selected.append(samples[-1])
    return selected


def run_sitl(scenario_key: str) -> dict:
    scenario = SCENARIOS[scenario_key]
    with tempfile.TemporaryDirectory(prefix="rotary-sitl-live-") as temp:
        output = Path(temp) / "run"
        command = [
            "cargo",
            "run",
            "--quiet",
            "--manifest-path",
            str(ROOT / "tools" / "sitl" / "Cargo.toml"),
            "--bin",
            "rip-sitl",
            "--",
            "--scenario",
            str(scenario),
            "--output",
            str(output),
        ]
        completed = subprocess.run(
            command,
            cwd=ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        if completed.returncode != 0:
            detail = completed.stderr.strip() or completed.stdout.strip() or "unknown SITL failure"
            raise RuntimeError(detail)
        manifest = load_json(output / "manifest.json")
        payload = normalize(load_jsonl(output / "trace.jsonl"), manifest)
        payload["source"]["transport"] = "fresh rip-sitl run streamed by local SSE bridge"
        return payload


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        print(f"[{self.log_date_time_string()}] {fmt % args}")

    def finish(self) -> None:
        # StreamRequestHandler.finish() flushes wfile after do_GET returns. If the
        # browser intentionally closed EventSource, Windows may report that final
        # flush as WSAECONNABORTED. That is not a server failure.
        try:
            super().finish()
        except OSError as error:
            if not is_client_disconnect(error):
                raise

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/api/health":
            body = json.dumps({"ok": True, "service": "rotary-sim-viewer-live"}).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if parsed.path == "/api/live":
            self.stream_live(parsed.query)
            return
        super().do_GET()

    def write_sse(self, event: str, payload: object) -> bool:
        try:
            self.wfile.write(sse_payload(event, payload))
            self.wfile.flush()
            return True
        except OSError as error:
            if is_client_disconnect(error):
                return False
            raise

    def stream_live(self, query: str) -> None:
        params = parse_qs(query)
        scenario_key = params.get("scenario", ["balance"])[0]
        if scenario_key not in SCENARIOS:
            self.send_error(400, "unknown scenario")
            return
        try:
            speed = float(params.get("speed", ["1"])[0])
            fps = float(params.get("fps", ["60"])[0])
        except ValueError:
            self.send_error(400, "speed and fps must be numeric")
            return
        if not math.isfinite(speed) or not (0.1 <= speed <= 10.0):
            self.send_error(400, "speed must be in [0.1, 10]")
            return
        if not math.isfinite(fps) or not (5.0 <= fps <= 120.0):
            self.send_error(400, "fps must be in [5, 120]")
            return

        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        # One Live click owns one finite SITL experiment. Replaying finite 5 s
        # scenarios in an endless server loop caused an artificial state reset at
        # every scenario boundary, visible as a periodic twitch in the 3-D model.
        # A genuinely continuous live lane should eventually come from an
        # incremental SITL observer, not from stitching deterministic runs.
        run_index = 1
        try:
            if not self.write_sse("status", {
                "phase": "simulating",
                "scenario": scenario_key,
                "run": run_index,
            }):
                return

            payload = run_sitl(scenario_key)
            frames = display_frames(payload["samples"], fps)
            if not self.write_sse("meta", {
                "schema": payload["schema"],
                "source": payload["source"],
                "state_order": payload["state_order"],
                "scenario": scenario_key,
                "run": run_index,
                "source_samples": len(payload["samples"]),
                "display_frames": len(frames),
                "display_fps_limit": fps,
                "speed": speed,
            }):
                return

            previous_t = None
            for sample in frames:
                t_s = float(sample["t_s"])
                if previous_t is not None:
                    time.sleep(max(0.0, (t_s - previous_t) / speed))
                if not self.write_sse("sample", sample):
                    return
                previous_t = t_s

            self.write_sse("status", {
                "phase": "run-complete",
                "scenario": scenario_key,
                "run": run_index,
            })
        except Exception as error:  # surface real local tool failures to the UI
            if is_client_disconnect(error):
                return
            try:
                self.write_sse("stream-error", {"message": str(error)})
            except Exception as write_error:
                if not is_client_disconnect(write_error):
                    raise


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    url = f"http://{args.host}:{args.port}/tools/visualization/rotary-sim-viewer/"
    print("Rotary Simulation Console live server")
    print(f"viewer: {url}")
    print("Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
