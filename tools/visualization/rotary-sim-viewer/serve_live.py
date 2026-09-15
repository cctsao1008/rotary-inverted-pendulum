#!/usr/bin/env python3
"""Serve the Rotary viewer and stream persistent incremental SITL over SSE.

The browser remains a read-only evidence consumer. This server launches the
existing Rust semantic path through `rip-sitl-live`, forwards timestamped
samples at a display-limited rate, and terminates the child when the browser
stops or disconnects. No Furuta dynamics or controller logic live here.
"""

from __future__ import annotations

import argparse
import errno
import json
import math
import subprocess
import time
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]

SCENARIOS = {
    "balance": ROOT / "tools" / "sitl" / "scenarios" / "rotary_balance.toml",
    "swingup": ROOT / "tools" / "sitl" / "scenarios" / "rotary_swingup.toml",
}

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


def live_command(scenario_key: str) -> list[str]:
    return [
        "cargo",
        "run",
        "--quiet",
        "--manifest-path",
        str(ROOT / "tools" / "sitl" / "Cargo.toml"),
        "--bin",
        "rip-sitl-live",
        "--",
        "--scenario",
        str(SCENARIOS[scenario_key]),
    ]


def stop_process(process: subprocess.Popen[str] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=1.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1.0)


def log_capture_diagnostics(
    sample: dict,
    previous_regime: str | None,
    previous_reference_phase: str | None,
    last_log_t: float | None,
) -> tuple[str | None, str | None, float | None]:
    """Print sparse explanatory diagnostics around upright/capture/recenter events.

    The values are emitted by rip-sitl-live. This function only formats them;
    it does not recompute control or change the live stream.
    """
    regime = sample.get("control_regime")
    diagnostics = sample.get("hybrid_diagnostics")
    estimated = sample.get("estimated_state")
    if not isinstance(diagnostics, dict) or not isinstance(estimated, list) or len(estimated) != 4:
        return regime or previous_regime, previous_reference_phase, last_log_t

    t_s = float(sample.get("t_s", 0.0))
    theta = float(estimated[0])
    theta_dot = float(estimated[1])
    near_upright = abs(theta) <= math.radians(25.0)
    eligible = bool(diagnostics.get("capture_eligible", False))
    regime_changed = previous_regime is not None and regime != previous_regime

    reference = diagnostics.get("balance_reference")
    reference_phase = reference.get("phase") if isinstance(reference, dict) else None
    reference_phase_changed = (
        previous_reference_phase is not None
        and reference_phase is not None
        and reference_phase != previous_reference_phase
    )
    periodic_near = near_upright and (last_log_t is None or t_s - last_log_t >= 0.050)

    if regime_changed or reference_phase_changed or periodic_near:
        requested = sample.get("requested_arm_torque_nm")
        applied = sample.get("applied_arm_torque_nm")
        terms = diagnostics.get("state_feedback_terms_nm")
        if isinstance(terms, dict):
            term_text = (
                f"uθ={float(terms.get('theta', float('nan'))): .5f} "
                f"uθd={float(terms.get('theta_dot', float('nan'))): .5f} "
                f"uφ={float(terms.get('phi', float('nan'))): .5f} "
                f"uφd={float(terms.get('phi_dot', float('nan'))): .5f}Nm "
            )
        else:
            term_text = ""

        if isinstance(reference, dict):
            target = reference.get("recenter_target_phi_rad")
            target_text = (
                f"target={float(target): .3f}rad " if target is not None else "target=--- "
            )
            reference_text = (
                f"phase={str(reference_phase):9s} "
                f"φref={float(reference.get('phi_ref_rad', float('nan'))): .3f}rad "
                f"φdref={float(reference.get('phi_dot_ref_rad_s', float('nan'))): .3f}rad/s "
                f"φe={float(reference.get('phi_error_rad', float('nan'))): .3f}rad "
                f"φde={float(reference.get('phi_dot_error_rad_s', float('nan'))): .3f}rad/s "
                f"{target_text}"
            )
        else:
            reference_text = ""

        active_controller = diagnostics.get("active_controller", "unknown")
        global_zero_shadow = float(diagnostics.get("global_zero_shadow_torque_nm", float("nan")))
        print(
            "[capture] "
            f"t={t_s:8.4f}s regime={str(regime):7s} "
            f"ctrl={str(active_controller):28s} "
            f"theta={math.degrees(theta):8.3f}deg theta_dot={theta_dot:8.3f}rad/s "
            f"E={float(diagnostics['pendulum_energy_j']):.5f}/"
            f"{float(diagnostics['target_energy_j']):.5f}J "
            f"swing={float(diagnostics['swing_torque_nm']): .5f}Nm "
            f"capture={float(diagnostics.get('capture_torque_nm', float('nan'))): .5f}Nm "
            f"track={float(diagnostics['balance_torque_nm']): .5f}Nm "
            f"global0={global_zero_shadow: .5f}Nm "
            f"{term_text}"
            f"{reference_text}"
            f"selected={float(diagnostics['capture_blended_torque_nm']): .5f}Nm "
            f"req={float(requested) if requested is not None else float('nan'): .5f}Nm "
            f"applied={float(applied) if applied is not None else float('nan'): .5f}Nm "
            f"eligible={eligible}"
        )
        last_log_t = t_s

    return regime or previous_regime, reference_phase or previous_reference_phase, last_log_t


class Handler(SimpleHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def log_message(self, fmt: str, *args) -> None:
        print(f"[{self.log_date_time_string()}] {fmt % args}")

    def handle(self) -> None:
        # Browser Stop intentionally aborts the long-lived EventSource socket.
        # On Windows, the disconnect can surface while BaseHTTPRequestHandler is
        # trying to read the next request line, after do_GET() has already
        # returned. Catch it at the outer request-handler boundary as well as in
        # the SSE writer/finish paths.
        try:
            super().handle()
        except OSError as error:
            if not is_client_disconnect(error):
                raise

    def finish(self) -> None:
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

        # This request owns one long-lived SSE response and must never fall back
        # into HTTP/1.1 keep-alive request parsing after the browser presses Stop.
        self.close_connection = True
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "close")
        self.send_header("X-Accel-Buffering", "no")
        self.end_headers()

        process: subprocess.Popen[str] | None = None
        try:
            if not self.write_sse(
                "status",
                {"phase": "starting", "scenario": scenario_key},
            ):
                return

            process = subprocess.Popen(
                live_command(scenario_key),
                cwd=ROOT,
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=1,
            )
            assert process.stdout is not None

            meta_sent = False
            last_display_t: float | None = None
            next_display_t: float | None = None
            wall_anchor: float | None = None
            sim_anchor: float | None = None
            display_period = 1.0 / fps
            previous_regime: str | None = None
            previous_reference_phase: str | None = None
            last_diag_log_t: float | None = None

            for raw in process.stdout:
                line = raw.strip()
                if not line:
                    continue
                try:
                    message = json.loads(line)
                except json.JSONDecodeError as error:
                    raise RuntimeError(f"invalid rip-sitl-live JSON: {error}: {line[:160]}") from error

                message_type = message.get("type")
                if message_type == "meta":
                    meta = {
                        "schema": message["schema"],
                        "source": message["source"],
                        "state_order": message["state_order"],
                        "scenario": scenario_key,
                        "display_fps_limit": fps,
                        "speed": speed,
                        "transport": "persistent incremental rip-sitl-live",
                    }
                    if not self.write_sse("meta", meta):
                        return
                    meta_sent = True
                    continue

                if message_type != "sample":
                    continue
                if not meta_sent:
                    raise RuntimeError("rip-sitl-live emitted a sample before meta")

                sample = message["sample"]
                previous_regime, previous_reference_phase, last_diag_log_t = log_capture_diagnostics(
                    sample,
                    previous_regime,
                    previous_reference_phase,
                    last_diag_log_t,
                )
                sim_t = float(sample["t_s"])
                if next_display_t is None:
                    next_display_t = sim_t
                if sim_t + 1e-12 < next_display_t:
                    continue

                if wall_anchor is None:
                    wall_anchor = time.perf_counter()
                    sim_anchor = sim_t
                assert sim_anchor is not None
                due = wall_anchor + (sim_t - sim_anchor) / speed
                delay = due - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)

                if not self.write_sse("sample", sample):
                    return
                last_display_t = sim_t
                next_display_t += display_period

            stderr = process.stderr.read() if process.stderr is not None else ""
            code = process.wait()
            if code != 0:
                raise RuntimeError(stderr.strip() or f"rip-sitl-live exited with {code}")
            if last_display_t is not None:
                self.write_sse("end", {"t_s": last_display_t})
        except OSError as error:
            if not is_client_disconnect(error):
                try:
                    self.write_sse("stream-error", {"message": str(error)})
                except OSError as nested:
                    if not is_client_disconnect(nested):
                        raise
        except Exception as error:  # noqa: BLE001 - report backend failures to the viewer.
            try:
                self.write_sse("stream-error", {"message": str(error)})
            except OSError as nested:
                if not is_client_disconnect(nested):
                    raise
        finally:
            stop_process(process)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    args = parser.parse_args()

    server = ThreadingHTTPServer((args.host, args.port), Handler)
    print("Rotary Simulation Console live server")
    print(f"viewer: http://{args.host}:{args.port}/tools/visualization/rotary-sim-viewer/")
    print("Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, exiting.")
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
