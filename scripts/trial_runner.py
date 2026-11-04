#!/usr/bin/env python3
"""One-task-at-a-time trial runner for the RoboAPI console (HTTP only; never opens CAN).

Each trial starts a run, follows its action chunks until one of the budgets fires, stops, returns
Home, records the verdict/rating/comment, then blocks until Enter so the scene can be restored.

Stop criteria (all client side - the model protocol has no completion flag):
  * --max-seconds   wall-clock budget per trial (default 60 s)
  * --chunks        cap on action chunks, 0 = no cap (default 0)
  * --stall-chunks  consecutive chunks whose net displacement stays under --stall-mm (default 3)
  * the runtime itself stops (error / operator Robot Stop)

On exit the arm is left in gravity compensation and the success rate prints.
"""

from __future__ import annotations

import argparse
import atexit
import csv
import datetime
import json
import pathlib
import subprocess
import sys
import time
import urllib.error
import urllib.request

DEFAULT_URL = "http://127.0.0.1:8080"
HOME_DIR = pathlib.Path.home() / "roboapi"
TRIALS_DIR = HOME_DIR / "trials"
MODE_FILE = HOME_DIR / "state" / "yam_mode"
RESULTS = {"s": "success", "f": "fail", "q": "quit", "": "success"}


def _leave_gravity():
    try:
        MODE_FILE.parent.mkdir(parents=True, exist_ok=True)
        MODE_FILE.write_text("gravity" + chr(10), encoding="utf-8")
    except OSError:
        pass


atexit.register(_leave_gravity)


def http(method: str, path: str, body=None, timeout: float = 60.0):
    data = None if body is None else json.dumps(body).encode()
    request = urllib.request.Request(DEFAULT_URL + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.loads(response.read().decode() or "{}")
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode(errors="replace")
        raise RuntimeError(f"{method} {path} -> HTTP {exc.code}: {detail[:300]}") from None
    except urllib.error.URLError as exc:
        raise RuntimeError(f"cannot reach the console at {DEFAULT_URL}: {exc.reason}") from None


def status():
    return http("GET", "/api/status", timeout=10.0)


def wait_home(timeout_s: float = 15.0) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            if http("GET", "/api/calibration/check", timeout=5.0).get("at_home"):
                return True
        except RuntimeError:
            pass
        time.sleep(0.3)
    return False


def set_mode(value: str) -> None:
    try:
        MODE_FILE.parent.mkdir(parents=True, exist_ok=True)
        MODE_FILE.write_text(value + chr(10), encoding="utf-8")
    except OSError:
        pass


def go_home(timeout_s: float = 35.0):
    """Ask for Home and only report success when the call itself succeeded and the arm arrived."""
    set_mode("servo")
    time.sleep(0.6)
    error = ""
    try:
        http("POST", "/api/robot/reset", {}, timeout=timeout_s)
    except RuntimeError as exc:
        error = str(exc)
    if error:
        return False, error
    arrived = wait_home(timeout_s)
    time.sleep(0.4)
    return arrived and wait_home(3.0), error


def refresh_review_links() -> None:
    """Refresh the readable-name symlinks in ~/roboapi/review (read-only for the console data)."""
    script = HOME_DIR / "scripts" / "link_tasks.py"
    if not script.exists():
        return
    try:
        subprocess.run([sys.executable, str(script)], check=False, timeout=60,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def load_task(args):
    if args.instruction:
        return args.task or "adhoc", args.instruction
    path = pathlib.Path(args.tasks_file).expanduser()
    if not path.exists():
        raise SystemExit("no --instruction given and tasks file not found: " + str(path))
    raw = path.read_text(encoding="utf-8")
    if path.suffix in (".yaml", ".yml"):
        try:
            import yaml
        except ModuleNotFoundError:
            raise SystemExit("PyYAML missing in this interpreter; run scripts/run_trials.sh") from None
        data = yaml.safe_load(raw)
    else:
        data = json.loads(raw)
    tasks = data.get("tasks", data) if isinstance(data, dict) else data
    if not isinstance(tasks, list) or not tasks:
        raise SystemExit("tasks file must hold a non-empty list of {name, instruction}")
    if args.task:
        for item in tasks:
            if item.get("name") == args.task:
                return item["name"], item["instruction"]
        raise SystemExit("task " + repr(args.task) + " not in " + str(path))
    item = tasks[0]
    return item.get("name", "task"), item["instruction"]


def chunk_magnitude(event):
    output = event.get("output") or {}
    delta = output.get("delta_xyz_m") or []
    if len(delta) != 3:
        return None
    return (sum(float(v) ** 2 for v in delta)) ** 0.5


def run_one(instr, timeout_s, chunks, max_seconds, stall_chunks, stall_mm):
    before = status()
    if not before.get("alive") or not before.get("control_ready"):
        raise RuntimeError("robot is not under control; enable control in the console first")
    if before.get("mode") != "idle":
        raise RuntimeError("console is busy (mode=" + str(before.get("mode")) + ")")
    probe = http("GET", "/api/calibration/check", timeout=5.0)
    if not probe.get("at_home"):
        raise RuntimeError("robot is not at Home; refusing to start a trial")

    print("   推理中（上限 %.0fs，停滞判定 %d 块 < %.0f mm）..." % (max_seconds, stall_chunks, stall_mm),
          flush=True)
    started = time.time()
    http("POST", "/api/policy/start", {"instruction": instr, "new_task": True}, timeout=30.0)
    task_id = status().get("task_id") or ""

    seen = set()
    for event in before.get("inference_events") or []:
        if event.get("request_id"):
            seen.add(event["request_id"])
    recent, total, count = [], 0.0, 0
    stop_kind = "unknown"
    deadline = started + max_seconds

    while True:
        if time.time() >= deadline:
            stop_kind = "time"
            break
        time.sleep(0.2)
        now = status()
        for event in now.get("inference_events") or []:
            rid = event.get("request_id")
            if rid and rid in seen:
                continue
            if rid:
                seen.add(rid)
            if event.get("status") != "ok":
                continue
            magnitude = chunk_magnitude(event)
            if magnitude is None:
                continue
            count += 1
            total += magnitude
            recent.append(magnitude)
            print("   chunk %d: 净位移 %5.1f mm" % (count, magnitude * 1000), flush=True)
            if chunks and count >= chunks:
                stop_kind = "chunks"
                break
            if (len(recent) >= stall_chunks
                    and all(v * 1000.0 < stall_mm for v in recent[-stall_chunks:])):
                stop_kind = "stalled"
                break
        if stop_kind != "unknown":
            break
        if now.get("mode") not in ("running", "paused"):
            stop_kind = "runtime_stop"
            break

    print("   停止（原因 %s，共 %d 块）. 回 Home ..." % (stop_kind, count), flush=True)
    http("POST", "/api/stop", {}, timeout=20.0)
    for _ in range(30):
        if status().get("mode") == "idle":
            break
        time.sleep(0.1)
    at_home, reset_error = go_home(15.0)
    if not at_home:
        time.sleep(0.5)
        at_home, reset_error = go_home(15.0)

    final = status()
    events = final.get("inference_events") or []
    last = events[-1] if events else {}
    return {
        "task_id": task_id,
        "chunks_run": count,
        "total_displacement_m": round(total, 4),
        "stop_kind": stop_kind,
        "duration_s": round(time.time() - started, 1),
        "at_home": at_home,
        "infer_ms": last.get("infer_ms"),
        "action_ms": last.get("action_ms"),
        "stop_reason": final.get("stop_reason"),
        "reset_error": reset_error,
        "error": last.get("error_kind") or final.get("last_error"),
    }


def ask(prompt: str) -> str:
    try:
        return input(prompt)
    except EOFError:
        return "q"


def main() -> int:
    global DEFAULT_URL
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", help="task name (selects from --tasks-file)")
    parser.add_argument("--instruction", help="instruction text (overrides the tasks file)")
    parser.add_argument("--tasks-file", default=str(HOME_DIR / "tasks.yaml"))
    parser.add_argument("--trials", type=int, default=20)
    parser.add_argument("--chunks", type=int, default=0, help="max chunks per trial, 0 = no cap")
    parser.add_argument("--max-seconds", type=float, default=60.0, help="wall-clock budget per trial")
    parser.add_argument("--stall-chunks", type=int, default=3,
                        help="consecutive small chunks that end a trial")
    parser.add_argument("--stall-mm", type=float, default=5.0, help="net displacement counted as stalled")
    parser.add_argument("--comment", default="", help="comment appended to every trial")
    parser.add_argument("--timeout", type=float, default=120.0, help="seconds to wait for one chunk")
    parser.add_argument("--csv", default="", help="output csv path")
    parser.add_argument("--url", default=DEFAULT_URL, help="console base url")
    parser.add_argument("--dry-run", action="store_true", help="check the console, run nothing")
    args = parser.parse_args()
    DEFAULT_URL = args.url

    name, instruction = load_task(args)
    print("task       : " + name)
    print("instruction: " + instruction)
    print("trials     : %d, 每轮上限 %.0fs, 停滞 %d 块 < %.0f mm, chunks 上限 %s"
          % (args.trials, args.max_seconds, args.stall_chunks, args.stall_mm,
             args.chunks if args.chunks else "不限"))

    try:
        state = status()
    except RuntimeError as exc:
        print("ERROR:", exc)
        return 2
    print("console    : mode=%s alive=%s control_ready=%s camera=%s"
          % (state.get("mode"), state.get("alive"), state.get("control_ready"),
             state.get("camera_connected")))
    if args.dry_run:
        print("dry run only; nothing started")
        return 0
    if not state.get("alive") or not state.get("control_ready"):
        print("ERROR: enable control in the console before running trials")
        return 2

    print(chr(10) + "启动前回到 Home ...")
    at_home, home_error = go_home(40.0)
    if not at_home:
        print("ERROR: 回 Home 失败，未开始任何推理。")
        if home_error:
            print("      控制台返回:", home_error)
        else:
            print("      臂未在 Home 容差内（检查是否有人在动它，或先点页面『初始位置』）。")
        return 2
    print("已在 Home（已复核），开始试验。")

    TRIALS_DIR.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = pathlib.Path(args.csv) if args.csv else TRIALS_DIR / (name + "_" + stamp + ".csv")
    fields = ["ts", "task", "trial", "task_id", "result", "rating", "comment", "instruction",
              "chunks_run", "total_displacement_m", "stop_kind", "duration_s", "at_home",
              "infer_ms", "action_ms", "stop_reason", "reset_error", "error"]
    results, ratings = [], []

    print(chr(10) + "按 Enter 开始第一次推理，输入 q 退出。")
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=fields)
        writer.writeheader()
        for trial in range(1, args.trials + 1):
            gate = ask("")
            if gate.strip().lower() == "q":
                break
            print(chr(10) + "--- trial %d/%d ---" % (trial, args.trials))
            try:
                outcome = run_one(instruction, args.timeout, args.chunks, args.max_seconds,
                                  args.stall_chunks, args.stall_mm)
            except RuntimeError as exc:
                print("运行时错误:", exc)
                outcome = {"task_id": "", "chunks_run": 0, "total_displacement_m": 0.0,
                           "stop_kind": "error", "duration_s": 0.0, "at_home": False,
                           "infer_ms": None, "action_ms": None, "stop_reason": "error",
                           "reset_error": "", "error": str(exc)[:200]}
            print("完成: 块=%s 累计位移=%.3f m 原因=%s 用时=%.1fs 回Home=%s"
                  % (outcome["chunks_run"], outcome["total_displacement_m"], outcome["stop_kind"],
                     outcome["duration_s"], outcome["at_home"]))
            verdict = ask("结果  s=成功 / f=失败 / q=退出 [默认 s] > ").strip().lower()
            if verdict == "q":
                results.append("quit")
                break
            result = RESULTS.get(verdict, "fail")
            rating = ask("评分 1-5（可选，回车跳过）> ").strip()
            note = ask("备注（可选，回车跳过）> ").strip()
            comment = "; ".join(x for x in (args.comment, note) if x)
            results.append(result)
            if rating in {"1", "2", "3", "4", "5"}:
                ratings.append(int(rating))
            refresh_review_links()
            writer.writerow({"ts": datetime.datetime.now().isoformat(timespec="seconds"),
                             "task": name, "trial": trial, "result": result, "rating": rating,
                             "comment": comment, "instruction": instruction, **outcome})
            fh.flush()
            if trial < args.trials:
                ask(chr(10) + "请还原场景，完成后按 Enter 继续下一次（q 退出）> ")
    refresh_review_links()
    set_mode("gravity")

    done = [r for r in results if r in ("success", "fail")]
    ok = done.count("success")
    rate = (ok / len(done) * 100) if done else 0.0
    print(chr(10) + "=" * 40)
    print("任务 %s: 成功 %d/%d = %.1f%%" % (name, ok, len(done), rate))
    if ratings:
        print("平均评分: %.2f (n=%d)" % (sum(ratings) / len(ratings), len(ratings)))
    print("CSV: " + str(csv_path))
    print("机械臂已切到重力补偿，可手推收回。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
