"""TUI 会话的异步日志、运行指标和 Linux CPU 采样。"""

from collections import Counter
from dataclasses import asdict, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import hashlib
from importlib.metadata import PackageNotFoundError, version
import json
import os
from pathlib import Path
import platform
from queue import Empty, Full, Queue
import threading
import time
import uuid

from .. import log
from ..diagnostics import trace_schema
from ..performance import Distribution, PerformanceProbe


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def _json_default(value):
    if is_dataclass(value):
        return asdict(value)
    if isinstance(value, Enum):
        return value.name
    if isinstance(value, Path):
        return str(value)
    raise TypeError(type(value).__name__)


def _json(value):
    return json.dumps(value, default=_json_default, ensure_ascii=False, allow_nan=False)


def _atomic_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(_json(value)+"\n", encoding="utf-8")
    temporary.replace(path)


class LinuxCPU:
    def __init__(self):
        self.hz = os.sysconf("SC_CLK_TCK") if os.name == "posix" else 100
        self.previous = {}
        self.cpu_previous = {}
        self.last_ns = None

    @staticmethod
    def _stat(path):
        text = path.read_text()
        fields = text[text.rfind(")")+2:].split()
        return dict(name=text[text.find("(")+1:text.rfind(")")],
                    ticks=int(fields[11])+int(fields[12]), start_ticks=int(fields[19]),
                    state=fields[0], last_cpu=int(fields[36]))

    def sample(self, pids):
        now = time.monotonic_ns()
        elapsed = (now-self.last_ns)/1e9 if self.last_ns is not None else None
        self.last_ns = now
        result = dict(processes=[], host_cpu_percent=None, per_cpu_percent={})
        stat = Path("/proc/stat")
        if not stat.exists():
            result["available"] = False
            return result
        for line in stat.read_text().splitlines():
            fields = line.split()
            if not fields or not fields[0].startswith("cpu"):
                continue
            values = list(map(int, fields[1:9]))
            total, idle = sum(values), values[3]+values[4]
            old = self.cpu_previous.get(fields[0])
            self.cpu_previous[fields[0]] = total, idle
            percent = 100*(1-(idle-old[1])/(total-old[0])) if old and total > old[0] else None
            if fields[0] == "cpu":
                result["host_cpu_percent"] = percent
            else:
                result["per_cpu_percent"][fields[0]] = percent
        active = set()
        for pid in sorted(set(pid for pid in pids if pid)):
            directory = Path("/proc")/str(pid)
            try:
                process = self._stat(directory/"stat")
                rows = [("process", pid, process)]
                for task in (directory/"task").iterdir():
                    try:
                        rows.append(("thread", int(task.name), self._stat(task/"stat")))
                    except (OSError, ValueError, IndexError):
                        pass
                metrics = dict(pid=pid, threads=[])
                for kind, ident, data in rows:
                    key = kind, pid, ident, data["start_ticks"]
                    active.add(key)
                    before = self.previous.get(key)
                    self.previous[key] = data["ticks"]
                    data["cpu_percent_one_core"] = (100*max(0, data["ticks"]-before)/self.hz/elapsed
                        if before is not None and elapsed and elapsed > 0 else None)
                    if kind == "process":
                        metrics.update(data)
                    else:
                        metrics["threads"].append(dict(tid=ident, **data))
                for line in (directory/"status").read_text().splitlines():
                    if line.startswith("VmRSS:"):
                        metrics["rss_bytes"] = int(line.split()[1])*1024
                result["processes"].append(metrics)
            except (OSError, ValueError, IndexError):
                continue
        self.previous = {k: v for k, v in self.previous.items() if k in active}
        policies = list(Path("/sys/devices/system/cpu/cpufreq").glob("policy*"))
        governors = Counter()
        for policy in policies:
            try:
                governors[(policy/"scaling_governor").read_text().strip()] += 1
            except OSError:
                pass
        result["governors"] = dict(governors)
        if hasattr(os, "getloadavg"):
            result["load_average"] = os.getloadavg()
        return result


class RuntimeRecorder:
    def __init__(self, options):
        root = Path(options.log_dir).expanduser() if options.log_dir else Path(
            os.environ.get("XDG_STATE_HOME", str(Path.home()/".local/state")))/"arx-x5-control"/"runs"
        self.directory = root.resolve()/(datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
                                       +f"-{os.getpid()}-{uuid.uuid4().hex[:6]}")
        self.options = options
        self.probe = PerformanceProbe()
        self.error = None
        self.dropped = 0
        self._queue = Queue(maxsize=4096)
        self._stop = threading.Event()
        self._binding = None
        self._session = None
        self._files = {}
        self._totals = {}
        self._counts = Counter()
        self._peaks = {}
        self._cpu_maxima = {}
        self._final = {}
        self._status = "running"
        self._failure = None
        self._origin_ns = time.monotonic_ns()
        self._cpu = LinuxCPU()
        self._subscription = None
        self._thread = None

    def __enter__(self):
        try:
            return self._start()
        except BaseException:
            for output in self._files.values():
                output.close()
            if self._subscription is not None:
                self._subscription.__exit__(None, None, None)
            raise

    def _start(self):
        self.directory.mkdir(parents=True)
        try:
            package_version = version("arx-x5-control")
        except PackageNotFoundError:
            package_version = "source"
        files = ("sdk_adapter/link.py", "can_io.py", "can_process.py", "can_transport.py",
                 "sdk_adapter/tui.py", "sdk_adapter/tui_app.py", "sdk_adapter/telemetry.py",
                 "performance.py", "diagnostics.py")
        package_root = Path(__file__).resolve().parent.parent
        metadata = dict(schema_version=1, started_utc=utc_now(), monotonic_origin_ns=self._origin_ns,
                        package_version=package_version, hostname=platform.node(), kernel=platform.release(),
                        python=platform.python_version(), pid=os.getpid(), logical_cpus=os.cpu_count(),
                        affinity=sorted(os.sched_getaffinity(0)) if hasattr(os, "sched_getaffinity") else None,
                        options=vars(self.options), metric_period_s=1, state_period_s=.1,
                        source_sha256={name: hashlib.sha256((package_root/name).read_bytes()).hexdigest()
                                       for name in files})
        _atomic_json(self.directory/"metadata.json", metadata)
        for name in ("metrics.jsonl", "states.jsonl", "events.jsonl", "runtime.log", "ui.log"):
            self._files[name] = (self.directory/name).open("w", encoding="utf-8")
        self._files["control_trace.bin"] = (self.directory/"control_trace.bin").open("wb")
        _atomic_json(self.directory/"control_trace_schema.json", trace_schema())
        _atomic_json(self.directory/"summary.json", dict(schema_version=1, status="running"))
        pointer = self.directory.parent/(".latest-"+uuid.uuid4().hex)
        try:
            pointer.symlink_to(self.directory.name, target_is_directory=True)
            pointer.replace(self.directory.parent/"latest")
        except OSError:
            pointer.unlink(missing_ok=True)
            (self.directory.parent/"latest.txt").write_text(str(self.directory)+"\n")
        self._cpu.sample([os.getpid()])
        self._subscription = log.subscribe(lambda row: self.emit("runtime", row))
        self._subscription.__enter__()
        self._thread = threading.Thread(target=self._run, name="arx-run-logger", daemon=True)
        self._thread.start()
        return self

    def attach(self, link, motor_io):
        self._binding = link, motor_io
        self.emit("session", dict(controller_origin_ns=link._origin_ns, configurations=link.configurations))

    def session(self, session):
        self._session = session

    def emit(self, kind, data):
        if self.error is not None:
            return
        row = dict(kind=kind, utc=utc_now(), monotonic_ns=time.monotonic_ns(), data=data)
        try:
            self._queue.put_nowait(row)
        except Full:
            self.dropped += 1

    def _write(self, row):
        kind, data = row["kind"], row["data"]
        if kind in ("ui", "runtime"):
            message = data["text"] if kind == "ui" else data["message"]
            level = data["level"]
            self._files["ui.log" if kind == "ui" else "runtime.log"].write(f"{row['utc']} {level} {message}\n")
        else:
            self._files["events.jsonl"].write(_json(row)+"\n")

    def _performance(self, source, snapshot):
        if snapshot is None:
            return
        self._files["metrics.jsonl"].write(_json(dict(kind="performance", source=source, **snapshot))+"\n")
        for name, distribution in snapshot["timings"].items():
            key = source+"."+name
            if key not in self._totals:
                self._totals[key] = Distribution()
            self._totals[key].merge(distribution)
        self._counts.update({source+"."+name: value for name, value in snapshot["counters"].items()})
        for name, value in snapshot["peaks"].items():
            key = source+"."+name
            self._peaks[key] = max(value, self._peaks.get(key, 0))

    def _collect(self):
        link_stats, io_stats = {}, {}
        self.probe.tick("logger.period", time.monotonic_ns())
        if self._binding is not None:
            link, motor_io = self._binding
            link_stats = link.stats
            self._performance("host", link.performance_snapshot())
            if callable(getattr(motor_io, "stats", None)):
                io_stats = motor_io.stats()
            if callable(getattr(motor_io, "pop_performance", None)):
                for snapshot in motor_io.pop_performance():
                    self._performance("can", snapshot)
        self._performance("tui", self.probe.snapshot())
        cpu = self._cpu.sample([os.getpid(), io_stats.get("worker_pid")])
        roles = {t.native_id: t.name for t in threading.enumerate() if t.native_id is not None}
        if io_stats.get("io_thread_id"):
            roles[io_stats["io_thread_id"]] = "arx-can-io"
        for process in cpu["processes"]:
            for thread in process["threads"]:
                thread["role"] = roles.get(thread["tid"], thread["name"])
            value = process["cpu_percent_one_core"]
            if value is not None:
                key = str(process["pid"])
                self._cpu_maxima[key] = max(value, self._cpu_maxima.get(key, 0))
        row = dict(kind="sample", utc=utc_now(), monotonic_ns=time.monotonic_ns(), cpu=cpu,
                   link=link_stats, can=io_stats, recorder=dict(dropped=self.dropped, error=self.error))
        self._files["metrics.jsonl"].write(_json(row)+"\n")
        self._final = row

    def _control_trace(self):
        if self._binding is not None:
            self._files["control_trace.bin"].write(self._binding[0].pop_control_trace())

    def _states(self):
        if self._session is None:
            return
        arms = []
        for arm_id, view in self._session.views.items():
            snapshot = view.arm.snapshot()
            if snapshot is not None:
                arms.append(dict(arm_id=arm_id, robot_state=snapshot.robot_state, mode=snapshot.mode,
                    joint_angles_deg=snapshot.joint_angles_deg, joint_velocities_deg_s=snapshot.joint_velocities_deg_s,
                    gripper_raw_deg=snapshot.gripper_raw_deg, temperatures=snapshot.temperatures,
                    motor_lost_mask=snapshot.motor_lost_mask, motor_inited_mask=snapshot.motor_inited_mask,
                    feedback_age_s=snapshot.age, target_rad=list(view.target), hold=view.hold))
        if arms:
            self._files["states.jsonl"].write(_json(dict(monotonic_ns=time.monotonic_ns(), utc=utc_now(), arms=arms))+"\n")

    def _summary(self, *, final=False):
        _atomic_json(self.directory/"summary.json", dict(schema_version=1, status=self._status,
            error=self.error or self._failure, duration_s=(time.monotonic_ns()-self._origin_ns)/1e9,
            dropped_log_records=self.dropped,
            timing_totals={key: value.snapshot() for key, value in self._totals.items()} if final else {},
            counters=dict(self._counts), peaks=self._peaks,
            process_cpu_peak_percent_one_core=self._cpu_maxima, final=self._final))

    def _run(self):
        next_metrics = time.monotonic()+1
        next_states = time.monotonic()+.1
        try:
            while not self._stop.is_set() or not self._queue.empty():
                try:
                    self._write(self._queue.get(timeout=.05))
                except Empty:
                    pass
                self._control_trace()
                now = time.monotonic()
                if now >= next_states:
                    self._states()
                    next_states = now+.1
                if now >= next_metrics:
                    self._collect()
                    self._summary()
                    for output in self._files.values():
                        output.flush()
                    next_metrics = now+1
            self._control_trace()
            self._collect()
            if self._binding is not None:
                for event in self._binding[0].drain_events():
                    self._write(dict(kind="control", utc=utc_now(), monotonic_ns=time.monotonic_ns(), data=event))
            self._summary(final=True)
        except Exception as error:
            self.error = str(error)
            self._status = "error"
            try:
                self._summary()
            except Exception:
                pass
        finally:
            for output in self._files.values():
                try:
                    output.close()
                except OSError as error:
                    self.error = self.error or str(error)

    def __exit__(self, exc_type, exc, traceback):
        self._failure = str(exc) if exc is not None else None
        self._status = "interrupted" if exc_type is KeyboardInterrupt else ("error" if exc or self.error else "complete")
        self._stop.set()
        if self._thread is not None:
            self._thread.join(5)
            if self._thread.is_alive():
                self.error = self.error or "日志线程关闭超时"
        if self._subscription is not None:
            self._subscription.__exit__(exc_type, exc, traceback)
