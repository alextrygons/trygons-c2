"""Child processes (ffmpeg readers / test cameras) that must die with the program that started
them - also when that program is killed (a test timeout, Task Manager), so no orphaned ffmpeg
keeps encoding in the background and eating the PC.

    dies_with_parent(proc)      after subprocess.Popen(...)

Windows: the child is put into a Job object of this process with KILL_ON_JOB_CLOSE - when
this process ends for any reason the OS closes the job and kills everything in it. Elsewhere
(Ubuntu gateway PC: systemd already kills a service's children) it does nothing.
"""
import sys

_job = None


def _job_handle():
    global _job
    if _job is not None:
        return _job
    import ctypes
    from ctypes import wintypes
    k32 = ctypes.windll.kernel32

    class JOBOBJECT_BASIC_LIMIT_INFORMATION(ctypes.Structure):
        _fields_ = [("PerProcessUserTimeLimit", ctypes.c_int64), ("PerJobUserTimeLimit", ctypes.c_int64),
                    ("LimitFlags", wintypes.DWORD), ("MinimumWorkingSetSize", ctypes.c_size_t),
                    ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", wintypes.DWORD),
                    ("Affinity", ctypes.c_size_t), ("PriorityClass", wintypes.DWORD),
                    ("SchedulingClass", wintypes.DWORD)]

    class IO_COUNTERS(ctypes.Structure):
        _fields_ = [(n, ctypes.c_uint64) for n in ("ReadOperationCount", "WriteOperationCount",
                                                    "OtherOperationCount", "ReadTransferCount",
                                                    "WriteTransferCount", "OtherTransferCount")]

    class JOBOBJECT_EXTENDED_LIMIT_INFORMATION(ctypes.Structure):
        _fields_ = [("BasicLimitInformation", JOBOBJECT_BASIC_LIMIT_INFORMATION), ("IoInfo", IO_COUNTERS),
                    ("ProcessMemoryLimit", ctypes.c_size_t), ("JobMemoryLimit", ctypes.c_size_t),
                    ("PeakProcessMemoryUsed", ctypes.c_size_t), ("PeakJobMemoryUsed", ctypes.c_size_t)]
    h = k32.CreateJobObjectW(None, None)
    if not h:
        return None
    info = JOBOBJECT_EXTENDED_LIMIT_INFORMATION()
    info.BasicLimitInformation.LimitFlags = 0x2000          # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
    if not k32.SetInformationJobObject(h, 9, ctypes.byref(info), ctypes.sizeof(info)):   # JobObjectExtendedLimitInformation
        return None
    _job = h
    return h


def dies_with_parent(proc):
    """proc: a subprocess.Popen. -> True when the OS will kill it with this process."""
    if sys.platform != "win32" or proc is None:
        return False
    try:
        import ctypes
        h = _job_handle()
        if h is None:
            return False
        return bool(ctypes.windll.kernel32.AssignProcessToJobObject(h, int(proc._handle)))
    except Exception:
        return False
