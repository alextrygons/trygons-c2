"""Which monitors this PC has, and moving a window to one of them (section C3).

list_monitors(root) -> [(x0, y0, x1, y1), ...] in Windows' order (the main
monitor first); without the Windows API one entry = the whole screen.
move_to(window, i) puts the window on monitor i, maximised.
Ctrl+Shift+1 .. 9 (bind_keys) moves the window that has the keyboard focus.
The window places are remembered by the app (gs_layout.json).
"""
import sys
import tkinter as tk


def list_monitors(root=None):
    mons = []
    if sys.platform == "win32":
        try:
            import ctypes
            from ctypes import wintypes

            class MONITORINFO(ctypes.Structure):
                _fields_ = [("cbSize", wintypes.DWORD), ("rcMonitor", wintypes.RECT), ("rcWork", wintypes.RECT),
                            ("dwFlags", wintypes.DWORD)]
            user32 = ctypes.windll.user32
            proc_t = ctypes.WINFUNCTYPE(ctypes.c_int, wintypes.HMONITOR, wintypes.HDC, ctypes.POINTER(wintypes.RECT),
                                        wintypes.LPARAM)

            def cb(hmon, _hdc, _rect, _lp):
                mi = MONITORINFO()
                mi.cbSize = ctypes.sizeof(MONITORINFO)
                if user32.GetMonitorInfoW(hmon, ctypes.byref(mi)):
                    r = mi.rcWork
                    entry = (r.left, r.top, r.right, r.bottom)
                    if mi.dwFlags & 1:                          # the main monitor first
                        mons.insert(0, entry)
                    else:
                        mons.append(entry)
                return 1
            user32.EnumDisplayMonitors(0, 0, proc_t(cb), 0)
        except Exception:
            mons = []
    if not mons and root is not None:
        try:
            mons = [(0, 0, root.winfo_screenwidth(), root.winfo_screenheight())]
        except tk.TclError:
            pass
    return mons or [(0, 0, 1920, 1080)]


def monitor_of(win, mons=None):
    """Index of the monitor the window's centre is on."""
    mons = mons or list_monitors(win)
    try:
        cx = win.winfo_rootx() + win.winfo_width() // 2
        cy = win.winfo_rooty() + win.winfo_height() // 2
    except tk.TclError:
        return 0
    for i, (x0, y0, x1, y1) in enumerate(mons):
        if x0 <= cx < x1 and y0 <= cy < y1:
            return i
    return 0


def move_to(win, i, app=None):
    """Window `win` (Tk / Toplevel) onto monitor i, maximised there."""
    if win is None:
        return False
    mons = list_monitors(win)
    if not 0 <= i < len(mons):
        return False
    x0, y0, x1, y1 = mons[i]
    try:
        try:
            win.state("normal")
        except tk.TclError:
            pass
        win.geometry("%dx%d+%d+%d" % (max(400, x1 - x0 - 16), max(300, y1 - y0 - 40), x0, y0))
        win.update_idletasks()
        try:
            win.state("zoomed")
        except tk.TclError:
            pass
    except tk.TclError:
        return False
    if app is not None:
        app.layout_changed()
    return True


def bind_keys(win, app):
    """Ctrl+Shift+<n>: this window onto monitor n."""
    for n in range(1, 10):
        # Shift+digit gives "!", "@", ... on most layouts: bind the key code too
        win.bind("<Control-Shift-Key-%d>" % n, lambda e, n=n: move_to(win, n - 1, app))
        win.bind("<Control-Shift-KP_%d>" % n, lambda e, n=n: move_to(win, n - 1, app))
    win.bind("<Control-Shift-Key>", lambda e: _by_keycode(e, win, app), add="+")


def _by_keycode(e, win, app):
    # keycodes 49..57 = the digit row 1..9 (Windows), whatever the Shift symbol is
    if 49 <= e.keycode <= 57:
        move_to(win, e.keycode - 49, app)
