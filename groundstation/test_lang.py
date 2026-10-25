"""Language tests (section E): Greek + English screen texts.

  L1 el.txt and en.txt: both parse, the same keys, the same {names} in every entry
  L2 every key the program uses exists in both files (static keys in the code +
     the families made at run time: modes, alarms, results, gears ...)
  L3 no hard-coded screen text in the screen modules (text="...", say("..."))
  L4 live switch ΕΛ <-> EN without restart: buttons, tabs, titles follow; no key
     missing while drawing; the captain screen fits 1920 px in both languages
  L5 command descriptions / gateway sentences travel in English (protocol,
     audit) and are shown in the screen language (from_en round trip)

Run:  python test_lang.py        (exit code 0 = all passed)
"""
import os
import re
import string
import sys
import tempfile
import tkinter as tk

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
os.environ["TRYGONS_GS_SETTINGS"] = os.path.join(tempfile.gettempdir(), "trygons_gs_test_lang.json")
os.environ.pop("TRYGONS_LANG", None)
os.environ.setdefault("TRYGONS_SOUND", "off")                    # silent: no sound, no voice making
import i18n  # noqa: E402
import alerts  # noqa: E402
import proto  # noqa: E402

results = []
SCREEN_MODULES = ["app.py", "mapview.py", "driver.py", "hud.py", "hudar.py", "mapwin.py", "soundtest.py", "monitors.py",
                  "sound.py"]


def report(name, ok, detail):
    results.append(ok)
    print("[%s] %s - %s" % ("PASS" if ok else "FAIL", name, detail), flush=True)


def names(text):
    return sorted({f[1] for f in string.Formatter().parse(text) if f[1]})


def code_keys():
    """Static keys used in the screen modules."""
    keys = set()
    for fn in os.listdir(HERE):
        if not fn.endswith(".py") or fn.startswith("test_"):
            continue
        s = open(os.path.join(HERE, fn), encoding="utf-8").read()
        keys |= set(re.findall(r'''(?:\bT|\bTE|i18n\.T|i18n\.TE)\(\s*"([a-zA-Z0-9_.]+)"''', s))
        keys |= set(re.findall(r'''(?:\blbl|\bbtn|\bchk|i18n\.bind)\([^"\n]*?"([a-z]+\.[a-zA-Z0-9_.]+)"''', s))
        keys |= set(k for k in re.findall(r'''"((?:cap|tab|menu|map|ais|drv|hud|what|wiz|joy|bench|plot|snd|say|ar|lay|mw)'''
                                          r'''\.[a-zA-Z0-9_]+)"''', s) if not k.endswith("_"))
    return {k for k in keys if not k.endswith((".", "_"))}       # "mode." + name: a family, checked below


def families():
    """Keys made at run time (T("mode." + name) ...)."""
    f = ["mode." + m for m in proto.MODES]
    f += ["alarm." + c for c, _l in alerts.ALARM_LEVELS.values()] + ["alarm." + alerts.NO_DATA]
    f += ["result.%d" % r for r in list(proto.RESULTS) + list(alerts.GW_RESULTS_EN)]
    f += ["gear.%d" % i for i in range(3)] + ["pos.%d" % i for i in range(4)]
    f += ["dr.%d" % i for i in range(len(proto.DR_REASONS))] + ["auto.%d" % i for i in range(4)]
    f += ["train.stage_%d" % i for i in range(3)] + ["train.phase_%d" % i for i in range(11)]
    f += ["train.result_%d" % i for i in range(12)] + ["confirm.%d" % i for i in range(1, len(proto.CONFIRM_KINDS))]
    f += ["joy.st_%d" % i for i in range(4)] + ["joywhy.%d" % i for i in proto.JOY_WHY]
    f += ["jdev.%d" % i for i in range(2)] + ["jhat.%d" % i for i in range(4)]
    f += ["jm.%d" % i for i in range(len(proto.JM_BUTTONS))] + ["btn.%d" % i for i in range(6)]
    f += ["wiz.hint_%d" % i for i in range(len(proto.JM_BUTTONS))]
    f += ["prog.end_%d" % i for i in range(len(proto.PROG_END))] + ["wp.act_%d" % i for i in range(len(proto.WP_ACTIONS))]
    f += ["prog.err_%d" % i for i in range(len(proto.PROG_ERRORS))] + ["cap.imu_acc_%d" % i for i in range(4)]
    f += ["bench.refuse_%d" % i for i in range(1, 4)] + ["cap.rm_state_%d" % i for i in range(3)]
    f += ["cap.rm_why_%d" % i for i in range(4)] + ["cap.pw_head_fwd", "cap.pw_head_rev"]
    f += ["cap.joy_src_%d" % i for i in range(2)] + ["cap.thr_in_%d" % i for i in range(2)]
    f += ["cap.v_" + k for k in ("heading", "target", "yaw", "outboard", "throttle", "gear", "gps", "pos", "sog", "src",
                                 "unc", "drift", "current", "cruise", "wp", "train")]
    f += ["cap.btn_manual", "cap.btn_hdg_lock", "cap.btn_auto", "cap.sj_steer", "cap.sj_thr"]
    f += ["cap.vis_stage%d" % i for i in (1, 2, 3)] + ["cap.jm_" + k for k in ("nz_lo", "nz_hi", "lever_delta",
                                                                              "pickup_win")]
    f += ["cap.pg_" + k for k in ("lat", "lon", "kn", "radius", "action")]
    f += ["cap.param_" + k for k in ("value", "min", "max")] + ["map.layer_" + k for k in ("depth", "contours", "marks",
                                                                                       "ais")]
    f += ["menu.lang_" + k for k in i18n.LANGS] + ["menu.move_" + k for k in ("captain", "driver", "map")]
    f += ["cap.sim_b_" + k for k in ("take", "auto", "hdg", "stby", "m1", "p1")]
    f += ["cap.net_col_" + k for k in ("channel", "state", "internet", "latency", "loss", "today", "month", "signal")]
    f += ["vis." + k for k in ("ACTIVE", "PAUSED", "DEGRADED", "OFFLINE")]
    f += ["avoid." + k for k in ("CLEAR", "SLOW", "AVOID", "RETURN", "PROPOSE", "ALARM_ONLY", "DRIVER_DECISION")]
    try:
        import hud
        f += hud.dynamic_keys()
    except ImportError:
        pass
    try:
        import sound
        f += sound.dynamic_keys()
    except ImportError:
        pass
    try:
        import mapwin
        f += ["mw.theme_" + t for t in mapwin.THEME_ORDER]
    except ImportError:
        pass
    return f


HARD = re.compile(r'''(?:\btext=|\.say\(|\.event\(|\.title\(|askyesno\(|label=)\s*"([^"]*[A-Za-zͰ-Ͽ]{3,}[^"]*)"''')
ALLOWED = {"Segoe UI", "Consolas"}


def main():
    # L1
    el, en = i18n.load("el"), i18n.load("en")
    only_el, only_en = sorted(set(el) - set(en)), sorted(set(en) - set(el))
    bad_names = [k for k in el if k in en and names(el[k]) != names(en[k])]
    report("L1 el.txt and en.txt: same keys, same {names}", not only_el and not only_en and not bad_names and len(en) > 500,
           "%d entries; only in el %s; only in en %s; different {names} %s" % (len(en), only_el[:5], only_en[:5],
                                                                                bad_names[:5]))
    # L2
    need = sorted(code_keys() | set(families()))
    miss_el = [k for k in need if k not in el]
    miss_en = [k for k in need if k not in en]
    report("L2 every key the program uses is in both files", not miss_el and not miss_en,
           "%d keys used; missing in el %s; missing in en %s" % (len(need), miss_el[:8], miss_en[:8]))
    # L3
    hard = []
    for fn in SCREEN_MODULES:
        p = os.path.join(HERE, fn)
        if not os.path.exists(p):
            continue
        for n, line in enumerate(open(p, encoding="utf-8"), 1):
            for m in HARD.finditer(line):
                if m.group(1) not in ALLOWED and not re.fullmatch(r"[a-z0-9_.%]+", m.group(1)):     # event codes
                    hard.append("%s:%d %r" % (fn, n, m.group(1)[:40]))
    report("L3 no hard-coded screen text in the screen modules", not hard, "%d found %s" % (len(hard), hard[:6]))

    # L4 live switch in the real captain screen
    import app as A
    root = tk.Tk()
    errors = []
    root.report_callback_exception = lambda *a: errors.append(repr(a[1]))
    ap = A.App(root, layout=os.path.join(tempfile.gettempdir(), "trygons_layout_test_lang.json"))
    for _ in range(10):
        root.update()
    lang0 = i18n.lang()
    seen = {}
    for lang in ("el", "en", "el"):
        i18n.set_lang(lang)
        for _ in range(5):
            root.update()
        seen.setdefault(lang, []).append((root.title(), ap.conn_btn.cget("text"), ap.nb.tab(0, "text"),
                                          ap.mode_btns["MANUAL"].cget("text"), root.winfo_reqwidth()))
    import settings
    settings.reload()
    remembered = settings.get("ui_lang") == "el"
    el_t, en_t = seen["el"][0], seen["en"][0]
    fit = all(v[4] <= 1920 for vs in seen.values() for v in vs)
    ok = lang0 == "el" and el_t[0] != en_t[0] and en_t[1] == "Connect" and el_t[1] == "Σύνδεση" and \
        en_t[2] == "Map" and el_t[3] == "ΧΕΙΡΟΚΙΝΗΤΟ" and seen["el"][1][:4] == el_t[:4] and remembered and fit and \
        not i18n.MISSING and not errors
    report("L4 Greek by default; live switch EL <-> EN (titles, buttons, tabs); remembered; fits 1920 px in both",
           ok, "default %s; el %s; en %s; widths el %s en %s; remembered %s; missing %s; errors %s" % (
               lang0, el_t[:4], en_t[:4], [v[4] for v in seen["el"]], [v[4] for v in seen["en"]], remembered,
               sorted(i18n.MISSING)[:5], errors[:2]))
    ap.quit()

    # L5 English in the protocol, the screen language on the screen
    i18n.set_lang("el", save=False)
    bad = []
    n = 0
    for k, tpl in en.items():
        if not k.startswith(("what.", "gw.")):
            continue
        vals = {nm: "X%d" % i for i, nm in enumerate(names(tpl))}
        eng = i18n.TE(k, **vals)
        n += 1
        if i18n.from_en(eng) != i18n.T(k, **vals):
            bad.append(k)
    sample = i18n.from_en(i18n.TE("what.heading", d="+10"))
    report("L5 command / gateway sentences: English in the protocol and audit, the screen language on the screen",
           not bad and sample == "πορεία +10",
           "%d sentences round-tripped, wrong %s; 'heading +10' -> '%s'" % (n, bad[:5], sample))
    bad_n = results.count(False)
    print("LANGUAGE TESTS: %d passed, %d failed" % (len(results) - bad_n, bad_n))
    return 1 if bad_n else 0


if __name__ == "__main__":
    sys.exit(main())
