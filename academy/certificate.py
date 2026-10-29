"""The instructor view's certificate PDF (ACADEMY_SPEC A "certificate PDF", deferred from step 16 to step 21).

One A4 page drawn with Pillow (already used by the ground station's HUD; no new package) and saved as a PDF:
the name, the user, the level IN FORCE and its expiry, the practical test (date, examiner, kind), the exams
passed, the lessons / scenarios done, the driving hours and the currency, the issuing instructor and the date.
Only an instructor's 2FA session (accounts.Store.session_ok); every certificate is logged (kind "certificate").
A level that is not in force (expired / OBSERVER) is printed as such - a certificate never claims more than the
logbook. No network module here (test_academy AC1).
"""
import datetime
import os

import accounts
import simguard

A4 = (1240, 1754)                       # 150 dpi
FONT_FILES = (("segoeui.ttf", "segoeuib.ttf"), ("DejaVuSans.ttf", "DejaVuSans-Bold.ttf"), ("arial.ttf", "arialbd.ttf"))
FONT_DIRS = (os.path.join(os.environ.get("WINDIR", r"C:\Windows"), "Fonts"), "/usr/share/fonts/truetype/dejavu",
             "/usr/share/fonts/TTF", "/usr/share/fonts/dejavu")


def _font(size, bold=False):
    from PIL import ImageFont
    for pair in FONT_FILES:
        name = pair[1 if bold else 0]
        for d in FONT_DIRS:
            p = os.path.join(d, name)
            if os.path.isfile(p):
                return ImageFont.truetype(p, size)
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def facts(store, user, today=None):
    """What the certificate says (from the account + logbook only)."""
    today = today or datetime.date.today()
    a = store.accounts[user]
    row = {r["user"]: r for r in store.summary(today)}[user]
    log = a.logbook
    prac = [e for e in log if e.get("kind") in ("practical", "check_ride") and e.get("passed")]
    exams = [e for e in log if e.get("kind") == "exam" and e.get("passed")]
    return {"user": user, "name": a.name or user, "level": a.level, "effective": a.effective_level(today),
            "expiry": a.expiry.isoformat() if a.expiry and a.effective_level(today) != "OBSERVER" else None,
            "practical": [(str(e.get("t", ""))[:10], e.get("kind"), e.get("by"), e.get("target")) for e in prac],
            "exams": [(str(e.get("t", ""))[:10], e.get("id"), e.get("score")) for e in exams],
            "lessons": row["lessons"], "scenarios": len({e.get("id") for e in log if e.get("kind") == "scenario"
                                                          and e.get("done")}),
            "hours": row["hours"], "current": row["current"], "last_drive": row["last_drive"],
            "date": today.isoformat()}


def make(store, user, path, session, T, today=None):
    """Writes the PDF to path; -> path. T = the academy's text function (the screen language)."""
    why = store.session_ok(session, role="instructor", today=today)
    if why:
        raise simguard.Refused(why)
    if not str(path).lower().endswith(".pdf"):
        raise ValueError("a .pdf file name")
    from PIL import Image, ImageDraw
    f = facts(store, user, today)
    img = Image.new("RGB", A4, "white")
    dr = ImageDraw.Draw(img)
    W = A4[0]
    dr.rectangle((40, 40, W - 40, A4[1] - 40), outline="#1f3b57", width=6)
    y = 110
    dr.text((W // 2, y), T("cert.title"), fill="#1f3b57", font=_font(64, True), anchor="mm")
    y += 80
    dr.text((W // 2, y), T("cert.academy"), fill="#333333", font=_font(32), anchor="mm")
    y += 110
    dr.text((W // 2, y), f["name"], fill="#000000", font=_font(56, True), anchor="mm")
    y += 60
    dr.text((W // 2, y), T("cert.user", user=user), fill="#555555", font=_font(28), anchor="mm")
    y += 110
    lv = T("level.%s" % f["effective"])
    dr.text((W // 2, y), T("cert.level", level=lv), fill="#1f3b57", font=_font(48, True), anchor="mm")
    y += 60
    dr.text((W // 2, y), T("cert.expiry", expiry=f["expiry"] or "-") if f["expiry"] else T("cert.not_in_force"),
            fill="#000000" if f["expiry"] else "#c0392b", font=_font(32), anchor="mm")
    y += 100
    body = _font(28)
    lines = []
    for d, kind, by, target in f["practical"]:
        lines.append(T("cert.practical", date=d, kind=T("pt.kind_%s" % ("upgrade" if kind == "practical" else kind)),
                       by=by or "-", level=T("level.%s" % target) if target in accounts.LEVELS else "-"))
    for d, eid, score in f["exams"]:
        lines.append(T("cert.exam", date=d, id=eid, score=score))
    lines.append(T("cert.lessons", n=f["lessons"], s=f["scenarios"]))
    lines.append(T("cert.hours", h=f["hours"], last=f["last_drive"] or "-",
                   current=T("acc.yes") if f["current"] else T("acc.no")))
    for ln in lines[:22]:
        dr.text((120, y), ln, fill="#000000", font=body)
        y += 44
    y = A4[1] - 260
    dr.text((120, y), T("cert.issued", by=session.user, date=f["date"]), fill="#000000", font=body)
    dr.line((W - 520, y + 30, W - 120, y + 30), fill="#000000", width=2)
    dr.text((W - 320, y + 60), T("cert.signature"), fill="#555555", font=_font(24), anchor="mm")
    dr.text((W // 2, A4[1] - 110), T("cert.note"), fill="#777777", font=_font(22), anchor="mm")
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    img.save(path, "PDF", resolution=150.0)
    store.log(user, "certificate", by=session.user, level=f["effective"], expiry=f["expiry"],
              file=os.path.basename(path))
    return path
