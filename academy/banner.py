"""The banner every academy screen shows (CLAUDE.md section S, ACADEMY_SPEC ground rules).

The text lives in lang/el.txt / lang/en.txt (key BANNER_KEY); the constants below are the
owner's exact wording and test_academy.py AC8 checks the lang files against them, so a changed
file can never drop or soften the banner. Rule for every academy screen (steps 16-22): it calls
banner_text() and draws it at the top, always visible, in the theme's CAUTION colour
(groundstation/theme.py: one look); test_academy AC8 fails for an academy module that uses
tkinter without the banner.
"""
import text

BANNER_KEY = "academy.banner"
BANNER_EL = "ΕΚΠΑΙΔΕΥΣΗ – ΠΡΟΣΟΜΟΙΩΣΗ"     # "–" = EN DASH U+2013, as the owner wrote it
BANNER_EN = "TRAINING – SIMULATION"


def banner_text(lang_code=None):
    return text.T(BANNER_KEY, lang_code)
