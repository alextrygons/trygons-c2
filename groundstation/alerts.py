"""Alarm levels for the screens and the sounds (sections A4 / D1).

The boat's alarm bits (proto.ALARMS) are sorted into
  WARNING   red, MASTER WARNING lamp, repeating urgent tone + voice until acknowledged
  CAUTION   amber, MASTER CAUTION lamp, single chime + voice once
  ADVISORY  no lamp, soft sound / short voice
Each one has an event code (language-neutral, used in the logs, the sound
files and the language files: "alarm.<code>" is its screen text,
"say.<code>" its voice callout).

DISPLAY ONLY: acknowledging a lamp here silences this PC; it never sends
anything to the boat (the boat's ALARM ACK stays the captain's button).
"""
import i18n
import proto

WARNING, CAUTION, ADVISORY, FEEDBACK = 3, 2, 1, 0
LEVEL_NAMES = {WARNING: "WARNING", CAUTION: "CAUTION", ADVISORY: "ADVISORY", FEEDBACK: "FEEDBACK"}

# bit -> (event code, level)
ALARM_LEVELS = {
    0x001: ("compass_lost", WARNING),
    0x002: ("gps_lost", CAUTION),
    0x004: ("gps_suspect", CAUTION),
    0x008: ("dr_limit", CAUTION),
    0x010: ("arrival_uncertain", CAUTION),
    0x020: ("arrived", ADVISORY),
    0x040: ("gear_fault", WARNING),
    0x080: ("training", ADVISORY),
    0x100: ("joystick_fault", WARNING),
    0x200: ("joystick_not_centred", CAUTION),
    0x400: ("joystick_took_control", ADVISORY),
    0x800: ("pc_link_lost", CAUTION),
    0x1000: ("not_neutral_at_start", CAUTION),
    0x2000: ("monitor_tripped", WARNING),
    0x4000: ("loop_overrun", CAUTION),
    0x8000: ("log_failure", CAUTION),
    0x10000: ("gateway_lost", CAUTION),
    0x20000: ("gateway_storage", CAUTION),
    0x40000: ("remote_link_lost", CAUTION),
    0x80000: ("channel_fallback", CAUTION),
    0x100000: ("remote_driving_link_lost", WARNING),
    0x200000: ("slamming", CAUTION),
    0x400000: ("ais_silent", ADVISORY),
    0x800000: ("collision_risk", WARNING),
    0x1000000: ("collision_decision", WARNING),
    0x2000000: ("shallow_water", WARNING),
    0x4000000: ("obstacle_ahead", CAUTION),
    0x8000000: ("emergency_stop", WARNING),
    0x10000000: ("profile", CAUTION),
    0x20000000: ("link_lost", WARNING),
    0x40000000: ("engine_killed", WARNING),
}
NO_DATA = "no_data"                 # the PC gets no telemetry (not an alarm bit): WARNING


def code(bit):
    return ALARM_LEVELS.get(bit, ("bit_%x" % bit, CAUTION))[0]


def level(bit):
    return ALARM_LEVELS.get(bit, ("", CAUTION))[1]


def active(bits):
    """[(bit, code, level)] of the bits set, most severe first."""
    out = [(bit, code(bit), level(bit)) for bit, _name in proto.ALARMS if bits & bit]
    out.sort(key=lambda a: -a[2])
    return out


def text(bit):
    """The alarm in the screen language."""
    return i18n.T("alarm." + code(bit))


def texts(bits):
    return [i18n.T("alarm." + c) for _b, c, _l in active(bits)]


def lamp_bits(bits, lvl):
    m = 0
    for bit, _c, lv in active(bits):
        if lv == lvl:
            m |= bit
    return m


class MasterLamps:
    """MASTER WARNING / MASTER CAUTION: on while there is an alarm of that level
    the driver has not acknowledged yet. A new alarm lights the lamp again.
    Acknowledging (click / key) only switches the lamp off and silences the
    repeating sound on this PC."""

    def __init__(self):
        self.acked = {WARNING: 0, CAUTION: 0}
        self.bits = 0
        self.no_data = False
        self.no_data_acked = False

    def update(self, bits, no_data=False):
        self.bits = bits
        for lv in (WARNING, CAUTION):
            self.acked[lv] &= lamp_bits(bits, lv)          # an alarm that went away may come back later
        if not no_data:
            self.no_data_acked = False
        self.no_data = no_data

    def on(self, lv):
        if lv == WARNING and self.no_data and not self.no_data_acked:
            return True
        return bool(lamp_bits(self.bits, lv) & ~self.acked[lv])

    def ack(self):
        for lv in (WARNING, CAUTION):
            self.acked[lv] = lamp_bits(self.bits, lv)
        if self.no_data:
            self.no_data_acked = True

    def unacked_codes(self, lv):
        out = [c for b, c, l_ in active(self.bits) if l_ == lv and not self.acked[lv] & b]
        if lv == WARNING and self.no_data and not self.no_data_acked:
            out.insert(0, NO_DATA)
        return out


def result_text(d):
    """An ACK's text in the screen language (code -> "result.<n>"); an extra part the
    gateway added (": why", " (age ...)") is kept."""
    r = d.get("result")
    base = proto.RESULTS.get(r) or GW_RESULTS_EN.get(r)
    key = "result.%s" % r
    if not i18n.has(key, "en"):
        return d.get("text", "")
    rest = ""
    t = d.get("text") or ""
    if base and t.startswith(base):
        rest = t[len(base):]
        if rest.startswith(": "):
            rest = ": " + i18n.from_en(rest[2:])            # the gateway's reason, in the screen language too
    return i18n.T(key) + rest


GW_RESULTS_EN = {109: "driver screen not open", 110: "rejected by the driver",
                 111: "no answer from the driver (timeout)", 112: "driver screen lost",
                 113: "single-operator mode: confirm on the captain screen first", 114: "shallow water on the route",
                 116: "no manual licence: REMOTE MANUAL / MANEUVER only for a licensed captain",
                 117: "only the captain (or a station on the boat) holds HELM",
                 118: "captain only: REMOTE MANUAL / MANEUVER", 119: "no captain holds HELM",
                 120: "the route leaves the geofence"}
