# Tour narration (Greek): clip <-> tour step

Source: `voice_raw/` (local, not in git). Conversion: `tools/make_tour_voice.py`, Opus/OGG 64 kbit/s mono, no trim / normalise / other processing (the 44.1 kHz audio is resampled to Opus' 48 kHz inside the encoder). Files: `academy/tour_voice/el/` + `manifest.json`. Clip number = the stop's number in `academy/tour.py` `stops('el')`. Clips 085-126 are tour narration of the alarms (stops 85-86 lamps, 87-126 alarm.*), not the live alarm callouts.

Played by the tour in Greek; a stop without a (matching) clip uses the synthetic Greek voice; English always uses the synthetic English voice.

| # | file | duration s | tour step id | first 3 words match |
|---|------|-----------:|--------------|----|
| 001 | 001_intro.welcome.ogg | 23.92 | intro.welcome | yes |
| 002 | 002_intro.safety.ogg | 31.98 | intro.safety | yes |
| 003 | 003_s1.overview.ogg | 19.46 | s1.overview | yes |
| 004 | 004_s1.status.ogg | 26.95 | s1.status | yes |
| 005 | 005_s1.master.ogg | 19.99 | s1.master | yes |
| 006 | 006_s1.mirror_l.ogg | 17.48 | s1.mirror_l | yes |
| 007 | 007_s1.mirror_r.ogg | 13.17 | s1.mirror_r | yes |
| 008 | 008_s1.alarms.ogg | 23.94 | s1.alarms | yes |
| 009 | 009_s1.notices.ogg | 16.13 | s1.notices | yes |
| 010 | 010_s1.throttle.ogg | 23.04 | s1.throttle | yes |
| 011 | 011_s1.speed.ogg | 11.60 | s1.speed | yes |
| 012 | 012_s1.depth.ogg | 16.30 | s1.depth | yes |
| 013 | 013_s1.waypoint.ogg | 12.79 | s1.waypoint | yes |
| 014 | 014_s1.traffic.ogg | 21.62 | s1.traffic | yes |
| 015 | 015_s1.rot.ogg | 11.52 | s1.rot | yes |
| 016 | 016_s1.gmeter.ogg | 11.82 | s1.gmeter | yes |
| 017 | 017_s1.minimap.ogg | 13.40 | s1.minimap | yes |
| 018 | 018_s1.steer.ogg | 16.14 | s1.steer | yes |
| 019 | 019_s1.cdi.ogg | 26.47 | s1.cdi | yes |
| 020 | 020_s1.relays.ogg | 16.60 | s1.relays | yes |
| 021 | 021_s1.bottom.ogg | 33.08 | s1.bottom | yes |
| 022 | 022_s1.save.ogg | 13.62 | s1.save | yes |
| 023 | 023_s1.logo.ogg | 11.11 | s1.logo | yes |
| 024 | 024_s1.tape.ogg | 16.65 | s1.tape | yes |
| 025 | 025_s1.btn_lang.ogg | 12.09 | s1.btn_lang | yes |
| 026 | 026_s1.btn_night.ogg | 12.79 | s1.btn_night | yes |
| 027 | 027_s1.btn_declutter.ogg | 16.57 | s1.btn_declutter | yes |
| 028 | 028_s1.btn_scale.ogg | 13.74 | s1.btn_scale | yes |
| 029 | 029_s1.btn_ar.ogg | 16.59 | s1.btn_ar | yes |
| 030 | 030_s1.btn_layout.ogg | 25.98 | s1.btn_layout | yes |
| 031 | 031_s1.btn_text_size.ogg | 19.36 | s1.btn_text_size | yes |
| 032 | 032_s1.btn_scr.ogg | 35.99 | s1.btn_scr | yes |
| 033 | 033_s1.btn_ptt.ogg | 34.54 | s1.btn_ptt | yes |
| 034 | 034_s1.btn_obstacles.ogg | 19.51 | s1.btn_obstacles | yes |
| 035 | 035_s1.btn_clear_slowdown.ogg | 21.51 | s1.btn_clear_slowdown | yes |
| 036 | 036_s1.btn_slam_restore.ogg | 28.33 | s1.btn_slam_restore | yes |
| 037 | 037_s1.drive.ogg | 39.29 | s1.drive | yes |
| 038 | 038_s1.btn_maneuver.ogg | 37.19 | s1.btn_maneuver | yes |
| 039 | 039_s1.accept.ogg | 61.05 | s1.accept | yes |
| 040 | 040_s2.map.ogg | 21.47 | s2.map | yes |
| 041 | 041_s2.modebar.ogg | 35.38 | s2.modebar | yes |
| 042 | 042_s2.battle_short.ogg | 10.00 | s2.battle_short | yes |
| 043 | 043_s2.status.ogg | 49.84 | s2.status | yes |
| 044 | 044_s2.alarms.ogg | 24.67 | s2.alarms | yes |
| 045 | 045_s2.helm.ogg | 16.17 | s2.helm | yes |
| 046 | 046_s2.link.ogg | 16.98 | s2.link | yes |
| 047 | 047_s2.control.ogg | 17.56 | s2.control | yes |
| 048 | 048_s2.standby.ogg | 17.73 | s2.standby | yes |
| 049 | 049_s2.traffic.ogg | 11.31 | s2.traffic | yes |
| 050 | 050_s2.relays.ogg | 17.35 | s2.relays | yes |
| 051 | 051_s2.banners.ogg | 16.68 | s2.banners | yes |
| 052 | 052_s2.bar1.ogg | 43.73 | s2.bar1 | yes |
| 053 | 053_s2.bar2.ogg | 27.80 | s2.bar2 | yes |
| 054 | 054_s2.camview.ogg | 37.14 | s2.camview | yes |
| 055 | 055_s2.tracks.ogg | 47.64 | s2.tracks | yes |
| 056 | 056_s2.screenrec.ogg | 31.79 | s2.screenrec | yes |
| 057 | 057_s2.loudhailer.ogg | 25.89 | s2.loudhailer | yes |
| 058 | 058_s2.edit.ogg | 17.78 | s2.edit | yes |
| 059 | 059_tools.connection.ogg | 12.18 | tools.connection | yes |
| 060 | 060_tools.control.ogg | 12.83 | tools.control | yes |
| 061 | 061_tools.program.ogg | 12.70 | tools.program | yes |
| 062 | 062_tools.ais.ogg | 11.36 | tools.ais | yes |
| 063 | 063_tools.plot.ogg | 9.69 | tools.plot | yes |
| 064 | 064_tools.config.ogg | 12.29 | tools.config | yes |
| 065 | 065_tools.joystick.ogg | 11.30 | tools.joystick | yes |
| 066 | 066_tools.cameras.ogg | 8.92 | tools.cameras | yes |
| 067 | 067_tools.remote.ogg | 12.58 | tools.remote | yes |
| 068 | 068_tools.network.ogg | 10.65 | tools.network | yes |
| 069 | 069_tools.system.ogg | 9.17 | tools.system | yes |
| 070 | 070_tools.bench.ogg | 14.19 | tools.bench | yes |
| 071 | 071_tools.engine.ogg | 14.50 | tools.engine | yes |
| 072 | 072_tools.sound.ogg | 27.24 | tools.sound | yes |
| 073 | 073_tools.loudhailer.ogg | 51.61 | tools.loudhailer | yes |
| 074 | 074_tools.uav.ogg | 33.08 | tools.uav | yes |
| 075 | 075_tools.events.ogg | 8.89 | tools.events | yes |
| 076 | 076_mode.standby.ogg | 15.50 | mode.standby | yes |
| 077 | 077_mode.manual.ogg | 12.65 | mode.manual | yes |
| 078 | 078_mode.hdg_lock.ogg | 66.08 | mode.hdg_lock | yes |
| 079 | 079_mode.auto.ogg | 50.11 | mode.auto | yes |
| 080 | 080_mode.training.ogg | 10.33 | mode.training | yes |
| 081 | 081_mode.remote.ogg | 20.15 | mode.remote | yes |
| 082 | 082_mode.bench.ogg | 10.95 | mode.bench | yes |
| 083 | 083_mode.maneuver.ogg | 15.66 | mode.maneuver | yes |
| 084 | 084_mode.dp.ogg | 19.02 | mode.dp | yes |
| 085 | 085_lamp.warning.ogg | 15.58 | lamp.warning | yes |
| 086 | 086_lamp.caution.ogg | 19.32 | lamp.caution | yes |
| 087 | 087_alarm.compass_lost.ogg | 19.64 | alarm.compass_lost | yes |
| 088 | 088_alarm.gps_lost.ogg | 13.92 | alarm.gps_lost | yes |
| 089 | 089_alarm.gps_suspect.ogg | 19.06 | alarm.gps_suspect | yes |
| 090 | 090_alarm.dr_limit.ogg | 17.94 | alarm.dr_limit | yes |
| 091 | 091_alarm.arrival_uncertain.ogg | 14.60 | alarm.arrival_uncertain | yes |
| 092 | 092_alarm.arrived.ogg | 8.95 | alarm.arrived | yes |
| 093 | 093_alarm.gear_fault.ogg | 15.15 | alarm.gear_fault | yes |
| 094 | 094_alarm.training.ogg | 9.88 | alarm.training | yes |
| 095 | 095_alarm.joystick_fault.ogg | 16.09 | alarm.joystick_fault | yes |
| 096 | 096_alarm.joystick_not_centred.ogg | 17.83 | alarm.joystick_not_centred | yes |
| 097 | 097_alarm.joystick_took_control.ogg | 12.37 | alarm.joystick_took_control | yes |
| 098 | 098_alarm.pc_link_lost.ogg | 14.68 | alarm.pc_link_lost | yes |
| 099 | 099_alarm.not_neutral_at_start.ogg | 16.68 | alarm.not_neutral_at_start | yes |
| 100 | 100_alarm.monitor_tripped.ogg | 19.66 | alarm.monitor_tripped | yes |
| 101 | 101_alarm.loop_overrun.ogg | 10.81 | alarm.loop_overrun | yes |
| 102 | 102_alarm.log_failure.ogg | 18.41 | alarm.log_failure | yes |
| 103 | 103_alarm.gateway_lost.ogg | 14.13 | alarm.gateway_lost | yes |
| 104 | 104_alarm.gateway_storage.ogg | 11.09 | alarm.gateway_storage | yes |
| 105 | 105_alarm.remote_link_lost.ogg | 16.21 | alarm.remote_link_lost | yes |
| 106 | 106_alarm.channel_fallback.ogg | 9.90 | alarm.channel_fallback | yes |
| 107 | 107_alarm.remote_driving_link_lost.ogg | 20.70 | alarm.remote_driving_link_lost | yes |
| 108 | 108_alarm.slamming.ogg | 26.54 | alarm.slamming | yes |
| 109 | 109_alarm.ais_silent.ogg | 13.01 | alarm.ais_silent | yes |
| 110 | 110_alarm.collision_risk.ogg | 17.91 | alarm.collision_risk | yes |
| 111 | 111_alarm.collision_decision.ogg | 16.83 | alarm.collision_decision | yes |
| 112 | 112_alarm.shallow_water.ogg | 12.81 | alarm.shallow_water | yes |
| 113 | 113_alarm.obstacle_ahead.ogg | 13.26 | alarm.obstacle_ahead | yes |
| 114 | 114_alarm.emergency_stop.ogg | 16.72 | alarm.emergency_stop | yes |
| 115 | 115_alarm.profile.ogg | 16.51 | alarm.profile | yes |
| 116 | 116_alarm.link_lost.ogg | 28.33 | alarm.link_lost | yes |
| 117 | 117_alarm.engine_killed.ogg | 18.02 | alarm.engine_killed | yes |
| 118 | 118_alarm.low_battery.ogg | 14.72 | alarm.low_battery | yes |
| 119 | 119_alarm.relay_silent.ogg | 13.71 | alarm.relay_silent | yes |
| 120 | 120_alarm.dp_heat.ogg | 14.11 | alarm.dp_heat | yes |
| 121 | 121_alarm.dp_offspot.ogg | 17.03 | alarm.dp_offspot | yes |
| 122 | 122_alarm.dp_overpower.ogg | 14.81 | alarm.dp_overpower | yes |
| 123 | 123_alarm.can_node.ogg | 19.15 | alarm.can_node | yes |
| 124 | 124_alarm.can_bus.ogg | 16.10 | alarm.can_bus | yes |
| 125 | 125_alarm.geofence.ogg | 17.32 | alarm.geofence | yes |
| 126 | 126_alarm.no_data.ogg | 15.12 | alarm.no_data | yes |
| 127 | 127_joy.x.ogg | 16.11 | joy.x | yes |
| 128 | 128_joy.y.ogg | 19.30 | joy.y | yes |
| 129 | 129_joy.take.ogg | 11.89 | joy.take | yes |
| 130 | 130_joy.auto.ogg | 12.26 | joy.auto | yes |
| 131 | 131_joy.confirm.ogg | 15.94 | joy.confirm | yes |
| 132 | 132_joy.usb.ogg | 21.75 | joy.usb | yes |
| 133 | 133_panel.standby.ogg | 17.40 | panel.standby | yes |
| 134 | 134_panel.hdg_lock.ogg | 12.05 | panel.hdg_lock | yes |
| 135 | 135_panel.minus1.ogg | 9.50 | panel.minus1 | yes |
| 136 | 136_panel.plus1.ogg | 9.37 | panel.plus1 | yes |
| 137 | 137_panel.minus10.ogg | 9.12 | panel.minus10 | yes |
| 138 | 138_panel.plus10.ogg | 13.48 | panel.plus10 | yes |
| 139 | 139_panel.estop.ogg | 17.38 | panel.estop | yes |
| 140 | 140_uav.page.ogg | 23.38 | uav.page | yes |
| 141 | 141_uav.scope.ogg | 19.28 | uav.scope | yes |
| 142 | 142_uav.route.ogg | 31.76 | uav.route | yes |
| 143 | 143_uav.launch.ogg | 31.17 | uav.launch | yes |
| 144 | 144_uav.stop.ogg | 27.63 | uav.stop | yes |
| 145 | 145_uav.video.ogg | 27.93 | uav.video | yes |
| 146 | 146_uav.battery.ogg | 24.26 | uav.battery | yes |
| 147 | 147_uav.fly.ogg | 31.41 | uav.fly | yes |
| 148 | 148_uav.logs.ogg | 17.20 | uav.logs | yes |
| 149 | 149_uav.legal.ogg | 29.38 | uav.legal | yes |

## Tour steps without a clip

none

## Clips without a step

none

## Mismatches (first three words differ; clip NOT placed)

none
