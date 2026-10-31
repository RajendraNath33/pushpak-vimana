#!/usr/bin/env bash
# Launch ArduCopter SITL (hexacopter) near Haldwani, Uttarakhand.
# Prerequisite: ArduPilot source tree built per https://ardupilot.org/dev/docs/SITL-setup-landing-page.html
set -euo pipefail
cd "${ARDUPILOT_DIR:-$HOME/ardupilot}"
Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --console --map \
  --custom-location=29.2183,79.5130,520,0 \
  --add-param-file="$(dirname "$(realpath "$0")")/../params/pushpak_vimana_baseline.parm"
# Then in MAVProxy:  wp load demo_delivery_outbound.waypoints ; mode auto ; arm throttle
