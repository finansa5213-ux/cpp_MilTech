#!/bin/sh
# --no-mavproxy: MAVLink виходить напряму з ArduPilot.
# У Rover-4.6.x використовуються --serial0/--serial1; старий --uartA не потрiбен.
#   SERIAL0 -> QGroundControl
#   SERIAL1 -> c2_service
exec /ardupilot/Tools/autotest/sim_vehicle.py \
  --vehicle "${VEHICLE:-Rover}" \
  --no-rebuild \
  --no-mavproxy \
  --speedup "${SPEEDUP:-1}" \
  --custom-location="${LAT:-48.983498065911704},${LON:-37.82220625228142},${ALT:-95},${DIR:-90}" \
  -A "--serial0=udpclient:${GCS_HOST:-127.0.0.1}:${GCS_PORT:-14550} --serial1=udpclient:${C2_HOST:-127.0.0.1}:${C2_PORT:-14551}"
