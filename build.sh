#!/usr/bin/env bash
# this arg has moved to settings.json
#-c "--board Bela --analog-channels 0 --digital-channels 16 --verbose --mute-speaker yes --use-analog no --use-digital yes --disable-led --stop-button-pin -1 --period 16 --high-performance-mode" \
../Bela/scripts/build_project.sh \
  -p bela-lsl-timing \
  --force \
  -m "CPPFLAGS='-std=c++17 -I./include' LDLIBS=/root/Bela/projects/bela-lsl-timing/lib/liblsl.so" \
  ./
