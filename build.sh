#!/usr/bin/env bash
../Bela/scripts/build_project.sh . \
  --force \
  -c "-N no -G yes --disable-led --stop-button-pin -1 -B 16 --high-performance-mode" \
  -m 'CPPFLAGS="-std=c++17 -I./include" LDLIBS="./lib/liblsl.so"'
