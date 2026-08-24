#!/usr/bin/env bash
# Перевірка логіки прошивки на хості, без ESP-IDF і без плати.
# IDF-заголовки підмінені заглушками з test/stub — компілюється звичайним g++.
set -e
cd "$(dirname "$0")/.."
OUT=$(mktemp -d)
g++ -std=gnu++17 -O0 -Wall -Wextra -Wno-unused-parameter \
    -I test/stub -I main -o "$OUT/core" test/test_host.cpp main/servo.cpp main/mpu6050.cpp
g++ -std=gnu++17 -O0 -Wall -Wextra -Wno-unused-parameter \
    -I test/stub -I main -o "$OUT/uart" test/test_uart.cpp
"$OUT/core"
echo
"$OUT/uart"
rm -rf "$OUT"
