#!/usr/bin/env bash
# ============================================================
# run_all.sh — прогін симулятора по всіх сценаріях T01…T10.
#
# Для кожного сценарію:
#   1. підкладає його config.json / targets.json / ammo.json
#   2. запускає симулятор
#   3. складає отриманий simulation.json у results/TXX/
#
# Результати — справжні: числа рахує ваш балістичний рушій.
#
# Використання:
#   ./run_all.sh [шлях_до_бінарника] [каталог_сценаріїв] [каталог_результатів]
#
# Типово:
#   бінарник   ../homework_08/homework_08
#   сценарії   scenarios
#   результати results
# ============================================================
set -u

SIM_BIN="${1:-../homework_08/homework_08}"
SCEN_DIR="${2:-scenarios}"
OUT_DIR="${3:-results}"

# --- Перевірки перед стартом --------------------------------
if [[ ! -x "$SIM_BIN" ]]; then
    echo "ПОМИЛКА: симулятор не знайдено або не виконуваний: $SIM_BIN" >&2
    echo "Підказка: спочатку зберіть проєкт (cmake --build --preset debug)" >&2
    exit 1
fi

if [[ ! -d "$SCEN_DIR" ]]; then
    echo "ПОМИЛКА: немає каталогу сценаріїв: $SCEN_DIR" >&2
    echo "Підказка: python3 tools/make_scenarios.py" >&2
    exit 1
fi

# Абсолютний шлях — бо далі буде cd у каталог симулятора.
SIM_BIN="$(realpath "$SIM_BIN")"
SCEN_DIR="$(realpath "$SCEN_DIR")"
mkdir -p "$OUT_DIR"
OUT_DIR="$(realpath "$OUT_DIR")"

# Симулятор читає вхідні файли з поточного каталогу і туди ж пише
# результат, тому працюємо саме в теці, де лежить бінарник.
SIM_DIR="$(dirname "$SIM_BIN")"

echo "Симулятор:  $SIM_BIN"
echo "Сценарії:   $SCEN_DIR"
echo "Результати: $OUT_DIR"
echo

ok=0
failed=0

for scen in "$SCEN_DIR"/*/; do
    test_id="$(basename "$scen")"

    if [[ ! -f "$scen/config.json" ]]; then
        echo "  $test_id: пропуск — немає config.json"
        continue
    fi

    # Підкладаємо вхідні дані сценарію поруч із бінарником.
    cp "$scen/config.json" "$scen/targets.json" "$scen/ammo.json" "$SIM_DIR/" 2>/dev/null

    # Прибираємо старий результат, щоб не сплутати з новим.
    rm -f "$SIM_DIR/simulation.json"

    # Запуск. Вивід симулятора ховаємо — він довгий; при збої покажемо.
    log="$(cd "$SIM_DIR" && "$SIM_BIN" 2>&1)"
    rc=$?

    if [[ $rc -ne 0 ]] || [[ ! -f "$SIM_DIR/simulation.json" ]]; then
        echo "  $test_id: ЗБІЙ (код $rc)"
        echo "$log" | tail -5 | sed 's/^/          /'
        ((failed++))
        continue
    fi

    mkdir -p "$OUT_DIR/$test_id"
    cp "$SIM_DIR/simulation.json" "$OUT_DIR/$test_id/simulation.json"

    # Коротка довідка: скільки цілей оброблено.
    processed="$(grep -o '"processed"[^,}]*' "$OUT_DIR/$test_id/simulation.json" | head -1)"
    size="$(wc -c < "$OUT_DIR/$test_id/simulation.json")"
    echo "  $test_id: готово  ($processed, $size байт)"
    ((ok++))
done

echo
echo "Успішно: $ok   Збоїв: $failed"

if [[ $ok -gt 0 ]]; then
    echo
    echo "Далі — відправка на сервер:"
    echo "  ./homework_15 --student <ВАШ_ID>"
fi

exit 0
