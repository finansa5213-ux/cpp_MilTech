#!/usr/bin/env python3
"""
make_fixtures.py — генератор ЛОКАЛЬНИХ тестових фікстур для ДЗ15.

ПРИЗНАЧЕННЯ
    Створює набір файлів results/T01…T10/simulation.json, щоб прогнати
    HTTP-клієнт по всіх гілках його логіки без справжнього сервера.

ЦЕ НЕ РЕЗУЛЬТАТИ СИМУЛЯЦІЇ.
    Числа тут згенеровані скриптом, а не обчислені балістичним рушієм.
    Фікстури призначені виключно для мок-сервера (127.0.0.1).
    Для здачі на cppmiltech.com.ua потрібні файли, отримані реальним
    прогоном симулятора на офіційних вхідних даних.

Фікстури навмисно різнорідні — кожна перевіряє свій сценарій:
    T01      не чіпається (там ваш справжній результат)
    T02–T07  валідні набори різного розміру
    T08      частина цілей із valid:false (балістика не розв'язується)
    T09      порожній dropPoints (симуляція без жодної обробленої цілі)
    T10      великий набір — перевірка, що клієнт тягне об'ємне тіло

Окремо створюється broken/T99/simulation.json з навмисно битим JSON —
для перевірки гілки BadLocalFile.

Запуск:
    python3 make_fixtures.py [--dir results] [--force]
"""
import argparse
import json
import math
import os
import random

# Константи, узгоджені з config.json проєкту (altitude=100, attackSpeed=30).
FALL_TIME = 5.749755859375        # час падіння боєприпасу, с
PROJECTILE_REACH = 112.70         # горизонтальний виніс боєприпасу, м
CRUISE_SPEED = 27.5               # усталена швидкість дрона, м/с

# Набори цілей для кожного теста: (x, y) у метрах від старту дрона (0,0).
TARGET_SETS = {
    "T02": [(120.0, 0.0), (180.0, 40.0), (-90.0, 60.0)],
    "T03": [(200.0, 0.0), (150.0, 90.0), (-120.0, 40.0), (60.0, 150.0)],
    "T04": [(250.0, -30.0), (170.0, 70.0)],
    "T05": [(140.0, 20.0), (190.0, -55.0), (-80.0, 95.0), (75.0, 130.0), (210.0, 10.0)],
    "T06": [(160.0, 45.0)],
    "T07": [(130.0, -25.0), (205.0, 35.0), (-110.0, 70.0), (95.0, 140.0), (175.0, -80.0),
            (240.0, 15.0)],
    "T08": [(155.0, 30.0), (185.0, -40.0), (-95.0, 85.0), (110.0, 125.0)],
    "T10": [(x * 1.0, y * 1.0) for x, y in
            [(120, 0), (145, 35), (170, -30), (195, 60), (-85, 90), (60, 140),
             (215, 20), (-130, 45), (100, -110), (230, -50), (80, 165), (250, 5)]],
}


def solve_drop(target):
    """
    Обчислює правдоподібну точку скиду для цілі.

    Модель спрощена: дрон летить із (0,0) прямо на ціль і скидає боєприпас
    за PROJECTILE_REACH метрів до неї. Це відтворює геометрію справжнього
    результату, але НЕ є розрахунком балістичного рушія.
    """
    tx, ty = target
    dist = math.hypot(tx, ty)
    if dist < 1e-6:
        return (0.0, 0.0), 0.0

    fire_dist = max(dist - PROJECTILE_REACH, 0.0)
    ux, uy = tx / dist, ty / dist
    fire = (round(ux * fire_dist, 6), round(uy * fire_dist, 6))
    flight = round(fire_dist / CRUISE_SPEED, 6)
    return fire, flight


def build_simulation(targets, invalid_indices=()):
    """Складає структуру simulation.json у форматі, який пише симулятор."""
    drop_points = []
    for idx, target in enumerate(targets):
        valid = idx not in invalid_indices
        fire, flight = solve_drop(target)
        drop_points.append({
            "fallTime": FALL_TIME if valid else 0.0,
            "firePoint": {"x": fire[0] if valid else 0.0,
                          "y": fire[1] if valid else 0.0},
            "flightTime": flight if valid else 0.0,
            "predictedTarget": {"x": target[0], "y": target[1]},
            "targetIndex": idx,
            "valid": valid,
        })
    return {"dropPoints": drop_points, "processed": len(targets)}


def write_fixture(base_dir, test_id, payload, force):
    """Записує один файл фікстури, не чіпаючи наявний без --force."""
    path = os.path.join(base_dir, test_id, "simulation.json")
    if os.path.exists(path) and not force:
        print(f"  {test_id}: пропуск — файл уже існує (--force щоб перезаписати)")
        return False

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)
    count = len(payload.get("dropPoints", []))
    print(f"  {test_id}: створено ({count} цілей)")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default="results", help="каталог фікстур")
    parser.add_argument("--force", action="store_true",
                        help="перезаписувати наявні файли (T01 не чіпається ніколи)")
    args = parser.parse_args()

    random.seed(30)  # детермінованість: однаковий запуск — однакові файли

    print(f"Генерую фікстури у {args.dir}/\n")
    print("УВАГА: це синтетичні дані для мок-сервера, а не результати симуляції.\n")

    t01 = os.path.join(args.dir, "T01", "simulation.json")
    if os.path.exists(t01):
        print("  T01: недоторканий — там справжній результат симулятора")
    else:
        print("  T01: ВІДСУТНІЙ — покладіть туди simulation.json із прогону симулятора")

    for test_id, targets in sorted(TARGET_SETS.items()):
        # T08 — сценарій із нерозв'язною балістикою для двох цілей.
        invalid = {1, 3} if test_id == "T08" else ()
        write_fixture(args.dir, test_id, build_simulation(targets, invalid), args.force)

    # T09 — порожній результат: симуляція відпрацювала, цілей не оброблено.
    write_fixture(args.dir, "T09", {"dropPoints": [], "processed": 0}, args.force)

    # Окрема фікстура з битим JSON — гілка BadLocalFile у клієнті.
    broken_path = os.path.join("broken", "T99", "simulation.json")
    os.makedirs(os.path.dirname(broken_path), exist_ok=True)
    with open(broken_path, "w", encoding="utf-8") as fh:
        fh.write('{"dropPoints": [{"targetIndex": 0, "valid": true,,, ')
    print(f"\n  broken/T99: створено навмисно битий JSON")

    print("\nГотово. Прогін по мок-серверу:")
    print("  python3 tools/mock_server.py 8080")
    print("  ./homework_15 --student 2009 --host 127.0.0.1 --port 8080")
    print("  ./homework_15 --student 2009 --host 127.0.0.1 --port 8080 \\")
    print("                --results broken --tests T99")


if __name__ == "__main__":
    main()
