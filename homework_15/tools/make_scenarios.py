#!/usr/bin/env python3
"""
make_scenarios.py — генератор вхідних сценаріїв T01–T10 для ДЗ15.

Створює scenarios/TXX/{config.json, targets.json, ammo.json}.
Далі run_all.sh проганяє на них симулятор і збирає simulation.json.

ПРИНЦИП ПОБУДОВИ
    Кожен сценарій змінює РІВНО ОДНУ групу параметрів відносно базового
    T01. Це дозволяє приписати будь-яку різницю в результаті конкретній
    причині, а не здогадуватись, що саме подіяло.

    T01        базовий (штатні дані проєкту) — еталон для порівняння
    T02–T04    змінюється тільки боєприпас
    T05–T06    змінюється тільки висота
    T07        змінюється тільки кінематика дрона
    T08        змінюються тільки цілі (рух замість нерухомості)
    T09        змінюється тільки кількість цілей
    T10        граничний випадок: балістика не розв'язується

ЗАМІРЯНІ ХАРАКТЕРИСТИКИ СОЛВЕРА (висота 100 м, швидкість 30 м/с)
    VOG-17        fallTime 5.75 c    валідний
    M67           fallTime 5.40 c    валідний
    RKG-3         fallTime 4.85 c    валідний
    GLIDING-RKG   fallTime 5.36 c    валідний
    GLIDING-VOG   —                  НЕ розв'язується на жодній дистанції

    Причина відмови GLIDING-VOG: у computeTimeOfFlight кубічне рівняння
    дає від'ємний корінь (t ~ -8.03), спрацьовує перевірка `return t > 0`.
    Це не залежить від дистанції до цілі — валідність визначається
    комбінацією маси, опору, підйомної сили, висоти й швидкості.

УВАГА ЩОДО T10
    Сценарій T10 навмисно провалює балістику на ВСІХ цілях. У початковій
    версії MissionProcessor.cpp це спричиняє нескінченний цикл: гілка
    «Більше цілей немає» не інкрементує currentIdx, тому hasNext()
    назавжди лишається true. Перед запуском T10 застосуйте виправлення
    (див. scenarios/README.md).

Запуск:
    python3 make_scenarios.py [--dir scenarios] [--force]
"""
import argparse
import json
import os

AMMO_CATALOG = [
    {"name": "VOG-17",      "mass": 0.35, "drag": 0.07, "lift": 0.0},
    {"name": "M67",         "mass": 0.60, "drag": 0.10, "lift": 0.0},
    {"name": "RKG-3",       "mass": 1.20, "drag": 0.10, "lift": 0.0},
    {"name": "GLIDING-VOG", "mass": 0.45, "drag": 0.10, "lift": 1.0},
    {"name": "GLIDING-RKG", "mass": 1.40, "drag": 0.10, "lift": 1.0},
]

TIME_STEPS = 60
ARRAY_TIME_STEP = 5.0

# П'ять штатних цілей проєкту — спільна база для T01-T07 і T10.
BASE_TARGETS = [(150.0, 0.0), (200.0, 50.0), (-100.0, 80.0),
                (50.0, 120.0), (180.0, -60.0)]


def make_config(ammo="VOG-17", altitude=100.0, attack_speed=30.0,
                accel_path=50.0, angular_speed=1.0):
    """config.json у форматі FileConfigLoader."""
    return {
        "drone": {
            "position": {"x": 0.0, "y": 0.0},
            "altitude": altitude,
            "initialDirection": 0.0,
            "attackSpeed": attack_speed,
            "accelerationPath": accel_path,
            "angularSpeed": angular_speed,
            "turnThreshold": 0.05,
        },
        "ammo": ammo,
        "simulation": {"timeStep": 0.1, "hitRadius": 2.0},
        "targetArrayTimeStep": ARRAY_TIME_STEP,
        "maxSteps": 10000,
    }


def static_track(x, y):
    """Нерухома ціль: усі позиції треку однакові -> velocity = 0."""
    return [{"x": float(x), "y": float(y)} for _ in range(TIME_STEPS)]


def moving_track(x0, y0, vx, vy):
    """
    Рухома ціль зі швидкістю (vx, vy) м/с.

    Провайдер рахує швидкість як різницю сусідніх точок треку, поділену
    на targetArrayTimeStep. Тому крок між точками = v * ARRAY_TIME_STEP.
    """
    return [{"x": round(x0 + vx * ARRAY_TIME_STEP * i, 4),
             "y": round(y0 + vy * ARRAY_TIME_STEP * i, 4)}
            for i in range(TIME_STEPS)]


def targets_doc(tracks):
    return {"targetCount": len(tracks), "timeSteps": TIME_STEPS,
            "targets": [{"positions": t} for t in tracks]}


def base_targets_doc():
    return targets_doc([static_track(x, y) for x, y in BASE_TARGETS])


def build_scenarios():
    """Опис сценаріїв: що змінено, навіщо, що очікується."""
    return {
        "T01": {
            "changed": "нічого - штатні дані проєкту",
            "purpose": "Еталон. Відносно нього інтерпретуються всі інші "
                       "результати. Має збігатися з simulation.json, який "
                       "дає симулятор на файлах з data/.",
            "expect": "5 цілей оброблено, усі valid, fallTime 5.75 c",
            "config": make_config(),
            "targets": base_targets_doc(),
        },
        "T02": {
            "changed": "боєприпас: VOG-17 -> M67",
            "purpose": "Ізолює вплив маси й опору на балістику. M67 майже "
                       "вдвічі важчий (0.60 проти 0.35 кг) і має більший "
                       "опір (0.10 проти 0.07).",
            "expect": "той самий набір цілей, але fallTime 5.40 c - "
                      "падає швидше, точки скиду зміщені",
            "config": make_config(ammo="M67"),
            "targets": base_targets_doc(),
        },
        "T03": {
            "changed": "боєприпас: VOG-17 -> RKG-3",
            "purpose": "Найважчий неперативний боєприпас (1.20 кг). "
                       "Продовжує ряд T01-T02-T03 за зростанням маси, "
                       "даючи монотонну залежність часу падіння.",
            "expect": "fallTime 4.85 c - найкоротший серед неперативних",
            "config": make_config(ammo="RKG-3"),
            "targets": base_targets_doc(),
        },
        "T04": {
            "changed": "боєприпас: VOG-17 -> GLIDING-RKG (lift = 1.0)",
            "purpose": "Вмикає гілку розрахунку з підйомною силою: у "
                       "computeHorizontalDistance коефіцієнти C3-C5 "
                       "залежать від lift, при lift=0 вони вироджуються.",
            "expect": "fallTime 5.36 c, помітно інший горизонтальний виніс",
            "config": make_config(ammo="GLIDING-RKG"),
            "targets": base_targets_doc(),
        },
        "T05": {
            "changed": "висота: 100 -> 150 м",
            "purpose": "Ізолює вплив висоти скиду. Висота входить у кубічне "
                       "рівняння через вільний член c = 6*m^2*Z0. Обрано 150 м "
                       "як максимум, де розрахунок ще математично коректний: "
                       "від ~180 м аргумент арккосинуса виходить за [-1;1], "
                       "обрізається, і fallTime насичується фальшивим 10.0 с.",
            "expect": "fallTime 8.23 c (проти 5.75 у T01), точки скиду далі "
                      "від цілей",
            "config": make_config(altitude=150.0),
            "targets": base_targets_doc(),
        },
        "T06": {
            "changed": "висота: 100 -> 50 м",
            "purpose": "Протилежний край діапазону висот. Малий виніс "
                       "означає, що дрон має підлітати майже впритул.",
            "expect": "короткий fallTime, точки скиду близько до цілей",
            "config": make_config(altitude=50.0),
            "targets": base_targets_doc(),
        },
        "T07": {
            "changed": "кінематика: швидкість 30 -> 45 м/с, кутова 1.0 -> 2.0, "
                       "розгін 50 -> 30 м",
            "purpose": "Балістика змінюється (V0 входить у коефіцієнти), але "
                       "головне навантаження - на стейт-машину: швидші "
                       "повороти й коротший розгін змінюють кількість тіків "
                       "у станах TURNING та ACCELERATING.",
            "expect": "ті самі цілі, але суттєво менше тіків до ATTACK",
            "config": make_config(attack_speed=45.0, angular_speed=2.0,
                                  accel_path=30.0),
            "targets": base_targets_doc(),
        },
        "T08": {
            "changed": "цілі: нерухомі -> рухомі",
            "purpose": "Вмикає екстраполяцію: predicted = pos + velocity * "
                       "fallTime. При нерухомих цілях цей доданок нульовий, "
                       "тож увесь блок прогнозу лишався неперевіреним.",
            "expect": "predictedTarget помітно відрізняється від початкової "
                      "позиції цілі - на v * 5.75 метрів",
            "config": make_config(),
            "targets": targets_doc([
                moving_track(150.0, 0.0, 8.0, 2.0),      # віддаляється
                moving_track(200.0, 50.0, -6.0, 4.0),    # наближається
                moving_track(-100.0, 80.0, 0.0, -10.0),  # поперечний рух
                moving_track(50.0, 120.0, 5.0, 5.0),     # діагональ
            ]),
        },
        "T09": {
            "changed": "кількість цілей: 5 -> 9, розкидані по всіх напрямках",
            "purpose": "Навантажує вибір наступної цілі й повороти: цілі "
                       "розставлені так, що дрон мусить розвертатися на "
                       "великі кути, а не летіти по дузі в один бік.",
            "expect": "найбільший simulation.json; багато переходів "
                      "ACCELERATING - TURNING",
            "config": make_config(),
            "targets": targets_doc([
                static_track(130.0, -25.0), static_track(205.0, 35.0),
                static_track(-110.0, 70.0), static_track(95.0, 140.0),
                static_track(175.0, -80.0), static_track(240.0, 15.0),
                static_track(-60.0, -95.0), static_track(45.0, 185.0),
                static_track(-180.0, -40.0),
            ]),
        },
        "T10": {
            "changed": "боєприпас: VOG-17 -> GLIDING-VOG",
            "purpose": "Граничний випадок: балістика не розв'язується "
                       "ЖОДНОГО разу. Перевіряє обробку помилок - "
                       "DroneErrorCode::BALLISTICS_FAIL, recoverFromError() "
                       "і коректний вихід, коли цілей більше немає.",
            "expect": "усі dropPoints із valid=false; програма завершується "
                      "штатно. УВАГА: без виправлення currentIdx у "
                      "recoverFromError() тут нескінченний цикл",
            "config": make_config(ammo="GLIDING-VOG"),
            "targets": base_targets_doc(),
        },
    }


def write_scenario(base_dir, test_id, spec, force):
    path = os.path.join(base_dir, test_id)
    marker = os.path.join(path, "config.json")
    if os.path.exists(marker) and not force:
        print(f"  {test_id}: пропуск - уже існує (--force щоб перезаписати)")
        return False

    os.makedirs(path, exist_ok=True)
    for name, payload in (("config.json", spec["config"]),
                          ("targets.json", spec["targets"]),
                          ("ammo.json", AMMO_CATALOG)):
        with open(os.path.join(path, name), "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2, ensure_ascii=False)

    # Опис поруч зі сценарієм - щоб не шукати його в іншому файлі.
    with open(os.path.join(path, "README.txt"), "w", encoding="utf-8") as fh:
        fh.write(f"{test_id}\n{'=' * len(test_id)}\n\n")
        fh.write(f"Змінено відносно T01:\n  {spec['changed']}\n\n")
        fh.write(f"Призначення:\n  {spec['purpose']}\n\n")
        fh.write(f"Очікуваний результат:\n  {spec['expect']}\n")
    return True


def write_index(base_dir, scenarios):
    """Зведена таблиця всіх сценаріїв - для звіту."""
    path = os.path.join(base_dir, "README.md")
    with open(path, "w", encoding="utf-8") as fh:
        fh.write("# Тестові сценарії T01-T10 (ДЗ15)\n\n")
        fh.write("Кожен сценарій змінює рівно одну групу параметрів "
                 "відносно базового T01,\nщоб різницю в результаті можна "
                 "було приписати конкретній причині.\n\n")
        fh.write("| Тест | Що змінено | Очікуваний результат |\n")
        fh.write("|---|---|---|\n")
        for tid in sorted(scenarios):
            s = scenarios[tid]
            changed = s["changed"].replace("\n", " ")
            expect = s["expect"].split(";")[0].split(" - ")[0]
            fh.write(f"| {tid} | {changed} | {expect} |\n")

        fh.write("\n## Детальні описи\n\n")
        for tid in sorted(scenarios):
            s = scenarios[tid]
            fh.write(f"### {tid}\n\n")
            fh.write(f"**Змінено:** {s['changed']}\n\n")
            fh.write(f"**Призначення:** {s['purpose']}\n\n")
            fh.write(f"**Очікування:** {s['expect']}\n\n")

        fh.write("## Необхідне виправлення перед запуском T10\n\n")
        fh.write("У `homework_08/src/MissionProcessor.cpp`, функція "
                 "`recoverFromError()`,\nгілка «цілей більше немає» не "
                 "інкрементує лічильник, через що\n`hasNext()` назавжди "
                 "лишається `true` і головний цикл не завершується:\n\n")
        fh.write("```cpp\n")
        fh.write("    if (start >= total) {\n")
        fh.write('        std::cerr << "  [SM] Більше цілей немає після '
                 'пропуску #"\n')
        fh.write('                  << currentIdx << "\\n";\n')
        fh.write("        currentIdx = total;   // <- додати цей рядок\n")
        fh.write("        drone.stop();\n")
        fh.write("        return false;\n")
        fh.write("    }\n")
        fh.write("```\n\n")
        fh.write("Баг виявляється лише коли балістика провалюється на "
                 "останній цілі,\nтому на штатних даних проєкту він не "
                 "проявлявся.\n")
    print(f"\nЗведену таблицю записано: {path}")


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dir", default="scenarios")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()

    scenarios = build_scenarios()
    print(f"Генерую сценарії у {args.dir}/\n")

    created = 0
    for tid in sorted(scenarios):
        spec = scenarios[tid]
        if write_scenario(args.dir, tid, spec, args.force):
            print(f"  {tid}: {spec['changed']}")
            created += 1

    write_index(args.dir, scenarios)
    print(f"Створено сценаріїв: {created}")
    print("\nДалі:")
    print("  bash tools/run_all.sh ../homework_08/homework_08")


if __name__ == "__main__":
    main()
