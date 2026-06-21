# Домашнє завдання 8 — Структура проєкту, STL-контейнери, DroneStateMachine

## Зміст

- [Задача](#задача)
- [Що змінено відносно ДЗ7](#що-змінено-відносно-дз7)
- [Структура файлів](#структура-файлів)
- [Архітектура](#архітектура)
- [DroneStateMachine](#dronetatemachine)
- [Відновлення після помилки](#відновлення-після-помилки)
- [Збірка та запуск](#збірка-та-запуск)
- [Вхідні дані](#вхідні-дані)
- [Приклад виводу](#приклад-виводу)

---

## Задача

1. Привести структуру проєкту до ладу — розкласти `.h` і `.cpp` по підпапках.
2. Замінити C-стильові конструкції на STL-контейнери скрізь де це доречно.
3. Додати явну стейт машину руху дрона (`DroneStateMachine`) — замість
   фіктивної константної позиції дрон тепер реально симулює підліт до цілі.

---

## Що змінено відносно ДЗ7

### Структура проєкту

| ДЗ7                    | ДЗ8                                                               |
|---                     |---                                                                |
| Плоский `include/`     | Підпапки `interfaces/` `solvers/` `providers/` `config/` `drone/` |
| Плоский `src/`         | Підпапки `solvers/` `providers/` `config/` `drone/`               |
| `Factory.h/.cpp`       | Перейменовано → `ComponentFactory.h/.cpp`                         |
| `cxx_std_14`           | Підвищено → `cxx_std_17`                                          |
| Попередження лише MSVC | Додано `-Wall -Wextra -Wpedantic` для GCC/Clang                   |

### Заміна C-стилю на STL

| Файл                                     | Було                                           | Стало                             |
|---                                       |---                                             |---                                |
| `Types.h`                                | `char name[32]` в `AmmoParams`                 | `std::string`                     |
| `Types.h`                                | `char ammoName[32]` в `MissionConfig`          | `std::string`                     |
| `JsonTargetProvider`                     | `Coord**` + `new[]/delete[]` + `freeTracks()`  | `std::vector<std::vector<Coord>>` |
| `JsonTargetProvider`                     | `char filename_[256]`                          | `std::string`                     |
| `FileConfigLoader`                       | `char configPath_[256]`, `char ammoPath_[256]` | `std::string`                     |
| `FileConfigLoader::loadAmmoFile()`       | лінійний цикл + `strcmp`                       | `std::unordered_map` + `.find()`  |
| `FileConfigLoader`, `JsonTargetProvider` | `for (int i=0; i<n; ++i)`                      | `for (const auto& item : j)`      |
| `ComponentFactory`                       | `strchr` + `memcpy`                            | `std::string::find` + `substr`    |
| `main.cpp`                               | `snprintf` для param-рядка                     | `std::string` + `std::to_string`  |

### MissionProcessor

| ДЗ7                                      | ДЗ8                                                        |
|---                                       |---                                                         |
| `dronePos_` = константа `cfg_.startPos`  | `drone_.data().pos` — реальна позиція після симуляції      |
| `step()` = один виклик `solver->solve()` | `step()` = цикл `tick()` до стану `ATTACK` або `ERROR`     |
| При помилці: `++currentIdx_`, далі       | При помилці: `findNearestNextTarget()` → найближча ціль    | 
| Немає лічильника пропусків               | `skippedTargets_` + геттер `skippedTargets()`              |
| `dp.totalTime` = час падіння бомби       | `dp.totalTime` = час підльоту; `dp.fallTime` = час падіння |

---

## Структура файлів

```
homework_08/
├── CMakeLists.txt
├── include/
│   ├── Types.h                          # Coord, AmmoParams, Target, DropPoint, MissionConfig
│   ├── MissionProcessor.h               # оркестратор місії зі стейт машиною
│   ├── json.hpp                         # nlohmann/json (single-header)
│   ├── interfaces/
│   │   ├── IBallisticSolver.h           # абстракція балістичного солвера
│   │   ├── IConfigLoader.h              # абстракція завантажувача конфігу
│   │   └── ITargetProvider.h            # абстракція провайдера цілей
│   ├── drone/
│   │   └── DroneStateMachine.h          # ★ явна стейт машина дрона
│   ├── solvers/
│   │   └── AnalyticalSolver.h           # Кардано + ряд Тейлора до t^5
│   ├── providers/
│   │   └── JsonTargetProvider.h         # цілі з targets.json
│   └── config/
│       ├── FileConfigLoader.h           # config.json + ammo.json
│       └── ComponentFactory.h           # фабричні функції
└── src/
    ├── main.cpp
    ├── MissionProcessor.cpp
    ├── drone/
    │   └── DroneStateMachine.cpp        # ★ реалізація переходів
    ├── solvers/
    │   └── AnalyticalSolver.cpp
    ├── providers/
    │   └── JsonTargetProvider.cpp
    └── config/
        ├── FileConfigLoader.cpp
        └── ComponentFactory.cpp
```

---

## Архітектура

```
main()
 ├─ createLoader()    → FileConfigLoader   (IConfigLoader)
 ├─ createProvider()  → JsonTargetProvider (ITargetProvider)
 ├─ createSolver()    → AnalyticalSolver   (IBallisticSolver)
 └─ MissionProcessor(loader, provider, solver)
       │
       ├─ DroneStateMachine drone_    ← ★ новий компонент
       │
       └─ step()
            ├─ 1. solver->solve()         → firePoint (балістика)
            ├─ 2. drone_.retarget()       → передаємо ціль машині
            ├─ 3. while tick() != ATTACK  → дрон летить
            │       TURNING → ACCELERATING → CRUISE → ATTACK
            └─ 4. dp.firePoint = drone_.pos  (реальна точка скиду)
```

---

## DroneStateMachine

### Стани

| Стан              | Опис                                                                    |
|---                |---                                                                      |
| `NOT_INITIALIZED` | до виклику `init()`                                                     |
| `TURNING`         | повертається до курсу на `firePoint` з кутовою швидкістю `angularSpeed` |
| `ACCELERATING`    | лінійний розгін від 0 до `attackSpeed` на відрізку `accelPath`          |
| `CRUISE`          | постійна крейсерська швидкість `attackSpeed`                            |
| `ATTACK`          | дрон у точці скиду — термінальний, `tick()` нічого не робить            |
| `STOP`            | зупинений ззовні через `stop()`                                         |
| `ERROR`           | помилка, причина в `DroneStateData::errorCode`                          |

### Діаграма переходів

```
NOT_INITIALIZED
      │ init()
      ▼
   TURNING ◄──────────────────────────────────────────┐
      │ |кут| <= turnThreshold                        │  кут збився
      ▼                                               │
ACCELERATING ───────────────────────────────────────►─┤
      │ speed >= 95% attackSpeed                      │  кут збився
      ▼                                               │
   CRUISE ──────────────────────────────────────────►─┘
      │ dist(pos, firePoint) <= hitRadius
      ▼
   ATTACK  ← термінальний

  Будь-який → ERROR   якщо stepCount > maxSteps
  Будь-який → STOP    через stop() ззовні
```

### DroneErrorCode

| Код               | Причина                                  |
|---                |---                                       |
| `NONE`            | помилки немає                            |
| `TIMEOUT`         | `stepCount > maxSteps` — дрон не долетів |
| `BALLISTICS_FAIL` | `solver->solve()` повернув `valid=false` |
| `INVALID_TARGET`  | `firePoint` збігається з `dronePos`      |
| `TURN_STALL`      | зарезервовано                            |

---

## Відновлення після помилки

При переході в `ERROR` `MissionProcessor` не зупиняє місію, а викликає
`findNearestNextTarget()` — знаходить індекс цілі з мінімальною відстанню
від поточної позиції дрона і переходить до неї:

```
ERROR виникла
  └─► findNearestNextTarget()
        ├─ перебирає всі цілі від currentIdx+1
        ├─ знаходить min distance(dronePos, target.pos)
        ├─ currentIdx_ = bestIdx        ← може перестрибнути кілька
        ├─ drone_.retarget(nextTarget)  ← машина одразу летить туди
        └─ якщо цілей немає → drone_.stop()
```

Лог при відновленні:
```
  [SM] Ціль #2 [TIMEOUT] stepCount=10001 > maxSteps=10000
       — шукаємо найближчу наступну
  [SM] Пропускаємо #2 → переходимо до найближчої цілі #4
       (dist=38.2 м, пропущено проміжних: 1)
```

---

## Збірка та запуск

```bash
# з кореня репозиторію
cmake -B build
cmake --build build --target homework_08
cd build && ./homework_08
```

Або напряму через g++:

```bash
cd homework_08
g++ -std=c++17 -Iinclude \
  src/main.cpp src/MissionProcessor.cpp \
  src/drone/DroneStateMachine.cpp \
  src/solvers/AnalyticalSolver.cpp \
  src/providers/JsonTargetProvider.cpp \
  src/config/FileConfigLoader.cpp \
  src/config/ComponentFactory.cpp \
  -o homework_08
```

---

## Вхідні дані

Три JSON-файли у `data/` (CMake копіює їх поруч із бінарником):

**`config.json`**
```json
{
  "drone": {
    "position": { "x": 0.0, "y": 0.0 },
    "altitude": 150.0,
    "initialDirection": 0.0,
    "attackSpeed": 20.0,
    "accelerationPath": 50.0,
    "angularSpeed": 0.5,
    "turnThreshold": 0.1
  },
  "simulation": { "timeStep": 0.05, "hitRadius": 5.0 },
  "targetArrayTimeStep": 1.0,
  "ammo": "VOG-17",
  "maxSteps": 10000
}
```

**`ammo.json`**
```json
[{ "name": "VOG-17", "mass": 0.28, "drag": 0.003, "lift": 0.1 }]
```

---

## Приклад виводу

```
[main] Конфіг: ammo=VOG-17  altitude=150.000  attackSpeed=20.000
[main] Боєприпас: VOG-17  m=0.280  d=0.003  l=0.100
[main] Цілей: 3

=== Обробка цілей ===

  [SM] Ціль #0 firePoint=(87.4, 183.2)  dronePos=(0.0, 0.0)  стан=TURNING
    [SM] TURNING → ACCELERATING  t=3.15 с  pos=(6.3, 2.1)  spd=2.0 м/с
    [SM] ACCELERATING → CRUISE   t=5.80 с  pos=(44.2, 91.1)  spd=20.0 м/с
    [SM] CRUISE → ATTACK         t=9.12 с  pos=(87.1, 183.0)  spd=20.0 м/с
  [SM] → ATTACK за 183 тіків (9.12 с)

  ✓ Ціль #0  fire=(87.1, 183.0)  pred=(102.1, 205.6)
             підліт=9.12 с  падіння=5.23 с

[main] Оброблено: 3 цілей  |  пропущено через помилки: 0
[main] Результат → simulation.json
```
