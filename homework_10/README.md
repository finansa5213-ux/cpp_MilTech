# ДЗ10 — Багатопоточність: цілі, фізика та місія в окремих потоках

Проєкт розносить симуляцію на **три потоки** (`std::thread`):

| Потік | Клас | Період | Роль |
|---|---|---|---|
| 1 | `ThreadSafeTargetProvider` | `arrayTimeStep` | рух цілей, віддає копії `Target{pos, velocity}` |
| 2 | `DronePhysics` | `physicsTimeStep` | єдиний інтегратор стану дрона; приймає команди, віддає телеметрію |
| 3 | `MissionProcessor` | `simTimeStep` | вибір цілі, балістика, стейт-машина-планувальник, лог |

## Синхронізація

- Увесь спільний стан — під `std::mutex` (телеметрія дрона, знімок цілей) або
  `std::atomic` (стоп/готовність/старт-прапорці).
- Команди фізиці — через власний шаблонний `ThreadSafeQueue<DroneCommand>`
  (`mutex` + `condition_variable`).
- Потоки завершуються лише через `join()`, без `detach()`. `main()` чекає
  тільки на потік місії; провайдер і фізика зупиняються стоп-прапорцями.
- Цілі рушають лише за `start()` — після того, як усі `isThreadReady()`
  підтвердили готовність (немає розсинхронізації на старті).

## Розподіл ролей рішення/інтеграції

`MissionProcessor` **не зберігає і не інтегрує** стан дрона. Стейт-машина
(класи станів з ДЗ9) лишається в ньому як **планувальник**: щокроку вона
синхронізується з телеметрії (`syncKinematics`) і вирішує лише *режим*
(`DroneMode`). Саме інтегрування виконує `DronePhysics`.

> Примітка щодо `DroneCommand`: умова фіксує поля `{state, angleSpeed}`. Оскільки
> у ДЗ9 `enum DroneState` прибрали (патерн State), введено легкий `enum DroneMode`,
> а команда додатково несе `firePoint` і параметри сегмента, щоб фізика могла
> інтегрувати кінематику ДЗ9 (поворот до цілі, розгін за дистанцією).

## timeSecSinceStart (важливо для чекера)

Кроки потоків розкладені в реальному часі й **не рівномірні** (планувальник ОС,
`sleep`). Тому `DroneTelemetry` несе `timeSecSinceStart` — модельний час
*останнього оновлення фізики* (сума `dt` усіх фізичних кроків), який знімається
**під тим самим м'ютексом**, що й позиція. `MissionProcessor` пише це значення
в кожен крок `simulation.json`. Без цього поля чекер вважав би кроки
рівномірними (`крок × simTimeStep`) і позиції розійшлися б із часом.

Перевірено: інтервали між кроками реально різні (0.07…0.10 с), `timeSecSinceStart`
монотонно зростає.

## Параметри конфігурації (`config.json` → `simulation`)

- `physicsTimeStep` (дефолт 0.01) — крок інтегрування фізики.
- `targetTimeStep` (дефолт 0.05) — інформативний крок провайдера.
- `timeScale` (дефолт 1.0) — прискорення часу: потік спить `dt / timeScale`.
- `simTimeStep` — наявний (читається з `simulation.timeStep`), `arrayTimeStep` —
  наявний (`targetArrayTimeStep`). Відсутні параметри беруться з дефолтів.

## Збірка та запуск

```bash
cmake -S . -B build
cmake --build build
cd build && ./homework_10

# Перевірка гонок (Linux/WSL):
cmake -S . -B build-tsan -DUSE_TSAN=ON
cmake --build build-tsan && ./build-tsan/homework_10
```

Прямою компіляцією:

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
    src/*.cpp src/config/*.cpp src/drone/*.cpp \
    src/solvers/*.cpp src/providers/*.cpp -pthread -o homework_10
```

## Вихідний файл `simulation.json`

Масив `steps`; кожен крок: `position`, `direction`, `state` (поточний
`DroneMode`), `targetIndex`, `dropPoint`, `aimPoint`, `predictedTarget` і
`timeSecSinceStart`.
