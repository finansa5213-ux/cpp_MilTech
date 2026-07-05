# Домашнє завдання 9 — Розумні вказівники, патерн State, табличний солвер

> Проєкт розвиває попереднє ДЗ (структура проєкту + STL + `DroneStateMachine`).
> У цьому ДЗ виконано три незалежні завдання: переведення на розумні
> вказівники, реалізація патерна **State** для стейт-машини дрона та
> додавання **табличного балістичного калькулятора** з 5-вимірною
> інтерполяцією.

---

## Стисло: що було змінено

### 1. Розумні вказівники (`std::unique_ptr`)

- Фабрика (`ComponentFactory`) повертає `std::unique_ptr<I...>` замість
  голих вказівників, об'єкти створюються через `std::make_unique`.
- `MissionProcessor` тепер **володіє** компонентами
  (`loader_`, `provider_`, `solver_`) як `std::unique_ptr`; конструктор
  приймає їх через `std::move()`.
- `changeSolver()` приймає `std::unique_ptr<IBallisticSolver>`.
- Усі ручні `new` / `delete` прибрано (зокрема з `main.cpp`) — пам'ять
  звільняється автоматично.
- Невласницький доступ — через `.get()` (наприклад, `dynamic_cast` для
  виклику `loadFromFile()`).

### 2. Патерн State замість `switch/case`

- `enum DroneState` і весь `switch/case` у `DroneStateMachine` **видалено
  повністю**.
- Додано базовий клас `IDroneState` з методом
  `std::unique_ptr<IDroneState> execute(DroneContext&)`.
- Кожен стан — окремий клас:
  - активні: `StateTurning`, `StateAccelerating`, `StateCruise`;
  - термінальні: `StateAttack`, `StateStop`, `StateError`.
- Спільні дані (позиція, швидкість, курс, конфіг, ціль) винесено в
  структуру `DroneContext`, що передається між станами за посиланням.
- Перехід між станами — `return std::make_unique<NextState>()`;
  `nullptr` означає «стан не змінився».
- Поведінку стейт-машини збережено повністю — змінено лише форму
  (класи замість `switch`), а не логіку переходів.

### 3. Табличний балістичний солвер (`TableSolver`)

- Нова реалізація інтерфейсу `IBallisticSolver`, зареєстрована у фабриці
  як `SolverType::TABLE`.
- Дані зберігаються у структурі `BallisticTable` — 5-вимірна сітка
  (осі `Z0`, `V0`, `m`, `d`, `l` з нерівномірним кроком) у вигляді
  плоского `std::vector` з індексацією `index(iz,iv,im,id,il)`.
- Для значень між вузлами — **багатовимірна лінійна інтерполяція**
  (вкладені `lerp`, згортання гіперкуба 32 -> 16 -> 8 -> 4 -> 2 -> 1).
- Для значень за межами таблиці — `clamp` до крайнього вузла.
- Завантаження таблиці — з текстового файлу `data/ballistic_table.txt`.
- Параметри боєприпасів у `data/ammo.json` оновлено згідно з умовою.

---

## Параметри боєприпасів (оновлені)

| Боєприпас     | m    | d     | l     |
|---            |---   |---    |---    |
| VOG-17        | 0.35 | 0.004 | 0     |
| M67           | 0.60 | 0.005 | 0     |
| RKG-3         | 1.20 | 0.007 | 0     |
| GLIDING-VOG   | 0.45 | 0.005 | 0.005 |
| GLIDING-RKG   | 1.40 | 0.007 | 0.005 |

---

## Структура файлів (ключові зміни)

```
include/
  config/ComponentFactory.h      # фабрика -> unique_ptr, додано SolverType::TABLE
  drone/DroneStateMachine.h      # IDroneState + класи станів + DroneContext
  solvers/TableSolver.h          # BallisticTable + TableSolver  (новий)
  MissionProcessor.h             # компоненти у власності (unique_ptr)
src/
  config/ComponentFactory.cpp    # make_unique
  drone/DroneStateMachine.cpp    # execute() кожного стану (замість handle*)
  solvers/TableSolver.cpp        # load() + lookup() з 5D-інтерполяцією  (новий)
  MissionProcessor.cpp           # std::move у конструкторі, isTerminal()/stateName()
  main.cpp                       # без new/delete, демонстрація changeSolver()->TABLE
data/
  ammo.json                      # оновлені m/d/l
  ballistic_table.txt            # 5D-таблиця (осі: 4*4*5*3*2 = 480 вузлів)  (новий)
```

---

## Формат файлу таблиці `ballistic_table.txt`

```
nZ nV nM nD nL                 # розміри по кожній осі
<nZ значень осі Z0>
<nV значень осі V0>
<nM значень осі m>
<nD значень осі d>
<nL значень осі l>
<t hDist>  ...                 # nZ*nV*nM*nD*nL пар; порядок Z0->V0->m->d->l
```

> Якщо викладач надає власну таблицю — достатньо замінити цей файл,
> формат збігається. Поточний файл згенеровано з аналітичної моделі,
> тому у вузлах сітки `TableSolver` дає той самий результат, що й
> `AnalyticalSolver` (перевірка коректності інтерполяції).

---

## Збірка та запуск

```bash
cmake -S . -B build
cmake --build build
cd build
./homework_09          # на Windows: homework_09.exe
```

Якщо `cmake` недоступний, можна зібрати напряму:

```bash
g++ -std=c++17 -Wall -Wextra -Wpedantic -Iinclude \
    src/*.cpp src/config/*.cpp src/drone/*.cpp \
    src/solvers/*.cpp src/providers/*.cpp -o homework_09
```

Програма обробляє цілі аналітичним солвером, демонструє `reset()`,
а потім повторює прохід із табличним солвером через
`changeSolver(createSolver(SolverType::TABLE))`.
