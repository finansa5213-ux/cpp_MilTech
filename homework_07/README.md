# homework_07 — рефакторинг ДЗ№3 у класи з інтерфейсами

## Архітектура
- 3 інтерфейси: `ITargetProvider`, `IBallisticSolver`, `IConfigLoader`
- 3 реалізації: `JsonTargetProvider`, `AnalyticalSolver`, `FileConfigLoader`
- Фабрика з `enum class` створює реалізації за типом
- `MissionProcessor` — клас алгоритму, працює через вказівники
  на інтерфейси (патерн Стратегія)

## Збірка
З кореня репозиторію:
    cmake -B build
    cmake --build build --target homework_07

## Запуск
    cd build/homework_07
    ./homework_07

Виходить `simulation.json` із точками скиду для всіх цілей з `data/targets.json`.