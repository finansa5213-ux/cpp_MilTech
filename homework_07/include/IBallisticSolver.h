#pragma once
// ============================================================
// IBallisticSolver — абстрактний інтерфейс розв'язувача балістики.
//
// Призначення: відокремити МЕТОД обчислення точки скиду від
// решти системи. Сьогодні є аналітичне рішення (AnalyticalSolver,
// формули з ДЗ1). Завтра можна додати TableSolver (інтерполяція
// готових таблиць) чи NumericalSolver (числове інтегрування ОДУ) —
// без жодних змін у MissionProcessor.
//
// Це класичний патерн "Стратегія": MissionProcessor тримає
// IBallisticSolver* і може його підмінити на льоту через
// changeSolver().
// ============================================================

#include "Types.h"

class IBallisticSolver {
public:
    // Обчислити точку скиду.
    //
    // Параметри:
    //   dronePos    — поточна позиція дрона (x, y)
    //   target      — ціль (позиція + швидкість для прогнозу)
    //   altitude    — висота дрона над ціллю
    //   ammo        — параметри боєприпасу
    //   attackSpeed — швидкість дрона під час скиду
    //
    // Повертає DropPoint з полем valid=true при успіху.
    // Якщо балістика не розв'язується (наприклад, кубічне рівняння
    // не має додатного кореня) — valid=false.
    virtual DropPoint solve(const Coord& dronePos,
                            const Target& target,
                            float altitude,
                            const AmmoParams& ammo,
                            float attackSpeed) = 0;

    virtual ~IBallisticSolver() {}
};
