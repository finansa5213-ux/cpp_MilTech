#pragma once
// ============================================================
// MissionProcessor — основний клас алгоритму місії.
// ============================================================

#include "ITargetProvider.h"
#include "IBallisticSolver.h"
#include "IConfigLoader.h"

class MissionProcessor {
private:
    // Зовнішні компоненти (не володіємо)
    IConfigLoader*    loader_;
    ITargetProvider*  provider_;
    IBallisticSolver* solver_;

    // Внутрішній стан ітератора
    int               currentIdx_;
    bool              initialized_;

    // Кешовані дані після init()
    MissionConfig     cfg_;
    AmmoParams        ammo_;
    Coord             dronePos_;

public:
    MissionProcessor(IConfigLoader* loader,
                     ITargetProvider* provider,
                     IBallisticSolver* solver);
    ~MissionProcessor() {}

    // 1) Завантажити конфіг через IConfigLoader та підготувати
    //    дані для ітерації.
    bool       init(const char* configSource = nullptr);

    // 2) Чи є ще необроблені цілі.
    bool       hasNext() const;

    // 3) Обробити наступну ціль: взяти Target з провайдера,
    //    обчислити DropPoint через solver, повернути.
    //    Лічильник просувається після кожного виклику.
    DropPoint  step();

    // 4) Скинути ітератор у початок.
    void       reset();

    // 5) Підмінити solver на льоту (Стратегія).
    void       changeSolver(IBallisticSolver* s);

    // Допоміжне: доступ до поточних кешованих даних — для main(),
    // щоб не дублювати запити до лоадера.
    const MissionConfig& config()    const { return cfg_; }
    const AmmoParams&    ammo()      const { return ammo_; }
    int                  currentIndex() const { return currentIdx_; }
};
