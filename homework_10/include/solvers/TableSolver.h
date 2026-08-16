#pragma once
// ============================================================
// TableSolver — табличний балістичний солвер.
//
// Реалізує IBallisticSolver. Замість аналітичного розв'язку
// диференціального рівняння руху використовує 5-вимірну таблицю
// заздалегідь обчислених значень (час польоту + горизонтальна
// дистанція) з багатовимірною лінійною інтерполяцією.
//
// Осі таблиці: Z0 (висота), V0 (швидкість), m (маса),
//              d (опір), l (підйомна сила).
// ============================================================
#include "interfaces/IBallisticSolver.h"
#include <vector>
#include <string>

// ------------------------------------------------------------
// BallisticTable — 5-вимірна сітка з нерівномірним кроком.
// ------------------------------------------------------------
struct BallisticTable {
    // 5 осей — кожна зі своїм набором вузлів (нерівномірний крок)
    std::vector<float> axisZ0;   // висота
    std::vector<float> axisV0;   // швидкість
    std::vector<float> axisM;    // маса
    std::vector<float> axisD;    // опір
    std::vector<float> axisL;    // підйомна сила

    // Результат у кожному вузлі сітки
    struct Result {
        float t     = 0.f;   // час польоту
        float hDist = 0.f;   // горизонтальна дистанція
    };

    // Плоский масив розміром |Z0|*|V0|*|M|*|D|*|L|
    std::vector<Result> data;

    // Індекс у плоскому масиві: [iZ0][iV0][iM][iD][iL]
    size_t index(int iz, int iv, int im, int id, int il) const {
        return ((((size_t)iz * axisV0.size() + iv) * axisM.size() + im)
                * axisD.size() + id) * axisL.size() + il;
    }
    const Result& at(int iz, int iv, int im, int id, int il) const {
        return data[index(iz, iv, im, id, il)];
    }

    bool empty() const { return data.empty(); }

    // Завантаження з текстового файлу.
    bool load(const std::string& path);

    // Багатовимірна лінійна інтерполяція (вкладені lerp) з clamp
    // на межах таблиці.
    Result lookup(float Z0, float V0, float m, float d, float l) const;
};

// ------------------------------------------------------------
// TableSolver
// ------------------------------------------------------------
class TableSolver : public IBallisticSolver {
private:
    BallisticTable table_;
    bool           loaded_ = false;

public:
    explicit TableSolver(const std::string& tablePath = "ballistic_table.txt");
    ~TableSolver() override = default;

    bool isLoaded() const { return loaded_; }

    DropPoint solve(const Coord&      dronePos,
                    const Target&     target,
                    float             altitude,
                    const AmmoParams& ammo,
                    float             attackSpeed) override;
};
