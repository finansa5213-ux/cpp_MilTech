#pragma once
// ============================================================
// MissionProcessor — «бортовий автопілот» ДЗ11.
//
// Два потоки (ДЗ10):
//   runRx()      — читає UART, годує dlink::Parser, розкладає
//                  кадри у LinkState (під мьютексом);
//   runMission() — на кожній новій телеметрії: вибір цілі,
//                  балістика (AnalyticalSolver), рішення про
//                  скид (CCIP), команда CONTROL у UART.
//
// Фізику дрона інтегрує ЧЕКЕР — за нашими CONTROL-командами.
// ============================================================
#include <memory>

#include "DroneController.h"
#include "LinkState.h"
#include "interfaces/IBallisticSolver.h"
#include "io/GpioSignals.h"
#include "io/UartLink.h"

class MissionProcessor {
public:
    MissionProcessor(UartLink& uart, GpioSignals& gpio, LinkState& state,
                     std::unique_ptr<IBallisticSolver> solver);

    void runRx();      // тіло потоку приймача UART
    void runMission(); // основний цикл автопілота (потік main)

private:
    // вибрати найближчу до дрона ціль; false — цілей ще немає
    bool pickTarget(const std::map<int, TargetTrack>& targets,
                    const Coord& dronePos, int& id, Target& out) const;

    UartLink&    uart_;
    GpioSignals& gpio_;
    LinkState&   st_;
    std::unique_ptr<IBallisticSolver> solver_;
    DroneController ctrl_;

    bool     dropped_  = false;
    uint32_t dropTms_  = 0;
};
