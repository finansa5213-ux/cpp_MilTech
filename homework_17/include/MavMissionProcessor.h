#pragma once
// ============================================================
// MavMissionProcessor — автопілот ДЗ11, розширений телеметрією MAVLink 2.
//
// Це копія MissionProcessor з ДЗ11. Усі відмінності позначені міткою
// [ДЗ17] і зводяться до трьох точок підключення Dz11Bridge:
//   1) кожен кадр телеметрії  -> bridge_.onTelemetry()
//   2) момент скиду           -> bridge_.onDrop()
//   3) тайм-аут очікування    -> bridge_.tick()
// Плюс коректне завершення: цикл не виходить, доки не відпрацювали повтори
// COMMAND_LONG, а після зупинки телеметрії ДЗ11 працює режим вибігу.
//
// Оригінальний опис ДЗ11:
//
// Два потоки (дух ДЗ10):
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
#include "dz11_bridge.hpp" // [ДЗ17]
#include "LinkState.h"
#include "interfaces/IBallisticSolver.h"
#include "io/GpioSignals.h"
#include "io/UartLink.h"

class MavMissionProcessor {
public:
    // [ДЗ17] додано параметр bridge — міст у MAVLink.
    MavMissionProcessor(UartLink& uart, GpioSignals& gpio, LinkState& state,
                        std::unique_ptr<IBallisticSolver> solver,
                        mavtel::Dz11Bridge& bridge);

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
    mavtel::Dz11Bridge& bridge_; // [ДЗ17]

    bool     dropped_  = false;
    uint32_t dropTms_  = 0;

    // [ДЗ17] Вибіг після зупинки телеметрії ДЗ11: доганяємо повтори
    // COMMAND_LONG, не обриваючи потік MAVLink.
    void coastUntilDropResolved();
};
