// ============================================================
// MissionProcessor.cpp — автопілот: RX-потік + цикл місії.
// ============================================================
#include "MissionProcessor.h"

#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace std::chrono;
using namespace std::chrono_literals;

MissionProcessor::MissionProcessor(UartLink& uart, GpioSignals& gpio,
                                   LinkState& state,
                                   std::unique_ptr<IBallisticSolver> solver)
    : uart_(uart), gpio_(gpio), st_(state), solver_(std::move(solver)) {}

// ------------------------------------------------------------
// Потік RX: байти з UART -> Parser -> LinkState.
// ------------------------------------------------------------
void MissionProcessor::runRx()
{
    dlink::Parser parser;
    uint8_t buf[256];
    uint8_t payload[260];
    uint8_t type = 0, len = 0;

    while (!st_.stop.load()) {
        int n = uart_.readBytes(buf, sizeof buf);
        if (n < 0) {
            std::fprintf(stderr, "[rx] pomylka chytannya UART\n");
            st_.stop = true;
            st_.cv.notify_all();
            return;
        }
        if (n == 0) { usleep(2000); continue; } // даних нема — коротка пауза

        for (int i = 0; i < n; ++i) {
            if (!parser.feed(buf[i], type, payload, len)) continue;

            std::lock_guard<std::mutex> lk(st_.mtx);
            switch (type) {
            case dlink::PKT_TELEMETRY:
                if (len == sizeof(dlink::Telemetry)) {
                    dlink::Telemetry t;
                    std::memcpy(&t, payload, sizeof t);
                    st_.onTelemetry(t);
                    st_.cv.notify_all();
                }
                break;
            case dlink::PKT_TARGET:
                if (len == sizeof(dlink::TargetPos)) {
                    dlink::TargetPos tp;
                    std::memcpy(&tp, payload, sizeof tp);
                    st_.onTarget(tp);
                }
                break;
            case dlink::PKT_AMMO:
                if (len == sizeof(dlink::AmmoCfg)) {
                    std::memcpy(&st_.ammo, payload, sizeof st_.ammo);
                    st_.hasAmmo = true;
                    std::printf("[rx] AMMO %.15s m=%.3f d=%.4f l=%.4f hitR=%.1f tsiley=%u\n",
                                st_.ammo.name, st_.ammo.mass, st_.ammo.drag,
                                st_.ammo.lift, st_.ammo.hitRadius, st_.ammo.nTargets);
                }
                break;
            case dlink::PKT_RESULT:
                if (len == sizeof(dlink::Result)) {
                    std::memcpy(&st_.result, payload, sizeof st_.result);
                    st_.hasResult = true;
                    st_.cv.notify_all();
                }
                break;
            default:
                break; // PKT_CONFIG та інше — не потрібні для місії
            }
        }
    }
}

// ------------------------------------------------------------
// Вибір цілі: найближча з відомих (на один виліт — один скид).
// ------------------------------------------------------------
bool MissionProcessor::pickTarget(const std::map<int, TargetTrack>& targets,
                                  const Coord& dronePos,
                                  int& id, Target& out) const
{
    float best = 1e30f;
    bool  ok   = false;
    for (const auto& [tid, tr] : targets) {
        if (!tr.seen) continue;
        const float d = distance(dronePos, tr.pos);
        if (d < best) {
            best = d;
            id   = tid;
            out  = Target{tr.pos, tr.hasVel ? tr.vel : Coord{}};
            ok   = true;
        }
    }
    return ok;
}

// ------------------------------------------------------------
// Основний цикл місії.
// ------------------------------------------------------------
void MissionProcessor::runMission()
{
    // «Я готовий» — за цим сигналом чекер починає слати AMMO/телеметрію.
    gpio_.assertStart();
    std::printf("[mission] chekayu AMMO ta telemetriyu vid chekera...\n");

    uint64_t   seenSeq   = 0;
    auto       lastRx    = steady_clock::now();
    bool       everRx    = false;
    unsigned   quietWarn = 0;

    while (!st_.stop.load()) {
        dlink::Telemetry t{};
        dlink::AmmoCfg   ammo{};
        bool hasAmmo = false;
        std::map<int, TargetTrack> targets;

        // -------- чекаємо нову телеметрію / RESULT --------
        {
            std::unique_lock<std::mutex> lk(st_.mtx);
            bool woke = st_.cv.wait_for(lk, 500ms, [&] {
                return st_.telemSeq > seenSeq || st_.hasResult || st_.stop.load();
            });
            if (st_.stop.load()) break;

            if (st_.hasResult) { // вердикт з UART (реальна плата)
                const auto& r = st_.result;
                std::printf("[mission] REZULTAT: %s tsil=%d promah=%.2f m (skyd t=%.2f s)\n",
                            r.hit ? "HIT" : "MISS", (int)r.targetId,
                            (double)r.miss_m, r.drop_t_ms / 1000.0);
                break;
            }
            if (!woke) { // тайм-аут без нових даних
                lk.unlock();
                const auto quiet = steady_clock::now() - lastRx;
                if (dropped_ && quiet > 2s) {
                    std::printf("[mission] telemetriya zamovkla pislya skydu — misiya zavershena "
                                "(dyv. verdykt chekera)\n");
                    break;
                }
                if (!everRx && ++quietWarn % 20 == 0)
                    std::printf("[mission] dosi nemaye danykh: perevirte port, provody i "
                                "chy zapushcheno cheker\n");
                continue;
            }
            seenSeq = st_.telemSeq;
            t       = st_.telem;
            hasAmmo = st_.hasAmmo;
            ammo    = st_.ammo;
            targets = st_.targets; // копія знімка цілей
        }
        lastRx = steady_clock::now();
        everRx = true;

        const Coord pos{t.x, t.y};

        // -------- ще нема боєприпасу або цілей: тримаємо позицію --------
        int    tgtId = -1;
        Target tgt;
        if (!hasAmmo || !pickTarget(targets, pos, tgtId, tgt)) {
            uart_.sendControl(0.f, 0.f);
            continue;
        }

        // -------- балістика від ПОТОЧНОГО стану --------
        const AmmoParams ap{ammo.name, ammo.mass, ammo.drag, ammo.lift};
        const float vSolve = std::max(t.speed, 1.0f); // проти виродження на старті
        const DropPoint dp = solver_->solve(pos, tgt, t.z, ap, vSolve);

        const Coord aim = dp.valid ? dp.predictedTarget : tgt.pos;
        const float D   = distance(pos, aim);

        // Усередині мінімальної дальності (виніс більший за відстань) — гальмуємо,
        // менша швидкість => менший виніс.
        const bool brake = !dropped_ && dp.valid && D < dp.carry;

        // -------- умова скиду (CCIP) --------
        // Точка падіння = позиція + виніс уздовж поточного курсу.
        // Скид, коли прогнозоване падіння в межах запасу від цілі.
        if (!dropped_ && dp.valid && t.speed > 0.5f) {
            const Coord dirU{std::cos(t.dir), std::sin(t.dir)};
            const Coord impact = pos + dirU * dp.carry;
            const float miss   = distance(impact, dp.predictedTarget);
            const float margin = std::max(1.0f, 0.4f * ammo.hitRadius);
            if (miss <= margin) {
                gpio_.pulseDrop(80); // 50-100 мс; зараховується перший імпульс
                dropped_ = true;
                dropTms_ = t.t_ms;
                std::printf("[mission] DROP t=%.2f s poz=(%.1f,%.1f) v=%.1f m/s "
                            "ochikuvanyi promah=%.2f m (tsil %d)\n",
                            t.t_ms / 1000.0, (double)t.x, (double)t.y,
                            (double)t.speed, (double)miss, tgtId);
            }
        }

        // -------- команда керування --------
        ControlCmd cmd;
        if (!dropped_)
            cmd = ctrl_.steer(t, aim, brake);   // ведемо дрон до точки скиду
        // після скиду: accel=0, turnRate=0 — летимо прямо, чекаємо вердикт
        uart_.sendControl(cmd.accel, cmd.turnRate);
    }
}
