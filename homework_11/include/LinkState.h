#pragma once
// ============================================================
// LinkState — спільний стан між потоком RX (UART) і потоком
// місії. Дух ДЗ10: увесь спільний стан під std::mutex,
// прапорці зупинки — std::atomic, пробудження споживача —
// std::condition_variable на новій телеметрії.
// ============================================================
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <mutex>

#include "Types.h"
#include "drone_link.h"

// Трек однієї цілі: остання позиція + оцінка швидкості
// (різниця позицій між пакетами TARGET, згладжена EMA).
struct TargetTrack {
    Coord    pos;
    Coord    vel;
    uint32_t t_ms   = 0;     // час телеметрії, коли прийшов пакет
    bool     seen   = false;
    bool     hasVel = false;
};

class LinkState {
public:
    std::mutex              mtx;
    std::condition_variable cv;    // нова телеметрія / RESULT / stop

    dlink::Telemetry telem{};
    uint64_t         telemSeq = 0; // лічильник кадрів телеметрії

    std::map<int, TargetTrack> targets;

    dlink::AmmoCfg ammo{};
    bool           hasAmmo = false;

    dlink::Result result{};
    bool          hasResult = false;

    std::atomic<bool> stop{false};

    // Виклики нижче — ПІД mtx (з потоку RX).

    void onTelemetry(const dlink::Telemetry& t) {
        telem = t;
        ++telemSeq;
    }

    void onTarget(const dlink::TargetPos& tp) {
        TargetTrack& tr  = targets[tp.id];
        const Coord  np{tp.x, tp.y};
        const uint32_t now = telem.t_ms; // таймстемп останньої телеметрії

        if (tr.seen && now > tr.t_ms) {
            const float dt = (float)(now - tr.t_ms) * 1e-3f;
            if (dt > 1e-3f) {
                const Coord v = (np - tr.pos) / dt;
                // EMA-згладжування, щоб шум квантування не смикав прогноз
                tr.vel    = tr.hasVel ? tr.vel * 0.6f + v * 0.4f : v;
                tr.hasVel = true;
            }
        }
        tr.pos  = np;
        tr.t_ms = now;
        tr.seen = true;
    }
};
