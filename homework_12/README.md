# ДЗ 12 — Docker Compose як system bring-up: C2-сервіс

C2-сервіс — машина станів, що арбітрує команди автономії між `auto_stub` та FC (ArduRover SITL).
Waypoint'и передаються в FC тільки у стані `ARMED_GUIDED`; у `DISARMED`, `ARMED_HOLD`, `ARMED_MANUAL` — блокуються.

## Реалізовано

- `edge/c2/src/c2_controller.cpp` — машина станів C2 (`tick()`), логи, healthcheck-файл `/tmp/c2_healthy`
- `edge/c2/Dockerfile` — multi-stage build (ubuntu:22.04, CMake, MAVSDK 2.12.9)
- `edge/docker-compose.yml` — `restart: unless-stopped`, монтування конфігу та логів

## Запуск

### 1. SITL

```bash
docker compose -f sim/compose.sitl.yml up -d --build
docker exec fc_sim sh -lc 'tail -n 80 /tmp/Rover.log'   # чекати UDP 14550/14551
```

### 2. QGC

Підключити QGroundControl до UDP 14550. Для C2-сценарію: **Guided + Arm**, Start Mission не натискати.

### 3. Бортовий стек

```bash
cd edge
docker compose up -d --build
docker compose ps          # auto_stub і c2_service мають стати healthy
```

## Перевірка

```bash
docker compose logs -f c2_service
cat edge/logs/c2.log
```

Очікувані логи:

```
[C2] config: fc_port=14551
[C2] state: DISARMED -> ARMED_GUIDED
[C2] fwd: north=... east=...
[C2] blocked: waypoint in ARMED_HOLD
```

- Arm + Guided → waypoint'и передаються (`fwd`)
- Hold / Manual / Disarm → waypoint'и блокуються (`blocked`)
- Повернення Manual → Guided працює без перезапуску C2
- `edge/logs/c2.log` зберігається після `docker compose stop`

## Структура

```
homework_12/
  sim/                  # ArduRover SITL (не змінювати)
  qgc/lyman-patrol.plan # маршрут для auto_stub / QGC
  edge/
    docker-compose.yml  # c2_service + auto_stub
    config/c2_config.json
    c2/                 # C2-сервіс (C++20, MAVSDK, CMake)
```
