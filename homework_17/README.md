# homework_17 — ДЗ Заняття 34: телеметрія дрона по MAVLink 2 / UDP

Розширення ДЗ11. Автопілот працює так само: читає телеметрію і цілі з UART від
чекера ДЗ11, рахує балістику, шле `PKT_CONTROL` і дає імпульс DROP на GPIO.
Додано другий канал: **той самий кадр телеметрії** паралельно йде в MAVLink 2
по UDP на `127.0.0.1:14550`, а в момент скиду — `COMMAND_LONG`
(`MAV_CMD_USER_1`) з повторами до `COMMAND_ACK`.

ДЗ11 при цьому не змінюється: його модулі підключаються з `../homework_11`
напряму, дублювання коду немає.

## Структура

```
include/
  transport.hpp          інтерфейс каналу байтів (тестування без сокета)
  udp_socket.hpp         неблокуючий UDP поверх ITransport
  geo.hpp                локальні метри -> градуси, робота з курсом
  mavlink_link.hpp       кадри MAVLink 2 + FSM повторів команди скиду
  dz11_bridge.hpp        міст dlink::Telemetry -> MAVLink (уся конверсія тут)
  MavMissionProcessor.h  цикл місії ДЗ11 з мітками [ДЗ17] у точках підключення
src/                     реалізації + main.cpp
tests/                   GoogleTest: геометрія, одиниці полів, повтори, міст
tools/fake_checker.py    локальний «псевдочекер» MAVLink для самоперевірки
```

Підключається з ДЗ11 без змін: `AnalyticalSolver`, `DroneController`,
`UartLink`, `GpioSignals`, `LinkState`, `Types.h`, `drone_link.h`.

## Що саме додано до ДЗ11

Усі відмінності позначені в коді міткою `[ДЗ17]` і зводяться до трьох викликів:

| Місце в циклі місії | Виклик |
|---|---|
| кожен кадр `PKT_TELEMETRY` | `bridge_.onTelemetry(t, nowMs())` |
| момент `gpio_.pulseDrop(80)` | `bridge_.onDrop(t, nowMs())` |
| тайм-аут очікування телеметрії | `bridge_.tick(nowMs())` |

Плюс `coastUntilDropResolved()` — режим вибігу після циклу.

### Чому потрібен вибіг

Чекер ДЗ11 припиняє слати телеметрію одразу після оцінки скиду, а на 5 спроб
`COMMAND_LONG` з таймаутом 500 мс потрібно до 2.5 с. Якби програма вийшла
разом із ДЗ11, чекер ДЗ17 побачив би обірваний потік і незавершені повтори.

У режимі вибігу позиція екстраполюється по останній швидкості
(`x += vx * dt`), тому перевірка «зміна позиції відповідає швидкостям»
лишається виконаною, курс не пливе, а `time_boot_ms` зростає монотонно.

### Конвенція осей

У ДЗ11 напрямок польоту рахується як `{cos(dir), sin(dir)}` — отже `dir` це
математичний кут від осі X проти годинникової стрілки. Мапимо `x` в схід,
`y` в північ.

Компасний курс для `hdg` і `ATTITUDE.yaw` береться **з вектора швидкості**
(`atan2(vx, vy)`), а не з `t.dir`. Так курс гарантовано узгоджений із напрямком
руху — саме це перевіряє чекер. `t.dir` використовується лише як запасний
варіант при майже нульовій швидкості.

## Збірка

```bash
sudo apt install libgpiod-dev socat gpiod
git clone --depth 1 https://github.com/mavlink/c_library_v2.git   # у корені репозиторію
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Без `libgpiod` ціль пропускається з попередженням — як у ДЗ11.

## Запуск

Ланцюжок той самий, що в ДЗ11, плюс слухач MAVLink на 14550.

```bash
# 1) віртуальна пара UART
socat -d -d pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB

# 2) чекер ДЗ11 — джерело фізики (друкує ім'я свого gpio-sim чипа!)
sudo ./checker 1 --uart /tmp/ttyB --start-line 24 --drop-line 23

# 3) слухач MAVLink: QGroundControl АБО чекер ДЗ17
./checker-linux-x86_64 14550

# 4) автопілот
./build/homework_17/drone --uart /tmp/ttyA --gpiochip gpiochipN \
    --start-line 24 --drop-line 23 --dest 127.0.0.1:14550
```

Порядок важливий: слухач MAVLink має вже слухати порт, коли стартує дрон.

## QGroundControl

Замість кроку 3 запустіть QGC — він слухає UDP :14550 за замовчуванням, дрон
з'явиться на мапі сам. QGC **не** відповідає ACK на `MAV_CMD_USER_1`, тому
програма чесно робить 5 спроб і друкує `[mavlink] ACK ne otrymano pislya 5 sprob`,
а телеметрія при цьому не зупиняється.

Чекер ДЗ17 і QGC одночасно не піднімуться — обидва займають 14550.

### WSL2 + QGC на Windows

`127.0.0.1` з WSL2 до Windows не долетить. Або дзеркальна мережа у
`C:\Users\<user>\.wslconfig`:

```ini
[wsl2]
networkingMode=mirrored
```

(потім `wsl --shutdown`), або явна адреса хоста:

```bash
--dest $(ip route show default | awk '{print $3}'):14550
```

Чекерів це не стосується — вони запускаються всередині WSL2.

## Самоперевірка без чекера ДЗ17

```bash
python3 homework_17/tools/fake_checker.py --ack-on 2 &
# далі кроки 1, 2, 4 зі списку вище
```

Скрипт розбирає потік із перевіркою CRC, друкує темпи і вміє відповідати ACK на
N-у спробу. `--ack-on 0` — не відповідати ніколи.

## Відповідність вимогам ДЗ

| Вимога | Де реалізовано |
|---|---|
| MAVLink 2, стабільні sysid/compid | `MavlinkLink` (конструктор знімає `OUT_MAVLINK1`), sysid=1, compid=`MAV_COMP_ID_AUTOPILOT1` |
| HEARTBEAT ~1 Гц, QUADROTOR/ACTIVE | `Dz11Bridge::sendHeartbeatIfDue()`, темп не залежить від частоти кадрів ДЗ11 |
| Телеметрія >=2 Гц | кожен кадр `PKT_TELEMETRY`; у вибігу — 10 Гц |
| `time_boot_ms` монотонний | `t.t_ms` з ДЗ11; у вибігу нарощується по реальному часу |
| lat/lon x1e7, alt мм, vx/vy см/с, hdg сот. градуса | `degTo1e7`, `metersToMm`, `mpsToCms`, `headingToCentiDeg` |
| hdg відповідає напрямку руху | курс із вектора швидкості, той самий кут у `ATTITUDE.yaw` |
| COMMAND_LONG з координатами скиду | `Dz11Bridge::onDrop()`; `param5/6` — позиція дрона в момент випуску (чекер вимагає <=500 м), `param7` — висота |
| Повтор до ACK, до 5 спроб, таймаут | `MavlinkLink::poll()`, `kMaxDropAttempts=5`, `kAckTimeoutMs=500` |
| Після ACK — тиша | `DropState::Acked` блокує повтори |
| Телеметрія не зупиняється під час повторів | повтори неблокуючі + режим вибігу |

## Що перевірено

- Збірка під `-Wall -Wextra -Wpedantic` без попереджень (GCC 13 і 15).
- 25 юніт-тестів: геометрія, одиниці полів MAVLink, автомат повторів,
  конверсія осей ДЗ11, темп HEARTBEAT, узгодженість позиції у вибігу.
- Наскрізний прогін із чекером ДЗ11 і чекером ДЗ17 — на робочій машині
  (потрібні socat і gpio-sim).
