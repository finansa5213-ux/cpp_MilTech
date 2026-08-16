# ДЗ11 — Керування дроном по UART і автоматичний скид

Бортовий автопілот: читає з UART телеметрію і позиції цілей від чекера,
шле назад команди керування `PKT_CONTROL` (accel, turnRate) і в розрахований
момент дає імпульс скиду на GPIO. **Той самий код** працює у симуляції
(socat + gpio-sim) і на реальній Raspberry Pi (termios + libgpiod).

## Архітектура (база — ДЗ10)

| Компонент | Роль |
|---|---|
| `UartLink` | порт 115200 8N1 raw, неблокуюче читання, надсилання `PKT_CONTROL` |
| `GpioSignals` | libgpiod: START=1 на старті (тримається), DROP — імпульс 80 мс |
| `LinkState` | спільний стан RX↔місія: телеметрія, треки цілей, AMMO, RESULT (`mutex` + `condition_variable` + `atomic` stop) |
| `MissionProcessor` | потік RX (UART→`dlink::Parser`→стан) + цикл місії (вибір цілі, балістика, момент скиду) |
| `DroneController` | окремий модуль керування: рішення місії → accel/turnRate у [-1..1] |
| `AnalyticalSolver` | балістика з ДЗ10 (Кардано + ряд Тейлора); додано поле `DropPoint::carry` — горизонтальний виніс |

Відмінність від ДЗ10: `DronePhysics` вилучено з контуру — фізику інтегрує
чекер за нашими командами. Джерела даних: `PKT_TELEMETRY`/`PKT_TARGET`/`PKT_AMMO`
замість локального інтегратора і json-файлів.

## Логіка автопілота

1. START=1 → чекер шле AMMO і починає такт телеметрії.
2. Кожен кадр телеметрії: вибір найближчої цілі; швидкість цілі оцінюється
   різницею позицій `PKT_TARGET` (EMA-згладжування) — працює і по рухомих цілях.
3. Солвер від поточного стану дає час падіння `t_fall`, виніс `carry` і
   прогноз цілі `predicted = target + v_target * t_fall`.
4. Курс тримається на `predicted` (P-регулятор, +turnRate = вліво). Якщо
   дистанція менша за виніс — гальмо (менша швидкість → менший виніс).
5. Скид (CCIP): точка падіння `impact = pos + dir * carry`; коли
   `|impact - predicted| <= max(1 м, 0.4 * hitRadius)` — імпульс DROP.
   Скид одноразовий; після нього шлемо `accel=0, turnRate=0` і чекаємо вердикт
   (`PKT_RESULT` на платі / консоль чекера у sim).

## Збірка

```bash
sudo apt install g++ cmake libgpiod-dev socat gpiod
cmake -S . -B build && cmake --build build
# перевірка гонок: cmake -S . -B build-tsan -DUSE_TSAN=ON && cmake --build build-tsan
```

## Запуск у симуляції (потрібен модуль ядра gpio-sim)

```bash
# 1) віртуальна пара UART
socat -d -d pty,raw,echo=0,link=/tmp/ttyA pty,raw,echo=0,link=/tmp/ttyB
# 2) чекер (друкує ім'я свого gpio-sim чипа!)
sudo ./checker 1 --uart /tmp/ttyB --start-line 24 --drop-line 23
# 3) автопілот (підставити надрукований gpiochipN)
./build/homework_11 --uart /tmp/ttyA --gpiochip gpiochipN --start-line 24 --drop-line 23
```

## Запуск на Raspberry Pi

Другий UART: у `/boot/firmware/config.txt` додати `dtoverlay=uart3`,
перезавантажити, перевірити `ls /dev/ttyAMA*`. Ім'я GPIO-чипа хедера
перевірити `gpiodetect` (Pi 5: `gpiochip0`, на старих ядрах `gpiochip4`).

Перемички (студент → чекер, 3.3 В):

| Сигнал | З'єднання |
|---|---|
| телеметрія | чекер TX пін8 (GPIO14) → студ RX пін29 (GPIO5) |
| CONTROL | студ TX пін7 (GPIO4) → чекер RX пін10 (GPIO15) |
| START | студ GPIO24 (пін18) → чекер GPIO27 (пін13) |
| DROP | студ GPIO23 (пін16) → чекер GPIO22 (пін15) |

```bash
sudo ./checker_pi_arm64 1 --hw --uart /dev/serial0 --gpiochip gpiochip0 --start-line 27 --drop-line 22
./build/homework_11 --uart /dev/ttyAMA1 --gpiochip gpiochip0 --start-line 24 --drop-line 23
```

Прогнати місії 1..10 (`./checker N ...`). Вердикт: HIT, якщо промах ≤ hitRadius.
