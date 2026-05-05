# Changelog

Усі помітні зміни в цьому репо фіксуються тут.
Формат - [Keep a Changelog](https://keepachangelog.com/uk/1.1.0/), дати в ISO 8601.

## 2026-04-30

### Added

- Block 2 / Lesson 2.4: `debug` CMake preset, VS Code launch config для
  `homework_04` і build task, який використовує Debug-збірку через preset.
- Block 2 / Lessons 2.4-2.5: стартовий код `homework_05` для діагностики
  телеметрії з C++ бібліотекою, виконуваним файлом, проблемними вхідними
  файлами і навмисними помилками під час виконання.
  `CMakeLists.txt` для `homework_05` не додано навмисно, це частина ДЗ 5.
- Block 2 / Lesson 2.4: demo `demos/lesson_2_4/debug_probe` для локального
  консольного GDB, core dump, Valgrind і віддаленого GDB/gdbserver на
  Raspberry Pi / ARM64-пристрої.
- Block 2 / Lesson 2.4: ARM64 cross-compilation preset `aarch64-debug` і
  CMake toolchain `cmake/toolchains/aarch64-linux-gnu.cmake`.
- Block 2 / Lessons 2.3-2.4: VS Code tasks для CMake configure/build і
  видалення локальної `build/` директорії. `CMake: configure and build`
  зроблено default build task для `Ctrl+Shift+B`.
  ([#10](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/10))
- Block 2 / Lessons 2.3-2.6: clang-tidy конфіг, debug/diagnostics tools
  (`gdb`, `gdbserver`, `gdb-multiarch`, `valgrind`) і налаштування clangd через
  `build/compile_commands.json`.
  ([#9](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/9))

### Changed

- Block 2 / Lessons 2.1-2.6: оновлено setup docs під GitHub Template flow,
  додано пояснювальні коментарі до student-facing tooling/config файлів і
  зменшено `.gitignore` до мінімального C++/CMake набору.
  ([#9](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/9))
- Block 2 / Lesson 2.4: clangd тепер читає compile database з
  `build/debug/compile_commands.json`, щоб відповідати `debug` preset.
- Block 2 / Lesson 2.4: README/preps і CI build flow переведено на
  `cmake --preset debug` і `cmake --build --preset debug`.
- Block 2 / Lesson 2.4: CI build flow додатково перевіряє
  `aarch64-debug` cross-build.
- Block 2 / Lesson 2.4: remote GDB інструкції уточнено: `satelite` лишається
  SSH alias, а `target remote` використовує IP пристрою.
- Block 2 / Lesson 2.4: core dump інструкції доповнено fallback-сценарієм для
  WSL/Docker, де core-файл може перехоплюватись системним handler-ом.
- Block 2 / Lesson 2.4: devcontainer runtime args доповнено `SYS_PTRACE`,
  `seccomp=unconfined` і `core=-1` для GDB/core dump сценаріїв.
- Block 2 / Lesson 2.4: core dump README доповнено WSL host setup через
  `kernel.core_pattern`, перевіркою `core.%e.%p` після rebuild/reopen
  devcontainer і запуском GDB по фактичному `core*` файлу.
- Block 2 / Lesson 2.4: core dump README уточнює, що `ulimit -c unlimited`
  потрібно виконати у поточній shell-сесії, якщо перевірка показує `0`.
- Block 2 / Lesson 2.4: devcontainer отримав `libclang-rt-18-dev` і
  `llvm-18`, щоб ASan/UBSan збірка через Clang лінкувалась і показувала
  читабельний stack trace з file:line.
- Block 2 / Lesson 2.4: `.gitignore` ігнорує локальні `core` / `core.*`
  файли, які з'являються під час core dump demo.
- Block 2 / Lesson 2.4: clang-tidy diagnostics у clangd вимкнено до заняття
  2.6 через devcontainer settings і `.clangd`; конфіг і пакет clang-tidy
  лишаються в repo/container для майбутнього ввімкнення.
- Block 2 / Lesson 2.4: `initializeCommand` запускає
  `.devcontainer/scripts/initialize` через `bash`, щоб копіювання файлів без
  executable bit не ламало старт devcontainer.
- Block 2 / Lesson 2.4: README/preps доповнено snapshot sync flow через
  `git clone` + `tar`, `git status` і commit, щоб оновлення курс-репо не
  залежали від ручного списку файлів.

## 2026-04-25

### Changed

- Devcontainer user setup спростили. Прибрали хардкод `useradd -u 1000`;
  покладаємось на `updateRemoteUserUID` від VS Code і `devcontainers/ci`, який
  синхронізує UID контейнерного користувача з host-ом на старті. Dockerfile
  тепер лише видаляє дефолтного `ubuntu` умовно через `if id ubuntu`, щоб
  звільнити слот UID 1000.
  ([#5](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/5))

### Added

- GitHub Actions workflow `build.yml` через `devcontainers/ci@v0.3`. CI будує
  проєкт у тому самому devcontainer-і, що й локальний VS Code, запускається на
  push у `main` і на кожен PR.
  ([#4](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/4))

## 2026-04-23

### Added

- Пакети `ssh` і `git` у devcontainer з коробки. Базовий git-workflow і
  SSH-операції тепер працюють без донастройки.
  ([#3](https://github.com/robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY/pull/3))
- Стартові матеріали блоку 2: корневий `CMakeLists.txt`, `homework_04/`
  (starter для wheel odometry), `.devcontainer/` з Dockerfile і
  initialize-скриптом, `preps/` з інструкціями для Linux, macOS, Windows 11
  + WSL2 і Dev Containers CLI.
