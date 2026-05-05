# C++ для військових технологій

Шаблон репозиторію курсу. Містить dev-контейнер з C++ toolchain, стартову
структуру проєкту і окремі папки під кожну домашню роботу.

Основний спосіб користуватись цим репо - створити власну копію через кнопку
**Use this template** на GitHub. Курс-репо залишається read-only для вас,
PR-и на вашій копії йдуть у ваш main.

## Швидкий старт

1. **Створити власну копію через Use this template:**
   - Відкрити сторінку курс-репо на GitHub.
   - Натиснути `Use this template` -> `Create a new repository`.
   - В Owner dropdown обрати свій акаунт, задати назву (наприклад
     `cpp-miltech`), створити.

   Це дає самостійне репо у вашому акаунті з чистою git-історією і без
   fork-зв'язку з курс-репо.

2. **Склонувати власний репо і відкрити у VS Code:**

   > **Windows:** клонувати всередині WSL (відкрити `wsl` у PowerShell),
   > не на `C:\`. Dev Containers очікує POSIX-середовище (shell-скрипти,
   > `$HOME`, `$USER`); на Windows host це не працює - VS Code падає на
   > `.devcontainer/scripts/initialize` (error 193), а `${localEnv:HOME}`
   > і `${localEnv:USER}` порожні. Деталі - у
   > [preps/windows.md](preps/windows.md).

   ```bash
   git clone https://github.com/<your-username>/cpp-miltech.git
   cd cpp-miltech
   code .
   ```
   VS Code запропонує `Reopen in Container` - погодитись. Перший запуск
   тягне і білдить Docker image (5-10 хв), наступні запуски миттєві.

3. **Зібрати весь репо всередині контейнера:**
   ```bash
   cmake --preset debug
   cmake --build --preset debug
   ```
   Виконувані файли домашніх робіт з'являться в `build/debug/homework_XX/`.
   Demo-код занять лежить у відповідних піддиректоріях `build/debug/demos/`.

4. **Зібрати ARM64 debug binary для Raspberry Pi / Radxa / Jetson:**
   ```bash
   cmake --preset aarch64-debug
   cmake --build --preset aarch64-debug
   ```
   ARM64 артефакти з'являться в `build/aarch64-debug/`. Цей preset
   використовує toolchain `cmake/toolchains/aarch64-linux-gnu.cmake`.

## Доступ для перевірки

Додати обох лекторів як collaborators свого репо, щоб вони могли бачити
код і робити review на pull request-ах:

- `Settings` -> `Collaborators` -> `Add people` -> `yevhenkuznetsov`, роль
  `Read` достатня.
- `Settings` -> `Collaborators` -> `Add people` -> `farnblack`, роль
  `Read` достатня.

Без цього лектори не побачать приватний репо взагалі і не зможуть залишати
formal review (approve / request changes) навіть на публічному.

## Оновлення з курс-репо

Зміни в курс-репо не синкаються автоматично у локальний репо. Якщо
щось важливе оновлюється - буде анонс у Slack-каналі курсу.

Рекомендований спосіб підтягнути snapshot курс-репо без затирання ДЗ 4:

```bash
tmp_dir=$(mktemp -d)

git clone --depth 1 \
  git@github.com:robot-dreams-code/C-PLUS-PLUS-FOR-MILITARY-TECHNOLOGY.git \
  "$tmp_dir/course"

(
  cd "$tmp_dir/course"
  tar --exclude='./.git' --exclude='./homework_04' -cf - .
) | tar -xf -

rm -rf "$tmp_dir"

git status
git add .
git commit -m "chore: sync course repository updates"
```

Команда копіює tooling/devcontainer/CMake/demo/homework_05 з актуального
курс-репо, але не затирає `homework_04/`. `git status` перед комітом потрібен
як контроль того, які файли зміняться у локальному репо.

**Чому не git merge / cherry-pick:** курс-репо позначений як GitHub
Template. Копія через "Use this template" - це новий репо з єдиним
"Initial commit", без зв'язку з курс-репо в історії. Спільного предка
нема, тому git нічого не може злити автоматично.

Чому Template, а не fork: fork залишає історію спільною, і merge для
нього працював би. Але PR на fork-у GitHub дефолтно цілить у upstream
(у курс-репо). Курс-репо має лишатися read-only, PR-и студентів - іти
у їхні репо. Template це знімає, ціна - нема прямого git-sync.

`git merge course/main` падає з `refusing to merge unrelated histories`.
З `--allow-unrelated-histories` merge пройде, але вийде каша: дві
кореневі коміти і конфлікти на більшості файлів. Cherry-pick не падає
сам, але конфліктує на локально модифікованих файлах.

Простіше - snapshot copy за командою вище. Без merge unrelated histories і
без ручного вибору окремих файлів.

## Перед стартом

Потрібно:
- **Docker Engine** (рекомендовано на всіх платформах). `preps/` містить
  інструкції: Linux напряму, macOS через Colima, Windows всередині WSL2.
  Docker Desktop теж працює, але потребує додаткових налаштувань (WSL2
  integration, file sharing, ресурси), які `preps/` не покриває.
- **VS Code** з розширенням `ms-vscode-remote.remote-containers` (Dev Containers).

Покрокові інструкції під конкретну платформу:
- [Windows 11 + WSL2](preps/windows.md)
- [Linux](preps/linux.md)
- [macOS](preps/macos.md)

Загальний огляд інструментів в preps/ - у [preps/README.md](preps/README.md).

## Структура репо

```
.
├── .devcontainer/         # Docker image + конфіг dev-контейнера
├── .github/workflows/     # GitHub Actions: CI build через devcontainer
├── CHANGELOG.md           # лог помітних змін по PR-ах
├── CMakeLists.txt         # корневий CMakeLists, підтягує homework_XX через add_subdirectory
├── cmake/toolchains/       # CMake toolchain-файли для cross-compilation
├── demos/                  # demo-код для занять
├── homework_XX/           # окрема домашка, кожна зі своїм CMakeLists.txt
└── preps/                 # інструкції зі сетапу за платформами
```

## Як додати нову домашку

Коли лектор анонсував нову ДЗ або ви починаєте роботу над уже анонсованою:

1. Створити папку `homework_NN/` з власним `CMakeLists.txt`, `src/`, `include/`
   і даними якщо є.
2. Додати у корневий CMakeLists.txt рядок:
   ```cmake
   add_subdirectory(homework_NN)
   ```
3. Комітити і пушити у свій репо. CI (якщо налаштований у вашому репо)
   перевірить що все збирається.

## Що робить CI

Workflow `.github/workflows/build.yml` запускається на кожен PR (і на push у
main). Він:
1. Будує devcontainer image з Dockerfile.
2. Виконує native debug build:
   `cmake --preset debug && cmake --build --preset debug`.
3. Виконує ARM64 cross debug build:
   `cmake --preset aarch64-debug && cmake --build --preset aarch64-debug`.
4. Падає якщо хоч одна `homework_XX` або demo target не компілюється з
   тулчейном з devcontainer-а (gcc-13, clang-18, C++20).

Якщо CI червоний локально зібралося, але на CI ні - швидше за все
devcontainer треба rebuild локально (Dev Containers: Rebuild Container),
щоб підхопити свіжі версії тулів.
