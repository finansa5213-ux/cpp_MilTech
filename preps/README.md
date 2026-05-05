# Preps

Мінімум щоб devcontainer у цьому проєкті запрацював.

## Кроки

1. **Поставити Docker** - див. [linux.md](linux.md) / [windows.md](windows.md) / [macos.md](macos.md) для своєї ОС.
2. **Створити власний репо з template** через `Use this template` на GitHub.
3. **Склонувати власний репо**. Команди в тому ж OS-файлі, секція "Як взяти код". На Windows - клон йде **в WSL**, не на `C:\`.
4. **Відкрити у VS Code**: `code .` з Ubuntu-терміналу після клону (не з Windows Explorer - `devcontainer.json` цього проекту зламається без WSL-сесії).
5. **Встановити Dev Containers extension**: VS Code запропонує сам (із `.vscode/extensions.json`). Якщо ні - `Ctrl+Shift+X` (на macOS `Cmd+Shift+X`), пошук `Dev Containers`, Install.
6. **Reopen in Container**: VS Code запропонує - погодитись. Перший білд 3-5 хв.

## Якщо вже клонували репо раніше

Якщо репо клоновано до нових оновлень - дочекатися Slack-анонсу від лектора.
Курс-репо створене як GitHub Template, тому звичайний `git pull` з upstream
не працює надійно для студентських копій.

Для підтягування snapshot-а курс-репо без затирання ДЗ 4:

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

`git status` перед комітом потрібен як перевірка, що `homework_04/` не
затерто, а оновлення tooling/devcontainer/CMake/demo/homework_05 підтягнуто.

Якщо клонована саме власна копія репозиторію, локальні зміни з неї
підтягуються як звичайно:

```bash
cd ~/projects/cpp-miltech
git pull
```

Якщо `cd` каже "No such file or directory" - перевірити куди було клоновано раніше (за інструкцією мало бути `~/projects/`, але могло бути інакше).

Потім у VS Code: Command Palette (Ctrl+Shift+P, на macOS Cmd+Shift+P) -> `Dev Containers: Rebuild Container`. Саме Rebuild, не Reopen - треба перебудувати образ, інакше фікси в Dockerfile не підхопляться.

## Альтернативні редактори

Не VS Code? Див. [devcontainers-cli.md](devcontainers-cli.md) - Neovim, JetBrains, Zed, Helix, Sublime, Emacs.

## Перевірка

Всередині контейнера (VS Code terminal):

```bash
cmake --preset debug
cmake --build --preset debug
./build/debug/homework_04/ugv_odometry homework_04/data/straight.txt
```

Перші дві команди мають завершитись без помилки. Остання команда запускає
стартовий executable для ДЗ 4; у стартовій версії він може нічого не друкувати,
бо реалізація ще є частиною домашнього завдання. Якщо збірка не працює - в чат курсу.
