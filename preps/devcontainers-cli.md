# Будь-який редактор замість VS Code

Є офіційний CLI `@devcontainers/cli`, який робить те саме, що VS Code: читає `devcontainer.json`, будує образ, запускає контейнер, виконує команди всередині.

Потрібен Node.js (16+):

```bash
npm install -g @devcontainers/cli
```

Перевірка:

```bash
devcontainer --version
```

## Базовий флоу

В корені проєкту:

```bash
devcontainer up --workspace-folder .           # build + start
devcontainer exec --workspace-folder . bash    # зайти в контейнер
```

Одна команда без входу в shell:

```bash
devcontainer exec --workspace-folder . cmake --preset debug
devcontainer exec --workspace-folder . cmake --build --preset debug
```

Після зміни `Dockerfile`:

```bash
devcontainer up --workspace-folder . --remove-existing-container
```

## Як при цьому редагувати код

Файли замаунчені на хост, тож редагування - будь-яким локальним редактором. Команди - через `devcontainer exec` в окремому терміналі.

- **Neovim / Vim / Helix / Sublime / Emacs** - редагування на хості, команди через `devcontainer exec`. Простий і робочий шлях.
- **JetBrains (CLion, IntelliJ, Rider)** - має нативну підтримку: `File -> Remote Development -> Dev Containers`. Вони розуміють `devcontainer.json` напряму, без посередника.

## Що втрачається порівняно з VS Code

- Автоматичне встановлення extensions у контейнер (це специфіка VS Code).
- Інтегрований debug UI (свій редактор - свій debug UI, налаштовуй `gdb` сам).
- Automatic port forwarding - робиться через `forwardPorts` у `devcontainer.json` + ручне підключення.

Для C++ workflow це все некритично: більшість роботи через shell.
