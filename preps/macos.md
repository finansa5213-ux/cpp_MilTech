# macOS

Без Docker Desktop - через Colima.

```bash
brew install colima docker docker-buildx
```

Підключити buildx як CLI plugin:

```bash
mkdir -p ~/.docker/cli-plugins
ln -sfn "$(brew --prefix)/opt/docker-buildx/bin/docker-buildx" ~/.docker/cli-plugins/docker-buildx
```

Запустити:

```bash
colima start
```

Перший запуск тягне linuxkit image - пару хвилин.

Перевірка:

```bash
docker run hello-world
docker buildx version
```

Автостарт при логіні (опційно):

```bash
brew services start colima
```

На Apple Silicon (M1/M2/M3/M4) все запускається нативно як `arm64`. Якщо раптом попаде образ тільки `amd64` - `docker run --platform linux/amd64 ...` (повільніше, через qemu).

## Як взяти код

Спочатку створити власний репозиторій через `Use this template` на сторінці
курс-репо в GitHub. Потім клонувати власний репозиторій:

```bash
git config --global user.name "Your Name"
git config --global user.email "you@example.com"

mkdir -p ~/projects
cd ~/projects
git clone https://github.com/<your-username>/cpp-miltech.git
cd cpp-miltech
code .
```

VS Code запропонує встановити Dev Containers extension (із `.vscode/extensions.json`) - погодитись. Якщо промпт не зявився: `Cmd+Shift+X`, пошук `Dev Containers`, Install.

Далі VS Code запропонує `Reopen in Container` - теж погодитись.
