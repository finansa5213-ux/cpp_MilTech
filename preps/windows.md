# Windows 11

WSL2 з Ubuntu, всередині - Docker Engine.

Перед встановленням - у BIOS/UEFI має бути увімкнена віртуалізація. Перевірка: Task Manager -> Performance -> CPU, внизу має бути `Virtualization: Enabled`.

Якщо `Disabled` - reboot, зайти в BIOS/UEFI (F2 / Del / F10 при старті, залежить від вендора), знайти опцію типу `Intel Virtualization Technology` / `VT-x` / `AMD-V` / `SVM Mode` (зазвичай в секціях `Advanced`, `CPU Configuration` або `Security`), увімкнути, зберегти, вийти.

В **admin PowerShell**:

```powershell
wsl --install -d Ubuntu
```

Reboot. Після reboot Ubuntu запуститься - задати username + пароль.

Перевірка (PowerShell):

```powershell
wsl -l -v
```

Має бути `Ubuntu` / `Running` / `VERSION: 2`. Якщо `VERSION: 1` - `wsl --set-version Ubuntu 2`.

Далі всередині Ubuntu:

```bash
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker "$USER"
```

Щоб daemon стартував автоматично при кожному запуску WSL, увімкнути systemd - додати в `/etc/wsl.conf`:

```
[boot]
systemd=true
```

З PowerShell: `wsl --shutdown`. Знов відкрити Ubuntu - це одночасно підхопить нову групу `docker` і стартоне daemon.

Перевірка:

```bash
docker run hello-world
```

**Важливо:** код тримати в `~/projects/...` всередині WSL, не на `C:\`. Через `/mnt/c/` IO в рази повільніше, VS Code буде нестерпно гальмувати.

## Як взяти код

Спочатку створити власний репозиторій через `Use this template` на сторінці
курс-репо в GitHub. Потім клонувати власний репозиторій у WSL (не на `C:\`).
В Ubuntu-терміналі:

```bash
sudo apt update && sudo apt install -y git

git config --global user.name "Your Name"
git config --global user.email "you@example.com"

mkdir -p ~/projects
cd ~/projects
git clone https://github.com/<your-username>/cpp-miltech.git
cd cpp-miltech
code .
```

VS Code запропонує встановити Dev Containers extension (із `.vscode/extensions.json`) - погодитись. Якщо промпт не зявився: `Ctrl+Shift+X`, пошук `Dev Containers`, Install.

Далі VS Code запропонує `Reopen in Container` - теж погодитись.

## Якщо щось не те

`Cannot create process, error code: 193` - проект відкритий з Windows, а не з WSL. `code .` треба робити з Ubuntu-терміналу, не з Windows Explorer.

`No space left on device` у шляху `/root/...` - встановлена тільки службова WSL-дистрибуція `docker-desktop`, без окремої Ubuntu. Це by design, в неї не можна ставити VS Code Server. Фікс: `wsl --install -d Ubuntu` з admin PowerShell, потім клонувати в нову Ubuntu.

Внизу ліворуч у VS Code нема префіксу `WSL: Ubuntu` - сесія VS Code не в WSL. Закрити, зайти в Ubuntu через `wsl`, `cd` у папку проекту, `code .` звідти.

`${localEnv:HOME}` / `${localEnv:USER}` у логах Dev Containers - той самий симптом: на Windows ці змінні порожні, треба відкривати з WSL де вони визначені.
