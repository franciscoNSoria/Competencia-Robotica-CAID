# Guía: primeros pasos con el repositorio

Del repo vacío a tener los STL subidos. Hacer en orden. Cada comando lleva su explicación.

## 0. Requisitos (una vez por PC)

```bash
git --version                                     # cualquier versión 2.x sirve
git config --global user.name "Nombre Apellido"   # nombre que aparece en los commits
git config --global user.email "mail@de-github"   # el mismo mail de la cuenta de GitHub
gh auth login                                     # inicia sesión en GitHub desde la terminal (elegir HTTPS)
```

`gh` es la GitHub CLI. Si no está instalada, ver la sección 1.1 de `github-projects.md`. Con
`gh auth login` git usa esas credenciales y no pide la contraseña en cada push.

## 1. Clonar (una vez por PC)

```bash
mkdir -p ~/caid && cd ~/caid
git clone https://github.com/franciscoNSoria/Competencia-Robotica-CAID.git
cd Competencia-Robotica-CAID
```

- `mkdir -p ~/caid` crea una carpeta para el proyecto en tu home. `-p` no da error si ya existe.
- `git clone` descarga el repo y crea la carpeta `Competencia-Robotica-CAID` con el historial
  completo. Además deja configurado `origin`, el nombre del remoto en GitHub.
- Si el repo todavía está vacío, git avisa *"You appear to have cloned an empty repository"*. Es
  normal.
- No clones dentro de carpetas sincronizadas (Dropbox, OneDrive, Google Drive): la sincronización
  corrompe la carpeta `.git`.

## 2. Primer commit y ramas (solo el dueño del repo, una única vez)

```bash
git commit --allow-empty -m "chore: inicio del repositorio"
git push -u origin main
git switch -c dev
git push -u origin dev
```

- `--allow-empty` crea un commit sin archivos. Sirve como punto de partida común para todas las
  ramas.
- `git push -u origin main` sube la rama `main`. `-u` la vincula con la del remoto: después alcanza
  con `git push` o `git pull`.
- `git switch -c dev` crea la rama `dev` a partir de `main` y se cambia a ella.

## 3. `dev` como rama por defecto (solo el dueño, una única vez)

En GitHub: **"Settings"** → **"Default branch"** → ícono de las dos flechas → elegir `dev` →
**"Update"** → **"I understand, update the default branch."**

Por qué: GitHub solo cierra una tarea con `Closes #n` cuando el PR apunta a la rama por defecto.
Los formularios de issue y la plantilla de PR también funcionan solo desde esa rama. Nuestros PR
van a `dev`, así que `dev` tiene que ser la rama por defecto. `main` sigue siendo la rama de
versiones (ver `CONTRIBUTING.md`).

## 4. Traer la estructura base

La estructura base del repo (carpetas, README, guías, plantillas) llega por un pull request desde
la rama de trabajo de Claude. Para revisarla:

1. Abrir el PR en GitHub (pestaña **"Pull requests"**).
2. Mirar los archivos en **"Files changed"**.
3. Si está bien: **"Squash and merge"** → **"Confirm squash and merge"**.

Después, en la PC:

```bash
git switch dev
git pull
```

`git pull` trae los cambios de GitHub a tu copia local.

## 5. Subir los STL (primera tarea con el flujo completo)

```bash
git switch dev && git pull                        # partir siempre de dev actualizado
git switch -c feature/HW-01-stl-originales        # rama nueva para esta tarea
cp /ruta/donde/estan/los/stl/*.stl hardware/stl/  # copiar los STL originales
git status                                        # ver qué archivos nuevos detecta git
git add hardware/stl/                             # marcarlos para el próximo commit
git commit -m "feat(hardware): STL originales de la organización"
git push -u origin feature/HW-01-stl-originales   # subir la rama a GitHub
```

Después, en GitHub:

1. Aparece un banner amarillo con **"Compare & pull request"**. Hacer clic.
2. Verificar que **base** sea `dev` y **compare** sea tu rama.
3. Completar la plantilla y hacer clic en **"Create pull request"**.
4. Revisión y **"Squash and merge"**.

Si los STL tienen extensión en mayúsculas (`.STL`), usar `cp /ruta/*.STL hardware/stl/`. Si algún
archivo pesa más de 50 MB, avisar antes de subirlo: GitHub rechaza archivos de más de 100 MB.

## 6. Verificar el entorno

Con la estructura base ya en `dev`:

```bash
source /opt/ros/jazzy/setup.bash
bash tools/setup/check_env.sh
```

Lo que salga como `[FALTA]` se instala con el comando que muestra el script (ver
`docs/guias/entorno.md`).

## 7. Siguiente: el tablero

Seguir `docs/guias/github-projects.md` desde el paso 3 (labels y milestones) para armar el tablero
de GitHub Projects.
