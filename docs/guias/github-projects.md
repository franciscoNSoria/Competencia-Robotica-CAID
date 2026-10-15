# Guía: GitHub Projects y flujo de trabajo

Repo: `franciscoNSoria/Competencia-Robotica-CAID` (público, cuenta personal, GitHub Free).
Verificado el 14/10/2026 contra el código fuente de la documentación oficial (`github/docs`, commit `bcb11fe`) y de GitHub CLI (`cli/cli`, rama `trunk`, última versión etiquetada `v2.102.0`).

- Lo que va entre comillas, como **"New project"**, es el texto exacto de la interfaz de GitHub, que está en inglés.
- Lo marcado **[no verificado]** no figura en la documentación oficial. Comprobalo en pantalla la primera vez.

---

## 0. Cómo se traduce nuestro método a GitHub

| Lo nuestro | Cómo se arma en GitHub |
|---|---|
| Tarea | Un issue creado con el formulario **Tarea** |
| Columna (carril) | Campo **Columna** del Project, de tipo single select, con una opción por carril |
| Lugar de la tarea en la cadena | Campo numérico **Orden** (1, 2, 3...) |
| Avance | Campo **Status**: Pendiente → En curso → En revisión → Hecha |
| Unión que habilita una columna nueva | Dependencia "blocked by": la primera tarea de la columna nueva queda bloqueada por la última tarea de cada columna que necesita |
| Microtareas | Lista de casillas en el cuerpo del issue. Usá sub-issues solo si una microtarea merece su propio PR |
| Hito de integración | Milestone, después PR de `dev` a `main` y por último un tag |
| Código de una tarea | Rama `feature/<ID>-<desc>`, PR a `dev`, 1 aprobación y **"Squash and merge"** |

Seguí la guía en orden:

1. Tu máquina (paso 1).
2. El repositorio (pasos 2 a 4).
3. El Project (pasos 5 a 8).
4. La carga de tareas (paso 9).
5. La protección de ramas (paso 10).
6. El trabajo de todos los días (pasos 11 y 12).

---

## 1. Preparar tu máquina (una sola vez)

### 1.1 Instalar GitHub CLI (`gh`)

Instalalo desde el repositorio oficial de GitHub CLI para Debian y Ubuntu (comandos de `cli/cli/docs/install_linux.md`):

```bash
sudo mkdir -p -m 755 /etc/apt/keyrings
out=$(mktemp) && wget -nv -O$out https://cli.github.com/packages/githubcli-archive-keyring.gpg
cat $out | sudo tee /etc/apt/keyrings/githubcli-archive-keyring.gpg > /dev/null
sudo chmod go+r /etc/apt/keyrings/githubcli-archive-keyring.gpg
sudo mkdir -p -m 755 /etc/apt/sources.list.d
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
sudo apt update && sudo apt install gh -y
```

### 1.2 Revisar la versión

Corré `gh --version`. Esta guía necesita:

- **2.94.0 o más nueva** para los flags `--parent`, `--blocked-by` y `--blocking` de `gh issue create`, y para `--add-sub-issue`, `--add-blocked-by` y similares de `gh issue edit`. Llegaron en v2.94.0, publicada el 10/06/2026.
- **2.97.0 o más nueva** para `gh project item-edit ... --field "Status" --value "En curso"`, es decir, para cargar campos usando nombres en lugar de IDs. Llegó en v2.97.0, publicada el 30/07/2026.

### 1.3 Iniciar sesión

```bash
gh auth login
gh auth refresh -s project     # los comandos `gh project` necesitan el scope "project"
gh auth status
```

### 1.4 Variables que usa esta guía

```bash
export OWNER=franciscoNSoria
export REPO=franciscoNSoria/Competencia-Robotica-CAID
```

---

## 2. Preparar el repositorio

> Esta sección ya se hizo una vez al crear el repo (ver el paso a paso de `docs/guias/primeros-pasos.md`).
> Queda documentada para entender por qué el repo está configurado así.

### 2.1 Crear `main` y `dev`

El repo nació vacío. El primer commit se hizo en `main` y desde ahí salió `dev`:

```bash
git commit --allow-empty -m "chore: inicio del repositorio"
git push -u origin main
git switch -c dev
git push -u origin dev
```

### 2.2 Hacer que `dev` sea la rama por defecto (importante)

`main` sigue siendo la rama de versiones. Solo recibe PR desde `dev` en cada hito, con su tag. Hay que cambiar la rama por defecto por estas tres razones, las tres verificadas en la documentación:

1. **Cierre automático de issues.** GitHub interpreta `Closes #n` solo cuando el PR apunta a la **rama por defecto**. Si apunta a otra rama, "these keywords are ignored". Con `main` por defecto, nuestros PR a `dev` **no cerrarían las tareas** y las tarjetas no pasarían solas a Hecha.
2. **Plantillas.** El formulario de issue, `config.yml` y la plantilla de PR se activan solo cuando están en la rama por defecto.
3. **Ramas creadas desde un issue.** **"Create a branch"** crea la rama a partir de la rama por defecto.

Pasos en la web (necesitás ser admin, y el repo tiene que tener más de una rama):

1. En el repo, hacé clic en **"Settings"**. Si no ves la pestaña, buscala en el menú ⋯.
2. En **"Default branch"**, a la derecha del nombre, hacé clic en el ícono de dos flechas.
3. Elegí `dev`.
4. Hacé clic en **"Update"** y después en **"I understand, update the default branch."**

### 2.3 Elegir cómo se mergean los PR

Hacelo en **"Settings"**, página **"General"**, sección **"Pull Requests"**:

- Activá **"Allow merge commits"**. Es el método para `dev` → `main`.
- Activá **"Allow squash merging"**. Es el método para tarea → `dev`. En su desplegable elegí el mensaje por defecto con título y descripción del PR. La documentación nombra esa opción "pull request title and description"; el texto exacto del menú **[no verificado]**.
- Desactivá **"Allow rebase merging"**.
- Activá **"Automatically delete head branches"**. Borra la rama de la tarea al mergear. `dev` y `main` no se borran porque las protege la regla **"Restrict deletions"** del paso 10 (la documentación dice "repository rules can also prevent branches being automatically deleted").

¿Por qué `dev` → `main` va con merge commit y no con squash? La documentación aclara que squash "works best for short-lived branches". En ramas de vida larga, como `dev`, aconseja merge commit para no resolver los mismos conflictos una y otra vez.

Lo mismo por CLI:

```bash
gh repo edit "$REPO" --default-branch dev --enable-merge-commit --enable-squash-merge --enable-rebase-merge=false --delete-branch-on-merge
```

### 2.4 Invitar al equipo

1. Entrá a **"Settings"**.
2. En la sección **"Access"** de la barra lateral, hacé clic en **"Collaborators"**.
3. Hacé clic en **"Add people"**.
4. Buscá el usuario y hacé clic en **"Add NAME to REPOSITORY"**.
5. Cada persona acepta la invitación que le llega por mail.

Qué tenés que saber:

- Un repo personal tiene solo dos niveles: dueño (vos) y colaboradores.
- Los colaboradores pueden:
  - leer y hacer push;
  - crear, mergear y cerrar PR;
  - gestionar labels y milestones;
  - aprobar PR, y su aprobación cuenta para habilitar el merge.
- Solo el dueño cambia settings, rulesets y la rama por defecto.
- GitHub limita cuántas invitaciones podés mandar en 24 h. No publica el número.

---

## 3. Labels y milestones

### 3.1 Labels

Usá pocas, y solo para lo que el campo Columna no dice:

| Label | Para qué sirve |
|---|---|
| `tarea` | La lleva todo issue creado con el formulario. La pone el formulario solo |
| `necesita-robot` | La tarea solo se puede probar con el robot físico. Sirve para filtrar mientras se termina de armar |
| `espera-externa` | Espera algo de afuera: respuesta de organizadores, compras o impresión 3D |
| `stretch` | Solo si sobra tiempo (por ejemplo, YOLO) |
| `columna` | Marca el issue padre que agrupa una columna. Solo si usás el paso 9.3 |

Conservá las labels que trae GitHub por defecto (`bug`, `documentation`). Las demás podés borrarlas.

Crear una label en la web:

1. Andá a **Issues** y hacé clic en **"Labels"**.
2. Hacé clic en **"New label"**.
3. Completá **"Label name"**, **"Description"** y el color.
4. Hacé clic en **"Create label"**.

Por CLI (`--force` actualiza la label si ya existe):

```bash
gh label create tarea          -R "$REPO" --color 1D76DB --description "Tarea del tablero" --force
gh label create necesita-robot -R "$REPO" --color D93F0B --description "Solo se prueba con el robot fisico" --force
gh label create espera-externa -R "$REPO" --color FBCA04 --description "Espera organizadores, compras o impresion 3D" --force
gh label create stretch        -R "$REPO" --color C5DEF5 --description "Solo si sobra tiempo" --force
gh label create columna        -R "$REPO" --color 5319E7 --description "Issue padre de una columna" --force
```

> El formulario solo aplica labels que **ya existen**. Por eso creá las labels antes de usar el formulario.

### 3.2 Milestones: un milestone por hito de integración

Crear un milestone en la web:

1. Andá a **Issues** y hacé clic en **"Milestones"**.
2. Hacé clic en **"New Milestone"**.
3. Completá título, descripción y fecha.
4. Hacé clic en **"Create milestone"**.

Por CLI (API REST). `due_on` va en UTC con formato `YYYY-MM-DDTHH:MM:SSZ`:

```bash
gh api repos/$REPO/milestones -f title="H1 Base teleoperada"   -f due_on="2026-10-19T23:59:59Z" -f description="Demo: teleoperacion con video"
gh api repos/$REPO/milestones -f title="H2 Autonomia minima"   -f due_on="2026-10-24T23:59:59Z" -f description="..."
gh api repos/$REPO/milestones -f title="H3 Mision completa"    -f due_on="2026-10-28T23:59:59Z" -f description="..."
gh api repos/$REPO/milestones -f title="H4 Version 1.0"        -f due_on="2026-10-31T23:59:59Z" -f description="..."
```

Nombres y fechas son **ejemplos**: ajustalos al plan. `23:59:59Z` equivale a las 20:59:59 en Córdoba (UTC−3). Cada hito se cierra con el procedimiento del paso 12.

---

## 4. Plantillas: formulario "Tarea", `config.yml` y plantilla de PR

Van en estas rutas:

- `.github/ISSUE_TEMPLATE/1-tarea.yml`. El número adelante fija el orden en el selector de plantillas.
- `.github/ISSUE_TEMPLATE/config.yml`
- `.github/pull_request_template.md`. Ya existe en el repo y trae `Closes #`.

Tienen que estar en `dev`, la rama por defecto. Subilas **antes** de activar los rulesets del paso 10, o después por PR.

### 4.1 Reglas de sintaxis

Todas verificadas en la documentación:

- **Claves obligatorias:** `name`, `description` y `body`.
  - `name` tiene que tener más de 3 caracteres y no repetirse con otra plantilla.
- **Claves opcionales:**
  - `title`, `labels` y `assignees`;
  - `projects`, con formato `OWNER/NUMERO`. Quien abre el issue necesita permiso de escritura en ese Project;
  - `type`, que no sirve acá porque los issue types son de organizaciones.
- **Tipos de campo:** `markdown`, `textarea`, `input`, `dropdown`, `checkboxes` y `upload`.
- **`id`:** solo letras, números, `-` y `_`, sin repetirse.
- **`validations: required: true`:** funciona solo en repos públicos. El nuestro es público.
- **`dropdown`:** las `options` no se repiten y no pueden incluir "None".
- **Valores booleanos:** no uses `yes`, `no`, `true`, `false`, `on` u `off` sin comillas.
- **Estado de la función:** los issue forms están en "public preview" y pueden cambiar.

### 4.2 `.github/ISSUE_TEMPLATE/1-tarea.yml`

Este archivo se validó con un parser YAML.

```yaml
name: Tarea
description: Una tarea de una columna del tablero (se hace en una rama y se cierra con un PR).
title: "[COL-00] "
labels: ["tarea"]
# projects: ["franciscoNSoria/1"]   # opcional: solo si todos tienen permiso Write en el Project
body:
  - type: markdown
    attributes:
      value: |
        Completá todo. Después de crear la tarea, en la barra lateral **Projects** cargá los campos
        **Columna** y **Orden** del Project (el desplegable de abajo no llena el campo del Project).
  - type: dropdown
    id: columna
    attributes:
      label: Columna
      description: Carril al que pertenece la tarea. Tiene que coincidir con una opción del campo "Columna" del Project.
      options:
        - FW Firmware ESP32
        - CAM Camara y video
        - PER Percepcion
        - SIM Simulacion
        - NAV Navegacion
        - MAN Brazo y pinza
        - DOC Memoria y presentacion
        - INT Integracion
    validations:
      required: true
  - type: textarea
    id: para-que
    attributes:
      label: Para qué
      description: Qué problema resuelve y qué otra tarea o columna usa este resultado.
      placeholder: "Ej.: publica /wheel_ticks para que NAV pueda calcular odometría."
    validations:
      required: true
  - type: textarea
    id: microtareas
    attributes:
      label: Microtareas
      description: Pasos chicos, cada uno verificable. Se tildan a medida que se hacen.
      value: |
        - [ ] 
        - [ ] 
        - [ ] 
    validations:
      required: true
  - type: textarea
    id: criterio
    attributes:
      label: Criterio de terminado
      description: Qué tiene que pasar para mover la tarea a Hecha.
      value: |
        - [ ] Compila (`colcon build` y/o `pio run`) y pasa el CI
        - [ ] Probado (en simulación o en el robot) y explicado en el PR
        - [ ] Documentado en: 
    validations:
      required: true
  - type: textarea
    id: dependencias
    attributes:
      label: Dependencias
      description: "Tareas que tienen que estar Hechas antes (#número). Cargalas también en la barra lateral: Relationships > Mark as blocked by."
      placeholder: "Ej.: #12, #15 (o 'Ninguna')"
    validations:
      required: false
```

Tené en cuenta:

- Las opciones de "Columna" son un **ejemplo**. Usá exactamente los mismos nombres que pongas en el campo Columna del Project (paso 6.2). Conviene que no tengan tildes ni comas, así se escriben fácil en los filtros y en `gh`.
- **Limitación importante:** lo que se elige en el desplegable "Columna" del formulario solo queda como texto en el cuerpo del issue. **No carga el campo Columna del Project**, porque un formulario no puede escribir campos del Project. Ese campo se carga después de crear la tarea (paso 9.1) o con `gh` (paso 9.6).
- Cuando se habilite una columna nueva, agregá la opción en **dos lugares**: en el campo del Project y en `options` de este archivo.

### 4.3 `.github/ISSUE_TEMPLATE/config.yml`

```yaml
blank_issues_enabled: false
contact_links:
  - name: Reglamento del desafío
    url: https://github.com/franciscoNSoria/Competencia-Robotica-CAID/blob/dev/docs/reglamento/desafio-laberinto.md
    about: Revisá el reglamento antes de abrir una tarea nueva.
```

Con `blank_issues_enabled: false`, quien tiene permiso Write igual ve la opción **"Blank issue"**, marcada como "Maintainers only". Como todos ustedes son colaboradores, todos la van a ver. Eso no es un error.

### 4.4 Plantilla de PR

La plantilla `.github/pull_request_template.md` ya está en el lugar correcto. Si algún día querés varias plantillas, van en `.github/PULL_REQUEST_TEMPLATE/`.

---

## 5. Crear el Project y vincularlo

### 5.1 Crear el Project

En la web:

1. Hacé clic en tu foto (arriba a la derecha) y después en **"Your profile"**.
2. Abrí la pestaña **"Projects"** y hacé clic en **"New project"**.
3. En **"Start from scratch"**, elegí **"Board"**. Hay plantillas, pero no hacen falta.
4. En **"Project name"**, escribí `CAID 2026 Laberinto`.
5. Hacé clic en **"Create project"**.

Por CLI:

```bash
gh project create --owner "@me" --title "CAID 2026 Laberinto"
```

El comando imprime la URL del Project. El número es lo último de la URL (`.../users/franciscoNSoria/projects/1`).

### 5.2 Vincular el Project al repo

En la web:

1. En el repo, abrí la pestaña **"Projects"**.
2. Hacé clic en **"Link a project"**.
3. Buscá el Project y hacé clic en él.

Por CLI:

```bash
gh project link 1 --owner "$OWNER" --repo Competencia-Robotica-CAID
```

### 5.3 Fijar el repositorio por defecto

Así los issues que se crean desde el Project van a este repo:

1. En el Project, abrí el menú ⋯ (arriba a la derecha) y hacé clic en **"Settings"**.
2. En **"Default repository"**, elegí el repo.
3. Hacé clic en **"Save changes"**.

### 5.4 Descripción y README del Project

1. En **"Settings"**, escribí la descripción en **"Add a description"** y hacé clic en **"Save"**.
2. En **"README"**, poné los enlaces a `CONTRIBUTING.md` y a esta guía, y hacé clic en **"Save"**.

### 5.5 Dar acceso al equipo

Ser colaborador del repo **no** da permiso de escritura en tu Project personal. Hay que invitar a cada persona aparte:

1. En **"Settings"**, hacé clic en **"Manage access"**.
2. En **"Invite collaborators"**, buscá a la persona.
3. Elegí el rol **"Write"**. Los roles son Read, Write y Admin.
4. Hacé clic en **"Invite"**.

### 5.6 Visibilidad

Se cambia en **"Settings"**, sección **"Danger zone"**, opción **"Visibility"**: **"Private"** o **"Public"**.

Te sugiero **Public**: el repo ya es público y el tablero muestra el proceso, algo útil para la memoria técnica. La decisión es tuya.

---

## 6. Campos

Límites: hasta **50 campos** por Project (los del sistema también cuentan) y hasta **50 opciones** por campo single select.

### 6.1 Status (viene incluido)

1. Abrí el menú ⋯, hacé clic en **"Settings"** y después en el campo **"Status"**.
2. Hacé clic en el lápiz de cada opción. Podés cambiar **"Label text"**, **"Color"** y **"Description"**. Hacé clic en **"Save"**.
3. Renombrá las opciones:
   - "Todo" → `Pendiente`;
   - "In Progress" → `En curso`;
   - "Done" → `Hecha`.

   Los nombres originales están **parcialmente verificados**: la documentación usa "Todo", "In progress" y "Done".
4. Agregá la opción `En revisión` entre En curso y Hecha.
5. Elegí `Pendiente` como **valor por defecto**. Los campos single select aceptan un default que se aplica a cada ítem nuevo.
6. Después de renombrar, revisá los workflows del paso 8 y confirmá que pongan "Hecha" **[no verificado que se actualicen solos]**.

### 6.2 Columna (single select)

En la web:

1. Abrí una vista de tabla.
2. Hacé clic en **"+"**, en el encabezado de más a la derecha.
3. Hacé clic en **"New field"** y escribí `Columna`.
4. Elegí **"Single select"**.
5. Cargá las opciones en **"Options"**, usando **"Add option"** para cada una.
6. Hacé clic en **"Save"**.

Por CLI:

```bash
gh project field-create 1 --owner "@me" --name "Columna" --data-type SINGLE_SELECT --single-select-options "FW Firmware ESP32,CAM Camara y video,PER Percepcion,SIM Simulacion,NAV Navegacion,MAN Brazo y pinza,DOC Memoria y presentacion,INT Integracion"
```

Las opciones se separan con comas, así que no uses comas dentro de los nombres. El color y la descripción de cada opción solo se cambian desde la web.

### 6.3 Orden (número)

Mismo camino que 6.2, pero elegí **"Number"**. Por CLI:

```bash
gh project field-create 1 --owner "@me" --name "Orden" --data-type NUMBER
```

### 6.4 Inicio y Fin (fecha, opcional, para el roadmap)

```bash
gh project field-create 1 --owner "@me" --name "Inicio" --data-type DATE
gh project field-create 1 --owner "@me" --name "Fin" --data-type DATE
```

Los campos de fecha no admiten valor por defecto.

### 6.5 Iteration (no lo recomiendo acá)

Este campo sirve para bloques de tiempo repetidos:

- Se configura en la web: **"New field"**, después **"Iteration"**, **"Starts on"** y la duración en **"days"** o **"weeks"**.
- Al crearlo arma tres iteraciones automáticamente.

Para 18 días no aporta. Además, `gh project field-create` solo crea campos TEXT, SINGLE_SELECT, DATE y NUMBER.

### 6.6 Text (opcional)

Sirve para notas libres. Se filtra con `campo:"texto exacto"`.

### 6.7 Mostrar campos ocultos útiles

1. En la tabla, hacé clic en **"+"**.
2. En **"Hidden fields"**, activá **"Parent issue"** y **"Sub-issue progress"**.

---

## 7. Vistas

Todo se configura con el botón **"View"** (ícono de engranaje) que está al lado del buscador de la vista:

- **Vista nueva:** **"New view"**, a la derecha de las pestañas.
- **Guardar cambios:** cuando una vista tiene cambios sin guardar aparece un punto al lado del nombre. Guardalos con **"View"** → **"Save changes"**.
- **Renombrar o duplicar:** **"Rename view"** o **"Duplicate view"**.

### 7.1 Vista "Columnas" (la principal)

1. En **"Layout"**, elegí **"Board"**.
2. En **"Column field"**, elegí `Columna`.
3. En **"Sort"**, elegí `Orden`.
4. En **"Fields"**, mostrá Status, Assignees, Labels y Orden.
5. En **"Field sum"**, activá el conteo de tarjetas por columna.
6. Si usás issues padre (paso 9.3), filtrá con `-label:columna`.

Si la vista está ordenada, no podés reordenar a mano dentro de una columna. Está bien: manda Orden. Tené en cuenta que arrastrar una tarjeta a otra columna **le cambia la Columna**.

### 7.2 Vista "Kanban"

1. En **"Layout"**, elegí **"Board"**.
2. En **"Column field"**, elegí `Status`.
3. En **"Group by"**, elegí `Columna`. Así cada carril queda como una franja horizontal.

Arrastrar una tarjeta entre columnas le cambia el Status.

Podés poner un límite en "En curso": abrí el menú ⋯ de esa columna y usá **"Set column limit"** (por ejemplo, 5). No bloquea nada, solo resalta la columna cuando se pasa del límite.

### 7.3 Vista "Tabla"

1. En **"Layout"**, elegí **"Table"**.
2. En **"Group by"**, elegí `Columna`.
3. En **"Sort"**, elegí `Orden`.
4. Mostrá Status, Assignees, Milestone, Linked pull requests, Parent issue y Sub-issue progress.
5. En **"Slice by"**, elegí Labels. Sirve para ver de un vistazo las tareas `necesita-robot`.

No se puede agrupar por título, labels, reviewers ni linked pull requests. Tampoco se puede usar "Slice by" con título, reviewers ni linked pull requests.

### 7.4 Vista "Mis tareas"

Usá tabla o board con este filtro:

```
assignee:@me -status:Hecha
```

### 7.5 Vista "Roadmap" (opcional)

1. En **"Layout"**, elegí **"Roadmap"**.
2. En **"Date fields"**, poné **"Start date"** = Inicio y **"Target date"** = Fin.
3. En **"Markers"**, activá los milestones.
4. Con la lupa, elegí el zoom **"Month"**.

Solo sirve si cargan las fechas de Inicio y Fin.

### Filtros útiles

| Para ver | Filtro |
|---|---|
| Tareas en curso | `status:"En curso"` |
| Una columna | `columna:"NAV Navegacion"` |
| Tareas que necesitan el robot | `label:necesita-robot` |
| Tareas de un hito | `milestone:"H1 Base teleoperada"` |
| Tareas sin asignar | `no:assignee` |
| Sub-issues de un issue padre | `parent-issue:franciscoNSoria/Competencia-Robotica-CAID#5` |

Para negar un filtro, poné un guion adelante (`-status:Hecha`). Para pedir varios valores del mismo campo, separalos con coma (`label:a,b`).

---

## 8. Automatizaciones (Workflows)

Para editar un workflow:

1. Abrí el menú ⋯ y hacé clic en **"Workflows"**.
2. En **"Default workflows"**, elegí uno.
3. Hacé clic en **"Edit"**, configuralo y hacé clic en **"Save and turn on workflow"**.

### 8.1 Workflows activos desde el principio

Hay dos: "issue o PR cerrado → Status Done" y "PR mergeado → Status Done". En la lista suelen llamarse "Item closed" y "Pull request merged" **[nombres exactos no verificados]**. Después de renombrar Status, revisá que digan **Hecha**.

### 8.2 "Item added to project"

1. En **"When"**, marcá `issues`.
2. En **"Set"**, elegí **Status: Pendiente**.
3. Activalo.

Si ya pusiste `Pendiente` como valor por defecto (6.1), este paso es redundante pero no molesta.

### 8.3 "Auto-add to project"

1. Hacé clic en **"Edit"**.
2. En **"Filters"**, elegí el repo `Competencia-Robotica-CAID` y escribí el filtro `is:issue`.
3. Hacé clic en **"Save and turn on workflow"**.

Detalles:

- Solo agregamos issues. El PR igual aparece vinculado dentro del issue, así que no hace falta una tarjeta aparte.
- **GitHub Free permite 1 solo workflow de auto-add.**
- No agrega issues que ya existían, solo los nuevos o los que se modifican. Para los anteriores usá el paso 9.2.
- Filtros admitidos: `is`, `label`, `reason`, `assignee` y `no`.

Los cambios que hace una automatización aparecen en el historial del issue a nombre de **@github-project-automation**.

---

## 9. Cargar las tareas

### 9.1 Una tarea desde la web

1. Andá a **Issues**, hacé clic en **"New issue"** y elegí **"Tarea"**.
2. Completá el formulario y hacé clic en **"Create"**.
3. En la barra lateral, en **"Projects"**, cargá **Columna** y **Orden**. El auto-add ya agregó la tarea al Project; si no lo hizo, elegí el Project ahí mismo.

Atajo: en la vista "Columnas", usá el **"+"** de abajo de una columna y elegí **"Create new issue"**. Las vistas agrupadas por un campo le asignan ese valor al issue nuevo, así que este atajo debería cargar la Columna sola.

### 9.2 Agregar issues que ya existen

Tenés tres caminos:

- En el Project, usá el **"+"** de la última fila, elegí **"Add item from repository"**, seleccioná los issues y hacé clic en **"Add selected items"**.
- En la lista de Issues del repo, tildá los issues, hacé clic en **"Projects"** y elegí el Project.
- Pegá la URL del issue en la última fila del Project.

El límite es 50.000 ítems por Project.

### 9.3 Sub-issues (opcional)

Cómo crearlos en la web:

- Desde el issue padre, al pie de la descripción, hacé clic en **"Create sub-issue"**.
- Para sumar un issue que ya existe, usá la flecha al lado de **"Create sub-issue"** y elegí **"Add existing issue"**.
- Con **"Create more sub-issues"** podés cargar varios seguidos.

Límites y permisos: hasta **100 sub-issues por padre** y hasta **8 niveles** de anidamiento. Hace falta permiso triage o más; los colaboradores lo tienen.

Dos usos que tienen sentido acá:

- **Un issue padre por columna.** Llevan la label `columna` y sus tareas son los sub-issues. El campo "Sub-issue progress" muestra cuánto avanzó cada carril.
- **Partir una tarea grande** en sub-tareas que tengan su propio PR.

Para microtareas chicas alcanza con la lista de casillas del cuerpo del issue.

Por CLI:

```bash
gh issue create -R "$REPO" --title "..." --body "..." --parent 5   # nace como hijo del #5
gh issue edit 5 -R "$REPO" --add-sub-issue 12,13                   # suma hijos a un padre
gh issue edit 5 -R "$REPO" --remove-sub-issue 13                   # quita un hijo
gh issue edit 12 -R "$REPO" --remove-parent                        # el #12 deja de tener padre
```

### 9.4 Dependencias "blocked by" y "blocking"

Están disponibles en GitHub Free y requieren permiso triage o más.

- **Marcar una dependencia:** abrí el issue y, en la barra lateral derecha, hacé clic en **"Relationships"**. Elegí **"Mark as blocked by"** o **"Mark as blocking"**. Después buscá y seleccioná los issues.
- **Quitar una dependencia:** en **"Relationships"**, elegí **"Change blocked by"** o **"Change blocking"** y deseleccioná el issue.
- Un issue bloqueado muestra el ícono **"Blocked"** en el tablero y en la lista de Issues.

Por CLI:

```bash
gh issue create -R "$REPO" --title "..." --body "..." --blocked-by 12,15
gh issue edit 20 -R "$REPO" --add-blocked-by 12,15
gh issue edit 20 -R "$REPO" --remove-blocked-by 15
gh issue view 20 -R "$REPO"                          # muestra las filas "Blocked by" y "Blocking"
gh issue view 20 -R "$REPO" --json blockedBy,blocking
```

### 9.5 Receta: habilitar una columna nueva en una unión

1. Agregá la opción nueva al campo Columna (6.2) y al YAML del formulario (4.2).
2. Creá la primera tarea de esa columna con Columna = la nueva y Orden = 1.
3. Marcala como **"Mark as blocked by"** respecto de la última tarea de cada columna que aporta algo. Por ejemplo, el #14 de FW y el #22 de NAV.
4. Mientras alguno de esos issues siga abierto, la tarjeta muestra "Blocked". Cuando los dos se cierran por sus PR, cualquiera puede tomar la tarea.

Usá "blocked by" **solo para estos cruces entre columnas**:

- Dentro de una misma columna, el Orden ya marca la secuencia.
- Para esperas externas, usá la label `espera-externa`.

### 9.6 Carga masiva por CLI (gh 2.97 o más nueva)

```bash
#!/usr/bin/env bash
set -euo pipefail
OWNER=franciscoNSoria
REPO=franciscoNSoria/Competencia-Robotica-CAID
P=1                                   # número del Project
PROJ="CAID 2026 Laberinto"            # título del Project (para --project)

num() { sed 's#.*/##'; }              # gh issue create imprime la URL; esto deja el número

campos() {                            # $1 = número de issue, $2 = Columna, $3 = Orden
  local url="https://github.com/$REPO/issues/$1"
  gh project item-edit "$P" --owner "$OWNER" --url "$url" --field "Columna" --value "$2"
  gh project item-edit "$P" --owner "$OWNER" --url "$url" --field "Orden" --value "$3"
}

# Issue padre de la columna (opcional)
PADRE=$(gh issue create -R "$REPO" --title "Columna NAV Navegacion" --label columna --project "$PROJ" --body "Carril NAV. Sus tareas son los sub-issues de este issue." | num)

# Tareas de la columna
N1=$(gh issue create -R "$REPO" --title "[NAV-01] Odometria desde ticks" --label tarea --milestone "H1 Base teleoperada" --parent "$PADRE" --project "$PROJ" --body-file tareas/NAV-01.md | num)
campos "$N1" "NAV Navegacion" 1

N2=$(gh issue create -R "$REPO" --title "[NAV-02] Seguimiento de pared" --label tarea --parent "$PADRE" --project "$PROJ" --body-file tareas/NAV-02.md | num)
campos "$N2" "NAV Navegacion" 2

# Tarea de unión: habilita INT cuando terminen FW-03 (número en $FW3) y NAV-02
J=$(gh issue create -R "$REPO" --title "[INT-01] Teleoperacion de punta a punta" --label tarea --blocked-by "$FW3,$N2" --project "$PROJ" --body-file tareas/INT-01.md | num)
campos "$J" "INT Integracion" 1
```

Cómo funciona:

- Cada `tareas/XXX.md` repite las secciones del formulario: `### Columna`, `### Para qué`, `### Microtareas`, `### Criterio de terminado` y `### Dependencias`.
- `--project` agrega el issue al Project en el momento, así `item-edit` lo encuentra.
- `--value` acepta nombres de opción sin distinguir mayúsculas. Funciona con campos single select, texto, número y fecha. No funciona con Iteration.

Si tenés una versión de gh entre 2.94 y 2.96, cargá los campos por ID:

```bash
gh project view 1 --owner "@me" --format json --jq .id          # ID del Project (PVT_...)
gh project field-list 1 --owner "@me" --format json              # IDs de campos y de opciones
ITEM=$(gh project item-add 1 --owner "@me" --url https://github.com/$REPO/issues/12 --format json --jq .id)
gh project item-edit --id "$ITEM" --project-id PVT_xxx --field-id PVTSSF_xxx --single-select-option-id xxxx
gh project item-edit --id "$ITEM" --project-id PVT_xxx --field-id PVTF_xxx --number 2
```

---

## 10. Proteger `main` y `dev` con rulesets

Los rulesets están disponibles en repos **públicos** con GitHub Free. Vamos a crear dos, uno por rama.

### 10.1 Crear el ruleset de `dev`

1. En el repo, entrá a **"Settings"**.
2. En la barra lateral, bajo **"Code, planning, and automation"**, hacé clic en **"Rulesets"**. La documentación lo indica dos veces: si se despliega un grupo, volvé a hacer clic en **"Rulesets"**.
3. Hacé clic en **"New ruleset"** y después en **"New branch ruleset"**.
4. En **"Ruleset name"**, escribí `dev protegida`.
5. Cambiá el estado de **"Disabled"** (el valor por defecto) a **"Active"**. Si te lo olvidás, el ruleset existe pero no aplica nada.
6. En **"Bypass list"** tenés dos opciones:
   - Dejarla **vacía**. Es la opción recomendada.
   - Agregarte para emergencias: hacé clic en **"Add bypass"**, elegí el rol de administrador del repo y hacé clic en **"Add Selected"**. Después, en el menú ⋯ que está al lado de **"Always allow"**, elegí **"For pull requests only"**. Así podés mergear un PR sin aprobación, pero nunca hacer push directo. El nombre del rol que aparece en la lista para un repo personal está **[no verificado]**.
7. En **"Target branches"**, hacé clic en **"Add a target"** e incluí el patrón `dev`.
8. En **"Branch protections"**, marcá estas reglas:
   - **"Restrict deletions"**. Viene marcada.
   - **"Require a pull request before merging"**, con estas opciones:
     - aprobaciones requeridas: **1**;
     - método de merge permitido: **solo Squash**.

     Las etiquetas exactas de esas opciones están **parcialmente verificadas**. La API las llama `required_approving_review_count` (de 0 a 10) y `allowed_merge_methods`. Dejá sin marcar las demás opciones: descartar aprobaciones viejas con commits nuevos, review de Code Owners y aprobación del último push.
   - **"Block force pushes"**. Viene marcada.
   - **"Require status checks to pass"**, pero **solo cuando ya exista el CI**:
     - Agregá los nombres de los jobs (por ejemplo, `colcon-build` y `pio-build`) y confirmá con **"+"**.
     - Un check solo se puede elegir si ya terminó bien en este repo en los últimos 7 días.
     - Los nombres de los jobs tienen que ser únicos.
     - Dejá sin marcar **"Require branches to be up to date before merging"**: obliga a actualizar las ramas cada vez que alguien mergea.
9. **No** marques estas reglas:
   - **"Restrict updates"**: solo dejaría escribir a quien esté en la Bypass list, así que bloquearía los merges;
   - **"Require signed commits"**;
   - **"Require linear history"** en `main`.
10. Hacé clic en **"Create"**.

### 10.2 Crear el ruleset de `main`

Repetí los mismos pasos con dos cambios: el nombre es `main protegida` y el patrón es `main`. En el método de merge permitido, elegí **solo Merge**.

### 10.3 Alternativa por CLI

`gh api repos/{owner}/{repo}/rulesets --input file.json` es un ejemplo de la propia ayuda de gh. Guardá esto como `ruleset-dev.json`:

```json
{
  "name": "dev protegida",
  "target": "branch",
  "enforcement": "active",
  "conditions": { "ref_name": { "include": ["refs/heads/dev"], "exclude": [] } },
  "rules": [
    { "type": "deletion" },
    { "type": "non_fast_forward" },
    { "type": "pull_request",
      "parameters": {
        "required_approving_review_count": 1,
        "dismiss_stale_reviews_on_push": false,
        "require_code_owner_review": false,
        "require_last_push_approval": false,
        "required_review_thread_resolution": false,
        "allowed_merge_methods": ["squash"]
      } }
  ]
}
```

Después corré:

```bash
gh api repos/$REPO/rulesets --input ruleset-dev.json
```

Para `main`, copiá el archivo cambiando `dev` por `main` y `["squash"]` por `["merge"]`.

Cuando exista el CI, agregá esta regla a la lista `rules`:

```json
{ "type": "required_status_checks", "parameters": { "strict_required_status_checks_policy": false, "required_status_checks": [ { "context": "colcon-build" } ] } }
```

### 10.4 Qué implica esto con 5 personas

- **El autor no puede aprobar su propio PR.** Cada merge necesita una segunda persona; hay 4 revisores posibles por PR.
- Si alguien marca **"Request changes"**, esa misma persona tiene que aprobar después, o alguien con Write tiene que descartar su review.
- Si nadie está en la Bypass list, **vos también vas a necesitar una aprobación**. Probalo con un PR de prueba el primer día.

### 10.5 Proteger los tags (opcional)

1. Hacé clic en **"New tag ruleset"**.
2. Como target, poné el patrón `v*`.
3. Marcá **"Restrict deletions"** y **"Block force pushes"**.

Así nadie borra ni mueve los tags de los hitos.

---

## 11. Cómo tomo una tarea y la llevo a Hecha

### 1. Elegir la tarea

En la vista **"Columnas"**, elegí una columna. Tomá la tarjeta en **Pendiente** con el **Orden más bajo** que **no** tenga el ícono **"Blocked"**. Si lleva `necesita-robot` y el robot todavía no está, pasá a otra columna.

### 2. Asignártela

En el issue, hacé clic en **"Assignees"** y elegí tu usuario.

```bash
gh issue edit 12 -R "$REPO" --add-assignee @me
```

### 3. Moverla a "En curso"

Arrastrala a **En curso** en la vista "Kanban", o cambiá el Status desde la sección **"Projects"** de la barra lateral del issue.

```bash
gh project item-edit 1 --owner "$OWNER" --url https://github.com/$REPO/issues/12 --field "Status" --value "En curso"
```

### 4. Crear la rama vinculada al issue

En la web:

1. En la barra lateral del issue, en **"Development"**, hacé clic en **"Create a branch"**.
2. En **"Branch name"**, escribí `feature/NAV-02-seguimiento-pared`. La rama sale de `dev` porque `dev` es la rama por defecto.
3. En **"What's next"**, elegí trabajar localmente.
4. Hacé clic en **"Create branch"**.
5. En tu PC, corré:

```bash
git fetch origin
git switch feature/NAV-02-seguimiento-pared
```

O todo en un solo paso:

```bash
gh issue develop 12 -R "$REPO" --name feature/NAV-02-seguimiento-pared --base dev --checkout
```

La rama queda listada en **"Development"** del issue.

### 5. Trabajar y hacer commits

- Hacé commits chicos con el formato `tipo(alcance): descripción`. Por ejemplo, `feat(nav): giro en U con odometria`.
- Subí seguido con `git push -u origin HEAD`.
- Tildá las microtareas del issue a medida que las terminás.
- Antes del PR, traé lo último de `dev`:

```bash
git fetch origin && git merge origin/dev
```

### 6. Abrir el PR

1. Hacé clic en **"Compare & pull request"** en el banner amarillo.
2. Controlá que base sea `dev` y compare sea tu rama.
3. Poné como título `[NAV-02] Seguimiento de pared`.
4. Completá la plantilla con **`Closes #12`**.
5. En **"Reviewers"**, elegí a otra persona.
6. Hacé clic en **"Create Pull Request"**. Si todavía no está listo, usá **"Create Draft Pull Request"**.

Por CLI:

```bash
gh pr create --base dev --web                          # termina en el navegador, con la plantilla
gh pr create --base dev --title "[NAV-02] Seguimiento de pared" --body "Closes #12 ..." --reviewer USUARIO
```

Después mové la tarjeta a **En revisión**.

### 7. Revisión (la hace otra persona)

1. En el PR, abrí **"Files changed"** y comentá las líneas que haga falta.
2. Hacé clic en **"Review changes"** y elegí **"Approve"** o **"Request changes"**.
3. Hacé clic en **"Submit review"**.

Si piden cambios, el autor hace commits nuevos en la misma rama y los sube; el PR se actualiza solo. El CI tiene que dar verde.

### 8. Mergear

Con el PR aprobado:

1. En el desplegable de merge, elegí **"Squash and merge"**.
2. Revisá el mensaje y hacé clic en **"Confirm squash and merge"**.

La rama se borra sola.

```bash
gh pr merge 34 --squash --delete-branch
```

### 9. Lo que pasa solo

1. El PR entró en `dev`, que es la rama por defecto, así que GitHub **cierra el #12** por el `Closes #12`.
2. El workflow que pasa los issues cerrados a Hecha **mueve la tarjeta a Hecha**.
3. Si la tarea tiene padre, su "Sub-issue progress" sube.
4. Las tareas que estaban bloqueadas por el #12 se liberan cuando se cierran todos sus bloqueos.

### 10. Seguir

```bash
git switch dev && git pull
```

Y volvé al paso 1.

Una tarea está **Hecha** solo si se cumple su "Criterio de terminado", incluida la documentación, y todo entró en el mismo PR.

---

## 12. Cerrar un hito (de `dev` a `main` con tag)

1. **Revisar el milestone.** En **Issues** → **"Milestones"**, controlá que el hito tenga todo cerrado y que `dev` pase la demo del hito.
2. **Abrir el PR:**

   ```bash
   gh pr create --base main --head dev --title "Hito H1: base teleoperada" --body "..." --reviewer USUARIO
   ```

3. **Mergear.** Una vez aprobado, mergeá con **"Create a merge commit"** y **"Confirm merge"**. Es el único método que permite el ruleset de `main`. `dev` no se borra porque la protege "Restrict deletions".
4. **Poner el tag:**

   ```bash
   git switch main && git pull
   git tag -a v0.2.0-h1 -m "H1: base teleoperada"
   git push origin v0.2.0-h1
   ```

5. **Cerrar el milestone:**

   ```bash
   gh api -X PATCH repos/$REPO/milestones/1 -f state=closed
   ```

---

## 13. Si algo no funciona

| Síntoma | Causa probable | Qué hacer |
|---|---|---|
| El issue no se cerró al mergear | El PR no apuntaba a la rama por defecto, o `Closes #n` no estaba en la descripción | Revisá el paso 2.2 y cerrá el issue a mano |
| El formulario o la plantilla de PR no aparecen | Los archivos no están en la rama por defecto | Subilos a `dev` |
| No puedo aprobar | Sos el autor del PR | Pedile la revisión a otra persona |
| El check no aparece en el ruleset | No terminó bien en los últimos 7 días | Corré el CI una vez y volvé a buscarlo |
| `unknown flag: --parent` | gh anterior a 2.94 | Actualizá gh (1.1) |
| `item-edit` rechaza `--field` | gh anterior a 2.97 | Actualizá gh o usá IDs (9.6) |
| El auto-add no agregó issues viejos | Solo actúa sobre issues nuevos o modificados | Agregalos con 9.2 |
| No puedo crear un segundo auto-add | GitHub Free permite uno | Usá un solo filtro, por ejemplo `is:issue` |
| La tarjeta quedó en una columna equivocada | El formulario no carga el campo Columna | Cargá Columna desde la barra lateral o con `item-edit` |

---

## Fuentes (leídas el 14/10/2026)

### Projects

- **Crear un Project:** https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/creating-projects/creating-a-project.md
- **Vincular a un repo y repositorio por defecto:** https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/managing-your-project/adding-your-project-to-a-repository.md
- **Acceso y visibilidad:**
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/managing-your-project/managing-access-to-your-projects.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/managing-your-project/managing-visibility-of-your-projects.md
- **Campos:**
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/understanding-fields/about-single-select-fields.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/understanding-fields/about-text-and-number-fields.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/understanding-fields/about-date-fields.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/understanding-fields/about-iteration-fields.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/understanding-fields/about-parent-issue-and-sub-issue-progress-fields.md
- **Vistas:**
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/customizing-views-in-your-project/changing-the-layout-of-a-view.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/customizing-views-in-your-project/customizing-the-board-layout.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/customizing-views-in-your-project/customizing-the-roadmap-layout.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/customizing-views-in-your-project/managing-your-views.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/customizing-views-in-your-project/filtering-projects.md
- **Automatizaciones:**
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/automating-your-project/using-the-built-in-automations.md
  - https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/automating-your-project/adding-items-automatically.md
- **Agregar ítems:** https://github.com/github/docs/blob/main/content/issues/planning-and-tracking-with-projects/managing-items-in-your-project/adding-items-to-your-project.md

### Issues, labels y milestones

- **Sub-issues:** https://github.com/github/docs/blob/main/content/issues/tracking-your-work-with-issues/using-issues/adding-sub-issues.md
- **Dependencias:** https://github.com/github/docs/blob/main/content/issues/tracking-your-work-with-issues/using-issues/creating-issue-dependencies.md
- **Labels:** https://github.com/github/docs/blob/main/content/issues/using-labels-and-milestones-to-track-work/managing-labels.md
- **Milestones:** https://github.com/github/docs/blob/main/content/issues/using-labels-and-milestones-to-track-work/creating-and-editing-milestones-for-issues-and-pull-requests.md
- **Vincular un PR a un issue:** https://github.com/github/docs/blob/main/content/issues/tracking-your-work-with-issues/using-issues/linking-a-pull-request-to-an-issue.md
- **Crear una rama desde un issue:** https://github.com/github/docs/blob/main/content/issues/tracking-your-work-with-issues/using-issues/creating-a-branch-for-an-issue.md

### Plantillas

- https://github.com/github/docs/blob/main/content/communities/using-templates-to-encourage-useful-issues-and-pull-requests/syntax-for-issue-forms.md
- https://github.com/github/docs/blob/main/content/communities/using-templates-to-encourage-useful-issues-and-pull-requests/syntax-for-githubs-form-schema.md
- https://github.com/github/docs/blob/main/content/communities/using-templates-to-encourage-useful-issues-and-pull-requests/configuring-issue-templates-for-your-repository.md
- https://github.com/github/docs/blob/main/content/communities/using-templates-to-encourage-useful-issues-and-pull-requests/creating-a-pull-request-template-for-your-repository.md
- https://github.com/github/docs/blob/main/content/communities/using-templates-to-encourage-useful-issues-and-pull-requests/common-validation-errors-when-creating-issue-forms.md

### Rulesets

- https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/managing-rulesets/creating-rulesets-for-a-repository.md
- https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/managing-rulesets/available-rules-for-rulesets.md
- https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/managing-rulesets/about-rulesets.md
- https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/managing-rulesets/troubleshooting-rules.md

### Repositorio, merges y colaboradores

- **Rama por defecto:** https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/managing-branches-in-your-repository/changing-the-default-branch.md
- **Métodos de merge:**
  - https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/configuring-pull-request-merges/configuring-commit-squashing-for-pull-requests.md
  - https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/configuring-pull-request-merges/configuring-commit-merging-for-pull-requests.md
  - https://github.com/github/docs/blob/main/content/repositories/configuring-branches-and-merges-in-your-repository/configuring-pull-request-merges/managing-the-automatic-deletion-of-branches.md
- **Merges y revisiones:**
  - https://github.com/github/docs/blob/main/content/pull-requests/reference/pull-request-merges.md
  - https://github.com/github/docs/blob/main/content/pull-requests/how-tos/merge-and-close-pull-requests/merging-a-pull-request.md
  - https://github.com/github/docs/blob/main/data/reusables/repositories/request-changes-tips.md
  - https://github.com/github/docs/blob/main/content/pull-requests/how-tos/merge-and-close-pull-requests/troubleshooting-required-status-checks.md
- **Colaboradores:**
  - https://github.com/github/docs/blob/main/content/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/inviting-collaborators-to-a-personal-repository.md
  - https://github.com/github/docs/blob/main/content/repositories/managing-your-repositorys-settings-and-features/repository-access-and-collaboration/permission-levels-for-a-personal-account-repository.md

### API REST

- **Milestones, sub-issues y dependencias:** https://github.com/github/docs/blob/main/src/rest/data/fpt-2022-11-28/issues.json
- **Rulesets:** https://github.com/github/docs/blob/main/src/rest/data/fpt-2022-11-28/repos.json

### GitHub CLI (código fuente)

- **Comandos de issues:**
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/issue/create/create.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/issue/edit/edit.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/issue/develop/develop.go
- **Comandos de Projects:**
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/project/project.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/project/create/create.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/project/link/link.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/project/field-create/field_create.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/project/field-list/field_list.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/project/item-add/item_add.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/project/item-edit/item_edit.go
- **Otros comandos:**
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/label/create.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/api/api.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/repo/edit/edit.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/pr/create/create.go
  - https://github.com/cli/cli/blob/trunk/pkg/cmd/pr/merge/merge.go
- **Instalación:** https://github.com/cli/cli/blob/trunk/docs/install_linux.md

### Estado del repo

- https://api.github.com/repos/franciscoNSoria/Competencia-Robotica-CAID
- https://api.github.com/repos/franciscoNSoria/Competencia-Robotica-CAID/branches