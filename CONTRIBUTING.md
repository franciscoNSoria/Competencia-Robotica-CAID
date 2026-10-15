# Cómo trabajamos

Reglas cortas para que 5 personas trabajen en paralelo sin pisarse.

## 1. Ramas

```
main      ●──────────────●──────────────●────── v1.0.0      solo cambia en cada hito (con tag)
           \            /              /
dev         ●──●──●──●──●──●──●──●──●──●──                  integración: acá se juntan las tareas
               \   /         \     /
     feature/NAV-02-...    feature/PER-01-...                una rama por tarea, vive 1 o 2 días
```

| Rama | Para qué | Quién escribe |
|---|---|---|
| `main` | Lo que llevaríamos a la competencia hoy. Siempre funciona. | Solo por PR desde `dev` en cada hito |
| `dev` | Integración de todas las tareas terminadas | Solo por PR desde ramas de tarea |
| `feature/<ID>-<descripcion>` | Una tarea del roadmap (ej.: `feature/DESC-01-urdf-chasis`) | Quien toma la tarea |
| `fix/<ID>-<descripcion>` | Corregir un error | Quien lo corrige |

Nadie hace push directo a `main` ni a `dev`: las dos ramas están protegidas.

**`dev` es la rama por defecto del repo.** Así, los PR de las tareas apuntan a `dev` sin que nadie
lo elija a mano, `Closes #n` cierra la tarea al mergear y los formularios funcionan. `main` sigue
siendo la rama de versiones.

## 2. Tomar una tarea

1. En el tablero de GitHub Projects, elegir la **primera tarea sin hacer** de una columna, con sus
   dependencias cumplidas.
2. Asignártela ("Assignees") y moverla a **En curso**.
3. Crear la rama desde `dev` actualizado:

   ```bash
   git switch dev && git pull
   git switch -c feature/NAV-02-seguimiento-de-pared
   ```

## 3. Commits

Pequeños y con mensaje claro, con el formato `tipo(alcance): descripción`:

| Tipo | Uso |
|---|---|
| `feat` | Funcionalidad nueva |
| `fix` | Corrección de un error |
| `docs` | Solo documentación |
| `test` | Pruebas |
| `refactor` | Reorganizar código sin cambiar el comportamiento |
| `chore` | Mantenimiento (configuración, dependencias) |

Ejemplos: `feat(firmware): watchdog que frena los motores sin órdenes`,
`docs(memoria): sección de percepción`.

## 4. Pull request (PR)

1. `git push -u origin feature/...` y abrir el PR **hacia `dev`**.
2. En la descripción escribir `Closes #<número del issue>`: al mergear, la tarea se cierra sola.
3. Completar el checklist de la plantilla (compila, probado, documentado).
4. **Otra persona** revisa y aprueba. El autor no puede aprobar su propio PR.
5. Se mergea con **"Squash and merge"**: toda la tarea queda como un solo commit en `dev`.
6. Borrar la rama (GitHub ofrece el botón después del merge).

## 5. Hitos: de `dev` a `main`

Cuando se cumple la demo de un hito del roadmap (`docs/roadmap/ROADMAP.md`):

1. Se prueba `dev` completo.
2. PR de `dev` a `main` con **"Create a merge commit"**, para conservar la historia.
3. Tag del hito, por ejemplo:

   ```bash
   git switch main && git pull
   git tag -a v0.2.0-h1 -m "H1: teleoperación de punta a punta"
   git push origin v0.2.0-h1
   ```

## 6. Documentar mientras se trabaja

Cada tarea dice dónde documentar ("Documentar en"). La tarea **no está terminada** si falta esa
documentación: es parte del criterio de terminado y se revisa en el PR.
