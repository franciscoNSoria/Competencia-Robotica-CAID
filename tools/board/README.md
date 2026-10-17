# Cargador del tablero

`cargar_tablero.py` crea en GitHub las 68 tareas del roadmap (una por issue), las pone en el tablero
con su columna y su orden, crea las etiquetas y los hitos, y marca qué tareas esperan a otras.

- **Datos:** `tareas.json` (generado a partir del roadmap).
- **Requisitos:** `gh` 2.97 o más nueva, con sesión iniciada y el permiso `project`; Python 3.
- **Seguridad:** por defecto es un *simulacro*: muestra los comandos y no cambia nada.
  Los cambios reales requieren `--aplicar`.

Uso, desde la raíz del repo:

```bash
python3 tools/board/cargar_tablero.py                       # simulacro de todo
python3 tools/board/cargar_tablero.py --solo DESC-01        # simulacro de una tarea
python3 tools/board/cargar_tablero.py --solo DESC-01 --aplicar   # crea solo esa (prueba)
python3 tools/board/cargar_tablero.py --aplicar             # crea todo lo que falte
```

Se puede repetir: lo ya creado se recuerda en `~/.caid_tablero_estado.json` y no se duplica.
Ese archivo es local de cada PC: si otra persona corre el programa, crearía duplicados.
Por eso lo corre **una sola persona**.
