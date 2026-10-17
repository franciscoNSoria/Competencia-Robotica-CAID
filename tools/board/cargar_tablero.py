#!/usr/bin/env python3
"""Carga las tareas del roadmap en GitHub: issues, etiquetas, hitos y tablero.

Por defecto NO hace nada: solo muestra, en modo simulacro, qué comandos de `gh`
ejecutaría. Para hacer los cambios de verdad hay que agregar --aplicar.

Uso (desde la carpeta donde están este archivo y tareas.json):

    python3 cargar_tablero.py                    # simulacro de todo
    python3 cargar_tablero.py --solo INF-01      # simulacro de una sola tarea
    python3 cargar_tablero.py --solo INF-01 --aplicar   # crea SOLO esa tarea
    python3 cargar_tablero.py --aplicar          # crea todo (lo que falte)

Se puede correr varias veces: lo que ya se creó se recuerda en un archivo de estado
(~/.caid_tablero_estado.json) y no se duplica.
"""
import argparse
import json
import re
import os
import subprocess
import sys
import tempfile
import time
from pathlib import Path

# --- Datos de tu repo y tu tablero (cambiar acá si algo difiere) -------------------------
DUENO = "franciscoNSoria"
REPO = "franciscoNSoria/Competencia-Robotica-CAID"
PROYECTO_NUMERO = "1"                    # el número que aparece en la URL: .../projects/1
PROYECTO_TITULO = "CAID 2026 Laberinto"  # el nombre del tablero

ARCHIVO_TAREAS = Path(__file__).with_name("tareas.json")
ARCHIVO_ESTADO = Path.home() / ".caid_tablero_estado.json"

# GitHub limita cuánto contenido se puede crear de golpe. Para no chocar con ese límite:
PAUSA_ENTRE_TAREAS = float(os.environ.get("CAID_PAUSA", "4"))   # segundos entre una tarea y la siguiente
ESPERA_SI_LIMITE = float(os.environ.get("CAID_ESPERA", "70"))   # segundos que se espera si GitHub dice "rate limit"
REINTENTOS_LIMITE = 6

# Etiquetas que se crean en el repo: nombre -> (color sin #, descripción)
ETIQUETAS = {
    "tarea": ("1D76DB", "Tarea del roadmap"),
    "necesita-robot": ("D93F0B", "Solo se prueba con el robot fisico"),
    "espera-externa": ("FBCA04", "Espera respuesta de la organizacion, compras o impresion"),
    "stretch": ("C5DEF5", "Solo si sobra tiempo"),
}


class Ejecutor:
    """Corre comandos de `gh`. En simulacro los muestra y no los ejecuta."""

    def __init__(self, aplicar):
        self.aplicar = aplicar

    def gh(self, *args, leer=False, tolerar=()):
        """Ejecuta `gh <args>`. Con leer=True se ejecuta siempre (solo consulta, no cambia nada).

        `tolerar`: textos de error que NO son un problema (por ejemplo "already exists"):
        si el error contiene alguno, se sigue adelante en vez de frenar.
        """
        comando = ["gh", *args]
        if not self.aplicar and not leer:
            print("  [simulacro] " + " ".join(_mostrar(a) for a in comando))
            return ""
        resultado = subprocess.run(comando, capture_output=True, text=True)
        intentos = 0
        while (resultado.returncode != 0 and "rate limit" in resultado.stderr.lower()
               and intentos < REINTENTOS_LIMITE):
            intentos += 1
            print(f"  GitHub pide esperar (límite de consultas). Espero {ESPERA_SI_LIMITE:.0f} s "
                  f"y reintento ({intentos}/{REINTENTOS_LIMITE})...")
            time.sleep(ESPERA_SI_LIMITE)
            resultado = subprocess.run(comando, capture_output=True, text=True)
        if resultado.returncode != 0 and any(t in resultado.stderr for t in tolerar):
            print("  (ya estaba hecho, se sigue)")
            return ""
        if resultado.returncode != 0:
            print(f"\nERROR al ejecutar: {' '.join(comando)}\n{resultado.stderr}", file=sys.stderr)
            sys.exit(1)
        return resultado.stdout.strip()


def _mostrar(texto):
    """Acorta los textos largos para que el simulacro se pueda leer."""
    texto = str(texto)
    return repr(texto[:60] + "...") if len(texto) > 60 else (repr(texto) if " " in texto else texto)


def cuerpo_issue(t):
    """Arma el texto del issue (lo que se ve al abrir la tarjeta)."""
    micro = "\n".join(f"- [ ] {m}" for m in t["microtareas"])
    depende = ", ".join(t["depende_de"]) if t["depende_de"] else "ninguna (se puede empezar ya)"
    partes = [
        f"**Columna:** {t['columna']} · **Orden en la columna:** {t['orden']} · "
        f"**Duración estimada:** {t['dias']} d · **Hito:** {t['hito']}",
        f"## Para qué\n{t['para_que']}",
        f"## Microtareas\n{micro}",
        f"## Entregable\n{t['entregable']}",
        f"## Terminada cuando\n{t['criterio']}",
        f"## Documentar en\n{t['documentar_en']}",
        f"## Depende de\n{depende}",
        f"## Rama sugerida\n`{t['rama']}`",
    ]
    if t["archivos"]:
        partes.append("## Archivos que se tocan\n" + "\n".join(f"- `{a}`" for a in t["archivos"]))
    return "\n\n".join(partes) + "\n"


def cargar_estado():
    if ARCHIVO_ESTADO.exists():
        estado = json.loads(ARCHIVO_ESTADO.read_text())
        estado.setdefault("campos_hechos", [])
        return estado
    return {"issues": {}, "campos_hechos": [], "bloqueos_hechos": []}


def guardar_estado(estado, aplicar):
    if aplicar:
        ARCHIVO_ESTADO.write_text(json.dumps(estado, indent=1))


def leer_campos(gh):
    salida = gh.gh("project", "field-list", PROYECTO_NUMERO, "--owner", DUENO, "--format", "json", leer=True)
    campos = json.loads(salida)["fields"] if salida else []
    return {c["name"]: c for c in campos}


def paso_campos(gh, datos):
    """Verifica que el tablero tenga los campos Columna (con todas sus opciones) y Orden.

    Devuelve los códigos internos (IDs) que GitHub usa para el proyecto y los campos. Se buscan
    UNA sola vez acá: usar estos códigos después es mucho más barato (en consultas) que pasarle
    la URL de cada tarea a `gh` en cada orden.
    """
    print("\n== 1. Campos del tablero ==")
    por_nombre = leer_campos(gh)
    if "Columna" not in por_nombre:
        sys.exit("ERROR: el tablero no tiene el campo 'Columna'. Crealo primero (ver la guía).")
    opciones = {o["name"]: o["id"] for o in por_nombre["Columna"].get("options", [])}
    faltan = [c for c in datos["columnas"] if c not in opciones]
    if faltan:
        sys.exit("ERROR: al campo 'Columna' le faltan estas opciones (escritas exactamente así):\n  "
                 + "\n  ".join(faltan))
    print("  Campo 'Columna': OK, con sus 12 opciones.")
    if "Orden" in por_nombre:
        print("  Campo 'Orden': OK, ya existe.")
    else:
        print("  Campo 'Orden': no existe, se crea (es un número: la posición dentro de la columna).")
        gh.gh("project", "field-create", PROYECTO_NUMERO, "--owner", DUENO, "--name", "Orden", "--data-type", "NUMBER")
        por_nombre = leer_campos(gh) if gh.aplicar else por_nombre
    proyecto_id = gh.gh("project", "view", PROYECTO_NUMERO, "--owner", DUENO, "--format", "json",
                        "--jq", ".id", leer=True) or "PVT_simulacro"
    return {
        "proyecto": proyecto_id,
        "columna": por_nombre["Columna"]["id"],
        "orden": por_nombre.get("Orden", {}).get("id", "PVTF_orden_simulacro"),
        "opciones": opciones,
    }


def paso_etiquetas(gh):
    print("\n== 2. Etiquetas del repo ==")
    for nombre, (color, desc) in ETIQUETAS.items():
        # --force: si la etiqueta ya existe, la actualiza en vez de dar error
        gh.gh("label", "create", nombre, "-R", REPO, "--color", color, "--description", desc, "--force")


def paso_hitos(gh, datos):
    print("\n== 3. Hitos (milestones) ==")
    salida = gh.gh("api", f"repos/{REPO}/milestones?state=all&per_page=100", "--jq", ".[].title", leer=True)
    existentes = set(salida.splitlines()) if salida else set()
    for h in datos["hitos"]:
        if h["titulo"] in existentes:
            print(f"  {h['titulo']}: ya existe.")
            continue
        gh.gh("api", f"repos/{REPO}/milestones", "-f", f"title={h['titulo']}",
              "-f", f"due_on={h['fecha']}T23:59:59Z", "-f", f"description={h['descripcion']}")


def issues_existentes(gh):
    """Busca en el repo las tareas ya creadas (por el prefijo [ID] del título).

    Sirve para no duplicar si un corrida anterior se cortó justo después de crear un issue.
    """
    salida = gh.gh("issue", "list", "-R", REPO, "--state", "all", "--limit", "500",
                   "--json", "number,title,url", leer=True)
    encontrados = {}
    for it in (json.loads(salida) if salida else []):
        m = re.match(r"^\[([A-Z]+-\d+)\]", it["title"])
        if m:
            encontrados[m.group(1)] = {"numero": it["number"], "url": it["url"]}
    return encontrados


def items_existentes(gh):
    """Tarjetas que ya están en el tablero: {url del issue: código de la tarjeta}."""
    salida = gh.gh("project", "item-list", PROYECTO_NUMERO, "--owner", DUENO, "--limit", "500",
                   "--format", "json", leer=True)
    items = {}
    for it in (json.loads(salida)["items"] if salida else []):
        c = it.get("content") or {}
        url = c.get("url") or (f"https://github.com/{REPO}/issues/{c['number']}" if c.get("number") else None)
        if url:
            items[url] = it["id"]
    return items


def item_de_issue(gh, numero):
    """Pregunta a GitHub cuál es el código de la tarjeta de un issue dentro de nuestro tablero."""
    consulta = ("query($o:String!,$r:String!,$n:Int!){repository(owner:$o,name:$r)"
                "{issue(number:$n){projectItems(first:20){nodes{id project{number}}}}}}")
    filtro = (".data.repository.issue.projectItems.nodes[]"
              f" | select(.project.number=={int(PROYECTO_NUMERO)}) | .id")
    return gh.gh("api", "graphql", "-f", f"query={consulta}", "-f", f"o={REPO.split('/')[0]}",
                 "-f", f"r={REPO.split('/')[1]}", "-F", f"n={numero}", "--jq", filtro, leer=True)


def paso_issues(gh, datos, estado, solo, ids):
    print("\n== 4. Tareas: un issue por tarea, dentro del tablero ==")
    siguiente_falso = 1000  # en simulacro se inventan números para poder mostrar los pasos siguientes
    hitos = {h["id"]: h["titulo"] for h in datos["hitos"]}
    for tid, datos_issue in issues_existentes(gh).items():
        if tid not in estado["issues"]:
            print(f"  [{tid}] ya existía en el repo (#{datos_issue['numero']}): se recupera, no se duplica.")
            estado["issues"][tid] = datos_issue
    guardar_estado(estado, gh.aplicar)
    items_tablero = items_existentes(gh)
    for t in datos["tareas"]:
        if solo and t["id"] != solo:
            continue
        if t["id"] not in estado["issues"]:
            print(f"  [{t['id']}] {t['titulo'][:70]}")
            with tempfile.NamedTemporaryFile("w", suffix=".md", delete=False, encoding="utf-8") as f:
                f.write(cuerpo_issue(t))
                ruta_cuerpo = f.name
            # Se crea el issue SIN --project: meterlo al tablero es un paso aparte (más abajo).
            args = ["issue", "create", "-R", REPO, "--title", f"[{t['id']}] {t['titulo']}",
                    "--body-file", ruta_cuerpo, "--milestone", hitos[t["hito"]]]
            for et in t["etiquetas"]:
                args += ["--label", et]
            url = gh.gh(*args)
            if gh.aplicar:
                numero = int(url.rstrip("/").split("/")[-1])
            else:
                numero, url = siguiente_falso, f"https://github.com/{REPO}/issues/{siguiente_falso}"
                siguiente_falso += 1
            # Se recuerda el issue ANTES de seguir: si algo falla después, no se crea un duplicado.
            estado["issues"][t["id"]] = {"numero": numero, "url": url}
            guardar_estado(estado, gh.aplicar)
        else:
            print(f"  [{t['id']}] ya creado (#{estado['issues'][t['id']]['numero']}).")
        # Tablero: meter la tarjeta (si ya estaba, se reutiliza) y asignar columna y orden.
        # Se recuerda aparte: si algo falla, se reintenta solo esta parte.
        if t["id"] not in estado["campos_hechos"]:
            url = estado["issues"][t["id"]]["url"]
            item = items_tablero.get(url) or estado.get("items", {}).get(t["id"])
            if not item:
                item = gh.gh("project", "item-add", PROYECTO_NUMERO, "--owner", DUENO, "--url", url,
                             "--format", "json", "--jq", ".id", tolerar=("already exists",))
            if not item and gh.aplicar:
                # La tarjeta ya estaba en el tablero (por ejemplo, la agregó el tablero solo), así que
                # `item-add` no devolvió su código. Se le pregunta a GitHub directamente por esa tarea.
                numero = estado["issues"][t["id"]]["numero"]
                for _ in range(4):
                    item = item_de_issue(gh, numero)
                    if item:
                        break
                    time.sleep(3)
                if not item:  # último recurso: buscarla en la lista completa del tablero
                    items_tablero.update(items_existentes(gh))
                    item = items_tablero.get(url)
                if not item:
                    sys.exit(f"ERROR: la tarjeta de {t['id']} (#{numero}) no aparece en el tablero. "
                             "Revisá el tablero y volvé a correr el programa.")
            item = item or "PVTI_simulacro"   # solo en simulacro (no se usa de verdad)
            estado.setdefault("items", {})[t["id"]] = item
            base = ["project", "item-edit", "--id", item, "--project-id", ids["proyecto"]]
            gh.gh(*base, "--field-id", ids["columna"], "--single-select-option-id", ids["opciones"][t["columna"]])
            gh.gh(*base, "--field-id", ids["orden"], "--number", str(t["orden"]))
            estado["campos_hechos"].append(t["id"])
            guardar_estado(estado, gh.aplicar)
            if gh.aplicar:
                time.sleep(PAUSA_ENTRE_TAREAS)


def paso_bloqueos(gh, datos, estado, solo):
    """Marca 'bloqueada por' solo entre tareas de columnas distintas (son las uniones)."""
    print("\n== 5. Uniones: tareas que esperan a otras columnas ('blocked by') ==")
    for t in datos["tareas"]:
        if solo and t["id"] != solo:
            continue
        if not t["bloqueada_por"] or t["id"] in estado["bloqueos_hechos"]:
            continue
        numeros = []
        for dep in t["bloqueada_por"]:
            if dep not in estado["issues"]:
                print(f"  [{t['id']}] se omite: {dep} todavía no está creada (correr de nuevo cuando estén todas).")
                break
            numeros.append(str(estado["issues"][dep]["numero"]))
        else:
            propio = estado["issues"].get(t["id"])
            if not propio:
                continue
            print(f"  [{t['id']}] queda bloqueada por {', '.join(t['bloqueada_por'])}")
            gh.gh("issue", "edit", str(propio["numero"]), "-R", REPO, "--add-blocked-by", ",".join(numeros))
            estado["bloqueos_hechos"].append(t["id"])
            guardar_estado(estado, gh.aplicar)
            if gh.aplicar:
                time.sleep(PAUSA_ENTRE_TAREAS)


def main():
    parser = argparse.ArgumentParser(description="Carga el roadmap en GitHub (por defecto, solo simulacro).")
    parser.add_argument("--aplicar", action="store_true", help="hacer los cambios de verdad")
    parser.add_argument("--solo", metavar="ID", help="procesar una sola tarea, por ejemplo INF-01")
    args = parser.parse_args()

    datos = json.loads(ARCHIVO_TAREAS.read_text(encoding="utf-8"))
    if args.solo and args.solo not in {t["id"] for t in datos["tareas"]}:
        sys.exit(f"ERROR: la tarea {args.solo} no existe en tareas.json")
    gh = Ejecutor(args.aplicar)
    estado = cargar_estado()

    print("MODO: " + ("APLICAR (cambia GitHub de verdad)" if args.aplicar else "SIMULACRO (no cambia nada)"))
    ids = paso_campos(gh, datos)
    paso_etiquetas(gh)
    paso_hitos(gh, datos)
    paso_issues(gh, datos, estado, args.solo, ids)
    paso_bloqueos(gh, datos, estado, args.solo)
    print("\nListo." + ("" if args.aplicar else " (Fue un simulacro: agregá --aplicar para hacerlo de verdad.)"))


if __name__ == "__main__":
    main()
