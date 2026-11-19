"""
Leer y guardar los YAML de calibración de caid_vision (colores.yaml, lente.yaml).

Se guardan en dos lugares:
  - install/caid_vision/share/caid_vision/config/  lo que leen los nodos (sirve sin recompilar)
  - src/caid_vision/config/                        para subirlo a git y que no se pise al compilar
"""
import os

from ament_index_python.packages import get_package_share_directory
import yaml


def carpeta_share():
    return os.path.join(get_package_share_directory('caid_vision'), 'config')


def carpeta_src():
    """src/caid_vision/config del workspace (layout normal de colcon), o None si no está."""
    share = get_package_share_directory('caid_vision')        # ws/install/caid_vision/share/...
    ws = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    src = os.path.join(ws, 'src', 'caid_vision', 'config')
    return src if os.path.isdir(src) else None


def ruta(nombre):
    """Ruta del YAML que leen los nodos."""
    return os.path.join(carpeta_share(), nombre)


def cargar_yaml(ruta_archivo):
    if not ruta_archivo or not os.path.exists(ruta_archivo):
        return None
    with open(ruta_archivo) as fh:
        return yaml.safe_load(fh)


def guardar_yaml(nombre, datos, cabecera=''):
    """Guarda en share y en src. Devuelve la lista de rutas escritas."""
    texto = cabecera + yaml.safe_dump(datos, sort_keys=False, allow_unicode=True,
                                      default_flow_style=None)
    escritas = []
    for carpeta in (carpeta_share(), carpeta_src()):
        if carpeta is None:
            continue
        os.makedirs(carpeta, exist_ok=True)
        destino = os.path.join(carpeta, nombre)
        with open(destino, 'w') as fh:
            fh.write(texto)
        escritas.append(destino)
    return escritas


def rangos_desde_yaml(d):
    """{'rojo': [[[h,s,v],[h,s,v]], ...]} -> formato de detector.RANGOS."""
    return {color: [(tuple(int(x) for x in a), tuple(int(x) for x in b)) for a, b in tramos]
            for color, tramos in d.items()}


def rangos_a_yaml(rangos):
    return {color: [[list(map(int, a)), list(map(int, b))] for a, b in tramos]
            for color, tramos in rangos.items()}
