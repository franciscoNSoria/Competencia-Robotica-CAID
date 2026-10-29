"""Pruebas del núcleo de navegación (sin ROS). Correr: python3 -m pytest test/ -q"""
import math

from caid_navigation.camara import Camara, ubicar_flecha
from caid_navigation.conocimiento import Conocimiento
from caid_navigation.geometria import CELDA
from caid_navigation.giros import buscar_giro, cajas_vecinas
from caid_navigation.planificador import Costos, planificar
from caid_navigation.simulacion import oraculo
from caid_navigation.simulacion.correr import a_mapa, correr, errores_de_mapa
from caid_navigation.simulacion.mundo import Mundo
from caid_navigation.simulacion.robot import RobotSim
from caid_navigation.sonar import Barrido, leer_frente

DIAG = {(1, 1), (1, -1), (-1, 1), (-1, -1)}


def _cajas(bloques):
    return cajas_vecinas(lambda dx, dy: (dx, dy) in bloques)


def test_no_gira_180_en_un_pasillo():
    # 278 x 163 mm: las esquinas quedan a 162 mm del centro de giro y el pasillo deja 160 mm.
    assert buscar_giro(_cajas(DIAG | {(0, 1), (0, -1)}), math.pi, 0.004) is None
    assert buscar_giro(_cajas(DIAG | {(1, 0), (0, 1), (0, -1)}), math.pi, 0.004) is None


def test_gira_90_en_una_esquina_corrido():
    s, lat = buscar_giro(_cajas(DIAG | {(1, 0), (0, -1)}), math.pi / 2, 0.006)
    assert s < 0 and lat > 0          # frena antes y llega corrido hacia el lado al que gira
    assert buscar_giro(_cajas(DIAG), math.pi, 0.006) == (0.0, 0.0)   # en un cruce, centrado


def test_en_un_callejon_retrocede():
    con = Conocimiento()
    # Pasillo (0,0)-(1,0)-(2,0) cerrado al fondo y a los costados; se llega mirando +x.
    for i in range(3):
        con.marcar((i, 0), 1, False)
        con.marcar((i, 0), 3, False)
        con.visitar((i, 0))
    con.marcar((0, 0), 0, True)
    con.marcar((1, 0), 0, True)
    con.marcar((2, 0), 0, False)
    con.marcar((0, 0), 2, True)
    con.marcar((-1, 0), 1, True)
    con.marcar((-1, 0), 3, True)
    con.visitar((-1, 0))
    plan = planificar(con, ((2, 0), 0), lambda c, h: c == (-1, 0), Costos())
    assert plan is not None
    assert [a[0] for a in plan[0]] == ['retroceder'] * 3


def test_frente_marca_pared_y_sospecha():
    # Robot en el centro de la celda 0 mirando +x; pared al final de la celda 2.
    f = 2.5 * CELDA - 0.1399
    abiertos, cerrado, sospecha = leer_frente((0.0, 0.0, 0.0), f)
    assert cerrado == ((2, 0), 0) and len(abiertos) == 2 and sospecha is None
    _, cerrado, sospecha = leer_frente((0.0, 0.0, 0.0), 0.50)   # eco a mitad de la celda 2
    assert cerrado is None and sospecha[0] == (2, 0)


def test_barrido_ve_un_bloque_pegado_a_la_pared():
    m = Mundo('e4_isla')
    r = RobotSim(m, ruido=False)
    # Bloque de 8 cm en la celda (4, 6) del YAML, contra su pared E; el robot mira al norte
    # desde la celda de abajo (5, 6).
    x, y = m.lab.centro(5, 6)
    b = Barrido((0, 0), 1)
    for k in range(-24, 25, 3):
        ang = math.pi / 2 + math.radians(k)
        r.pose = [x, y, ang]
        b.agregar((0.0, 0.0, ang), r.rango('frente'))
    pasable, desvio, pts = b.resultado(margen=0.005)
    assert pts and pasable and desvio > 0.0     # pasa corrido hacia el oeste (+lateral)


def test_flecha_se_ubica_en_su_celda_y_pared():
    m = Mundo('e2_bifurcaciones')
    r = RobotSim(m, ruido=False)
    cam = Camara()
    con = Conocimiento()
    # El robot ya recorrió el pasillo de la partida: la T está 4 celdas adelante.
    for i in range(4):
        con.marcar((i, 0), 0, True)
        con.marcar((i, 0), 1, False)
        con.marcar((i, 0), 3, False)
    pose_mundo = (m.inicio[0] + 2 * CELDA, m.inicio[1], m.inicio[2])
    det = oraculo.detecciones(m, r, cam, pose_mundo, 0.0)
    pose = a_mapa(m, pose_mundo)
    vistas = {(f['color'], ubicar_flecha(con, cam, pose, 0.0, f)['dir']) for f in det['flechas']}
    # En la T (celda (4, 0)) el rojo va al norte (+y = 1) y el verde y el azul al sur (3).
    assert ('rojo', 1) in vistas and ('azul', 3) in vistas and ('verde', 3) in vistas


def test_mision_completa_e2_sin_choques():
    r = correr('e2_bifurcaciones', 900, vision='oraculo', semilla=1)
    assert r.completa, r.resumen()
    assert not r.choques
    assert errores_de_mapa(r.robot.mundo, r.nav.con) == []
