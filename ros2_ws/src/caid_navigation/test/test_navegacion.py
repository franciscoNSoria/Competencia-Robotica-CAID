"""Pruebas del núcleo de navegación (sin ROS). Correr: python3 -m pytest test/ -q"""
import math

from caid_navigation.camara import Camara, forma_de_cubo, ubicar_flecha
from caid_navigation.conocimiento import Conocimiento
from caid_navigation.control import Recta
from caid_navigation.geometria import ang_dif, CELDA
from caid_navigation.giros import buscar_giro, cajas_vecinas
from caid_navigation.localizacion import Estimador
from caid_navigation.mision import Navegador
from caid_navigation.planificador import Costos, planificar
from caid_navigation.simulacion import oraculo
from caid_navigation.simulacion.correr import a_mapa, correr, errores_de_mapa
from caid_navigation.simulacion.mundo import Mundo
from caid_navigation.simulacion.robot import RobotSim
from caid_navigation.sonar import Barrido, leer_frente, RegistroLateral

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


def test_rumbo_se_corrige_mientras_el_control_lo_mueve():
    # Pasillo de E1 (fila de la partida, celdas 1 a 3 con pared a los dos lados). El estimador
    # arranca con 5° de error y el control lateral (que usa ese rumbo) gira el robot mientras
    # avanza: la regresión resta lo que giró según la odometría y deja solo el error.
    m = Mundo('e1_serpentina')
    r = RobotSim(m, ruido=False)
    con = Conocimiento()
    for i in range(1, 4):
        con.marcar((i, 0), 1, False)
        con.marcar((i, 0), 3, False)
        con.marcar((i, 0), 0, i < 3)
        con.visitar((i, 0))
    con.marcar((0, 0), 0, True)
    est = Estimador()
    est.pose = [0.0, 0.0, math.radians(5)]
    ctrl = Recta(0, 1, 0.96, 0.0)
    for _ in range(400):
        est.odometria(*r.odom)
        rangos = {k: r.rango(k) for k in ('frente', 'izquierda', 'derecha')}
        v, w, listo = ctrl.comando(est.pose)
        est.corregir(con, rangos, v)
        r.paso(0.05, v, w)
        if listo:
            break
    verdad = a_mapa(m, r.pose)
    assert est.correcciones['rumbo'] >= 1
    assert abs(ang_dif(est.pose[2], verdad[2])) < math.radians(2)


def test_registro_lateral_no_toma_lo_de_la_celda_vecina_como_pared():
    # Robot corrido 7 cm a la izquierda: el sensor izquierdo queda a 8,5 mm del borde de la celda.
    # Un eco a 18 cm (menos que el umbral de 20 cm) está adentro de la celda vecina: el borde está
    # abierto. Un eco a 1,2 cm es la pared.
    pose = (-0.096, 0.07, 0.0)
    for lectura, abierto in ((0.18, True), (0.012, False)):
        con = Conocimiento()
        reg = RegistroLateral()
        for _ in range(6):
            reg.actualizar(con, pose, {'izquierda': lectura, 'derecha': 0.3})
        assert con.abierto((0, 0), 1) is abierto
        votos = dict(con.votos)
        for _ in range(12):     # quieto: no vuelve a votar
            reg.actualizar(con, pose, {'izquierda': lectura, 'derecha': 0.3})
        assert con.votos == votos


def test_forma_de_cubo():
    cam = Camara()

    def caja(z0, z1, dist):
        arriba = cam.cy + cam.fy * (cam.altura - z1) / dist
        abajo = cam.cy + cam.fy * (cam.altura - z0) / dist
        return (300, arriba, 20, abajo - arriba)
    for dist in (0.4, 0.8, 1.4):
        # cubo de 20 mm: se ve la arista de arriba de atrás, 2 cm más lejos
        arriba = caja(0.1285, 0.1285, dist + 0.02)[1]
        _, _, w, h = caja(0.1085, 0.1285, dist)
        abajo = caja(0.1085, 0.1285, dist)[1] + h
        assert forma_de_cubo(cam, (300, arriba, w, abajo - arriba))
        assert not forma_de_cubo(cam, caja(0.0885, 0.1085, dist))        # canasto
    for dist in (0.4, 0.8):
        # pedazo de flecha (razón 0,21, como en el render); de más lejos ya no se distingue
        assert not forma_de_cubo(cam, caja(0.090, 0.128, dist))


def test_detecta_trabado():
    nav = Navegador()
    nav.t, nav.v_cmd = 0.0, 0.1
    odom = (1.0, 0.0, 0.0)
    trabado = []
    for k in range(25):
        nav.t = k * 0.05
        trabado.append(nav._vigilar_trabado(odom))
    assert not any(trabado[:16]) and trabado[-1]     # recién después de 0,8 s
