from caid_navigation.mapa import girar, Mapa, vecina

# maze_2.world visto desde arriba: # = bloque de pared de 0,45 m, I = inicio (mira hacia +y).
MAZE_2 = """
#####################
#.....#.............#
#####.#.#######.###.#
#.....#.......#.#...#
#.#####.#I#.###.#.###
#.#.....###.....#...#
#.#.###.....###.###.#
#.#...#.......#.....#
#.###.#.#####.#####.#
#...#.#.....#.....#.#
###.#.#####.#####.#.#
#...#.....#.....#.#.#
#.#######.#####.#.#.#
#.......#...#...#...#
#######.###.#.#######
#.....#.....#.......#
#.###.###.#######.###
#...#.......#.....#.#
###.###.###.#.#####.#
#.........#.#........
#####################
"""

# Las paredes alrededor de X no tocan el borde: son una isla, con un pasillo que da toda la vuelta.
ISLA = """
#######
#.....#
#.###.#
#.#X#.#
#.#.#.#
#..I..#
#######
"""


def leer(texto):
    filas = texto.strip().splitlines()
    libres, inicio = set(), None
    for j, fila in enumerate(filas):
        for i, ch in enumerate(fila):
            celda = (i, len(filas) - 1 - j)
            if ch != '#':
                libres.add(celda)
            if ch == 'I':
                inicio = celda
    return libres, inicio


def explorar(texto, rumbo, max_pasos=5000):
    """Simula al robot ideal: mide en cada celda, gira o avanza según decide el mapa."""
    libres, celda = leer(texto)
    mapa = Mapa(inicio=celda)
    for _ in range(max_pasos):
        mapa.registrar(celda, rumbo, *(vecina(celda, girar(rumbo, rel)) in libres
                                       for rel in (0, 1, 3)))
        d = mapa.decidir(celda, rumbo)
        if d is None:
            return mapa, celda, libres
        if d == rumbo:
            celda = vecina(celda, d)
        else:
            rumbo = d
    raise AssertionError('no terminó')


def alcanzables(libres, inicio):
    vistas, pila = {inicio}, [inicio]
    while pila:
        c = pila.pop()
        for d in range(4):
            n = vecina(c, d)
            if n in libres and n not in vistas:
                vistas.add(n)
                pila.append(n)
    return vistas


def test_maze_2_completo_y_vuelve():
    mapa, final, libres = explorar(MAZE_2, rumbo=1)
    assert final == mapa.inicio
    # La salida del borde (fila de abajo, a la derecha) también es una celda libre.
    assert mapa.visitadas == alcanzables(libres, mapa.inicio)


def test_isla():
    mapa, final, libres = explorar(ISLA, rumbo=1)
    _, inicio = leer(ISLA)
    assert final == inicio
    assert mapa.visitadas == alcanzables(libres, inicio)


def test_camino_mas_corto():
    mapa, _, _ = explorar(ISLA, rumbo=1)
    # Desde I hasta la celda de arriba a la izquierda: 2 a la izquierda y 4 hacia arriba.
    ruta = mapa.camino((3, 1), lambda c: c == (1, 5))
    assert len(ruta) == 6


def test_ir_a_destino():
    mapa, _, _ = explorar(ISLA, rumbo=1)
    # Desde la esquina de arriba a la izquierda al inicio: primero hacia abajo (-y).
    assert mapa.decidir((1, 5), 0, destino=(3, 1)) == 3
    assert mapa.decidir((3, 1), 0, destino=(3, 1)) is None


def test_lado_desconocido_se_mira_girando():
    mapa = Mapa()
    mapa.registrar((0, 0), 0, frente=False, izquierda=False, derecha=False)
    # Lo único sin medir es atrás: decide girar hacia ahí, no avanzar.
    assert mapa.decidir((0, 0), 0) == 2


def test_marcadores_se_arman():
    from builtin_interfaces.msg import Time
    from caid_navigation.marcadores import marcadores
    mapa, _, _ = explorar(ISLA, rumbo=1)
    msg = marcadores(mapa, 0.45, [(0.0, 0.0), (0.3, 0.0)], (3, 3), Time())
    assert len(msg.markers) == 6
    assert msg.markers[1].points or msg.markers[2].points


def test_limites_cierran_el_area_abierta():
    # Área abierta sin paredes: sin límites la exploración no terminaría nunca.
    mapa = Mapa(limites=(0, 2, -1, 1))
    celda, rumbo = (0, 0), 0
    for _ in range(500):
        mapa.registrar(celda, rumbo, True, True, True)
        d = mapa.decidir(celda, rumbo)
        if d is None:
            break
        if d == rumbo:
            celda = vecina(celda, d)
        else:
            rumbo = d
    assert d is None
    assert mapa.visitadas == {(x, y) for x in range(3) for y in range(-1, 2)}


def test_paso_recorrido_no_se_cierra():
    mapa = Mapa()
    mapa.recorrer((0, 0), 0)
    # Una mala lectura después de girar no puede cerrar el pasillo por el que vino.
    mapa.registrar((1, 0), 1, frente=False, izquierda=False, derecha=False)
    assert mapa.abierto((1, 0), 2) is True


def test_flecha_va_antes_que_la_exploracion():
    # T: llega mirando a +x, pared adelante, pasos a izquierda (+y) y derecha (-y).
    mapa = Mapa()
    mapa.recorrer((-1, 0), 0)
    mapa.registrar((0, 0), 0, frente=False, izquierda=True, derecha=True)
    assert mapa.decidir((0, 0), 0) == 3                  # sin flecha: derecha primero
    assert mapa.decidir((0, 0), 0, preferida=1) == 1     # flecha a la izquierda: manda
    # Si por ese lado ya pasó, la flecha no lo hace volver: sigue la exploración.
    mapa.recorrer((0, 0), 1)
    mapa.visitadas.add((0, 1))
    assert mapa.decidir((0, 0), 0, preferida=1) == 3


def test_salida_cerrada_y_pilar_olvidado():
    mapa = Mapa()
    mapa.registrar((0, 0), 0, frente=True, izquierda=False, derecha=False)
    mapa.marcar_lado((0, 0), 2, True)          # atrás hay piso (la zona de partida)
    mapa.cerrar((0, 0), 2)
    assert 2 not in mapa.pendientes((0, 0))     # la salida no se explora
    assert mapa.camino((1, 0), lambda c: c == (-1, 0)) is None
    mapa.abrir((0, 0), 2)
    assert mapa.camino((0, 0), lambda c: c == (-1, 0)) == [2]
    # Un pilar medido como pared adelante: al sacarlo se olvida y se vuelve a medir.
    mapa.registrar((1, 0), 0, frente=False, izquierda=False, derecha=False)
    mapa.recorrer((0, 0), 0)
    mapa.olvidar((1, 0))
    assert mapa.abierto((1, 0), 0) is None
    assert mapa.abierto((1, 0), 2) is True      # por donde vino no se olvida


def test_lado_que_cambia_no_hace_loop():
    # Un obstáculo chico: dos veces se ve como pared, una vez no. Gana la pared.
    mapa = Mapa()
    for frente in (False, False, True):
        mapa.registrar((0, 0), 0, frente, False, False)
    assert mapa.abierto((0, 0), 0) is False
    # Si el robot frenó antes de pasar, queda pared aunque después se mida libre.
    mapa.registrar((0, 0), 1, frente=True, izquierda=False, derecha=False)
    mapa.bloquear((0, 0), 1)
    for _ in range(3):
        mapa.registrar((0, 0), 1, frente=True, izquierda=False, derecha=False)
    assert mapa.abierto((0, 0), 1) is False
    mapa.olvidar((0, 0))                      # se sacó el obstáculo: se vuelve a medir
    mapa.registrar((0, 0), 1, frente=True, izquierda=False, derecha=False)
    assert mapa.abierto((0, 0), 1) is True


def test_comparar_y_reiniciar():
    mapa = Mapa()
    mapa.registrar((0, 0), 0, frente=True, izquierda=False, derecha=False)
    assert mapa.comparar((0, 0), 0, True, False, False) == (3, 0)
    assert mapa.comparar((0, 0), 0, False, True, False) == (1, 2)     # no coincide: perdido
    assert mapa.comparar((0, 0), 2, True, False, False) == (2, 0)     # atrás no se midió
    mapa.cerrar((0, 0), 2)
    mapa.reiniciar()
    assert not mapa.visitadas and mapa.abierto((0, 0), 0) is None
    assert 2 not in mapa.pendientes((0, 0))                           # la salida sigue cerrada


# Pasillo recto hacia +x con ramas laterales y una curva al final, para probar el largo de flecha.
#   y=2        ....
#   y=1    .   .   .      (ramas laterales en x=2 y x=4)
#   y=0    I...........
CON_RAMAS = """
#########
#...#.#.#
#.#.#.#.#
#I......#
#########
"""


def _mapa_con_ramas():
    """Mapa de CON_RAMAS con el pasillo de abajo recorrido: las ramas quedan sin visitar."""
    libres, inicio = leer(CON_RAMAS)
    mapa = Mapa(inicio=inicio)
    for celda in sorted(c for c in libres if c[1] == inicio[1]):
        mapa.registrar(celda, 0, *(vecina(celda, girar(0, rel)) in libres for rel in (0, 1, 3)))
    return mapa, inicio


def test_la_guia_de_la_flecha_no_se_desvia_por_las_ramas():
    from caid_navigation.mapa import seguir_guia
    mapa, inicio = _mapa_con_ramas()
    # Flecha que dice "3 o más tramos rectos" hacia +x: cruza las dos ramas laterales sin doblar.
    guia = {'color': 'verde', 'dir': 0, 'tramos': 3}
    celda = inicio
    for _ in range(4):
        d, guia = seguir_guia(mapa, celda, guia)
        assert d == 0, celda
        assert guia['tramos'] == 3          # "tres o más" no se agota
        celda = vecina(celda, d)
    # La diferencia: en (3,1), con la celda de adelante ya visitada y la rama todavía no,
    # la exploración dobla hacia la rama y la guía sigue derecho.
    assert mapa.pendientes((3, 1)) == [1]
    assert mapa.decidir((3, 1), 0, preferida=None) == 1
    d, _ = seguir_guia(mapa, (3, 1), {'color': 'verde', 'dir': 0, 'tramos': 3})
    assert d == 0


def test_la_guia_dobla_sola_cuando_el_pasillo_dobla_y_se_agota():
    from caid_navigation.mapa import seguir_guia
    mapa, _ = _mapa_con_ramas()
    # En el fondo del pasillo (x=7) el único camino es hacia +y: la guía de 2 tramos dobla.
    guia = {'color': 'verde', 'dir': 0, 'tramos': 2}
    d, guia = seguir_guia(mapa, (7, 1), guia)
    assert (d, guia['tramos'], guia['dir']) == (1, 1, 1)
    # Con un solo tramo, al cerrarse el pasillo la guía se agota y decide la exploración.
    d, guia = seguir_guia(mapa, (7, 3), {'color': 'verde', 'dir': 1, 'tramos': 1})
    assert (d, guia) == (None, None)


def test_la_guia_se_descarta_en_una_bifurcacion_que_no_puede_resolver():
    from caid_navigation.mapa import seguir_guia
    mapa, _ = _mapa_con_ramas()
    # Llegando a (2,1) desde +y, el pasillo se abre a dos lados: la flecha no alcanza.
    d, guia = seguir_guia(mapa, (2, 1), {'color': 'verde', 'dir': 3, 'tramos': 2})
    assert (d, guia) == (None, None)


def test_camino_optimista_cruza_lo_no_medido_y_esquiva():
    # Zona de partida abierta: casi nada medido. Con optimista se planifica igual.
    mapa = Mapa()
    mapa.registrar((0, 0), 2, frente=True, izquierda=True, derecha=True)
    meta = (-3, 0)
    assert mapa.camino((0, 0), lambda c: c == meta) is None
    assert mapa.camino((0, 0), lambda c: c == meta, optimista=True) == [2, 2, 2]
    # Esquivando un canasto en el medio, da la vuelta.
    ruta = mapa.camino((0, 0), lambda c: c == meta, optimista=True, evitar={(-2, 0)})
    assert ruta is not None and len(ruta) == 5
    # Una pared medida no se cruza aunque sea optimista.
    mapa.marcar_lado((0, 0), 2, False)
    assert mapa.camino((0, 0), lambda c: c == (-1, 0), optimista=True) != [2]


def test_camino_optimista_tiene_tope():
    # Sin nada medido y un objetivo imposible, el plano no tiene fin: tiene que cortar.
    mapa = Mapa()
    assert mapa.camino((0, 0), lambda c: False, optimista=True, max_celdas=500) is None
