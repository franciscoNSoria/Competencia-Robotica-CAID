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
