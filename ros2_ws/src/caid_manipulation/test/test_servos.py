from caid_manipulation.servos import ESPERANDO, GIRANDO, LIBRE, ServosBrazo


def _correr(s, segundos, dt=0.02):
    for _ in range(int(segundos / dt)):
        s.paso(dt)


def test_rampa_y_limites():
    s = ServosBrazo()
    s.orden([95, 300, 180, 150, 15, 60])         # hombro pedido fuera de rango: va al límite 125
    _correr(s, 0.5)
    assert 60 < s.actual[1] < 125                # va a 60 grados/s
    _correr(s, 2.0)
    assert abs(s.actual[1] - 125) < 1e-6


def test_la_base_espera_el_reposo_seguro():
    s = ServosBrazo()
    s.orden([95, 100, 120, 150, 15, 90])
    _correr(s, 2.0)
    assert abs(s.actual[1] - 100) < 1e-6
    s.orden([150, 100, 120, 150, 15, 90])        # girar la base con el brazo estirado
    s.paso(0.02)
    assert s.fase == ESPERANDO
    _correr(s, 0.5)
    assert abs(s.actual[0] - 95) < 1e-6          # la base todavía no se movió
    _correr(s, 3.0)
    assert s.fase in (GIRANDO, LIBRE)
    _correr(s, 3.0)
    assert s.fase == LIBRE and abs(s.actual[0] - 150) < 1e-6
    assert abs(s.actual[1] - 100) < 1e-6         # y el brazo volvió a lo pedido


def test_estado_tiene_23_numeros():
    assert len(ServosBrazo().estado()) == 23


def test_orden_con_nan_se_descarta():
    s = ServosBrazo()
    assert not s.orden([float('nan'), 60, 180, 150, 15, 90])
