"""Pruebas de autonomo_logica.py y del nodo (robot_autonomo.py) sin ROS ni robot.   python3 -m pytest tools/autonomo -q"""
import os
import random
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import autonomo_logica as al  # noqa: E402
import robot_autonomo as ra  # noqa: E402

P = al.Parametros()


def vagabundo(semilla=1):
    v = al.Vagabundo(al.Parametros(), random.Random(semilla))
    v.iniciar(0.0)
    return v


class Logica(unittest.TestCase):
    def test_quieto_hasta_que_el_operador_arma(self):
        v = al.Vagabundo(P, random.Random(1))
        self.assertEqual(v.paso(0.0, 2.0, 1.0, 1.0, al.G_LIBRE, 1), (0.0, 0.0))
        v.iniciar(1.0)
        self.assertEqual(v.paso(1.1, 2.0, 1.0, 1.0, al.G_LIBRE, 2)[0], P.v_crucero)

    def test_avanza_derecho_con_espacio(self):
        v = vagabundo()
        for i in range(30):
            self.assertEqual(v.paso(0.1 * i, 1.5, 1.0, 1.0, al.G_LIBRE, i), (P.v_crucero, 0.0))
        self.assertEqual(v.estado, al.AVANZAR)

    def test_sin_eco_adelante_sigue_avanzando(self):
        v = vagabundo()
        self.assertEqual(v.paso(0.1, None, None, None, al.G_LIBRE, 1), (P.v_crucero, 0.0))

    def test_obstaculo_adelante_gira_hacia_el_lado_con_mas_espacio(self):
        v = vagabundo()
        v.paso(0.0, 1.0, 0.8, 0.4, al.G_LIBRE, 1)
        lin, ang = v.paso(0.1, P.d_evitar - 0.01, 0.8, 0.4, al.G_LIBRE, 2)
        self.assertEqual((v.estado, lin), (al.GIRAR, 0.0))
        self.assertEqual(ang, P.w_giro)                         # izquierda: la izquierda tiene 0,8 y la derecha 0,4
        v = vagabundo()
        v.paso(0.0, 1.0, 0.4, 0.8, al.G_LIBRE, 1)
        self.assertEqual(v.paso(0.1, 0.2, 0.4, 0.8, al.G_LIBRE, 2), (0.0, -P.w_giro))

    def test_un_lado_sin_eco_cuenta_como_libre(self):
        v = vagabundo()
        v.paso(0.0, 1.0, None, 0.5, al.G_LIBRE, 1)
        self.assertEqual(v.paso(0.1, 0.2, None, 0.5, al.G_LIBRE, 2)[1], P.w_giro)

    def test_no_gira_en_el_lugar_con_un_lado_pegado(self):
        for izq, der in ((0.08, 0.9), (0.9, 0.08), (0.05, 0.05)):
            v = vagabundo()
            v.paso(0.0, 1.0, izq, der, al.G_LIBRE, 1)
            self.assertEqual(v.paso(0.1, 0.2, izq, der, al.G_LIBRE, 2), (0.0, 0.0))
            self.assertEqual(v.estado, al.DETENIDO)
            self.assertIn('encajonado', v.motivo)

    def test_la_guardia_frenada_tambien_hace_girar(self):
        v = vagabundo()
        v.paso(0.0, None, 0.8, 0.5, al.G_LIBRE, 1)
        self.assertEqual(v.paso(0.1, None, 0.8, 0.5, al.G_FRENADA, 2), (0.0, P.w_giro))

    def test_el_giro_termina_con_dos_mediciones_libres_seguidas_y_el_tiempo_minimo(self):
        p = al.Parametros(giro_extra_s=(0.0, 0.0))
        v = al.Vagabundo(p, random.Random(1))
        v.iniciar(0.0)
        v.paso(0.0, 1.0, 0.8, 0.5, al.G_LIBRE, 1)
        v.paso(0.1, 0.3, 0.8, 0.5, al.G_LIBRE, 2)               # empieza el giro
        n = 2
        # libre desde el principio, pero no se sale antes de giro_min_s
        for i in range(2, 5):
            n += 1
            self.assertEqual(v.paso(0.1 * i, 1.0, 0.8, 0.5, al.G_LIBRE, n)[0], 0.0)
        self.assertEqual(v.estado, al.GIRAR)
        n += 1
        lin, _ = v.paso(0.1 + p.giro_min_s + 0.1, 1.0, 0.8, 0.5, al.G_LIBRE, n)
        self.assertEqual((lin, v.estado), (p.v_crucero, al.AVANZAR))

    def test_una_medicion_repetida_no_cuenta_dos_veces(self):
        p = al.Parametros(giro_extra_s=(0.0, 0.0), giro_min_s=0.0)
        v = al.Vagabundo(p, random.Random(1))
        v.iniciar(0.0)
        v.paso(0.0, 1.0, 0.8, 0.5, al.G_LIBRE, 1)
        v.paso(0.1, 0.3, 0.8, 0.5, al.G_LIBRE, 2)
        v.paso(0.2, 1.0, 0.8, 0.5, al.G_LIBRE, 3)               # primera libre
        v.paso(0.3, 1.0, 0.8, 0.5, al.G_LIBRE, 3)               # la misma medicion: no cuenta
        self.assertEqual(v.estado, al.GIRAR)
        v.paso(0.4, 1.0, 0.8, 0.5, al.G_LIBRE, 4)               # segunda, distinta
        self.assertEqual(v.estado, al.AVANZAR)

    def test_sin_eco_en_el_giro_no_cuenta_como_libre(self):
        v = vagabundo()
        v.paso(0.0, 1.0, 0.8, 0.5, al.G_LIBRE, 1)
        v.paso(0.1, 0.3, 0.8, 0.5, al.G_LIBRE, 2)
        for i in range(3, 20):
            v.paso(0.1 * i, None, 0.8, 0.5, al.G_LIBRE, i)
        self.assertEqual(v.estado, al.GIRAR)

    def test_se_rinde_si_no_encuentra_salida(self):
        v = vagabundo()
        v.paso(0.0, 1.0, 0.8, 0.5, al.G_LIBRE, 1)
        t, n = 0.1, 2
        v.paso(t, 0.3, 0.8, 0.5, al.G_LIBRE, n)
        while v.estado == al.GIRAR and t < 20:
            t += 0.1
            n += 1
            v.paso(t, 0.3, 0.8, 0.5, al.G_LIBRE, n)
        self.assertEqual(v.estado, al.DETENIDO)
        self.assertGreaterEqual(t, P.giro_max_s)
        self.assertIn('sin encontrar salida', v.motivo)

    def test_pasada_media_vuelta_se_conforma_con_menos_espacio(self):
        v = al.Vagabundo(al.Parametros(giro_extra_s=(0.0, 0.0)), random.Random(1))
        v.iniciar(0.0)
        v.paso(0.0, 1.0, 0.8, 0.5, al.G_LIBRE, 1)
        v.paso(0.1, 0.3, 0.8, 0.5, al.G_LIBRE, 2)
        t, n = 0.1, 2
        while t < P.giro_max_s / 2 + 0.5:                       # 0,60 m: mas que d_evitar + 0,10 pero menos que d_libre
            t += 0.1
            n += 1
            v.paso(t, 0.60, 0.8, 0.5, al.G_LIBRE, n)
            if v.estado != al.GIRAR:
                break
        self.assertEqual(v.estado, al.AVANZAR)
        self.assertGreater(t, P.giro_max_s / 2)

    def test_empuja_lejos_de_una_pared_cercana(self):
        v = vagabundo()
        self.assertEqual(v.paso(0.1, 1.5, 0.15, 0.6, al.G_LIBRE, 1), (P.v_crucero, -P.w_lateral))
        v = vagabundo()
        self.assertEqual(v.paso(0.1, 1.5, 0.6, 0.15, al.G_LIBRE, 1), (P.v_crucero, P.w_lateral))
        v = vagabundo()
        self.assertEqual(v.paso(0.1, 1.5, 0.15, 0.18, al.G_LIBRE, 1)[1], -P.w_lateral)  # dos lados cerca: el mas cercano manda
        v = vagabundo()
        self.assertEqual(v.paso(0.1, 1.5, 0.15, 0.15, al.G_LIBRE, 1)[1], 0.0)           # justo en el medio: derecho

    def test_sin_telemetria_se_frena_y_si_vuelve_pronto_sigue(self):
        v = vagabundo()
        v.paso(0.0, 1.5, 1.0, 1.0, al.G_LIBRE, 1)
        self.assertEqual(v.paso(1.0, 1.5, 1.0, 1.0, al.G_LIBRE, 1, datos_frescos=False), (0.0, 0.0))
        self.assertEqual(v.paso(2.0, 1.5, 1.0, 1.0, al.G_LIBRE, 2, datos_frescos=False), (0.0, 0.0))
        self.assertEqual(v.estado, al.AVANZAR)
        self.assertEqual(v.paso(2.5, 1.5, 1.0, 1.0, al.G_LIBRE, 3)[0], P.v_crucero)

    def test_sin_telemetria_mucho_tiempo_hay_que_armar_de_nuevo(self):
        v = vagabundo()
        v.paso(0.0, 1.5, 1.0, 1.0, al.G_LIBRE, 1)
        v.paso(1.0, 1.5, 1.0, 1.0, al.G_LIBRE, 1, datos_frescos=False)
        v.paso(1.0 + P.sin_datos_rearmar_s + 0.5, 1.5, 1.0, 1.0, al.G_LIBRE, 1, datos_frescos=False)
        self.assertEqual(v.estado, al.DETENIDO)
        self.assertEqual(v.paso(10.0, 1.5, 1.0, 1.0, al.G_LIBRE, 5), (0.0, 0.0))
        v.iniciar(11.0)
        self.assertEqual(v.paso(11.1, 1.5, 1.0, 1.0, al.G_LIBRE, 6)[0], P.v_crucero)

    def test_sonar_frontal_congelado_detiene(self):
        v = vagabundo()
        t = 0.0
        while t < 3.0 and v.estado != al.DETENIDO:
            v.paso(t, 1.5, 1.0, 1.0, al.G_LIBRE, 7)             # el contador no avanza
            t += 0.1
        self.assertEqual(v.estado, al.DETENIDO)
        self.assertIn('sonar frontal', v.motivo)
        self.assertLessEqual(t, P.sonar_mudo_s + 0.5)

    def test_detener_y_reintentar(self):
        v = vagabundo()
        v.detener(1.0)
        self.assertEqual(v.paso(1.1, 1.5, 1.0, 1.0, al.G_LIBRE, 1), (0.0, 0.0))
        self.assertEqual(v.detenciones, 1)

    def test_con_la_misma_semilla_hace_lo_mismo(self):
        def corrida(sem):
            v = al.Vagabundo(P, random.Random(sem))
            v.iniciar(0.0)
            salida, n = [], 0
            for i in range(300):
                n += 1
                frente = 0.3 if (i // 25) % 2 == 1 else 1.5
                salida.append(v.paso(0.1 * i, frente, 0.5, 0.5, al.G_LIBRE, n))
            return salida
        self.assertEqual(corrida(5), corrida(5))
        self.assertNotEqual(corrida(5), corrida(6))


class EnlaceFalso:
    def __init__(self):
        self.estado = None
        self.t_estado = None
        self.vel, self.cmd = [], []

    def poner(self, ahora, frente=1.5, izq=1.0, der=1.0, guardia=al.G_LIBRE, n=1, firmware_viejo=False, diam=65.0, trocha=300.0):
        e = [0.0] * (26 if firmware_viejo else 34)
        e[20], e[23] = diam, trocha
        if not firmware_viejo:
            e[26:34] = [-1.0 if frente is None else frente, -1.0 if izq is None else izq, -1.0 if der is None else der, guardia, 0.20,
                        0.60, n, 5.0]
        self.estado, self.t_estado = e, ahora

    def enviar_vel(self, lineal, angular):
        self.vel.append((lineal, angular))

    def enviar_cmd(self, ident, *args):
        self.cmd.append((ident,) + args)


class Nodo(unittest.TestCase):
    def setUp(self):
        self.msgs = []
        self.e = EnlaceFalso()
        self.dir = tempfile.mkdtemp()
        self.calib = dict(diametro_m=0.065, trocha_m=0.300, frenado_m=0.0223, frenado_giro_m=0.0015)
        self.a = ra.Autonomo(self.e, d_stop=0.20, d_lento=0.60, semilla=1, carpeta=self.dir, imprimir=self.msgs.append, ahora=0.0,
                             calib=self.calib)

    def tearDown(self):
        self.a.cerrar()
        shutil.rmtree(self.dir, ignore_errors=True)

    def test_firmware_viejo_o_sin_conexion_no_arranca(self):
        self.a.tecla('g', 0.0)
        self.assertEqual(self.a.vag.estado, al.ESPERA)
        self.e.poner(0.0, firmware_viejo=True)
        self.a.tecla('g', 0.1)
        self.assertEqual(self.a.vag.estado, al.ESPERA)
        self.assertTrue(any('No arranco' in m for m in self.msgs))

    def test_no_arranca_con_la_guardia_apagada_ni_con_telemetria_vieja(self):
        self.e.poner(0.0, guardia=al.G_APAGADA)
        self.a.tecla('g', 0.0)
        self.assertEqual(self.a.vag.estado, al.ESPERA)
        self.e.poner(0.0)
        self.a.tecla('g', 5.0)                                  # la telemetria tiene 5 s
        self.assertEqual(self.a.vag.estado, al.ESPERA)

    def test_pide_la_guardia_cada_segundo_aunque_este_quieto(self):
        for i in range(25):
            self.e.poner(0.1 * i, n=i)
            self.a.paso(0.1 * i)
        pedidos = [c for c in self.e.cmd if c[0] == 8]
        self.assertEqual(len(pedidos), 3)
        self.assertEqual(pedidos[0], (8, 1, 0.20, 0.60, 5.0))

    def test_arranca_avanza_y_esquiva(self):
        self.e.poner(0.0)
        self.a.paso(0.0)
        self.a.tecla('g', 0.0)
        self.assertEqual(self.a.vag.estado, al.AVANZAR)
        self.e.poner(0.1, n=2)
        self.a.paso(0.1)
        self.assertEqual(self.e.vel[-1], (0.20, 0.0))
        self.e.poner(0.2, frente=0.3, n=3)
        self.a.paso(0.2)
        self.assertEqual(self.a.vag.estado, al.GIRAR)
        self.assertEqual(self.e.vel[-1][0], 0.0)
        self.assertNotEqual(self.e.vel[-1][1], 0.0)
        self.assertTrue(os.path.exists(self.a.ruta_csv))

    def test_espacio_frena(self):
        self.e.poner(0.0)
        self.a.paso(0.0)
        self.a.tecla('g', 0.0)
        self.e.poner(0.1, n=2)
        self.a.paso(0.1)
        self.a.tecla(' ', 0.2)
        self.assertEqual(self.a.vag.estado, al.DETENIDO)
        self.assertEqual(self.e.vel[-1], (0.0, 0.0))
        self.assertIn((4,), self.e.cmd)
        self.e.poner(0.3, n=3)
        self.a.paso(0.3)
        self.assertEqual(self.e.vel[-1], (0.0, 0.0))

    def test_si_se_corta_la_telemetria_manda_cero(self):
        self.e.poner(0.0)
        self.a.paso(0.0)
        self.a.tecla('g', 0.0)
        self.e.poner(0.1, n=2)
        self.a.paso(0.1)
        self.a.paso(1.0)                                        # sin telemetria nueva hace 0,9 s
        self.assertEqual(self.e.vel[-1], (0.0, 0.0))

    def test_si_la_esp32_se_reinicia_la_guardia_queda_apagada_y_se_detiene(self):
        self.e.poner(0.0)
        self.a.paso(0.0)
        self.a.tecla('g', 0.0)
        for i in range(1, 40):
            self.e.poner(0.1 * i, n=i, guardia=al.G_APAGADA if i > 5 else al.G_LIBRE)
            self.a.paso(0.1 * i)
        self.assertEqual(self.a.vag.estado, al.DETENIDO)
        self.assertIn('apagada', self.a.vag.motivo)

    def test_se_detiene_al_cumplir_el_tiempo(self):
        a = ra.Autonomo(self.e, minutos=0.05, semilla=1, imprimir=self.msgs.append, ahora=0.0)
        self.e.poner(0.0)
        a.paso(0.0)
        a.tecla('g', 0.0)
        t = 0.0
        while t < 10 and a.vag.estado != al.DETENIDO:
            t += 0.1
            self.e.poner(t, n=int(t * 10))
            a.paso(t)
        self.assertEqual(a.vag.estado, al.DETENIDO)
        self.assertIn('se cumplieron', a.vag.motivo)
        self.assertAlmostEqual(t, 3.0, delta=0.2)

    def test_solo_guardia_no_manda_velocidades(self):
        a = ra.Autonomo(self.e, solo_guardia=True, imprimir=self.msgs.append, ahora=0.0)
        for i in range(30):
            self.e.poner(0.1 * i, n=i)
            a.paso(0.1 * i)
        a.tecla('g', 3.0)
        self.assertEqual(self.e.vel, [])
        self.assertTrue(any(c[0] == 8 for c in self.e.cmd))
        self.assertEqual(a.vag.estado, al.ESPERA)

    def test_al_cerrar_apaga_la_guardia_salvo_que_se_pida_dejarla(self):
        self.a.cerrar()
        self.assertEqual(self.e.cmd[-1][:2], (8, 0))
        self.assertEqual(self.e.vel[-1], (0.0, 0.0))
        e2 = EnlaceFalso()
        b = ra.Autonomo(e2, dejar_guardia=True, ahora=0.0)
        b.cerrar()
        self.assertFalse(any(c[0] == 8 for c in e2.cmd))

    def test_manda_la_calibracion_si_la_esp32_tiene_la_de_fabrica_y_no_arranca_hasta_que_la_tenga(self):
        self.e.poner(0.0, diam=67.0, trocha=131.0)
        self.a.paso(0.0)
        self.assertIn((5, 0.065, 0.300, 0.0223, 0.0015), self.e.cmd)
        self.a.tecla('g', 0.1)
        self.assertEqual(self.a.vag.estado, al.ESPERA)
        self.assertTrue(any('calibracion' in m for m in self.msgs))
        n = len([c for c in self.e.cmd if c[0] == 5])
        self.a.paso(0.5)
        self.assertEqual(len([c for c in self.e.cmd if c[0] == 5]), n)       # una vez por segundo, no mas
        self.e.poner(1.0)                                                      # ya la tiene
        self.a.paso(2.0)
        self.assertEqual(len([c for c in self.e.cmd if c[0] == 5]), n)
        self.e.poner(2.0)
        self.a.tecla('g', 2.0)
        self.assertEqual(self.a.vag.estado, al.AVANZAR)

    def test_cargar_calibracion(self):
        c, hay = ra.cargar_calibracion(self.dir)
        self.assertFalse(hay)
        self.assertEqual(c['trocha_m'], 0.300)
        with open(os.path.join(self.dir, 'calibracion.json'), 'w') as f:
            f.write('{"diametro_m": 0.0642, "trocha_m": 0.29, "curvas": []}')
        c, hay = ra.cargar_calibracion(self.dir)
        self.assertTrue(hay)
        self.assertEqual((c['diametro_m'], c['trocha_m'], c['frenado_m']), (0.0642, 0.29, 0.0223))

    def test_leer_estado(self):
        self.assertIsNone(ra.leer_estado(None))
        self.assertIsNone(ra.leer_estado([0.0] * 26))
        self.e.poner(0.0, frente=0.5, izq=None, der=0.25, guardia=al.G_LENTA, n=9)
        t = ra.leer_estado(self.e.estado)
        self.assertEqual((t['frente'], t['izq'], t['der'], t['guardia'], t['n_frente']), (0.5, None, 0.25, al.G_LENTA, 9))

    def test_linea_de_estado(self):
        self.assertIn('sin sonares', self.a.linea(0.0))
        self.e.poner(0.0, frente=0.5, izq=None, der=0.25, guardia=al.G_LENTA)
        s = self.a.linea(0.0)
        self.assertIn('frente  0.50', s)
        self.assertIn('lenta', s)
        self.assertIn('--', s)


class Simulador(unittest.TestCase):
    @unittest.skipUnless(shutil.which('gcc'), 'hace falta gcc')
    def test_corrida_corta_del_simulador(self):
        import simular as sm
        r = sm.Simulacion(sm.cargar_guardia(), 3, 0.5).correr()
        self.assertGreater(r.recorrido, 0.5)
        self.assertGreaterEqual(r.vag.giros, 0)


if __name__ == '__main__':
    unittest.main()
