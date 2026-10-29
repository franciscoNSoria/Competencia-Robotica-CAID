#!/usr/bin/env python3
"""Pruebas de gen_maze.py: geometria de las flechas, formato del SDF y reglas de validacion.

Uso (desde cualquier carpeta):  python3 test_gen_maze.py
No necesita Gazebo: lee el SDF generado como XML y rehace las cuentas por su cuenta.
"""
import math
import os
import sys
import unittest
import xml.etree.ElementTree as ET

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_maze as g  # noqa: E402

MAZES = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'mazes')


def rotar(rpy, v):
    m = g.rpy_a_matriz(*rpy)
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def pose(texto):
    return [float(x) for x in texto.split()]


class TestGeometria(unittest.TestCase):
    def test_rpy_ida_y_vuelta(self):
        for pared in 'NESO':
            for d in ('NS' if pared in 'EO' else 'EO'):
                for tilt in (-30, -12, 0, 7, 30):
                    u, p, n = g.marco_flecha(pared, d, tilt)
                    y = g._cruz(p, u)
                    rpy = g.matriz_a_rpy(u, y, p)
                    m = g.rpy_a_matriz(*rpy)
                    for i in range(3):
                        self.assertAlmostEqual(m[i][0], u[i], places=9)
                        self.assertAlmostEqual(m[i][1], y[i], places=9)
                        self.assertAlmostEqual(m[i][2], p[i], places=9)

    def test_marco_en_el_plano_de_la_pared(self):
        for pared in 'NESO':
            n_pared = g._vec_mundo(pared)
            for d in ('NS' if pared in 'EO' else 'EO'):
                for tilt in (-30, 0, 30):
                    u, p, normal = g.marco_flecha(pared, d, tilt)
                    self.assertAlmostEqual(sum(a * b for a, b in zip(u, n_pared)), 0, places=9)
                    self.assertAlmostEqual(sum(a * b for a, b in zip(p, n_pared)), 0, places=9)
                    self.assertGreaterEqual(p[2], 0)
                    self.assertAlmostEqual(abs(u[2]), math.sin(math.radians(abs(tilt))), places=9)

    def test_sentido_visto_desde_el_pasillo(self):
        # pared norte: el espectador mira al norte y su derecha es el este
        u, _, _ = g.marco_flecha('N', 'E', 0)
        self.assertAlmostEqual(u[0], 1.0)
        u, _, _ = g.marco_flecha('N', 'O', 0)
        self.assertAlmostEqual(u[0], -1.0)
        # pared este: mira al este y su derecha es el sur
        u, _, _ = g.marco_flecha('E', 'S', 0)
        self.assertAlmostEqual(u[1], -1.0)
        # inclinacion positiva = antihoraria vista desde el pasillo: una flecha a la derecha sube la punta
        u, _, _ = g.marco_flecha('N', 'E', 20)
        self.assertGreater(u[2], 0)
        u, _, _ = g.marco_flecha('N', 'O', 20)
        self.assertLess(u[2], 0)

    def test_medidas_de_las_flechas_contra_el_reglamento(self):
        for largo, total in ((1, 0.080), (2, 0.110), (3, 0.150)):
            piezas = g.piezas_flecha(largo)
            u_min = min(c - a / 2 for c, a, _ in piezas)
            u_max = max(c + a / 2 for c, a, _ in piezas)
            self.assertAlmostEqual(u_max - u_min, total, places=6)
            self.assertAlmostEqual(max(h for _, _, h in piezas), 0.070, places=6)      # ancho de punta
            self.assertAlmostEqual(piezas[0][2], 0.030, places=6)                        # ancho del cuerpo
            self.assertAlmostEqual(piezas[0][1], g.FLECHA_CUERPO_LARGO_M[largo], places=6)


class TestMundos(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.labs = {}
        for nombre in sorted(os.listdir(MAZES)):
            if nombre.endswith('.yaml'):
                lab = g.cargar(os.path.join(MAZES, nombre))
                msgs, info = g.validar(lab)
                cls.labs[lab.id] = (lab, msgs, info)

    def test_hay_seis_escenarios_sin_errores(self):
        self.assertGreaterEqual(len(self.labs), 6)
        for nid, (lab, msgs, _) in self.labs.items():
            errores = [m for m in msgs if m[0] == 'ERROR']
            self.assertEqual(errores, [], nid)

    def test_sdf_bien_formado_y_nombres_unicos(self):
        for nid, (lab, msgs, info) in self.labs.items():
            raiz = ET.fromstring(g.generar_sdf(lab, info))
            mundo = raiz.find('world')
            nombres = [m.get('name') for m in mundo.findall('model')]
            self.assertEqual(len(nombres), len(set(nombres)), nid)
            for m in mundo.findall('model'):
                links = [lk.get('name') for lk in m.findall('link')]
                self.assertEqual(len(links), len(set(links)), nid + ' ' + m.get('name'))
                for lk in m.findall('link'):
                    vis = [v.get('name') for v in lk.findall('visual')]
                    self.assertEqual(len(vis), len(set(vis)))

    def test_paredes_cubren_exactamente_los_bloques(self):
        for nid, (lab, msgs, info) in self.labs.items():
            raiz = ET.fromstring(g.generar_sdf(lab, info))
            modelo = [m for m in raiz.find('world').findall('model') if m.get('name') == 'paredes'][0]
            cubiertas = set()
            for lk in modelo.findall('link'):
                x, y, z = pose(lk.find('pose').text)[:3]
                sx, sy, sz = pose(lk.find('collision/geometry/box/size').text)
                self.assertAlmostEqual(sz, lab.alto, places=4)
                self.assertAlmostEqual(z, lab.alto / 2, places=4)
                for r in range(lab.nfilas):
                    for c in range(lab.ncols):
                        cx, cy = lab.centro(r, c)
                        if abs(cx - x) < sx / 2 and abs(cy - y) < sy / 2:
                            cubiertas.add((r, c))
            esperadas = {(r, c) for r in range(lab.nfilas) for c in range(lab.ncols) if lab.car(r, c) == '#'}
            self.assertEqual(cubiertas, esperadas, nid)

    def test_flechas_en_su_pared_apuntando_donde_dice(self):
        for nid, (lab, msgs, info) in self.labs.items():
            raiz = ET.fromstring(g.generar_sdf(lab, info))
            modelo = [m for m in raiz.find('world').findall('model') if m.get('name') == 'flechas'][0]
            links = modelo.findall('link')
            self.assertEqual(len(links), len(lab.flechas))
            for fl, lk in zip(lab.flechas, links):
                x, y, z, roll, pitch, yaw = pose(lk.find('pose').text)
                r, c = fl['celda']
                cx, cy = lab.centro(r, c)
                n = g._vec_mundo(fl['pared'])
                # a 1,5 mm de la cara del bloque (0,5 de luz + la mitad del espesor) y dentro de la celda libre
                prof = (x - cx) * n[0] + (y - cy) * n[1]
                self.assertAlmostEqual(prof, lab.celda / 2 - 0.0015, places=5, msg=nid)
                # el eje X del link es la flecha; la punta debe quedar del lado que dice `dir`
                u = rotar((roll, pitch, yaw), (1, 0, 0))
                D = g._vec_mundo(fl['dir'])
                horiz = math.hypot(u[0], u[1])
                self.assertAlmostEqual(math.degrees(math.atan2(abs(u[2]), horiz)), abs(fl.get('inclinacion', 0)), delta=0.01, msg=nid)
                self.assertGreater(u[0] * D[0] + u[1] * D[1], 0.8)
                # la normal de las cajas (eje Y local) es perpendicular a la pared
                y_loc = rotar((roll, pitch, yaw), (0, 1, 0))
                self.assertAlmostEqual(abs(y_loc[0] * n[0] + y_loc[1] * n[1]), 1.0, places=6)
                # las tiras no atraviesan la pared: sus centros quedan del lado del pasillo
                for v in lk.findall('visual'):
                    px, py, pz = pose(v.find('pose').text)[:3]
                    w = rotar((roll, pitch, yaw), (px, py, pz))
                    mundo = (x + w[0], y + w[1])
                    self.assertLess((mundo[0] - cx) * n[0] + (mundo[1] - cy) * n[1], lab.celda / 2)
                    self.assertGreater(z + w[2], 0.0)
                    self.assertLess(z + w[2], lab.alto)

    def test_inclinacion_maxima(self):
        for nid, (lab, _, _) in self.labs.items():
            for fl in lab.flechas:
                self.assertLessEqual(abs(fl.get('inclinacion', 0)), 30.0)


class TestValidacion(unittest.TestCase):
    """Cada prueba parte de e2_bifurcaciones.yaml (valido) y le rompe una cosa."""

    def fixture(self):
        return g.cargar(os.path.join(MAZES, 'e2_bifurcaciones.yaml'))

    def errores(self, lab):
        return [m[1] for m in g.validar(lab)[0] if m[0] == 'ERROR']

    def test_el_fixture_es_valido(self):
        self.assertEqual(self.errores(self.fixture()), [])

    def test_largo_equivocado(self):
        lab = self.fixture()
        fl = lab.flechas[0]
        fl['largo'] = 1 if fl['largo'] != 1 else 2
        self.assertTrue(any('declara largo' in e for e in self.errores(lab)))

    def test_direccion_equivocada(self):
        lab = self.fixture()
        fl = [f for f in lab.flechas if f['pared'] in 'NS'][0]
        fl['dir'] = g.OPUESTO[fl['dir']]
        self.assertTrue(any('primer paso' in e or 'gris' in e for e in self.errores(lab)))

    def test_color_equivocado(self):
        lab = self.fixture()
        fl = [f for f in lab.flechas if f['color'] == 'rojo'][0]
        fl['color'] = 'azul'
        self.assertTrue(any('primer paso' in e or 'declara largo' in e for e in self.errores(lab)))

    def test_inclinacion_fuera_de_reglamento(self):
        lab = self.fixture()
        lab.flechas[0]['inclinacion'] = 31
        self.assertTrue(any('inclinacion' in e for e in self.errores(lab)))

    def test_flecha_en_pared_inexistente(self):
        lab = self.fixture()
        # la celda (5, 3) es un pasillo este-oeste: su pared norte existe; la celda (5, 6) no tiene pared al oeste
        lab.flechas.append({'celda': [5, 6], 'pared': 'O', 'dir': 'N', 'largo': 3, 'color': 'gris'})
        self.assertTrue(any('no hay pared' in e for e in self.errores(lab)))

    def test_flecha_fuera_de_la_cara(self):
        lab = self.fixture()
        lab.flechas[0]['desplaza_mm'] = 150
        self.assertTrue(any('se sale de la cara' in e for e in self.errores(lab)))

    def test_borde_abierto(self):
        lab = self.fixture()
        lab.filas[0] = lab.filas[0][:3] + '.' + lab.filas[0][4:]
        self.assertTrue(any('borde' in e for e in self.errores(lab)))

    def test_bloque_de_20cm_cierra_el_pasillo(self):
        lab = self.fixture()
        lab.objetos.append({'tipo': 'bloque_20cm', 'celda': [5, 4]})
        self.assertTrue(any('no es alcanzable' in e for e in self.errores(lab)))

    def test_bloque_de_8cm_deja_pasar_solo_pegado_a_la_pared(self):
        lab = self.fixture()
        lab.objetos.append({'tipo': 'bloque_8cm', 'celda': [5, 4], 'dy_mm': 115})
        self.assertEqual(self.errores(lab), [])
        lab = self.fixture()
        lab.objetos.append({'tipo': 'bloque_8cm', 'celda': [5, 4]})
        self.assertTrue(any('no es alcanzable' in e for e in self.errores(lab)))

    def test_cubo_en_zona_prohibida(self):
        lab = self.fixture()
        r, c = lab.buscar('R')[0]
        lab.filas[r] = lab.filas[r][:c] + 'Z' + lab.filas[r][c + 1:]
        errs = self.errores(lab)
        self.assertTrue(any('cubo rojo' in e for e in errs))

    def test_deposito_sin_pared_donde_apoyarse(self):
        lab = self.fixture()
        lab.canastos_pared = 'E'
        self.assertTrue(any('no tiene pared' in e for e in self.errores(lab)))


class TestSemanticaIndependiente(unittest.TestCase):
    """Rehace la regla de las flechas con otro codigo (BFS propio y recorrido por celdas), sin usar largo_de_flecha."""

    def test_cada_flecha_dice_la_verdad(self):
        for nombre in sorted(os.listdir(MAZES)):
            if not nombre.endswith('.yaml'):
                continue
            lab = g.cargar(os.path.join(MAZES, nombre))
            libres, pasable = g.construir_grafo(lab)

            def adyacentes(u):
                for d, (dr, dc) in g.DIRS.items():
                    v = (u[0] + dr, u[1] + dc)
                    eje = 'h' if d in 'EO' else 'v'
                    if v in libres and pasable[u][eje] and pasable[v][eje]:
                        yield d, v

            destinos = {}
            for simb in 'RGB':
                meta = lab.buscar(simb)[0]
                dist = {meta: 0}
                pend = [meta]
                while pend:
                    nuevo = []
                    for u in pend:
                        for _, v in adyacentes(u):
                            if v not in dist:
                                dist[v] = dist[u] + 1
                                nuevo.append(v)
                    pend = nuevo
                destinos[simb] = dist
            for fl in lab.flechas:
                celda = tuple(fl['celda'])
                objetivos = lab.significado_colores.get(fl['color'], [])
                for simb in objetivos:
                    dist = destinos[simb]
                    # camino greedy: desde la celda, ir en la direccion de la flecha mientras siga acercando al cubo
                    dr, dc = g.DIRS[fl['dir']]
                    sig = (celda[0] + dr, celda[1] + dc)
                    self.assertEqual(dist[sig], dist[celda] - 1, '%s %s' % (nombre, fl))
                    pasos = 0
                    actual = celda
                    while True:
                        sig = (actual[0] + dr, actual[1] + dc)
                        if sig in dist and dist[sig] == dist[actual] - 1 and sig in [v for _, v in adyacentes(actual)]:
                            pasos += 1
                            actual = sig
                        else:
                            break
                    self.assertEqual(min(pasos, 3), fl['largo'], '%s %s cubo %s' % (nombre, fl, simb))

    def test_cada_cubo_y_deposito_se_alcanza_desde_s(self):
        for nombre in sorted(os.listdir(MAZES)):
            if not nombre.endswith('.yaml'):
                continue
            lab = g.cargar(os.path.join(MAZES, nombre))
            libres, pasable = g.construir_grafo(lab)
            s = lab.buscar('S')[0]
            visto = {s}
            pend = [s]
            while pend:
                u = pend.pop()
                for _, v in g.vecinos(lab, libres, pasable, u):
                    if v not in visto:
                        visto.add(v)
                        pend.append(v)
            for simb in 'RGB123':
                self.assertIn(lab.buscar(simb)[0], visto, '%s %s' % (nombre, simb))


class TestAlturasYEntrada(unittest.TestCase):
    """Cubos y canastos a la altura de la base del brazo, entrada del laberinto entre paredes."""

    @classmethod
    def setUpClass(cls):
        cls.lab = g.cargar(os.path.join(MAZES, 'e2_bifurcaciones.yaml'))
        cls.abierto = g.cargar(os.path.join(MAZES, 'e6_salida_abierta.yaml'))

    def modelos(self, lab):
        msgs, info = g.validar(lab)
        raiz = ET.fromstring(g.generar_sdf(lab, info))
        return {m.get('name'): m for m in raiz.find('world').findall('model')}

    def test_cubo_apoya_a_108_5_mm_sobre_un_soporte(self):
        for lab in (self.lab, self.abierto):
            mods = self.modelos(lab)
            for color in ('rojo', 'verde', 'azul'):
                z_cubo = pose(mods['cubo_' + color].find('pose').text)[2]
                self.assertAlmostEqual(z_cubo, 0.1085 + 0.010 + 0.0002, places=5)
                soporte = mods['soporte_cubo_' + color]
                zs = pose(soporte.find('pose').text)[2]
                cil = soporte.find('link/collision/geometry/cylinder')        # pilar: cilindro negro
                alto = float(cil.find('length').text)
                self.assertAlmostEqual(float(cil.find('radius').text), 0.020, places=5)
                self.assertLess(max(pose(soporte.find('link/visual/material/diffuse').text)[:3]), 0.1)
                self.assertAlmostEqual(zs + alto / 2, 0.1085, places=5)          # la cara de arriba del soporte
                self.assertAlmostEqual(zs - alto / 2, 0.0, places=5)             # y apoya en el piso

    def test_canasto_con_el_borde_a_108_5_mm(self):
        for lab in (self.lab, self.abierto):
            mods = self.modelos(lab)
            for color in ('rojo', 'verde', 'azul'):
                z_base = pose(mods['canasto_' + color].find('pose').text)[2]
                paredes = [v for v in mods['canasto_' + color].findall('link/visual') if v.get('name') == 'v_pared_n'][0]
                zp = pose(paredes.find('pose').text)[2]
                alto = pose(paredes.find('geometry/box/size').text)[2]
                self.assertAlmostEqual(z_base + zp + alto / 2, 0.1085, places=5)
                soporte = mods['soporte_canasto_' + color]
                zs = pose(soporte.find('pose').text)[2]
                self.assertAlmostEqual(zs * 2, z_base, places=5)

    def test_el_cubo_queda_encima_del_soporte_y_no_en_el_piso(self):
        mods = self.modelos(self.lab)
        px, py = pose(mods['soporte_cubo_rojo'].find('pose').text)[:2]
        cx, cy = pose(mods['cubo_rojo'].find('pose').text)[:2]
        self.assertAlmostEqual(px, cx, places=5)
        self.assertAlmostEqual(py, cy, places=5)

    def test_el_soporte_del_cubo_va_pegado_a_una_pared(self):
        lab = self.lab
        r, c = lab.buscar('R')[0]
        x, y, lado = g.posicion_cubo(lab, 'R', (r, c))
        self.assertIsNotNone(lado)
        cx, cy = lab.centro(r, c)
        v = g._vec_mundo(lado)
        hacia = (x - cx) * v[0] + (y - cy) * v[1]
        self.assertAlmostEqual(hacia + lab.pedestal_lado / 2, lab.celda / 2 - g.CUBO_HOLGURA_PARED_M, places=6)

    def test_altura_de_flechas_configurable(self):
        lab = g.cargar(os.path.join(MAZES, 'e2_bifurcaciones.yaml'))
        lab.altura_flechas = 0.125
        for fl in lab.flechas:
            fl.pop('altura_mm', None)
        centro, _ = g.posicion_flecha(lab, lab.flechas[0])
        self.assertAlmostEqual(centro[2], 0.125, places=6)

    def test_soporte_en_el_eje_del_pasillo_bloquea(self):
        lab = g.cargar(os.path.join(MAZES, 'e2_bifurcaciones.yaml'))
        lab.cubos_cfg['R'] = {'lado': 'E'}          # contra la pared del fondo: en el eje del pasillo
        errs = [m[1] for m in g.validar(lab)[0] if m[0] == 'ERROR']
        self.assertTrue(any('no es alcanzable' in e for e in errs))

    def test_soporte_que_tapa_una_flecha(self):
        lab = g.cargar(os.path.join(MAZES, 'e2_bifurcaciones.yaml'))
        lab.cubos_cfg['R'] = {'lado': 'N'}
        r, c = lab.buscar('R')[0]
        lab.flechas.append({'celda': [r, c], 'pared': 'N', 'dir': 'O', 'largo': 3, 'color': 'verde'})
        errs = [m[1] for m in g.validar(lab)[0] if m[0] == 'ERROR']
        self.assertTrue(any('tapada' in e for e in errs), errs)

    def test_entrada_unica_y_entre_paredes(self):
        lab = self.abierto
        self.assertEqual([m for m in g.validar(lab)[0] if m[0] == 'ERROR'], [])
        # una segunda abertura en el muro que separa el area abierta
        otro = g.cargar(os.path.join(MAZES, 'e6_salida_abierta.yaml'))
        fila = otro.filas[3]
        otro.filas[3] = fila[:8] + '.' + fila[9:]
        errs = [m[1] for m in g.validar(otro)[0] if m[0] == 'ERROR']
        self.assertTrue(any('unica entrada' in e for e in errs), errs)
        # la entrada sin el bloque de un costado
        otro = g.cargar(os.path.join(MAZES, 'e6_salida_abierta.yaml'))
        fila = otro.filas[6]
        otro.filas[6] = fila[:8] + '.' + fila[9:]
        errs = [m[1] for m in g.validar(otro)[0] if m[0] == 'ERROR']
        self.assertTrue(any('no esta limitada por paredes' in e or 'unica entrada' in e for e in errs), errs)

    def test_borde_con_area_abierta_es_valido_pero_un_pasillo_no(self):
        otro = g.cargar(os.path.join(MAZES, 'e6_salida_abierta.yaml'))
        fila = otro.filas[0]
        otro.filas[0] = fila[:12] + '.' + fila[13:]
        errs = [m[1] for m in g.validar(otro)[0] if m[0] == 'ERROR']
        self.assertTrue(any('borde' in e for e in errs))


class TestSignificadoDeColores(unittest.TestCase):
    """El significado del color es un parametro; el gris no significa nada y se ignora."""

    def fixture(self):
        return g.cargar(os.path.join(MAZES, 'e2_bifurcaciones.yaml'))

    def errores(self, lab):
        return [m[1] for m in g.validar(lab)[0] if m[0] == 'ERROR']

    def test_los_mazes_entregados_no_usan_gris(self):
        for nombre in sorted(os.listdir(MAZES)):
            if nombre.endswith('.yaml'):
                lab = g.cargar(os.path.join(MAZES, nombre))
                self.assertEqual([f for f in lab.flechas if f['color'] == 'gris'], [], nombre)

    def test_una_flecha_gris_se_ignora_aunque_apunte_a_cualquier_lado(self):
        lab = self.fixture()
        lab.flechas.append({'celda': [5, 4], 'pared': 'S', 'dir': 'O', 'largo': 1, 'color': 'gris'})
        msgs = g.validar(lab)[0]
        self.assertEqual([m for m in msgs if m[0] == 'ERROR'], [])
        self.assertTrue(any('decorativas (se ignoran): 1' in m[1] for m in msgs))

    def test_gris_no_cuenta_para_la_cobertura(self):
        lab = self.fixture()
        n = len([m for m in g.validar(lab)[0] if m[0] == 'AVISO'])
        lab.flechas = [f for f in lab.flechas if f['color'] == 'gris']
        self.assertGreater(len([m for m in g.validar(lab)[0] if m[0] == 'AVISO']), n)

    def test_se_puede_cambiar_el_significado(self):
        lab = self.fixture()
        # con el significado cambiado, las flechas rojas ya no guian al cubo rojo: el validador lo nota
        lab.significado_colores['rojo'] = ['B']
        self.assertTrue(any('primer paso' in e or 'declara largo' in e for e in self.errores(lab)))
        # y si el color no significa nada, esas flechas se ignoran y ya no hay error
        lab = self.fixture()
        lab.significado_colores['rojo'] = []
        self.assertEqual(self.errores(lab), [])

    def test_significado_invalido(self):
        lab = self.fixture()
        lab.significado_colores['violeta'] = ['R']
        self.assertTrue(any('no existe' in e for e in self.errores(lab)))
        lab = self.fixture()
        lab.significado_colores['rojo'] = ['X']
        self.assertTrue(any('R, G y/o B' in e for e in self.errores(lab)))

    def test_sugerencia_usa_el_significado(self):
        lab = self.fixture()
        lab.flechas = []
        lab.significado_colores.update({'rojo': ['G'], 'verde': ['R'], 'azul': ['B']})
        sug = g.sugerir_flechas(lab)
        self.assertTrue(sug)
        self.assertNotIn('gris', [f['color'] for f in sug])
        lab.flechas = sug
        self.assertEqual(self.errores(lab), [])
        self.assertEqual({f['color'] for f in sug}, {'rojo', 'verde', 'azul'})


if __name__ == '__main__':
    unittest.main(verbosity=2)
