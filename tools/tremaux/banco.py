"""Corre el simulador de torneo sin cámara ni cubo, sobre un laberinto, y cuenta choques / cobertura / vuelta."""
import os
import sys

AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, AQUI)
sys.path.insert(0, os.path.join(AQUI, '..', 'torneo'))
sys.path.insert(0, os.path.join(AQUI, '..', 'autonomo'))
import simular as sa  # noqa: E402
import simular_torneo as st  # noqa: E402
import torneo_logica as tl  # noqa: E402
import laberintos  # noqa: E402


class SimSinCamara(st.Sim):
    def camara(self):
        # sin cámara; aprovecha para cortar la corrida cuando el piloto se detiene por seguridad
        if self.piloto.estado == tl.DETENIDO:
            self.t_max = self.t


def correr(texto, semilla, clase_piloto, pesimista=False, t_max=900.0):
    g = sa.cargar_guardia()
    lab = st.Laberinto(texto, señuelos=False)
    s = SimSinCamara(g, lab, semilla, pesimista=pesimista, t_max=t_max)
    s.piloto = clase_piloto(None, trocha=s.trocha_cfg, diametro=s.diam,
                            imprimir=(lambda m: s.log.append('%6.1f %s' % (s.t, m))))
    s.libres_total = len(lab.libres)
    s.correr()
    return s
