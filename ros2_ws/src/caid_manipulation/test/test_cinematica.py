import math

from caid_manipulation import cinematica as k


def test_calibracion_contra_las_fotos():
    # Puntos medidos (grados): hombro con codo 180 y codo con hombro 60.
    for h, a in ((10, 122.9), (25, 109.2), (40, 94.5), (55, 78.3), (85, 47.7)):
        assert abs(k.alfa(h) - a) < 5.5, h
    for c, b in ((80, -72.6), (100, -56.0), (140, -19.6), (180, 8.1)):
        assert abs(k.beta(c) - b) < 3.5, c


def test_reposo_en_el_urdf():
    j = k.servos_a_articulaciones(**k.REPOSO)
    assert abs(math.degrees(j['arm_shoulder_joint']) - 13.3) < 0.2
    assert abs(math.degrees(j['arm_elbow_joint']) + 50.0) < 0.3
    assert abs(j['arm_wrist_joint'] + j['arm_shoulder_joint'] + j['arm_elbow_joint']) < 1e-9
    assert j['arm_yaw_joint'] == 0.0


def test_pose_de_agarre_de_las_fotos_da_16_cm_al_costado():
    # Páginas 17 y 19 del PDF (la misma pose vista de costado y de frente): hombro 104, codo 140.
    # El equipo pide el cubo a 16 cm del costado del robot (81,5 mm de medio ancho).
    r, _ = k.punta_plano(104.0, 140.0)
    assert abs((r - 81.5) - 160.0) < 6.0


def test_ik_y_fk_se_deshacen():
    for x, y, z in ((0.20, 0.0, 0.010), (0.0, 0.175, 0.010), (-0.02, -0.175, 0.05),
                    (0.17, 0.05, 0.08)):
        sol = k.ik(x, y, z)
        assert sol is not None, (x, y, z)
        px, py, pz = k.punta(*sol)
        assert math.dist((px, py, pz), (x, y, z)) < 1e-6


def test_fuera_de_alcance():
    assert k.ik(0.60, 0.0, 0.0) is None
