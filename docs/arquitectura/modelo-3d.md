# Modelo 3D del robot (`caid_description`)

Descripción del robot en xacro/URDF. Es el insumo de TF, RViz y Gazebo.

## De dónde salen las medidas

Todas las cotas están en [`hardware/cad/dimensions.yaml`](../../hardware/cad/dimensions.yaml), en milímetros, y
salen del ensamblado de Fusion 360 (`hardware/cad/ensamblado/Robot_armadillo_armado.stl`). El modelo las lee con
`xacro.load_yaml` y las pasa a metros. El brazo se divide con `tools/meshes/dividir_brazo.py`.

## Convenciones

- Estándar REP-103: x adelante, y a la izquierda, z arriba.
- `base_footprint` está en el piso. `base_link` está en el centro de la placa de arriba, sobre su cara superior,
  a 0,1085 m del piso.
- En el Fusion el frente del robot es −x. El marco del brazo conserva esa orientación (giro de 180° sobre z).

## Árbol de links y articulaciones

```
base_footprint
└── base_link
    ├── wheel_fl_link, wheel_fr_link, wheel_rl_link, wheel_rr_link   (continuous, eje y)
    ├── us_front, us_left, us_right                                  (fixed; el eje x apunta hacia donde mira)
    ├── camera_pan_link  (camera_pan_joint, revolute, eje z, ±90°)
    │   └── camera_link
    │       └── camera_optical_frame                                 (z hacia adelante, REP-103)
    └── arm_base_link  (arm_mount_joint, fixed)
        └── arm_turret_link  (arm_yaw_joint, eje z, ±90°)
            └── arm_upper_link  (arm_shoulder_joint, eje y, -0,5 a 2,4 rad)
                └── arm_forearm_link  (arm_elbow_joint, eje y, -2,0 a 1,2 rad, relativo al brazo principal)
                    └── arm_wrist_link  (arm_wrist_joint, eje y, ±2,8 rad)
                        ├── arm_claw_a_link  (arm_claw_a_joint, eje z, -0,6 a 0)
                        ├── arm_claw_b_link  (arm_claw_b_joint, eje z, imita a la garra a con signo contrario)
                        └── gripper_tcp                              (fixed; punta de la pinza)
```

## Posiciones principales (respecto de `base_link`)

| Elemento | x (mm) | y (mm) | z (mm) |
|---|---|---|---|
| Ruedas delanteras | +58,5 | ±65,5 | −75 |
| Ruedas traseras | −62,5 | ±65,5 | −75 |
| `us_front` (mira a +x) | 139,9 | 0 | −28,5 |
| `us_left` / `us_right` (miran a ±y) | 96,0 | ±81,5 | −34,75 |
| Eje de giro de la cámara | 94,0 | 0 | 0 |
| Base del brazo | −21,3 | 0 | 0 |

Ruedas: radio 33,5 mm y ancho 25 mm.

## Cómo verlo

```bash
cd ros2_ws && colcon build --symlink-install --packages-select caid_description
source install/setup.bash
ros2 launch caid_description display.launch.py
```

El argumento `arm` del xacro elige el brazo: `arm:=actuated` (por defecto), `arm:=fixed` (rígido) o `arm:=none`.
Pruebas: `colcon test --packages-select caid_description` (comprueban el árbol y que `check_urdf` pase).

## Provisorio y pendiente

- Masas, inercias y límites de las articulaciones del brazo son **provisorios**; los calibra la tarea MAN-01 con el
  robot real.
- Las **garras** (posición de sus ejes) son provisorias: hay dudas sobre la posición de los engranajes en el ensamblado.
- El URDF no admite lazos cerrados: el paralelogramo del brazo se aproxima con una cadena abierta y una muñeca virtual
  (para que la pinza quede horizontal, muñeca = −(hombro + codo)). Las barras de unión se ven desfasadas al mover el brazo.
- El brazo **no tiene colisiones** todavía.
- **Alcance al piso**: con la pinza horizontal, la punta no baja de unos 45 mm del piso (el centro de un cubo está a
  10 mm). En el modelo, con la pinza inclinada ~40° hacia abajo llega a ~21 mm. Decisión de diseño pendiente; depende
  de la altura real de la placa de arriba sobre el piso (96,5 u 108,5 mm), a medir en el robot.