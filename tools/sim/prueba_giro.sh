#!/usr/bin/env bash
# Prueba de giro en el lugar: compara el giro real (pose de Gazebo) con el de la odometría.
# Uso, con la simulación corriendo: bash tools/sim/prueba_giro.sh [vel_rad_s] [segundos]

VEL=${1:-0.5}
SEG=${2:-3}

# Pose real del robot (x y z roll pitch yaw); reintenta si Gazebo no responde.
pose() {
    local out
    for _ in 1 2 3; do
        out=$(gz model -m caid_bot -p 2>/dev/null | tail -2 | tr -d '[]')
        if echo "$out" | grep -qE '^ *-?[0-9]'; then
            echo "$out"
            return
        fi
        sleep 1
    done
}

# Orientación de la odometría (z w), con el tipo explícito para no depender del descubrimiento.
odom() {
    ros2 topic echo /sim/odom nav_msgs/msg/Odometry --once \
        --field pose.pose.orientation | grep -E '^(z|w):' | awk '{print $2}' | tr '\n' ' '
}

antes=$(pose)
odom_antes=$(odom)
ros2 topic pub --once /sim/cmd_vel geometry_msgs/msg/Twist "{angular: {z: $VEL}}" > /dev/null
sleep "$SEG"
ros2 topic pub --once /sim/cmd_vel geometry_msgs/msg/Twist "{}" > /dev/null
sleep 2
despues=$(pose)
odom_despues=$(odom)

python3 - "$antes" "$despues" "$odom_antes" "$odom_despues" <<'EOF'
import math
import sys


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def pose(txt):
    v = [float(x) for x in txt.split()]
    return v[0], v[1], v[5]


def yaw_odom(txt):
    z, w = [float(x) for x in txt.split()]
    return 2 * math.atan2(z, w)


try:
    x0, y0, a0 = pose(sys.argv[1])
    x1, y1, a1 = pose(sys.argv[2])
    real = math.degrees(wrap(a1 - a0))
    odom = math.degrees(wrap(yaw_odom(sys.argv[4]) - yaw_odom(sys.argv[3])))
except ValueError:
    print('No pude leer la pose o la odometria. Datos recibidos:')
    for dato in sys.argv[1:]:
        print(repr(dato))
    sys.exit(1)

print('giro real       : %.1f grados' % real)
print('giro odometria  : %.1f grados' % odom)
print('real / odometria: %.0f %%' % (100 * real / odom))
print('desplazamiento  : %.1f cm' % (100 * math.hypot(x1 - x0, y1 - y0)))
EOF