#include "core_one.h"

#include <math.h>

#include "core_zero.h"
#include "geometry_msgs/msg/twist.h"
#include "nav_msgs/msg/odometry.h"
#include "rcl/rcl.h"
#include "rclc/executor.h"
#include "rclc/publisher.h"
#include "rclc/subscription.h"
#include "rclc/timer.h"
#include "sensor_msgs/msg/joint_state.h"
#include "sensor_msgs/msg/range.h"
#include "std_msgs/msg/int16.h"

static rcl_publisher_t g_pub_odom = {0};
static rcl_publisher_t g_pub_front = {0};
static rcl_publisher_t g_pub_left = {0};
static rcl_publisher_t g_pub_right = {0};
static bool g_ros_ready = false;

static void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    setpoint_escribir(msg->linear.x, msg->angular.z);
}

static void servo_camara_callback(const void *msgin) {
    const std_msgs__msg__Int16 *msg = (const std_msgs__msg__Int16 *)msgin;
    gestionar_servo_camara(msg->data);
}

static void brazo_callback(const void *msgin) {
    const sensor_msgs__msg__JointState *msg = (const sensor_msgs__msg__JointState *)msgin;
    if (msg->position.size > 0) {
        ejecutar_brazo_desde_ros(msg->position.data, msg->position.size);
    }
}

void iniciar_wifi(void) {
    /*
     * En el firmware final se crea la conexión Wi‑Fi/UDP + XRCE-DDS aquí.
     * Por el momento queda como punto de extensión del skeleton.
     */
}

void gestionar_servo_camara(int16_t angulo_grados) {
    mover_servo_camara(angulo_grados);
}

void ejecutar_brazo_desde_ros(const float *posiciones, size_t count) {
    ejecutar_trayectoria_brazo(posiciones, count);
}

void publicar_telemetria(const telemetria_t *snapshot) {
    if (!g_ros_ready || snapshot == NULL) {
        return;
    }

    nav_msgs__msg__Odometry odom = {0};
    sensor_msgs__msg__Range front = {0};
    sensor_msgs__msg__Range left = {0};
    sensor_msgs__msg__Range right = {0};

    odom.pose.pose.position.x = (float)snapshot->ticks_izq * 0.001f;
    odom.pose.pose.position.y = (float)snapshot->ticks_der * 0.001f;

    front.range = isfinite(snapshot->dist_frente_m) ? snapshot->dist_frente_m : 0.0f;
    left.range = isfinite(snapshot->dist_izq_m) ? snapshot->dist_izq_m : 0.0f;
    right.range = isfinite(snapshot->dist_der_m) ? snapshot->dist_der_m : 0.0f;

    front.min_range = 0.0f;
    front.max_range = 4.0f;
    left.min_range = 0.0f;
    left.max_range = 4.0f;
    right.min_range = 0.0f;
    right.max_range = 4.0f;

    rcl_publish(&g_pub_odom, &odom, NULL);
    rcl_publish(&g_pub_front, &front, NULL);
    rcl_publish(&g_pub_left, &left, NULL);
    rcl_publish(&g_pub_right, &right, NULL);
}

static void timer_telemetria_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)timer;
    (void)last_call_time;

    telemetria_t snapshot = {0};
    if (xQueueReceive(cola_telemetria, &snapshot, 0) == pdPASS) {
        publicar_telemetria(&snapshot);
    }
}

void vTaskCore0_MicroROS(void *pvParameters) {
    (void)pvParameters;

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_timer_t timer_telemetria;

    rcl_subscription_t sub_cmd_vel;
    rcl_subscription_t sub_servo_camara;
    rcl_subscription_t sub_brazo;

    geometry_msgs__msg__Twist msg_twist;
    std_msgs__msg__Int16 msg_camara;
    sensor_msgs__msg__JointState msg_brazo;

    iniciar_wifi();

    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        vTaskDelete(NULL);
    }

    if (rclc_node_init_default(&node, "esp32_caid_node", "", &support) != RCL_RET_OK) {
        vTaskDelete(NULL);
    }

    rclc_subscription_init_default(
        &sub_cmd_vel,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel"
    );

    rclc_subscription_init_default(
        &sub_servo_camara,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "/cam_angle"
    );

    rclc_subscription_init_default(
        &sub_brazo,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "/arm_pos"
    );

    rclc_publisher_init_default(
        &g_pub_odom,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom"
    );

    rclc_publisher_init_default(
        &g_pub_front,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/ultrasonicos/frente"
    );

    rclc_publisher_init_default(
        &g_pub_left,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/ultrasonicos/izquierda"
    );

    rclc_publisher_init_default(
        &g_pub_right,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        "/ultrasonicos/derecha"
    );

    rclc_timer_init_default(
        &timer_telemetria,
        &support,
        RCL_MS_TO_NS(PERIODO_TELEMETRIA_MS),
        timer_telemetria_callback
    );

    g_ros_ready = true;

    rclc_executor_init(&executor, &support.context, 4, &allocator);

    rclc_executor_add_subscription(
        &executor,
        &sub_cmd_vel,
        &msg_twist,
        &cmd_vel_callback,
        ON_NEW_DATA
    );

    rclc_executor_add_subscription(
        &executor,
        &sub_servo_camara,
        &msg_camara,
        &servo_camara_callback,
        ON_NEW_DATA
    );

    rclc_executor_add_subscription(
        &executor,
        &sub_brazo,
        &msg_brazo,
        &brazo_callback,
        ON_NEW_DATA
    );

    rclc_executor_add_timer(&executor, &timer_telemetria);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
