// Callbacks de Suscriptores
// Subscripcion a los comandos de velocidad enviados por la PC segun las desiciones tomadas por los nodos de ros2 que estan consumiendo el sream de la camara
void cmd_vel_callback(const void * msgin) {
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    // Proteger escritura con Mutex
    if (xSemaphoreTake(mutex_velocidad, portMAX_DELAY) == pdTRUE) {
        global_target_v = msg->linear.x;
        global_target_w = msg->angular.z;
        tiempo_ultimo_comando = millis(); // Resetea el Watchdog del Core 0
        xSemaphoreGive(mutex_velocidad);
    }
}

// Segun lo que ordene la PC la camara se mueve para tener un mayor rango de vision
void servo_camara_callback(const void * msgin) {
    const std_msgs__msg__Int16 * msg = (const std_msgs__msg__Int16 *)msgin;
    
    // Mover el servo de 0 a 180 grados
    // Interactua directamente con el hardware
    escribir_pwm_servo_camara(msg->data); 
}

// Se recibe la orden desde la PC de mover el brazo porque se detecto un cubo 
void brazo_callback(const void * msgin) {
    const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *)msgin;
    // Mover articulaciones del brazo y cerrar pinza para agarrar el cubo
    ejecutar_trayectoria_brazo(msg->position[0], msg->position[1], msg->position[2], msg->position[3]);
}

// Callback de Temporizador para Publicar Telemetría (ej. a 10 Hz)
void timer_telemetria_callback(rcl_timer_t * timer, int64_t last_call_time) {
    struct TelemetriaRobot datos_core0;
    
    // Extraer datos de la cola (sin bloquear eternamente)
    if (xQueueReceive(queue_telemetria, &datos_core0, 0) == pdPASS) {
        
        // Armar y publicar odometría para enviar a la PC
        odom_msg.pose.pose.position.x = calcular_x(datos_core0.ticks);
        // ...
        rcl_publish(&publisher_odom, &odom_msg, NULL);
        
        // Armar y publicar rangos de los 3 ultrasonidos para enviar a la PC
        // TODO: Conviene mandarlo como tres topicos separados o todos juntos?
        rango_msg_frente.range = datos_core0.dist_frente;
        rcl_publish(&publisher_ultrasonico_frente, &rango_msg_frente, NULL);
        // (Repetir para izquierdo y derecho)
    }
}

// Tarea Principal del Core 1
void vTaskCore1_MicroROS(void *pvParameters) {
    // Inicializar Wi-Fi y Transporte UDP
    iniciar_wifi();
    
    /// Asignador de memoria de micro-ROS
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;
    rcl_timer_t timer_telemetria;

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "esp32_caid_node", "", &support);
    
    // Crear Suscriptores asociándolos a los tipos de mensaje correctos
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

    // TODO: Revisar cual es mejor tipo de dato para mandar la posicion del brazo
    // Cuantos servos son?
    rclc_subscription_init_default(
        &sub_brazo, 
        &node, 
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), 
        "/arm_pos"
    );

    // 4. Crear Publicadores y el Timer (Ej: 10 Hz -> 100 ms)
    // (Asumiendo que rclc_publisher_init_default ya se hizo para odom y ultrasonido)
    rclc_timer_init_default(
        &timer_telemetria, 
        &support, 
        RCL_MS_TO_NS(100), 
        timer_telemetria_callback
    );
    
    // 5. INICIALIZAR EXECUTOR CON LA CAPACIDAD CORRECTA
    // ¡Ojo acá! El "4" es porque tenemos 3 Suscriptores + 1 Timer
    rclc_executor_init(&executor, &support.context, 4, &allocator);
    
    // 6. Añadir los Suscriptores y sus Callbacks al Executor
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

    // Añadir el Timer al Executor
    rclc_executor_add_timer(&executor, &timer_telemetria);
    
    while(1) {
        // Ejecutar las tareas de micro-ROS (escuchar red y procesar callbacks)
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
        
        // Pequeño delay de RTOS para no saturar el Watchdog de FreeRTOS
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}