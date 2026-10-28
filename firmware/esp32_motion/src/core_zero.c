void vTaskCore0_BucleControl(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrecuencia = pdMS_TO_TICKS(20); // 50 Hz

    while(1) {
        // Leer Hardware
        // Lecturas crudas para el PID
        // Los calculos para obtener los datos mas "humanos" son delegados al core 1
        leer_encoders(&ticks_izq, &ticks_der); // ticks
        leer_ultrasonicos(&dist_frente, &dist_izq, &dist_der); // echo

        // Watchdog
        // Si pasa una cierta cantidad de tiempo sin recibir ningun comando desde la computadora -> core_one -> queue
        // por seguridad se setean las velocidades en cero para evitar chocar
        if (tiempo_sin_recibir_comandos > 500ms) {
            emergency_stop();
        }
        
        // Apaga los motores si esta cerca de chocar
        if (dist_frente < UMBRAL_CRITICO) {
            emergency_stop();
        }

        // Parada de emergencia 
        // Apaga los motores para evitar colisiones
        emergency_stop () {
            target_v = 0; // velocidad lineal
            target_w = 0; // velocidad angular
        }

        // Chequeo de Bloqueo (Stall)
        if (target_v > 0 && (ticks_izq == 0 && ticks_der == 0 || dist_frente < UMBRAL_CRITICO)) {
            // Apagar motores, marcar error
        }

        // Calcular PID
        // Control PID para asegurar que el robot se mueva recto a travez de los pasillos
        // y corrija su posicion para que no se choque con las paredes que tiene a sus costados
        control_pid(target_v, target_w, ...);
        
        // Manda el duty del pwm a los motores para que giren a la velocidad que corresponde
        escribir_motores(pwm_izq, pwm_der);

        // Enviar datos al Core 1 (No bloqueante)
        // Empaqueta los datos en un struct y delega la responsabilidad de publicar los datos de /odom y /dist_frente /dist_izq /dist_der al core 1
        struct TelemetriaRobot datos_actuales;

        xQueueSend(telemetriaQueue, &datos_actuales, 0);

        // TODO: Deberia recibir los daos de /cmd_vel del Core 1?
        // TODO: Faltaria el mutex de velocidad compartido entre nodos?

        // Dormir exactamente hasta el próximo ciclo de 20ms
        // Esto hace que la telemetria se mande cada ciclo de xFrecuencia
        vTaskDelayUntil(&xLastWakeTime, xFrecuencia);
    }
}