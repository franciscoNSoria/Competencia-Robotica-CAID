/* Prueba 00: enlace inalambrico ESP32 <-> agente micro-ROS <-> PC. Correrla primero.
 * No mueve nada. Mide la ida y vuelta de un mensaje (la PC manda un numero y la ESP32 lo devuelve), la senal del Wi-Fi y
 * el tiempo desde la ultima orden. El LED de la placa se puede prender y apagar desde la PC.
 *
 * Ordenes (/prueba/cmd = [0, orden, args]):  1 n  eco del numero n      2 x  LED (1 prendido, 0 apagado)
 * Estado (/prueba/estado):  [0, contador, ultimo_n, ms_desde_ultima_orden, rssi_dbm, segundos_encendida, memoria_libre_kB] */
#define PRUEBA_ID 0
#define PRUEBA_NOMBRE "enlace"
#include "esp_system.h"
#include "prueba_uros.h"

static volatile float ultimo_n = -1.0f;

static void prueba_init(void)
{
}

static void prueba_ciclo(void)
{
}

static void prueba_comando(const float *a, int n)
{
	int orden = (int)lroundf(a[0]);
	if (orden == 1 && n > 1) {
		ultimo_n = a[1];
	} else if (orden == 2 && n > 1) {
		led_estado(a[1] > 0.5f);
	}
}

static int prueba_estado(float *e)
{
	wifi_ap_record_t ap;
	int rssi = 0;
	if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
		rssi = ap.rssi;
	}
	e[2] = ultimo_n;
	e[3] = (float)(ms_ahora() - ultimo_cmd_ms);
	e[4] = (float)rssi;
	e[5] = (float)(esp_timer_get_time() / 1000000);
	e[6] = (float)(esp_get_free_heap_size() / 1024);
	return 7;
}

static void prueba_parar(void)
{
	led_estado(0);
}
