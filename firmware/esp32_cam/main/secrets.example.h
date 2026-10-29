/*
 * Modelo de secrets.h. ESTE archivo se sube al repo; secrets.h NO (está en .gitignore).
 *
 * Uso: copiar este archivo como secrets.h en la misma carpeta y completar los datos.
 */
#pragma once

#define WIFI_SSID      "nombre_de_tu_red"     // solo redes de 2.4 GHz
#define WIFI_PASS      "clave_de_tu_red"

// IP fija. Dejar CAM_STATIC_IP en "" para que el router asigne una (DHCP).
#define CAM_STATIC_IP  ""                     // ej: "192.168.1.50"
#define CAM_GATEWAY    ""                     // ej: "192.168.1.1" (la IP del router)
#define CAM_NETMASK    "255.255.255.0"
