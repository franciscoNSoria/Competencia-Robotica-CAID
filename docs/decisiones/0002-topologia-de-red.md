# 0002. Topología de red

- **Estado:** Propuesta (falta confirmar con la organización)
- **Fecha:** 2026-10-14

## Contexto

El reglamento (Art. 8 y 10) dice que el robot se comunica con la PC por Wi-Fi mediante UDP/TCP,
que la PC está "solo conectada a la red del ESP32-CAM" y que está prohibido usar "conexiones
distintas a la red Wi-Fi de la ESP32-CAM". También menciona una "red local de organización".
No queda claro si la red la crea la ESP32-CAM o un router de la organización.

Conceptos:

- **Modo estación (STA):** el dispositivo se conecta a una red que ya existe, como un celular a
  un router. Necesita conocer el nombre (SSID) y la clave de esa red.
- **Modo punto de acceso (AP o SoftAP):** el dispositivo **crea** su propia red Wi-Fi y los demás
  se conectan a él. La ESP32-CAM puede hacerlo: aparece una red, por ejemplo `CAID-ROBOT`, a la que
  se conectan la PC y la ESP32. Por defecto la ESP32 en modo AP usa la IP `192.168.4.1`.
- **UDP:** protocolo de transporte que manda paquetes sueltos, sin conexión previa ni
  reenvíos. Es rápido y con latencia baja. Si se pierde un paquete, no se reenvía: para control en
  tiempo real conviene, porque una orden nueva vale más que una vieja que llega tarde. micro-ROS
  usa UDP entre la ESP32 y el **agente micro-ROS** que corre en la PC (puerto 8888). El agente
  traduce esos mensajes a tópicos ROS 2 normales.
- **TCP:** protocolo con conexión, ordenado y con reenvíos. Es confiable, pero ante pérdidas la
  latencia sube. HTTP va sobre TCP.
- **HTTP:** el protocolo de la web. La ESP32-CAM corre un servidor web chico; la PC pide
  `http://<ip-de-la-cam>:81/stream` y recibe una respuesta que no termina nunca, con una imagen
  JPEG tras otra (formato MJPEG). Un nodo de la PC lee ese flujo y lo publica como tópico de
  imagen en ROS 2.

## Decisión

Un mismo firmware que soporta dos configuraciones, elegidas por parámetro (sin recompilar):

1. **Desarrollo (casa o laboratorio):** las dos ESP32 y la PC se conectan como estaciones al
   router de siempre. La PC mantiene internet.
2. **Competencia:** la ESP32-CAM crea la red (AP). La PC (con IP fija, por ejemplo
   `192.168.4.100`) y la ESP32 de control se conectan a ella. Así se cumple "solo la red de la
   ESP32-CAM".

Flujo de datos en ambos casos:

```
ESP32 de control --UDP 8888 (micro-ROS)--> agente micro-ROS en la PC --> tópicos ROS 2
ESP32-CAM -------HTTP :81/stream (MJPEG)--> nodo puente en la PC ------> /camera/image_raw/compressed
```

**Plan B (si la organización exige que solo la ESP32-CAM use Wi-Fi):** la ESP32 de control se
comunica con la ESP32-CAM por cable (UART) y la CAM reenvía esos bytes a la PC. El firmware de
micro-ROS ya permite cambiar de transporte (Wi-Fi o serie) en la compilación.

## Pendiente de verificar

- Que la ESP32-CAM en modo AP reenvíe el tráfico entre la PC y la ESP32 mientras transmite video
  (prueba de la columna de cámara y red).
- La respuesta escrita de la organización sobre la topología.

## Consecuencias

- En desarrollo nadie pierde internet y todo funciona igual que en competencia.
- Con una sola placa Wi-Fi, la PC conectada al AP de la CAM pierde internet. En competencia está
  bien: el reglamento lo pide así, y la PC tiene que poder arrancar todo sin internet.
- SSID, clave e IP del agente se guardan en la memoria de la ESP32 (o en `secrets.h`, que no se
  versiona) y no quedan escritos en el código.
