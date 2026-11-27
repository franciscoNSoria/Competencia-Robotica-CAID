/*
 * Firmware ESP32-CAM (AI-Thinker) - CAID 2026 - tarea CAM-01
 *
 * Se conecta como estación (STA) al router y sirve:
 *   http://<ip>/            página de prueba con el video
 *   http://<ip>/status      JSON: versión, hash, ajustes, fps, señal WiFi
 *   http://<ip>/control?var=<ajuste>&val=<n>   cambia un ajuste de la cámara
 *                                     (fps_max: tope de cuadros por segundo del stream)
 *   http://<ip>/capture     una sola foto JPEG
 *   http://<ip>:81/stream   video MJPEG (lo que lee ROS)
 *   http://<ip>/servo       página con sliders para mover y calibrar el servo del GPIO13
 *   http://<ip>/servo?ang=<0-180> | ?us=<400-2600> | ?min=<us>&med=<us>&max=<us>  mover / calibrar
 *
 * Las claves WiFi van en main/secrets.h (no se sube a git).
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_app_desc.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_http_server.h"
#include "esp_camera.h"
#include "servo.h"
#if CONFIG_SPIRAM
#include "esp_psram.h"
#endif

#if __has_include("secrets.h")
#include "secrets.h"
#else
#error "Falta main/secrets.h: copia main/secrets.example.h como main/secrets.h y completa tus datos"
#endif

static const char *TAG = "cam";

/* ================= PINES AI-THINKER ================= */
#define CAM_PIN_PWDN    32
#define CAM_PIN_RESET   -1
#define CAM_PIN_XCLK    0
#define CAM_PIN_SIOD    26
#define CAM_PIN_SIOC    27
#define CAM_PIN_D7      35
#define CAM_PIN_D6      34
#define CAM_PIN_D5      39
#define CAM_PIN_D4      36
#define CAM_PIN_D3      21
#define CAM_PIN_D2      19
#define CAM_PIN_D1      18
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC   25
#define CAM_PIN_HREF    23
#define CAM_PIN_PCLK    22
// Servo en GPIO13 y LED flash (GPIO4) apagado: ver servo.c

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,
    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,       // timer 0 / canal 0 de LEDC: no usar para otra cosa
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,      // 640x480 (pedido en CAM-01)
    .jpeg_quality = 12,               // 10-12: menor número = mejor calidad y más pesado
    .fb_count = 2,
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_LATEST   // siempre el cuadro más nuevo: menos latencia
};

/* ================= ESTADO PARA /status ================= */
static volatile float s_fps = 0.0f;
/* Tope de fps del stream. Sin tope la ESP32 manda todo lo que puede y satura el WiFi: cuando la
 * señal tiene un bajón, los cuadros se acumulan en el buffer TCP y llegan a los tirones. Con
 * margen, un bajón se absorbe sin que caigan los fps. ROS usa 15. */
static volatile int s_fps_max = 15;
static bool s_psram = false;

/* ================= STREAM MJPEG (puerto 81) ================= */
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
/* Separador + encabezado de cada cuadro, en un solo envío (antes eran dos paquetes chicos) */
static const char *STREAM_PART = "\r\n--" PART_BOUNDARY "\r\n"
                                 "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler(httpd_req_t *req)
{
    char part_buf[128];
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");

    ESP_LOGI(TAG, "Cliente de stream conectado");
    int frames = 0;
    int64_t t0 = esp_timer_get_time();

    int64_t proximo = esp_timer_get_time();

    while (true) {
        // Respetar el tope de fps: esperar hasta el turno del próximo cuadro
        int fmax = s_fps_max;
        if (fmax > 0) {
            int64_t falta = proximo - esp_timer_get_time();
            if (falta > 1000) vTaskDelay(pdMS_TO_TICKS(falta / 1000));
            int64_t ahora = esp_timer_get_time();
            proximo += 1000000 / fmax;
            if (proximo < ahora) proximo = ahora;   // venía atrasado: no recuperar de golpe
        }

        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            ESP_LOGE(TAG, "Fallo la captura");
            res = ESP_FAIL;
            break;
        }
        size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART, (unsigned)fb->len);

        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);
        if (res != ESP_OK) break;   // el cliente cerró la conexión

        // fps promedio de cada segundo
        frames++;
        int64_t now = esp_timer_get_time();
        if (now - t0 >= 1000000) {
            s_fps = frames * 1000000.0f / (float)(now - t0);
            frames = 0;
            t0 = now;
        }
    }
    s_fps = 0.0f;
    ESP_LOGI(TAG, "Cliente de stream desconectado");
    return res;
}

/* ================= /capture ================= */
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

/* ================= /control?var=&val= ================= */
static esp_err_t control_handler(httpd_req_t *req)
{
    char query[128], var[32], val[16];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK ||
        httpd_query_key_value(query, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(query, "val", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Uso: /control?var=<ajuste>&val=<numero>");
        return ESP_OK;
    }

    int v = atoi(val);
    sensor_t *s = esp_camera_sensor_get();
    int r = -1;
    bool conocido = true;

    if      (!strcmp(var, "framesize"))  r = (v >= 0 && v < FRAMESIZE_INVALID) ? s->set_framesize(s, (framesize_t)v) : -1;
    else if (!strcmp(var, "quality"))    r = s->set_quality(s, v);          // 4-63
    else if (!strcmp(var, "brightness")) r = s->set_brightness(s, v);       // -2..2
    else if (!strcmp(var, "contrast"))   r = s->set_contrast(s, v);         // -2..2
    else if (!strcmp(var, "saturation")) r = s->set_saturation(s, v);       // -2..2
    else if (!strcmp(var, "awb"))        r = s->set_whitebal(s, v);         // balance de blancos auto 0/1
    else if (!strcmp(var, "awb_gain"))   r = s->set_awb_gain(s, v);         // 0/1
    else if (!strcmp(var, "wb_mode"))    r = s->set_wb_mode(s, v);          // 0 auto, 1 sol, 2 nublado, 3 oficina, 4 casa
    else if (!strcmp(var, "aec"))        r = s->set_exposure_ctrl(s, v);    // exposición auto 0/1
    else if (!strcmp(var, "aec_value"))  r = s->set_aec_value(s, v);        // 0-1200 (con aec=0)
    else if (!strcmp(var, "agc"))        r = s->set_gain_ctrl(s, v);        // ganancia auto 0/1
    else if (!strcmp(var, "gain"))       r = s->set_agc_gain(s, v);         // 0-30 (con agc=0)
    else if (!strcmp(var, "vflip"))      r = s->set_vflip(s, v);            // 0/1
    else if (!strcmp(var, "hmirror"))    r = s->set_hmirror(s, v);          // 0/1
    else if (!strcmp(var, "fps_max"))    { s_fps_max = (v < 0) ? 0 : (v > 60 ? 60 : v); r = 0; } // 0 = sin tope
    else if (!strcmp(var, "servo"))      { servo_grados((float)v); r = 0; } // grados 0-180 (calibrado)
    else if (!strcmp(var, "servo_us"))   { servo_us(v); r = 0; }             // pulso en us, sin calibrar
    else conocido = false;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (!conocido) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Ajuste desconocido");
        return ESP_OK;
    }
    if (r != 0) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "La camara rechazo el valor");
        return ESP_OK;
    }
    ESP_LOGI(TAG, "control: %s = %d", var, v);
    return httpd_resp_sendstr(req, "OK");
}

/* ================= /status ================= */
static esp_err_t status_handler(httpd_req_t *req)
{
    char json[768];
    char sha[17] = {0};
    esp_app_get_elf_sha256(sha, sizeof(sha));

    sensor_t *s = esp_camera_sensor_get();
    wifi_ap_record_t ap;
    int rssi = (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) ? ap.rssi : 0;

    snprintf(json, sizeof(json),
             "{\"firmware\":\"esp32_cam\",\"version\":\"%s\",\"elf_sha256\":\"%s\","
             "\"uptime_s\":%lld,\"psram\":%s,\"rssi_dbm\":%d,\"fps\":%.1f,\"fps_max\":%d,"
             "\"framesize\":%d,\"quality\":%d,\"brightness\":%d,\"contrast\":%d,"
             "\"saturation\":%d,\"awb\":%d,\"awb_gain\":%d,\"wb_mode\":%d,"
             "\"aec\":%d,\"aec_value\":%d,\"agc\":%d,\"gain\":%d,"
             "\"vflip\":%d,\"hmirror\":%d,"
             "\"servo_us\":%d,\"servo_grados\":%.1f,\"servo_min_us\":%d,\"servo_med_us\":%d,"
             "\"servo_max_us\":%d}",
             esp_app_get_description()->version, sha,
             (long long)(esp_timer_get_time() / 1000000), s_psram ? "true" : "false",
             rssi, s_fps, s_fps_max,
             (int)s->status.framesize, (int)s->status.quality, (int)s->status.brightness,
             (int)s->status.contrast, (int)s->status.saturation, (int)s->status.awb,
             (int)s->status.awb_gain, (int)s->status.wb_mode, (int)s->status.aec,
             (int)s->status.aec_value, (int)s->status.agc, (int)s->status.agc_gain,
             (int)s->status.vflip, (int)s->status.hmirror,
             servo_us_actual(), servo_grados_actual(), servo_cal_min(), servo_cal_med(),
             servo_cal_max());

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

/* ================= / (página de prueba) ================= */
static const char INDEX_HTML[] =
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width'><title>ESP32-CAM CAID</title></head>"
    "<body style='margin:0;background:#111;color:#eee;font-family:sans-serif'>"
    "<p style='padding:8px;margin:0'>ESP32-CAM CAID &middot; "
    "<a style='color:#8cf' href='/status'>/status</a> &middot; "
    "<a style='color:#8cf' href='/capture'>/capture</a> &middot; "
    "<a style='color:#8cf' href='/servo'>/servo</a></p>"
    "<img id=s style='width:100%;max-width:640px;display:block'>"
    "<script>document.getElementById('s').src='http://'+location.hostname+':81/stream'</script>"
    "</body></html>";

static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_sendstr(req, INDEX_HTML);
}

/* ================= /servo (calibración con sliders) ================= */
static const char SERVO_HTML[] =
    "<!doctype html><html><head><meta charset=utf-8>"
    "<meta name=viewport content='width=device-width'><title>Servo CAID</title>"
    "<style>body{margin:0;padding:12px;background:#111;color:#eee;font-family:sans-serif}"
    "input[type=range]{width:100%;height:36px}button{font-size:16px;padding:8px 12px;margin:4px}"
    "h2{margin:16px 0 4px}.v{font-size:28px;color:#7f7}</style></head><body>"
    "<p><a style='color:#8cf' href='/'>&larr; cámara</a></p>"
    "<h2>1. Pulso (microsegundos)</h2>"
    "<p>Mové despacio hasta el tope de cada lado: donde el servo deja de girar o zumba, "
    "volvé un poco. Ese es el límite.</p>"
    "<div class=v><span id=us>-</span> us</div>"
    "<input type=range id=rus min=400 max=2600 step=5>"
    "<p><button onclick=\"marca('min')\">Este es 0&deg;</button>"
    "<button onclick=\"marca('med')\">Este es 90&deg;</button>"
    "<button onclick=\"marca('max')\">Este es 180&deg;</button></p>"
    "<p>0&deg; = <b id=cmin>-</b> us &middot; 90&deg; = <b id=cmed>-</b> us &middot; "
    "180&deg; = <b id=cmax>-</b> us "
    "<button onclick=guardar()>Guardar calibración</button> <span id=ok></span></p>"
    "<h2>2. Ángulo (con la calibración)</h2>"
    "<div class=v><span id=ang>-</span>&deg;</div>"
    "<input type=range id=rang min=0 max=180 step=1>"
    "<p><button onclick=ir(0)>0&deg;</button><button onclick=ir(45)>45&deg;</button>"
    "<button onclick=ir(90)>90&deg;</button><button onclick=ir(135)>135&deg;</button>"
    "<button onclick=ir(180)>180&deg;</button></p>"
    "<script>"
    "var cal={min:0,med:0,max:0},t=0,pend=null;"
    "function q(u){return fetch(u).then(r=>r.json()).then(mostrar)}"
    "function mostrar(d){us.textContent=d.us;ang.textContent=d.grados.toFixed(0);"
    "rus.value=d.us;rang.value=d.grados;if(!cal.min){cal.min=d.min;cal.med=d.med;cal.max=d.max}"
    "pinta()}"
    "function pinta(){cmin.textContent=cal.min;cmed.textContent=cal.med;"
    "cmax.textContent=cal.max}"
    /* manda como mucho cada 60 ms mientras se arrastra */
    "function enviar(u){pend=u;if(t)return;t=setTimeout(function(){t=0;var x=pend;pend=null;"
    "q(x)},60)}"
    "rus.oninput=function(){us.textContent=rus.value;enviar('/servo?us='+rus.value)};"
    "rang.oninput=function(){ang.textContent=rang.value;enviar('/servo?ang='+rang.value)};"
    "function ir(a){q('/servo?ang='+a)}"
    "function marca(k){cal[k]=+rus.value;pinta();ok.textContent='(sin guardar)'}"
    "function guardar(){q('/servo?min='+cal.min+'&med='+cal.med+'&max='+cal.max).then(function(){"
    "ok.textContent='guardado'})}"
    "q('/servo?estado=1');"
    "</script></body></html>";

static esp_err_t servo_handler(httpd_req_t *req)
{
    char query[96], val[16];
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) {
        httpd_resp_set_type(req, "text/html");
        return httpd_resp_sendstr(req, SERVO_HTML);
    }
    char vmin[16], vmed[16], vmax[16];
    if (httpd_query_key_value(query, "min", vmin, sizeof(vmin)) == ESP_OK &&
        httpd_query_key_value(query, "max", vmax, sizeof(vmax)) == ESP_OK) {
        int med = (httpd_query_key_value(query, "med", vmed, sizeof(vmed)) == ESP_OK) ? atoi(vmed) : 0;
        servo_calibrar(atoi(vmin), med, atoi(vmax));
    } else if (httpd_query_key_value(query, "us", val, sizeof(val)) == ESP_OK) {
        servo_us(atoi(val));
    } else if (httpd_query_key_value(query, "ang", val, sizeof(val)) == ESP_OK) {
        servo_grados((float)atof(val));
    }
    char json[160];
    snprintf(json, sizeof(json), "{\"us\":%d,\"grados\":%.1f,\"min\":%d,\"med\":%d,\"max\":%d}",
             servo_us_actual(), servo_grados_actual(), servo_cal_min(), servo_cal_med(),
             servo_cal_max());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, json);
}

/* ================= SERVIDORES HTTP ================= */
static void start_webservers(void)
{
    // Puerto 80: página, estado, control y foto
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = 80;
    cfg.ctrl_port = 32768;
    httpd_handle_t web = NULL;
    if (httpd_start(&web, &cfg) == ESP_OK) {
        httpd_uri_t u_index   = { .uri = "/",        .method = HTTP_GET, .handler = index_handler };
        httpd_uri_t u_status  = { .uri = "/status",  .method = HTTP_GET, .handler = status_handler };
        httpd_uri_t u_control = { .uri = "/control", .method = HTTP_GET, .handler = control_handler };
        httpd_uri_t u_capture = { .uri = "/capture", .method = HTTP_GET, .handler = capture_handler };
        httpd_register_uri_handler(web, &u_index);
        httpd_register_uri_handler(web, &u_status);
        httpd_register_uri_handler(web, &u_control);
        httpd_register_uri_handler(web, &u_capture);
        httpd_uri_t u_servo   = { .uri = "/servo",   .method = HTTP_GET, .handler = servo_handler };
        httpd_register_uri_handler(web, &u_servo);
    } else {
        ESP_LOGE(TAG, "No arranco el servidor del puerto 80");
    }

    // Puerto 81: solo el stream (aparte, porque el stream ocupa su servidor mientras dura)
    cfg.server_port = 81;
    cfg.ctrl_port = 32769;
    httpd_handle_t stream = NULL;
    if (httpd_start(&stream, &cfg) == ESP_OK) {
        httpd_uri_t u_stream = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler };
        httpd_register_uri_handler(stream, &u_stream);
    } else {
        ESP_LOGE(TAG, "No arranco el servidor del puerto 81");
    }
}

/* ================= WIFI (modo estación) ================= */
static bool s_servers_started = false;

static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi desconectado, reintentando...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "Conectado. IP: " IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "  Pagina:  http://" IPSTR "/", IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "  Stream:  http://" IPSTR ":81/stream", IP2STR(&e->ip_info.ip));
        if (!s_servers_started) {
            start_webservers();
            s_servers_started = true;
        }
    }
}

static void wifi_init_sta(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();

    if (sizeof(CAM_STATIC_IP) > 1) {   // CAM_STATIC_IP no está vacío -> IP fija
        esp_netif_dhcpc_stop(netif);
        esp_netif_ip_info_t ip = {0};
        esp_netif_str_to_ip4(CAM_STATIC_IP, &ip.ip);
        esp_netif_str_to_ip4(CAM_NETMASK, &ip.netmask);
        if (sizeof(CAM_GATEWAY) > 1) esp_netif_str_to_ip4(CAM_GATEWAY, &ip.gw);
        ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip));
        ESP_LOGI(TAG, "IP fija: %s", CAM_STATIC_IP);
    } else {
        ESP_LOGI(TAG, "IP por DHCP");
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_ps(WIFI_PS_NONE);   // sin ahorro de energía: mucha menos latencia
    esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20);  // 20 MHz: aguanta mejor la interferencia que 40
    esp_wifi_set_max_tx_power(80);   // 20 dBm, el máximo (unidades de 0,25 dBm)
    ESP_LOGI(TAG, "Conectando a la red \"%s\"...", WIFI_SSID);
}

/* ================= CÁMARA ================= */
static esp_err_t init_camera(void)
{
#if CONFIG_SPIRAM
    s_psram = esp_psram_is_initialized();
#endif
    if (!s_psram) {
        ESP_LOGW(TAG, "Sin PSRAM: bajo a QVGA con 1 buffer");
        camera_config.fb_location = CAMERA_FB_IN_DRAM;
        camera_config.fb_count = 1;
        camera_config.frame_size = FRAMESIZE_QVGA;
        camera_config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
    }

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error al iniciar la camara: 0x%x", err);
        return err;
    }
    sensor_t *s = esp_camera_sensor_get();
    // Cámara montada girada 180° (2026-11-05): se invierten los dos. Lo hace el sensor, no
    // cuesta fps. Si se vuelve a montar derecha: vflip 1, hmirror 0.
    s->set_vflip(s, 0);
    s->set_hmirror(s, 1);
    ESP_LOGI(TAG, "Camara lista (PSRAM: %s)", s_psram ? "si" : "no");
    return ESP_OK;
}

/* ================= MAIN ================= */
void app_main(void)
{
    char sha[17] = {0};
    esp_app_get_elf_sha256(sha, sizeof(sha));
    ESP_LOGI(TAG, "Firmware esp32_cam v%s (sha %s)", esp_app_get_description()->version, sha);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    servo_init();   // GPIO13 (y LED flash apagado); lee la calibración de NVS

    if (init_camera() != ESP_OK) {
        ESP_LOGE(TAG, "Reinicio en 2 s...");
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    }
    wifi_init_sta();
}
