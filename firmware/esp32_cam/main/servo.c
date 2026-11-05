#include "servo.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "nvs.h"

static const char *TAG = "servo";

#define SERVO_MODO   LEDC_LOW_SPEED_MODE
#define SERVO_TIMER  LEDC_TIMER_1
#define SERVO_CANAL  LEDC_CHANNEL_1
#define SERVO_HZ     50
#define SERVO_BITS   LEDC_TIMER_14_BIT     // 16384 cuentas en 20 ms: 1,22 us por cuenta
#define PERIODO_US   (1000000 / SERVO_HZ)

#define NVS_ESPACIO  "servo"

static int s_us = 1500;
static int s_min = 1000;     // 0 grados (se pisa con lo guardado en NVS)
static int s_med = 1500;     // 90 grados
static int s_max = 2000;     // 180 grados

static int recortar(int us)
{
    if (us < SERVO_US_LIMITE_MIN) return SERVO_US_LIMITE_MIN;
    if (us > SERVO_US_LIMITE_MAX) return SERVO_US_LIMITE_MAX;
    return us;
}

static void cargar_calibracion(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_ESPACIO, NVS_READONLY, &h) != ESP_OK) return;   // nunca se calibró
    int32_t a, b, m;
    if (nvs_get_i32(h, "min", &a) == ESP_OK && nvs_get_i32(h, "max", &b) == ESP_OK) {
        s_min = recortar(a);
        s_max = recortar(b);
        // calibraciones viejas no tienen 90 grados: punto medio
        s_med = (nvs_get_i32(h, "med", &m) == ESP_OK) ? recortar(m) : (s_min + s_max) / 2;
    }
    nvs_close(h);
}

void servo_init(void)
{
    // LED flash apagado (GPIO4 se prende solo al arrancar en algunas placas)
    gpio_reset_pin(LED_FLASH_GPIO);
    gpio_set_direction(LED_FLASH_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_FLASH_GPIO, 0);
    cargar_calibracion();
    ledc_timer_config_t t = {
        .speed_mode = SERVO_MODO,
        .timer_num = SERVO_TIMER,
        .duty_resolution = SERVO_BITS,
        .freq_hz = SERVO_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = {
        .gpio_num = SERVO_GPIO,
        .speed_mode = SERVO_MODO,
        .channel = SERVO_CANAL,
        .timer_sel = SERVO_TIMER,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
    servo_us(s_med);                 // arranca en 90 grados
    ESP_LOGI(TAG, "Servo en GPIO%d, calibración 0°=%d us, 90°=%d us, 180°=%d us", SERVO_GPIO,
             s_min, s_med, s_max);
}

void servo_us(int us)
{
    s_us = recortar(us);
    uint32_t cuentas = (uint32_t)((int64_t)s_us * (1 << SERVO_BITS) / PERIODO_US);
    ledc_set_duty(SERVO_MODO, SERVO_CANAL, cuentas);
    ledc_update_duty(SERVO_MODO, SERVO_CANAL);
}

void servo_grados(float grados)
{
    if (grados < 0) grados = 0;
    if (grados > 180) grados = 180;
    float us = (grados <= 90) ? s_min + (s_med - s_min) * grados / 90.0f
                              : s_med + (s_max - s_med) * (grados - 90) / 90.0f;
    servo_us((int)(us + 0.5f));
}

bool servo_calibrar(int min_us, int med_us, int max_us)
{
    s_min = recortar(min_us);
    s_max = recortar(max_us);
    s_med = (med_us > 0) ? recortar(med_us) : (s_min + s_max) / 2;
    nvs_handle_t h;
    if (nvs_open(NVS_ESPACIO, NVS_READWRITE, &h) != ESP_OK) return false;
    bool ok = nvs_set_i32(h, "min", s_min) == ESP_OK && nvs_set_i32(h, "med", s_med) == ESP_OK
              && nvs_set_i32(h, "max", s_max) == ESP_OK && nvs_commit(h) == ESP_OK;
    nvs_close(h);
    ESP_LOGI(TAG, "Calibración guardada: 0°=%d us, 90°=%d us, 180°=%d us (%s)", s_min, s_med,
             s_max, ok ? "ok" : "ERROR");
    return ok;
}

int servo_us_actual(void) { return s_us; }
int servo_cal_min(void) { return s_min; }
int servo_cal_med(void) { return s_med; }
int servo_cal_max(void) { return s_max; }

float servo_grados_actual(void)
{
    // inversa de los dos tramos (sirve aunque max < min, servo montado al revés)
    bool primer_tramo = (s_med >= s_min) ? (s_us <= s_med) : (s_us >= s_med);
    if (primer_tramo) {
        return (s_med == s_min) ? 0 : 90.0f * (s_us - s_min) / (float)(s_med - s_min);
    }
    return (s_max == s_med) ? 90 : 90.0f + 90.0f * (s_us - s_med) / (float)(s_max - s_med);
}
