#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "accel_gyro.h"
#include "common.h"
#include "bmi270.h"
#include "bmi270_interface.c"
#include "esp_timer.h"
#include <math.h>

#include "driver/ledc.h"



// M1 = GPIO 4
// M2 = GPIO 5
// M3 = GPIO 0
// M4 = GPIO 21


// gyroscope is x = pitch, y = roll, z = yaw.

#define GPIO_LED_A_BLUE 1 // LED A
#define GPIO_LED_B_RED 20 // LED B
#define GPIO_LED_C_GREEN 10 // LED C

#define GPIO_MOTOR_1 4
#define GPIO_MOTOR_2 5
#define GPIO_MOTOR_3 0
#define GPIO_MOTOR_4 21

int control_throttle = 0;
int control_roll = 0;
int control_pitch = 0;
int control_yaw = 0;

int applied_throttle = 0;
int applied_roll = 0;
int applied_pitch = 0;
int applied_yaw = 0;

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define DUTY_RESOLUTION    LEDC_TIMER_13_BIT // resolution of PWM duty
#define LED_FULL_DUTY      (1<<DUTY_RESOLUTION) // maximum duty cycle


void led_initialize(void) {
    ledc_fade_func_install(0);
}

void gpio_normalize(void) {
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = (1ULL<<GPIO_MOTOR_4);
    gpio_config(&io_conf);
}

// e.g. 8 means that the max speed is 1/8 full.
#define SPEED_DIV 1 

#define SPEED_UPDATE 4

int32_t smooth_motor_speed(int32_t target, int32_t current) {
    return target;

    if (target > 255) {
        target = 255;
    } else if (target < 0) {
        target = 0;
    }

    if (target > current) {
        return current + SPEED_UPDATE;
    } else if (target < current) {
        return current - SPEED_UPDATE;
    }
    return current;
}

uint32_t motor_0_speed = 0;
uint32_t motor_1_speed = 0;
uint32_t motor_2_speed = 0;
uint32_t motor_3_speed = 0;

// Task to control motors based on stored throttle, roll, pitch, yaw
void control_task(void* arg) {
    int32_t motor_0_target = applied_throttle + applied_roll - applied_pitch + applied_yaw;
    int32_t motor_1_target = applied_throttle - applied_roll - applied_pitch - applied_yaw;
    int32_t motor_2_target = applied_throttle + applied_roll + applied_pitch - applied_yaw;
    int32_t motor_3_target = applied_throttle - applied_roll + applied_pitch + applied_yaw;

    motor_0_speed = smooth_motor_speed(motor_0_target, motor_0_speed);
    motor_1_speed = smooth_motor_speed(motor_1_target, motor_1_speed);
    motor_2_speed = smooth_motor_speed(motor_2_target, motor_2_speed);
    motor_3_speed = smooth_motor_speed(motor_3_target, motor_3_speed);

    // printf("m1 m2 m3 m4 %li %li %li %li \n", motor_speed0, motor_speed1, motor_speed2, motor_speed3);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,       // timer mode
        .duty_resolution = DUTY_RESOLUTION,      // resolution of PWM duty
        .timer_num = LEDC_HS_TIMER,              // timer index
        .freq_hz = 5000,                         // frequency of PWM signal
        .clk_cfg = LEDC_AUTO_CLK,                // Auto select the source clock
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_0 = {
        .channel    = LEDC_CHANNEL_0,
        .duty       = LED_FULL_DUTY / 256 * (motor_0_speed) / SPEED_DIV,
        .gpio_num   = GPIO_MOTOR_1,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config_t ledc_channel_1 = {
        .channel    = LEDC_CHANNEL_1,
        .duty       = LED_FULL_DUTY / 256 * (motor_1_speed) / SPEED_DIV,
        .gpio_num   = GPIO_MOTOR_2,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config_t ledc_channel_2 = {
        .channel    = LEDC_CHANNEL_2,
        .duty       = LED_FULL_DUTY / 256 * (motor_2_speed) / SPEED_DIV,
        .gpio_num   = GPIO_MOTOR_3,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };
    ledc_channel_config_t ledc_channel_3 = {
        .channel    = LEDC_CHANNEL_3,
        .duty       = LED_FULL_DUTY / 256 * (motor_3_speed) / SPEED_DIV,
        .gpio_num   = GPIO_MOTOR_4,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = LEDC_HS_TIMER
    };

    ledc_channel_config(&ledc_channel_0);
    ledc_channel_config(&ledc_channel_1);
    ledc_channel_config(&ledc_channel_2);
    ledc_channel_config(&ledc_channel_3);

    // vTaskDelay(3 / portTICK_PERIOD_MS); //portTICK_PERIOD_MS);
}

void gyro_accel_sample(struct bmi2_dev * bmi2_dev);

static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = 0;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 6,
        .scl_io_num = 7,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
        // .master.clk_speed = 1000000,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, 0, 0, 0);
}

// with USB cable backwards (towards me):
// xa is roll; axis points towards me / usb port
// ya is 


void bmi270_task(void *arg) {
    struct bmi2_dev bmi270_sensor;
    struct bmi2_sens_axes_data acc_data;
    struct bmi2_sens_axes_data gyro_data;

    printf("starting bmi270 task...");
    i2c_port_t i2c_num = (i2c_port_t)arg;

    esp_err_t e = NULL;

    if ((e = bmi270_init_mine(i2c_num, &bmi270_sensor)) != ESP_OK) {
        ESP_LOGE("BMI270", "Failed to initialize sensor");
        ESP_LOGE("BMI270", "Error: %d, %X", e, e);
        vTaskDelete(NULL);
        return;
    }

    gyro_accel_sample(&bmi270_sensor);
}

#define WIFI_SSID "Robert Drone"
#define WIFI_CHANNEL 1
#define WIFI_PASS "secretpass"

void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = 0, // strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = "", //WIFI_PASS,
            .max_connection = 10,
            .authmode = WIFI_AUTH_OPEN,
            .pmf_cfg = {
                    .required = false,
            },
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

esp_err_t command_post_handler(httpd_req_t *req) {
    size_t buf_len;
    char buf[200];

    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
        char param[32];

        if (httpd_query_key_value(buf, "throttle", param, sizeof(param)) == ESP_OK) {
            control_throttle = atoi(param);
        }
        if (httpd_query_key_value(buf, "roll", param, sizeof(param)) == ESP_OK) {
            control_roll = atoi(param);
        }
        if (httpd_query_key_value(buf, "pitch", param, sizeof(param)) == ESP_OK) {
            control_pitch = atoi(param);
        }
        if (httpd_query_key_value(buf, "yaw", param, sizeof(param)) == ESP_OK) {
            control_yaw = atoi(param);
        }

        httpd_resp_sendstr(req, "Command received");
        return ESP_OK;
    }

    httpd_resp_send_500(req);
    return ESP_FAIL;
}


void config_http() {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    httpd_start(&server, &config);

    httpd_uri_t gpio_uri = {
        .uri       = "/command",
        .method    = HTTP_POST,
        .handler   = command_post_handler,
        .user_ctx  = NULL
    };

    httpd_register_uri_handler(server, &gpio_uri);
}

void app_main(void) {
    i2c_port_t i2c_num = 0;
    // gpio_num_t scl_pin = GPIO_NUM_6;
    // gpio_num_t sda_pin = GPIO_NUM_7;

    // if (i2c_master_init(i2c_num, scl_pin, sda_pin) != ESP_OK) {
    if (i2c_master_init() != ESP_OK) {
        ESP_LOGE("I2C", "Failed to initialize I2C driver");
        return;
    }

    led_initialize();
    gpio_normalize();
    xTaskCreate(bmi270_task, "Gyro Task", 4096, (void *)i2c_num, 5, NULL);
    // xTaskCreate(control_task, "Control Task", 2048, NULL, 5, NULL);
    wifi_init_softap();
    config_http();
}

typedef struct {
    float p;
    float i;
    float d;
    float last_error;
    float integral;
} pid_controller;

pid_controller pid_x = {0.5, 0.00, 0.00, 0, 0};
pid_controller pid_y = {0.5, 0.00, 0.00, 0, 0};

static float x_estimate = 0, y_estimate = 0, z_estimate = 0;

float alpha = 0.5;

static float rad2deg = 360.0 / (2 * M_PI);
static float byte_to_deg = 360.0 / 256.0;
static float deg_to_byte = 256.0 / 360.0;

float pid_update(pid_controller* pid, float estimate, float target, float dt) {
    float error = target - estimate;

    pid->integral += error * dt;

    float derivative = (error - pid->last_error) / dt;
    pid->last_error = error;

    float output = pid->p * error + pid->i * pid->integral + pid->d * derivative;

    return output;
}

void gyro_accel_sample(struct bmi2_dev * bmi2_dev) {
    int8_t rslt;
    struct bmi2_sens_data sensor_data = { { 0 } };
    uint16_t int_status = 0;
    uint64_t prev_time_us = esp_timer_get_time();

    float xg = 0, yg = 0, zg = 0;
    float xa = 0, ya = 0, za = 0;

    bool first = false;

    rslt = set_accel_gyro_config(bmi2_dev);

    if (rslt != BMI2_OK) {
        // error
    }

    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };
    rslt = bmi270_sensor_enable(sensor_list, 2, bmi2_dev);

    float total = 0;
    int count = 0;

    while (1) {
        // printf("waiting");
        /* To get the data ready interrupt status of accel and gyro. */
        rslt = bmi2_get_int_status(&int_status, bmi2_dev);

        /* To check the data ready interrupt status and print the status for 10 samples. */
        if ((int_status & BMI2_ACC_DRDY_INT_MASK) && (int_status & BMI2_GYR_DRDY_INT_MASK)) {
            // getting sensor data takes ~2ms.
            rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev);

            /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
            xa = lsb_to_mps2(sensor_data.acc.x, 2, bmi2_dev->resolution);
            ya = lsb_to_mps2(sensor_data.acc.y, 2, bmi2_dev->resolution);
            za = lsb_to_mps2(sensor_data.acc.z, 2, bmi2_dev->resolution);

            /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
            xg = lsb_to_dps(sensor_data.gyr.x, 2000, bmi2_dev->resolution);
            yg = lsb_to_dps(sensor_data.gyr.y, 2000, bmi2_dev->resolution);
            zg = lsb_to_dps(sensor_data.gyr.z, 2000, bmi2_dev->resolution);

            // Integrate
            uint64_t current_time_us = esp_timer_get_time();
            float dt = (current_time_us - prev_time_us) / 1000000.0f; // convert us to s

            prev_time_us = current_time_us;

            // float math takes .15ms

            float pitch = atan2(ya, sqrt(xa * xa + za * za)) * rad2deg;
            float roll = atan2(-xa, za) * rad2deg;

            // Complementary filter:
            // angle = alpha * (angle + gyro * dt) + (1 - alpha) * accel

            x_estimate = alpha * (x_estimate + xg * dt) + (1 - alpha) * pitch;
            y_estimate = alpha * (y_estimate + yg * dt) + (1 - alpha) * roll;

            // PID controller
            float x_target = control_pitch * byte_to_deg;
            float y_target = control_roll * byte_to_deg;

            float x_output = pid_update(&pid_x, x_estimate, x_target, dt);
            float y_output = pid_update(&pid_y, y_estimate, y_target, dt);

            applied_pitch = (int) (x_output * deg_to_byte);
            applied_roll = (int) (y_output * deg_to_byte);
            applied_throttle = control_throttle;
            applied_yaw = control_yaw;

            // printf("xa %4.2f \t ya %4.2f \t za %4.2f \t xg %4.2f \t yg %4.2f \t zg %4.2f \n", xa, ya, za, xg, yg, zg);
            // printf("xest %4.2f \t yest %4.2f; \t xtar %4.2f \t ytar %4.2f; xout %4.2f yout %4.2f; xapp %d, yapp %d; \n", x_estimate, y_estimate, x_target, y_target, x_output, y_output, applied_pitch, applied_roll);

            total += dt;
            count += 1;

            if (count % 100 == 0) {
                printf("%f ms \n", total/count * 1000);
            }

            control_task(NULL);

            // 2.9ms for full loop; 2.75ms for just gyroscope.
        }
    }
}
