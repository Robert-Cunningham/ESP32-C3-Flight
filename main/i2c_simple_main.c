// /* i2c - Simple example Simple I2C example that shows how to initialize I2C as well as reading and writing from and to registers for a sensor connected over I2C.  The sensor used in this example is a MPU9250 inertial measurement unit.  For other examples please check: https://github.com/espressif/esp-idf/tree/master/examples See README.md file to get detailed usage of this example.  This example code is in the Public Domain (or CC0 licensed, at your option.) Unless required by applicable law or agreed to in writing, this software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  */ /* #include <stdio.h> #include "esp_log.h" #include "driver/i2c.h" #include "bmiconf.c" static const char *TAG = "i2c-simple-example"; 
// #define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL      /*!< GPIO number used for I2C master clock */
// #define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA      /*!< GPIO number used for I2C master data  */
// #define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
// #define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_TIMEOUT_MS       1000
// 
// #define MPU9250_SENSOR_ADDR                 0x68        /*!< Slave address of the MPU9250 sensor */
// #define MPU9250_WHO_AM_I_REG_ADDR           0x00        /*!< Register addresses of the "who am I" register */
// 
// #define MPU9250_PWR_MGMT_1_REG_ADDR         0x6B        /*!< Register addresses of the power managment register */
// #define MPU9250_RESET_BIT                   7
// 
// static esp_err_t mpu9250_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     return i2c_master_write_read_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// }
// 
// static esp_err_t mpu9250_register_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     int ret;
//     uint8_t write_buf[2] = {reg_addr, data};
// 
//     ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// 
//     return ret;
// }
// 


// void app_main(void)
// {
//     uint8_t data[2];
//     ESP_ERROR_CHECK(i2c_master_init());
//     ESP_LOGI(TAG, "I2C initialized successfully");
// 
//     ESP_ERROR_CHECK(mpu9250_register_read(MPU9250_WHO_AM_I_REG_ADDR, data, 1));
//     ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
// 
//     ESP_ERROR_CHECK(mpu9250_register_write_byte(0x7C, 0x00, 1));
// 
//     i2c_master_write_to_device(I2C_MASTER_NUM, MPU9250_SENSOR_ADDR, bmi270_config_file, sizeof(bmi270_config_file), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
// 
// 
//     // ESP_ERROR_CHECK(mpu9250_register_write_byte(0x7E, 0xB6));
// 
// 
//     ESP_ERROR_CHECK(i2c_driver_delete(I2C_MASTER_NUM));
//     ESP_LOGI(TAG, "I2C de-initialized successfully");
// }

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_system.h"
#include "esp_log.h"
// #include "BMI270-Sensor-API/bmi270.h"
// #include "BMI270-Sensor-API/bmi270.c"
// #include "common.h"
#include "bmi270.h"
#include "bmi270_custom.c"
// #include "common.h"

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi2_dev);
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);
static void gyro_accel_sample(struct bmi2_dev * bmi2_dev);

// static esp_err_t i2c_master_init(i2c_port_t i2c_num, gpio_num_t scl, gpio_num_t sda) {
//     i2c_config_t conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = sda,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_io_num = scl,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = 400000 // 400 kHz
//     };
//     esp_err_t err = i2c_param_config(i2c_num, &conf);
//     if (err != ESP_OK) {
//         return err;
//     }
//     return i2c_driver_install(i2c_num, conf.mode, 0, 0, 0);
// }

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

    i2c_port_t i2c_num = (i2c_port_t)arg;

    esp_err_t e = NULL;

    if ((e = bmi270_init_mine(i2c_num, &bmi270_sensor)) != ESP_OK) {
        ESP_LOGE("BMI270", "Failed to initialize sensor");
        ESP_LOGE("BMI270", "Error: %d, %X", e, e);
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        gyro_accel_sample(&bmi270_sensor);
        /*
        struct bmi2_sens_data sensor_data[2];

        if (bmi2_get_sensor_data(sensor_data, &bmi270_sensor) == BMI2_OK) {
            ESP_LOGI("BMI270", "Accel: X: %d, Y: %d, Z: %d", sensor_data[0].acc.x, sensor_data[0].acc.y, sensor_data[0].acc.z);
            ESP_LOGI("BMI270", "Gyro: X: %d, Y: %d, Z: %d", sensor_data[1].gyr.x, sensor_data[1].gyr.y, sensor_data[1].gyr.z);
        } else {
            ESP_LOGE("BMI270", "Failed to read sensor data");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Delay for 100ms
        */
    }
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

    xTaskCreate(bmi270_task, "bmi270_task", 4096, (void *)i2c_num, 5, NULL);
}

void gyro_accel_sample(struct bmi2_dev * bmi2_dev) {
    /* Accel and gyro configuration settings. */
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Variable to define limit to print accel data. */
    uint8_t limit = 10;

    /* Assign accel and gyro sensor to variable. */
    uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

    /* Create an instance of sensor data structure. */
    struct bmi2_sens_data sensor_data = { { 0 } };

    /* Initialize the interrupt status of accel and gyro. */
    uint16_t int_status = 0;

    uint8_t indx = 1;

    float xg = 0, yg = 0, zg = 0;
    float xa = 0, ya = 0, za = 0;

    rslt = set_accel_gyro_config(bmi2_dev);

    // bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE:
            * Accel and Gyro enable must be done after setting configurations
            */
        rslt = bmi270_sensor_enable(sensor_list, 2, bmi2_dev);
        // bmi2_error_codes_print_result(rslt);

        /* Loop to print accel and gyro data when interrupt occurs. */
        while (indx <= limit)
        {
            /* To get the data ready interrupt status of accel and gyro. */
            rslt = bmi2_get_int_status(&int_status, bmi2_dev);
            // bmi2_error_codes_print_result(rslt);

            /* To check the data ready interrupt status and print the status for 10 samples. */
            if ((int_status & BMI2_ACC_DRDY_INT_MASK) && (int_status & BMI2_GYR_DRDY_INT_MASK))
            {
                /* Get accel and gyro data for x, y and z axis. */
                rslt = bmi2_get_sensor_data(&sensor_data, bmi2_dev);
                // bmi2_error_codes_print_result(rslt);

                // printf("\n*******  Accel(Raw and m/s2) Gyro(Raw and dps) data : %d  *******\n", indx);

                

                // printf("\nAcc_x = %d\t", sensor_data.acc.x);
                // printf("Acc_y = %d\t", sensor_data.acc.y);
                // printf("Acc_z = %d", sensor_data.acc.z);

                /* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
                xa = lsb_to_mps2(sensor_data.acc.x, 2, bmi2_dev->resolution);
                ya = lsb_to_mps2(sensor_data.acc.y, 2, bmi2_dev->resolution);
                za = lsb_to_mps2(sensor_data.acc.z, 2, bmi2_dev->resolution);

                /* Print the data in m/s2. */
                // printf("\nAcc_ms2_X = %4.2f, Acc_ms2_Y = %4.2f, Acc_ms2_Z = %4.2f\n", x, y, z);

                // printf("\nGyr_X = %d\t", sensor_data.gyr.x);
                // printf("Gyr_Y = %d\t", sensor_data.gyr.y);
                // printf("Gyr_Z= %d\n", sensor_data.gyr.z);

                /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                xg = lsb_to_dps(sensor_data.gyr.x, 2000, bmi2_dev->resolution);
                yg = lsb_to_dps(sensor_data.gyr.y, 2000, bmi2_dev->resolution);
                zg = lsb_to_dps(sensor_data.gyr.z, 2000, bmi2_dev->resolution);

                /* Print the data in dps. */
                // printf("Gyro_DPS_X = %4.2f, Gyro_DPS_Y = %4.2f, Gyro_DPS_Z = %4.2f\n", x, y, z);

                printf("%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f \n", xa, ya, za, xg, yg, zg);

                indx++;
            }
        }
    }
}

#include <stdio.h>
#include "bmi270_legacy.h"
#include "common.h"

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/*! Macros to select the sensors                   */
#define ACCEL          UINT8_C(0x00)
#define GYRO           UINT8_C(0x01)

static int8_t set_accel_gyro_config(struct bmi2_dev *bmi2_dev)
{
    /* Status of api are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer and gyro configuration. */
    struct bmi2_sens_config config[2];

    /* Configure the type of feature. */
    config[ACCEL].type = BMI2_ACCEL;
    config[GYRO].type = BMI2_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi270_get_sensor_config(config, 2, bmi2_dev);
    // bmi2_error_codes_print_result(rslt);

    /* Map data ready interrupt to interrupt pin. */
    rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi2_dev);
    // bmi2_error_codes_print_result(rslt);

    if (rslt == BMI2_OK)
    {
        /* NOTE: The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;

        /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
        config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

        /* The bandwidth parameter is used to configure the number of sensor samples that are averaged
         * if it is set to 2, then 2^(bandwidth parameter) samples
         * are averaged, resulting in 4 averaged samples.
         * Note1 : For more information, refer the datasheet.
         * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
         * this has an adverse effect on the power consumed.
         */
        config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

        /* Enable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         * For more info refer datasheet.
         */
        config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

        /* The user can change the following configuration parameters according to their requirement. */
        /* Set Output Data Rate */
        config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

        /* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
        config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

        /* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
        config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

        /* Enable/Disable the noise performance mode for precision yaw rate sensing
         * There are two modes
         *  0 -> Ultra low power mode(Default)
         *  1 -> High performance mode
         */
        config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

        /* Enable/Disable the filter performance mode where averaging of samples
         * will be done based on above set bandwidth and ODR.
         * There are two modes
         *  0 -> Ultra low power mode
         *  1 -> High performance mode(Default)
         */
        config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

        /* Set the accel and gyro configurations. */
        rslt = bmi270_set_sensor_config(config, 2, bmi2_dev);
        // bmi2_error_codes_print_result(rslt);
    }

    return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (dps / ((half_scale))) * (val);
}
