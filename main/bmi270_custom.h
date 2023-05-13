// #include "BMI270-Sensor-API/bmi270.h"
#include "bmi270.h"
#include "driver/i2c.h"

typedef struct {
	i2c_port_t i2c_port;
	struct bmi2_dev bmi2_dev;
} BMI_Handle_t;