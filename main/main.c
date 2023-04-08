#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/timers.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "inv.h"
#include "driver/i2c.h"
#include "as5600.h"

#define STEP_IO0         GPIO_NUM_5
#define STEP_IO1         GPIO_NUM_17
#define DIR_IO0          GPIO_NUM_4
#define DIR_IO1          GPIO_NUM_16

#define I2C_MASTER_SDA_IO0   GPIO_NUM_13
#define I2C_MASTER_SCL_IO0   GPIO_NUM_15
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_NUM0              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C_MASTER_NUM1              1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_SDA_IO1   GPIO_NUM_2
#define I2C_MASTER_SCL_IO1   GPIO_NUM_0

#define steps_per_deg_0 13.33
#define steps_per_deg_1 13.33

volatile uint8_t step0_state = 0;
volatile uint8_t step1_state = 0;
volatile uint32_t step_counts[2];


static const char *TAG = "SCARA";

static bool IRAM_ATTR timer0_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    if (step0_state) {
        step_counts[0] = step_counts[0] - 1;
    }

    if (step_counts[0] <= 0) {
        gptimer_stop(timer);
    }
    step0_state = !step0_state;
    gpio_set_level(STEP_IO0, step0_state);

    return (high_task_awoken == pdTRUE);
}

static bool IRAM_ATTR timer1_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;

    if (step1_state) {
            step_counts[1] = step_counts[1] - 1;
    }

    if (step_counts[1] <= 0) {
        gptimer_stop(timer);
    }

    step1_state = !step1_state;
    gpio_set_level(STEP_IO1, step1_state);

    return (high_task_awoken == pdTRUE);
}

//============FUNCTIONS=================================
void move_arm_by_ang(const float (delta_angs)[2], gptimer_handle_t gptimer0, gptimer_handle_t gptimer1) 
{
    step_counts[0] = abs(delta_angs[0] * steps_per_deg_0);
    step_counts[1] = abs(delta_angs[1] * steps_per_deg_1);

    if (delta_angs[0] < 0)
    {
        gpio_set_level(DIR_IO0, 1);
    } else {
        gpio_set_level(DIR_IO0, 0);
    }

    if (delta_angs[1] < 0)
    {
        gpio_set_level(DIR_IO1, 1);
    } else {
        gpio_set_level(DIR_IO1, 0);
    }

    ESP_LOGI("MOVE ARM", "stepcounts %lu %lu", (unsigned long)step_counts[0], (unsigned long)step_counts[1]);

    gptimer_start(gptimer0);
    gptimer_start(gptimer1);
}

esp_err_t i2c_master_init(int i2c_master_port, int sda_pin, int scl_pin)
{

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void app_main(void)
{
//-------------CONFIG GPIO--------------------------------
    gpio_set_direction(DIR_IO0, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_IO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_IO0, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_IO1, GPIO_MODE_OUTPUT);

//-------------CONFIG TIMER 0-----------------------------------
    gptimer_handle_t gptimer0 = NULL;
    gptimer_config_t timer0_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000000, // 10MHz, 1 tick=0.1us
    };
    gptimer_new_timer(&timer0_config, &gptimer0);

    gptimer_event_callbacks_t cbs0 = {
        .on_alarm = timer0_alarm_cb,
    };
    gptimer_register_event_callbacks(gptimer0, &cbs0, NULL);

    gptimer_enable(gptimer0);

    gptimer_alarm_config_t alarm0_config = {
        .reload_count = 0,
        .alarm_count = 1000,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(gptimer0, &alarm0_config);

    //-------------CONFIG TIMER 1-----------------------------------
    gptimer_handle_t gptimer1 = NULL;
    gptimer_config_t timer1_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000000, // 10MHz, 1 tick=1us
    };
    gptimer_new_timer(&timer1_config, &gptimer1);

    gptimer_event_callbacks_t cbs1 = {
        .on_alarm = timer1_alarm_cb,
    };
    gptimer_register_event_callbacks(gptimer1, &cbs1, NULL);

    gptimer_enable(gptimer1);

    gptimer_alarm_config_t alarm1_config = {
        .reload_count = 0,
        .alarm_count = 1000,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(gptimer1, &alarm1_config);

    //---------------CONFIG I2C----------------------------------
    
    i2c_master_init(I2C_MASTER_NUM0, I2C_MASTER_SDA_IO0, I2C_MASTER_SCL_IO0);
    i2c_master_init(I2C_MASTER_NUM1, I2C_MASTER_SDA_IO1, I2C_MASTER_SCL_IO1);

    //================PROGRAM=================================
    ESP_LOGI(TAG, "STARTING PROGRAM SECTION");

    gpio_set_level(DIR_IO0, 0);
    gpio_set_level(DIR_IO1, 1);
    
    float delta_angs[2];
    float curr_angs[2] = {90, 0};
    float xyz[3] = {0, 200, 0};
    float goal_angs[2] = {0, 0};


    calculate_invkin(&xyz[0], &goal_angs[0], &curr_angs[0], &delta_angs[0]);

    move_arm_by_ang(delta_angs, gptimer0, gptimer1);

    uint16_t angle = 0;

    while(1) {
        angle = as_read_angle(I2C_MASTER_NUM0, 0x36);
        ESP_LOGI(TAG, "ANGLE %i", angle);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }

}