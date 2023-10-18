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
#include "driver/uart.h"
#include "math.h"
#include "string.h"

//to move negative angle, dir should be 1
#define STEP_IO0         GPIO_NUM_5
#define DIR_IO0          GPIO_NUM_17

#define STEP_IO1         GPIO_NUM_16
#define DIR_IO1          GPIO_NUM_4

#define STEP_IO2         GPIO_NUM_23
#define DIR_IO2          GPIO_NUM_18

#define MIDANGLE_B       1892
#define MIDANGLE_OPPOSITE_B 3940    //MIDANGLE_B +- 2048 (must be in range 0-4096)

#define I2C_MASTER_SDA_IO0   GPIO_NUM_13
#define I2C_MASTER_SCL_IO0   GPIO_NUM_15
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_NUM0              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

// #define I2C_MASTER_NUM1              1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
// #define I2C_MASTER_SDA_IO1   GPIO_NUM_18
// #define I2C_MASTER_SCL_IO1   GPIO_NUM_23

#define STEPS_PER_DEG_01 106.66667  //64x microstep, 3x reduction, 1.8deg motor
#define STEPS_PER_MM_Z 203.68 //steps per mm for 0,9deg motor. 10mm pulley length = 31.42mm/rotation, 400steps/rotation gives 12.73 steps/mm, 203.68 for 16x microstep 

#define STEPCLOCK_FREQ 10000000
#define STARTSPEED 1000

//static const int RX_BUF_SIZE = 2048;

//#define TXD_PIN1 (GPIO_NUM_4)
//#define RXD_PIN1 (GPIO_NUM_16)

#define MOT0_ENDSTOP_PIN    GPIO_NUM_12
#define MOT2_ENDSTOP_PIN    GPIO_NUM_14

static const char *TAG = "SCARA";

//=============GLOBALS==================

bool mot0_moving = 0;
bool mot1_moving = 0;
bool motz_moving = 0;
bool homed = 0;
volatile uint8_t step0_state = 0;
volatile uint8_t step1_state = 0;
volatile uint8_t step2_state = 0;
volatile uint32_t step_counts[3] = {0,0,0};
gptimer_handle_t gptimer0 = NULL;
gptimer_handle_t gptimer1 = NULL;
gptimer_handle_t gptimer2 = NULL;
gptimer_handle_t gptimer3 = NULL;
//===============CALLBACKS============

static bool IRAM_ATTR timer0_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    if (step0_state) {
        step_counts[0] = step_counts[0] - 1;
    }

    if (step_counts[0] <= 0) {
        gptimer_stop(timer);
        mot0_moving = 0;
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
        mot1_moving = 0;
    }

    step1_state = !step1_state;
    gpio_set_level(STEP_IO1, step1_state);

    return (high_task_awoken == pdTRUE);
}

static bool IRAM_ATTR timer2_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    if (step2_state) {
        step_counts[2] = step_counts[2] - 1;
    }

    if (step_counts[2] <= 0) {
        gptimer_stop(timer);
        motz_moving = 0;
    }
    step2_state = !step2_state;
    gpio_set_level(STEP_IO2, step2_state);

    return (high_task_awoken == pdTRUE);
}

static bool IRAM_ATTR timer3_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    // if (step0_state) {
    //     step0_count = step0_count - 1;
    // }

    // if (step0_count <= 0) {
    //     gptimer_stop(timer);
    //      motz_moving = 0;
    // }
    // step0_state = !step0_state;
    // gpio_set_level(STEP_IO0, step0_state);

    return (high_task_awoken == pdTRUE);
}

//============FUNCTIONS=================================
void move_arm_by_ang(const float (delta_angs)[2], float delta_z, const float (speeds)[3], gptimer_handle_t gptimer0, gptimer_handle_t gptimer1, gptimer_handle_t gptimer2, gptimer_handle_t gptimer3) 
{
    float delta_angs_compensated[2] = {delta_angs[0], delta_angs[1] + delta_angs[0]};
    
    ESP_LOGI("MOVE", "comp delta angs %f %f", delta_angs_compensated[0], delta_angs_compensated[1]);

    if (delta_angs_compensated[0] < 0)
    {
        gpio_set_level(DIR_IO0, 1);
    } else {
        gpio_set_level(DIR_IO0, 0);
    }

    if (delta_angs_compensated[1] < 0)
    {
        gpio_set_level(DIR_IO1, 1);
    } else {
        gpio_set_level(DIR_IO1, 0);
    }

    if (delta_z < 0)
    {
        gpio_set_level(DIR_IO2, 1);
    } else {
        gpio_set_level(DIR_IO2, 0);
    }

    ESP_LOGI("DEBUG", "configuring alarm0");
    gptimer_alarm_config_t alarm0_config = {
        .alarm_count = (int)(1.0/(2.0*STEPS_PER_DEG_01*speeds[0]*(1.0/STEPCLOCK_FREQ))), 
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer0, &alarm0_config);
    ESP_LOGI("DEBUG", "configured alarm0");

    gptimer_alarm_config_t alarm1_config = {
        .alarm_count = (int)(1.0/(2.0*STEPS_PER_DEG_01*speeds[1]*(1.0/STEPCLOCK_FREQ))),
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer1, &alarm1_config);
    ESP_LOGI("DEBUG", "configured alarm1");

    step_counts[0] = fabs(delta_angs_compensated[0] * STEPS_PER_DEG_01);
    step_counts[1] = fabs(delta_angs_compensated[1] * STEPS_PER_DEG_01);
    step_counts[2] = fabs(delta_z*STEPS_PER_MM_Z);
    
    if (step_counts[0] != 0) {
        mot0_moving = 1;
        gptimer_start(gptimer0);
    }
    if (step_counts[1] != 0) {
        mot1_moving = 1;
        gptimer_start(gptimer1);
    }
    if (step_counts[2] != 0) {
       motz_moving = 1;
       gptimer_start(gptimer2);
    }

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

void homing(gptimer_handle_t gptimer0, gptimer_handle_t gptimer1, gptimer_handle_t gptimer2, gptimer_handle_t gptimer3) {
    uint16_t angB = as_read_angle(I2C_MASTER_NUM0, 0x36); // value * 0.08789 = angle. 1.8degree*3 = 61
    float delta_angs[2];
    float delta_z = 0;
    float speeds[3] = {5, 5, 5};

    //home motor z to 0 position
    delta_angs[0] = 0.00;
    delta_angs[1] = 0.00;
    delta_z = -0.5;
    while (!gpio_get_level(MOT2_ENDSTOP_PIN)) {
        move_arm_by_ang(delta_angs, delta_z, speeds, gptimer0, gptimer1, gptimer2, gptimer3);
        while (motz_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        ESP_LOGI("HOMING", "IN LOOP HOMING MOT2");
    }

    delta_z = 0;
    //then home motor 1 position arm B along Y axis
    while (angB > MIDANGLE_B+1 || angB < MIDANGLE_B-1) {
        angB = as_read_angle(I2C_MASTER_NUM0, 0x36);
        if (angB < MIDANGLE_OPPOSITE_B && angB > MIDANGLE_B-1) {
            delta_angs[0] = 0;
            delta_angs[1] = -0.08;
        } else {
            delta_angs[0] = 0;
            delta_angs[1] = 0.08;
        }
        move_arm_by_ang(delta_angs, delta_z, speeds, gptimer0, gptimer1, gptimer2, gptimer3);
        while (mot1_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        ESP_LOGI("HOMING", "IN LOOP HOMING MOT1, ang: %i", angB);
    } 

    //TODO then home motor 0 to 0 degree angle
    delta_angs[0] = -0.08;
    delta_angs[1] = 0.08;
    while (!gpio_get_level(MOT0_ENDSTOP_PIN)) {
        move_arm_by_ang(delta_angs, delta_z, speeds, gptimer0, gptimer1, gptimer2, gptimer3);
        while (mot0_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        ESP_LOGI("HOMING", "IN LOOP HOMING MOT0");
    }

    homed = 1;
}

//===================TASKS====================
static void movement_task(void *arg)
{
    float delta_angs[2] = {0, 0};
    float curr_angs[2] = {90, 0};
    float delta_z = 0;
    float curr_z = 0;
    float goal_xyz[3] = {0, 200, 0};
    float goal_angs[2] = {0, 0};
    float speeds[3] = {10, 10, 10};

    //=========================================

    gptimer_config_t timer0_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = STEPCLOCK_FREQ, // 10MHz, 1 tick=0.1us
    };

    gptimer_new_timer(&timer0_config, &gptimer0);
    gptimer_new_timer(&timer0_config, &gptimer1);
    gptimer_new_timer(&timer0_config, &gptimer2);
    gptimer_new_timer(&timer0_config, &gptimer3);

    gptimer_event_callbacks_t cbs0 = {
        .on_alarm = timer0_alarm_cb,
    };
    gptimer_event_callbacks_t cbs1 = {
        .on_alarm = timer1_alarm_cb,
    };
    gptimer_event_callbacks_t cbs2 = {
        .on_alarm = timer2_alarm_cb,
    };
    gptimer_event_callbacks_t cbs3 = {
        .on_alarm = timer3_alarm_cb,
    };

    gptimer_register_event_callbacks(gptimer0, &cbs0, NULL);
    gptimer_register_event_callbacks(gptimer1, &cbs1, NULL);
    gptimer_register_event_callbacks(gptimer2, &cbs2, NULL);
    gptimer_register_event_callbacks(gptimer3, &cbs3, NULL);

    gptimer_enable(gptimer0);
    gptimer_enable(gptimer1);
    gptimer_enable(gptimer2);
    gptimer_enable(gptimer3);

    gptimer_alarm_config_t alarm0_config = {
        .reload_count = 0,
        .alarm_count = STARTSPEED,
        .flags.auto_reload_on_alarm = true,
    };

    gptimer_set_alarm_action(gptimer0, &alarm0_config);
    gptimer_set_alarm_action(gptimer1, &alarm0_config);
    gptimer_set_alarm_action(gptimer2, &alarm0_config);
    gptimer_set_alarm_action(gptimer3, &alarm0_config);
    //==============================================

    vTaskDelay(5000/portTICK_PERIOD_MS);

    char chr[10];
    int x = 0;
    int y = 0;
    int z = 0;
    int error = 0;

    //GCODE G28 - home,   G1 Xxxx Yxxx Zxxx
    while(1){

        strcpy(chr, "");
        scanf("%9s", chr);
        
        if (!strcmp(chr, "G28")) {
            homing(gptimer0, gptimer1, gptimer2, gptimer3);
            curr_angs[0] = 0.0;
            curr_angs[1] = 90.0;
        }

        if(!strcmp(chr, "G1")) {
            strcpy(chr, "");
            scanf("%9s", chr);
            memmove(chr, chr+1, strlen(chr));
            x = atoi(chr);     

            strcpy(chr, "");
            scanf("%9s", chr);
            memmove(chr, chr+1, strlen(chr));
            y = atoi(chr);
            
            strcpy(chr, "");
            scanf("%9s", chr);
            memmove(chr, chr+1, strlen(chr));
            z = atoi(chr);

            printf("Read XYZ: %i %i %i", x, y, z);
            
            goal_xyz[0] = x;
            goal_xyz[1] = y;
            goal_xyz[2] = z;

            calculate_invkin(&goal_xyz[0], &goal_angs[0], &curr_angs[0], &delta_angs[0], &curr_z, &delta_z ,&error);

            if (error == 0 && homed == 1) {
                move_arm_by_ang(delta_angs, delta_z, speeds, gptimer0, gptimer1, gptimer2, gptimer3);

                while (mot0_moving != 0 || mot1_moving != 0 || motz_moving != 0) {
                    vTaskDelay(400/portTICK_PERIOD_MS);
                    ESP_LOGI("BLOCK LOOP", "MOT0mov: %i MOT1mov: %i, MOTzmov: %i", mot0_moving, mot1_moving, motz_moving);
                }

                curr_angs[0] = goal_angs[0];
                curr_angs[1] = goal_angs[1];
                curr_angs[2] = goal_angs[2];
                curr_z = goal_xyz[2];
            } 

        }

        vTaskDelay(500 / portTICK_PERIOD_MS);
    }


}

void app_main(void)
{
    //-------------CONFIG GPIO--------------------------------
    gpio_set_direction(DIR_IO0, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_IO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_IO0, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_IO1, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_IO2, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_IO2, GPIO_MODE_OUTPUT);
    gpio_set_direction(MOT0_ENDSTOP_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(MOT2_ENDSTOP_PIN, GPIO_MODE_INPUT);

    //---------------CONFIG I2C----------------------------------
    
    i2c_master_init(I2C_MASTER_NUM0, I2C_MASTER_SDA_IO0, I2C_MASTER_SCL_IO0);
    //i2c_master_init(I2C_MASTER_NUM1, I2C_MASTER_SDA_IO1, I2C_MASTER_SCL_IO1);

    //---------------CONFIG UART---------------------

    //uart_init();

    //================PROGRAM=================================
    ESP_LOGI(TAG, "STARTING PROGRAM SECTION");

    //xTaskCreate(rx_task1, "uart_rx_task_1", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(movement_task, "movement_task", 2048*2, NULL, configMAX_PRIORITIES, NULL);
    //servo42c_disable();

    while(1) {
        uint16_t ang1 = as_read_angle(I2C_MASTER_NUM0, 0x36); //0-4096, middle is 3100, opposite of middle is 1052
        ESP_LOGI("AS5600", "ANG1: %i", ang1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }


}