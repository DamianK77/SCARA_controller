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

#define MIDANGLE_B       1224
#define MIDANGLE_OPPOSITE_B 3272    //MIDANGLE_B +- 2048 (must be in range 0-4096)

#define I2C_MASTER_SDA_IO0   GPIO_NUM_13
#define I2C_MASTER_SCL_IO0   GPIO_NUM_15
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_NUM0              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C_MASTER_NUM1              1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_SDA_IO1   GPIO_NUM_18
#define I2C_MASTER_SCL_IO1   GPIO_NUM_23

#define STEPS_PER_DEG_01 106.66667  //64x microstep, 3x reduction, 1.8deg motor

#define STARTSPEED 1000

static const int RX_BUF_SIZE = 2048;

#define TXD_PIN1 (GPIO_NUM_4)
#define RXD_PIN1 (GPIO_NUM_16)

#define MOT0_ENDSTOP_PIN    GPIO_NUM_12

static const char *TAG = "SCARA";

//=============GLOBALS==================

bool mot0_moving = 0;
bool mot1_moving = 0;
bool motz_moving = 0;
volatile uint8_t step0_state = 0;
volatile uint8_t step1_state = 0;
volatile uint32_t step_counts[2];

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
void move_arm_by_ang(const float (delta_angs)[2], const float (speeds)[2], gptimer_handle_t gptimer0, gptimer_handle_t gptimer1, gptimer_handle_t gptimer2, gptimer_handle_t gptimer3) 
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

    step_counts[0] = fabs(delta_angs_compensated[0] * STEPS_PER_DEG_01);
    step_counts[1] = fabs(delta_angs_compensated[1] * STEPS_PER_DEG_01);

    mot0_moving = 1;
    mot1_moving = 1;
    
    if (step_counts[0] != 0) {
        gptimer_start(gptimer0);
    }
    if (step_counts[1] != 0) {
        gptimer_start(gptimer1);
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

void uart_init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // config uart1 and uart2 for motors 0 and 1
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN1, RXD_PIN1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

}

void homing(gptimer_handle_t gptimer0, gptimer_handle_t gptimer1, gptimer_handle_t gptimer2, gptimer_handle_t gptimer3) {
    //first, position arm B along Y axis
    uint16_t angB = as_read_angle(I2C_MASTER_NUM0, 0x36); // value * 0.08789 = angle. 1.8degree*3 = 61
    float delta_angs[2];
    float speeds[2] = {5, 5};
    while (angB > MIDANGLE_B+1 || angB < MIDANGLE_B-1) {
        angB = as_read_angle(I2C_MASTER_NUM0, 0x36);
        if (angB < MIDANGLE_OPPOSITE_B && angB > MIDANGLE_B-1) {
            delta_angs[0] = 0;
            delta_angs[1] = -0.08;
        } else {
            delta_angs[0] = 0;
            delta_angs[1] = 0.08;
        }
        move_arm_by_ang(delta_angs, speeds, gptimer0, gptimer1, gptimer2, gptimer3);
        while (mot1_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        ESP_LOGI("HOMING", "IN LOOP HOMING MOT1, ang: %i", angB);
    } 

    //TODO then home motor 0 to 0 degree angle
    delta_angs[0] = -0.08;
    delta_angs[1] = 0.08;
    while (!gpio_get_level(MOT0_ENDSTOP_PIN)) {
        move_arm_by_ang(delta_angs, speeds, gptimer0, gptimer1, gptimer2, gptimer3);
        while (mot0_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        ESP_LOGI("HOMING", "IN LOOP HOMING MOT0");
    }

}

//===================TASKS====================
static void movement_task(void *arg)
{
    float delta_angs[2] = {0, 0};
    float curr_angs[2] = {90, 0};
    float goal_xyz[3] = {0, 200, 0};
    float goal_angs[2] = {0, 0};
    float speeds[2] = {10, 10};

    //=========================================
    gptimer_handle_t gptimer0 = NULL;
    gptimer_handle_t gptimer1 = NULL;
    gptimer_handle_t gptimer2 = NULL;
    gptimer_handle_t gptimer3 = NULL;

    gptimer_config_t timer0_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 10000000, // 10MHz, 1 tick=0.1us
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

    uint8_t z_dir = 0;

    //calculate_invkin(&goal_xyz[0], &goal_angs[0], &curr_angs[0], &delta_angs[0]);

    //move_arm_by_ang(delta_angs, speeds, gptimer0, gptimer1, gptimer2, gptimer3);

    vTaskDelay(100/portTICK_PERIOD_MS);

    // while (1) {
    //     while (mot0_moving != 0 || mot1_moving != 0 || motz_moving != 0) {
    //         vTaskDelay(400/portTICK_PERIOD_MS);
    //         ESP_LOGI("BLOCK LOOP", "MOT0mov: %i MOT1mov: %i", mot0_moving, mot1_moving);
    //     }

    //     ESP_LOGI("BLOCK LOOP", "WAIT END");
    //     delta_angs[0] = -1.0*delta_angs[0];
    //     delta_angs[1] = -1.0*delta_angs[1];
    //     move_arm_by_ang(delta_angs, speeds, gptimer0, gptimer1, gptimer2, gptimer3);
    // }

    vTaskDelay(100/portTICK_PERIOD_MS);
    char chr[10];
    int x = 0;
    int y = 0;
    int z = 0;

    //GCODE G28 - home,   G1 Xxxx Yxxx Zxxx
    while(1){
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

            calculate_invkin(&goal_xyz[0], &goal_angs[0], &curr_angs[0], &delta_angs[0]);

            move_arm_by_ang(delta_angs, speeds, gptimer0, gptimer1, gptimer2, gptimer3);

            curr_angs[0] = goal_angs[0];
            curr_angs[1] = goal_angs[1];
            curr_angs[2] = goal_angs[2];

        }

        strcpy(chr, "");
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
    gpio_set_direction(MOT0_ENDSTOP_PIN, GPIO_MODE_INPUT);

    //---------------CONFIG I2C----------------------------------
    
    i2c_master_init(I2C_MASTER_NUM0, I2C_MASTER_SDA_IO0, I2C_MASTER_SCL_IO0);
    i2c_master_init(I2C_MASTER_NUM1, I2C_MASTER_SDA_IO1, I2C_MASTER_SCL_IO1);

    //---------------CONFIG UART---------------------

    //uart_init();

    //================PROGRAM=================================
    ESP_LOGI(TAG, "STARTING PROGRAM SECTION");

    //xTaskCreate(rx_task1, "uart_rx_task_1", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(movement_task, "movement_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //servo42c_disable();

    while(1) {
        //uint16_t ang1 = as_read_angle(I2C_MASTER_NUM0, 0x36); //0-4096, middle is 3100, opposite of middle is 1052
        //ESP_LOGI("AS5600", "ANG1: %i", ang1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }


}