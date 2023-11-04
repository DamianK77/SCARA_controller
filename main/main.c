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

#define MIDANGLE_B       1946
#define MIDANGLE_OPPOSITE_B 3994    //MIDANGLE_B +- 2048 (must be in range 0-4096)

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
#define STARTALARM 1000
#define START_SPEED 1 //in degrees per second

#define MAX_SPEED_0 30 //in degrees per second
#define MAX_SPEED_1 30 //in degrees per second
#define MAX_SPEED_2 10 //in mm per second

//static const int RX_BUF_SIZE = 2048;

#define TXD_PIN1 (GPIO_NUM_4)
#define RXD_PIN1 (GPIO_NUM_16)

#define MOT0_ENDSTOP_PIN    GPIO_NUM_12
#define MOT2_ENDSTOP_PIN    GPIO_NUM_14

static const char *TAG = "SCARA";

//=============GLOBALS==================

bool mot0_moving = 0;
bool mot1_moving = 0;
bool mot2_moving = 0;
bool homed = 0;
bool busy = 0;
volatile uint8_t step0_state = 0;
volatile uint8_t step1_state = 0;
volatile uint8_t step2_state = 0;

volatile uint32_t step_counts[3] = {0,0,0};
volatile uint32_t target_steps[3] = {0,0,0};

float curr_angs[3] = {90, 0, 0};
float goal_xyz[3] = {0, 200, 0};

float accels[3] = {100, 100, 100};//TODO: calculate alarm_accels from accels
uint32_t alarm_accels[3] = {500, 500, 500};

uint32_t target_alarms[3] = {0, 0, 0};
uint32_t curr_alarms[3] = {0, 0, 0};
uint32_t endaccel_steps[3] = {0, 0, 0};

float curr_z = 0;
gptimer_handle_t gptimer0 = NULL;
gptimer_handle_t gptimer1 = NULL;
gptimer_handle_t gptimer2 = NULL;

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
        target_steps[0] = 0;
    }
    step0_state = !step0_state;
    gpio_set_level(STEP_IO0, step0_state);

    if (step_counts[0] > target_steps[0]/2) { // accelerating
        if (curr_alarms[0] > target_alarms[0] && curr_alarms[0] + target_alarms[0] > alarm_accels[0]) { // acceleration phase
            curr_alarms[0] = curr_alarms[0] - alarm_accels[0];
        } else if (curr_alarms[0] + target_alarms[0] <= alarm_accels[0]) {
            curr_alarms[0] = target_alarms[0];
        } else if (!endaccel_steps[0]) { // end of acceleration phase, save steps it took to accelerate
            endaccel_steps[0] = target_steps[0] - step_counts[0];
        }
    } else { //decelerating
        if (step_counts[0] < endaccel_steps[0] || endaccel_steps[0] == 0) { //deceleration phase
            curr_alarms[0] = curr_alarms[0] + alarm_accels[0];
        } 
    }

        gptimer_alarm_config_t alarm0_config = {
            .alarm_count = curr_alarms[0],
            .flags.auto_reload_on_alarm = true,
            .reload_count = 0,
        };
        gptimer_set_alarm_action(timer, &alarm0_config);

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
        target_steps[1] = 0;
    }
    
    step1_state = !step1_state;
    gpio_set_level(STEP_IO1, step1_state);

    if (step_counts[1] > target_steps[1]/2) { // accelerating
        if (curr_alarms[1] > target_alarms[1] && curr_alarms[1] + target_alarms[1] > alarm_accels[1]) { // acceleration phase
            curr_alarms[1] = curr_alarms[1] - alarm_accels[1];
        } else if (curr_alarms[1] + target_alarms[1] <= alarm_accels[1]) {
            curr_alarms[1] = target_alarms[1];
        } else if (!endaccel_steps[1]) { // end of acceleration phase, save steps it took to accelerate
            endaccel_steps[1] = target_steps[1] - step_counts[1];
        }
    } else { //decelerating
        if (step_counts[1] < endaccel_steps[1] || endaccel_steps[1] == 0) { //deceleration phase
            curr_alarms[1] = curr_alarms[1] + alarm_accels[1];
        } 
    }

        gptimer_alarm_config_t alarm1_config = {
            .alarm_count = curr_alarms[1],
            .flags.auto_reload_on_alarm = true,
            .reload_count = 0,
        };
        gptimer_set_alarm_action(timer, &alarm1_config);

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
        mot2_moving = 0;
        target_steps[2] = 0;
    }
    step2_state = !step2_state;
    gpio_set_level(STEP_IO2, step2_state);

    return (high_task_awoken == pdTRUE);
}

//============FUNCTIONS=================================
int speed2alarm(float speed, int axis) 
{
    if (axis == 0) {
        return (int)(1.0/(2.0*STEPS_PER_DEG_01*speed*(1.0/STEPCLOCK_FREQ)));//*2.0 because 2 cycles per step
    } else if (axis == 1) {
        return (int)(1.0/(2.0*STEPS_PER_DEG_01*speed*(1.0/STEPCLOCK_FREQ)));
    } else if (axis == 2) {
        return (int)(1.0/(2.0*STEPS_PER_MM_Z*speed*(1.0/STEPCLOCK_FREQ)));
    } else {
        return 0;
    }
}

void move_arm_by_ang(const float (delta_angs)[3], float delta_z, float speed, gptimer_handle_t gptimer0, gptimer_handle_t gptimer1, gptimer_handle_t gptimer2) 
{
    float delta_angs_compensated[3] = {delta_angs[0], delta_angs[1] + delta_angs[0], 0};
    float goal_speeds[3] = {speed, speed, speed};
    
    //ESP_LOGI("MOVE", "comp delta angs %f %f", delta_angs_compensated[0], delta_angs_compensated[1]);

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

    step_counts[0] = fabs(delta_angs_compensated[0] * STEPS_PER_DEG_01);
    step_counts[1] = fabs(delta_angs_compensated[1] * STEPS_PER_DEG_01);
    step_counts[2] = fabs(delta_z*STEPS_PER_MM_Z);
    target_steps[0] = step_counts[0];
    target_steps[1] = step_counts[1];
    target_steps[2] = step_counts[2];

    //calculate times for each motor assuming max speed
    float times[3] = {0, 0, 0};
    times[0] = (step_counts[0]/goal_speeds[0])/STEPS_PER_DEG_01;
    times[1] = (step_counts[1]/goal_speeds[1])/STEPS_PER_DEG_01;
    times[2] = (step_counts[2]/goal_speeds[2])/STEPS_PER_MM_Z;

    float max_time = 0;
    for (int i = 0; i < 3; i++) {
        if (times[i] > max_time) {
            max_time = times[i];
        }
    }

    //calculate speeds for each motor based on joint time-based interpolation
    goal_speeds[0] = speed*times[0]/max_time;
    goal_speeds[1] = speed*times[1]/max_time;
    goal_speeds[2] = speed*times[2]/max_time;

    //calculate max alarms for timers
    target_alarms[0] = speed2alarm(goal_speeds[0], 0);
    target_alarms[1] = speed2alarm(goal_speeds[1], 1);
    target_alarms[2] = speed2alarm(goal_speeds[2], 2);

    //calculate current alarms for timers
    curr_alarms[0] = speed2alarm(START_SPEED, 0);
    curr_alarms[1] = speed2alarm(START_SPEED, 1);
    curr_alarms[2] = speed2alarm(START_SPEED, 2);

    //zero endaccel counters
    endaccel_steps[0] = 0;
    endaccel_steps[1] = 0;
    endaccel_steps[2] = 0;

    //config speeds
    gptimer_alarm_config_t alarm0_config = {
        .alarm_count = speed2alarm(START_SPEED, 0), 
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer0, &alarm0_config);

    gptimer_alarm_config_t alarm1_config = {
        .alarm_count = speed2alarm(START_SPEED, 1),
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer1, &alarm1_config);

    gptimer_alarm_config_t alarm2_config = {
        .alarm_count = speed2alarm(START_SPEED, 2),
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer2, &alarm2_config);

    
    if (step_counts[0] != 0) {
        mot0_moving = 1;
        gptimer_start(gptimer0);
    }
    if (step_counts[1] != 0) {
        mot1_moving = 1;
        gptimer_start(gptimer1);
    }
    if (step_counts[2] != 0) {
       mot2_moving = 1;
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

void homing(gptimer_handle_t gptimer0, gptimer_handle_t gptimer1, gptimer_handle_t gptimer2) {
    uint16_t angB = as_read_angle(I2C_MASTER_NUM0, 0x36); // value * 0.08789 = angle. 1.8degree*3 = 61
    float delta_angs[3];
    float delta_z = 0;
    float speed = 60;

    //home motor z to 0 position
    delta_angs[0] = 0.00;
    delta_angs[1] = 0.00;
    delta_z = -0.5;
    while (!gpio_get_level(MOT2_ENDSTOP_PIN)) {
        move_arm_by_ang(delta_angs, delta_z, speed, gptimer0, gptimer1, gptimer2);
        while (mot2_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        //ESP_LOGI("HOMING", "IN LOOP HOMING MOT2");
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
        move_arm_by_ang(delta_angs, delta_z, speed, gptimer0, gptimer1, gptimer2);
        while (mot1_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        //ESP_LOGI("HOMING", "IN LOOP HOMING MOT1, ang: %i", angB);
    } 

    //TODO then home motor 0 to 0 degree angle
    delta_angs[0] = -0.08;
    delta_angs[1] = 0.08;
    while (!gpio_get_level(MOT0_ENDSTOP_PIN)) {
        move_arm_by_ang(delta_angs, delta_z, speed, gptimer0, gptimer1, gptimer2);
        while (mot0_moving) {
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
        //ESP_LOGI("HOMING", "IN LOOP HOMING MOT0");
    }

    homed = 1;
}

//===================TASKS====================
static void movement_task(void *arg)
{
    float delta_angs[3] = {0, 0, 0};
    float delta_z = 0;
    float goal_angs[3] = {0, 0, 0};

    //=========================================

    gptimer_config_t timer0_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = STEPCLOCK_FREQ, // 10MHz, 1 tick=0.1us
    };

    gptimer_new_timer(&timer0_config, &gptimer0);
    gptimer_new_timer(&timer0_config, &gptimer1);
    gptimer_new_timer(&timer0_config, &gptimer2);

    gptimer_event_callbacks_t cbs0 = {
        .on_alarm = timer0_alarm_cb,
    };
    gptimer_event_callbacks_t cbs1 = {
        .on_alarm = timer1_alarm_cb,
    };
    gptimer_event_callbacks_t cbs2 = {
        .on_alarm = timer2_alarm_cb,
    };

    gptimer_register_event_callbacks(gptimer0, &cbs0, NULL);
    gptimer_register_event_callbacks(gptimer1, &cbs1, NULL);
    gptimer_register_event_callbacks(gptimer2, &cbs2, NULL);

    gptimer_enable(gptimer0);
    gptimer_enable(gptimer1);
    gptimer_enable(gptimer2);

    gptimer_alarm_config_t alarm0_config = {
        .reload_count = 0,
        .alarm_count = STARTALARM,
        .flags.auto_reload_on_alarm = true,
    };

    gptimer_set_alarm_action(gptimer0, &alarm0_config);
    gptimer_set_alarm_action(gptimer1, &alarm0_config);
    gptimer_set_alarm_action(gptimer2, &alarm0_config);
    //==============================================

    vTaskDelay(5000/portTICK_PERIOD_MS);

    char chr[10];
    int x = 0;
    int y = 0;
    int z = 0;
    int speed = 0;
    int error = 0;

    //GCODE G28 - home,   G1 Xxxx Yxxx Zxxx
    while(1){

        strcpy(chr, "");
        scanf("%9s", chr);
        
        if (!strcmp(chr, "G28")) {
            busy = 1;
            homing(gptimer0, gptimer1, gptimer2);
            curr_angs[0] = 0.0;
            curr_angs[1] = 90.0;
            busy = 0;
        }

        if(!strcmp(chr, "G1")) {
            busy = 1;
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

            strcpy(chr, "");
            scanf("%9s", chr);
            memmove(chr, chr+1, strlen(chr));
            speed = atoi(chr);

            //printf("Read XYZ speed: %i %i %i %i", x, y, z, speed);
            
            goal_xyz[0] = x;
            goal_xyz[1] = y;
            goal_xyz[2] = z;

            calculate_invkin(&goal_xyz[0], &goal_angs[0], &curr_angs[0], &delta_angs[0], &curr_z, &delta_z ,&error);

            if (error == 0 && homed == 1 && speed > 0 && speed < 201) {
                move_arm_by_ang(delta_angs, delta_z, speed, gptimer0, gptimer1, gptimer2);

                while (mot0_moving != 0 || mot1_moving != 0 || mot2_moving != 0) {
                    vTaskDelay(40/portTICK_PERIOD_MS);
                    //ESP_LOGI("BLOCK LOOP", "MOT0mov: %i MOT1mov: %i, mot2mov: %i", mot0_moving, mot1_moving, mot2_moving);
                }

                curr_angs[0] = goal_angs[0];
                curr_angs[1] = goal_angs[1];
                curr_angs[2] = goal_angs[2];
                curr_z = goal_xyz[2];

            } 
        busy = 0;
        //uart_flush_input(UART_NUM_0);
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
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
    //(TAG, "STARTING PROGRAM SECTION");

    //xTaskCreate(rx_task1, "uart_rx_task_1", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(movement_task, "movement_task", 4096*2, NULL, configMAX_PRIORITIES, NULL);
    //servo42c_disable();

    while(1) {
        //uint16_t ang1 = as_read_angle(I2C_MASTER_NUM0, 0x36); //0-4096, middle is 3100, opposite of middle is 1052
        //ESP_LOGI("AS5600", "ANG1: %i", ang1);
        printf("%i %f %f %f %f %f %f %i\n", homed, goal_xyz[0], goal_xyz[1], goal_xyz[2], curr_angs[0], curr_angs[1], curr_angs[2], busy);
        
         //printf(">alarm:%i >stepcount:%i >endaccel_steps:%i\n", (int)curr_alarms[1], (int)step_counts[1], (int)endaccel_steps[1]);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }


}