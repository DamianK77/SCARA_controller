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
#include "AX_servo.h"

//to move negative angle, dir should be 1
#define STEP_IO0         GPIO_NUM_5
#define DIR_IO0          GPIO_NUM_17

#define STEP_IO1         GPIO_NUM_16
#define DIR_IO1          GPIO_NUM_4

#define STEP_IO2         GPIO_NUM_23
#define DIR_IO2          GPIO_NUM_18

#define MIDANGLE_B       1908
#define MIDANGLE_OPPOSITE_B MIDANGLE_B+2048    //MIDANGLE_B +- 2048 (must be in range 0-4096)

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
#define START_SPEED 10 //in degrees per second

#define MAX_SPEED_0 30 //in degrees per second
#define MAX_SPEED_1 30 //in degrees per second
#define MAX_SPEED_2 10 //in mm per second
#define MAX_DEGS_val 1023 //in degrees per second

#define LMOVE_RES 0.2 //in mm
#define ACCEL_PERCENTAGE 0.1 //percentage of movement that will be spent accelerating and decelerating

//static const int RX_BUF_SIZE = 2048;

#define TXD_PIN1 (GPIO_NUM_27)
#define RXD_PIN1 (GPIO_NUM_26)

#define MOT0_ENDSTOP_PIN    GPIO_NUM_12
#define MOT2_ENDSTOP_PIN    GPIO_NUM_14

static const char *TAG = "SCARA";

//=============GLOBALS==================

bool mot0_moving = 0;
bool mot1_moving = 0;
bool mot2_moving = 0;
bool homed = 0;
bool busy = 0;
bool accel_enabled = false;
volatile uint8_t step0_state = 0;
volatile uint8_t step1_state = 0;
volatile uint8_t step2_state = 0;

volatile uint32_t step_counts[3] = {0,0,0};
volatile uint32_t target_steps[3] = {0,0,0};

float curr_angs[4] = {90, 0, 0};// 1 2 a
float goal_xyza[4] = {0, L2, 0, 0};

uint32_t alarm_step_a[3] = {1, 1, 1};

uint32_t target_alarms[3] = {0, 0, 0};
uint32_t curr_alarms[3] = {0, 0, 0};
uint32_t endaccel_steps[3] = {0, 0, 0};

float curr_z = 0;
gptimer_handle_t gptimer0 = NULL;
gptimer_handle_t gptimer1 = NULL;
gptimer_handle_t gptimer2 = NULL;

AX_conf_t AX_conf = {
    .uart = UART_NUM_1,
    .tx_pin = TXD_PIN1,
    .rx_pin = RXD_PIN1,
    .baudrate = 57600,
};
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

    if (accel_enabled) {
        
        if (step_counts[0] > target_steps[0]/2 && step_counts[0] > target_steps[0] - endaccel_steps[0]) {
            curr_alarms[0] = curr_alarms[0] - alarm_step_a[0];
        }

        if (step_counts[0] < target_steps[0]/2 && step_counts[0] < endaccel_steps[0]) {
            curr_alarms[0] = curr_alarms[0] + alarm_step_a[0];
        }

        gptimer_alarm_config_t alarm0_config = {
            .alarm_count = curr_alarms[0],
            .flags.auto_reload_on_alarm = true,
            .reload_count = 0,
        };
        gptimer_set_alarm_action(timer, &alarm0_config);
    }

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

    if (accel_enabled) {
    
        if (step_counts[1] > target_steps[1]/2 && step_counts[1] > target_steps[1] - endaccel_steps[1]) {
            curr_alarms[1] = curr_alarms[1] - alarm_step_a[1];
        }

        if (step_counts[1] < target_steps[1]/2 && step_counts[1] < endaccel_steps[1]) {
            curr_alarms[1] = curr_alarms[1] + alarm_step_a[1];
        }

        gptimer_alarm_config_t alarm1_config = {
            .alarm_count = curr_alarms[1],
            .flags.auto_reload_on_alarm = true,
            .reload_count = 0,
        };
        gptimer_set_alarm_action(timer, &alarm1_config);
    }

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

//ax12a 300deg/1025 = 0.293deg/1
uint16_t AXdeg2val(float deg) {
    return (uint16_t)(512+(deg/0.293));
}

//1 = 0,098rpm, 1rpm = 6deg/s so 1 = 0.588deg/s
uint16_t AXdegpersec2val(float degs) {
    uint16_t val = (uint16_t)degs/0.588;
    if (val > MAX_DEGS_val) {
        val = MAX_DEGS_val;
    }
    return val;
}

void move_arm_by_ang(const float (delta_angs)[3], float delta_z, float ang_a, float speed, gptimer_handle_t gptimer0, gptimer_handle_t gptimer1, gptimer_handle_t gptimer2) 
{
    float delta_angs_compensated[3] = {delta_angs[0], delta_angs[1] + delta_angs[0], 0};
    float goal_speeds[4] = {speed, speed, speed, speed};
    
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

    //calculate times for each motor assuming max speed
    float times[4] = {0, 0, 0, 0};
    times[0] = (step_counts[0]/goal_speeds[0])/STEPS_PER_DEG_01;
    times[1] = (step_counts[1]/goal_speeds[1])/STEPS_PER_DEG_01;
    times[2] = (step_counts[2]/goal_speeds[2])/STEPS_PER_MM_Z;
    times[3] = fabs(ang_a - curr_angs[2])/goal_speeds[3];

    //log times[3], ang_a, curr_angs[3] and goal_speeds[3]
    //ESP_LOGI("MOVE", "time: %f, ang_a: %f, curr_angs[3]: %f, goal_speeds[3]: %f", times[3], ang_a, curr_angs[3], goal_speeds[3]);

    float max_time = 0;
    for (int i = 0; i < 4; i++) {
        if (times[i] > max_time) {
            max_time = times[i];
        }
    }

    //calculate speeds for each motor based on joint time-based interpolation
    goal_speeds[0] = speed*times[0]/max_time;
    goal_speeds[1] = speed*times[1]/max_time;
    goal_speeds[2] = speed*times[2]/max_time;
    goal_speeds[3] = speed*times[3]/max_time;

    if (accel_enabled) {
        target_steps[0] = step_counts[0];
        target_steps[1] = step_counts[1];
        target_steps[2] = step_counts[2];
        //calculate max alarms for timers
        target_alarms[0] = speed2alarm(goal_speeds[0], 0);
        target_alarms[1] = speed2alarm(goal_speeds[1], 1);
        target_alarms[2] = speed2alarm(goal_speeds[2], 2);

        //calculate starting alarms for timers
        curr_alarms[0] = speed2alarm(START_SPEED, 0);
        curr_alarms[1] = speed2alarm(START_SPEED, 1);
        curr_alarms[2] = speed2alarm(START_SPEED, 2);

        //amount of steps to accelerate over
        endaccel_steps[0] = (uint32_t)(target_steps[0]*ACCEL_PERCENTAGE);
        endaccel_steps[1] = (uint32_t)(target_steps[1]*ACCEL_PERCENTAGE);
        endaccel_steps[2] = (uint32_t)(target_steps[2]*ACCEL_PERCENTAGE);

        // ESP_LOGI("MOVE", "1 endaccel_steps: %i %i %i", (int)endaccel_steps[0], (int)endaccel_steps[1], (int)endaccel_steps[2]);
        //calculate alarm difference for each step
        if (endaccel_steps[0] != 0) {
        alarm_step_a[0] = (uint32_t)((curr_alarms[0] - target_alarms[0])/endaccel_steps[0]);
        }
        if (endaccel_steps[1] != 0) {
        alarm_step_a[1] = (uint32_t)((curr_alarms[1] - target_alarms[1])/endaccel_steps[1]);
        }
        if (endaccel_steps[2] != 0) {
        alarm_step_a[2] = (uint32_t)((curr_alarms[2] - target_alarms[2])/endaccel_steps[2]);
        }
        //protect against no acceleration if operation gets rounded down
        if (alarm_step_a[0] == 0) {
            alarm_step_a[0] = 1;
        }
        if (alarm_step_a[1] == 0) {
            alarm_step_a[1] = 1;
        }
        if (alarm_step_a[2] == 0) {
            alarm_step_a[2] = 1;
        }

        // ESP_LOGI("MOVE", "2 alarm_step_a: %i %i %i", (int)alarm_step_a[0], (int)alarm_step_a[1], (int)alarm_step_a[2]);

        //config speeds
        gptimer_alarm_config_t alarm0_config = {
            //.alarm_count = speed2alarm(START_SPEED, 0), 
            .alarm_count = curr_alarms[0],
            .flags.auto_reload_on_alarm = true,
            .reload_count = 0,
        };
        gptimer_set_alarm_action(gptimer0, &alarm0_config);

        gptimer_alarm_config_t alarm1_config = {
            //.alarm_count = speed2alarm(START_SPEED, 1),
            .alarm_count = curr_alarms[1],
            .flags.auto_reload_on_alarm = true,
            .reload_count = 0,
        };
        gptimer_set_alarm_action(gptimer1, &alarm1_config);

        gptimer_alarm_config_t alarm2_config = {
            //.alarm_count = speed2alarm(START_SPEED, 2),
            .alarm_count = curr_alarms[2],
            .flags.auto_reload_on_alarm = true,
            .reload_count = 0,
        };
        gptimer_set_alarm_action(gptimer2, &alarm2_config);

        // ESP_LOGI("MOVE", "3 current_alarms: %i %i %i", (int)curr_alarms[0], (int)curr_alarms[1], (int)curr_alarms[2]);


    } else {
    //config speeds
    gptimer_alarm_config_t alarm0_config = {
        //.alarm_count = speed2alarm(START_SPEED, 0), 
        .alarm_count = speed2alarm(goal_speeds[0], 0),
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer0, &alarm0_config);

    gptimer_alarm_config_t alarm1_config = {
        //.alarm_count = speed2alarm(START_SPEED, 1),
        .alarm_count = speed2alarm(goal_speeds[1], 1),
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer1, &alarm1_config);

    gptimer_alarm_config_t alarm2_config = {
        //.alarm_count = speed2alarm(START_SPEED, 2),
        .alarm_count = speed2alarm(goal_speeds[2], 2),
        .flags.auto_reload_on_alarm = true,
        .reload_count = 0,
    };
    gptimer_set_alarm_action(gptimer2, &alarm2_config);
    }

    //calculate speed for dynamixel
    uint16_t a_speed = AXdegpersec2val(goal_speeds[3]);
    
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


    AX_servo_set_pos_w_spd(AX_conf, 2, AXdeg2val(ang_a), a_speed);

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
    float speed = 10;
    

    //home motor A
    
    AX_servo_set_pos(AX_conf, 2, AXdeg2val(0));
    
    while (AX_servo_is_moving(AX_conf, 2)) {
        vTaskDelay(1/portTICK_PERIOD_MS);
    }

    //home motor z to 0 position
    delta_angs[0] = 0.00;
    delta_angs[1] = 0.00;
    delta_z = -0.5;
    while (!gpio_get_level(MOT2_ENDSTOP_PIN)) {
        move_arm_by_ang(delta_angs, delta_z, 0, speed, gptimer0, gptimer1, gptimer2);
        while (mot2_moving) {
            vTaskDelay(1/portTICK_PERIOD_MS);
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
        move_arm_by_ang(delta_angs, delta_z, 0, speed, gptimer0, gptimer1, gptimer2);
        while (mot1_moving) {
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        //ESP_LOGI("HOMING", "IN LOOP HOMING MOT1, ang: %i", angB);
    } 

    //TODO then home motor 0 to 0 degree angle
    delta_angs[0] = -0.08;
    delta_angs[1] = 0.08;
    while (!gpio_get_level(MOT0_ENDSTOP_PIN)) {
        move_arm_by_ang(delta_angs, delta_z, 0, speed, gptimer0, gptimer1, gptimer2);
        while (mot0_moving) {
            vTaskDelay(1/portTICK_PERIOD_MS);
        }
        //ESP_LOGI("HOMING", "IN LOOP HOMING MOT0");
    }

    curr_angs[0] = 0.0;
    curr_angs[1] = 90.0;
    curr_angs[2] = 0.0;
    curr_z = 0.0;
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
            accel_enabled = false;
            homing(gptimer0, gptimer1, gptimer2);
            curr_angs[0] = 0.0;
            curr_angs[1] = 90.0;
            busy = 0;
        }

        if(!strcmp(chr, "G1")) {
            busy = 1;
            accel_enabled = true;
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
            
            goal_xyza[0] = x;
            goal_xyza[1] = y;
            goal_xyza[2] = z;
            goal_xyza[3] = 0;

            calculate_invkin(&goal_xyza[0], &goal_angs[0], &curr_angs[0], &delta_angs[0], &curr_z, &delta_z ,&error);

            if (error == 0 && homed == 1 && speed > 0 && speed < 201) {
                move_arm_by_ang(delta_angs, delta_z, goal_angs[2], speed, gptimer0, gptimer1, gptimer2);

                while (mot0_moving != 0 || mot1_moving != 0 || mot2_moving != 0) {
                    vTaskDelay(1/portTICK_PERIOD_MS);
                    //ESP_LOGI("BLOCK LOOP", "MOT0mov: %i MOT1mov: %i, mot2mov: %i", mot0_moving, mot1_moving, mot2_moving);
                }

                curr_angs[0] = goal_angs[0];
                curr_angs[1] = goal_angs[1];
                curr_angs[2] = goal_angs[2];
                curr_z = goal_xyza[2];

            } 
        busy = 0;
        //uart_flush_input(UART_NUM_0);
        }

        if (!strcmp(chr, "G2")) { // LMOVE
            busy = 1;
            accel_enabled = false;
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

            //calculate path
            float path[4] = {0, 0, 0, 0};
            path[0] = x - goal_xyza[0];
            path[1] = y - goal_xyza[1];
            path[2] = z - goal_xyza[2];
            path[3] = 0 - goal_xyza[3];

            //ESP_LOGI("G2", "path: %f %f %f", path[0], path[1], path[2]);

            //calculate path length
            float path_length = sqrt(pow(path[0], 2) + pow(path[1], 2) + pow(path[2], 2));
            //calculate number of micromoves
            int num_moves = (int)(path_length/LMOVE_RES);
            //ESP_LOGI("G2", "num_moves: %i", num_moves);
            //calculate micromove length
            float micromove_length_xyza[4] = {0, 0, 0, 0};
            micromove_length_xyza[0] = path[0]/num_moves;
            micromove_length_xyza[1] = path[1]/num_moves;
            micromove_length_xyza[2] = path[2]/num_moves;
            micromove_length_xyza[3] = path[3]/num_moves;
            //ESP_LOGI("G2", "micromove_length_xyz: %f %f %f %f", micromove_length_xyza[0], micromove_length_xyza[1], micromove_length_xyza[2], micromove_length_xyza[3]);
            //move micromoves
            for (int i=0; i < num_moves; i++) {
                goal_xyza[0] = goal_xyza[0] + micromove_length_xyza[0];
                goal_xyza[1] = goal_xyza[1] + micromove_length_xyza[1];
                goal_xyza[2] = goal_xyza[2] + micromove_length_xyza[2];
                goal_xyza[3] = goal_xyza[3] + micromove_length_xyza[3];

                calculate_invkin(&goal_xyza[0], &goal_angs[0], &curr_angs[0], &delta_angs[0], &curr_z, &delta_z ,&error);

                if (error == 0 && homed == 1 && speed > 0 && speed < 201) {
                    move_arm_by_ang(delta_angs, delta_z, goal_angs[2], speed, gptimer0, gptimer1, gptimer2);

                    while (mot0_moving != 0 || mot1_moving != 0 || mot2_moving != 0) {
                        vTaskDelay(1/portTICK_PERIOD_MS);
                        //ESP_LOGI("BLOCK LOOP", "MOT0mov: %i MOT1mov: %i, mot2mov: %i", mot0_moving, mot1_moving, mot2_moving);
                    }

                    curr_angs[0] = goal_angs[0];
                    curr_angs[1] = goal_angs[1];
                    curr_angs[2] = goal_angs[2];
                    curr_z = goal_xyza[2];

                } 
            }
            busy = 0;
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

    //---------------CONFIG AX12---------------------

    AX_servo_init(AX_conf);

    //================PROGRAM=================================
    //(TAG, "STARTING PROGRAM SECTION");

    //xTaskCreate(rx_task1, "uart_rx_task_1", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(movement_task, "movement_task", 4096*3, NULL, configMAX_PRIORITIES, NULL);
    //servo42c_disable();

    while(1) {
        //uint16_t ang1 = as_read_angle(I2C_MASTER_NUM0, 0x36); //0-4096, middle is 3100, opposite of middle is 1052
        //ESP_LOGI("AS5600", "ANG1: %i", ang1);
        //printf("%i %f %f %f %f %f %f %i\n", homed, goal_xyza[0], goal_xyza[1], goal_xyza[2], curr_angs[0], curr_angs[1], curr_angs[2], busy);

        printf(">alarm:%i >stepcount:%i >endaccel_steps:%i\n", (int)curr_alarms[1], (int)step_counts[1], (int)endaccel_steps[1]);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }


}