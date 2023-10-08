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

#define STEP_IO0         GPIO_NUM_27
#define DIR_IO0          GPIO_NUM_26


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

#define STARTSPEED 8000

static const int RX_BUF_SIZE = 2048;

#define TXD_PIN1 (GPIO_NUM_4)
#define RXD_PIN1 (GPIO_NUM_16)

static const char *TAG = "SCARA";

//=============GLOBALS==================

bool mot0_moving = 0;
bool mot1_moving = 0;
bool motz_moving = 0;
volatile uint8_t step0_state = 0;
volatile uint32_t step0_count = 0;

//===============CALLBACKS============

static bool IRAM_ATTR timer0_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
    
    if (step0_state) {
        step0_count = step0_count - 1;
    }

    if (step0_count <= 0) {
        gptimer_stop(timer);
         motz_moving = 0;
    }
    step0_state = !step0_state;
    gpio_set_level(STEP_IO0, step0_state);

    return (high_task_awoken == pdTRUE);
}

//============FUNCTIONS=================================
void move_arm_by_ang(const float (delta_angs)[2], const float (speeds)[2]) 
{
    float delta_angs_compensated[2] = {delta_angs[0], delta_angs[1] + delta_angs[0]};
    
    ESP_LOGI("MOVE", "comp delta angs %f %f", delta_angs_compensated[0], delta_angs_compensated[1]);

    for (int i = 0; i < 2; i++) {
        uint8_t speed = speeds[i];    //speed is 0-255 where 0-127 is 0-100% speed clockwise and 128-255 is 0-100% speed counter clockwise
        if (delta_angs_compensated[i] > 0) {
            speed = speed + 128;
        }
        uint32_t steps = (int)(fabs(delta_angs_compensated[i])*STEPS_PER_DEG_01);
        uint8_t stepbyte[4];
        stepbyte[0] = steps >> 24;
        stepbyte[1] = steps >> 16;
        stepbyte[2] = steps >>  8;
        stepbyte[3] = steps;
        uint8_t mot_id = 224+i;
        uint8_t checksum = (uint8_t)(mot_id + 0xFD + speed + stepbyte[0] + stepbyte[1] + stepbyte[2] + stepbyte[3]);
        char msg_mot[] = {mot_id,0xFD,speed,stepbyte[0],stepbyte[1],stepbyte[2],stepbyte[3],checksum};
        if (steps != 0) {
            uart_write_bytes(UART_NUM_1, (const char*) msg_mot, sizeof(msg_mot));
        }
    } 
}

void servo42c_disable() {
    for (int i = 0; i < 2; i++) {
        uint8_t mot_id = 224+i;
        uint8_t checksum = (uint8_t)(mot_id + 0xF3 + 0x00);
        char msg_mot[] = {mot_id,0xF3,0x00,checksum};
        uart_write_bytes(UART_NUM_1, (const char*) msg_mot, sizeof(msg_mot));
    }
}

void servo42c_enable() {
for (int i = 0; i < 2; i++) {
        uint8_t mot_id = 224+i;
        uint8_t checksum = (uint8_t)(mot_id + 0xF3 + 0x01);
        char msg_mot[] = {mot_id,0xF3,0x01,checksum};
        uart_write_bytes(UART_NUM_1, (const char*) msg_mot, sizeof(msg_mot));
    }
}

void servo42c_set_accel() {
for (int i = 0; i < 2; i++) {
        uint8_t mot_id = 224+i;
        uint8_t checksum = (uint8_t)(mot_id + 0xA4 + 0x00 + 0x01);
        char msg_mot[] = {mot_id,0xA4,0x00,0x01,checksum};
        uart_write_bytes(UART_NUM_1, (const char*) msg_mot, sizeof(msg_mot));
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

//===================TASKS====================

static void rx_task1(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK_1";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 10 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes", rxBytes);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
            if (data[0] == 224 && data[1] == 1 && data[2] == 225) {
                ESP_LOGI(RX_TASK_TAG, "MOT0 MOVING");
                mot0_moving = 1;
            } else if (data[0] == 224 && data[1] == 2 && data[2] == 226) {
                ESP_LOGI(RX_TASK_TAG, "MOT0 END");
                mot0_moving = 0;
            } else if (data[0] == 225 && data[1] == 1 && data[2] == 226) {
                ESP_LOGI(RX_TASK_TAG, "MOT1 MOVING");
                mot1_moving = 1;
            } else if (data[0] == 225 && data[1] == 2 && data[2] == 227) {
                ESP_LOGI(RX_TASK_TAG, "MOT1 END");
                mot1_moving = 0;
            } 

            if (data[3] == 224 && data[4] == 1 && data[5] == 225) {
                ESP_LOGI(RX_TASK_TAG, "MOT0 MOVING");
                mot0_moving = 1;
            } else if (data[3] == 224 && data[4] == 2 && data[5] == 226) {
                ESP_LOGI(RX_TASK_TAG, "MOT0 END");
                mot0_moving = 0;
            } else if (data[3] == 225 && data[4] == 1 && data[5] == 226) {
                ESP_LOGI(RX_TASK_TAG, "MOT1 MOVING");
                mot1_moving = 1;
            } else if (data[3] == 225 && data[4] == 2 && data[5] == 227) {
                ESP_LOGI(RX_TASK_TAG, "MOT1 END");
                mot1_moving = 0;
            }
        }
    }
    free(data);
}

static void movement_task(void *arg)
{
    float delta_angs[2] = {0, 0};
    float curr_angs[2] = {90, 0};
    float goal_xyz[3] = {0, 200, 0};
    float goal_angs[2] = {0, 0};
    float speeds[2] = {10, 10};

    //=========================================
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
        .alarm_count = STARTSPEED,
        .flags.auto_reload_on_alarm = true,
    };
    gptimer_set_alarm_action(gptimer0, &alarm0_config);
    //==============================================

    

    calculate_invkin(&goal_xyz[0], &goal_angs[0], &curr_angs[0], &delta_angs[0]);
    //calculate_fwdkin(&goal_xyz[0], &goal_angs[0]);

    vTaskDelay(5000/portTICK_PERIOD_MS);
    uint8_t z_dir = 0;

    //while(1) {

        // char chr[10];
        // while(1) {
        //     printf("\nEnter: ");
        //     scanf("%9s", chr);
        //     printf("\nEntered: %s\n", chr);
        //     strcpy(chr, "");
        //     vTaskDelay(5000/portTICK_PERIOD_MS);
        // }
        
        vTaskDelay(1500/portTICK_PERIOD_MS);

        //delta_angs[0] = -delta_angs[0];
        //delta_angs[1] = -delta_angs[1];
        move_arm_by_ang(delta_angs, speeds);

        step0_count = 1000;
        gptimer_start(gptimer0);
        gpio_set_level(DIR_IO0, z_dir);
        z_dir = !z_dir;
        motz_moving = 1;

        vTaskDelay(100/portTICK_PERIOD_MS);

        while (mot0_moving != 0 || mot1_moving != 0 || motz_moving != 0) {
            vTaskDelay(400/portTICK_PERIOD_MS);
            ESP_LOGI("BLOCK LOOP", "MOT0mov: %i MOT1mov: %i", mot0_moving, mot1_moving);
        }
    //}
}

static void command_receiver() 
{

}

void app_main(void)
{
    //-------------CONFIG GPIO--------------------------------
    gpio_set_direction(DIR_IO0, GPIO_MODE_OUTPUT);
    gpio_set_direction(STEP_IO0, GPIO_MODE_OUTPUT);

    //---------------CONFIG I2C----------------------------------
    
    i2c_master_init(I2C_MASTER_NUM0, I2C_MASTER_SDA_IO0, I2C_MASTER_SCL_IO0);
    i2c_master_init(I2C_MASTER_NUM1, I2C_MASTER_SDA_IO1, I2C_MASTER_SCL_IO1);

    //---------------CONFIG UART---------------------

    uart_init();

    //================PROGRAM=================================
    ESP_LOGI(TAG, "STARTING PROGRAM SECTION");

    gpio_set_level(DIR_IO0, 0);

    servo42c_enable();

    //servo42c_set_accel();

    xTaskCreate(rx_task1, "uart_rx_task_1", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(movement_task, "movement_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //servo42c_disable();


}