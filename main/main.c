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

#define STEP_IO0         GPIO_NUM_12
#define DIR_IO0          GPIO_NUM_14

#define I2C_MASTER_SDA_IO0   GPIO_NUM_13
#define I2C_MASTER_SCL_IO0   GPIO_NUM_15
#define I2C_MASTER_FREQ_HZ  100000
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_NUM0              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */

#define I2C_MASTER_NUM1              1                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_SDA_IO1   GPIO_NUM_18
#define I2C_MASTER_SCL_IO1   GPIO_NUM_23

#define STEPS_PER_DEG_01 26.667  //16x microstep, 3x reduction, 1.8deg motor

#define STARTSPEED 4000

static const int RX_BUF_SIZE = 2048;

#define TXD_PIN1 (GPIO_NUM_4)
#define RXD_PIN1 (GPIO_NUM_16)
#define TXD_PIN2 (GPIO_NUM_17)
#define RXD_PIN2 (GPIO_NUM_5)

static const char *TAG = "SCARA";

//============FUNCTIONS=================================
void move_arm_by_ang(const float (delta_angs)[2], const float (speeds)[2]) 
{
    float delta_angs_compensated[2] = {delta_angs[0], delta_angs[1] + delta_angs[0]};
    
    for (int i = 0; i < 2; i++) {
    uint8_t speed = speeds[i];    //speed is 0-255 where 0-127 is 0-100% speed clockwise and 128-255 is 0-100% speed counter clockwise
    if (delta_angs[i] > 0) {
        speed = speed + 127;
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
    uart_write_bytes(i+1, (const char*) msg_mot, sizeof(msg_mot));
    } 
}

void servo42c_disable() {
    for (int i = 0; i < 2; i++) {
        uint8_t mot_id = 224+i;
        uint8_t checksum = (uint8_t)(mot_id + 0xF3 + 0x00);
        char msg_mot[] = {mot_id,0xF3,0x00,checksum};
        uart_write_bytes(i+1, (const char*) msg_mot, sizeof(msg_mot));
    }
}

void servo42c_enable() {
for (int i = 0; i < 2; i++) {
        uint8_t mot_id = 224+i;
        uint8_t checksum = (uint8_t)(mot_id + 0xF3 + 0x01);
        char msg_mot[] = {mot_id,0xF3,0x01,checksum};
        uart_write_bytes(i+1, (const char*) msg_mot, sizeof(msg_mot));
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

void init(void) {
    const uart_config_t uart_config = {
        .baud_rate = 38400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // config uart1 and uart2
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_driver_install(UART_NUM_2, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_1, TXD_PIN1, RXD_PIN1, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_set_pin(UART_NUM_2, TXD_PIN2, RXD_PIN2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
}

//===================TASKS====================

static void rx_task1(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK_1";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_1, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes", rxBytes);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

static void rx_task2(void *arg)
{
    static const char *RX_TASK_TAG = "RX_TASK_2";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, RX_BUF_SIZE, 1000 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            ESP_LOGI(RX_TASK_TAG, "Read %d bytes", rxBytes);
            ESP_LOG_BUFFER_HEXDUMP(RX_TASK_TAG, data, rxBytes, ESP_LOG_INFO);
        }
    }
    free(data);
}

static void movement_task(void *arg)
{
    float delta_angs[2] = {45, 0};
    float curr_angs[2] = {90, 0};
    float goal_xyz[3] = {0, 200, 0};
    float goal_angs[2] = {0, 0};
    float speeds[2] = {30, 30};

    calculate_invkin(&goal_xyz[0], &goal_angs[0], &curr_angs[0], &delta_angs[0]);
    while(1) {
        delta_angs[0] = -delta_angs[0];
        delta_angs[1] = -delta_angs[1];
        move_arm_by_ang(delta_angs, speeds);
        vTaskDelay(3000/portTICK_PERIOD_MS);
    }
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

    init();

    //================PROGRAM=================================
    ESP_LOGI(TAG, "STARTING PROGRAM SECTION");

    gpio_set_level(DIR_IO0, 0);

    servo42c_enable();

    xTaskCreate(rx_task1, "uart_rx_task_1", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(rx_task2, "uart_rx_task_2", 1024*2, NULL, configMAX_PRIORITIES-2, NULL);
    xTaskCreate(movement_task, "movement_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    //servo42c_disable();


}