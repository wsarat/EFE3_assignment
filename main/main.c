#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#define UART_NUM         UART_NUM_0
#define UART_TX_PIN      GPIO_NUM_1
#define UART_RX_PIN      GPIO_NUM_3
#define LED_PIN          GPIO_NUM_2
#define SWITCH_PIN       GPIO_NUM_4

TaskHandle_t task1Handle, task2Handle, task3Handle, task4Handle;
SemaphoreHandle_t uartSemaphore, switchSemaphore;

void task1(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        ESP_LOGI("Task1", "Tsk1-P1 <-");
        
        // Run for about 100 ticks
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
        
        ESP_LOGI("Task1", "Tsk1-P1 ->");
        
        // Block for 10 milliseconds
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task2(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        ESP_LOGI("Task2", "Tsk2-P2 <-");
        
        // Run for about 10 ticks
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
        
        ESP_LOGI("Task2", "Tsk2-P2 ->");
        
        // Block for 250 milliseconds
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void task3(void *pvParameters)
{
    uint8_t rxData;
    
    while (1)
    {
        ESP_LOGI("Task3", "Tsk3-P3 <-");
        
        // Wait for character on UART
        if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Read received characters
            while (uart_read_bytes(UART_NUM, &rxData, 1, pdMS_TO_TICKS(10)) > 0)
            {
                if (rxData == 'L' || rxData == 'l')
                {
                    // Toggle LED
                    gpio_set_level(LED_PIN, !gpio_get_level(LED_PIN));
                }
            }
        }
        
        ESP_LOGI("Task3", "Tsk3-P3 ->");
        
        // Block until a new character is received on UART
        xSemaphoreTake(uartSemaphore, portMAX_DELAY);
    }
}

void task4(void *pvParameters)
{
    while (1)
    {
        ESP_LOGI("Task4", "Tsk4-P4 <-");
        
        // Wait for user switch press
        if (xSemaphoreTake(switchSemaphore, portMAX_DELAY) == pdTRUE)
        {
            // Run for about 10 ticks
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        ESP_LOGI("Task4", "Tsk4-P4 ->");
        
        // Block until the switch is pressed again
        xSemaphoreTake(switchSemaphore, portMAX_DELAY);
    }
}

void app_main()
{
    // Configure the default UART to use USB serial port
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("Task1", ESP_LOG_INFO);
    esp_log_level_set("Task2", ESP_LOG_INFO);
    esp_log_level_set("Task3", ESP_LOG_INFO);
    esp_log_level_set("Task4", ESP_LOG_INFO);
    uart_config_t uartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uartConfig);
    uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, 256, 0, 0, NULL, 0);
    
    // Initialize LED pin
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize switch pin
    gpio_set_direction(SWITCH_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SWITCH_PIN, GPIO_PULLUP_ONLY);
    
    // Create semaphores
    uartSemaphore = xSemaphoreCreateBinary();
    switchSemaphore = xSemaphoreCreateBinary();
    
    // Create tasks
    xTaskCreate(task1, "Task1", 2048, NULL, 1, &task1Handle);
    xTaskCreate(task2, "Task2", 2048, NULL, 2, &task2Handle);
    xTaskCreate(task3, "Task3", 2048, NULL, 3, &task3Handle);
    xTaskCreate(task4, "Task4", 2048, NULL, 4, &task4Handle);
    
    // Start the scheduler
    vTaskStartScheduler();
    
    // Cleanup (should never reach here)
    vTaskDelete(task1Handle);
    vTaskDelete(task2Handle);
    vTaskDelete(task3Handle);
    vTaskDelete(task4Handle);
}
