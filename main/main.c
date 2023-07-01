#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <strings.h>

#define UART_NUM         UART_NUM_0
#define UART_TX_PIN      GPIO_NUM_1
#define UART_RX_PIN      GPIO_NUM_3
#define LED_PIN          GPIO_NUM_2
#define BUTTON          GPIO_NUM_0

TaskHandle_t task1Handle, task2Handle, task3Handle, task4Handle;
SemaphoreHandle_t uartSemaphore, switchSemaphore;

#define BUF_SIZE    256
#define RD_BUF_SIZE (BUF_SIZE)
static QueueHandle_t uart0_queue;

void task1(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        printf("\tTsk1-P1 <-\n");
        
        // Run for about 100 ticks
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(100));
        
        printf("\tTsk1-P1 ->\n");
        
        // Block for 10 milliseconds
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void task2(void *pvParameters)
{
    TickType_t lastWakeTime = xTaskGetTickCount();
    
    while (1)
    {
        printf("\t\tTsk2-P2 <-\n");
        
        // Run for about 10 ticks
        vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
        
        printf("\t\tTsk2-P2 ->\n");
        
        // Block for 250 milliseconds
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

void task3(void *pvParameters)
{
    uint8_t rxData;
    bool ledOn = false;
    
    while (1)
    {
        // Wait for character on UART
        if (xSemaphoreTake(uartSemaphore, portMAX_DELAY) == pdTRUE)
        {
            uart_read_bytes(UART_NUM_0, &rxData, 1, portMAX_DELAY);

            if (rxData == 'l' || rxData == 'L') {
                printf("\t\t\tTsk3-P3 <-\n");

                // toggles LED
                ledOn = !ledOn;
                gpio_set_level(LED_PIN, ledOn);

                // Run for about 10 ticks
                vTaskDelay(pdMS_TO_TICKS(50));

                printf("\t\t\tTsk3-P3 ->\n");
            }
        }
    }
}

void task4(void *pvParameters)
{
    while (1)
    {
        // Wait for user switch press
        if (xSemaphoreTake(switchSemaphore, portMAX_DELAY) == pdTRUE)
        {
            printf("\t\t\t\tTsk4-P4 <-\n");

            // Run for about 10 ticks
            vTaskDelay(pdMS_TO_TICKS(10));

            printf("\t\t\t\tTsk4-P4 ->\n");
        }
    }
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            if (event.type == UART_DATA) 
                xSemaphoreGive(uartSemaphore);
        }
    }
}

static void IRAM_ATTR gpio_interrupt_handler(void *args)
{
    int pinNumber = (int)args;

    if (pinNumber == 0) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(switchSemaphore, &xHigherPriorityTaskWoken);
    }
}

void app_main()
{
    uart_config_t uartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM, &uartConfig);
    //uart_set_pin(UART_NUM, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    //Install UART driver, and get the queue.
    uart_driver_install(UART_NUM_0, BUF_SIZE, BUF_SIZE, 20, &uart0_queue, 0);

    // Initialize LED pin
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    
    // Initialize switch pin
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON, GPIO_PULLUP_ONLY);
    
    // Create semaphores
    uartSemaphore = xSemaphoreCreateBinary();
    switchSemaphore = xSemaphoreCreateBinary();
    
    // init pin 0 (button) interrupt
    gpio_set_direction(BUTTON, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTON, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(BUTTON, GPIO_INTR_LOW_LEVEL);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON, gpio_interrupt_handler, (void *)BUTTON);

    // Create tasks
    xTaskCreate(task1, "Task1", 2048, NULL, 1, &task1Handle);
    xTaskCreate(task2, "Task2", 2048, NULL, 2, &task2Handle);
    xTaskCreate(task3, "Task3", 2048, NULL, 3, &task3Handle);
    xTaskCreate(task4, "Task4", 2048, NULL, 4, &task4Handle);

    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
    
    /*
    https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html
    Users of FreeRTOS in ESP-IDF must never call vTaskStartScheduler() and vTaskEndScheduler(). Instead, ESP-IDF will start FreeRTOS automatically.

    // Start the scheduler
    vTaskStartScheduler();
    
    
    // Cleanup (should never reach here)
    vTaskDelete(task1Handle);
    vTaskDelete(task2Handle);
    vTaskDelete(task3Handle);
    vTaskDelete(task4Handle);
    */ 
}
