#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ultrasonic.h>
#include <esp_err.h>
#include <driver/gpio.h>

#define MAX_DISTANCE_CM 500 // 5m max
#define TRIGGER_GPIO 5
#define ECHO_GPIO 18

// Definir los pines para los LEDs
const int LED_PINS[10] = {19, 21, 22, 23, 25, 26, 27, 32, 33, 34};

void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    // Configurar los pines de los LEDs como salida
    for (int i = 0; i < 10; i++)
    {
        gpio_pad_select_gpio(LED_PINS[i]);
        gpio_set_direction(LED_PINS[i], GPIO_MODE_OUTPUT);
    }

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
        {
            printf("Distance: %0.04f cm\n", distance * 100);

            // Apagar todos los LEDs primero
            for (int i = 0; i < 10; i++)
            {
                gpio_set_level(LED_PINS[i], 0);
            }

            // Encender LEDs según la distancia
            int led_to_activate = (int)(distance * 100) / 50;
            if (led_to_activate > 0 && led_to_activate <= 10)
            {
                for (int i = 0; i < led_to_activate; i++)
                {
                    gpio_set_level(LED_PINS[i], 1);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main()
{
    xTaskCreate(ultrasonic_test, "ultrasonic_test", configMINIMAL_STACK_SIZE * 3, NULL, 5, NULL);
}
