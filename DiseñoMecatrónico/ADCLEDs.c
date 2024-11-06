#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

#define ADC1_CHAN0 ADC_CHANNEL_6 // Pin 34
#define ADC_ATTEN ADC_ATTEN_DB_11

// Pines de los LEDs
const int LED_PINS[10] = {19, 21, 22, 23, 25, 26, 27, 32, 33, 2};

adc_oneshot_unit_handle_t adc1_handle;
static int adc_raw; 
static float voltage;

esp_err_t config_ADC();
esp_err_t get_ADC_value();

void config_LEDs() {
    for (int i = 0; i < 10; i++) {
        esp_rom_gpio_pad_select_gpio(LED_PINS[i]);
        gpio_set_direction(LED_PINS[i], GPIO_MODE_OUTPUT);
    }
}

void app_main(void)
{
    // Configurar los LEDs
    config_LEDs();

    // Configurar el ADC
    esp_err_t ret = config_ADC(); 
    if (ret != ESP_OK) {
        printf("Failed to initialize ADC: %s\n", esp_err_to_name(ret));
        return;
    }

    while (true) {
        ret = get_ADC_value();
        if (ret != ESP_OK) {
            printf("Failed to get ADC value: %s\n", esp_err_to_name(ret));
        } else {
            printf("Raw data: %d\n", adc_raw);
            voltage = ((adc_raw * 5.0) / 4095.0); // Convertir a voltaje basado en referencia de 5V
            printf("Voltage: %2.2f V\n", voltage);

            // Apagar todos los LEDs primero
            for (int i = 0; i < 10; i++) {
                gpio_set_level(LED_PINS[i], 0);
            }

            // Encender LEDs según el voltaje
            int led_to_activate = (int)(voltage / 0.5); // Cada LED representa 0.5V (5V / 10 LEDs)
            if (led_to_activate > 0 && led_to_activate <= 10) {
                for (int i = 0; i < led_to_activate; i++) {
                    gpio_set_level(LED_PINS[i], 1);
                    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay de 100 ms para apreciar el encendido
                }
            }
        }

        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay general para dar tiempo al ADC
    } 
}

esp_err_t config_ADC() {
    // ADC init
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };

    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        printf("Error inicializando ADC unit: %s\n", esp_err_to_name(ret));
        return ret;
    }

    // ADC config
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT, // ancho de banda
        .atten = ADC_ATTEN, // Atenuacion
    };

    ret = adc_oneshot_config_channel(adc1_handle, ADC1_CHAN0, &config);
    if (ret != ESP_OK) {
        printf("Error configurando ADC channel: %s\n", esp_err_to_name(ret));
    }

    return ret; 
}

esp_err_t get_ADC_value() {
    esp_err_t ret = adc_oneshot_read(adc1_handle, ADC1_CHAN0, &adc_raw);
    if (ret != ESP_OK) {
        printf("Error ADC: %s\n", esp_err_to_name(ret));
        return ret;
    }
    return ESP_OK;
}
