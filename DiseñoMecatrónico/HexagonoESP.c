#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

/* Set the parameters according to your servo */
#define SERVO_MIN_PULSEWIDTH_US 500 /* Minimum pulse width in microsecond */
#define SERVO_MAX_PULSEWIDTH_US 2400 /* Maximum pulse width in microsecond */
#define SERVO_MIN_DEGREE 0 /* Minimum angle */
#define SERVO_MAX_DEGREE 180 /* Maximum angle */
#define SERVO1_PULSE_GPIO 26 /* GPIO connects to the PWM signal line of Servo 1 */
#define SERVO2_PULSE_GPIO 27 /* GPIO connects to the PWM signal line of Servo 2 */
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000 /* 1MHz, 1us per tick */
#define SERVO_TIMEBASE_PERIOD 20000 /* 20000 ticks, 20ms */

static const char *TAG = "PWM servo";

// Function declarations to avoid implicit declaration errors
static inline uint32_t angle_to_compare(int angle);

mcpwm_timer_handle_t timer = NULL;
mcpwm_oper_handle_t oper1 = NULL;
mcpwm_oper_handle_t oper2 = NULL;
mcpwm_cmpr_handle_t comparator1 = NULL;
mcpwm_cmpr_handle_t comparator2 = NULL;
mcpwm_gen_handle_t generator1 = NULL;
mcpwm_gen_handle_t generator2 = NULL;

esp_err_t mcpwm_config();

void update_servo_angle(int angle) {
    ESP_LOGI(TAG, "Setting angle of Servo 1: %d", angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(angle)));
}

void update_servo_angle2(int angle) {
    ESP_LOGI(TAG, "Setting angle of Servo 2: %d", angle);
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(angle)));
}

void app_main(void) {
    mcpwm_config();

    // Ángulos del hexágono para los servos (theta 2 y theta 5)
    float th2_values[] = {112.3753, 111.9200, 111.3706, 110.7224, 109.9699, 109.1058, 108.1206, 107.0020, 105.7334, 104.2924,
                          102.6469, 104.7981, 107.0192, 109.3166, 111.5456, 113.7115, 115.8181, 117.8686, 119.8657, 121.8115,
                          123.7075, 126.2593, 128.7902, 131.3081, 133.8199, 136.5696, 139.5061, 142.4106, 145.2933, 148.1633,
                          151.0285, 152.4321, 153.7792, 155.0720, 156.3124, 157.5014, 158.6397, 159.7277, 160.7650, 161.7509,
                          162.6843, 160.4565, 158.1723, 155.8368, 153.4550, 151.0322, 148.5731, 146.0824, 143.5643, 141.0228,
                          138.4614, 135.8492, 133.2617, 130.6903, 128.1264, 125.5612, 122.9857, 120.3905, 117.7650, 115.0977,
                          112.3753};

    float th5_values[] = {51.3674, 54.3604, 57.4301, 60.6000, 63.9018, 67.3796, 71.0980, 75.1618, 79.7620, 85.3358,
                          93.6279, 98.0406, 103.8532, 104.9577, 106.0510, 107.1324, 108.2015, 109.2574, 110.3000, 111.3289,
                          112.3433, 113.2155, 114.1082, 115.0213, 115.9553, 109.3863, 105.0904, 102.4853, 100.6157, 99.1988,
                          98.1046, 94.1197, 90.5848, 87.3578, 84.3560, 81.5258, 78.8300, 76.2414, 73.7391, 71.3067,
                          68.9304, 66.3380, 63.7651, 61.2116, 58.6776, 56.1637, 53.6714, 51.2021, 48.7581, 46.3418,
                          43.9562, 44.2857, 44.7063, 45.2167, 45.8168, 46.5065, 47.2869, 48.1597, 49.1279, 50.1952,
                          51.3674};

    int num_steps = sizeof(th2_values) / sizeof(th2_values[0]);

    while (1) {
        for (int i = 0; i < num_steps; i++) {
            // Actualiza los ángulos de los servos según los valores de th2 y th5
            update_servo_angle((int)th2_values[i]);
            update_servo_angle2((int)th5_values[i]);
            vTaskDelay(pdMS_TO_TICKS(500)); // Espera 500 ms entre cada paso
        }
    }
}


esp_err_t mcpwm_config() {
    ESP_LOGI(TAG, "Create timer and operators");

    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
        .period_ticks = SERVO_TIMEBASE_PERIOD,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    // Configure first operator for Servo 1
    mcpwm_operator_config_t operator_config1 = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config1, &oper1));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper1, timer));

    mcpwm_comparator_config_t comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper1, &comparator_config, &comparator1));

    mcpwm_generator_config_t generator_config1 = {
        .gen_gpio_num = SERVO1_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper1, &generator_config1, &generator1));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator1, angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator1, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator1, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator1, MCPWM_GEN_ACTION_LOW)));

    // Configure second operator for Servo 2
    mcpwm_operator_config_t operator_config2 = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config2, &oper2));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper2, timer));

    ESP_ERROR_CHECK(mcpwm_new_comparator(oper2, &comparator_config, &comparator2));

    mcpwm_generator_config_t generator_config2 = {
        .gen_gpio_num = SERVO2_PULSE_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper2, &generator_config2, &generator2));

    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator2, angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator2, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator2, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator2, MCPWM_GEN_ACTION_LOW)));

    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    return ESP_OK;
}

// Moved angle_to_compare function up for proper declaration
static inline uint32_t angle_to_compare(int angle) {
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
