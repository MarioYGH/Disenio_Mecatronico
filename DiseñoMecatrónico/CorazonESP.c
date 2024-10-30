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

    // Ángulos del corazón para los servos (theta 2 y theta 5)
    float th2_values[] = {
        150.1961, 150.0771, 149.6940, 149.0185, 148.0384, 146.7574, 145.1944, 143.3809, 141.3581, 139.1740,
        136.8801, 134.5295, 132.1750, 129.8668, 127.6522, 125.5744, 123.6723, 121.9803, 120.5279, 119.3413,
        118.4427, 117.8511, 117.5828, 117.6509, 118.0662, 118.8359, 119.9636, 121.4497, 123.2896, 125.4744,
        127.9908, 130.8202, 133.9396, 137.3207, 140.9310, 144.7329, 148.6841, 152.7378, 156.8424, 160.9411,
        164.9720, 168.8675, 172.5539, 175.9524, 178.9809, 181.5593, 183.6191, 185.1170, 186.0512, 186.4710,
        186.4768, 186.2051, 185.8049, 185.4161, 185.1521, 185.0921, 185.2798, 185.7250, 186.4075, 187.2820,
        188.2831, 189.3315, 190.3408, 191.2252, 191.9055, 192.3157, 192.4060, 192.1443, 191.5162, 190.5221,
        189.1753, 187.4989, 185.5241, 183.2870, 180.8291, 178.1947, 175.4309, 172.5867, 169.7125, 166.8587,
        164.0755, 161.4108, 158.9097, 156.6125, 154.5532, 152.7585, 151.2467, 150.0269, 149.0992, 148.4541,
        148.0741, 147.9335, 147.9993, 148.2315, 148.5842, 149.0052, 149.4375, 149.8206, 150.0931, 150.1961
    };

    float th5_values[] = {
        50.1374, 50.2162, 50.4187, 50.6931, 50.9873, 51.2509, 51.4363, 51.4999, 51.4017, 51.1060,
        50.5824, 49.8066, 48.7620, 47.4408, 45.8451, 43.9874, 41.8910, 39.5884, 37.1208, 34.5357,
        31.8856, 29.2263, 26.6150, 24.1094, 21.7665, 19.6410, 17.7849, 16.2459, 15.0658, 14.2782,
        13.9072, 13.9631, 14.4417, 15.3205, 16.5597, 18.1019, 19.8750, 21.7981, 23.7858, 25.7545,
        27.6281, 29.3417, 30.8443, 32.1013, 33.0944, 33.8236, 34.3060, 34.5766, 34.6875, 34.7057,
        34.7091, 34.7822, 35.0077, 35.4601, 36.1985, 37.2618, 38.6675, 40.4123, 42.4746, 44.8195,
        47.4019, 50.1711, 53.0732, 56.0539, 59.0597, 62.0396, 64.9455, 67.7328, 70.3611, 72.7936,
        74.9980, 76.9461, 78.6139, 79.9817, 81.0337, 81.7584, 82.1479, 82.1990, 81.9122, 81.2930,
        80.3525, 79.1073, 77.5809, 75.8037, 73.8127, 71.6512, 69.3677, 67.0148, 64.6474, 62.3210,
        60.0901, 58.0061, 56.1152, 54.4563, 53.0589, 51.9401, 51.1045, 50.5427, 50.2318, 50.1374
    };

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
