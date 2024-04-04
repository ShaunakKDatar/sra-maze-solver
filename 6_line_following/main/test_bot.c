#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095 // Change to 4095 for black background
#define WHITE_MARGIN 0    // Change to 0 for white line
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define BLACK_BOUNDARY 300 // Boundary value to distinguish between white and black readings

/*
 * weights given to respective line sensor
 */
const int weights[5] = {-5, -3, 1, 3, 5};

/*
 * Motor value bounds
 */
int optimum_duty_cycle = 57;
int lower_duty_cycle = 45;
int higher_duty_cycle = 65;
float left_duty_cycle = 0, right_duty_cycle = 0;

/*
 * Line Following PID Variables
 */
float error = 0, prev_error = 0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;

/*
 * Maze solving states
 */
typedef enum
{
    FOLLOW_LINE,
    TURN_LEFT,
    TURN_RIGHT,
    LEFT_INTERSECTION,
    RIGHT_INTERSECTION,
    INTERSECTION,
    DEAD_END,
    REACHED_GOAL
} MazeState;

MazeState current_state = FOLLOW_LINE;

#define MAX_MAZE_PATH_LENGTH 100 // Adjust the maximum length of the maze path as needed

typedef enum
{
    LEFT_TURN,
    RIGHT_TURN
} TurnDirection;

typedef struct
{
    TurnDirection direction;
} MazeTurn;

MazeTurn maze_path[MAX_MAZE_PATH_LENGTH];
int maze_path_length = 0;

void calculate_correction()
{
    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    int k = 0;

    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            all_black_flag = 0;
        }
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY)
        {
            k = 1;
        }
        if (line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY)
        {
            k = 0;
        }
        weighted_sum += (float)(weights[i]) * k;
        sum = sum + k;
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = (weighted_sum - 1) / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
    {
        if (prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
    }
    else
    {
        error = pos;
    }
}

bool check_after_intersection(motor_handle_t motor_a_0, motor_handle_t motor_a_1, line_sensor_array readings)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);  // Go ahead for 1 second
    set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle); // GO ahead for 1 second
    vTaskDelay(1000 / portTICK_PERIOD_MS);                       // Delay
    set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);                // Stop the bot
    set_motor_speed(motor_a_1, MOTOR_FORWARD, 0);                // Stop the bot
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if (readings.adc_reading[0] > BLACK_BOUNDARY && readings.adc_reading[4] > BLACK_BOUNDARY) // Check if even now the bot reads a white section
    {
        set_motor_speed(motor_a_0, MOTOR_FORWARD, -left_duty_cycle);                              // Get the bot back to it's original state
        set_motor_speed(motor_a_1, MOTOR_FORWARD, -right_duty_cycle);                             // Get the bot back to it's original state
        if (readings.adc_reading[0] > BLACK_BOUNDARY && readings.adc_reading[4] > BLACK_BOUNDARY) // Check if the bot is back to it's original state
        {
            set_motor_speed(motor_a_0, MOTOR_FORWARD, 0); // Stop the bot there
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 0); // Stop the bot there
        }
        current_state = REACHED_GOAL; // Change the state to reached goal
        return false;                 // No intersection detected
    }
    else
    {
        set_motor_speed(motor_a_0, MOTOR_FORWARD, -left_duty_cycle);
        set_motor_speed(motor_a_1, MOTOR_FORWARD, -right_duty_cycle);
        if (readings.adc_reading[0] > BLACK_BOUNDARY && readings.adc_reading[4] > BLACK_BOUNDARY)
        {
            set_motor_speed(motor_a_0, MOTOR_FORWARD, 0);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, 0);
        }
        return true;
    }
}

void detect_intersection(motor_handle_t motor_a_0, motor_handle_t motor_a_1, line_sensor_array readings)
{
    if (readings.adc_reading[0] > BLACK_BOUNDARY && readings.adc_reading[4] > BLACK_BOUNDARY && check_after_intersection(motor_a_0, motor_a_1, readings))
    {
        current_state = INTERSECTION;
    }
    else if (readings.adc_reading[0] > BLACK_BOUNDARY && check_after_intersection(motor_a_0, motor_a_1, readings))
    {
        current_state = LEFT_INTERSECTION;
    }
    else if (readings.adc_reading[4] > BLACK_BOUNDARY && check_after_intersection(motor_a_0, motor_a_1, readings))
    {
        current_state = RIGHT_INTERSECTION;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void follow_line(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
}

void maze_solve_task(void *arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));

    while (true)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        for (int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }

        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

        detect_intersection(motor_a_0, motor_a_1, line_sensor_readings);

        if (current_state == LEFT_INTERSECTION || current_state == RIGHT_INTERSECTION || current_state == INTERSECTION)
        {
            ESP_LOGI("debug", "intersection reached!! YAYYY");
        }

        follow_line(motor_a_0, motor_a_1);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void app_main()
{
    xTaskCreate(&maze_solve_task, "maze_solve_task", 4096, NULL, 1, NULL);

    start_tuning_http_server();
}