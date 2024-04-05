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
#define BLACK_BOUNDARY 900 // Boundary value to distinguish between white and black readings

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
 * Circular buffer for storing past LSA readings
 */
#define BUFFER_SIZE 500 // Adjust buffer size as needed
line_sensor_array lsa_buffer[BUFFER_SIZE];
int buffer_index = 0;

/*
 * Maze solving states
 */
typedef enum
{
    FOLLOW_LINE,
    INTERSECTION,
    END,
    GOAL,
} MazeState;

MazeState current_state = FOLLOW_LINE;

#define MAX_MAZE_PATH_LENGTH 100 // Adjust the maximum length of the maze path as needed

typedef enum
{
    LEFT_TURN,
    RIGHT_TURN,
    STRAIGHT,
    DEAD_END,
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

void left_on_intersection(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_BACKWARD, optimum_duty_cycle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    current_state = FOLLOW_LINE;
}

void right_on_intersection(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_BACKWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    current_state = FOLLOW_LINE;
}

void uturn(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_BACKWARD, optimum_duty_cycle);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    set_motor_speed(motor_a_0, MOTOR_FORWARD, optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, optimum_duty_cycle);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    current_state = FOLLOW_LINE;
}

void calculate_error()
{
    // Use past LSA readings to calculate error
    int all_black_flag = 1;
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    int k = 0;

    // Use readings from the circular buffer
    for (int j = 0; j < BUFFER_SIZE; j++)
    {
        for (int i = 0; i < 5; i++)
        {
            if (lsa_buffer[j].adc_reading[i] > BLACK_BOUNDARY)
            {
                all_black_flag = 0;
            }
            if (lsa_buffer[j].adc_reading[i] > BLACK_BOUNDARY)
            {
                k = 1;
            }
            if (lsa_buffer[j].adc_reading[i] < BLACK_BOUNDARY)
            {
                k = 0;
            }
            weighted_sum += (float)(weights[i]) * k;
            sum = sum + k;
        }
    }

    if (sum != 0)
    {
        pos = (weighted_sum - 1) / sum;
    }

    if (all_black_flag == 1)
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

void follow_line(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
}

void detect_intersection(line_sensor_array line_sensor_readings)
{
    if (current_state == GOAL)
    {
        return;
    }
    if (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY)
    {
        current_state = INTERSECTION;
    }
    if (line_sensor_readings.adc_reading[0] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] < BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] < BLACK_BOUNDARY)
    {
        current_state = END;
    }
}

void goal_test(line_sensor_array line_sensor_readings)
{
    if (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY)
    {
        current_state = GOAL;
    }
}

void store_lsa_reading(line_sensor_array current_reading)
{
    // Store the current reading in the circular buffer
    lsa_buffer[buffer_index] = current_reading;
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;
}

void maze_solve_task(void *arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));
    bool exploration = true;

    while (true)
    {
        line_sensor_readings = read_line_sensor(line_sensor);
        for (int i = 0; i < 5; i++)
        {
            line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }

        // Store current LSA reading
        store_lsa_reading(line_sensor_readings);

        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);

        current_state = FOLLOW_LINE;
        follow_line(motor_a_0, motor_a_1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        detect_intersection(line_sensor_readings);
        if (exploration)
        {
            switch (current_state)
            {
            case FOLLOW_LINE:
            {
                follow_line(motor_a_0, motor_a_1);
                break;
            }
            case INTERSECTION:
            {
                left_on_intersection(motor_a_0, motor_a_1);
                maze_path[maze_path_length].direction = LEFT_TURN;
                maze_path_length++;
                break;
            }
            case END:
            {
                uturn(motor_a_0, motor_a_1);
                maze_path[maze_path_length].direction = DEAD_END;
                maze_path_length++;
                break;
            }
            case GOAL:
            {
                exploration = false;
                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            }
        }

        if (!exploration)
        {
            break;
        }
        goal_test(line_sensor_readings);
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void app_main()
{
    xTaskCreate(&maze_solve_task, "maze_solve_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}