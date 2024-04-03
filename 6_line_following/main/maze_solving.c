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
    DETECT_INTERSECTION,
    END_OF_MAZE,
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

// Function to record a left turn in the maze path
void record_left_turn()
{
    if (maze_path_length < MAX_MAZE_PATH_LENGTH)
    {
        maze_path[maze_path_length].direction = LEFT_TURN;
        maze_path_length++;
    }
}

// Function to record a right turn in the maze path
void record_right_turn()
{
    if (maze_path_length < MAX_MAZE_PATH_LENGTH)
    {
        maze_path[maze_path_length].direction = RIGHT_TURN;
        maze_path_length++;
    }
}

// Intersection detection based on LSA weights
bool detect_intersection(line_sensor_array readings)
{
    if (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY)
    {
        set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
        if (line_sensor_readings.adc_reading[0] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[1] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[2] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[3] > BLACK_BOUNDARY && line_sensor_readings.adc_reading[4] > BLACK_BOUNDARY)
        {
            set_motor_speed(motor_a_0, MOTOR_FORWARD, -left_duty_cycle);
            set_motor_speed(motor_a_1, MOTOR_FORWARD, -right_duty_cycle);
            return false;
        }
        else
        {
            return true;
        }
    }
}

// Function to calculate error based on white boundary
void calculate_error()
{
    int all_white_flag = 1; // assuming initially all white condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;
    int k = 0;

    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY) // Check for white instead of black
        {
            all_white_flag = 0;
        }
        if (line_sensor_readings.adc_reading[i] > BLACK_BOUNDARY) // Check for white instead of black
        {
            k = 1;
        }
        if (line_sensor_readings.adc_reading[i] < BLACK_BOUNDARY) // Check for white instead of black
        {
            k = 0;
        }
        weighted_sum += (float)(weights[i]) * k;
        sum = sum + k;
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = (weighted_sum - 1) / sum; // This will give us the position with respect to the line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if (all_white_flag == 1) // If all white then we check for previous error to assign current error.
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

void calculate_correction()
{
    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void follow_line(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle);
}

void turn_left(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_BACKWARD, left_duty_cycle - optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_FORWARD, right_duty_cycle + optimum_duty_cycle);
}

void turn_right(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_FORWARD, left_duty_cycle + optimum_duty_cycle);
    set_motor_speed(motor_a_1, MOTOR_BACKWARD, right_duty_cycle - optimum_duty_cycle);
}

void stop_robot(motor_handle_t motor_a_0, motor_handle_t motor_a_1)
{
    set_motor_speed(motor_a_0, MOTOR_STOP, 0);
    set_motor_speed(motor_a_1, MOTOR_STOP, 0);
}

void maze_solve_task(void *arg)
{
    motor_handle_t motor_a_0, motor_a_1;
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_0, MOTOR_A_0));
    ESP_ERROR_CHECK(enable_motor_driver(&motor_a_1, MOTOR_A_1));
    adc_handle_t line_sensor;
    ESP_ERROR_CHECK(enable_line_sensor(&line_sensor));

    bool exploration_done = false; // Flag to indicate exploration completion
    int path_index = 0;            // Index to iterate through the recorded path

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

        switch (current_state)
        {
        case FOLLOW_LINE:
            if (error > 0)
            {
                follow_line(motor_a_0, motor_a_1);
            }
            else if (error < 0)
            {
                turn_left(motor_a_0, motor_a_1);
                record_left_turn(); // Record left turn
#ifdef CONFIG_ENABLE_OLED
                // Display message on OLED
                lv_label_set_text(label, "Turning left...");
                lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
#endif
            }
            else
            {
                turn_right(motor_a_0, motor_a_1);
                record_right_turn(); // Record right turn
#ifdef CONFIG_ENABLE_OLED
                // Display message on OLED
                lv_label_set_text(label, "Turning right...");
                lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
#endif
            }

            // Check for intersection
            if (detect_intersection(line_sensor_readings))
            {
                if (exploration_done)
                {
                    // Use recorded path for solving
                    current_state = maze_path[path_index].direction == LEFT_TURN ? TURN_LEFT : TURN_RIGHT;
                    path_index++; // Move to the next turn in the recorded path
                }
                else
                {
                    // Continue exploration (existing logic for recording turns)
                }
            }
            break;

        case TURN_LEFT:
            turn_left(motor_a_0, motor_a_1);
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Adjust delay for turning left
            current_state = FOLLOW_LINE;
            break;

        case TURN_RIGHT:
            turn_right(motor_a_0, motor_a_1);
            vTaskDelay(1000 / portTICK_PERIOD_MS); // Adjust delay for turning right
            current_state = FOLLOW_LINE;
            break;

        case DETECT_INTERSECTION:
            // Check for goal (black square)
            if (detect_goal(line_sensor_readings))
            {
                current_state = REACHED_GOAL;
#ifdef CONFIG_ENABLE_OLED
                // Display message on OLED
                lv_label_set_text(label, "Goal reached!");
                lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
#endif
                break;
            }
            // Check for end of maze
            if (detect_end_of_maze(line_sensor_readings))
            {
                current_state = END_OF_MAZE;
#ifdef CONFIG_ENABLE_OLED
                // Display message on OLED
                lv_label_set_text(label, "End of maze detected!");
                lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
#endif
                break;
            }
            // Otherwise, continue following the line
            follow_line(motor_a_0, motor_a_1);
            break;

        case END_OF_MAZE:
            // Stop the robot
            stop_robot(motor_a_0, motor_a_1);
            // Perform action to turn back or navigate back to the path
            // For simplicity, let's assume turning back
            turn_left(motor_a_0, motor_a_1);       // Turn around 180 degrees
            vTaskDelay(2000 / portTICK_PERIOD_MS); // Adjust delay for turning around
            current_state = FOLLOW_LINE;           // Return to following the line
            break;

        case REACHED_GOAL:
            // Stop the robot at the goal
            stop_robot(motor_a_0, motor_a_1);
            // You may perform additional actions here, such as signaling completion
            // Then, exit the maze solving task
            vTaskDelete(NULL);
            break;

        default:
            break;
        }

        // Check for exploration completion and introduce delay
        if (!exploration_done && detect_end_of_maze(line_sensor_readings))
        {
            exploration_done = true;
#ifdef CONFIG_ENABLE_OLED
            // Display message on OLED
            lv_label_set_text(label, "Exploration complete. Solving maze...");
            lv_obj_align(label, NULL, LV_ALIGN_CENTER, 0, 0);
#endif
            vTaskDelay(20000 / portTICK_PERIOD_MS); // Delay for 20 seconds
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&maze_solve_task, "maze_solve_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}
