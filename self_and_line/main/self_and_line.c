#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"
#include <math.h>

#define MODE NORMAL_MODE
#define BLACK_MARGIN 4095
#define WHITE_MARGIN 400
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000
#define MAX_PITCH_CORRECTION (90.0f)
#define MAX_PITCH_AREA (850.0f)
#define MAX_PITCH_RATE (850.0f)
#define MAX_PWM (80.0f)
#define MIN_PWM (60.0f)

/* Self Balancing Tuning Parameters
float forward_offset = 2.51f;
float forward_buffer = 3.1f;
*/
//                                        start with kp,kd=0
//                                            max pwm = 70/80 ... kp2= 6.5 kd2= 3.5 setpoint= 4 x = +6 y= +2

int optimum_duty_cycle = 60;
int lower_duty_cycle = 54;  // 50
int higher_duty_cycle = 66; // 76
float left_duty_cycle = 0, right_duty_cycle = 0;
const int weights[5] = {3, 1, 0, -1, -3};
float forward_pwm = 0;
float hlp = 0;
float llp = 0;
float x = 0;
float y = 0;

float error = 0, prev_error = 0, difference, cumulative_error, correction;
line_sensor_array line_sensor_readings;
/* 2    0  8  1.5 0   3   10            0.1  -0.1  67   67
   kp ki kd kp2 ki2 kd2   setpoint       x     y    llp  hlp                                    best result
minmax pwm 60 65
lowerhigherduty 60 80
purebalancingpart motor pwm
mixpart fwdpwm+correction
*/

/* 2 0 8 1.5 0 3 10 0.1 -0.1 67 67
minmax pwm 60 65
lowerhigherduty 60 80
purebalancingpart motor pwm
mixpart motorpwm+correction
not that great cuz when good balance then it enters mixedpart where motorpwm is very less
*/

void lsa_to_bar()
{
    uint8_t var = 0x00;
    bool number[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 5; i++)
    {
        number[7 - i] = (line_sensor_readings.adc_reading[i] < WHITE_MARGIN) ? 0 : 1; // If adc value is less than black margin, then set that bit to 0 otherwise 1.
        var = bool_to_uint8(number);                                                  // A helper function to convert bool array to unsigned int.
        ESP_ERROR_CHECK(set_bar_graph(var));                                          // Setting bar graph led with unsigned int value.
    }
}

void calculate_correction()
{
    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference; // yaw kp ki kd
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;

    for (int i = 0; i < 5; i++)
    {
        if (line_sensor_readings.adc_reading[i] > WHITE_MARGIN)
        {
            all_black_flag = 0;
        }
        weighted_sum += (float)(weights[i]) * (line_sensor_readings.adc_reading[i]);
        sum = sum + line_sensor_readings.adc_reading[i];
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
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

void calculate_motor_command(const float pitch_error, float *motor_cmd)
{

    static float prev_pitch_error = 0.0f;

    static float pitch_area = 0.0f;

    float pitch_error_difference = 0.0f;

    float pitch_correction = 0.0f, absolute_pitch_correction = 0.0f;

    float pitch_rate = 0.0f;

    float P_term = 0.0f, I_term = 0.0f, D_term = 0.0f;

    pitch_error_difference = pitch_error - prev_pitch_error;

    pitch_area += (pitch_error);

    pitch_rate = pitch_error_difference;

    P_term = read_pid_const2().kp2 * pitch_error;
    I_term = read_pid_const2().ki2 * bound(pitch_area, -MAX_PITCH_AREA, MAX_PITCH_AREA);
    D_term = read_pid_const2().kd2 * bound(pitch_rate, -MAX_PITCH_RATE, MAX_PITCH_RATE);

    pitch_correction = P_term + I_term + D_term;

    absolute_pitch_correction = fabsf(pitch_correction);

    *motor_cmd = bound(absolute_pitch_correction, 0, MAX_PITCH_CORRECTION);
    prev_pitch_error = pitch_error;
}

void self_and_line(void *arg)
{
 /**
	 * euler_angles are the complementary pitch and roll angles obtained from mpu6050
	 * mpu_offsets are the initial accelerometer angles at rest position
	*/
	float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};

	float pitch_angle, pitch_error;

	/**
	 * 1. motor_cmd : Stores theoritically calculated correction values obtained with PID.
	 * 2. motor_pwm : Variable storing bounded data obtained from motor_cmd which will be used for
	                  giving actual correction velocity to the motors
	*/
	float motor_cmd, motor_pwm = 0.0f;

	// Pitch angle where you want to go - pitch_cmd, setpoint and mpu_offsets are linked to one another
	float pitch_cmd = 0.0f;



	// Ensure successful initialisation of MPU-6050
	enable_mpu6050();
	
		// Function to enable Motor driver A in Normal Mode
	enable_motor_driver(a, NORMAL_MODE);

    ESP_ERROR_CHECK(enable_line_sensor());

	while (1)
	{
			/**
			 * read_mpu6050(euler_angle, mpu_offset) : Checking for successful calculation of complementary pitch 
			 *											and roll angles based on intial accelerometer angle
			*/
			// Ensure required values are obtained from mpu6050
			if (read_mpu6050(euler_angle, mpu_offset) == ESP_OK)
			{
				// To read PID setpoint from tuning_http_server
				pitch_cmd = 9;
				pitch_angle = euler_angle[1];
				pitch_error = pitch_cmd - pitch_angle;

				calculate_motor_command(pitch_error, &motor_cmd);

				//bound PWM values between max and min
				motor_pwm = bound((motor_cmd), MIN_PWM, MAX_PWM);

				// Bot tilts upwards
				if (pitch_error > 1.2)
				{
					// setting motor A0 with definite speed(duty cycle of motor driver PWM) in Backward direction
					set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, motor_pwm);
					// setting motor A1 with definite speed(duty cycle of motor driver PWM) in Backward direction
					set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, motor_pwm);
				}

				// Bot tilts downwards
				else if (pitch_error < -1.4)
				{
					// setting motor A0 with definite speed(duty cycle of motor driver PWM) in Forward direction
					set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, motor_pwm);
					// setting motor A1 with definite speed(duty cycle of motor driver PWM) in Forward direction
					set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, motor_pwm);
				}

				// Bot remains in desired region for vertical balance
				else
				{
					line_sensor_readings = read_line_sensor();
                    for(int i = 0; i < 5; i++)
                    {
                line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
            line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        }
        
        calculate_error();
        calculate_correction();
        // lsa_to_bar();
        
        left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);

        
        // ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
        // ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);
// #ifdef CONFIG_ENABLE_OLED
//         // Diplaying kp, ki, kd values on OLED 
//         if (read_pid_const().val_changed)
//         {
//             display_pid_values(read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, &oled_config);
//             reset_val_changed_pid_const();
//         }
// #endif

				}

				//ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
				// ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f :: Setpoint: %0.2f :: Roll: %0.2f | Pitch: %0.2f | PitchError: %0.2f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd, read_pid_const().setpoint, euler_angle[0], euler_angle[1], pitch_error);
				// ESP_LOGI("debug", "Pitch: %0.2f", pitch_angle);			
				vTaskDelay(10 / portTICK_PERIOD_MS);
			}
		
	}

	// Remove the task from the RTOS kernel management
	vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&self_and_line, "self_and_line", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}