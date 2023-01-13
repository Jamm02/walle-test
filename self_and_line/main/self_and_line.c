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
#define MAX_PWM (90.0f)
#define MIN_PWM (75.0f)

/* Self Balancing Tuning Parameters
float forward_offset = 2.51f;
float forward_buffer = 3.1f;
*/

float left_duty_cycle = 0, right_duty_cycle = 0;
const int weights[5] = {-3, -1, 0, 1, 3};
bool balanced = false;
float fwd_offset = 2.5;
float forward_angle = 0;
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

void LF();
void lsa_to_bar()
{
    uint8_t var = 0x00;
    bool number[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (int i = 0; i < 5; i++)
    {
        number[7 - i] = (line_sensor_readings.adc_reading[i] < WHITE_MARGIN) ? 0 : 1; // If adc value is less than white margin, then set that bit to 0 otherwise 1.
        var = bool_to_uint8(number);                                                  // A helper function to convert bool array to unsigned int.
        ESP_ERROR_CHECK(set_bar_graph(var));                                          // Setting bar graph led with unsigned int value.
    }
}

void calculate_correction()
{
    error = error*10;  // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp*error + read_pid_const().ki*cumulative_error + read_pid_const().kd*difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0; 
    float pos = 0;
    
    for(int i = 0; i < 5; i++)
    {
        if(line_sensor_readings.adc_reading[i] > 700)
        {
            all_black_flag = 0;
        }
        weighted_sum += (float)(weights[i]) * (line_sensor_readings.adc_reading[i]);
        sum = sum + line_sensor_readings.adc_reading[i];
    }

    if(sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if(all_black_flag == 1)  // If all black then we check for previous error to assign current error.
    {
        if(prev_error > 0)
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
	
	/** Error values **/
	// Stores pitch error of previous iteration
	static float prev_pitch_error = 0.0f;
	// Stores sum of product of errors with time interval obtained during each iteration
	static float pitch_area = 0.0f;
	// Stores difference between current error and previous iteration error
	float pitch_error_difference = 0.0f;

	/** Correction values **/
	// Variables for storing corrected values
	float pitch_correction = 0.0f, absolute_pitch_correction = 0.0f;
	// Helper variables for calculating integral and derivative correction
	float pitch_rate = 0.0f;

	// Variables storing correction values of different error terms
	float P_term = 0.0f, I_term = 0.0f, D_term = 0.0f;

	// Evaluated delta(error)
	pitch_error_difference = pitch_error - prev_pitch_error;

	// Evaluated area of the graph error vs time (cumulative error)
	pitch_area += (pitch_error);
	// evaluated delta(error)/delta(time) to calculate rate of change in error w.r.t time
	pitch_rate = pitch_error_difference;

	// Calculating p,i and d terms my multuplying corresponding proportional constants
	P_term = read_pid_const2().kp2 * pitch_error;
	I_term = read_pid_const2().ki2 * bound(pitch_area, -MAX_PITCH_AREA, MAX_PITCH_AREA);
	D_term = read_pid_const2().kd2 * bound(pitch_rate, -MAX_PITCH_RATE, MAX_PITCH_RATE);

	pitch_correction = P_term + I_term + D_term;

	/**
	 * Calculating absolute value of pitch_correction since duty cycle can't be negative. 
	 * Since it is a floating point variable, fabsf was used instead of abs
	*/
	absolute_pitch_correction = fabsf(pitch_correction);

	*motor_cmd = bound(absolute_pitch_correction, 0, MAX_PITCH_CORRECTION);
	prev_pitch_error = pitch_error;
}

    float euler_angle[2], mpu_offset[2] = {0.0f, 0.0f};

    float pitch_angle = 0, pitch_error = 0;

    /**
     * 1. motor_cmd : Stores theoritically calculated correction values obtained with PID.
     * 2. motor_pwm : Variable storing bounded data obtained from motor_cmd which will be used for
                      giving actual correction velocity to the motors
    */
    float motor_cmd, motor_pwm = 0.0f;
    /**
     * euler_angles are the complementary pitch and roll angles obtained from mpu6050
     * mpu_offsets are the initial accelerometer angles at rest position
     */

    // Pitch angle where you want to go - pitch_cmd, setpoint and mpu_offsets are linked to one another
    float pitch_cmd = 0.0f;

void self_and_line(void *arg)
{

    // Ensure successful initialisation of MPU-6050
    enable_mpu6050();

    // Function to enable Motor driver A in Normal Mode
    enable_motor_driver(a, NORMAL_MODE);

    ESP_ERROR_CHECK(enable_line_sensor());

    while (1){
        //  ESP_LOGI("debug", "optimum: %d ::  higher: %d  :: lower: %d :: setpoint: %f", read_pid_const2().optimum_duty_cycle, read_pid_const2().higher_duty_cycle, read_pid_const2().lower_duty_cycle, read_pid_const2().setpoint);

        read_mpu6050(euler_angle, mpu_offset);
        calculate_error();
        // printf("%f \n", error);g
        calculate_correction();
        calculate_motor_command(pitch_error, &motor_cmd);
        motor_pwm = bound((motor_cmd), MIN_PWM, MAX_PWM);

        //  line_sensor_readings = read_line_sensor();
        //     for(int i = 0; i < 5; i++)
        //     {
        //         line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN);
        //         line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], WHITE_MARGIN, BLACK_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
        //         line_sensor_readings.adc_reading[i] = 1000 - (line_sensor_readings.adc_reading[i]);
        //     }
            
        //          ESP_LOGI("debug", "LSA_1: %d \t LSA_2: %d \t LSA_3: %d \t LSA_4: %d \t LSA_5: %d", line_sensor_readings.adc_reading[0], line_sensor_readings.adc_reading[1], line_sensor_readings.adc_reading[2], line_sensor_readings.adc_reading[3], line_sensor_readings.adc_reading[4]);

            /**
             * read_mpu6050(euler_angle, mpu_offset) : Checking for successful calculation of complementary pitch
             *											and roll angles based on intial accelerometer angle
             */
            // Ensure required values are obtained from mpu6050
            // ESP_LOGI("debug","error %f :: correction %f\n", error, correction);
        // if (!balanced)
        // {
        //     // ESP_LOGI("debug", "bal? : %s :: ", "no");

            // To read PID setpoint from tuning_http_server
            pitch_cmd = read_pid_const2().setpoint;
            pitch_angle = euler_angle[1];
            pitch_error = pitch_cmd - pitch_angle;
            // printf("%f \n",pitch_error);
            // Bot tilts upwards
            if (pitch_error > read_pid_const2().pitcherrup)
            {
                // ESP_LOGI("debug", "%f \n", correction);
                //  if (error < -1*read_pid_const2().percent_lf || error > read_pid_const2().percent_lf){
                    // ESP_LOGI("debug","error %f :: correction %f :: pitcherr %f\n", error, correction, read_pid_const2().pitcherrup);

                    // setting motor A0 with definite speed(duty cycle of motor driver PWM) in Backward direction
                    // set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, (motor_pwm - bound(correction, -6, 6)));
                    // setting motor A1 with definite speed(duty cycle of motor driver PWM) in Backward direction
                    // set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, (motor_pwm + bound(correction, -6, 6)));
                    
                    // if((motor_pwm - bound(correction, -5, 5)) > (motor_pwm + bound(correction, -5, 5))){
                    //     ESP_LOGI("debug","going left from 1");
                    // }
                    // else{
                    //     ESP_LOGI("debug","going right from 1");
                        
                    // }
                // }
                
                // setting motor A0 with definite speed(duty cycle of motor driver PWM) in Backward direction
                // else{

                set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, motor_pwm);
                // setting motor A1 with definite speed(duty cycle of motor driver PWM) in Backward direction
                set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, motor_pwm);
                // }

            }

            // Bot tilts downwards
            else if (pitch_error < -1 * read_pid_const2().pitcherrdown)
            {
                // if (error < -1*read_pid_const2().percent_lf || error > read_pid_const2().percent_lf){
                //     // ESP_LOGI("debug","error %f :: correction %f :: pitcherr %f\n", error, correction, read_pid_const2().pitcherrdown);
                //     // setting motor A0 with definite speed(duty cycle of motor driver PWM) in Backward direction
                //     set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, (motor_pwm - bound(correction, -6, 6)));
                //     // setting motor A1 with definite speed(duty cycle of motor driver PWM) in Backward direction
                //     set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, (motor_pwm + bound(correction, -6, 6)));
                //     // if((motor_pwm - bound(correction, -5, 5)) > (motor_pwm + bound(correction, -5, 5))){
                //     //     ESP_LOGI("debug","going right from 2");
                //     // }
                //     // else{
                //     //     ESP_LOGI("debug","going left from 2");

                //     // }
                // }
                // else{
                // setting motor A0 with definite speed(duty cycle of motor driver PWM) in Forward direction
                set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, motor_pwm);
                // setting motor A1 with definite speed(duty cycle of motor driver PWM) in Forward direction
                set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, motor_pwm);
                // }
            }
            else
            {
                set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
                // setting motor A1 with definite speed(duty cycle of motor driver PWM) in Forward direction
                set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
                // pitch_cmd = forward_angle;
                // balanced = true;
            }
        }
        // Bot remains in desired region for vertical balance
    //     else
    //     {
            
    //     //    LF();

    //         // vTaskDelay(5 / portTICK_PERIOD_MS);
    //     }
    // }

}

void LF(){

            ESP_LOGI("debug", "LF\n");
            forward_angle = read_pid_const2().setpoint + fwd_offset;

           
            // lsa_to_bar();

            left_duty_cycle = bound((read_pid_const2().optimum_duty_cycle - correction), read_pid_const2().lower_duty_cycle, read_pid_const2().higher_duty_cycle);
            right_duty_cycle = bound((read_pid_const2().optimum_duty_cycle + correction), read_pid_const2().lower_duty_cycle, read_pid_const2().higher_duty_cycle);



            //Extra yaw correction during turns
            // ESP_LOGI("debug", "error %f", error);

            if(error>read_pid_const2().percent_lf)
            {
                ESP_LOGI("debug","error > 2.5 %f\n", error);

                right_duty_cycle-=6;
                left_duty_cycle+=6;   
            }
            else if(error<-read_pid_const2().percent_lf)
            {
                // printf("hello2\n");
                ESP_LOGI("debug","error < -4.5 %f\n", error);

                left_duty_cycle-=6;
                right_duty_cycle+=6;
            }

            // if(line_sensor_readings.adc_reading[0]>600 && line_sensor_readings.adc_reading[4]<450){
            //     right_duty_cycle+=6;
            //     left_duty_cycle-=6;
            // }
            // else if(line_sensor_readings.adc_reading[4]>600 && line_sensor_readings.adc_reading[0]<450){
            //     right_duty_cycle-=6;
            //     left_duty_cycle+=6;
            // }

            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);

            // calculate_motor_command(pitch_error, &motor_cmd);

            read_mpu6050(euler_angle, mpu_offset);
            pitch_cmd = read_pid_const2().setpoint;
            pitch_angle = euler_angle[1];
            pitch_error = pitch_cmd - pitch_angle;
            // ESP_LOGI("debug", "pitchang : %0.2f ", euler_angle[1]);

            // if (pitch_error > 0)
            // {
            //     set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            //     set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
            // }
            // if (pitch_error < 0)
            // {
            //     set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle);
            //     set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle);
            // }
            // else
            // {
            //     set_motor_speed(MOTOR_A_0, MOTOR_STOP, 0);
            //     set_motor_speed(MOTOR_A_1, MOTOR_STOP, 0);
            // }

            if (pitch_error > read_pid_const2().breakposi || pitch_error < -read_pid_const2().breakneg){
                pitch_cmd = read_pid_const2().setpoint;
                balanced = false;
            }

            // ESP_LOGI("debug", "kp:  %f    ::  kp2 :  %f  :: setpoint :  %f  optimumduty  :  %d  \n",read_pid_const().kp, read_pid_const2().kp2, read_pid_const2().setpoint, read_pid_const2().optimum_duty_cycle);
            // ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
}

void app_main()
{
    xTaskCreate(&self_and_line, "self_and_line", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}
