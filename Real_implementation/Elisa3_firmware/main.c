#include <avr\io.h>
#include <stdbool.h>
#include <avr\interrupt.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>
#include "variables.h"
#include "utility.h"
#include "speed_control.h"
#include "nRF24L01.h"
#include "behaviors.h"
#include "sensors.h"
#include "irCommunication.h"
#include "motors.h"
#define PI 3.141592654

unsigned long int start_state0 = 0;
unsigned long int start_state1 = 0; 
bool angle_state = 0;

typedef enum{
	idle,
	initialize,
	angle,
	straight,
    terminate,
    compute_sum,
    compute_formation,
	wait
} state;

bool angle_control(float *u, bool check, float threshold, float theta_cur){

    float angle_ref = 0.0;
	float theta_temp;
    float speed_a = 0;
    float K = 0.5;
    float error = 0.0;
    bool out;

    angle_ref = (float) atan2f(u[1], u[0]) * RAD_2_DEG;

	theta_temp = theta_cur* RAD_2_DEG; //theta = filtered_theta from 0 to 360
	if (theta_temp > 180){
		theta_temp -= 360;
	}

    error = angle_ref - theta_temp;
	//angle_temp = error;
    //error = 0;
	
    if (error > 180){
        error -= 360;
    } else if (error < -180) {
        error += 360;
    }

    speed_a = K*error;


    // limit speed
    if (speed_a > 15){
        speed_a = 15;
    }else if (speed_a < -15){
        speed_a = -15;
    }

	r_speed = speed_a;
	l_speed = -speed_a;

    if (fabsf(error) > 25){
        if (check){			
//			switch (angle_state){
				
//				case 0:
//					setLeftSpeed(0);
//					setRightSpeed(0);
					
//					if (getTime100MicroSec()-start_state0 > PAUSE_250_MSEC){
//						angle_state = 1;
//						start_state1 = getTime100MicroSec();
//					}
					
//					break;
//				case 1:
					setLeftSpeed(l_speed);
					setRightSpeed(r_speed);
					
//					if (getTime100MicroSec()-start_state1 > PAUSE_2_SEC){
//						angle_state = 1;
//						start_state0 = getTime100MicroSec();
//					}
					
//					break;
//			}
        }
        out = false;
    }else{
        if (check){
            //setLeftSpeed( 0);
            //setRightSpeed( 0);
        }
		angle_state = 0;
		start_state0 = getTime100MicroSec();
        out = true;
    }

    return out;
}


float *etc_control(float (*x_hat)[2], int id){

    static float u[2];

    u[0] = 0;
    u[1] = 0;

    for(int i=0;i<ROBOTS;i++){
        u[0] += -L[i]*(x_hat[i][0])+Bx[i];
        u[1] += -L[i]*(x_hat[i][1])+By[i];
    }

    return u;

}

bool trigger_etc(float *x_hat, const float *x){

    float t = getTime100MicroSec()/10000; //to seconds
    float f = 0.0;

    float c0 = 10;
    float c1 = 250;
    float alpha = 0.5*lambda_2;

    f = sqrt(pow((x_hat[0]-x[0]),2.0) + pow((x_hat[1]-x[1]),2.0)) - (c0+c1*exp(-alpha*(t)));

    return f> 0;
}

bool trigger_petc(float *fp, float (*x_hat)[2], const float *x, int id){

    float f = 0.0;
    float sigma = 0.01;
    static float u[2];
    float u_petc= 0;

    u[0] = 0;
    u[1] = 0;

    for(int i=0;i<ROBOTS;i++){
        u[0] += L[i]*(x_hat[i][0]);
        u[1] += L[i]*(x_hat[i][1]);
    }

    if( fabs(u[0]) > fabs(u[1])){
        u_petc = fabs(u[0]);
    }else{
        u_petc = fabs(u[1]);
    }

    f = sqrt(pow((fp[0]-x[0]),2.0) + pow((fp[1]-x[1]),2.0)) - sigma*u_petc;

    return f> 0;
}

float *da_control(const float (*x), float *x_ref, int id){

    static float u[2];
    float K_ = 50;

    u[0] = -K_*(x[0]-x_ref[0]);
    u[1] = -K_*(x[1]-x_ref[1]);

    //printf("u %d: %f, %f\n", id, u[0], u[1]);

//    for (int i=0; i<2;i++){
//        if (u[i] > -u_max){
//            u[i] = -u_max;
//        }else if (u[i] > u_max){
//            u[i] = u_max;
//        }
//    }
    return u;

}

//void slowStop(){
	
//	if (speed == 0 & r_speed > 0 & l_speed > 0){
//		r_speed = r_speed -2;
//		l_speed = l_speed -2;		
//	}
	
//}

//void slowStart(){
//	if (speed > 0 & r_speed < speed & l_speed < speed){
//		r_speed = r_speed +2;
//		l_speed = l_speed +2;
//	}
//}

int main(void) {

    static state NextState = idle;
	unsigned long int startTime = 0, endTime = 0, turnOffLedsTime = 0;
	unsigned char prevSelector=0;
	unsigned int i=0;
	unsigned int currRand=0, currRand2=0;
	//float speed = 0;
    float x_ref[2] = {0.0,0.0};
    float sum[2] = {0.0,0.0};
    float distance =0, smallest = 10000.0;
    int id_formation[100] = {ROBOTS+1};
    float x_refs[2] = {0, 200};
    float x_hat[100][2] = {0};
    float theta_ref = 0.0;
	float theta_acc = 0.0;
	float targetAngle=0;
    float *u;
    unsigned long int trigger_LED_timer = 0;
    bool trigger_flag = false;
	bool init_trigger = false;
	int Ni = 1;
	int com = 0;
	int D = 1;
	unsigned int rec[ROBOTS];
	unsigned int neigh[Ni];
	unsigned long int loop_time_previous = 0;
	unsigned long int comm_time_previous = 0;
	unsigned long int control_time_previous = 0;
	unsigned long int init_time_previous = 0;
	unsigned long int wait_time = 0;
	unsigned long int wait_time_previous = 0;
	unsigned long int straight_time = 0;
	unsigned long int straight_time_previous = 0;
	
	initPeripherals();

	initBehaviors();

	speedStepCounter = getTime100MicroSec();
	
	// I noticed that I have to wait a little before calibrating in order to have the sensors to be 
	// well calibrated (sensors noise eliminated). Don't sure why, maybe due to the sensitivity of the 
	// sensor that stabilizes...
	startTime = getTime100MicroSec();
	while((getTime100MicroSec() - startTime) < PAUSE_300_MSEC);
	calibrateSensors();

	startTime = getTime100MicroSec();
    resetOdometry();
	
	

    demoStartTime = getTime100MicroSec();
    unsigned long int period = getTime100MicroSec();
	
	while(1) {
		// turn on all IRs
		LED_IR1_LOW;
		LED_IR2_LOW;
		
		loop_time = getTime100MicroSec() - loop_time_previous;
		loop_time_previous = getTime100MicroSec();
		
		currentSelector = getSelector();	// update selector position

		control_time_previous = getTime100MicroSec();
		readAccelXYZ();						// update accelerometer values to compute the angle

		computeAngle();
		control_time = getTime100MicroSec() - control_time_previous;

		endTime = getTime100MicroSec();
		if((endTime-startTime) >= (PAUSE_2_SEC)) {
			readBatteryLevel();				// the battery level is updated every two seconds
    		startTime = getTime100MicroSec();
		}

        handleIRRemoteCommands();
		
		if ((NextState == straight || NextState == idle)){
			turn = false;
		}else{
			turn = true;
		}

		//comm_time_previous = getTime100MicroSec();
		reset_flag = false;
        if(handleRFCommands()){		
			for (int i = 0; i<ROBOTS; i++){
				x_hat[i][0] = Xpos_array[i];
				x_hat[i][1] = Ypos_array[i];
			}
		}
		
		if (turn){
			filtered_theta = theta;
		}
		//comm_time = getTime100MicroSec() - comm_time_previous;
		
		switch(currentSelector) {
			
			case 0: // just green color
					updateRedLed(255);
					updateGreenLed(0);
					updateBlueLed(255);
					setLeftSpeed(0);
					setRightSpeed(0);
					break;
			case 1:	// motors in direct power control (no speed control)
					//updateRedLed(pwm_red);
					//updateGreenLed(pwm_green);
					//updateBlueLed(pwm_blue);
					enableObstacleAvoidance();
					enableCliffAvoidance();
					
					u[0] = 0;
					u[1] = 100;
				
					updateRedLed(255); //255
					updateGreenLed(0);
					updateBlueLed(255); //255
											
					if (angle_control(u, true, 0, theta)){
						setLeftSpeed(50);
						setRightSpeed(50);
						updateRedLed(255); //255
						updateGreenLed(255);
						updateBlueLed(0); //255
					}

					break;

			case 2:	// motors calibration
					irEnabled = 1;
					if(calibrateOdomFlag==1) {
						handleCalibration();
					}
					break;

			case 3:	// write default calibration values; wait 2 seconds before start writing the calibration values
					// in eeprom in order to avoid rewriting the data involuntarily when moving the selector and passing 
					// through selector position 9
					switch(demoState) {
						case 0:
							demoStartTime = getTime100MicroSec();
							demoState = 1;
							break;

						case 1:
							if((getTime100MicroSec()-demoStartTime) >= (PAUSE_2_SEC)) {
								demoState = 2;
							}
							break;						

						case 2:
							if(!calibrationWritten) {
								calibrationWritten = 1;
								writeDefaultCalibration();
							}							
							break;
					}
					break;

            case 12:
                // PETC

                 enableObstacleAvoidance();

                 //control_time_previous = getTime100MicroSec();
                 u = etc_control(x_hat, id);
                 //control_time = getTime100MicroSec() - control_time_previous;
                 
                 //control_time_previous = getTime100MicroSec();
                 switch (NextState) {

	                 case idle: {
		                 // automatically move to next state -- eventually some kind of signal
		                 updateRedLed(255);
		                 updateGreenLed(0); //0
		                 updateBlueLed(255);
		                 setLeftSpeed(0);
		                 setRightSpeed(0);

		                 if(start_comm){
			                 if ((getTime100MicroSec() - demoStartTime) >= (PAUSE_2_SEC)) {
				                 NextState = angle;
				                 init_trigger = true;
			                 }
		                 }
	                 }break;

	                 case straight: {

		                 if (!trigger_flag) {
			                 updateRedLed(255);
			                 updateGreenLed(255);
			                 updateBlueLed(0); //0
		                 }

		                 speed = K * sqrt(pow(u[0], 2) + pow(u[1], 2));

		                 if (speed > u_max) {
			                 speed = u_max;
		                 }
		                 l_speed = speed;
		                 r_speed = speed;

		                 setLeftSpeed(speed);
		                 setRightSpeed(speed-offset_rwheel);
		                 
		                 //comm_time_previous = getTime100MicroSec();
		                 if (!(angle_control(u, false, 5, filtered_theta))){
			                 NextState = angle;
			                 theta = filtered_theta;
		                 }
		                 //comm_time = getTime100MicroSec() - comm_time_previous;
		                 //                        theta_ref = atan2(disY, disX);
	                 }break;

	                 case angle:{

		                 if (!trigger_flag) {
			                 updateRedLed(0); //0
			                 updateGreenLed(255);
			                 updateBlueLed(255);
		                 }

		                 //comm_time_previous = getTime100MicroSec();
		                 if (angle_control(u, true, 5, theta)){
			                 NextState = wait;
			                 wait_time_previous = getTime100MicroSec();

			                 setLeftSpeed(10);
			                 setRightSpeed(10);
		                 }
		                 //comm_time = getTime100MicroSec() - comm_time_previous;

	                 }break;
	                 
	                 case wait:{
		                 
		                 setLeftSpeed(0);
		                 setRightSpeed(0);
		                 
		                 wait_time = getTime100MicroSec() - wait_time_previous;
		                 
		                 if (wait_time > PAUSE_750_MSEC){
			                 NextState = straight;
		                 }
		                 
	                 }break;

	                 case terminate:{
		                 //setLeftSpeed(0);
		                 //setRightSpeed(0);

		                 updateRedLed(0); //0
		                 updateGreenLed(0); //0
		                 updateBlueLed(255);

	                 }break;

                 }
				//control_time = getTime100MicroSec() - control_time_previous;
                //if (fabsf(x_hat[0][0]-x_hat[1][0]) < error_radius && fabsf(x_hat[0][1]-x_hat[1][1]) < error_radius) {
				//	if(!(NextState == idle)){
				//		NextState = terminate;
				//	}
                //}

				init_time_previous = getTime100MicroSec();
                if(getTime100MicroSec()-period >= h | init_trigger) {
                    trigger_flag = false;
                    if (trigger_petc(fixed_position, x_hat, position, id)) {
                        for (i = 0; i < 2; i++) {
                            fixed_position[i] = position[i];
                            //x_hat[id][i] = position[i];
                        }
						xPos_fixed = position[0];
						yPos_fixed = position[1];
						if (reset_flag){
							count_resets++;
						}
						trigger_count++;
                        updateRedLed(255);
                        updateGreenLed(0); //0
                        updateBlueLed(0); //0
                        trigger_flag = true;
						init_trigger = false;
                        trigger_LED_timer = getTime100MicroSec();
                    }
                    period = getTime100MicroSec();
                }
				init_time = getTime100MicroSec() - init_time_previous;

                // fix LED for triggering
                if(getTime100MicroSec()-trigger_LED_timer >= PAUSE_250_MSEC && trigger_flag) {
                    trigger_flag = false;
                } else if (trigger_flag){
                    updateRedLed(255); 
                    updateGreenLed(0); //0
                    updateBlueLed(0); //0
                }

                break;
					
			case 13:
                // ETC

                enableObstacleAvoidance();

				//control_time_previous = getTime100MicroSec();
                u = etc_control(x_hat, id);
				//control_time = getTime100MicroSec() - control_time_previous;
 
				//control_time_previous = getTime100MicroSec();
                switch (NextState) {

                    case idle: {
                        // automatically move to next state -- eventually some kind of signal
                        updateRedLed(255);
                        updateGreenLed(0); //0
                        updateBlueLed(255);
                        setLeftSpeed(0);
                        setRightSpeed(0);

						if(start_comm){
							if ((getTime100MicroSec() - demoStartTime) >= (PAUSE_2_SEC)) {
	                            NextState = angle;
								init_trigger = true;
							}
						}
                    }break;

                    case straight: {

                        if (!trigger_flag) {
                            updateRedLed(255);
                            updateGreenLed(255);
                            updateBlueLed(0); //0
                        }

                        speed = K * sqrt(pow(u[0], 2) + pow(u[1], 2));

                        if (speed > u_max) {
                            speed = u_max;
                        }
						l_speed = speed;
						r_speed = speed;

                        setLeftSpeed(speed);
                        setRightSpeed(speed-offset_rwheel);
						
						//comm_time_previous = getTime100MicroSec();
						if (!(angle_control(u, false, 5, filtered_theta))){
							NextState = angle;
							theta = filtered_theta;
						}
						//comm_time = getTime100MicroSec() - comm_time_previous;
//                        theta_ref = atan2(disY, disX);
                    }break;

                    case angle:{

                        if (!trigger_flag) {
                            updateRedLed(0); //0
                            updateGreenLed(255);
                            updateBlueLed(255);
                        }

						//comm_time_previous = getTime100MicroSec();
                        if (angle_control(u, true, 5, theta)){
                            NextState = wait;
							wait_time_previous = getTime100MicroSec();

                            setLeftSpeed(10);
                            setRightSpeed(10);
                        }
						//comm_time = getTime100MicroSec() - comm_time_previous;

                    }break;
					
					case wait:{
						
						setLeftSpeed(0);
						setRightSpeed(0);
						
						wait_time = getTime100MicroSec() - wait_time_previous;
						
						if (wait_time > PAUSE_750_MSEC){
							NextState = straight;
						}
						
					}break;					

                    case terminate:{
                        //setLeftSpeed(0);
                        //setRightSpeed(0);

                        updateRedLed(0); //0
                        updateGreenLed(0); //0
                        updateBlueLed(255);

                    }break;

                }
				//control_time = getTime100MicroSec() - control_time_previous;

				init_time_previous = getTime100MicroSec();
                if(trigger_etc(fixed_position, position) | init_trigger) {
                    for (i = 0; i < 2; i++) {
                        fixed_position[i] = position[i];
                        
                    }
					xPos_fixed = position[0];
					yPos_fixed = position[1];
					if (reset_flag){
						count_resets++;
					}
					trigger_count++;
                    updateRedLed(255);
                    updateGreenLed(255); //0
                    updateBlueLed(255); //0
                    trigger_flag = true;
					init_trigger = false;
                    trigger_LED_timer = getTime100MicroSec();
                }
				init_time = getTime100MicroSec() - init_time_previous;
				
                // fix LED for triggering
                if(getTime100MicroSec()-trigger_LED_timer >= PAUSE_250_MSEC && trigger_flag) {
                    trigger_flag = false;
                } else if (trigger_flag){
                    updateRedLed(255);
                    updateGreenLed(0);//0
                    updateBlueLed(0); //0
                }

                break;
					
			case 14:
			// Move in straight line toward goal -- Phase algorithm
				enableObstacleAvoidance();
				
				// In order to visualise what is happening
				xPos_fixed = position[0];
				yPos_fixed = position[1];

				// controller
				//control_time_previous = getTime100MicroSec();
                u = da_control(position, x_ref, id);
				//control_time = getTime100MicroSec() - control_time_previous;

				//control_time_previous = getTime100MicroSec();
                switch (NextState){


                    case idle:{
                        // automatically move to next state -- eventually some kind of signal
                        updateRedLed(255); //
                        updateGreenLed(0); //0
                        updateBlueLed(255);
                        setLeftSpeed(0);
                        setRightSpeed(0);

						if (start_comm){
							if((getTime100MicroSec()-demoStartTime) >= (PAUSE_2_SEC)) {
								NextState = compute_sum;
								init_time_previous = getTime100MicroSec();
							}
						}
                    }break;

                    case compute_sum:

                        // compute average position
                        for (i=0; i<ROBOTS; i++){
                            sum[0] += x_hat[i][0];
                            sum[1] += x_hat[i][1];
                        }

                        x_ref[0] = sum[0]/(ROBOTS);
                        x_ref[1] = sum[1]/(ROBOTS);

                        NextState = compute_formation;

                        break;
                    case compute_formation:

                        // compute formation array
                        for(int j=0; j < ROBOTS; j++){
                            for(i=0; i<ROBOTS; i++){
                                //loop over agents per formation position
                                bool check = true;
                                for (int n=0; n<j;n++){

                                    if(id_formation[n] == i){
                                        check = false;
                                    }

                                }

                                if (check){
                                    distance = sqrt(pow(x_ref[0]+B[0][j]-x_hat[i][0],2.0) + pow(x_ref[1]+B[1][j]-x_hat[i][1],2.0));

                                    if(distance +0.05 < smallest){
                                        // compensate for some noise with 0.05
                                        smallest = distance;
                                        id_formation[j] = i;
                                    }

                                }
                            }

                            smallest = 10000;
                        }


                        int correct_id = 0;
                        for(int n=0;n<ROBOTS;n++){
                            if(id_formation[n] == id){
                                correct_id = n;
                            }
                        }

                        x_ref[0] += B[0][id];
                        x_ref[1] += B[1][id];
						
						//x_ref[0] = 895;
						//x_ref[1] = 1121;

                        NextState = angle;
						init_time = getTime100MicroSec() - init_time_previous;
                        break;
						
                    case straight: {

                        updateRedLed(255);
                        updateGreenLed(255);
                        updateBlueLed(0); //0

                        speed = K * sqrt(pow(u[0], 2) + pow(u[1], 2));

                        if (speed > u_max) {
                            speed = u_max;
                        }
						l_speed = speed;
						r_speed = speed;

                        setLeftSpeed(speed);
                        setRightSpeed(speed-offset_rwheel);

						straight_time = getTime100MicroSec() - straight_time_previous;
						//comm_time_previous = getTime100MicroSec();
						//if (straight_time > PAUSE_500_MSEC){
                        if (!(angle_control(u, false, 5, filtered_theta))){
                            NextState = angle;
							theta = filtered_theta;
                        }
						//}
						//comm_time = getTime100MicroSec() - comm_time_previous;

                        if (fabsf(position[0]-x_ref[0]) < error_radius && fabsf(position[1]-x_ref[1]) < error_radius){
                            NextState = terminate;
                        }

                    }break;
					
                    case angle:{

                        if (!trigger_flag) {
                            updateRedLed(0); //0
                            updateGreenLed(255);
                            updateBlueLed(255);
                        }

						//comm_time_previous = getTime100MicroSec();
                        if (angle_control(u, true, 5, theta)){
                            NextState = wait;
							wait_time_previous = getTime100MicroSec();

                            setLeftSpeed(0);
                            setRightSpeed(0);
                        }
						//comm_time = getTime100MicroSec() - comm_time_previous;

                        if (fabsf(position[0]-x_ref[0]) < error_radius && fabsf(position[1]-x_ref[1]) < error_radius){
                            NextState = terminate;

                        }

                    }break;
					
					case wait:{
						
						setLeftSpeed(0);
						setRightSpeed(0);
					
						wait_time = getTime100MicroSec() - wait_time_previous;
						
						if (wait_time > PAUSE_750_MSEC){
							NextState = straight;
							straight_time_previous = getTime100MicroSec();
						}
						
					}break;	

                    case terminate:{
                        setLeftSpeed(0);
                        setRightSpeed(0);

                        updateRedLed(255);
                        updateGreenLed(0); //0
                        updateBlueLed(255);

                    }break;
					//control_time = getTime100MicroSec() - control_time_previous;
                }
				
				break;

			case 15:// clock calibration
					//usart0Transmit(irCommand,1);
					//currentOsccal = OSCCAL;
					//usart0Transmit(currentOsccal,1);
					break;
		}

		comm_time_previous = getTime100MicroSec();
		handleMotorsWithSpeedController();
        //position[0] = xPos;
        //position[1] = yPos; //used to be yPos and xPos
		comm_time = getTime100MicroSec()- comm_time_previous;
		//handleMotorsWithNoController(); 
		

		if(prevSelector != currentSelector) {	// in case the selector is changed, reset the robot state
			disableObstacleAvoidance();
			disableCliffAvoidance();
			GREEN_LED0_OFF;
			GREEN_LED1_OFF;
			GREEN_LED2_OFF;
			GREEN_LED3_OFF;
			GREEN_LED4_OFF;
			GREEN_LED5_OFF;
			GREEN_LED6_OFF;
			GREEN_LED7_OFF;
			pwm_red = 255;
			pwm_green = 255;
			pwm_blue = 255;
			updateRedLed(pwm_red);
			updateGreenLed(pwm_green);
			updateBlueLed(pwm_blue);
			setRightSpeed(0);
			setLeftSpeed(0);
			rgbState = 0;
			calibrationWritten = 0;
			demoState = 0;
            demoStartTime = getTime100MicroSec();
			irCommState = 0;
			trigger_count = 0;
			trigger_flag = false;
		}
		prevSelector = currentSelector;


	} // while(1)

}
