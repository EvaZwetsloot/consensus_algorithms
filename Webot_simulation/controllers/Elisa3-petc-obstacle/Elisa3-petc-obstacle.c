#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/accelerometer.h>
#include <webots/light_sensor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>
#include <webots/motor.h>
#include <webots/gps.h>
#include <math.h>


#define ROBOTS 16
#define D 2
#define LEDS_NUMBER 8
#define pi 3.142857
#define NOISE_THR 50

int L[ROBOTS][ROBOTS] = {{4, -1, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0}, {-1, 4, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0}, {-1, 0, 4, -1, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0}, {0, -1, -1, 4, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0}, {-1, 0, 0, 0, 4, -1, -1, 0, 0, 0, 0, 0, -1, 0, 0, 0}, {0, -1, 0, 0, -1, 4, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0}, {0, 0, -1, 0, -1, 0, 4, -1, 0, 0, 0, 0, 0, 0, -1, 0}, {0, 0, 0, -1, 0, -1, -1, 4, 0, 0, 0, 0, 0, 0, 0, -1}, {-1, 0, 0, 0, 0, 0, 0, 0, 4, -1, -1, 0, -1, 0, 0, 0}, {0, -1, 0, 0, 0, 0, 0, 0, -1, 4, 0, -1, 0, -1, 0, 0}, {0, 0, -1, 0, 0, 0, 0, 0, -1, 0, 4, -1, 0, 0, -1, 0}, {0, 0, 0, -1, 0, 0, 0, 0, 0, -1, -1, 4, 0, 0, 0, -1}, {0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 4, -1, -1, 0}, {0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, -1, 4, 0, -1}, {0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, 0, 4, -1}, {0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, -1, 4}};
float Bx[ROBOTS][ROBOTS] = {{0.000, 0.038, 0.146, 0.000, 0.500, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000}, {-0.038, 0.000, 0.000, 0.271, 0.000, 0.653, 0.000, 0.000, 0.000, 0.924, 0.000, 0.000, 0.000, 0.000, 0.000, -0.000}, {-0.146, -0.000, 0.000, 0.162, 0.000, 0.000, 0.707, 0.000, 0.000, 0.000, 0.707, 0.000, 0.000, 0.000, 0.000, -0.000}, {-0.000, -0.271, -0.162, 0.000, 0.000, 0.000, 0.000, 0.653, 0.000, 0.000, 0.000, 0.383, 0.000, -0.000, -0.000, -0.000}, {-0.500, -0.000, -0.000, -0.000, 0.000, 0.191, 0.354, 0.000, 0.000, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000}, {-0.000, -0.653, -0.000, -0.000, -0.191, 0.000, 0.000, 0.271, 0.000, 0.000, 0.000, 0.000, -0.000, -0.383, -0.000, -0.000}, {-0.000, -0.000, -0.707, -0.000, -0.354, -0.000, 0.000, 0.108, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.707, -0.000}, {-0.000, -0.000, -0.000, -0.653, -0.000, -0.271, -0.108, 0.000, 0.000, 0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.924}, {-1.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.000, 0.000, -0.038, -0.146, -0.000, -0.500, -0.000, -0.000, -0.000}, {-0.000, -0.924, -0.000, -0.000, -0.000, -0.000, -0.000, 0.000, 0.038, 0.000, -0.000, -0.271, -0.000, -0.653, -0.000, -0.000}, {-0.000, -0.000, -0.707, -0.000, -0.000, -0.000, -0.000, 0.000, 0.146, 0.000, 0.000, -0.162, -0.000, -0.000, -0.707, -0.000}, {-0.000, -0.000, -0.000, -0.383, -0.000, -0.000, 0.000, 0.000, 0.000, 0.271, 0.162, 0.000, -0.000, -0.000, -0.000, -0.653}, {-0.000, -0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.500, 0.000, 0.000, 0.000, 0.000, -0.191, -0.354, -0.000}, {-0.000, -0.000, -0.000, 0.000, 0.000, 0.383, 0.000, 0.000, 0.000, 0.653, 0.000, 0.000, 0.191, 0.000, -0.000, -0.271}, {-0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.707, 0.000, 0.000, 0.000, 0.707, 0.000, 0.354, 0.000, 0.000, -0.108}, {-0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.924, 0.000, 0.000, 0.000, 0.653, 0.000, 0.271, 0.108, 0.000}};
float By[ROBOTS][ROBOTS] = {{0.000, -0.191, -0.354, -0.000, -0.500, -0.000, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000}, {0.191, 0.000, -0.000, -0.271, -0.000, -0.271, -0.000, -0.000, 0.000, 0.383, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000}, {0.354, 0.000, 0.000, -0.108, -0.000, -0.000, 0.000, 0.000, 0.000, 0.000, 0.707, 0.000, 0.000, 0.000, 0.000, 0.000}, {0.000, 0.271, 0.108, 0.000, -0.000, -0.000, 0.000, 0.271, 0.000, 0.000, 0.000, 0.924, 0.000, 0.000, 0.000, 0.000}, {0.500, 0.000, 0.000, 0.000, 0.000, 0.038, 0.146, 0.000, 0.000, 0.000, 0.000, 0.000, 1.000, 0.000, 0.000, 0.000}, {0.000, 0.271, 0.000, 0.000, -0.038, 0.000, 0.000, 0.271, 0.000, 0.000, 0.000, 0.000, 0.000, 0.924, 0.000, 0.000}, {0.000, 0.000, 0.000, -0.000, -0.146, -0.000, 0.000, 0.162, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.707, 0.000}, {0.000, 0.000, -0.000, -0.271, -0.000, -0.271, -0.162, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.000, 0.383}, {-0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.000, 0.000, 0.191, 0.354, 0.000, 0.500, 0.000, 0.000, 0.000}, {-0.000, -0.383, -0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.191, 0.000, 0.000, 0.271, 0.000, 0.271, 0.000, -0.000}, {-0.000, -0.000, -0.707, -0.000, -0.000, -0.000, -0.000, -0.000, -0.354, -0.000, 0.000, 0.108, 0.000, 0.000, 0.000, -0.000}, {-0.000, -0.000, -0.000, -0.924, -0.000, -0.000, -0.000, -0.000, -0.000, -0.271, -0.108, 0.000, 0.000, 0.000, -0.000, -0.271}, {-0.000, -0.000, -0.000, -0.000, -1.000, -0.000, -0.000, -0.000, -0.500, -0.000, -0.000, -0.000, 0.000, -0.038, -0.146, -0.000}, {-0.000, -0.000, -0.000, -0.000, -0.000, -0.924, -0.000, -0.000, -0.000, -0.271, -0.000, 0.000, 0.038, 0.000, -0.000, -0.271}, {-0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.707, -0.000, -0.000, -0.000, -0.000, 0.000, 0.146, 0.000, 0.000, -0.162}, {-0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.000, -0.383, -0.000, 0.000, 0.000, 0.271, 0.000, 0.271, 0.162, 0.000}};
int K = 5;
double h = 0.004;
double delay = 0.1;

typedef enum{
  idle,
  obstacle,
  angle_control,
  straight,
  obstacle_straight,
} state;


static WbDeviceTag leds[LEDS_NUMBER];
static const char *leds_names[LEDS_NUMBER] = {
  "led0", "led1", "led2", "led3",
  "led4", "led5", "led6", "led7",
};
WbDeviceTag ledrgb;
WbDeviceTag distance_sensor[8];
WbDeviceTag distance_ambient_sensor[8];
WbDeviceTag ground_sensor[4];
WbDeviceTag ground_ambient_sensor[4];
WbDeviceTag receiver;
WbDeviceTag emitter;
WbDeviceTag gps;
WbDeviceTag accelerometer;
WbDeviceTag left_motor, right_motor;
int time_step;
char ackPayload[81] = {0};
double disX =0, disY = 0, X=0;
double distanceY = 0, distanceX = 0;
static state NextState = idle;
bool side_obstacle_detected = false;
int data_size = 16384;

void prep_msg(int *prox_value, int *ground_value, const double *accValue, int *prox_ambient_value, int *ground_ambient_value, const double *position, bool next, bool trigger, int trigger_counter, double u_max){

  ackPayload[0] = wb_emitter_get_channel(emitter);
  ackPayload[1] = 3; // first packet
  ackPayload[2] = prox_value[0]&0xFF;
  ackPayload[3] = prox_value[0]>>8;
  ackPayload[4] = prox_value[1]&0xFF;
  ackPayload[5] = prox_value[1]>>8;
  ackPayload[6] = prox_value[2]&0xFF;
  ackPayload[7] = prox_value[2]>>8;
  ackPayload[8] = prox_value[3]&0xFF;
  ackPayload[9] = prox_value[3]>>8;
  ackPayload[10] = prox_value[5]&0xFF;
  ackPayload[11] = prox_value[5]>>8;
  ackPayload[12] = prox_value[6]&0xFF;
  ackPayload[13] = prox_value[6]>>8;
  ackPayload[14] = prox_value[7]&0xFF;
  ackPayload[15] = prox_value[7]>>8;
  ackPayload[16] = 0;
  ackPayload[17] = 4; // second packet
  ackPayload[18] = prox_value[4]&0xFF;
  ackPayload[19] = prox_value[4]>>8;
  ackPayload[20] = ground_value[0]&0xFF;
  ackPayload[21] = ground_value[0]>>8;
  ackPayload[22] = ground_value[1]&0xFF;
  ackPayload[23] = ground_value[1]>>8;
  ackPayload[24] = ground_value[2]&0xFF;
  ackPayload[25] = ground_value[2]>>8;
  ackPayload[26] = ground_value[3]&0xFF;
  ackPayload[27] = ground_value[3]>>8;
  ackPayload[28] = ((int)accValue[0])&0xFF;
  ackPayload[29] = ((int)accValue[0])>>8;
  ackPayload[30] = ((int)accValue[1])&0xFF;
  ackPayload[31] = ((int)accValue[1])>>8;
  ackPayload[32] = 0; // not implemented (TV remote input) 
  ackPayload[33] = 5; // third packet
  ackPayload[34] = prox_ambient_value[0]&0xFF;
  ackPayload[35] = prox_ambient_value[0]>>8;
  ackPayload[36] = prox_ambient_value[1]&0xFF;
  ackPayload[37] = prox_ambient_value[1]>>8;
  ackPayload[38] = prox_ambient_value[2]&0xFF;
  ackPayload[39] = prox_ambient_value[2]>>8;
  ackPayload[40] = prox_ambient_value[3]&0xFF;
  ackPayload[41] = prox_ambient_value[3]>>8;
  ackPayload[42] = prox_ambient_value[5]&0xFF;
  ackPayload[43] = prox_ambient_value[5]>>8;
  ackPayload[44] = prox_ambient_value[6]&0xFF;
  ackPayload[45] = prox_ambient_value[6]>>8;
  ackPayload[46] = prox_ambient_value[7]&0xFF;
  ackPayload[47] = prox_ambient_value[7]>>8;
  ackPayload[48] = 0; // not implemented (selector)
  ackPayload[49] = 6; // fourth packet
  ackPayload[50] = prox_ambient_value[4]&0xFF;
  ackPayload[51] = prox_ambient_value[4]>>8;
  ackPayload[52] = ground_ambient_value[0]&0xFF;
  ackPayload[53] = ground_ambient_value[0]>>8;
  ackPayload[54] = ground_ambient_value[1]&0xFF;
  ackPayload[55] = ground_ambient_value[1]>>8;
  ackPayload[56] = ground_ambient_value[2]&0xFF;
  ackPayload[57] = ground_ambient_value[2]>>8;
  ackPayload[58] = ground_ambient_value[3]&0xFF;
  ackPayload[59] = ground_ambient_value[3]>>8;
  ackPayload[60] = ((int)accValue[2])&0xFF;
  ackPayload[61] = ((int)accValue[2])>>8;
  ackPayload[62] = 0; // not implemented (battery => 2 bytes)
  ackPayload[63] = 0;
  ackPayload[64] = 0; // free
  ackPayload[65] = 7; // fifth packet
  ackPayload[66] = 0; // not implemented (left steps => 4 bytes)
  ackPayload[67] = 0;
  ackPayload[68] = 0;
  ackPayload[69] = 0;
  ackPayload[70] = 0; // not implemented (right steps => 4 bytes)
  ackPayload[71] = ((int) (u_max*256))&0xFF;
  ackPayload[72] = ((int) (u_max*256))>>8;
  ackPayload[73] = trigger_counter&0xFF;;
  ackPayload[74] = trigger_counter>>8; // not implemented (theta => 2 bytes)
  ackPayload[75] = trigger;
  ackPayload[76] = ((int) floor(position[0]*data_size))&0xFF; // not implemented (xpos => 2 bytes) [11 fractional bits, 5 integer]
  ackPayload[77] = ((int) floor(position[0]*data_size))>>8;
  ackPayload[78] = ((int) floor(position[1]*data_size))&0xFF; // not implemented (ypos => 2 bytes)
  ackPayload[79] = ((int) floor(position[1]*data_size))>>8;
  ackPayload[80] = next; // free

}


static void step() {
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
}

static void init_devices() {
  int i;
  char device_name[4];
  char device_name2[6];
 
 
  for (i=0; i<LEDS_NUMBER; i++) {
    leds[i]=wb_robot_get_device(leds_names[i]);
  }
 
  ledrgb = wb_robot_get_device("ledrgb");
 
  accelerometer = wb_robot_get_device("accelerometer");
  wb_accelerometer_enable(accelerometer, time_step*4);
 
  for (i = 0; i < 8; i++) {
    sprintf(device_name, "ps%d", i);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],time_step);
    sprintf(device_name, "ls%d", i);
    distance_ambient_sensor[i] = wb_robot_get_device(device_name);
    wb_light_sensor_enable(distance_ambient_sensor[i],time_step);
        
  }

  for (i = 0; i < 4; i++) {
    sprintf(device_name, "fs%d", i);
    ground_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(ground_sensor[i],time_step);
    sprintf(device_name2, "lsfs%d", i);
    ground_ambient_sensor[i] = wb_robot_get_device(device_name2);
    wb_light_sensor_enable(ground_ambient_sensor[i],time_step*4);
        
  }
    
  receiver = wb_robot_get_device("receiver");
  wb_receiver_enable(receiver, time_step);
  
  emitter = wb_robot_get_device("emitter");  
  
  gps = wb_robot_get_device("gps"); 
  wb_gps_enable(gps, time_step);
      
  step();
  
}

double IIR_filter(double x_new, int i){

  double y[8] = {0};
  static double y_old[8] = {0};
  static double x_old[8] = {0}; 
  int c1 = 14, c2 = 16;

  y[i] = c1*y_old[i] + x_new + x_old[i];
  y[i] = y[i]/c2;

  
  y_old[i] = y[i];
  x_old[i] = x_new;

  return y[i];
}

bool check_for_obstacle(int id, int threshold, int side){

  static bool obstacle_detected = false;
  static double distance[8] = {0};
  static double threshold2 = 10;
  double temp_dis = 0, temp_dis2;
  int count = 0;
  int count_right = 0, count_left = 0;
  disX = 0;
  disY = 0;

  for (int i=0; i<8; i++){
  
      temp_dis = wb_distance_sensor_get_value(distance_sensor[i]);
      
      temp_dis2 = IIR_filter(temp_dis, i);
      
      if(temp_dis2 > NOISE_THR-threshold){
        distance[i] = temp_dis2;  
        
        if (i==0){ 
          obstacle_detected = true;  
          count++;
        }else if (i== 1 && side <=0){
          obstacle_detected = true; 
          count++;
        }else if (i== 7 && side >=0){
          obstacle_detected = true; 
          count++;
        }
        
      } else if(temp_dis2 > NOISE_THR-threshold2){
      
        if(i == 6 || i==2 || i==3 || i==5){
          side_obstacle_detected = !obstacle_detected;
          //distance[i] = temp_dis2;  
        }
      
      } else if ((i == 6 || i==5)){
        count_right++;
      } else if (i==2 || i==3 ){
        count_left++;
      }
      
      if (temp_dis2 < 5){
        distance[i] = 0;
      }
  
  }
  
  if (count == 0){
    obstacle_detected = false;
  }
  
  if (count_left == 2 && count_right == 2){
    side_obstacle_detected = false;
  }

  if(distance[1] > distance[7]){
    X = 1;
  }else {
    X = -1;
  }
  
  disX = -distance[0] - (distance[1]/2) - (distance[7]/2);
  disY = distance[2]/2 + (distance[1]/2) - distance[6]/2 - (distance[7]/2) + (distance[3]/2) - (distance[5]/2);    

  return obstacle_detected;
  
}

bool check_line_of_sight(double *u){

  if (((u[0] < 0 && disX<-20) || (u[0] > 0 && disX>20) ) && ((u[1] < 0 && disY<-20) || (u[1] > 0 && disY>20)) && (wb_distance_sensor_get_value(distance_sensor[0]) < 10)){
    return true;
  }else{ 
    return false;
  }
}


double *etc_control(double (*x_hat)[D], int (*L)[ROBOTS], int N, int id, int Ni){

  static double u[2];

  u[0] = 0;
  u[1] = 0;

  for(int i=0;i<ROBOTS;i++){
      u[0] += -L[id][i]*(x_hat[i][0])+Bx[id][i];
      u[1] += -L[id][i]*(x_hat[i][1])+By[id][i];
      
   }
   
  return u;

}


bool control_angle(double *u, const double *x, int id, double theta, bool print_text, double threshold, bool check){

  double lspeed= 0, rspeed =0, speed = 0;
  double error = 0;
  double theta_new = 0;
  double K = 0.3;
  bool out;
  
  theta_new = atan2(u[1], u[0]);  
  
  error = (theta_new-theta);
  
   if (error > pi){
      error = -2*pi+error;
   }else if (error < -pi){
      error = 2*pi+error;
   }
   
  error = error*57.3;
  
  if (print_text){
    printf("theta %d: %f, %f\n", id, theta_new, theta);
    printf("position %d: %f, %f\n", id, x[0], x[1]);
    printf("u %d: %f, %f\n", id, u[0], u[1]);
    printf("error %d: %f\n", id, error);
  }
  
  
  speed = K*error;
  
  
   if (speed > 15){
    speed = 15;
   }else if ( speed < -15){
    speed = -15;
   }
   
   lspeed = -speed;
   rspeed = speed;
  
    if (fabsf(error) > 15){  
      //printf("speed %d: %f, %f\n", id, lspeed, rspeed);
      if (check){
        wb_motor_set_velocity(left_motor, lspeed);
        wb_motor_set_velocity(right_motor, rspeed);
      }  
      out = false;
    }else{
      if (check){
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
      }
      out = true;
    }  

  return out;
}


bool trigger(double *fp, double (*x_hat)[D], const double *x, float t_angle, int (*L)[ROBOTS], int id, int N){

  double f = 0.0;
  float sigma = 0.04;  
  static double u[2];
  double u_max=0;

  u[0] = 0;
  u[1] = 0;

  for(int i=0;i<ROBOTS;i++){
      u[0] += L[id][i]*(x_hat[i][0]);
      u[1] += L[id][i]*(x_hat[i][1]);
      //printf("robot %d, x_hat[%d] is (%f, %f)\n", id, i, x_hat[i][0], x_hat[i][1]);
      //printf("L[%d][%d]: %d\n", id, i,  L[id][i]); 
      //printf("u %d: %f, %f\n", id, u[0], u[1]);
   }
   
   if( fabs(u[0]) > fabs(u[1])){
     u_max = fabs(u[0]);
   }else{
     u_max = fabs(u[1]);
   }

  
  f = sqrt(pow((fp[0]-x[0]),2.0) + pow((fp[1]-x[1]),2.0)) - sigma*u_max;
  
  return f> 0;
}

int main(int argc, char *argv[]) {

  /* define variables */
  long int currentColor = 0xff0000;
  char redValue=0, blueValue=0, greenValue=0;
  char *ptr;
  int prox_value[8];
  int prox_ambient_value[8];
  int ground_value[4];
  int ground_ambient_value[4];
  const double *accValue;
  const double *position;
  double disX_previous = 0, disY_previous = 0;
  int id;
  int N;
  int threshold = 0;
  int Ni = 0;
  int i = 0;
  float t_angle = 0;
  float t_total_angle = 0;
  double period = 0;
  double obstacle_time = 0;
  bool done = false;
  double u_max = 0;
  int trigger_counter =0;
  double t_trigger = 0;
  double start_move_straight = 0;

  //float theta = 0;
  double *u;
  double theta = 0;
  double original_angle = 0;
  bool next_step = false, next_step_check = false;
  bool first_time = true;
  bool next_step_sup = false;
  bool trigger_flag = true;

  
  //Initiate controller values from arguments
  id = strtol(argv[1], &ptr, 10);
  N = ROBOTS;
  double x_hat[ROBOTS][D];    

  /* initialize Webots */
  wb_robot_init();

  time_step = wb_robot_get_basic_time_step();

  /* get and enable devices */
  init_devices();
  
  /* get a handler to the motors and set target position to infinity (speed control). */
  left_motor = wb_robot_get_device("left wheel motor");
  right_motor = wb_robot_get_device("right wheel motor");
  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_velocity(right_motor, 0.0);
  
  step(); // to compute distance sensors values
  
  position = wb_gps_get_values(gps);
  double fixed_position[2] = {position[0], position[1]};
  double *fp;
  fp = fixed_position;
 

  /* main loop */
  while (!done) {   
 
    if(wb_receiver_get_queue_length(receiver) > 0) {
   
      const char *msg = wb_receiver_get_data(receiver);
      
      //Unpack x_hat
      int k = 0;
      for(i = 0; i < ROBOTS; i++){
          x_hat[i][0] = ((double) (msg[8+k]+(msg[9+k]<<8)))/data_size;
          x_hat[i][1] = ((double) (msg[10+k]+(msg[11+k]<<8)))/data_size;     
          
          //printf("robot %d, x_hat[%d] is (%f, %f)\n", id, i, x_hat[i][0], x_hat[i][1]);
          k= k+4;
      }
      //Unpack theta
      theta = ((double) (msg[6]+(msg[7]<<8)))/4069;
      
      next_step_sup = msg[4];
      done = msg[3];
      
      
      }   
      currentColor = redValue*65536 + greenValue*256 + blueValue;
      wb_led_set(ledrgb, currentColor); // 0x0000ff);
     
      u = etc_control(x_hat, L, N, id, Ni);      
          
   switch (NextState){
    
      case idle:{
        if (wb_robot_get_time() > 0.4){ 
            NextState = angle_control;
        }
         // get prox values
        for (i = 0; i < 8; i++) {
          prox_value[i] = wb_distance_sensor_get_value(distance_sensor[i]);
          prox_ambient_value[i] = wb_light_sensor_get_value(distance_ambient_sensor[i]);      
        }
        // get ground values
        for (i = 0; i < 4; i++) {
          ground_value[i] = wb_distance_sensor_get_value(ground_sensor[i]);
          ground_ambient_value[i] = wb_light_sensor_get_value(ground_ambient_sensor[i]);
        }
        accValue = wb_accelerometer_get_values(accelerometer);
     
      }break;
    
      case angle_control:{ 
      
            if (first_time){
              t_angle = wb_robot_get_time();
              first_time = false;
              next_step_check = false;
            }
            
            //printf("Robot %d, in angle control, time %f\n", id, t_angle);
          
            redValue = 100;
            blueValue = 0; 
            greenValue = 0;
            // next_step_check = false;
            
            if (control_angle(u, position, id, theta, false, 0.1, true) && !next_step_check){     
                next_step_check = true;
            }
            
            if(next_step_check){
                NextState = straight;
                t_total_angle += wb_robot_get_time()-t_angle;                    
                t_angle = 0;
                first_time = true;
            }                    

      } break;
      case straight:{
      
            redValue = 0;
            blueValue = 100; 
            greenValue = 0;
            
            //printf("check %f\n", 10*sqrt(pow(u[0],2.0)+pow(u[1],2.0)));
            wb_motor_set_velocity(left_motor, K*sqrt(pow(u[0],2.0)+pow(u[1],2.0)));
            wb_motor_set_velocity(right_motor, K*sqrt(pow(u[0],2.0)+pow(u[1],2.0)));
            
            if (K*sqrt(pow(u[0],2.0)+pow(u[1],2.0)) > u_max){
                u_max = K*sqrt(pow(u[0],2.0)+pow(u[1],2.0));
            }
 
            if (!(control_angle(u, position, id, theta, false, 0.1, false))){
              NextState = angle_control;
            }
 
            if (check_for_obstacle(id, threshold, 0)){  
            
              distanceX = disX;
              original_angle = theta;
              
              //if (!check_line_of_sight(u)){
                NextState = obstacle;
                distanceX = X;
                original_angle = theta;
                obstacle_time = wb_robot_get_time();
                threshold = 0;   
                wb_motor_set_velocity(left_motor, 0);
                wb_motor_set_velocity(right_motor, 0);
              //}        
            }      
              
              
        } break;
        case obstacle: {
        
            // next_step = false;
            next_step_check = true;
            
            //Rotate in one direction untill side is detected
             wb_motor_set_velocity(left_motor, -distanceX*5);
             wb_motor_set_velocity(right_motor, distanceX*5);
            
            if(side_obstacle_detected && wb_robot_get_time()-obstacle_time > 0.1){
              NextState = obstacle_straight;
              start_move_straight = wb_robot_get_time();
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            } 
            
            if((original_angle > theta-0.05 && original_angle < theta+0.05) && wb_robot_get_time()-obstacle_time > 0.2){
              // In case it continuous rotating
              NextState = angle_control;
            }
                  
            redValue = 200;
            blueValue = 0; 
            greenValue = 100;
            
            disX_previous = disX;
            disY_previous = disY;
            check_for_obstacle(id, threshold, 0);
            
        
        }break;
        case obstacle_straight:{
        
          next_step_check = true;
          
          if (side_obstacle_detected){
          
             wb_motor_set_velocity(left_motor, 5);
             wb_motor_set_velocity(right_motor, 5);
          
             obstacle_time = wb_robot_get_time();
              
          }else if (wb_robot_get_time()-obstacle_time > 0.4){
              NextState = angle_control;   
              side_obstacle_detected = false;
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);    
                
          } 

          if(wb_robot_get_time()-start_move_straight > 1.5){
              NextState = angle_control;   
              side_obstacle_detected = false;
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);  
          }                  
          
          if (check_for_obstacle(id, 0, distanceX)){
            NextState = obstacle;
            distanceX = X;
            original_angle = theta;
            side_obstacle_detected = false;
            obstacle_time = wb_robot_get_time();
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);  
            //printf("test\n");  
          }
          
          if(disX < disX_previous-15 || (X < 0 && (disY > disY_previous+15)) ||  (X > 0 && (disY < disY_previous-15))){
            NextState = obstacle;
            distanceX = X;
            original_angle = theta;
            side_obstacle_detected = false;
            obstacle_time = wb_robot_get_time();
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);  
            //printf("test2\n");  
          }
          
            disX_previous = disX;
            disY_previous = disY;
            check_for_obstacle(id, 20, 0);
          
          
            redValue = 100;
            blueValue = 0; 
            greenValue = 200;
        
        }break;
    }  
        
    // get position values
    if (wb_robot_get_time()-period >= h){
      //printf("test %d, time: %f \n", id, wb_robot_get_time()-period);
      if (trigger(fp, x_hat, position, t_total_angle, L, id, ROBOTS)){
      
            //printf("Robot %d trigger, time %f, (%f, %f) \n", id, wb_robot_get_time()-period, position[0], position[1]);
      
            for (i=0; i<D; i++){
              fixed_position[i] = position[i];
            }
            
            fp = fixed_position;
            trigger_counter++;
            trigger_flag = true; 
            t_trigger = wb_robot_get_time();   
            
      }
      period = wb_robot_get_time();
     }
     
    if ((trigger_flag && wb_robot_get_time()-t_trigger > delay) || wb_robot_get_time()<0.2){
    
      redValue = 0;
      blueValue = 0; 
      greenValue = 200;
      //printf("Robot %d trigger, time %f, (%f, %f) \n", id, wb_robot_get_time(), position[0], position[1]);
      
      prep_msg(prox_value, ground_value, accValue, prox_ambient_value, ground_ambient_value, fixed_position, next_step_check, trigger_flag, trigger_counter, u_max);               
    
      wb_emitter_send(emitter, ackPayload, 81); 
      trigger_flag = false;
    }
        
    wb_receiver_next_packet(receiver);        
  
    step();
    
  }
  
  wb_robot_cleanup();

  return 0;

//ciao
}
