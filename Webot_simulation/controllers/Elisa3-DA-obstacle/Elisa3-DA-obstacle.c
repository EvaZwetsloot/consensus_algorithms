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
#include <string.h>
#include <Elisa3-DA.h>


#define ROBOTS 8
#define LEDS_NUMBER 8
#define pi 3.142857
#define NOISE_THR 50
#define D 2

double u_max = 23;
int Diameter = 3;

int L[ROBOTS][ROBOTS] = {{3, -1, -1, 0, -1, 0, 0, 0}, {-1, 3, 0, -1, 0, -1, 0, 0}, {-1, 0, 3, -1, 0, 0, -1, 0}, {0, -1, -1, 3, 0, 0, 0, -1}, {-1, 0, 0, 0, 3, -1, -1, 0}, {0, -1, 0, 0, -1, 3, 0, -1}, {0, 0, -1, 0, -1, 0, 3, -1}, {0, 0, 0, -1, 0, -1, -1, 3}};
double B[D][ROBOTS] = {{0.5000, 0.2233, -0.0915, -0.3814, 0.3692, 0.0952, -0.2206, -0.4943}, {0.1198, -0.0390, -0.2027, -0.3357, 0.3395, 0.1961, 0.0319, -0.1098}};
int Ni[ROBOTS] = {3, 3, 3, 3, 3, 3, 3, 3};
double error = 0.01;
double tau = 0.1;

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
char ackPayload_robot[2+4*ROBOTS+2] = {0};
double disX =0, disY = 0, X=0;
double distanceY = 0, distanceX = 0;
int data_size = 16384;
static state NextState = initialize;
bool side_obstacle_detected = false;
state mode = initialize;

void robot_msg(double (*x_data)[D], int id){
  
  ackPayload_robot[0] = wb_emitter_get_channel(emitter);
  ackPayload_robot[1] = id; // first packet
  
  int i = 0;
  for(int k=0; k<ROBOTS; k++){
    ackPayload_robot[2+i] = ((int) floor(x_data[k][0]*data_size))&0xFF; // not implemented (xpos => 2 bytes) [11 fractional bits, 5 integer]
    ackPayload_robot[3+i] = ((int) floor(x_data[k][0]*data_size))>>8;
    ackPayload_robot[4+i] = ((int) floor(x_data[k][1]*data_size))&0xFF; // not implemented (ypos => 2 bytes)
    ackPayload_robot[5+i] = ((int) floor(x_data[k][1]*data_size))>>8;
    i = i+4;
    
    //printf("robot %d send to %d x_data[%d] is (%f, %f)\n", id, ackPayload_robot[0], k, x_data[k][0], x_data[k][1]);
             
  }
   
}

void robot_formation(int (*formation), int id){
  
  ackPayload_robot[0] = wb_emitter_get_channel(emitter);
  ackPayload_robot[1] = id; // first packet
  
  int i = 0;
  for(int k=0; k<ROBOTS; k++){
    ackPayload_robot[2+i] = ((int) formation[k])&0xFF; // not implemented (xpos => 2 bytes) [11 fractional bits, 5 integer]
    ackPayload_robot[3+i] = ((int) formation[k])>>8;
    i = i+2;
    
    //printf("robot %d send to %d x_data[%d] is (%f, %f)\n", id, ackPayload_robot[0], k, x_data[k][0], x_data[k][1]);
             
  }
  
   
}

void sup_msg(int *prox_value, int *ground_value, const double *accValue, int *prox_ambient_value, int *ground_ambient_value, bool flag, bool done, int id_send){

  ackPayload[0] = wb_emitter_get_channel(emitter);
  ackPayload[1] = (int) id_send; // first packet
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
  ackPayload[71] = 0;
  ackPayload[72] = 0;
  ackPayload[73] = 0;
  ackPayload[74] = 0; // not implemented (theta => 2 bytes)
  ackPayload[75] = 0;
  ackPayload[76] = 0; // not implemented (xpos => 2 bytes) [11 fractional bits, 5 integer]
  ackPayload[77] = 0;
  ackPayload[78] = 0; // not implemented (ypos => 2 bytes)
  ackPayload[79] = done;
  ackPayload[80] = flag; // free
  }

static void step() {
  if (wb_robot_step(time_step) == -1) {
    wb_robot_cleanup();
    exit(EXIT_SUCCESS);
  }
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
    //puts(device_name);
    distance_sensor[i] = wb_robot_get_device(device_name);
    wb_distance_sensor_enable(distance_sensor[i],time_step);
    sprintf(device_name, "ls%d", i);
    distance_ambient_sensor[i] = wb_robot_get_device(device_name);
    wb_light_sensor_enable(distance_ambient_sensor[i],time_step*4);
        
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

double *da_control(const double (*x), double *x_ref, int N, int id){

  static double u[2];
  double K = 50;

  u[0] = -K*(x[0]-x_ref[0]);
  u[1] = -K*(x[1]-x_ref[1]);
   
  //printf("u %d: %f, %f\n", id, u[0], u[1]);
  
  for (int i=0; i<D;i++){
    if (u[i] > -u_max){
      u[i] = -u_max;
    }else if (u[i] > u_max){
      u[i] = u_max;
    }
  }
   
  return u;

}

bool control_angle(double *u,const double *x, int id, double theta, bool check){

  double lspeed= 0, rspeed =0, speed = 0;
  double error = 0;
  double theta_new = 0;
  double K = 0.3;
  bool out = false;
  
  theta_new = atan2(u[1]-x[1], u[0]-x[0]);  
  
  error = (theta_new-theta);
  
  if (error > pi){
      error = -2*pi+error;
  }else if (error < -pi){
      error = 2*pi+error;
  }
  
  error = error*57.3;
  
  // if (id ==0){
  // printf("theta %d: %f, %f\n", id, theta_new, theta);
  // printf("position %d: %f, %f\n", id, x[0], x[1]);
  // printf("x_ref %d: %f, %f\n", id, u[0], u[1]);
  // }
  
  speed = K*error;
  
  
   if (speed > 15){
    speed = 15;
   }else if ( speed < -15){
    speed = -15;
   }
   
   lspeed = -speed;
   rspeed = speed;
  
  //printf("error %d: %f, %f\n", id, error, fabs(error));
  
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
  double x_data[ROBOTS][D];
  memset(x_data, 0, D*ROBOTS*8);
  int com = 0;
  double sum[D];
  memset(sum, 0, 8*D);
  int id;
  int N;
  int i = 0;
  int Ni = 0;
  int id_neigh;
  double t =0;
  bool delay_flag = true;
  bool send_data_to_robot = true;
  double time_init = 0;
  double time_rotate = 0;
  double threshold = 0;
  double original_angle = 0;
  int id_formation[ROBOTS] = {ROBOTS+1};
  double smallest = 100;
  //int id_smallest[ROBOTS][ROBOTS] = {ROBOTS+1};
  //int nb_rejects[ROBOTS+1] = {0}
  double distance = 0;
  double obstacle_time = 0;
  double disX_previous = 0, disY_previous = 0;
  double start_time =0;
  double start_move_straight = 0;

  //float theta = 0;
  double *u;
  double theta = 0;
  
  N = ROBOTS;
  
  // Initiate neighbours and id from arguments
  char* token;
  char* rest = argv[1];
  
  int m = 0;
  while ((token = strtok_r(rest, " ", &rest))){
     
    if (m ==0){
      id = strtol(token, &ptr, 10);
    }else{
      Ni = strtol(token, &ptr, 10);
    }
    
    m++;
  }

  int rec[ROBOTS];
  memset(rec, 0, 4*ROBOTS);
  int neigh[Ni];
  
  int k = 0;
  for (i=0;i<ROBOTS;i++){
      if (L[id][i] < 0){
        neigh[k] = i;
        k++;
       }        
  }
  
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
  double x_ref[D] = {position[0], position[1]};
  
  //Set own x0 in memory
  x_data[id][0] = position[0];
  x_data[id][1] = position[1]; 
  
  //printf("start robot %d, x_data[%d] is (%f, %f)\n", id, id, x_data[id][0], x_data[id][1]);
 

  /* main loop */
  start_time = wb_robot_get_time();
  while (true) {  
  
    position = wb_gps_get_values(gps);
  
    switch (NextState)
    {
    case initialize:
    {
        wb_receiver_set_channel(receiver, id+1);
        //printf("robot %d, (%f)\n", id, wb_robot_get_time());
        if(wb_receiver_get_queue_length(receiver) > 0) {
          //printf("received %d", id);
          
          redValue = 200;
          blueValue = 200; 
          greenValue = 0;
                
          const char *msg = wb_receiver_get_data(receiver);
          //printf("msg is %d, %d, %f, ...\n", msg[2], msg[7], ((double) (msg[6]+(msg[7]<<8)))/2048);
          
          id_neigh = (int) msg[1];
          rec[id_neigh]++;
          
          //Unpack x_hat
          int k = 0;
          for(i = 0; i < N; i++){
          
              if ((x_data[i][0] == 0 && x_data[i][1] == 0) || i == id){
                   x_data[i][0] = ((double) (msg[2+k]+(msg[3+k]<<8)))/data_size;
                   x_data[i][1] = ((double) (msg[4+k]+(msg[5+k]<<8)))/data_size;  
              }            
              //printf("robot %d id_n: %d, channel %d, x_hat[%d] is (%f, %f)\n", id, id_neigh, msg[0], i, x_data[i][0], x_data[i][1]);
              k= k+4;
          }

          wb_receiver_next_packet(receiver);                     
        }  
        
        // check number received vs number send
        if (delay_flag){
          NextState = compute_sum;
          send_data_to_robot = true;
          for (i=0; i< Ni; i++){
              if(rec[neigh[i]] < com){
                //printf("Robot %d check %d\n", id, com);
                send_data_to_robot = false;
              } 
              
              if(rec[neigh[i]] < Diameter){
                NextState = initialize;
              }else if (wb_receiver_get_queue_length(receiver) > 0){
                //clear queue first before moving on
                send_data_to_robot = false;
                NextState = initialize;
              }     
          }
         }
        
        if (delay_flag){
          t = wb_robot_get_time();
        }
        
        //printf("Robot %d, flag: %d, t %f\n", id, send_data_to_robot, t);
        
        if (send_data_to_robot){
        
          redValue = 0;
          blueValue = 0; 
          greenValue = 100;
            
          robot_msg(x_data, id);
          
          //printf("Robot %d, time %f, fix time %f\n", id, wb_robot_get_time(), t);
        
          //send data 
          if (wb_robot_get_time() > t + tau || t==0) { 
          
            com++;  
          
            //printf("Robot %d, send data, time %f, time old %f \n", id, wb_robot_get_time(), t);
            for (i=0; i< Ni; i++){  
              wb_emitter_set_channel(emitter, neigh[i]+1);
              wb_emitter_send(emitter, ackPayload_robot, 2+4*ROBOTS+2); 
            }
            
            send_data_to_robot = false;
            delay_flag = true;
          }else{
              delay_flag = false;
          }
        } 
        
        //NextState = idle;
      }
     break; 
     case compute_sum:{
     
       time_init = wb_robot_get_time();
     
       printf("Robot %d, time to initialize %f\n", id, time_init);
     
       // compute average position
       for (i=0; i<ROBOTS; i++){     
         sum[0] += x_data[i][0];
         sum[1] += x_data[i][1]; 
         
         printf("Robot %d, %d, [%f %f] \n", id, i,   x_data[i][0],  x_data[i][1]);
        }
         
         x_ref[0] = sum[0]/ROBOTS;
         x_ref[1] = sum[1]/ROBOTS;
                                
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
        // get accelerometer values
        accValue = wb_accelerometer_get_values(accelerometer);
        
        sup_msg(prox_value, ground_value, accValue, prox_ambient_value, ground_ambient_value, true, false, id);
        
        wb_emitter_set_channel(emitter, ROBOTS+2);
        wb_emitter_send(emitter, ackPayload, 81);  
        
        //printf("nb_packets remaining: %d\n", wb_receiver_get_queue_length(receiver));
      
        NextState = compute_formation; 
     
     }
     break;
     case compute_formation:{
     
          redValue = 100;
          blueValue = 0; 
          greenValue = 100;
     
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
               distance = sqrt(pow(x_ref[0]+B[0][i]-x_data[j][0],2.0) + pow(x_ref[1]+B[1][i]-x_data[j][1],2.0));
               
               if(distance + 0.05 < smallest){
                 // compensate for some noise with 0.1
                 printf("Robot %d, %d smallest %f, distance %f \n", id, j, smallest, distance);
                 smallest = distance;
                 id_formation[j] = i;
               }  
                 
           } 
         }
         
         smallest = 100;           
       }
       
         for (int k=0; k<ROBOTS;k++){
             printf("Robot %d, id_formation[%d] = %d\n", id, k, id_formation[k]);
         } 
       
       int correct_id = 0;
       for(int n=0;n<ROBOTS;n++){
       
         if(id_formation[n] == id){
             correct_id = n;
         }
       }

       //printf("Robot %d, [%f, %f]\n", id, x_ref[0], x_ref[1]);
       x_ref[0] += B[0][id];
       x_ref[1] += B[1][id];       
     
       //printf("Robot %d, [%f, %f]\n", id, x_ref[0], x_ref[1]);
       NextState = control_theta;
     
     }
     break;
     case control_theta:{
     
          redValue = 100;
          blueValue = 0; 
          greenValue = 0;
     
         // get angle of robot
         wb_receiver_set_channel(receiver, id+1);
         if(wb_receiver_get_queue_length(receiver) > 0) {
       
       
          if (control_angle(x_ref, position, id, theta, true)){
            NextState = control_speed;
            time_rotate += wb_robot_get_time()-time_init;
            //printf("Robot %d, time to rotate %f\n", id, time_rotate);
            
          }
          }  
    
     }
     break;
     case control_speed:{
     
          redValue = 0;
          blueValue = 100; 
          greenValue = 0;     
     
        u = da_control(position, x_ref, N, id);
        
        //printf("Robot %d, control [%f, %f]\n", id, u[0], u[1]);
        //printf("Robot %d, position [%f, %f]\n", id, position[0], position[1]);
        //printf("Robot %d, x_ref [%f, %f]\n", id, x_ref[0], x_ref[1]);
        
        // if (u[0] >= 0){
          // desL = sqrt(pow(u[0],2)+pow(u[1],2));
          // desR = sqrt(pow(u[0],2)+pow(u[1],2));
        // } else{
          // desL = -sqrt(pow(u[0],2)+pow(u[1],2));
          // desR = -sqrt(pow(u[0],2)+pow(u[1],2));
        // }
        
        wb_motor_set_velocity(left_motor, sqrt(pow(u[0],2)+pow(u[1],2)));
        wb_motor_set_velocity(right_motor, sqrt(pow(u[0],2)+pow(u[1],2)));
        
        if (!(control_angle(x_ref, position, id, theta, false))){
           NextState = control_theta;
            //time_rotate += wb_robot_get_time()-time_init;
            //printf("Robot %d, time to rotate %f\n", id, time_rotate);
            
        }
        
        
        if(fabs(position[0]-x_ref[0]) < error && fabs(position[1]-x_ref[1]) < error){
          NextState = idle;
          wb_motor_set_velocity(left_motor, 0);
          wb_motor_set_velocity(right_motor, 0);
          
          printf("Robot %d, done time: %f\n", id, wb_robot_get_time()-start_time);
          printf("Robot %d, nb of comms: %d\n", id, com);
        }
       
        
        
        if(wb_receiver_get_queue_length(receiver) > 0){
          wb_receiver_next_packet(receiver);   
        }  
        
     }
     break;
     case idle:{
     
        redValue = 0;
        blueValue = 0; 
        greenValue = 100;
        
        sup_msg(prox_value, ground_value, accValue, prox_ambient_value, ground_ambient_value, false, true, id);
    
        wb_emitter_set_channel(emitter, ROBOTS+2);
        wb_emitter_send(emitter, ackPayload, 81); 
     }
     break;
     case avoid_obstacle:{      
                          
            //Rotate in one direction untill side is detected
             wb_motor_set_velocity(left_motor, -distanceX*5);
             wb_motor_set_velocity(right_motor, distanceX*5);
            
            if(side_obstacle_detected && wb_robot_get_time()-obstacle_time > 0.1){
              NextState = obstacle_straight;
              obstacle_time = wb_robot_get_time();
              start_move_straight = wb_robot_get_time();
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            } 
            
            if(wb_robot_get_time()-obstacle_time > 3.0 && !(check_for_obstacle(id, threshold, 0))){
              NextState = control_theta;
              obstacle_time = wb_robot_get_time();
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            }
            
            if((original_angle > theta-0.05 && original_angle < theta+0.05) && wb_robot_get_time()-obstacle_time > 0.2){
              // In case it continuous rotating
              NextState = control_theta;
              obstacle_time = wb_robot_get_time();
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);
            }
                  
            redValue = 200;
            blueValue = 0; 
            greenValue = 100;
            
            disX_previous = disX;
            disY_previous = disY;
            check_for_obstacle(id, threshold, 0);
     
     }
     break;
     case obstacle_straight:{
     
          if (side_obstacle_detected){
          
             wb_motor_set_velocity(left_motor, 5);
             wb_motor_set_velocity(right_motor, 5);
          
             obstacle_time = wb_robot_get_time();
              
          }else if (wb_robot_get_time()-obstacle_time > 0.4){
              NextState = control_theta;   
              side_obstacle_detected = false;
              
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);    
                
          } 

          if(wb_robot_get_time()-start_move_straight > 1.5){
              NextState = control_theta;   
              side_obstacle_detected = false;
              wb_motor_set_velocity(left_motor, 0);
              wb_motor_set_velocity(right_motor, 0);  
          }            
          
          if (check_for_obstacle(id, 0, distanceX)){
            NextState = avoid_obstacle;
            distanceX = X;
            original_angle = theta;
            side_obstacle_detected = false;
            obstacle_time = wb_robot_get_time();
            wb_motor_set_velocity(left_motor, 0);
            wb_motor_set_velocity(right_motor, 0);  
            //printf("test\n");  
          }
          
          if(disX < disX_previous-15 || (X < 0 && (disY > disY_previous+15)) ||  (X > 0 && (disY < disY_previous-15))){
            NextState = avoid_obstacle;
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
    
    if(wb_receiver_get_queue_length(receiver) > 0 && !(NextState == initialize)) {
        //Unpack theta
        
        const char *msg = wb_receiver_get_data(receiver);
        theta = ((double) (msg[6]+(msg[7]<<8)))/4096;  
        
        //printf("Robot %d, theta: (%f)\n", id, theta);   
        
        wb_receiver_next_packet(receiver); 
    }
    
    if(check_for_obstacle(id, 0, 0) && !(NextState == idle)){
        
       //if (!check_line_of_sight(u)){
         NextState = avoid_obstacle;
         threshold = 0;
         distanceX = X;
         obstacle_time = wb_robot_get_time();
         time_init = wb_robot_get_time();
         original_angle = theta;
       //}
    }
    
    if(wb_robot_get_time() > 350){
      NextState = idle;
    }
    
    currentColor = redValue*65536 + greenValue*256 + blueValue;
    wb_led_set(ledrgb, currentColor); // 0x0000ff);
  
    step();
    
  }
  
  wb_robot_cleanup();

  return 0;

//ciao
}
