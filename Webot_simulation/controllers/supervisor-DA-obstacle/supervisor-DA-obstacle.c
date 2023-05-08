#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#define ROBOTS 8
#define WALL_THR 300
#define SPEED 30
#define D 2
#define pi 3.142857


int L[ROBOTS][ROBOTS] = {{3, -1, -1, 0, -1, 0, 0, 0}, {-1, 3, 0, -1, 0, -1, 0, 0}, {-1, 0, 3, -1, 0, 0, -1, 0}, {0, -1, -1, 3, 0, 0, 0, -1}, {-1, 0, 0, 0, 3, -1, -1, 0}, {0, -1, 0, 0, -1, 3, 0, -1}, {0, 0, -1, 0, -1, 0, 3, -1}, {0, 0, 0, -1, 0, -1, -1, 3}};
double B[D][ROBOTS] = {{0.5000, 0.2233, -0.0915, -0.3814, 0.3692, 0.0952, -0.2206, -0.4943}, {0.1198, -0.0390, -0.2027, -0.3357, 0.3395, 0.1961, 0.0319, -0.1098}};
int Ni[ROBOTS] = {3, 3, 3, 3, 3, 3, 3, 3};

typedef enum{
  idle,
  angle_control
} state;

char speed(char value) {
    if(value >= 0) {
        return (value|0x80);
    } else {
        return ((-value)&0x7F);
    }
}

float randomFloat()
{
      float r = (float)rand()/(float)RAND_MAX * 1.8+0.1;
      return r;
}

int main() {

  int i=0, j=0;
  char txData[9];
  //char rxData[16] = {0};
  const char *robot_name[ROBOTS] = {"ELISA3-1", "ELISA3-2", "ELISA3-3", "ELISA3-4", "ELISA3-5"};
  WbNodeRef robot[ROBOTS];
  WbFieldRef robot_fields[2*ROBOTS];
  WbDeviceTag receiver;
  WbDeviceTag emitter;
  double position[ROBOTS][D];
  const double *pos;
  double theta;
  int count = 0; 
  static state NextState = idle;
  double count_done[ROBOTS] = {0};
  bool done_flag = false;
  int m = 0;
  FILE *file_ptr;
  
  file_ptr = fopen("Phase_HC_8_mesh_a.txt","a");
  

  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
  
  srand(time(NULL));
  // Create robots
  float init_position[2] = {0,0};
  char *robot_field;
  for (i=0; i<ROBOTS; i++){
    init_position[0] = randomFloat();
    init_position[1] = randomFloat();
    //init_position[0] = top_pos[0][i];
    //init_position[1] = top_pos[1][i];
    
    asprintf(&robot_name[i], "ELISA3-%d", i);
    asprintf(&robot_field, "DEF ELISA3-%d Elisa3 {controller \"Elisa3-DA-obstacle\" controllerArgs \"%d %d\" translation %f %f 0}", i,i, Ni[i] ,init_position[0], init_position[1]);
    wb_supervisor_field_import_mf_node_from_string(children_field, -1, robot_field);
  }
  
  for(i=0, j=0; i<ROBOTS; i++, j+=2) {
    robot[i] = wb_supervisor_node_get_from_def(robot_name[i]);
    robot_fields[j] = wb_supervisor_node_get_field(robot[i], "emitter_channel");
    robot_fields[j+1] = wb_supervisor_node_get_field(robot[i], "receiver_channel");
    wb_supervisor_field_set_sf_int32(robot_fields[j], i+1);
    wb_supervisor_field_set_sf_int32(robot_fields[j+1], i+1);   
  }
  
  receiver = wb_robot_get_device("receiver0");
  wb_receiver_enable(receiver, time_step);
  wb_receiver_set_channel(receiver, -1); // receive on all channels
  emitter = wb_robot_get_device("emitter0");  
  
  double start_time = wb_robot_get_time(); 

  while(1) { 
  
    wb_robot_step(time_step);
     
    switch (NextState){
    
      case idle:{
      
        bool init_flag = true;
        
        wb_receiver_set_channel(receiver, ROBOTS+2);
        for(j=0; j<ROBOTS; j++) {
          if(wb_receiver_get_queue_length(receiver) > 0) {
                const unsigned char *msg = wb_receiver_get_data(receiver);
                
                //printf("robot %d msg[80] %d\n", j, msg[80]);
                if(msg[80] == true){
                  count++;
                }
                
          wb_receiver_next_packet(receiver);   
          }
        }
        
        if (count == ROBOTS){
        
            //wb_emitter_set_channel(emitter, -1);
            
            NextState = angle_control;
        }     
      }
      break;
      case angle_control:{
      
          //printf("moved to angle\n");
          
          for(j=0; j<ROBOTS; j++) {
            //printf("channel: %d\n", wb_emitter_get_channel(emitter));
            pos = wb_supervisor_node_get_orientation(robot[j]);
            theta = atan2(pos[3], pos[0]);
            
           if (theta <0){
              theta += 2*pi;
            }
            
            if (theta== 2*pi){
                theta = 0;
            }
            
            
            txData[6] = ((int) floor(theta*4096))&0xFF;
            txData[7] = ((int) floor(theta*4096))>>8;
            
            //printf("theta %d, %f, %f\n", j, ((double) (txData[6]+(txData[7]<<8)))/2048, theta);
            
            wb_emitter_set_channel(emitter, j+1);
            wb_emitter_send(emitter, txData, 9);
           
            
          }  
      
      }
      break;
    
    }
    
      for (i=0;i<ROBOTS; i++){
        if(wb_receiver_get_queue_length(receiver) > 0) {
          const unsigned char *msg = wb_receiver_get_data(receiver);
          
          m = (int) msg[1];
          
          //printf("robot %d msg[79] %d\n", m, msg[79]);
          if(msg[79] == true){
            count_done[m] = 1;
          } else{
            count_done[m] = 0;
          }
              
          wb_receiver_next_packet(receiver);   
        }
      }
      
      done_flag = true;
      for (i=0;i<ROBOTS;i++){
        if(count_done[i] == 0){
          done_flag = false;
        }
      }
      
      if (done_flag){
         printf("Simulation done \n");
         fprintf(file_ptr, "%f, ", wb_robot_get_time()-start_time);
         fclose(file_ptr);
          wb_supervisor_simulation_reset();

          wb_supervisor_node_restart_controller(wb_supervisor_node_get_from_def("Supervisor"));
      }
      
  }
  
}

