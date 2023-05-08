 
#include <webots/robot.h>
#include <webots/supervisor.h>
#include <webots/receiver.h>
#include <webots/emitter.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#define ROBOTS 16
#define WALL_THR 300
#define SPEED 30
#define D 2
#define pi 3.142857
#define memory 100


int L[ROBOTS][ROBOTS] = {{4, -1, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0}, {-1, 4, 0, -1, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0}, {-1, 0, 4, -1, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0, 0}, {0, -1, -1, 4, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 0}, {-1, 0, 0, 0, 4, -1, -1, 0, 0, 0, 0, 0, -1, 0, 0, 0}, {0, -1, 0, 0, -1, 4, 0, -1, 0, 0, 0, 0, 0, -1, 0, 0}, {0, 0, -1, 0, -1, 0, 4, -1, 0, 0, 0, 0, 0, 0, -1, 0}, {0, 0, 0, -1, 0, -1, -1, 4, 0, 0, 0, 0, 0, 0, 0, -1}, {-1, 0, 0, 0, 0, 0, 0, 0, 4, -1, -1, 0, -1, 0, 0, 0}, {0, -1, 0, 0, 0, 0, 0, 0, -1, 4, 0, -1, 0, -1, 0, 0}, {0, 0, -1, 0, 0, 0, 0, 0, -1, 0, 4, -1, 0, 0, -1, 0}, {0, 0, 0, -1, 0, 0, 0, 0, 0, -1, -1, 4, 0, 0, 0, -1}, {0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, 0, 4, -1, -1, 0}, {0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, 0, -1, 4, 0, -1}, {0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, 0, 4, -1}, {0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, -1, 0, -1, -1, 4}};
float Bx[ROBOTS][ROBOTS] = {{0.000, 0.038, 0.146, 0.309, 0.500, 0.691, 0.854, 0.962, 1.000, 0.962, 0.854, 0.691, 0.500, 0.309, 0.146, 0.038}, {-0.038, 0.000, 0.108, 0.271, 0.462, 0.653, 0.815, 0.924, 0.962, 0.924, 0.815, 0.653, 0.462, 0.271, 0.108, -0.000}, {-0.146, -0.108, 0.000, 0.162, 0.354, 0.545, 0.707, 0.815, 0.854, 0.815, 0.707, 0.545, 0.354, 0.162, 0.000, -0.108}, {-0.309, -0.271, -0.162, 0.000, 0.191, 0.383, 0.545, 0.653, 0.691, 0.653, 0.545, 0.383, 0.191, -0.000, -0.162, -0.271}, {-0.500, -0.462, -0.354, -0.191, 0.000, 0.191, 0.354, 0.462, 0.500, 0.462, 0.354, 0.191, -0.000, -0.191, -0.354, -0.462}, {-0.691, -0.653, -0.545, -0.383, -0.191, 0.000, 0.162, 0.271, 0.309, 0.271, 0.162, 0.000, -0.191, -0.383, -0.545, -0.653}, {-0.854, -0.815, -0.707, -0.545, -0.354, -0.162, 0.000, 0.108, 0.146, 0.108, 0.000, -0.162, -0.354, -0.545, -0.707, -0.815}, {-0.962, -0.924, -0.815, -0.653, -0.462, -0.271, -0.108, 0.000, 0.038, 0.000, -0.108, -0.271, -0.462, -0.653, -0.815, -0.924}, {-1.000, -0.962, -0.854, -0.691, -0.500, -0.309, -0.146, -0.038, 0.000, -0.038, -0.146, -0.309, -0.500, -0.691, -0.854, -0.962}, {-0.962, -0.924, -0.815, -0.653, -0.462, -0.271, -0.108, 0.000, 0.038, 0.000, -0.108, -0.271, -0.462, -0.653, -0.815, -0.924}, {-0.854, -0.815, -0.707, -0.545, -0.354, -0.162, -0.000, 0.108, 0.146, 0.108, 0.000, -0.162, -0.354, -0.545, -0.707, -0.815}, {-0.691, -0.653, -0.545, -0.383, -0.191, -0.000, 0.162, 0.271, 0.309, 0.271, 0.162, 0.000, -0.191, -0.383, -0.545, -0.653}, {-0.500, -0.462, -0.354, -0.191, 0.000, 0.191, 0.354, 0.462, 0.500, 0.462, 0.354, 0.191, 0.000, -0.191, -0.354, -0.462}, {-0.309, -0.271, -0.162, 0.000, 0.191, 0.383, 0.545, 0.653, 0.691, 0.653, 0.545, 0.383, 0.191, 0.000, -0.162, -0.271}, {-0.146, -0.108, -0.000, 0.162, 0.354, 0.545, 0.707, 0.815, 0.854, 0.815, 0.707, 0.545, 0.354, 0.162, 0.000, -0.108}, {-0.038, 0.000, 0.108, 0.271, 0.462, 0.653, 0.815, 0.924, 0.962, 0.924, 0.815, 0.653, 0.462, 0.271, 0.108, 0.000}};
float By[ROBOTS][ROBOTS] = {{0.000, -0.191, -0.354, -0.462, -0.500, -0.462, -0.354, -0.191, 0.000, 0.191, 0.354, 0.462, 0.500, 0.462, 0.354, 0.191}, {0.191, 0.000, -0.162, -0.271, -0.309, -0.271, -0.162, -0.000, 0.191, 0.383, 0.545, 0.653, 0.691, 0.653, 0.545, 0.383}, {0.354, 0.162, 0.000, -0.108, -0.146, -0.108, 0.000, 0.162, 0.354, 0.545, 0.707, 0.815, 0.854, 0.815, 0.707, 0.545}, {0.462, 0.271, 0.108, 0.000, -0.038, -0.000, 0.108, 0.271, 0.462, 0.653, 0.815, 0.924, 0.962, 0.924, 0.815, 0.653}, {0.500, 0.309, 0.146, 0.038, 0.000, 0.038, 0.146, 0.309, 0.500, 0.691, 0.854, 0.962, 1.000, 0.962, 0.854, 0.691}, {0.462, 0.271, 0.108, 0.000, -0.038, 0.000, 0.108, 0.271, 0.462, 0.653, 0.815, 0.924, 0.962, 0.924, 0.815, 0.653}, {0.354, 0.162, 0.000, -0.108, -0.146, -0.108, 0.000, 0.162, 0.354, 0.545, 0.707, 0.815, 0.854, 0.815, 0.707, 0.545}, {0.191, 0.000, -0.162, -0.271, -0.309, -0.271, -0.162, 0.000, 0.191, 0.383, 0.545, 0.653, 0.691, 0.653, 0.545, 0.383}, {-0.000, -0.191, -0.354, -0.462, -0.500, -0.462, -0.354, -0.191, 0.000, 0.191, 0.354, 0.462, 0.500, 0.462, 0.354, 0.191}, {-0.191, -0.383, -0.545, -0.653, -0.691, -0.653, -0.545, -0.383, -0.191, 0.000, 0.162, 0.271, 0.309, 0.271, 0.162, -0.000}, {-0.354, -0.545, -0.707, -0.815, -0.854, -0.815, -0.707, -0.545, -0.354, -0.162, 0.000, 0.108, 0.146, 0.108, 0.000, -0.162}, {-0.462, -0.653, -0.815, -0.924, -0.962, -0.924, -0.815, -0.653, -0.462, -0.271, -0.108, 0.000, 0.038, 0.000, -0.108, -0.271}, {-0.500, -0.691, -0.854, -0.962, -1.000, -0.962, -0.854, -0.691, -0.500, -0.309, -0.146, -0.038, 0.000, -0.038, -0.146, -0.309}, {-0.462, -0.653, -0.815, -0.924, -0.962, -0.924, -0.815, -0.653, -0.462, -0.271, -0.108, 0.000, 0.038, 0.000, -0.108, -0.271}, {-0.354, -0.545, -0.707, -0.815, -0.854, -0.815, -0.707, -0.545, -0.354, -0.162, -0.000, 0.108, 0.146, 0.108, 0.000, -0.162}, {-0.191, -0.383, -0.545, -0.653, -0.691, -0.653, -0.545, -0.383, -0.191, 0.000, 0.162, 0.271, 0.309, 0.271, 0.162, 0.000}};
float top_pos[D][ROBOTS] = {{0.552, 1.253, 0.451, 1.214, 1.242, 1.900, 1.196, 1.886, 0.113, 0.805, 0.102, 0.757, 0.785, 1.549, 0.746, 1.450}, {1.788, 1.870, 0.995, 1.056, 1.474, 1.493, 0.623, 0.717, 1.282, 1.376, 0.508, 0.528, 0.947, 1.003, 0.130, 0.211}};
double delay = 0.1;
float error = 0.1;

char speed(char value) {
    if(value >= 0) {
        return (value|0x80);
    } else {
    
        return ((-value)&0x7F);
    }
}

float randomFloat()
{
      float r = (float)rand()/(float)RAND_MAX * 1.9+0.05;
      return r;
}


int main() {

  int i=0, j=0;
  char txData[9+2*D*ROBOTS];
  //char rxData[16] = {0};
  const char *robot_name[ROBOTS] = {"ELISA3-1", "ELISA3-2", "ELISA3-3", "ELISA3-4", "ELISA3-5"};
  WbNodeRef robot[ROBOTS];
  WbFieldRef robot_fields[2*ROBOTS];
  WbDeviceTag receiver;
  WbDeviceTag emitter;
  double position[ROBOTS][D];
  const double *actual_position[ROBOTS];
  int triggers[ROBOTS];
  double u_max[ROBOTS];
  int total_count =0;
  const double *pos;
  double theta;
  bool check_next_step = false;
  int data_size = 16384;
  double start_time =0;
  double start_wait = 0;
  double delay = 0.1;
  bool done = false;
  int m = 0;
  FILE *file_ptr;
  
  file_ptr = fopen("PETC_HC_16_angle_s.txt","a");
  

  wb_robot_init();
  int time_step = wb_robot_get_basic_time_step();
  
  WbNodeRef root_node = wb_supervisor_node_get_root();
  WbFieldRef children_field = wb_supervisor_node_get_field(root_node, "children");
  
  srand(time(NULL));
  // Create robots
  float init_position[2] = {0,0};
  char *robot_field;
  for (i=0; i<ROBOTS; i++){
    //init_position[0] = randomFloat();
    //init_position[1] = randomFloat();
    init_position[0] = top_pos[0][i];
    init_position[1] = top_pos[1][i];
        
    asprintf(&robot_name[i], "ELISA3-%d", i);
    asprintf(&robot_field, "DEF ELISA3-%d Elisa3 {controller \"Elisa3-petc-obstacle\" controllerArgs \"%d\" translation %f %f 0}", i,i, init_position[0], init_position[1]);
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
  
  start_time = wb_robot_get_time();

  // printf("reset : %f", wb_robot_get_time());

  while(!done) { 
  
    wb_robot_step(time_step);
 
    
          
    int count = 0;        
    for(j=0; j<ROBOTS; j++) {
      //printf("channel: %d\n", wb_emitter_get_channel(emitter));
      
      pos = wb_supervisor_node_get_orientation(robot[j]);
      actual_position[j] = wb_supervisor_node_get_position(robot[j]);
      theta = atan2(pos[3], pos[0]);
      
      txData[6] = ((int) floor(theta*4069))&0xFF;
      txData[7] = ((int) floor(theta*4069))>>8;
      
      //printf("theta %d, %d, %d, %f, %f\n", j, txData[6], txData[7], ((double) (txData[6]+(txData[7]<<8)))/2048, theta);
      
      
      // Include position of robots neighbours and its own
      //if (update_data_flag)
        int k = 0;
        for (i=0;i<ROBOTS;i++){
            //if (!(L[j][i] == 0))
            
              txData[8+k] = ((int) floor(position[i][0]*data_size))&0xFF;
              txData[9+k] = ((int) floor(position[i][0]*data_size))>>8;
              txData[10+k] = ((int) floor(position[i][1]*data_size))&0xFF;
              txData[11+k] = ((int) floor(position[i][1]*data_size))>>8;
              
             //printf("Supervisor %d (%f, %f)\n",j, ((double) (txData[8+k]+(txData[9+k]<<8)))/2048, ((double) (txData[10+k]+(txData[11+k]<<8)))/2048);
            
            k = k+4;
        } 

      
      txData[4] = check_next_step;
      txData[3] = false;
      
      wb_emitter_set_channel(emitter, j+1);
      wb_emitter_send(emitter, txData, 9+2*D*ROBOTS);
      
        if(wb_receiver_get_queue_length(receiver) > 0) {
        
          // if(wb_robot_get_time()-start_time > delay){
            const unsigned char *msg = wb_receiver_get_data(receiver);
            
            m = (int) msg[0]-1;
            //m = j;
            
            //printf("Robot %d, m %d\n", j, m);
    
            if (((double) (msg[76]+(msg[77]<<8)))/data_size > 0){
              position[m][0] = ((double) (msg[76]+(msg[77]<<8)))/data_size;
              position[m][1] = ((double) (msg[78]+(msg[79]<<8)))/data_size;
            }
            
            triggers[m] = (int) (msg[73]+(msg[74]<<8));
            
            u_max[m] = ((double) (msg[71]+(msg[72]<<8)))/256;
            //printf("Robot %d, trig: %d\n", j, triggers[j]);
            
            //printf("Supervisor %d (%f, %f) (%f, %d)\n",m, position_memory[m][0][id_mem_end], position_memory[j][1][id_mem_end], t[id_mem_end], id_mem_begin);
            
            
            if (msg[80] == true){
                count++;
                //printf("channel: %d, %d\n", msg[0], count);
            }
  
            //printf("Supervisor (%d): (%f, %f)\n", msg[0]-1,  position[m][0],  position[m][1]);
            //printf("Supervisor (%d): queue length = %d\n", msg[0], wb_receiver_get_queue_length(receiver));
        
            wb_receiver_next_packet(receiver);
          // } 
        }
      
     }
     
    if (count == ROBOTS){
        check_next_step = true;
    }else{
        check_next_step = false;
    }
    
  

  done = true;
  for (i=0; i< ROBOTS;i++){
      for (int k=0;k<ROBOTS;k++){
      
           //printf("Robot %d - %d [%f] [%f] \n", k, i, (fabs(actual_position[k][0]-actual_position[i][0]) ), fabs(Bx[i][k]));
           if( !((fabs(actual_position[k][0]-actual_position[i][0]) < fabs(Bx[i][k]) +error && fabs(actual_position[k][0]-actual_position[i][0]) > fabs(Bx[i][k]) -error) && (fabs(actual_position[k][1]-actual_position[i][1]) < fabs(By[i][k]) +error && fabs(actual_position[k][1]-actual_position[i][1]) > By[i][k] -error))){
               done = false;
           }
      
      } 
  }
  
  if ( wb_robot_get_time() -start_time > 2000){
      done = true;
  }
  
  if (done){
    printf("Simulation done \n");
    fprintf(file_ptr, "%f, ", wb_robot_get_time() -start_time);
    for (int m=0; m<ROBOTS;m++){
      fprintf(file_ptr,"%f, ", u_max[m]);
      total_count = total_count + triggers[m];
    }
    fprintf(file_ptr,"%d\n", total_count);
    //printf("Total number of events %d\n", total_count);
    
    for(j=0; j<ROBOTS; j++) {
        txData[3] = true;
        wb_emitter_set_channel(emitter, j+1);
        wb_emitter_send(emitter, txData, 9+2*D*ROBOTS);
    
    }
    fclose(file_ptr);
    
    wb_supervisor_simulation_reset();
    // for(i=0;i<ROBOTS;i++){
      // wb_supervisor_node_restart_controller(robot[i]);
    // }
    wb_supervisor_node_restart_controller(wb_supervisor_node_get_from_def("Supervisor"));
  }
  
  
}

  return 0;

}

