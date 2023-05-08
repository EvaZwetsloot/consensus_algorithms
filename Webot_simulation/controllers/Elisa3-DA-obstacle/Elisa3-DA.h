#ifndef ELISA3DA_H__
#define ELISA3DA_H__

#endif // ELISA3DA_H__


typedef enum{
  idle,
  initialize,
  compute_sum,
  compute_formation,
  control_theta,
  control_speed,
  avoid_obstacle,
  obstacle_straight
} state;

