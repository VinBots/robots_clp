#ifndef VEHICLE_H
#define VEHICLE_H

#include <map>
#include <string>
#include <vector>

using std::map;
using std::string;
using std::vector;

class Vehicle {
 public:
  // Constructors
  Vehicle();
  Vehicle(int lane, float s, float v, float a, string state="CS");

  // Destructor
  virtual ~Vehicle();

  // Vehicle functions
  vector<string> successor_states();

  void configure(vector<float> &config_data);
  
  void identify_traffic_around_ego(vector<vector<double>> &sensor_fusion);
  
  int best_lane();
  
  bool safety_check(int lane_index);
  
  void calculate_best_state(vector <string> possible_states);



  // public Vehicle variables
  struct collider{
    bool collision; // is there a collision?
    int  time; // time collision happens
  };

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, 
                                     {"LCR", -1}, {"PLCR", -1}};

  int L = 1;

  int preferred_buffer = 30; // impacts "keep lane" behavior.
  
  int step_horizon = 47;
  
  int steps_after_lane_change=0;
  
  bool lane_change_wait;

  int lane, goal_lane, num_lanes, goal_s, lanes_available;

  float s, horizon_s, d, v, target_speed, a, max_acceleration;
  
  vector <int> traffic_around;
  
  vector <Vehicle> vehicles_behind; //1 vehicle by lane so 3 vehicles
  
  vector <Vehicle> vehicles_ahead; // 1 vehicle by lane so 3 vehicles

  string state;
};

#endif  // VEHICLE_H