#include "vehicle.h"
#include <algorithm>
#include <iterator>
#include <map>
#include <string>
#include <vector>
#include <iostream>

using std::string;
using std::vector;

// Initializes Vehicle
Vehicle::Vehicle(){}

Vehicle::Vehicle(int lane, float s, float v, float a, string state) {
  this->lane = lane;
  this->s = s;
  this->v = v;
  this->a = a;
  this->state = state;
  max_acceleration = -1;
}

Vehicle::~Vehicle() {}

vector<string> Vehicle::successor_states() 
{
  
  vector<string> states;
  states.push_back("KL");
  string state = this->state;
  
  if (lane_change_wait && steps_after_lane_change < 100)
  {
    this->steps_after_lane_change+=1;

  }
  else
  {
    lane_change_wait = false;
    this->steps_after_lane_change = 0;
    
    if(state.compare("KL") == 0) {
      states.push_back("PLCL");
      states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
      if (lane != 0) {
        states.push_back("PLCL");
        states.push_back("LCL");
      }
    } else if (state.compare("PLCR") == 0) {
      if (lane != 2) {
        states.push_back("PLCR");
        states.push_back("LCR");
      }
    }
  }
  // If state is "LCL" or "LCR", then just return "KL"

  return states;
}

void Vehicle::configure(vector<float> &config_data) {
  // Called by simulator before simulation begins. Sets various parameters which
  //   will impact the ego vehicle.
  target_speed = config_data[0];
  max_acceleration = config_data[1];
  num_lanes = config_data[2];
}

void Vehicle::identify_traffic_around_ego(vector<vector<double>> &sensor_fusion)
{ 
  int vehicle_behind_index = -1;
  int vehicle_ahead_index = -1;
  float max_distance_sensor = 200;
  float distance_behind = 200;
  float distance_ahead = 200;
  vector <int> cars_around;

  for (int each_lane = 0;each_lane<3;++each_lane)          
  {
    distance_behind = 300;
    distance_ahead = 300;
    vehicle_behind_index = -1;
    vehicle_ahead_index = -1;

    for(int i = 0; i < sensor_fusion.size(); i++)
    {

      float car_i_d = sensor_fusion[i][6];
      float car_i_s = sensor_fusion[i][5];


      if(car_i_d < (2+4*each_lane+2) && car_i_d > (2+4*each_lane-2))
      {
        float distance_to_car = car_i_s - this->s;

        // Cars behind
        if (distance_to_car <=0.0)
        {
          if (-distance_to_car<distance_behind)
          {
            distance_behind = -distance_to_car;
            vehicle_behind_index = i;
          }
        }

        // Cars ahead
        else
        {
          if (distance_to_car<distance_ahead)
          {
            distance_ahead = distance_to_car;
            vehicle_ahead_index = i;
          } 
        }

      }

    }
    
    Vehicle temp_vehicle_ahead, temp_vehicle_behind;
    Vehicle empty_vehicle;
    empty_vehicle.s = -1;
    empty_vehicle.horizon_s = -1;

    empty_vehicle.v = -1;
    
    double vx,vy;
    if (vehicle_ahead_index!=-1)
    {
      vx = sensor_fusion[vehicle_ahead_index][3];
      vy = sensor_fusion[vehicle_ahead_index][4];
      temp_vehicle_ahead.v = sqrt(vx*vx+vy*vy);
      temp_vehicle_ahead.s = sensor_fusion[vehicle_ahead_index][5];
      temp_vehicle_ahead.horizon_s= temp_vehicle_ahead.s + this->step_horizon*.02*temp_vehicle_ahead.v;

      temp_vehicle_ahead.lane = each_lane;
      temp_vehicle_ahead.d = sensor_fusion[vehicle_ahead_index][6];
      this->vehicles_ahead[each_lane] = temp_vehicle_ahead;
    }
    else
    {
      empty_vehicle.lane = each_lane;
      this->vehicles_ahead[each_lane] = empty_vehicle;
    }
    
    
    if (vehicle_behind_index!=-1)
    {
      vx = sensor_fusion[vehicle_behind_index][3];
      vy = sensor_fusion[vehicle_behind_index][4];
      temp_vehicle_behind.v = sqrt(vx*vx+vy*vy);
      temp_vehicle_behind.s = sensor_fusion[vehicle_behind_index][5];
      temp_vehicle_behind.horizon_s = temp_vehicle_behind.s+ this->step_horizon*.02*temp_vehicle_behind.v;

      temp_vehicle_behind.lane = each_lane;
      temp_vehicle_behind.d = sensor_fusion[vehicle_behind_index][6];
      this->vehicles_behind[each_lane] = temp_vehicle_behind;
    }
    else
    {
      empty_vehicle.lane = each_lane;
      this->vehicles_behind[each_lane] = empty_vehicle;
    }
  }
}

int Vehicle::best_lane()
{
  int fastest_lane;
  float max_speed = 0.0;
  
  for (int i =0;i<3;++i)
  {
    if (vehicles_ahead[i].v == -1)
    {
      return i; // if no car ahead, always prefer the lane from the left to right
    }
    else
    {
      if (this->vehicles_ahead[i].v > max_speed)
      {
        max_speed = vehicles_ahead[i].v;
        fastest_lane = i;
      }
    }
   }
  return fastest_lane;
}

bool Vehicle::safety_check(int lane_index)
{
  float safety_distance_ahead = 45;
  float safety_distance_behind = 20;

  bool safety_test_ahead = false;
  bool safety_test_behind = false;
  
  if (vehicles_behind[lane_index].horizon_s==-1 || this->s > vehicles_behind[lane_index].horizon_s + safety_distance_behind)
  {
    safety_test_behind=true;
  }
  else
  {
    std::cout<<"Safety warning: vehicle behind in lane "<<lane_index<<"\n";
  }
  
  
  if (vehicles_ahead[lane_index].horizon_s==-1 || this->s + safety_distance_ahead < vehicles_ahead[lane_index].horizon_s)
  {
    safety_test_ahead=true;
  }
  else
  {
    std::cout<<"Safety warning: vehicle ahead in lane "<<lane_index<<"\n";
    //std::cout<<"Vehicle ahead Horizon S = "<<vehicles_ahead[lane_index].horizon_s<<" vs. EGO Horizon_S = "<<this->horizon_s<<"\n";
  }
  

  return safety_test_ahead & safety_test_behind;
  
}

void Vehicle::calculate_best_state(vector <string> possible_states)
{
  float cost;
  float best_cost = 100000;
  float cost_function1 = 0.0;
  bool safety_test;
  int fastest_lane = best_lane();
  int state_lane, intended_lane, best_lane;
  int best_state_index;
  
  for (int each_state_index = 0; each_state_index<possible_states.size();++each_state_index)
  {
    //std::cout<<"Possible State: "<<possible_states[each_state_index]<<"\n";
    
    safety_test = true;
    state_lane = this->lane;
    intended_lane = this->lane;

    if (possible_states[each_state_index].compare("LCL")==0)
    {
      safety_test = safety_check(lane -1);
      intended_lane = this->lane -1;
      state_lane = this->lane-1;
    }
    
    if (possible_states[each_state_index].compare("LCR")==0)
    {
      safety_test = safety_check(lane +1);
      intended_lane = this->lane +1;
      state_lane = this->lane+1;
    }
    
    if (possible_states[each_state_index].compare("PLCR")==0)
    {
      intended_lane = this->lane +1;
    }
    
    if (possible_states[each_state_index].compare("PLCL")==0)
    {
      intended_lane = lane -1;
    }

// this cost function returns a low value if lane and intended lane are close to best lane = fastest lane
	
    if (safety_test)
    {
      cost = fabs(2*fastest_lane - intended_lane - state_lane);
      //std::cout<<"State: "<<possible_states[each_state_index]<<" - Cost = "<<cost<<"\n";

      if (cost < best_cost)
      {
        best_cost = cost;
        best_state_index = each_state_index;
        best_lane = state_lane;
      }
    }
    else
    {
      std::cout<<"Safety test not passed!";
    }
    
  }
  this->lane = best_lane;
  this->state = possible_states[best_state_index];
  if (this->state.compare("LCL")==0 || this->state.compare("LCR")==0)
  {
    lane_change_wait=true;
  }
      
}


