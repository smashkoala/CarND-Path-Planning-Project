#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

const double STOP_COST = 0.5;
const double BUFFER_V = 3;
const double speed_limit = 50;
const double target_speed = 47;
const double max_acc = 10;
const double s_weight = 100;
const double v_weight = 1.0;
const double a_weight = 1.0;
const double l_weight = 1.0;
const double safety_margin_s_h = 30;
const double safety_margin_s_t = -50;

struct trajectory {
  int lane;
  double s;
  double d;
  double v;
  double a;
};

struct car_position {
  double s;
  double d;
  double v;
  double a;
};


double calculate_cost_pos(trajectory future_tra , trajectory current_tra, car_position other_car)
{
  double s_diff = other_car.s - future_tra.s;
  double d_diff = abs(other_car.d - future_tra.d);
  
  double s_cost = 0.0;
  if(d_diff < 3.0 ) {
    if(s_diff < safety_margin_s_h && s_diff >= 0) {
      s_cost = 2.0 - s_diff/safety_margin_s_h;
      if(s_diff < 5) {
        printf("Collision!!\n");
      }
    } else if(s_diff < 0 && s_diff > safety_margin_s_t && future_tra.lane != current_tra.lane) {
      s_cost = 2.0 - s_diff/safety_margin_s_t;
    }
  }
  
  double total_cost = s_weight * s_cost;
  return total_cost;
}

double calculate_cost(trajectory future_tra , trajectory current_tra)
{
  double v_cost = 0.0;
  if(future_tra.v < target_speed){
    v_cost = STOP_COST*((target_speed-future_tra.v)/target_speed);
  } else if(future_tra.v >= target_speed && future_tra.v < speed_limit){
    v_cost = (future_tra.v - target_speed) / BUFFER_V;
  } else {
    v_cost = 10.0;
  }
  
  double a_cost = 0.0;
  if(abs(future_tra.a) > max_acc){
    a_cost = 1;
  }
  
  double l_cost = 0.0;
  if(future_tra.lane < 0 || future_tra.lane > 2) {
    l_cost = 1000.0;    //Impossible
  } else if(future_tra.lane != current_tra.lane) {
    if(current_tra.v < 30) {
      l_cost = 1000.0;  //Impossible
    } else {
      l_cost = 0.3;
    }
  }
  
  double total_cost =  v_weight * v_cost + a_weight * a_cost + l_weight * l_cost;
  return total_cost;
}

enum actions_turn {
  keep_lane = 0,
  left_shift,
  right_shift,
  actions_turn_end,
};

enum actions_speed {
  keep = 0,
  speed_up,
  speed_down,
  actions_speed_end,
};

struct action_combinations {
  actions_turn  turn;
  actions_speed speed;
};

action_combinations decice_next_action(double cost_list[][3]) {
  action_combinations acb;
  
  acb.turn = keep_lane;
  acb.speed = keep;
  
  double min_cost = cost_list[0][0];
  for(int act_t = keep_lane ; act_t < actions_turn_end ; act_t++) {
    for(int act_s = keep ; act_s < actions_speed_end ; act_s++) {
      if(min_cost > cost_list[act_t][act_s]) {
        acb.turn = (actions_turn)act_t;
        acb.speed = (actions_speed)act_s;
        min_cost = cost_list[act_t][act_s];
      }
    }
  }
  return acb;
}

int get_lane_number(double d) {
  int lane = 0;
  if(d > 0 && d < 4) {
    lane = 0;
  } else if(d >=4 && d < 8) {
    lane = 1;
  } else {
    lane = 2;
  }
  return lane;
}

//const double acc = 4.0;
int genetate_trajectory(const trajectory current, trajectory& future, actions_turn a_turn, actions_speed a_speed, double time){
  switch(a_speed) {
    case keep:
      future.a = 0.0;
      future.v = current.v;
      break;
    case speed_up:
      future.a = 1.0;//Assumption
      future.v = current.v + future.a * time;
      break;
    case speed_down:
      future.a = -120.0;
      future.v = current.v + future.a * time;
      break;
    default:
      return -1;
      break;
  }
  
  switch (a_turn) {
    case keep_lane:
      future.lane = current.lane;
      break;
    case right_shift:
      future.lane = current.lane + 1;
      break;
    case left_shift:
      future.lane = current.lane - 1;
      break;
    default:
      return -1;
      break;
  }
  future.s = current.s + current.v * time + future.a*time*time/2;
  future.d = 2+4*future.lane;
  
  return 0;
}

void predict_car_position(car_position current, car_position &future, double time){
  future.v = current.v + current.a*time;
  future.a = current.a;
  future.s = current.s + current.v * time + current.a * time * time / 2;
  future.d = current.d;
}

void printCostList(double cost_list[][3])
{
  for(int k = 0 ; k < 3 ; k++) {
    for(int i = 0 ; i < 3 ; i++) {
      printf("t=%d s=%d cost:%f\n", k, i, cost_list[k][i]);
    }
  }
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;
  
  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    int lane = 1;
    double target_vel = 47;
    
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];
          double ref_vel = car_speed;

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];
          int prev_size = previous_path_x.size();

          if(prev_size > 0){
            car_s = end_path_s;
          }
          
          //transisition_function(sensor_fusion_data, current_lane, car_s, car_d)
          //This should return the next state
          //Changes made by this function is lane, target_vel or speed increase
          trajectory current_tra;
          current_tra.s = car_s;
          current_tra.d = car_d;
          current_tra.v = car_speed;
          current_tra.lane = get_lane_number(current_tra.d);

          double cost_list[3][3] = { {0} };
          trajectory trajectory_list[3][3] = { {{0}} };
          //Loop through all possible actions(state)
          for(int act_t = keep_lane ; act_t < actions_turn_end ; act_t++) {
            for(int act_s = keep ; act_s < actions_speed_end ; act_s++) {
              trajectory future_tra_01 = { 0 };
              genetate_trajectory(current_tra, future_tra_01, (actions_turn)act_t,
                                  (actions_speed)act_s, 0.1);
              trajectory future_tra_05 = { 0 };
              genetate_trajectory(current_tra, future_tra_05, (actions_turn)act_t,
                                  (actions_speed)act_s, 0.5);
              
              trajectory_list[act_t][act_s] = future_tra_05;
              
              //Checking sensor fusion data of all other cars
              for(int j = 0 ; j < sensor_fusion.size() ; j++){
                car_position current_car_p;
                current_car_p.d = sensor_fusion[j][6];
                double vx = sensor_fusion[j][3];
                double vy = sensor_fusion[j][4];
                current_car_p.v = sqrt(vx*vx+vy*vy);
                current_car_p.s = sensor_fusion[j][5];
                current_car_p.a = 0;
//                car_position future_car_p_01;
//                predict_car_position(current_car_p, future_car_p_01, 0.1);
//                cost_list[act_t][act_s] += calculate_cost_pos(future_tra_01, current_tra, future_car_p_01);
                car_position future_car_p_05;
                predict_car_position(current_car_p, future_car_p_05, 0.5);
                cost_list[act_t][act_s] += calculate_cost_pos(future_tra_05, current_tra, future_car_p_05);
              }
              cost_list[act_t][act_s] += calculate_cost(future_tra_05, current_tra);
            }
          }
          
          //Execute actions
          action_combinations acb = decice_next_action(cost_list);
          lane = trajectory_list[acb.turn][acb.speed].lane;
          if(acb.speed == speed_up) {
            printf("Speeding up\n");
            if(current_tra.v < 30) {
              ref_vel = current_tra.v + 2.0;
            } else {
              ref_vel = current_tra.v + 1.0;
            }
          } else if(acb.speed == speed_down) {
            printf("Slowing down\n");
            ref_vel = current_tra.v - 0.8;
          } else {
            ref_vel = current_tra.v;
          }

          if(current_tra.lane != lane) {
            printf("Lane change!!");
            printf("Actions turn = %d speed = %d\n", acb.turn, acb.speed);
            printf("Lane No. = %d\n", lane);
            printCostList(cost_list);
            for(int j = 0 ; j < sensor_fusion.size() ; j++){
              double s = sensor_fusion[j][5];
              double d = sensor_fusion[j][6];
              printf("sensor d:%lf, s:%lf\n", d, s);
              car_position current_car_p;
              current_car_p.d = sensor_fusion[j][6];
              double vx = sensor_fusion[j][3];
              double vy = sensor_fusion[j][4];
              current_car_p.v = sqrt(vx*vx+vy*vy);
              current_car_p.s = sensor_fusion[j][5];
              current_car_p.a = 0;
              car_position future_car_p;
              predict_car_position(current_car_p, future_car_p, 0.5);
              printf("future sensor d:%lf, s:%lf\n", future_car_p.d, future_car_p.s);
            }
            printf("Current tra d:%f s:%f\n", current_tra.d, current_tra.s);
            printf("Future tra d:%f s:%f\n", trajectory_list[acb.turn][acb.speed].d, trajectory_list[acb.turn][acb.speed].s);
          }
          json msgJson;

          vector<double> ptsx;
          vector<double> ptsy;
        
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);
        
          //
          //Putting older trajectory
          //
          if(prev_size < 2)//Waypoint is less than 2
          {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);
            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);
            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {//Waypoint is 2 or more. That means we can use previous way points.
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            
            double ref_x_prev = previous_path_x[prev_size-2];
            double ref_y_prev = previous_path_y[prev_size-2];
            ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);
            
            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }
          
          //
          //Trajectory generation
          //
          vector<double> next_wp0 = getXY(car_s+30, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          
          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);
          
          for(int k = 0 ; k < ptsx.size() ; k++) {
            //Shift car angles to 0 degree
            double shift_x = ptsx[k]-ref_x;
            double shift_y = ptsy[k]-ref_y;
            
            ptsx[k] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
            ptsy[k] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
          }
          
          tk::spline s;
          
          s.set_points(ptsx, ptsy);
          
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          
        
          for(int i = 0 ; i < previous_path_x.size() ; i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          
          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);
          
          double x_add_on = 0;
          
          for(int i = 0 ; i < 50-previous_path_x.size() ; i++)
          {
            double N = (target_dist/(0.02*ref_vel/2.24));
            double x_point = x_add_on + (target_x)/N;
            double y_point = s(x_point);
            
            x_add_on = x_point;
            
            double x_ref = x_point;
            double y_ref = y_point;
            
            x_point = (x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw));
            y_point = (x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw));
            
            x_point += ref_x;
            y_point += ref_y;
            
            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }
          
          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
