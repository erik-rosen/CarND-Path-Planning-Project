#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <chrono>
#include <algorithm>
#include "gnuplot-iostream.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std::chrono;

int main() {
  Gnuplot gp;
  gp<<"set term qt font 'Arial,9'"<<std::endl;

  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // Recorded vehicle states from sensor fusion;
  string other_vehicle_states_file_ = "../data/other_vehicles.csv";
  bool record_other_vehicles = false;
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);
  std::ofstream other_vehicle_states_;
  if(record_other_vehicles){
      other_vehicle_states_.open(other_vehicle_states_file_.c_str(), std::ios::out | std::ios::app);
  }
  long last_recorded_timestamp_ms_ = 0;

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  double ref_velocity = 0;
  int lane = 1;
  bool switching_lanes = false;

  h.onMessage([&ref_velocity,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&lane,&other_vehicle_states_,&last_recorded_timestamp_ms_,&max_s,&gp,&switching_lanes]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
      //Record when data was received
      milliseconds received_timestamp_ms = duration_cast< milliseconds >(
              system_clock::now().time_since_epoch()
      );
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
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

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if(prev_size >0){
              car_s = end_path_s;
          }


          //Write the vehicle states to file to enable us to use it for training.
          //Sample at every 0.5 seconds
          //We only open the filestream at the start if record_vehicles is true
          if(other_vehicle_states_.is_open() && (received_timestamp_ms.count() - last_recorded_timestamp_ms_) > 500){
              for(int i = 0; i < sensor_fusion.size(); i++){
                  //first write the timestamp when the data was received
                  other_vehicle_states_ << received_timestamp_ms.count();
                  other_vehicle_states_ << ",";
                  for(int j = 0; j < sensor_fusion[i].size(); j++){
                      //then write the state of the vehicle
                      other_vehicle_states_ << sensor_fusion[i][j] << ",";
                  }
                  //add s_dot and d_dot to the output
                  double x = sensor_fusion[i][1];
                  double y = sensor_fusion[i][2];
                  double vx = sensor_fusion[i][3];
                  double vy = sensor_fusion[i][4];
                  vector<double> v_frenet = getS_dotD_dot(x, y, vx, vy, map_waypoints_x, map_waypoints_y);
                  double s_dot = v_frenet[0];
                  double d_dot = v_frenet[1];
                  other_vehicle_states_ << s_dot << ",";
                  other_vehicle_states_ << d_dot << "\n";
              }
              last_recorded_timestamp_ms_ = received_timestamp_ms.count();
          }

          vehicle_state current_state;
          current_state.s = j[1]["s"];
          current_state.d = j[1]["d"];
          vector<double> v_frenet = getS_dotD_dot(car_x, car_y, car_speed*sin(car_yaw), car_speed*cos(car_yaw), map_waypoints_x, map_waypoints_y);
          current_state.s_dot = v_frenet[0];
          current_state.d_dot = v_frenet[1];
          vehicle_state vehicle_in_front = get_state_of_closest_vehicle_in_front(sensor_fusion,current_state, map_waypoints_x, map_waypoints_y, max_s);
          double dist = vehicle_in_front.s - current_state.s;
          if (dist<0){
              dist = max_s + dist;
          }
          //std::cout << "Vehicle in front s: " << vehicle_in_front.s << " s_dot: " << vehicle_in_front.s_dot << " d: " << vehicle_in_front.d << " dist: " << dist << std::endl;

          enum State { track, slow_down, keep_speed_limit };
          State state = keep_speed_limit;

          if(dist<40.0){
              state = track;
          }
          if(dist<20.0){
              state = slow_down;
          }

          if(dist>=40.0){
              state = keep_speed_limit;
          }

          //std::cout << state << std::endl;

          switch (state){
              case slow_down:
                  ref_velocity -=.224*2;
              case track:
                  if(ref_velocity<vehicle_in_front.s_dot*2.24-1){
                      ref_velocity +=.224;
                  }else{
                      ref_velocity -=.224;
                  }
              case keep_speed_limit:
                  if(ref_velocity < 49.0) {
                      ref_velocity += .224;
                  }else{
                      ref_velocity -= .224;
                  }
          }

          //Predict each cars position in the future to see whether the lane will be safe to switch to.
          //vector <vector<vehicle_state>> predicted_paths = path_predictions(sensor_fusion, map_waypoints_x, map_waypoints_y);

          //Generate lane switching trajectories to see whether there will be collisions.
          /*
          if(current_state.s_dot>0.1) {
              vector<vehicle_state> trajectory_left = generate_trajectory(current_state, 2);
              vector<vehicle_state> trajectory_center = generate_trajectory(current_state, 6);
              vector<vehicle_state> trajectory_right = generate_trajectory(current_state, 10);
              if (is_safe_trajectory(predicted_paths, trajectory_left)) {
                  std::cout << "| ";
              } else {
                  std::cout << "|x";
                  //std::cout << "Left not safe" << std::endl;
              }
              if (is_safe_trajectory(predicted_paths, trajectory_center)) {
                  std::cout << "| ";
              } else {
                  std::cout << "|x";
                  //std::cout << "Center not safe" << std::endl;
              }
              if (is_safe_trajectory(predicted_paths, trajectory_right)) {
                  std::cout << "| |" << std::endl;
              } else {
                  std::cout << "|x|" << std::endl;
                  //std::cout << "Right not safe" << std::endl;
              }
          }*/


          std::vector<std::pair<double, double>> ds_pts_car = {std::make_pair(current_state.d,current_state.s)};
          gp << "set xrange [0:12]\nset yrange ["<< current_state.s - 20 <<":"<< current_state.s + 50 <<"]\n";
          std::vector<std::pair<double, double>> ds_pts_other_cars;
          for(int i = 0; i<sensor_fusion.size();i++){
            ds_pts_other_cars.push_back(std::make_pair(sensor_fusion[i][6],sensor_fusion[i][5]));
          }

          //gp << "set arrow from 2, graph 0 to 2, graph 1 lc rgb 'green'" << std::endl;
          //gp << "set arrow from 4, graph 0 to 4, graph 1 nohead dt '-'" << std::endl;
          //gp << "set arrow from 6, graph 0 to 6, graph 1 lc rgb 'green'" << std::endl;
          //gp << "set arrow from 8, graph 0 to 8, graph 1 nohead dt '-'" << std::endl;
          //gp << "set arrow from 10, graph 0 to 10, graph 1 lc rgb 'green'" << std::endl;
          //gp << "plot '-' with points title 'ego car', '-' with points title 'other cars'" << std::endl;
          //gp.send1d(ds_pts_car);
          //gp.send1d(ds_pts_other_cars);


          //Check if lanes are safe
          vector <bool> lane_status = is_lane_safe(current_state, sensor_fusion, map_waypoints_x,
            map_waypoints_y, max_s);
          if (lane_status[0]) {
                std::cout << "| ";
          } else {
                std::cout << "|x";
                //std::cout << "Left not safe" << std::endl;
          }
          if (lane_status[1]) {
              std::cout << "| ";
          } else {
              std::cout << "|x";
              //std::cout << "Center not safe" << std::endl;
          }
          if (lane_status[2]) {
              std::cout << "| |" << std::endl;
          } else {
              std::cout << "|x|" << std::endl;
              //std::cout << "Right not safe" << std::endl;
          }



          if(closest_lane(current_state.d) == lane){
            switching_lanes = false;
          }

          if (!switching_lanes && state != keep_speed_limit ){
            if(lane-1>=0 && lane_status[lane-1]){ //left lane safe
              //switch to left lane
              switching_lanes = true;
              lane--;
            }
            else if(lane+1<=2 && lane_status[lane+1]){ //left lane safe
              //switch to right lane
              switching_lanes = true;
              lane++;
            }
          }

          json msgJson;


          vector<double> ptsx;
          vector<double> ptsy;

          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(prev_size <2)
          {
              double prev_car_x = car_x - cos(car_yaw);
              double prev_car_y = car_y - sin(car_yaw);

              ptsx.push_back(prev_car_x);
              ptsx.push_back(car_x);

              ptsy.push_back(prev_car_y);
              ptsy.push_back(car_y);
          }
          else
          {
              ref_x = previous_path_x[std::min(prev_size,5)-1];
              ref_y = previous_path_y[std::min(prev_size,5)-1];

              double ref_x_prev = previous_path_x[std::min(prev_size,5)-2];
              double ref_y_prev = previous_path_y[std::min(prev_size,5)-2];
              ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);
          }

          car_s = getFrenet(ref_x,ref_y,ref_yaw, map_waypoints_x, map_waypoints_y)[0];

          vector<double> next_wp0 = getXY(car_s + 30,2.0 + 4.0 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 60,2.0 + 4.0 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 90,2.0 + 4.0 * lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //shift to car's reference frame
          for (int i = 0; i < ptsx.size(); i++){
              double shift_x = ptsx[i]-ref_x;
              double shift_y = ptsy[i]-ref_y;

              ptsx[i] = (shift_x * cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
              ptsy[i] = (shift_x * sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i =0; i <std::min(prev_size,5); i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x*target_x + target_y*target_y);

          double x_add_on = 0;

          for(int i = 0; next_x_vals.size()<80; i++){
              double N = target_dist/(0.02*ref_velocity/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref*cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }



          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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