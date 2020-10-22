#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>
#include <iostream>
#include "spline.h"
#include <limits>

// for convenience
using std::string;
using std::vector;

struct vehicle_state{
    double s;
    double d;
    double s_dot;
    double d_dot;
};

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
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

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
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

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

vector<double> getS_dotD_dot(double x, double y, double vx, double vy,
                         const vector<double> &maps_x,
                         const vector<double> &maps_y) {
    int next_wp = NextWaypoint(x,y, atan2(vy,vx), maps_x, maps_y);

    int prev_wp;
    prev_wp = next_wp-1;
    if (next_wp == 0) {
        prev_wp  = maps_x.size()-1;
    }

    double n_x = maps_x[next_wp]-maps_x[prev_wp];
    double n_y = maps_y[next_wp]-maps_y[prev_wp];

    // find the unit vectors pointing in the s and d directions
    Eigen::Vector2d s(n_x, n_y);
    s.normalize();
    //d is perpendicular to s in the clockwise direction
    Eigen::Vector2d d(s[1], -s[0]);
    Eigen::Vector2d v(vx,vy);

    //project v onto s and d
    double s_dot = v.dot(s);
    double d_dot = v.dot(d);
    return {s_dot,d_dot};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

//Returns the closest lane based on the lateral offset in frenet coorinates (d)
int closest_lane(double d){
  vector <double> center_of_lanes = {2.0,6.0,10.0};
  double min_dist = std::numeric_limits<double>::max();
  int closest_lane;
  for (int j = 0; j < center_of_lanes.size(); j++) {

    double lateral_distance_to_jth_lane_ego = fabs(center_of_lanes[j] - d);

    if (lateral_distance_to_jth_lane_ego < min_dist) {
      min_dist = lateral_distance_to_jth_lane_ego;
      closest_lane = j;
    }
  }
  return closest_lane;
}

//Gets the state of the closest vehicle in the same lane.
vehicle_state get_state_of_closest_vehicle_in_front(vector <vector<double>> sensor_fusion, vehicle_state ego_state, const vector<double>& map_waypoints_x,
                                                    const vector<double>& map_waypoints_y, const double max_s){
  int ego_lane = closest_lane(ego_state.d);
  vehicle_state vehicle_in_front;
  int min_idx = -1;
  double min_dist = max_s;
  for(int k = 0; k < sensor_fusion.size(); ++k) {
    double s_other = sensor_fusion[k][5];
    double d_other = sensor_fusion[k][6];
    //check if in same lane
    if(closest_lane(d_other) != ego_lane){
      continue; //Car is in a different lane
    }

    double dist_to_vehicle = s_other - ego_state.s;
    //handle wraparound
    if (dist_to_vehicle < 0){
      dist_to_vehicle = max_s + dist_to_vehicle;
    }

    if (dist_to_vehicle<=min_dist){
      min_dist = dist_to_vehicle;
      min_idx = k;
    }
  }
  if(min_idx==-1){
    /* debugging
    std::cout << "No vehicle in same lane. Own car: " << ego_lane << " Other cars: ";
    for (int j =0; j < sensor_fusion.size(); j++) {
      std::cout << closest_lane(sensor_fusion[j][6])<< ", ";
    }
    std::cout << std::endl;
     */
    return vehicle_in_front;
  } else {
    double s = sensor_fusion[min_idx][5];
    double d = sensor_fusion[min_idx][6];
    double x = sensor_fusion[min_idx][1];
    double y = sensor_fusion[min_idx][2];
    double vx = sensor_fusion[min_idx][3];
    double vy = sensor_fusion[min_idx][4];
    vector<double> v_frenet = getS_dotD_dot(x, y, vx, vy, map_waypoints_x, map_waypoints_y);
    double s_dot = v_frenet[0];
    double d_dot = v_frenet[1];
    vehicle_in_front.s = s;
    vehicle_in_front.d = d;
    vehicle_in_front.s_dot = s_dot;
    vehicle_in_front.d_dot = d_dot;
    return vehicle_in_front;
  }
}

//Returns a vector representing whether each lane is safe to switch to
vector <bool> is_lane_safe(vehicle_state ego_state, vector <vector<double>> sensor_fusion, const vector<double>& map_waypoints_x,
                           const vector<double>& map_waypoints_y, double max_s){
    vector <double> center_of_lanes = {2.0,6.0,10.0};
    double buffer_meters_front = 35;
    double buffer_meters_behind = 15;
    double s_geofence_max = ego_state.s + buffer_meters_front;
    //Handle wraparound of s (can never be greater than max_s)
    if(s_geofence_max > max_s){
      //if s_geofence is greater than max_s, we need to subtract max_s from it, as s coordinates
      //reset to 0 between the car and the geofence in front of the car
      s_geofence_max = s_geofence_max - max_s;
    }
    double s_geofence_min = ego_state.s - buffer_meters_behind;
    vector<bool> lane_safe = {true, true, true};
    for(int i = 0; i < sensor_fusion.size(); i++) {
        double s_other = sensor_fusion[i][5];
        double d_other = sensor_fusion[i][6];
        if(s_other<s_geofence_max && s_other>s_geofence_min) {
            int closest_lane_other = closest_lane(d_other);
            int closest_lane_ego = closest_lane(ego_state.d);
            if(closest_lane_ego==closest_lane_other && s_other<ego_state.s){continue;} //Never consider cars in same lane behind you
            lane_safe[closest_lane_other] = false;
        }
    }
    return lane_safe;
}



#endif  // HELPERS_H