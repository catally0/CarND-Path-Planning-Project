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

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

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

  // Define the initial lane and target speed
  double delta_time = 0.02;

  int lane = 1;
  //0: Most left lane
  //1: Middle lane
  //2: Most right lane

  //Set limit
  double speed_limit = 22.0; // unit:m/s, 50mph * 1.61 / 3.6 = 22.36 m/s
  double acc_limit = 10.0; // unit:m/s^2, as defined by project rubric
  double jerk_limit = 10.0; // unit:m/s^3, as defined by project rubric

  //Vehicle target speed in m/s
  double set_speed = 0.0;
  double target_speed = 0.0;
  double car_acc = 0.0;
  double car_jerk = 0.0;
  //Comfort level, coefficient of jerk
  double comfort_level = 0.8; 

  //Time_gap to front car
  double time_gap = 1.0; // unit: s
  //1.0,1.3,1.8,2.3


  int StateMachine = 0;
  //0: Keep in the lane (driving with set speed)
  //1: Prepare changing lane (following front car)
  //2: Changing lane left
  //3: Changing lane right

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy, &delta_time, &lane, &set_speed, 
               &speed_limit,&acc_limit,&jerk_limit,&comfort_level,&time_gap,
               &car_acc, &car_jerk, &StateMachine, &target_speed]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
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

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          int prev_size = previous_path_x.size();
          int wp_size = int(time_gap / delta_time);
          //std::cout << "Prev size:" << prev_size << std::endl;
          if(prev_size > 0) {
            car_s = end_path_s;
          }

          if(StateMachine == 0)  // Keep in the lane at set_speed
          {
            target_speed = speed_limit;
            //Check front car
            double time_gap_distance = time_gap * car_speed;
            for(int i=0; i < sensor_fusion.size(); i++) {
              double d = sensor_fusion[i][6];
              if(d < (2+4*lane+2) && d > (2+4*lane-2)) {
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                double check_speed = sqrt(vx*vx+vy*vy);
                double check_car_s = sensor_fusion[i][5];

                check_car_s += (double)prev_size*0.02*check_speed;
                if(check_car_s > car_s && (check_car_s - car_s) < 30) {
                  StateMachine = 1;
                  std::cout<<"State Change from 0 -> 1"<<std::endl;
                } 
              }
            }

            if(set_speed <= target_speed) {
              set_speed += 0.2;
            }
            

            if(set_speed < 0) {
              set_speed = 0;
            }else if(set_speed > target_speed) {
              set_speed = target_speed;
            }
            //std::cout<<set_speed<<std::endl;
            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x;
            double ref_y;
            double ref_yaw;
            double prev_car_x;
            double prev_car_y;


            if(prev_size < 2){
              ref_x = car_x;
              ref_y = car_y;
              // I think it's a bug in the Q&A video to use cos(car_yaw).
              prev_car_x = car_x - cos(car_yaw); 
              prev_car_y = car_y - sin(car_yaw);
              ref_yaw = deg2rad(car_yaw);
            }else{
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              prev_car_x = previous_path_x[prev_size - 2];
              prev_car_y = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
            }

            //std::cout<<"ref:("<<ref_x<<","<<ref_y<<")\t yaw:"<<ref_yaw<<std::endl;

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);

            int next_wp_number = 3;
            int next_wp_dist = 30;
            vector<double> next_wp;
            for(int i=1; i<=next_wp_number; i++) {
              next_wp = getXY(car_s + i*next_wp_dist, (2.0 + 4.0*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_wp[0]);
              ptsy.push_back(next_wp[1]);
              //std::cout<<next_wp[0]<<","<<next_wp[1]<<std::endl;
            }

            //std::cout<<"PTS:";
            for(int i=0; i<ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
              ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
              //std::cout<<"("<<ptsx[i]<<","<<ptsy[i]<<")"<<std::endl;
            }

            tk::spline s;
            s.set_points(ptsx, ptsy);
            
            for(int i=0; i<prev_size; i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = set_speed * time_gap; //distance travelled every second
            double target_y = s(target_x);
            double target_dist = sqrt(target_x *target_x + target_y * target_y);

            
            double N = target_dist/(0.02*set_speed);
            //std::cout<<"target_dist:"<<target_dist<<"\tN:"<<N<<std::endl;
            double x_add_on = 0;
            double x_incr = target_x/N;
            double x_point;
            double y_point;

            for(int i=1; i<=50-prev_size; i++){
              x_point = x_add_on + x_incr;
              y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

          }
          else if (StateMachine == 1) 
          {
            //Check front car
            double time_gap_distance = time_gap * car_speed;
            bool car_in_front = false;
            bool left_lane_car = false;
            bool right_lane_car = false;
            double d;
            double vx;
            double vy;
            double check_speed;
            double check_car_s;
            int current_lane_CIPV_index = -1;
            double current_lane_CIPV_dist = 1000.0;
            double current_lane_CIPV_speed = -1;
            int left_lane_CIPV_index = -1;
            double left_lane_CIPV_dist = 1000.0;
            double left_lane_CIPV_speed = -1;
            int right_lane_CIPV_index = -1;
            double right_lane_CIPV_dist = 1000.0;
            double right_lane_CIPV_speed = -1;
            for(int i=0; i < sensor_fusion.size(); i++) {
              d = sensor_fusion[i][6];
              if(d < (2+4*lane+2) && d > (2+4*lane-2)) {
                vx = sensor_fusion[i][3];
                vy = sensor_fusion[i][4];
                check_speed = sqrt(vx*vx+vy*vy);
                check_car_s = sensor_fusion[i][5];

                check_car_s += (double)prev_size*0.02*check_speed;

                if((check_car_s - car_s) > -5 && (check_car_s - car_s) < 30) {
                  if(current_lane_CIPV_dist > (check_car_s - car_s)) {
                    current_lane_CIPV_dist = check_car_s - car_s;
                    current_lane_CIPV_index = i;
                    target_speed = check_speed;
                    //std::cout<<"CIPV id:"<<i<<"\t dist:"<<current_lane_CIPV_dist<<std::endl;
                  }
                  car_in_front = true;
                }
              }
              if(lane > 0) {
                if(d < (2+4*(lane-1)+2) && d > (2+4*(lane-1)-2)) {
                  vx = sensor_fusion[i][3];
                  vy = sensor_fusion[i][4];
                  check_speed = sqrt(vx*vx+vy*vy);
                  check_car_s = sensor_fusion[i][5];

                  check_car_s += (double)prev_size*0.02*check_speed;

                  if((check_car_s > car_s && (check_car_s - car_s) < 30) || 
                     (check_car_s < car_s && (car_s - check_car_s) < 15)) {
                    left_lane_car = true;
                  }

                  if(check_car_s > car_s) {
                    if(left_lane_CIPV_dist > (check_car_s - car_s)) {
                      left_lane_CIPV_dist = check_car_s - car_s;
                      left_lane_CIPV_speed = check_speed;
                      left_lane_CIPV_index = i;
                    }
                  }
                }
              }
              if(lane < 2) {
                if(d < (2+4*(lane+1)+2) && d > (2+4*(lane+1)-2)) {
                  vx = sensor_fusion[i][3];
                  vy = sensor_fusion[i][4];
                  check_speed = sqrt(vx*vx+vy*vy);
                  check_car_s = sensor_fusion[i][5];

                  check_car_s += (double)prev_size*0.02*check_speed;

                  if((check_car_s > car_s && (check_car_s - car_s) < 30) ||
                     (check_car_s < car_s && (car_s - check_car_s) < 15)) {
                    right_lane_car = true;
                  }

                  if(check_car_s > car_s) {
                    if(right_lane_CIPV_dist > (check_car_s - car_s)) {
                      right_lane_CIPV_dist = check_car_s - car_s;
                      right_lane_CIPV_index = i;
                      right_lane_CIPV_speed = check_speed;
                    }
                  }
                }
              }        
            }
            bool left_lane_change_ok = false;

            if(car_in_front == false) {
              StateMachine = 0;
              std::cout<<"State Change from 1 -> 0"<<std::endl;
            }else if((left_lane_car == false && lane > 0) && (right_lane_car == false && lane < 2)) {
              if(left_lane_CIPV_index < 0) {
                std::cout<<"State Change from 1 -> 2"<<std::endl;
                StateMachine = 2;
                std::cout<<"Current lane:"<<lane<<std::endl;
                lane-=1;
                std::cout<<"Target lane:"<<lane<<std::endl;
              }
              else if(right_lane_CIPV_index < 0) {
                std::cout<<"State Change from 1 -> 2"<<std::endl;
                StateMachine = 2;
                std::cout<<"Current lane:"<<lane<<std::endl;
                lane+=1;
                std::cout<<"Target lane:"<<lane<<std::endl;
              }
              else{
                if((left_lane_CIPV_dist + left_lane_CIPV_speed*time_gap) >= (right_lane_CIPV_dist + right_lane_CIPV_speed*time_gap)) {
                  std::cout<<"State Change from 1 -> 2"<<std::endl;
                  StateMachine = 2;
                  std::cout<<"Current lane:"<<lane<<std::endl;
                  lane-=1;
                  std::cout<<"Target lane:"<<lane<<std::endl;
                }
                else{
                  std::cout<<"State Change from 1 -> 2"<<std::endl;
                  StateMachine = 2;
                  std::cout<<"Current lane:"<<lane<<std::endl;
                  lane+=1;
                  std::cout<<"Target lane:"<<lane<<std::endl;
                }
              }

            }else if(left_lane_car == false && lane > 0) {
              std::cout<<"State Change from 1 -> 2"<<std::endl;
              StateMachine = 2;
              std::cout<<"Current lane:"<<lane<<std::endl;
              lane-=1;
              std::cout<<"Target lane:"<<lane<<std::endl;
              
            }else if(right_lane_car == false && lane < 2) {
              std::cout<<"State Change from 1 -> 2"<<std::endl;
              StateMachine = 2;
              std::cout<<"Current lane:"<<lane<<std::endl;
              lane+=1;
              std::cout<<"Target lane:"<<lane<<std::endl;
            }

            if(set_speed <= target_speed) {
              set_speed += 0.2;
            }else{
              set_speed -= 0.2;
            }
            

            if(set_speed < 0) {
              set_speed = 0;
            }else if(set_speed > speed_limit) {
              set_speed = speed_limit;
            }
            //std::cout<<set_speed<<std::endl;
            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x;
            double ref_y;
            double ref_yaw;
            double prev_car_x;
            double prev_car_y;


            if(prev_size < 2){
              ref_x = car_x;
              ref_y = car_y;
              // I think it's a bug in the Q&A video to use cos(car_yaw).
              prev_car_x = car_x - cos(car_yaw); 
              prev_car_y = car_y - sin(car_yaw);
              ref_yaw = deg2rad(car_yaw);
            }else{
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              prev_car_x = previous_path_x[prev_size - 2];
              prev_car_y = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
            }

            //std::cout<<"ref:("<<ref_x<<","<<ref_y<<")\t yaw:"<<ref_yaw<<std::endl;

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);

            int next_wp_number = 3;
            int next_wp_dist = 30;
            vector<double> next_wp;
            for(int i=1; i<=next_wp_number; i++) {
              next_wp = getXY(car_s + i*next_wp_dist, (2.0 + 4.0*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_wp[0]);
              ptsy.push_back(next_wp[1]);
              //std::cout<<next_wp[0]<<","<<next_wp[1]<<std::endl;
            }

            //std::cout<<"PTS:";
            for(int i=0; i<ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
              ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
              //std::cout<<"("<<ptsx[i]<<","<<ptsy[i]<<")"<<std::endl;
            }

            tk::spline s;
            s.set_points(ptsx, ptsy);
            
            for(int i=0; i<prev_size; i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = set_speed * time_gap; //distance travelled every second
            double target_y = s(target_x);
            double target_dist = sqrt(target_x *target_x + target_y * target_y);

            
            double N = target_dist/(0.02*set_speed);
            //std::cout<<"target_dist:"<<target_dist<<"\tN:"<<N<<std::endl;
            double x_add_on = 0;
            double x_incr = target_x/N;
            double x_point;
            double y_point;

            for(int i=1; i<=50-prev_size; i++){
              x_point = x_add_on + x_incr;
              y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
          }else if(StateMachine == 2) {
            //Check front car
            double time_gap_distance = time_gap * car_speed;

            double d;
            double vx;
            double vy;
            double check_speed;
            double check_car_s;
            for(int i=0; i < sensor_fusion.size(); i++) {
              d = sensor_fusion[i][6];
              if(d < (2+4*lane+2) && d > (2+4*lane-2)) {
                vx = sensor_fusion[i][3];
                vy = sensor_fusion[i][4];
                check_speed = sqrt(vx*vx+vy*vy);
                check_car_s = sensor_fusion[i][5];

                check_car_s += (double)prev_size*0.02*check_speed;

                if(check_car_s > car_s && (check_car_s - car_s) < 30) {
                  target_speed = check_speed;
                }
              }
            }

            if(car_d < (2+4*lane+2) && car_d > (2+4*lane-1)){
              StateMachine = 1;
              std::cout<<"State Change from 2 -> 1"<<std::endl;
            }

            if(set_speed <= target_speed) {
              set_speed += 0.2;
            }else{
              set_speed -= 0.2;
            }
            

            if(set_speed < 0) {
              set_speed = 0;
            }else if(set_speed > speed_limit) {
              set_speed = speed_limit;
            }
            //std::cout<<set_speed<<std::endl;
            vector<double> ptsx;
            vector<double> ptsy;

            double ref_x;
            double ref_y;
            double ref_yaw;
            double prev_car_x;
            double prev_car_y;


            if(prev_size < 2){
              ref_x = car_x;
              ref_y = car_y;
              // I think it's a bug in the Q&A video to use cos(car_yaw).
              prev_car_x = car_x - cos(car_yaw); 
              prev_car_y = car_y - sin(car_yaw);
              ref_yaw = deg2rad(car_yaw);
            }else{
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];
              prev_car_x = previous_path_x[prev_size - 2];
              prev_car_y = previous_path_y[prev_size - 2];
              ref_yaw = atan2(ref_y - prev_car_y, ref_x - prev_car_x);
            }

            //std::cout<<"ref:("<<ref_x<<","<<ref_y<<")\t yaw:"<<ref_yaw<<std::endl;

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ref_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ref_y);

            int next_wp_number = 3;
            int next_wp_dist = 30;
            vector<double> next_wp;
            for(int i=1; i<=next_wp_number; i++) {
              next_wp = getXY(car_s + i*next_wp_dist, (2.0 + 4.0*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
              ptsx.push_back(next_wp[0]);
              ptsy.push_back(next_wp[1]);
              //std::cout<<next_wp[0]<<","<<next_wp[1]<<std::endl;
            }

            //std::cout<<"PTS:";
            for(int i=0; i<ptsx.size(); i++) {
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
              ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
              ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
              //std::cout<<"("<<ptsx[i]<<","<<ptsy[i]<<")"<<std::endl;
            }

            tk::spline s;
            s.set_points(ptsx, ptsy);
            
            for(int i=0; i<prev_size; i++){
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double target_x = set_speed * time_gap; //distance travelled every second
            double target_y = s(target_x);
            double target_dist = sqrt(target_x *target_x + target_y * target_y);

            
            double N = target_dist/(0.02*set_speed);
            //std::cout<<"target_dist:"<<target_dist<<"\tN:"<<N<<std::endl;
            double x_add_on = 0;
            double x_incr = target_x/N;
            double x_point;
            double y_point;

            for(int i=1; i<=50-prev_size; i++){
              x_point = x_add_on + x_incr;
              y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref*cos(ref_yaw) - y_ref*sin(ref_yaw);
              y_point = x_ref*sin(ref_yaw) + y_ref*cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }
          }

          //End


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