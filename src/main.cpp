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

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
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
	

// start reference speed
	double ref_spd = 0.0; 
	// In simulator the lane is started in middle lane (1)
  int lane =1;

  h.onMessage([&ref_spd, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
						
						// Prediction
						// Detect cars around ego car
						// The data format of sensor fusion for each car is: [ id, x, y, vx, vy, s, d]. 

						int pre_size = previous_path_x.size(); // set prediction size same as previous size
						double cirtical_space = 30.0; // critical distance for collision

						// update car_s to end path position
						if (pre_size >0){
							car_s = end_path_s;
						}

						// initialize nearby car detection flags
						bool right_car = false;
						bool  left_car = false;
						bool ahead_car = false;

						// nearby cars detction
						for ( int i = 0; i < sensor_fusion.size(); i++ ) {
							int    ID = sensor_fusion[i][0];
							double  x = sensor_fusion[i][1];
							double  y = sensor_fusion[i][2];
							double vx = sensor_fusion[i][3];
							double vy = sensor_fusion[i][4];
							double  obj_s = sensor_fusion[i][5];
							double  d = sensor_fusion[i][6];

							int objcar_lane; // object car lane
							
							// object car speed calculation 
							double objcar_speed = sqrt(vx*vx + vy*vy);
							// predict obj car position in s direction 
							obj_s += (pre_size*0.02*objcar_speed);
							// the cars which in collision free regions are not considered.
							// find cars in the same lane as ego car, assume car is in the center line of each lane
							// only cars in nearby lanes will be evaluated
							if ( d > 0 && d < 4){
								objcar_lane = 0; // left lane
							}
							else if (d >= 4 && d <= 8){
								objcar_lane = 1; // middle lane
							}
							else{
								objcar_lane = 2; // right lane
							}
							
							double s_deviation = obj_s- car_s;
							// if there is a car in same lane, check distance
							if (lane == objcar_lane){
								if (s_deviation>0  && s_deviation<= cirtical_space){
								 ahead_car = true;
								}
							}
							// more safety concern about right lane change
							else if ((objcar_lane-lane)==1 &&lane !=2){
								if (s_deviation >= -cirtical_space && s_deviation <= 1.3*cirtical_space){
								 right_car = true;
								}
							// more sensitive about left lane change
							}
							else if ((objcar_lane-lane)==-1 &&lane !=0){
								if (s_deviation >= -0.8*cirtical_space && s_deviation <= cirtical_space){
								 left_car = true;
								}
							}
						}

						// Prediction END

						// Behaviour Planning
						double spd_limit = 49.5; 
						double acc_limit = .224;
						double spd_diff = 0;
						// first check speed to avoid collision with front car
						if (ahead_car){
								//spd_diff =0;
								//tar_lane = lane;
								//  cars in right and left, slow down car (default option)

							// left turn higher priority
							if (!left_car && lane >0){
								// change to left
								//tar_lane = lane -1;
								lane--;
							}
							else if (!right_car && lane <2){
								// change to right lane
								//tar_lane = lane +1;
								lane ++;
							}
							else{
								spd_diff -= acc_limit; // slow down car
              }
						}
						// no car ahead, keep going
						else {
							spd_diff += acc_limit;
						// change lane to middle lane only in case of no nearby cars
							if (!right_car &&!left_car ){
								lane =1;
							}
						}

						// Behaviour END

						// Trajectary Generation
						// Based on decided behaviour plan trajectary, d is decided by tar_lane

						// generate x,y postions of trajectory include previous and target
						vector<double> pos_x;
          	vector<double> pos_y;

						double ref_x = car_x;
						double ref_y = car_y;
						double ref_yaw = deg2rad(car_yaw);

						double prepos_x;
						double prepos_y;

						if (pre_size <2)
						{
							// calculate previous position (1m), no yaw change 
							prepos_x = car_x - cos(car_yaw);
							prepos_y = car_y - sin(car_yaw);
						}

						else // >=2
						{
							// redefine reference points as previous path end points
							ref_x = previous_path_x[pre_size-1];
							ref_y = previous_path_y[pre_size-1];

							prepos_x = previous_path_x[pre_size-2];
							prepos_y = previous_path_y[pre_size-2];

							ref_yaw = atan2(ref_y-prepos_y,ref_x-prepos_x);
						}

						pos_x.push_back(prepos_x);
						pos_y.push_back(prepos_y);
							
						pos_x.push_back(ref_x);
						pos_y.push_back(ref_y);

						// target positions (3 way points), in frenet add evenl 30m spaced points ahead of starting reference
						vector<double> nextpos0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> nextpos1 = getXY(car_s + 30*2, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
						vector<double> nextpos2 = getXY(car_s + 30*3, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);


						pos_x.push_back(nextpos0[0]);
						pos_x.push_back(nextpos1[0]);
						pos_x.push_back(nextpos2[0]);

						pos_y.push_back(nextpos0[1]);
						pos_y.push_back(nextpos1[1]);
						pos_y.push_back(nextpos2[1]);

						// points end

						// change to car's local coordinates
						 for ( int i = 0; i < pos_x.size(); i++ )
							{
								double shift_x = pos_x[i] - ref_x;
								double shift_y = pos_y[i] - ref_y;

								pos_x[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
								pos_y[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
							}

						 // create the spline
						 tk:: spline s;
 						// set pos_x and pos_y into spline
						 s.set_points(pos_x,pos_y);

						vector<double> next_x_vals;
          	vector<double> next_y_vals;
						// make use of previous_path 
						
						for (int i=0; i<pre_size; i++){
							next_x_vals.push_back(previous_path_x[i]);
							next_y_vals.push_back(previous_path_y[i]);
						}

						// calculate distance y position on certain meters ahead
						double target_x = 30;
						double target_y = s(target_x);

						double target_dist = sqrt(target_x*target_x + target_y*target_y);
						double x_step = 0;

            for( int i = 1; i <= 50 - pre_size; i++ ) {
              ref_spd += spd_diff;
							// not above speed limitation
              if ( ref_spd > spd_limit ) {
                ref_spd = spd_limit;
              } 
							// used for debug
							else if (ref_spd < 0.2){
								ref_spd = 0.2;
							}

              double N = target_dist/(0.02*ref_spd/2.24);
              double x_point = x_step + target_x/N;
              double y_point = s(x_point);

              x_step = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
						}
						// Trajectary END

						json msgJson;

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
