#define EIGEN_NO_DEBUG
#include <Eigen/Core>
using namespace Eigen;
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
#include "bb_state/TwistWithID.h"
#include "bb_state/State.h"
#include "geometry_msgs/Point.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_broadcaster.h"
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>


using namespace std;

class controller
{
  vector<Vector2f> path_with_information; //x&y are pathpoint coordinates, z is rake information
  vector<Vector2f> path_with_information_rake;
  vector<int> rake_vector;
  geometry_msgs::Twist path_with_informationpoint;

  int stopping_activator;
  int sign_of_error_plat, sign_of_error_rake;

  uint8_t rake_info_;
  ros::Subscriber pose_sub;
  ros::Publisher twistvelocityoutputpub;
  double angledifference;
  double angular_vel_correction;
  float helper_absolute_velocity;
  double absolute_vel_correction;
  int i_nearest;
  int i_second_nearest;
  double desired_heading_at_i_nearest;
  double desired_heading_at_i_nearest_plus;
  double circle_radius;
  double phi_to_plus;
  double angular_velocity_;
  double rectangular_error_rake;
  double rectangular_error_platform;
  const static double absolute_velocity_ = 0.2;
  double sq_distance_to_nearest_point_on_path;
  double sq_distance_to_second_nearest_point_on_path;
  int indices_now_platform;
  int indices_now_rake;
  bool start_toggle_;
  ros::Time last_time;
  double last_angle_error;
  double last_rect_error_platform;
  double last_rect_error_rake;
  double sumplat;
  double sumang;
  double sumrake;
  double angledifferenceIntegrated;
  double recterrIntegrated_platform;
  double recterrIntegrated_rake;
  int counter_for_stopping_delay;
  bb_state::TwistWithID Velocityoutput;



  // Controller Gains:

  //Delay time in 1/25 s: (in case of application, delay has to be 1 (means no delay))
  const static int delay = 1;

  //Create vectors in order to delay outputs:
  double absolute[delay];
  double ang[delay];

  //Lookahead added in tester(in 1/25 s):
  const static int lookahead = 11;

  //Point searching ranges (compared to the nearest point of last loop (indices now))
  const static int search_range_ahead_ = 10;
  const static int search_range_back_ = 0;

  //gain must be negative:
  const static double gain_angular_err = -1;
  const static double gain_angular_err_int = 0;
  const static double gain_angular_err_der = 0;

  //gain must be positive:
  const static double gain_rect_err_platform = 2;
  const static double gain_rect_err_platform_int = 0;
  const static double gain_rect_err_platform_der = 0;

  //gain rake
  const static double gain_rake_err = 0;
  const static double gain_rake_err_int = 0;
  const static double gain_rake_err_der = 0;

  //lookahead variable: (Designs the "lookahead" in default velocity prescriptions)
  const static int magic_number = 3;

  //Rake offset for calculation, in case if rake path is included into control:
  const static double rake_offset_length = 0.2;

  //End of Controller Gains..



public:

  void find_closest_and_second_closest(const vector<Vector2f> & path, Vector2f pose, 
    int index_now, int search_range_ahead_applied, Vector2f * pose_xy_help, 
    Vector2f * xy_nearest, Vector2f * xy_second_nearest) {
    //Iterate the points in order to find the closest point to the 
    // robots current position:
    sq_distance_to_nearest_point_on_path = 10000000000000;
    sq_distance_to_second_nearest_point_on_path = 100000000000000;


    for(int i = index_now - search_range_back_; 
            i < index_now + search_range_ahead_applied; i++) {
        double temp_var_for_distance = (*pose_xy_help - path_with_information[i]).squaredNorm();
        if(temp_var_for_distance < sq_distance_to_nearest_point_on_path){
            sq_distance_to_nearest_point_on_path = temp_var_for_distance;
            *xy_nearest = path_with_information[i];
            i_nearest = i;
        }
    }

    //find closest neighbour
    double distance_neighbours[2];
    if(i_nearest > 0) {
        distance_neighbours[0] = (*pose_xy_help - path_with_information[i_nearest - 1]).squaredNorm();
    }
    else {
        distance_neighbours[0] = 10000000;
    }

    distance_neighbours[1] = (*pose_xy_help - path_with_information[i_nearest + 1]).squaredNorm();

    if(distance_neighbours[0] <= distance_neighbours[1] && stopping_activator == 1) {
        i_second_nearest = i_nearest - 1;
        sq_distance_to_second_nearest_point_on_path = distance_neighbours[0];
    }
    else {
        i_second_nearest = i_nearest + 1;
        sq_distance_to_second_nearest_point_on_path = distance_neighbours[1];
    }
    *xy_second_nearest = path_with_information[i_second_nearest];
  }

  void pose_sub_Callback(const geometry_msgs::PoseStamped &laserpose) {

    // Floats should be sufficient for all measurements (~6 )
    //Get Localization Measurements:
    Vector2f pose (laserpose.pose.position.x, laserpose.pose.position.y);   

    double pose_theta = tf::getYaw(laserpose.pose.orientation);

    //Lookahead pose:
    pose_theta += (ang[0] * ((float)1 / 25.0)) * lookahead;
    Vector2f pose_xy = lookahead * (absolute[0] * ((float)1 / 25.0) * Vector2f(cos(pose_theta), sin(pose_theta)));

    //Support Variables, assigned to platform pose or rake pose respectively:

    //More supporting Variables
    double angle_of_line_between_two_nearest_points;
    double angle_of_line_between_two_nearest_points_specialplatform;
    int sign_of_error = 0;
    double angle_at_nearest_point;
    vector<geometry_msgs::Twist> path_with_information_help;
    double rectangular_error_help;
    int indices_now_help;
    int function_mode;



    // In the following the modes for turning on the stop are run through. 
    // The velocity commands, that are corresponding to the modes can be found later in this script.
    // Mode 5 is for robot to wait for a second, in order to avoid bad lookahead influences:

    if(rake_vector[indices_now_rake] == 128 && stopping_activator == 1){
      stopping_activator = 5;
    }

    if(stopping_activator == 5){
      counter_for_stopping_delay++;
      if(counter_for_stopping_delay > 25){
          stopping_activator = 2;
      }
    }

    //Mode two: Angular velocity = 0, Absolute Velocity = 0.1/-0.1 (finding the good spot to turn):
    if(stopping_activator == 2){
        //berechne ob vor oder hinter dem Pfad (verlängerung des Pfadanfangs!!), dann fahr mit 0.1 in diese Richtung
    }

    if(std::abs(rectangular_error_platform) < 0.01 && stopping_activator == 2){
        if (std::abs(rectangular_error_platform) < 0.01) ROS_WARN("Nette Worte");
        stopping_activator = 3;
    }

    //Mode three: Absolute velocity = 0, robot turns on the spot, 
    // only using angular velocity corrections of the controller:
    if(stopping_activator == 3){
    }

    if(abs(angledifference) < 0.1 and stopping_activator == 3){
      stopping_activator = 4;
      counter_for_stopping_delay = 0;
    }

    //Mode four: Fourth mode is normal path following without rake movements.
    if(stopping_activator == 4){
        //do whatever!! if platform index right continue to draw!!
    }

    //if(stopping_activator == 4 && int(path_with_information_rake[indices_now_rake-1].linear.z) == 0 && path_with_information_rake[indices_now_rake].linear.z != 0 || stopping_activator == 4 && path_with_information_rake[indices_now_rake+5].linear.z != 128.0 && path_with_information_rake[indices_now_rake+4].linear.z != 128.0){
    if(stopping_activator == 4 && i_second_nearest > indices_now_platform 
       || (stopping_activator == 4 && indices_now_rake > indices_now_platform)) 
    {
      stopping_activator = 1;
      indices_now_platform = indices_now_rake;
    }
    
    Vector2f pose_xy_help;

    //Two cases: Rake and Platform: (Both done once per loop)
    //function mode: signal for functions to recognise if rake or platform path is treated
  	int k = 0;
    for(;k<2;k++){
      if(k == 0){
        pose_xy_help = pose_xy;
        indices_now_help = indices_now_platform;
        function_mode = 0;
      }
      else{
        pose_xy_help = pose_xy - Vector2f(cos(pose_theta), sin(pose_theta)) * rake_offset_length;
        indices_now_help = indices_now_rake;
        function_mode = 1;
      }

      //Find the nearest point on the path: (High values are set to ensure that in comparison-if statements, the value will be higher):
      int search_range_ahead_applied;

      //First search is in on the entire path: (executed once only)
      if(start_toggle_ == true){
          indices_now_help = search_range_back_ ;
          search_range_ahead_applied = (int)(path_with_information.size() - indices_now_help) / 50;
          start_toggle_ = false;
      }
      else{
          search_range_ahead_applied = search_range_ahead_;
          if(indices_now_help < search_range_back_){
              indices_now_help = search_range_back_;
          }
      }

      Vector2f xy_nearest;
      Vector2f xy_second_nearest;

      // angle of line between nearest points for platform will 
      // be needed separately in later code:
      // Function angle_of_line_b.... is defined before main.


      if(k == 0) {
        find_closest_and_second_closest(path_with_information, pose_xy, 
          indices_now_help, search_range_ahead_applied, &pose_xy_help, &xy_nearest, &xy_second_nearest);
        angle_of_line_between_two_nearest_points_specialplatform = angle_of_line_between_points(i_nearest, i_second_nearest, function_mode);
        angle_of_line_between_two_nearest_points = angle_of_line_between_two_nearest_points_specialplatform;
      } else {
        find_closest_and_second_closest(path_with_information_rake, pose_xy, 
          indices_now_help, search_range_ahead_applied + 20, &pose_xy_help, &xy_nearest, &xy_second_nearest);
        angle_of_line_between_two_nearest_points = angle_of_line_between_points(i_nearest, i_second_nearest, function_mode);
      }

      //Calculate rectangular offset Error:
      double rectangular_error_abs;
      double distance_between_two_nearest_points = (xy_nearest - xy_second_nearest).squaredNorm();

      // If argument to supress a certain case
      if(sq_distance_to_nearest_point_on_path < 0.00001){
        rectangular_error_abs = 0;
        angle_at_nearest_point = 0;
      } else {
        //Cosine Law for rectangular error:
        angle_at_nearest_point = acos((sq_distance_to_nearest_point_on_path -
                                       sq_distance_to_second_nearest_point_on_path +
                                       distance_between_two_nearest_points) /
                                       (2 * sq_distance_to_nearest_point_on_path));

        rectangular_error_abs = std::abs(sin(angle_at_nearest_point) * 
                                std::sqrt(sq_distance_to_nearest_point_on_path));

        ROS_INFO("recterrabs: %f, angleatn: %f",rectangular_error_abs, angle_at_nearest_point);

        // Determine on which side of the path(left or right) according 
        // to the platforms heading the robot is:
        if (cos(angle_of_line_between_two_nearest_points) > 0){
          cout << "1" << endl;
          if ((xy_nearest.y() + tan(angle_of_line_between_two_nearest_points) * 
               (pose_xy_help.x() - xy_nearest.x())) <= pose_xy_help.y()) {
            sign_of_error = -1;
          }
          else sign_of_error = 1;
        }
        else if(cos(angle_of_line_between_two_nearest_points) < 0){
          cout << "2" << endl;
          if ((xy_nearest.y() + tan(angle_of_line_between_two_nearest_points) * 
               (pose_xy_help.x() - xy_nearest.x())) <= pose_xy_help.y()) {
            sign_of_error = 1;
          }
          else sign_of_error = -1;
        }
        else if( fmod( abs(angle_of_line_between_two_nearest_points), (2*M_PI)) - M_PI/2 < 0.00001){
          cout << "3" << endl;
          if (xy_nearest.x() < pose_xy_help.x()){
            sign_of_error = -1;
          }
          else sign_of_error = 1;
        }
        else{
          cout << "4" << endl;
          if (xy_nearest.x() < pose_xy_help.x()){
            sign_of_error = 1;
          }
          else sign_of_error = -1;
        }
      }

      rectangular_error_help = rectangular_error_abs * sign_of_error;

      //Assign rectangular error, input for controller:
      if(k == 0){
        rectangular_error_platform = rectangular_error_help;
        indices_now_platform = i_nearest;
        sign_of_error_plat = sign_of_error;
      }
      else{
        rectangular_error_rake = rectangular_error_help;
        indices_now_rake = i_nearest;
        sign_of_error_rake = sign_of_error;
      }
    }

    //Same: Great number for start in copmparing if argument:
    double refangle = 10000000000000;

    //Smallest angle is wanted to be measured
    for(int v=0;v<3; v++){
        if(abs((
            fmod(pose_theta, 2 * M_PI) - fmod(angle_of_line_between_two_nearest_points_specialplatform, 2 * M_PI)) + 
            ((v - 1) * 2 * M_PI)) < abs(refangle)){
            refangle = (fmod(pose_theta, 2 * M_PI) 
              - fmod(angle_of_line_between_two_nearest_points_specialplatform, 2 * M_PI))
              + ((v - 1) * 2 * M_PI);
        }
    }

    //Class variable angledifference is input for controller:
    angledifference = refangle;

    //Get the right angular velocity:
    double ang1, absolute1;
    ang1 = default_velocity_input() + get_velocity_corrections();

    //Absolute velocity is assigned to constant velocity gain, specified on top:
    absolute1 = absolute_velocity_;

    if(stopping_activator == 3){
        absolute1 = 0;
    }

    if(stopping_activator == 1){
        ang1 = default_velocity_input() + get_velocity_corrections();
    }
    else{
        ang1 = get_velocity_corrections();
    }

    if(stopping_activator == 2){
        if(0.12 < abs(angledifference) && abs(angledifference) < M_PI/2.0-0.12){
            ang1 = 0;
        }
        else{
            ang1 = - (angledifference - 1.57);
        }
        if(sign_of_error_plat == 1 && angledifference < 0){helper_absolute_velocity=-0.1;}
        else if(sign_of_error_plat == 1 && angledifference > 0){helper_absolute_velocity=0.1;}
        else if(sign_of_error_plat == -1 && angledifference > 0){helper_absolute_velocity=-0.1;}
        else if(sign_of_error_plat == -1 && angledifference < 0){helper_absolute_velocity=0.1;}

        absolute1 = helper_absolute_velocity;
    }

    if(stopping_activator == 5){
        absolute1 = ang1 = 0;
    }

    //(Quotient of absolute vel and ang vel forms local circle radius)
    //Saturation for angular velocity, not to get high values: (In case if ang is in saturation, abs is adjusted in such a way, that curvature is constant)
    if(abs(ang1) > 0.4){
      double radius = absolute1 / ang1;
      ang1 = (float)ang1 / abs(ang1) * 0.4;
      absolute1 = ang1 * radius;
    }

    //DELAY: (Continuous vector shifting in order to produce testing delay)
    for(int i = 0; i < delay; ++i){
        if(i < delay -1){
            absolute[i] = absolute[i+1];
            ang[i] = ang[i+1];
        }
        else{
            absolute[i] = absolute1;
            ang[i] = ang1;
        }
    }

    //STOP COMMAND AT THE END (of path):
    if(indices_now_platform > path_with_information.size() - 1 || rake_vector[indices_now_rake] == 129){
      ang[0] = 0;
      absolute[0] = 0;
    }

    //Creating message to be published:
    Velocityoutput.twist.linear.x = absolute[0];
    Velocityoutput.twist.angular.z = ang[0];
    Velocityoutput.id = 3;

    if(k >= 1){
        unsigned int temp_unsigned;
        if(stopping_activator == 2 && absolute1 > 0){
            temp_unsigned = (unsigned int)rake_vector[indices_now_rake - 20];
        }
        else if(stopping_activator == 2 && absolute < 0) {
            temp_unsigned = 0;
        }
        else if(stopping_activator == 5 || stopping_activator == 3) {
            temp_unsigned = 0;
        }
        else if(stopping_activator == 4) {
            temp_unsigned = (unsigned int)rake_vector[indices_now_platform];
        }
        else {
          temp_unsigned = (unsigned int)rake_vector[indices_now_rake];
        }

        ROS_INFO("before pub: %u", temp_unsigned);
        uint8_t temp_uint = temp_unsigned;
        ROS_INFO("before pub: %d", temp_uint);
        ROS_INFO("index platform: %d", indices_now_platform);
        ROS_INFO("absolute: %f, angular: %f", absolute1, ang1);

        twistvelocityoutputpub.publish(Velocityoutput);
    }
  }
  
  //Read in the path:
  void ReadFile(const char *filename, int moderead) {
    std::ifstream infile(filename);
    if (infile.is_open()) {
      std::string line;
      while(getline(infile, line)) {
        std::istringstream iss(line);
        int temp_rake;
        float x, y;
        if (!(iss >> x >> y >> temp_rake)) {
          ROS_WARN("Du kannst nicht lesen?");
        }
        ROS_INFO("Read Rake Info %i", temp_rake);
        Vector2f path_with_informationpoint(x, y);
        if(moderead == 0){
          path_with_information.push_back(path_with_informationpoint);
        }
        else{
          path_with_information_rake.push_back(path_with_informationpoint);
          rake_vector.push_back(temp_rake);
        }
      }
    }
    else ROS_WARN("failed to open file");
    infile.close();
  }

  //Actual controller:
  double get_velocity_corrections(){

    angledifferenceIntegrated  += angledifference;
    recterrIntegrated_platform += rectangular_error_platform;
    recterrIntegrated_rake     += rectangular_error_rake;

    ros::Time now_time = ros::Time::now();
    double deltatime = (now_time - last_time).toSec();
    last_time = now_time;

    //Calculate derivatives:

    double dangle = (angledifference - last_angle_error) / deltatime;
    double drectplat = (rectangular_error_platform - last_rect_error_platform) / deltatime;
    double drectrake = (rectangular_error_rake - last_rect_error_rake) / deltatime;

    //Assign last step values for next step:
    last_angle_error = angledifference;
    last_rect_error_platform = rectangular_error_platform;
    last_rect_error_rake = rectangular_error_rake;

    absolute_vel_correction = gain_rect_err_platform * rectangular_error_platform + gain_rect_err_platform_int * recterrIntegrated_platform + gain_rect_err_platform_der * drectplat;

    angular_vel_correction = gain_angular_err * angledifference + gain_angular_err_int * angledifferenceIntegrated + dangle * gain_angular_err_der;

    double ang_vel_correction_from_rake = gain_rake_err * rectangular_error_rake;

    //For Comparison, summed up error:
    sumplat +=  abs(rectangular_error_platform);
    sumang  +=  abs(angledifference);
    sumrake +=  abs(rectangular_error_rake);

    return (angular_vel_correction + absolute_vel_correction + ang_vel_correction_from_rake);
  }

  double default_velocity_input(){

    int i_nearest_plus = indices_now_platform + magic_number;
    //ROS_INFO("indices_now_platform %d", indices_now_platform);
    //To avoid segmentation faults at the beginning:
    if(indices_now_platform == 0) indices_now_platform = 1;

    if( abs( angle_of_line_between_points( indices_now_platform + 1, indices_now_platform, 0) - 
          angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0)) > M_PI )
      desired_heading_at_i_nearest = (angle_of_line_between_points(indices_now_platform + 1, indices_now_platform, 0) - 
                                      angle_of_line_between_points(indices_now_platform, indices_now_platform - 1, 0)) / 2;
    else
      desired_heading_at_i_nearest = (angle_of_line_between_points(indices_now_platform + 1, indices_now_platform, 0) + 
                                      angle_of_line_between_points(indices_now_platform, indices_now_platform - 1, 0)) / 2;

    if( abs( angle_of_line_between_points(i_nearest_plus + 1, i_nearest_plus, 0) - 
             angle_of_line_between_points(i_nearest_plus,i_nearest_plus - 1, 0)) > M_PI)
      desired_heading_at_i_nearest_plus = (angle_of_line_between_points(i_nearest_plus + 1, i_nearest_plus, 0) - 
                                           angle_of_line_between_points(i_nearest_plus, i_nearest_plus - 1, 0)) / 2;
    else
      desired_heading_at_i_nearest_plus = (angle_of_line_between_points(i_nearest_plus + 1, i_nearest_plus, 0) + 
                                           angle_of_line_between_points(i_nearest_plus, i_nearest_plus - 1, 0)) / 2;
    
    phi_to_plus = desired_heading_at_i_nearest_plus - desired_heading_at_i_nearest;

    double refphi = 1000000000000000000;

    for(int v=0;v<9; v++){
        if(abs(phi_to_plus + (v - 4) * 2 * M_PI) < abs(refphi)){
            refphi = phi_to_plus + (v - 4) * 2 * M_PI;
        }
    }

    phi_to_plus = refphi;
    double chord_length = (path_with_information[indices_now_platform] - path_with_information[i_nearest_plus]).norm();

    circle_radius = chord_length / (2 * sin(phi_to_plus/2));

    //Assign angular velocity:
    angular_velocity_ = absolute_velocity_ / circle_radius + angular_vel_correction + absolute_vel_correction;

    return angular_velocity_;
  }

  double angle_of_line_between_points(int i1, int i2, int mode){
    Vector2f p1;
    Vector2f p2;
    if(mode == 0){
      p1 = path_with_information[i1];
      p2 = path_with_information[i2];
    }
    else{
      p1 = path_with_information_rake[i1];
      p2 = path_with_information_rake[i2];
    }

    if (i1 > i2){
      Vector2f p = p1 - p2;
      return atan2(p.x(), p.y());
    }
    else {
      Vector2f p = p2 - p1;
      return atan2(p.x(), p.y());
    }
  }
  
  controller()
  {
    ros::NodeHandle n;
    pose_sub = n.subscribe("/localization/bot_pose",1000, &controller::pose_sub_Callback, this);
    twistvelocityoutputpub = n.advertise<bb_state::TwistWithID>("/move_io", 100, true);
    ros::Publisher statepub;
    statepub = n.advertise<bb_state::State>("robot_state", 100, true);
    bb_state::State state_msg;
    state_msg.state = 3;
    statepub.publish(state_msg);
    //this->ReadFile("/home/beachbot/server/PathToStart.path", 0);
    //this->ReadFile("/home/beachbot/server/PathToStart.path", 1);
    this->ReadFile("/home/beachbot/Paths/heartypath.txt", 0);
    this->ReadFile("/home/beachbot/Paths/heartrake.txt", 1);
    start_toggle_ = true;
    angledifferenceIntegrated = 0;
    recterrIntegrated_platform = 0;
    recterrIntegrated_rake = 0;
    sumplat = 0;
    sumang = 0;
    sumrake = 0;
    stopping_activator = 0;
  }
  
  ~controller()
  {
      //Publish one zero message when destructing:
      Velocityoutput.twist.linear.x = 0;

      Velocityoutput.twist.angular.z = 0;
      Velocityoutput.rake = 0;
      Velocityoutput.id = 3;

      twistvelocityoutputpub.publish(Velocityoutput);
  }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "Controller");
    controller *controllerinstance = new controller();

    double time = ros::Time::now().toNSec();

    ros::Rate loop_rate(25);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    delete controllerinstance;
    return 0;
}
