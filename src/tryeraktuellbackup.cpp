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
  vector<geometry_msgs::Twist> path_with_information; //linear.x&y are pathpoint coordinates, linear.z is rake information
  vector<geometry_msgs::Twist> path_with_information_rake;
  geometry_msgs::Twist path_with_informationpoint;
  uint8_t rake_info_;
  ros::Subscriber pose_sub;
  ros::Publisher twistvelocityoutputpub;
  double angledifference;
  double angular_vel_correction;
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
  const static double absolute_velocity_ = 0.25;
  double absolute1before;

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
  bb_state::TwistWithID Velocityoutput;
  //geometry_msgs::Twist Velocityoutput;
  int stopping_activator;
  int sign_of_error;
  int counter_for_stopping_delay;

  // Controller Gains:

    //Delay time in 1/25 s: (in case of application, delay has to be 1 (means no delay))
    const static int delay = 1;

    //Create vectors in order to delay outputs:
    double absolute[delay];
    double ang[delay];


    //Lookahead added in tester(in 1/25 s):
    const static int lookahead = 11;

    //Point searching ranges (compared to the nearest point of last loop (indices now))
    const static int search_range_ahead_ = 8;
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
    const static double rake_offset_length = 0.23;

  //End of Controller Gains..



public:

  void pose_sub_Callback(const geometry_msgs::PoseStamped &laserpose)
  {
    //Get Localization Measurements: (No lookahead for rake)
    double pose_x_nolookahead;
    double pose_y_nolookahead;
    double pose_theta_nolookahead;
    double pose_x = pose_x_nolookahead = laserpose.pose.position.x;
    double pose_y = pose_y_nolookahead = laserpose.pose.position.y;
    //pose_theta = laserpose.pose.position.z; (Was for Testing!!, perhaps still used somtime..)
    double pose_theta = pose_theta_nolookahead = tf::getYaw(laserpose.pose.orientation);
    double helping_counter = 0;

    //Lookahead pose:
    for(int u = 0; u < lookahead; u ++){
        pose_theta += ang[0] * ((float)1 / 25.0);
        pose_x += absolute[0] * ((float)1 / 25.0) * cos(pose_theta);
        pose_y += absolute[0] * ((float)1 / 25.0) * sin(pose_theta);
    }
    //End lookahead pose..

    //Support Variables, assigned to platform pose or rake pose respectively:
    double pose_x_help;
    double pose_y_help;
    double pose_theta_help;

    //More supporting Variables
    double angle_of_line_between_two_nearest_points;
    double angle_of_line_between_two_nearest_points_specialplatform;
    sign_of_error = 0;
    double angle_at_nearest_point;
    vector<geometry_msgs::Twist> path_with_information_help;
    double rectangular_error_help;
    int indices_now_help;
    int function_mode;
    double helper_absolute_velocity;
    double sign_of_error_plat;
    double sign_of_error_rake;


    //Stopping in order to do a sharp turn!!
    //$$$ AT LEAST += POINTS HAVE TO BE BETWEEN TWO SHARP EDGES! $$$
    int helping_index = indices_now_platform - 7;
    int index;


    if(stopping_activator != 1){
        bool toggler = true;

        while(toggler == true){
//            ROS_INFO("searching");
//            ROS_INFO("pathrakeinfo: %d, helping index: %d", int(path_with_information_rake[helping_index].linear.z), helping_index);
//            if(path_with_information_rake[helping_index].linear.z != 128.0){
//                ROS_INFO("first holds");
//            }
//            if(path_with_information_rake[helping_index - 1].linear.z == 128.0){
//                ROS_INFO("second holds");
//            }
            if(path_with_information_rake[helping_index-1].linear.z != 128.0 && path_with_information_rake[helping_index - 2].linear.z == 128.0){
                toggler = false;
            }
            index = helping_index;
            helping_index = helping_index + 1;

        }
        indices_now_platform = index;

    }


    if(path_with_information_rake[indices_now_rake].linear.z == 128 && stopping_activator == 1){
        stopping_activator = 5;
    }

    if(stopping_activator == 5){
        counter_for_stopping_delay++;
        if(counter_for_stopping_delay > 25){
            stopping_activator = 2;
        }
    }
    if(stopping_activator == 2){


        //berechne ob vor oder hinter dem Pfad (verlängerung des Pfadanfangs!!), dann fahr mit 0.1 in diese Richtung
    }
    if(std::abs(rectangular_error_platform) < 0.01 && stopping_activator == 2){
        if (std::abs(rectangular_error_platform) < 0.01) ROS_WARN("Nette Worte");
        stopping_activator = 3;
    }

    //2 ist zu 3 geworden, -> anpassen!!
    if(stopping_activator == 3){


    }
    if(abs(angledifference) < 0.3 and stopping_activator == 3){
        stopping_activator = 4;
        counter_for_stopping_delay = 0;
    }
    if(stopping_activator == 4){
        //do whatever!! if platform index right continue to draw!!

    }
    //if(stopping_activator == 4 && int(path_with_information_rake[indices_now_rake-1].linear.z) == 0 && path_with_information_rake[indices_now_rake].linear.z != 0 || stopping_activator == 4 && path_with_information_rake[indices_now_rake+5].linear.z != 128.0 && path_with_information_rake[indices_now_rake+4].linear.z != 128.0){
    if(stopping_activator == 4 && i_second_nearest > indices_now_platform || (stopping_activator == 4 && indices_now_rake > indices_now_platform)){

        stopping_activator = 1;
        indices_now_platform = indices_now_rake;
        helping_counter = 0;
    }
    //Two cases: Rake and Platform: (Both done once per loop)
    //function mode: signal for functions to recognise if rake or platform path is treated
    //



    ROS_WARN("hierhin gekommen!!!");
	int k = 0;
    for(;k<2;k++){

        if(k == 0){
            pose_x_help = pose_x;
            pose_y_help = pose_y;
            pose_theta_help = pose_theta;
            path_with_information_help = path_with_information;
            indices_now_help = indices_now_platform;
            function_mode = 0;
        }
        else{
            pose_theta_help = pose_theta;
            pose_x_help = pose_x - cos(pose_theta_help) * rake_offset_length;
            pose_y_help = pose_y - sin(pose_theta_help) * rake_offset_length;
            path_with_information_help = path_with_information_rake;
            indices_now_help = indices_now_rake;
            function_mode = 1;
        }
        //Find the nearest point on the path: (High values are set to ensure that in comparison-if statements, the value will be higher):
        double distance_to_nearest_point_on_path = 10000000000000;
        double distance_to_second_nearest_point_on_path = 100000000000000;
        int search_range_ahead_applied_;
        //First search is in on the entire path: (executed once only)
        if(start_toggle_ == true){
            indices_now_help = search_range_back_ ;
            search_range_ahead_applied_ = (int)(path_with_information_help.size() - indices_now_help) / 70;
            start_toggle_ = false;
        }
        else{
            search_range_ahead_applied_ = search_range_ahead_;

            if(indices_now_help < search_range_back_){
                indices_now_help = search_range_back_;
            }
        }

        double x_nearest;
        double y_nearest;
        double x_second_nearest;
        double y_second_nearest;


        //Enlarge the rake informations search range for better performance in sharp edges!!
        if(k==1){
            search_range_ahead_applied_ += 20;
        }





        //Iterate the points in order to find the closest point to the robots current position:
        for(int i=indices_now_help - search_range_back_; i < indices_now_help + search_range_ahead_applied_; i++){
            double temp_var_for_distance = sqrt(
                (pose_x_help - path_with_information_help[i].linear.x) *
                (pose_x_help - path_with_information_help[i].linear.x) +
                (pose_y_help - path_with_information_help[i].linear.y) *
                (pose_y_help - path_with_information_help[i].linear.y)
            );

            if(temp_var_for_distance < distance_to_nearest_point_on_path){
                distance_to_nearest_point_on_path = temp_var_for_distance;
                x_nearest = path_with_information_help[i].linear.x;
                y_nearest = path_with_information_help[i].linear.y;
                i_nearest = i;
            }
        }







        //find closest neighbour
        double distance_neighbours[2];
        if(i_nearest > 0){
            distance_neighbours[0]=sqrt(
                (pose_x_help - path_with_information_help[i_nearest-1].linear.x) *
                (pose_x_help - path_with_information_help[i_nearest-1].linear.x) +
                (pose_y_help - path_with_information_help[i_nearest-1].linear.y) *
                (pose_y_help - path_with_information_help[i_nearest-1].linear.y));
            }
        else{
            distance_neighbours[0] = 10000000;
        }

        distance_neighbours[1]=sqrt(
                (pose_x_help - path_with_information_help[i_nearest+1].linear.x) *
                (pose_x_help - path_with_information_help[i_nearest+1].linear.x) +
                (pose_y_help - path_with_information_help[i_nearest+1].linear.y) *
                (pose_y_help - path_with_information_help[i_nearest+1].linear.y));



        if(distance_neighbours[0]<=distance_neighbours[1] && stopping_activator == 1)
        {
            i_second_nearest=i_nearest-1;
            if(k==0){

            }
            x_second_nearest = path_with_information_help[i_second_nearest].linear.x;
            y_second_nearest = path_with_information_help[i_second_nearest].linear.y;
            distance_to_second_nearest_point_on_path = distance_neighbours[0];
        }
        else
        {

            i_second_nearest=i_nearest+1;
            x_second_nearest = path_with_information_help[i_second_nearest].linear.x;
            y_second_nearest = path_with_information_help[i_second_nearest].linear.y;
            distance_to_second_nearest_point_on_path = distance_neighbours[1];

        }




        //angle of line between nearest points for platform will be needed separately in later code:
        //Function angle_of_line_b.... is defined before main.
        if(k == 0){
            angle_of_line_between_two_nearest_points_specialplatform = angle_of_line_between_points(i_nearest, i_second_nearest, function_mode);
            angle_of_line_between_two_nearest_points = angle_of_line_between_two_nearest_points_specialplatform;
        }
        else{
            angle_of_line_between_two_nearest_points = angle_of_line_between_points(i_nearest, i_second_nearest, function_mode);
        }

        //Calculate rectangular offset Error:
        double rectangular_error_abs;

        double distance_between_two_nearest_points = sqrt((x_nearest-x_second_nearest)*(x_nearest-x_second_nearest)+
                                                          (y_nearest-y_second_nearest)*(y_nearest-y_second_nearest));

        //If argument tu supress a certain case..
        if(distance_to_nearest_point_on_path < 0.00001){
            rectangular_error_abs = 0;
            angle_at_nearest_point = 0;
        }
        else{
            //Cosine Law for rectangular error:
            angle_at_nearest_point = acos((distance_to_nearest_point_on_path * distance_to_nearest_point_on_path -
                             distance_to_second_nearest_point_on_path * distance_to_second_nearest_point_on_path +
                             distance_between_two_nearest_points * distance_between_two_nearest_points) /
                            (2 * distance_to_nearest_point_on_path * distance_between_two_nearest_points));

            rectangular_error_abs = abs(sin(angle_at_nearest_point) * distance_to_nearest_point_on_path);


            //Determine on which side of the path(left or right) according to the platforms heading the robot is:
            if (cos(angle_of_line_between_two_nearest_points) > 0){
                cout << "1" << endl;
                if ((y_nearest+tan(angle_of_line_between_two_nearest_points)*(pose_x_help-x_nearest)) <= pose_y_help){
                    sign_of_error = -1;
                }
                else sign_of_error = 1;
            }
            else if(cos(angle_of_line_between_two_nearest_points) < 0){
                cout << "2" << endl;

                if ((y_nearest+tan(angle_of_line_between_two_nearest_points)*(pose_x_help-x_nearest)) <= pose_y_help){
                    sign_of_error = 1;
                }
                else sign_of_error = -1;
            }
            else if (fmod(abs(angle_of_line_between_two_nearest_points),(2*M_PI)) - M_PI/2 < 0.00001){
                cout << "3" << endl;

                if (x_nearest < pose_x_help){
                    sign_of_error = -1;
                     }
                else sign_of_error = 1;
            }
            else{
                cout << "4" << endl;

                if (x_nearest < pose_x_help){
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
    //End of for loop rake - platform

    //Same: Great number for start in copmparing if argument:
    double refangle = 10000000000000;

    //Smallest angle is wanted to be measured
    for(int v=0;v<3; v++){
        if(abs((fmod(pose_theta,2*M_PI) - fmod(angle_of_line_between_two_nearest_points_specialplatform,2*M_PI))+((v-1) * 2 * M_PI)) < abs(refangle)){
            refangle = (fmod(pose_theta,2*M_PI) - fmod(angle_of_line_between_two_nearest_points_specialplatform,2*M_PI))+((v-1) * 2 * M_PI);
        }
    }

    //Class variable angledifference is input for controller:
    angledifference = refangle;

    //Get the right angular velocity:
    double ang1, absolute1;


    //Absolute velocity is assigned to constant velocity gain, specified on top:
    //ADJUSTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!AGAIN!!!!!!!!!!!!!!!!
    absolute1 = absolute_velocity_;

    if(stopping_activator == 3){
        absolute1 = 0;

    }

    //For straight following of goal point in sharp turns, no disturbance!!
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


    //Velocity saturation, ATTENTION LOOKAHEAD EFFECTS!!!
//    if(absolute1before - absolute1 > 0.005){
//        absolute1 = absolute1before - 0.005;
//        double radius = absolute1 / ang1;
//        if(abs(radius) > 0.000001){
//            ang1 = absolute1 / radius;
//        }
//    }
//    else if(absolute1before - absolute1 < -0.005){
//        absolute1 = absolute1before + 0.005;
//        double radius = absolute1 / ang1;
//        if(abs(radius) > 0.000001){
//            ang1 = absolute1 / radius;
//        }
//    }


    //DELAY: (Continuous vector shifting in order to produce testing delay)
    for(int i = 0;i<delay;++i){
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
    if(indices_now_platform > path_with_information.size() - 1 || path_with_information_rake[indices_now_rake].linear.z == 129){
        ang[0] = 0;
        absolute[0]=0;
    }

    //ADJUSTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    if(stopping_activator == 2 && angledifference < 0.01){
//        stopping_activator = 3;
//    }



	//ROS_INFO("indnow: %d", indices_now_platform);
//    //For Tests: TESTING DEVIATION REACTION!
//    if(i_nearest < 640 && i_nearest > 600){
//        ang[0] += 0.4;
//    }

    //Creating message to be published:

//    Velocityoutput.linear.x = absolute[0];
//    Velocityoutput.angular.z = ang[0];
    Velocityoutput.twist.linear.x = absolute[0];
    Velocityoutput.twist.angular.z = ang[0];
    Velocityoutput.id = 3;
    //Only when calculating rake path:
    //ROS_INFO("this is k: %d", k);
    if(k >= 1){
      //ROS_INFO("near publisher: %d", rake_info_);
      unsigned int temp_unsigned;
      if(stopping_activator == 2 && absolute1 > 0){
          temp_unsigned = (unsigned int)path_with_information_rake[indices_now_rake - 20].linear.z;
      }
      else if(stopping_activator == 2 && absolute < 0){
          temp_unsigned = 0;
      }
      else if(stopping_activator == 5 || stopping_activator == 3){
          temp_unsigned = 0;
      }
      else if(stopping_activator == 4){
          temp_unsigned = (unsigned int)path_with_information_rake[indices_now_platform].linear.z;
      }
      else{
      temp_unsigned = (unsigned int)path_with_information_rake[indices_now_rake].linear.z;
      }
      ROS_INFO("before pub: %u", temp_unsigned);
      uint8_t temp_uint = temp_unsigned;
      ROS_INFO("before pub: %d", temp_uint);
      Velocityoutput.rake = temp_unsigned;


    //TODO: rake message
    //Velocityoutput.rake = binäre Zahl als int
    // get rake info at nearest point: i_nearest 
    //path_with_information[i] is the nearest point
    //readfile: read out rake message
    //absolute1before = absolute1;

	
    //publish:
    twistvelocityoutputpub.publish(Velocityoutput);
  }
  }

  //Read in the path:
  void ReadFile(const char *filename, int moderead) {
      std::ifstream infile(filename);
      int counterfile = 0;
      if (infile.is_open()) {
          std::string line;
          while(getline(infile, line)) {
              //ROS_INFO("moderead %d", moderead);
              std::istringstream iss(line);
              int temp_rake;
                  if (!(iss >> path_with_informationpoint.linear.x >> path_with_informationpoint.linear.y >> path_with_informationpoint.linear.z)) {
                      ROS_WARN("Du kannst nicht lesen?");
                  }
                  ROS_INFO("rake info after reading %f", path_with_informationpoint.linear.z);

                  if(moderead == 0){
                      //ROS_INFO("verkackt - nicht!!!!!");
                      counterfile +=1;
                      path_with_information.push_back(path_with_informationpoint);
                      //ROS_INFO("%d", counterfile);
                      //ROS_INFO("size: %d",(int)path_with_information.size());
                  }
                  else{
                      //ROS_INFO("verkackt!!!!!");
                      path_with_information_rake.push_back(path_with_informationpoint);
                  }
          }
      }
      else ROS_WARN("failed to open file");
      infile.close();
  }

  //Actual controller:
  double get_velocity_corrections(){

    angledifferenceIntegrated += angledifference;

    recterrIntegrated_platform += rectangular_error_platform;

    recterrIntegrated_rake += rectangular_error_rake;


    ros::Time now_time = ros::Time::now();

    double deltatime = (now_time - last_time).toSec();

    //Calculate derivatives:

    double dangle = (angledifference - last_angle_error) / deltatime;
    double drectplat = (rectangular_error_platform - last_rect_error_platform) / deltatime;
    double drectrake = (rectangular_error_rake - last_rect_error_rake) / deltatime;

    //ROS_INFO("LOOK HERE: recterrintegrated: %f", recterrIntegrated_platform);

    //Assign last step values for next step:
    last_time = now_time;
    last_angle_error = angledifference;
    last_rect_error_platform = rectangular_error_platform;
    last_rect_error_rake = rectangular_error_rake;

    absolute_vel_correction = gain_rect_err_platform * rectangular_error_platform + gain_rect_err_platform_int * recterrIntegrated_platform + gain_rect_err_platform_der * drectplat;

    angular_vel_correction = gain_angular_err * angledifference + gain_angular_err_int * angledifferenceIntegrated + dangle * gain_angular_err_der;

    double ang_vel_correction_from_rake = gain_rake_err * rectangular_error_rake;
    //ROS_INFO("angvelcorr %f, absvelcorr %f", angular_vel_correction, absolute_vel_correction);

    //For Comparison, summed up error:
    sumplat += abs(rectangular_error_platform);
    sumang += abs(angledifference);
    sumrake += abs(rectangular_error_rake);

    //ROS_INFO("sumplat %f, sumang %f, sumrake %f", sumplat, sumang, sumrake);

    return (angular_vel_correction + absolute_vel_correction + ang_vel_correction_from_rake);
  }
  


  double default_velocity_input(){

    int i_nearest_plus = indices_now_platform + magic_number;
    //ROS_INFO("indices_now_platform %d", indices_now_platform);
    //To avoid segmentation faults at the beginning:
    if(indices_now_platform == 0){
        indices_now_platform = indices_now_platform + 1;
    }



//    if(angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0)>0 && angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0)
//            || angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0)<0 && angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0)>0){


//    }
//    vector<double> angledifferencevector;
//    double anglediff_1 = angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0) - angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0);
//    angledifferencevector.push_back(anglediff_1);
//    double anglediff_2 = angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0) - angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0);
//    angledifferencevector.push_back(anglediff_2);
//    double anglesum_1 = angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0) + angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0);
//    angledifferencevector.push_back(anglesum_1);
//    double anglesum_2 = - angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0) - angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0);
//    angledifferencevector.push_back(anglesum_2);

//    double greatangle = 1000000000000000;
//    for(int j = 0;j<4;j++)
//        if(angledifferencevector[j]<greatangle){



//        }



    if(abs(angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0)-angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0))>M_PI){
        desired_heading_at_i_nearest = (angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0) - angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0))/2;
    }
    else{
        desired_heading_at_i_nearest = (angle_of_line_between_points(indices_now_platform+1, indices_now_platform, 0) + angle_of_line_between_points(indices_now_platform, indices_now_platform-1, 0))/2;
    }

    if(abs(angle_of_line_between_points(i_nearest_plus+1, i_nearest_plus, 0)-angle_of_line_between_points(i_nearest_plus,i_nearest_plus-1, 0))>M_PI){
        desired_heading_at_i_nearest_plus = (angle_of_line_between_points(i_nearest_plus+1, i_nearest_plus, 0) - angle_of_line_between_points(i_nearest_plus,i_nearest_plus-1, 0))/2;
    }
    else{
        desired_heading_at_i_nearest_plus = (angle_of_line_between_points(i_nearest_plus+1, i_nearest_plus, 0) + angle_of_line_between_points(i_nearest_plus,i_nearest_plus-1, 0))/2;
    }








    phi_to_plus = desired_heading_at_i_nearest_plus - desired_heading_at_i_nearest;


    double refphi = 1000000000000000000;

    for(int v=0;v<9; v++){

        if(abs(phi_to_plus+(v-4) * 2 * M_PI) < abs(refphi)){

            refphi = phi_to_plus+(v-4) * 2 * M_PI;
        }
    }

    phi_to_plus = refphi;

    double chord_length =  sqrt((path_with_information[indices_now_platform].linear.x - path_with_information[i_nearest_plus].linear.x) *
                           (path_with_information[indices_now_platform].linear.x - path_with_information[i_nearest_plus].linear.x) +
                           (path_with_information[indices_now_platform].linear.y - path_with_information[i_nearest_plus].linear.y) *
                           (path_with_information[indices_now_platform].linear.y - path_with_information[i_nearest_plus].linear.y));


//    if(chord_length ==0)
//    {
//        chord_length = 0.0001;
//    }

    circle_radius = chord_length / (2 * sin(phi_to_plus/2));


    //Assign angular velocity:
    angular_velocity_ = absolute_velocity_ / circle_radius + angular_vel_correction + absolute_vel_correction;

    //ROS_INFO("phi to plus %f", phi_to_plus);
    return angular_velocity_;

  }

  double angle_of_line_between_points(int i1, int i2, int mode){

      double angle;
      vector<geometry_msgs::Twist> path_with_information_calc;
      if(mode == 0){
          path_with_information_calc = path_with_information;
      }
      else{
          path_with_information_calc = path_with_information_rake;
      }


      if (i1 > i2){
           angle = atan2((path_with_information_calc[i1].linear.y-path_with_information_calc[i2].linear.y),(path_with_information_calc[i1].linear.x-path_with_information_calc[i2].linear.x));
      }
      else angle = atan2((path_with_information_calc[i2].linear.y-path_with_information_calc[i1].linear.y),(path_with_information_calc[i2].linear.x-path_with_information_calc[i1].linear.x));

      return angle;
  }
  
  
  controller(const std::string &file1, const std::string &file2)
  {
    ros::NodeHandle n;
    pose_sub = n.subscribe("/localization/bot_pose",1000, &controller::pose_sub_Callback, this);
    //pose_sub = n.subscribe("ref_pose",1000, &controller::pose_sub_Callback, this);
    //twistvelocityoutputpub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100, true);
    twistvelocityoutputpub = n.advertise<bb_state::TwistWithID>("/move_io", 100, true);
    ros::Publisher statepub;
    statepub = n.advertise<bb_state::State>("robot_state", 100, true);
    bb_state::State state_msg;
    state_msg.state = 3;
    statepub.publish(state_msg);
    //this->ReadFile("/home/beachbot/server/PathToStart.path", 0);
    //this->ReadFile("/home/beachbot/server/PathToStart.path", 1);
    this->ReadFile(file1.c_str(), 0);
    this->ReadFile(file2.c_str(), 1);
    start_toggle_ = true;
    angledifferenceIntegrated = 0;
    recterrIntegrated_platform = 0;
    recterrIntegrated_rake = 0;
    sumplat = 0;
    sumang = 0;
    sumrake = 0;
    stopping_activator = 1;
    counter_for_stopping_delay = 0;
  }
  
  ~controller()
  {


      //Publish one zero message when destructing:
      //Velocityoutput.linear.x = 0;

      //Velocityoutput.angular.z = 0;
      //Velocityoutput.rake = 0;
      //Velocityoutput.id = 3;

      //twistvelocityoutputpub.publish(Velocityoutput);
      //cout << "killedcout" << endl;

  }
  
};

int main(int argc, char **argv){

    ros::init(argc, argv, "Controller");
    if (argc == 3) {
        controller *controllerinstance = new controller(argv[1], argv[2]);
        double time = ros::Time::now().toNSec();
        //ROS_INFO("reached here %f",time);

        ros::Rate loop_rate(25);

        while (ros::ok())
        {


            ros::spinOnce();

            //controllerinstance->get_velocity_corrections();

            //controllerinstance->default_velocity_input();


            loop_rate.sleep();
        }
        delete controllerinstance;
    }
    else ROS_ERROR("No filenames provided");
    return 0;
}
