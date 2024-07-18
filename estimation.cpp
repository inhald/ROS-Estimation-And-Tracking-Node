#include <ros/ros.h> 
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <eigen3/Eigen/Dense>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include "nav_msgs/Odometry.h" //Odometer


class Estimator {

    private:
        ros::NodeHandle nf; 
        
        //Subscriptions
        ros::Subscriber april_tag;  
        ros::Subscriber odom;
        ros::Subscriber tf;


        //camera
        int cam;

        //topics
        std::string depth_info_topic, depth_image_topic;
        std::string tag_detections_topic = "/tag_detections";
        std::string odom_topic = "/odom";
        std::string tf_topic = "/tf";


        //tag_pose measurement
        bool init=false;
        double tag_x, tag_y, tag_z;
        double tag_euler_x, tag_euler_y, tag_euler_z; 
        int time_a_secs, time_a_nanosecs;
        int time_b_secs, time_b_nanosecs;

        ros::Time time_a, time_b;


        //tag_pose prediction
        double x_est, y_est, v_est;


        double dt;


        //Kalman Matrices
        Eigen::MatrixXd I;
        Eigen::VectorXd x_hat;
        Eigen::MatrixXd A;
        Eigen::MatrixXd P;
        Eigen::MatrixXd H;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd R;

        Eigen::MatrixXd G;


        //Odometry Measurements
        double vel; double yaw;
        double dvel; double dyaw;
        Eigen::VectorXd U;

        //Kalman Update
        Eigen::VectorXd x_hat_new;
        Eigen::MatrixXd K;
        Eigen::VectorXd y;


        //Covariance matrix
        double sigma_ax  = .1; 
        double sigma_ay = .1;
        double sigma_alpha = .1;

        //Prediction
        int n;

        //Transformation
        geometry_msgs::PointStamped transformed_pt;


        //lidar frame position
        double lidar_x, lidar_y, lidar_z;




        

    public:

        Estimator(){

            nf.getParam("use_camera", cam);
            nf.getParam("odom_topic", odom_topic);

            vel = 0; yaw = 0; dyaw = 0; dvel = 0;

            if(cam){
                april_tag = nf.subscribe(tag_detections_topic,1, &Estimator::tag_callback, this);
                odom = nf.subscribe(odom_topic, 1, &Estimator::odom_callback, this);
                // tf = nf.subscribe(tf_topic, 1, &Estimator::transform_callback, this);
            }

            x_est = 0.0; y_est = 0.0; v_est = 0.0;
            dt = 1.0/30.; n = 1;

            U = Eigen::VectorXd::Zero(4);
            // std::cout << "U: \n" << std::endl;
            // std::cout << U << std::endl;        

            I = Eigen::MatrixXd::Identity(6,6);


            I << 1. , 0. , 0. , 0. , 0. , 0. ,
                 0. , 1. , 0. , 0. , 0. , 0. ,
                 0. , 0. , 1. , 0. , 0. , 0. ,
                 0. , 0. , 0. , 1. , 0. , 0. ,
                 0. , 0. , 0. , 0. , 1. , 0. ,
                 0. , 0. , 0. , 0. , 0. , 1. ;

            // std::cout << "I: \n" << std::endl;
            // std::cout << I << std::endl;


            x_hat = Eigen::VectorXd::Zero(6);
            x_hat << 0.  , 
                     0.  ,
                     0.  ,
                     0.  ,
                     0.  ,
                     0.  ; 


            // Init 

            // std::cout << "A: \n" << std::endl;
            // std::cout << A << std::endl;


            P= Eigen::MatrixXd::Zero(6,6);
            P << 10. , 0. , 0.  , 0.  , 0.  , 0.  , 
                 0. , 50. , 0.  , 0.  , 0.  , 0.  ,
                 0. , 0.  , 10. , 0.  , 0.  , 0.  ,
                 0. , 0.  , 0.  , 10. , 0.  , 0.  ,
                 0. , 0.  , 0.  , 0.  , 10. , 0.  ,
                 0. , 0.  , 0.  , 0.  , 0.  , 50. ; 
            


            // std::cout << "P: \n" << std::endl;
            // std::cout << P << std::endl;

            //Observability Matrix
            H = Eigen::MatrixXd::Zero(3,6);
            H << 1. , 0. , 0. , 0. , 0. , 0. ,
                 0. , 0. , 1. , 0. , 0. , 0. ,
                 0. , 0. , 0. , 0. , 1. , 0. ;

            // std::cout << "H: \n" << std::endl;
            // std::cout << H << std::endl;

            
            // std::cout << "Q: \n" << std::endl;
            // std::cout << Q << std::endl;
        

            // // Measurement noise 
            R = Eigen::MatrixXd::Zero(3,3);
            R << .05 ,  0.  , 0.  ,
                  0.  , .05 , 0.  ,
                  0.  , 0.   , .2  ;


            // std::cout << "R: \n" << std::endl;
            // std::cout << R << std::endl;



        }




        void odom_callback(const nav_msgs::OdometryConstPtr& odom_msg){
            double old_vel, old_yaw;
            old_vel = vel;
            old_yaw = yaw;

            vel = odom_msg->twist.twist.linear.x;
            yaw = 2*atan2(odom_msg->pose.pose.orientation.z, odom_msg->pose.pose.orientation.w);

            dvel = vel - old_vel;
            dyaw = yaw - old_yaw;



            U << vel, dvel, yaw, dyaw;
            // std::cout << "U: " << std::endl;
            // std::cout << U << std::endl;
            // std::cout << "\n" << std::endl;

        }

        void kalman_update( Eigen::MatrixXd& A, Eigen::MatrixXd& H, Eigen::MatrixXd& P, Eigen::MatrixXd& Q, Eigen::MatrixXd& R, Eigen::VectorXd& x_hat){

            //Predict 

            // Eigen::VectorXd x_hat_new(6);

            // Eigen::MatrixXd K(6,6);

            // Eigen::VectorXd y(3);

            x_hat_new = Eigen::VectorXd::Zero(6);

            K = Eigen::MatrixXd::Zero(6,3);

            y = Eigen::VectorXd::Zero(3);

            y << tag_x, tag_y, tag_euler_y;

            x_hat_new = A*x_hat + G*U;

            P = A*P*A.transpose() + Q;

            K = P*H.transpose()*(H*P*H.transpose() + R).inverse();

            // std::cout << K << std::endl;
 
            x_hat_new  += K * (y - H*x_hat_new);


            P = (I-K*H)*P*(I-K*H).transpose(); 


            x_hat = x_hat_new; 




        }



        void transform_callback(const tf2_msgs::TFMessageConstPtr& tf_msg){

            if(tf_msg->transforms.empty()) 
            { return;}

            // std::cout << tx << " " << ty << " " << tz << " " << std::endl;

            // tf2_ros::Buffer tfBuffer;
            // tf2_ros::TransformListener tfListener(tfBuffer);

            // geometry_msgs::TransformStamped transformStamped;
            // transformStamped = tfBuffer.lookupTransform("camera_color_optical_frame", "BOB", ros::Time::now());

            // geometry_msgs::Vector3 position = transformStamped.transform.translation;
            // std::cout << position.x << " " << position.y << " " << position.z << std::endl;


        }

        void tag_callback(const apriltag_ros::AprilTagDetectionArrayConstPtr& data){

            //hold previous detection stamp and return if no detection
            //calculate steps then apply modified kalman filter

            if(data->detections.empty()) return;

            else{

                time_a = ros::Time::now();
            
                for(const auto& detection : data->detections){
                    tag_x = detection.pose.pose.pose.position.z;
                    tag_y = -detection.pose.pose.pose.position.x;
                    tag_euler_y = detection.pose.pose.pose.orientation.y;
                    // time_a_secs = detection.pose.header.stamp.sec;
                    // time_a_nanosecs = detection.pose.header.stamp.nanosecs;
                }


                //Transform to Frame of Lidar

                if(init){

                    dt = (time_a - time_b).toSec();


                    // std::cout << dt << std::endl;

                    // if(dt > 1){ std::cout << "LARGER THAN 1!!" << std::endl;}

                }
                else{

                    time_b = time_a;

                    init = true; dt = 1./30.;
                }

                //Init Kalman Matrices

                //Estimate Error covariance

                G = Eigen::MatrixXd::Identity(6,4); 
                G << -dt , 0. , 0. , 0. ,
                  0. , -1. , 0. , 0. ,
                  0. , 0.  , 0. , 0. ,
                  0. , 0. , 0.  , 0. ,
                  -dt , 0. , 0. , 0. ,
                  0. , -1.  , 0. , 0. ;



                A = Eigen::MatrixXd::Zero(6,6);
                A << 1. , dt , 0. , 0. , 0. , 0. ,
                 0. , 1. , 0. , 0. , 0. , 0. ,
                 0. , 0. , 1. , dt , 0. , 0. ,
                 0. , 0. , 0. , 1. , 0. , 0. , 
                 0. , 0. , 0. , 0. , 1. , dt ,
                 0. , 0. , 0. , 0. , 0. , 1. ;


                Q=Eigen::MatrixXd::Zero(6,6);
                Q << .5*pow(dt,4)*pow(sigma_ax,2), .5*pow(dt,3)*pow(sigma_ax,2), 0. , 0.   , 0. , 0. , 
                .5*pow(dt,3)*pow(sigma_ax,2), pow(dt,2)*pow(sigma_ax,2), 0. , 0.   , 0. , 0. ,
                0.  , 0.   , .5*pow(dt,4)*pow(sigma_ay,2),.5*pow(dt,3)*pow(sigma_ay,2), 0. , 0. ,
                0.  , 0.   , .5*pow(dt,3)*pow(sigma_ay,2), pow(dt,2)*pow(sigma_ay,2), 0. , 0. ,
                0.  , 0.   , 0. , 0.   , .5*pow(dt,4)*pow(sigma_alpha,2), .5*pow(dt,3)*pow(sigma_alpha,2),
                0.  , 0.   , 0. , 0.   , .5*pow(dt,4)*pow(sigma_alpha,2), pow(dt,2)*pow(sigma_alpha,2);
                



                kalman_update(A, H, P, Q, R, x_hat);




                //printing

                std::cout << "x_hat: \n" <<  std::endl;
                std::cout << x_hat << std::endl;
                // std::cout << data->header.stamp.sec << std::endl;

                lidar_x = x_hat[0]+0.095;
                lidar_y = x_hat[2]-0.04; 

                // std::cout << lidar_y << std::endl;

                // std::cout << "\n";
                // std::cout << "x_measure: " << tag_x << std::endl;
                // std::cout << "y_measure: " << tag_y << std::endl;
                // std::cout << "euler_angle_z: " << tag_euler_z << std::endl;
                // std::cout << "\n";

                time_b = time_a;


            }


        }










};



int main(int argc, char **argv){

    ros::init(argc, argv, "estimation");

    Estimator est; 

    while(ros::ok()){ ros::spinOnce(); }



}