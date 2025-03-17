#include "unitree_go/msg/low_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <random>

#include <math.h>
#define _USE_MATH_DEFINES




class ImuOutput 
{
    public:
        ImuOutput()
        { 
            this->value << 0,0,0;
            this->bias << 0,0,0;
            this->noise << 0,0,0;
            this->noise_mean << 0,0,0;
            this->noise_standard_dev << 0,0,0;
            this->bias_mean << 0,0,0;
            this->bias_standard_dev << 0,0,0;
            
        }
    
        Eigen::Vector3d value;
        Eigen::Vector3d bias;
        Eigen::Vector3d noise;
        Eigen::Vector3d noise_mean;
        Eigen::Vector3d noise_standard_dev;
        Eigen::Vector3d bias_mean;
        Eigen::Vector3d bias_standard_dev;  
};

class FakeImuNode : public rclcpp::Node
{
    public:
        FakeImuNode()
        : Node("fake_imu")
        {
            this->declare_parameter("gravity", 1);
            this->declare_parameter("debug", 0);
            this->declare_parameter("pub_freq", 500);
            this->declare_parameter("run_time", 10);

            this->gravity_switch = this->get_parameter("gravity").as_int();
            this->debug_switch = this->get_parameter("debug").as_int();
            this->publishing_freq = this->get_parameter("pub_freq").as_int();
            this->run_time = this->get_parameter("run_time").as_int();


            RCLCPP_INFO_STREAM(this->get_logger(),"\nGo2 fake IMU lanched with parameters:\n"<<
            "Publishing Frequency: "<< publishing_freq << "Hz\n"<<
            "Run time: "<< run_time << "s\n" <<"Gravity:"<< gravity_switch);

            // ROS2
            lowstate_publisher_ = this->create_publisher<unitree_go::msg::LowState>("lowstate",10);
            pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/publishing_freq), std::bind(&FakeImuNode::pub_callback,this));
            timeout_timer_ = this->create_wall_timer(std::chrono::seconds(run_time), std::bind(&FakeImuNode::timeout_callback,this));
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Init 
            this->R_<<1,0,0,
                      0,1,0,
                      0,0,1;
            if(gravity_switch)
            {
                this->gravity_ << 0,0,9.81;
            }
            else
            {
                this->gravity_ << 0,0,0;
            }
            this->fake_a_ << 0,0,0;
            this->fake_g_ << 0,0,0;

            this->msg_tick = 0;

            /*
            a_.value << 0,0,0;
            a_.noise_mean << -0.139, 0.0291, -0.21;
            a_.noise_standard_dev << 0.0463, 0.05, 0.043;
            a_.bias_mean << 0,0,0; //-0.20, 0, 0;
            a_.bias_standard_dev <<0,0,0; // 0.0023, 0, 0;
            
            g_.noise_mean << -0.0012,-0.0007, -0.0004;
            g_.noise_standard_dev << 0.0089,0.0081,0.0083;
            */

            a_.value << 0,0,0; //1,1,1;
            a_.noise_mean << 0,0,0;
            a_.noise_standard_dev << 0,0,0;
            a_.bias_mean << 0,0,0;
            a_.bias_standard_dev <<0,0,0;
            
            g_.value <<  M_PI*0.1,0, 0; //0, M_PI*0.1,0;
            g_.noise_mean << 0,0,0;
            g_.noise_standard_dev << 0,0,0;
        
            RCLCPP_INFO_STREAM(this->get_logger(),"\nParameters:\n"<<
                                "Accelerometer:\n"<<
                                "\tvalue: "<<a_.value.transpose()<<"\n"<<
                                "\tbias: "<<a_.bias.transpose()<<"\n"<<
                                "\tnoise: "<<a_.noise.transpose()<<"\n"<<
                                "\tmean: "<<a_.noise_mean.transpose()<<"\n"<<
                                "\tstandard deviation: "<<a_.noise_standard_dev.transpose()<<"\n"<<
                                "\tbias mean: "<<a_.bias_mean.transpose()<<"\n"<<
                                "\tbias standard deviation: "<<a_.bias_standard_dev.transpose()<<"\n"<<

                                "Gyroscope:\n"<<
                                "\tvalue: "<<g_.value.transpose()<<"\n"<<
                                "\tbias: "<<g_.bias.transpose()<<"\n"<<
                                "\tnoise: "<<g_.noise.transpose()<<"\n"<<
                                "\tmean: "<<g_.noise_mean.transpose()<<"\n"<<
                                "\tstandard deviation: "<<g_.noise_standard_dev.transpose()<<"\n"<<
                                "\tbias mean: "<<g_.bias_mean.transpose()<<"\n"<<
                                "\tbias standard deviation: "<<g_.bias_standard_dev.transpose()<<"\n"
                                );  
        }

    private:
        void pub_callback()
        {
            calc_imu_data();
            unitree_go::msg::LowState lowstate_msg;

            for (int i = 0; i < 3; i++)
            {
                lowstate_msg.imu_state.accelerometer[i] = fake_a_[i];
                lowstate_msg.imu_state.gyroscope[i] = fake_g_[i];
            }

            this->msg_tick = this->msg_tick + (1.0/publishing_freq)*1000;

            lowstate_msg.tick = this->msg_tick;

            lowstate_msg.imu_state.quaternion[0] = q.x();
            lowstate_msg.imu_state.quaternion[1] = q.y();
            lowstate_msg.imu_state.quaternion[2] = q.z();
            lowstate_msg.imu_state.quaternion[3] = q.w();

            lowstate_publisher_->publish(lowstate_msg);   

            geometry_msgs::msg::TransformStamped t_;            
            t_.header.stamp = this->get_clock()->now();
            t_.header.frame_id = "map";
            t_.child_frame_id = "base";



            t_.transform.rotation.x = q.x();
            t_.transform.rotation.y = q.y();
            t_.transform.rotation.z = q.z();
            t_.transform.rotation.w = q.w();

            // Send the transformation
            tf_broadcaster_->sendTransform(t_);    
        }

        void timeout_callback()
        {
            RCLCPP_INFO_STREAM(this->get_logger(),"\n--- End of fake IMU run time ---\n"); 

            pub_timer_->cancel();
            timeout_timer_->cancel();
        }

        void calc_imu_data()
        {
            // fill noises values
            for(int i = 0; i < 3 ; i++)
            {
                a_.noise[i] = get_rand(a_.noise_mean[i],a_.noise_standard_dev[i]);
                a_.bias[i] = get_rand(a_.bias_mean[i],a_.bias_standard_dev[i]);

                g_.noise[i] = get_rand(g_.noise_mean[i],g_.noise_standard_dev[i]);
                g_.bias[i] = get_rand(g_.bias_mean[i],g_.bias_standard_dev[i]);
            }

            
            // fill out
            fake_a_ = a_.value + a_.bias - ( R_.transpose() * gravity_) + a_.noise ;
            fake_g_ = g_.value + g_.bias + g_.noise; 
            q = R_;

            Eigen::Vector3d phi = fake_g_*(1.0/publishing_freq); 
            R_ = R_ * this->Exp_SO3(phi) ;

            if(debug_switch)
            {
                RCLCPP_INFO_STREAM(this->get_logger(),"Value sent:\nAccel:\n" << fake_a_.transpose() << "\nGyro:\n" << fake_g_.transpose() << "\nRotation:\n" << R_ <<"\n norm acc: "<< fake_a_.norm());
            }
            
        }

        double get_rand(float mean, float std_dev)
        { // returns a random value using a normal distribution with parameters (mean, std_dev)

            std::random_device rd{}; //? creates obj every draw?
            std::mt19937 gen{rd()};

            std::normal_distribution<double> d{mean,std_dev};
            double sample = d(gen);
            return sample;
        }

        Eigen::Matrix3d Exp_SO3(const Eigen::Vector3d& w) {
            // Computes the vectorized exponential map for SO(3)
            Eigen::Matrix3d A = this->skew(w);
            double theta = w.norm();
            if (theta < TOLERANCE) {
                return Eigen::Matrix3d::Identity();
            } 
            Eigen::Matrix3d R =  Eigen::Matrix3d::Identity() + (sin(theta)/theta)*A + ((1-cos(theta))/(theta*theta))*A*A;
            return R;
        }

        const double TOLERANCE = 1e-10;

        Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
            // Convert vector to skew-symmetric matrix
            Eigen::Matrix3d M = Eigen::Matrix3d::Zero();
            M << 0, -v[2], v[1],
                v[2], 0, -v[0], 
                -v[1], v[0], 0;
                return M;
        }

        // Attributes ==========================================================
        int gravity_switch;
        int debug_switch;
        int publishing_freq;
        int run_time;

        rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowstate_publisher_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        rclcpp::TimerBase::SharedPtr timeout_timer_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

        Eigen::Matrix3d R_;
        Eigen::Vector3d gravity_;
        Eigen::Vector3d fake_a_;
        Eigen::Vector3d fake_g_;
        ImuOutput a_;
        ImuOutput g_;
        int msg_tick;
        Eigen::Quaterniond q;


};



//std::normal_distribution<float> d{mean[i], stddev[i]}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImuNode>());
  rclcpp::shutdown();
  return 0;
}