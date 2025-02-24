#include "unitree_go/msg/low_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <Eigen/Dense>
#include <random>

#define PUBLISHING_FREQ 500 //Hz
#define RUN_TIME 1 //sec
#define DEBUG 0

class ImuOutput 
{
    public:
        ImuOutput()
        { 
            this->value << 0,0,0;
            this->bias << 0,0,0;
            this->noise << 0,0,0;
            this->mean << 0,0,0;
            this->standard_dev << 0,0,0;
            this->bias_mean << 0,0,0;
            this->bias_standard_dev << 0,0,0;
            
        }
    
        Eigen::Vector3d value;
        Eigen::Vector3d bias;
        Eigen::Vector3d noise;
        Eigen::Vector3d mean;
        Eigen::Vector3d standard_dev;
        Eigen::Vector3d bias_mean;
        Eigen::Vector3d bias_standard_dev;  
};

class FakeImuNode : public rclcpp::Node
{
    public:
        FakeImuNode()
        : Node("fake_imu")
        {
            RCLCPP_INFO_STREAM(this->get_logger(),"\nGo2 fake IMU lanched with parameters:\n"<<
            "Publishing Frequency: "<< PUBLISHING_FREQ << "Hz\n"<<
            "Run time: "<< RUN_TIME << "s");

            // ROS2
            lowstate_publisher_ = this->create_publisher<unitree_go::msg::LowState>("lowstate",10);
            pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000/PUBLISHING_FREQ), std::bind(&FakeImuNode::pub_callback,this));
            timeout_timer_ = this->create_wall_timer(std::chrono::seconds(RUN_TIME), std::bind(&FakeImuNode::timeout_callback,this));

            // Init 
            this->R_<<1,0,0,
                      0,1,0,
                      0,0,1;
            this->gravity_ << 0,0,-9.81;
            this->fake_a_ << 0,0,0;
            this->fake_g_ << 0,0,0;

            a_.value << 1,1,1;
            a_.standard_dev << 0.001,0.001,0.001;

        
            RCLCPP_INFO_STREAM(this->get_logger(),"\nParameters:\n"<<
                                "Accelerometer:\n"<<
                                "\tvalue: "<<a_.value.transpose()<<"\n"<<
                                "\tbias: "<<a_.bias.transpose()<<"\n"<<
                                "\tnoise: "<<a_.noise.transpose()<<"\n"<<
                                "\tmean: "<<a_.mean.transpose()<<"\n"<<
                                "\tstandard deviation: "<<a_.standard_dev.transpose()<<"\n"<<
                                "\tbias mean: "<<a_.bias_mean.transpose()<<"\n"<<
                                "\tbias standard deviation: "<<a_.bias_standard_dev.transpose()<<"\n"<<

                                "Gyroscope:\n"<<
                                "\tvalue: "<<g_.value.transpose()<<"\n"<<
                                "\tbias: "<<g_.bias.transpose()<<"\n"<<
                                "\tnoise: "<<g_.noise.transpose()<<"\n"<<
                                "\tmean: "<<g_.mean.transpose()<<"\n"<<
                                "\tstandard deviation: "<<g_.standard_dev.transpose()<<"\n"<<
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

            lowstate_publisher_->publish(lowstate_msg);       
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
                a_.noise[i] = get_rand(a_.mean[i],a_.standard_dev[i]);
                a_.bias[i] = get_rand(a_.bias_mean[i],a_.bias_standard_dev[i]);

                g_.noise[i] = get_rand(g_.mean[i],g_.standard_dev[i]);
                g_.bias[i] = get_rand(g_.bias_mean[i],g_.bias_standard_dev[i]);
            }

            fake_a_ = a_.value + a_.bias - ( R_ * gravity_) + a_.noise ;
            fake_g_ = g_.value + g_.bias + g_.noise; 

            if(DEBUG)
            {
                RCLCPP_INFO_STREAM(this->get_logger(),"Value sent:\nAccel:\n" << fake_a_.transpose() << "\nGyro:\n" << fake_g_.transpose());
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

        // Attributes ==========================================================
        rclcpp::Publisher<unitree_go::msg::LowState>::SharedPtr lowstate_publisher_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        rclcpp::TimerBase::SharedPtr timeout_timer_;

        Eigen::Matrix3d R_;
        Eigen::Vector3d gravity_;
        Eigen::Vector3d fake_a_;
        Eigen::Vector3d fake_g_;
        ImuOutput a_;
        ImuOutput g_;



};



//std::normal_distribution<float> d{mean[i], stddev[i]}; 

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeImuNode>());
  rclcpp::shutdown();
  return 0;
}