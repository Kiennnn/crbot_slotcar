#include "functional"
#include "gazebo/gazebo.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "ignition/math/Vector3.hh"
#include "ros/ros.h"
#include "thread"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Pose2D.h"
#include "math.h"

namespace gazebo
{
class CrbotSlotcarPlugin : public ModelPlugin
{
  private:
    physics::ModelPtr model;
    event::ConnectionPtr updateConnection;
    std::unique_ptr<ros::NodeHandle> rosNode;
    ros::Subscriber rosSub;
    ros::CallbackQueue rosQueue;
    std::thread rosQueueThread;
    geometry_msgs::Pose2D current_pose;
    geometry_msgs::Pose2D start_pose;

  public:
    CrbotSlotcarPlugin()
    {
    }

  public:
    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        this->model = _model;
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CrbotSlotcarPlugin::OnUpdate, this));
        ROS_WARN("Plugin attached!");
        if (!ros::isInitialized()) {
            int argc = 0;
            char** argv = NULL;
            ros::init(argc, argv, "crbot_slotcar", ros::init_options::NoSigintHandler);
        }

        this->rosNode.reset(new ros::NodeHandle("crbot_slotcar"));

        ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Pose2D>(
            "/goal_pose", 1, boost::bind(&CrbotSlotcarPlugin::OnRosMsg, this, _1), ros::VoidPtr(), &this->rosQueue);
        this->rosSub = this->rosNode->subscribe(so);

        this->rosQueueThread = std::thread(std::bind(&CrbotSlotcarPlugin::QueueThread, this));

        this->start_pose.x = this->model->WorldPose().X();
        this->start_pose.y = this->model->WorldPose().Y();
        this->start_pose.theta = this->model->WorldPose().Yaw();
    }

  private:
    void QueueThread()
    {
        static const double timeout = 0.01;
        while (this->rosNode->ok()) {
            this->rosQueue.callAvailable(ros::WallDuration(timeout));
        }
    }

  public:
    void OnUpdate()
    {
        this->current_pose.x = this->model->WorldPose().X();
        this->current_pose.y = this->model->WorldPose().Y();
        this->current_pose.theta = this->model->WorldPose().Yaw();
        std::cout << "Current pose: " << this->current_pose.x << " " << this->current_pose.y << " "
                  << this->current_pose.theta << "\n";
    }

  public:
    void UpdateStartPose()
    {
        this->start_pose.x = this->model->WorldPose().X();
        this->start_pose.y = this->model->WorldPose().Y();
        this->start_pose.theta = this->model->WorldPose().Yaw();
    }

  public:
    void OnRosMsg(const geometry_msgs::Pose2DConstPtr& goal_pose_msg)
    {
        this->CalVelocity(goal_pose_msg->x, goal_pose_msg->y);
    }

  public:
    void CalVelocity(const double& goal_x, const double& goal_y)
    {
        double angle_to_turn;
        double x_vel, y_vel, yaw_vel;
        angle_to_turn = atan2(goal_y - this->start_pose.y, goal_x - this->start_pose.x);

        if (goal_y - 0.1 <= this->start_pose.y && this->start_pose.y <= goal_y + 0.1 &&
            goal_x - 0.1 <= this->current_pose.x && this->current_pose.x <= goal_x + 0.1) {
            yaw_vel = 0;
            x_vel = 0;
            y_vel = 0;
            this->UpdateStartPose();
        } else {
            if (abs(angle_to_turn - this->current_pose.theta) >= 0.01) {
                yaw_vel = 1;
                x_vel = 0;
                y_vel = 0;
            } else {
                yaw_vel = 0;
                x_vel = 1 * cos(angle_to_turn);
                y_vel = 1 * sin(angle_to_turn);
            }
        }
        this->SetVelocity(x_vel, y_vel, yaw_vel);
    }

  public:
    void SetVelocity(const double& x_vel, const double& y_vel, const double& yaw_vel)
    {
        this->model->SetLinearVel(ignition::math::Vector3d(x_vel, y_vel, 0));
        this->model->SetAngularVel(ignition::math::Vector3d(0, 0, yaw_vel));
    }
};

GZ_REGISTER_MODEL_PLUGIN(CrbotSlotcarPlugin)
}  // namespace gazebo