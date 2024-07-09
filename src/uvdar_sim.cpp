#include <ros/ros.h>
#include <ros/package.h>

#include <nav_msgs/Odometry.h>
#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/param_loader.h>

#include <random>

namespace e = Eigen;

namespace Eigen
{
  typedef Matrix< double, 6, 6 > 	Matrix6d;
}

namespace uvdar {

  struct NodeReferrer {
    std::string name;
    int index = -1;
    int ID = -1;
  };
  struct NodePose {
    std::string name;
    nav_msgs::Odometry pose;
    bool initialized=false;
  };

  struct NormalRandomVariable
  {
    NormalRandomVariable(Eigen::MatrixXd const& covar)
      : NormalRandomVariable(Eigen::VectorXd::Zero(covar.rows()), covar)
    {}

    NormalRandomVariable(Eigen::VectorXd const& mean, Eigen::MatrixXd const& covar)
      : mean(mean)
    {
      Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(covar);
      transform = eigenSolver.eigenvectors() * eigenSolver.eigenvalues().cwiseSqrt().asDiagonal();
    }

    Eigen::VectorXd mean;
    Eigen::MatrixXd transform;

    Eigen::VectorXd operator()() const
    {
      static std::mt19937 gen{ std::random_device{}() };
      static std::normal_distribution<> dist;

      return mean + transform * Eigen::VectorXd{ mean.size() }.unaryExpr([&]([[maybe_unused]] auto x) { return dist(gen); });
    }
  };

  class UVDARMultirobotSimulator {
    private: 
      //TODO: mutex these
      std::vector<NodeReferrer> _targets_;
      std::vector<NodeReferrer> _observers_;

      std::vector<NodePose> _node_poses_;
      std::string _ground_truth_topic_;
      std::string _uvdar_topic_;

      std::string _output_frame_;
      std::mutex transformer_mutex;
      mrs_lib::Transformer transformer_;

      std::vector<ros::Subscriber> sub_positions_;
      std::vector<NodePose> node_poses_;

      std::vector<ros::Publisher> pub_uvdar_estimate_;

      double _uvdar_rate_;
      std::vector<ros::Timer> uvdar_rate_timer_;
    public:
      using node_callback_t = boost::function<void (const nav_msgs::OdometryConstPtr& msg)>;

      /**
       * @brief Constructor - loads parameters and initializes necessary structures
       *
       * @param nh Private NodeHandle of this ROS node
       */
      /* Constructor //{ */
      UVDARMultirobotSimulator(ros::NodeHandle& nh) {
        mrs_lib::ParamLoader param_loader(nh, "UVDARMultirobotSimulator");

        std::vector<std::string> _target_name_list, _observer_name_list;
        param_loader.loadParam("target_name_list", _target_name_list, _target_name_list);
        if (_target_name_list.empty()) {
          ROS_ERROR("[UVDARMultirobotSimulator]: No target names were supplied!");
          return;
        }
        param_loader.loadParam("observer_name_list", _observer_name_list, _observer_name_list);
        if (_observer_name_list.empty()) {
          ROS_ERROR("[UVDARMultirobotSimulator]: No observer names were supplied!");
          return;
        }

        //create structures for holding poses of nodes and for referring to them from a list of observers and targets
        for (auto &n : _target_name_list){
          int ID = getID(n); 

          auto is_named = [n](const NodePose &np) -> bool { return np.name == n; };
          if (auto it = std::find_if(begin(_node_poses_), end(_node_poses_), is_named); it != std::end(_node_poses_)){
            _targets_.push_back({.name=n, .index = (int)(it-_node_poses_.begin()),.ID=ID});
          }
          else{
            _node_poses_.push_back({.name=n,.pose=nav_msgs::Odometry(),.initialized=false});
            _targets_.push_back({.name=n, .index = (int)(_node_poses_.size()-1),.ID=ID});
          }

        }
        for (auto &n : _observer_name_list){
          auto is_named = [n](const NodePose &np) -> bool { return np.name == n; };
          if (auto it = std::find_if(begin(_node_poses_), end(_node_poses_), is_named); it != std::end(_node_poses_)){
            _observers_.push_back({.name=n, .index = (int)(it-_node_poses_.begin())});
          }
          else{
            _node_poses_.push_back({.name=n,.pose=nav_msgs::Odometry(),.initialized=false});
            _observers_.push_back({.name=n, .index = (int)(_node_poses_.size()-1)});
          }

        }

        param_loader.loadParam("ground_truth_topic", _ground_truth_topic_, std::string());
        param_loader.loadParam("uvdar_topic", _uvdar_topic_, std::string());


        for (unsigned int i = 0; i < (unsigned int)(_node_poses_.size()); i++){

          auto size = std::snprintf(nullptr, 0, _ground_truth_topic_.c_str(), _node_poses_[i].name.c_str() );
          char topic_raw[size+1];
          std::sprintf(topic_raw, _ground_truth_topic_.c_str(), _node_poses_[i].name.c_str() );
          std::string topic = topic_raw;

          node_callback_t callback = [node_index=i,this] (const nav_msgs::OdometryConstPtr& msg) { 
            ProcessPosition(msg, node_index);
          };
          ROS_INFO_STREAM("[UVDARMultirobotSimulator]: Subscribing to " << topic << ".");
          sub_positions_.push_back(nh.subscribe(topic, 1, callback));

        }

        param_loader.loadParam("output_frame", _output_frame_, std::string("local_origin"));

        param_loader.loadParam("uvdar_rate", _uvdar_rate_, double(-1.0));

        for (unsigned int i = 0; i < (unsigned int)(_observers_.size()); i++){
          auto size = std::snprintf(nullptr, 0, _uvdar_topic_.c_str(), _observers_[i].name.c_str() );
          char topic_raw[size+1];
          std::sprintf(topic_raw, _uvdar_topic_.c_str(), _observers_[i].name.c_str() );
          std::string topic = topic_raw;

          ROS_INFO_STREAM("[UVDARMultirobotSimulator]: Advertising measured poses from observer " << _observers_[i].name << ".");
          pub_uvdar_estimate_.push_back(nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>(topic, 1));
          uvdar_rate_timer_.push_back(nh.createTimer(ros::Rate(_uvdar_rate_).expectedCycleTime(), boost::bind(&UVDARMultirobotSimulator::ProduceEstimate, this, _1, i), false, true));
        }



        transformer_ = mrs_lib::Transformer("UVDARPoseCalculator");
      }
      //}


      /**
       * @brief Callback to process current position of UAVs (nodes)
       *
       * @param msg The input message - odometry of a given node
       * @param node_index The index of the node from a list
       */
      /* ProcessPosition //{ */
      void ProcessPosition(const nav_msgs::OdometryConstPtr& msg, size_t node_index) {
        _node_poses_[node_index].pose = *msg;
        _node_poses_[node_index].initialized = true;
        return;
      }
      //}

      /**
       * @brief Timer callback for producing UVDAR estimates from a given observer
       *
       * @param msg The input message - odometry of a given node
       * @param node_index The index of the node from a list
       */
      /* ProduceEstimate //{ */
      void ProduceEstimate([[maybe_unused]] const ros::TimerEvent& te, int observer_index) {
        if (!(_node_poses_[_observers_[observer_index].index].initialized)){
          ROS_INFO_STREAM("[UVDARMultirobotSimulator]: Pose of observer " << _observers_[observer_index].name << " not yet initialized...");
          return;
        }

        mrs_msgs::PoseWithCovarianceArrayStamped msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = _observers_[observer_index].name + "/" + _output_frame_;

        for (auto &tg : _targets_){
          if (!(_node_poses_[tg.index].initialized)){
            ROS_INFO_STREAM("[UVDARMultirobotSimulator]: Pose of target " << _targets_[observer_index].name << " not yet initialized...");
            continue;
          }

          auto curr_pose = _node_poses_[tg.index];

          geometry_msgs::TransformStamped world2fcu, fcu2output;
          {
            std::scoped_lock lock(transformer_mutex);
            auto world2fcu_tmp = transformer_.getTransform(curr_pose.pose.header.frame_id,_observers_[observer_index].name+"/fcu", msg.header.stamp-ros::Duration(0.2));
            if (!world2fcu_tmp){
              ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARMultirobotSimulator]: Could not obtain transform from " << curr_pose.pose.header.frame_id << " to " <<  _observers_[observer_index].name+"/fcu" << "!");
              return;
            }
            world2fcu = world2fcu_tmp.value();

            auto fcu2output_tmp = transformer_.getTransform(_observers_[observer_index].name+"/fcu", _observers_[observer_index].name+"/"+_output_frame_, msg.header.stamp-ros::Duration(0.2));
            if (!fcu2output_tmp){
              ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARMultirobotSimulator]: Could not obtain transform from " << _observers_[observer_index].name+"/fcu"<< " to " <<  _observers_[observer_index].name+"/"+_output_frame_ << "!");
              return;
            }
            fcu2output = fcu2output_tmp.value();
          }


          geometry_msgs::PoseWithCovarianceStamped target_pose_world;
          target_pose_world.pose = curr_pose.pose.pose;
          auto target_pose_fcu_tmp = transformer_.transform(target_pose_world, world2fcu);
          if (!target_pose_fcu_tmp){
            ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARMultirobotSimulator]: Could not perform transform from " << curr_pose.pose.header.frame_id << " to " <<  _observers_[observer_index].name+"/fcu" << "!");
            return;
          }
          auto target_pose_fcu = target_pose_fcu_tmp.value();

          auto target_estimate_fcu = generateMeasurement(target_pose_fcu);

          auto target_estimate_output_tmp = transformer_.transform(target_estimate_fcu, fcu2output);
          if (!target_estimate_output_tmp){
            ROS_ERROR_STREAM_THROTTLE(1.0,"[UVDARMultirobotSimulator]: Could not perform transform from " << _observers_[observer_index].name+"/fcu"<< " to " <<  _observers_[observer_index].name+"/"+_output_frame_ << "!");
            return;
          }
          auto target_estimate_output = target_estimate_output_tmp.value();

          mrs_msgs::PoseWithCovarianceIdentified pci;
          pci.id = tg.ID;
          pci.pose = target_estimate_output.pose.pose;
          pci.covariance = target_estimate_output.pose.covariance;

          msg.poses.push_back(pci);
        }


        pub_uvdar_estimate_[observer_index].publish(msg);

        return;
      }
      //}

      //TODO make this ID more flexible
      int getID(std::string name){
        std::string substring = name.substr(3);//currently fixed with naming of "uav#"
        int ID = atoi(substring.c_str());
        return ID;
      }

      geometry_msgs::PoseWithCovarianceStamped generateMeasurement(geometry_msgs::PoseWithCovarianceStamped input){
        e::Vector3d p(
            input.pose.pose.position.x,
            input.pose.pose.position.y,
            input.pose.pose.position.z
            );
        e::Quaterniond q(
            input.pose.pose.orientation.w,
            input.pose.pose.orientation.x,
            input.pose.pose.orientation.y,
            input.pose.pose.orientation.z
            );





        e::Matrix6d C = e::Matrix6d::Zero();
        C.topLeftCorner(3,3) = 0.5*e::Matrix3d::Identity();
        C.bottomRightCorner(3,3) = 0.5*e::Matrix3d::Identity();

        auto noise_gen = NormalRandomVariable(C);

        auto noise = noise_gen();

        e::Vector3d pn = p + noise.topLeftCorner(3,1);
        e::Vector3d wn = noise.bottomLeftCorner(3,1);
        e::Quaterniond qtemp =
          e::AngleAxisd(wn.x(), e::Vector3d::UnitX()) *
          e::AngleAxisd(wn.y(), e::Vector3d::UnitY()) *
          e::AngleAxisd(wn.z(), e::Vector3d::UnitZ());
        e::Quaterniond qn = qtemp*q;
        




        geometry_msgs::PoseWithCovarianceStamped output;
        output.pose.pose.position.x = pn.x();
        output.pose.pose.position.y = pn.y();
        output.pose.pose.position.z = pn.z();
        output.pose.pose.orientation.w = qn.w();
        output.pose.pose.orientation.x = qn.x();
        output.pose.pose.orientation.y = qn.y();
        output.pose.pose.orientation.z = qn.z();

        for (int i=0; i<6; i++){
          for (int j=0; j<6; j++){
            output.pose.covariance[6*j+i] = C(j,i);
          }
        }

        return output;
      }
  };


} //uvdar

int main(int argc, char** argv) {
  ros::init(argc, argv, "uvdar_multirobot_simulator");

  ros::NodeHandle nh("~");
  uvdar::UVDARMultirobotSimulator ums(nh);
  ROS_INFO("[UVDARMultirobotSimulator]: UVDAR Multirobot Simulator node initiated");
  ros::spin();
  return 0;
}
