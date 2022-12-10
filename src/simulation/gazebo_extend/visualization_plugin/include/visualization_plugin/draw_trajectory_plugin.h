#ifndef DRAW_TRAJECTORY_PLUGIN_HH
#define DRAW_TRAJECTORY_PLUGIN_HH

#include <ros/ros.h>
#include <ignition/msgs.hh>
#include <ignition/transport.hh>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/rendering/Visual.hh>

namespace gazebo
{
  class GZ_PLUGIN_VISIBLE DrawTrajectoryPlugin : public VisualPlugin
  {
    /// \brief Constructor
    public: DrawTrajectoryPlugin();

    /// \brief Destructor
    public: ~DrawTrajectoryPlugin();

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

    /// \brief Update the plugin once every iteration of simulation.
    private: void OnUpdate();

    private:
    ros::NodeHandle* nh;
    bool enable_draw = true;
    rendering::VisualPtr visual;
    ignition::math::Vector3d position;
    event::ConnectionPtr updateConnection;
    ignition::transport::Node node;
    ignition::msgs::Marker markerMsg;
    ignition::math::Vector3d prevPoint;
    int step = 1;
    int length = 120;
  };
}
#endif