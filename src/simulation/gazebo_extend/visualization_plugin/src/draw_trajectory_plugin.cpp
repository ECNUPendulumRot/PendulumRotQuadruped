#include "visualization_plugin/draw_trajectory_plugin.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_VISUAL_PLUGIN(DrawTrajectoryPlugin)

DrawTrajectoryPlugin::DrawTrajectoryPlugin() : VisualPlugin(){}


DrawTrajectoryPlugin::~DrawTrajectoryPlugin(){}


void DrawTrajectoryPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  visual = _visual;
  if(_sdf->HasElement("position")){
    position = _sdf->Get<ignition::math::Vector3d>("position");
  }
  else{
    position = ignition::math::Vector3d(0, 0, 0);
  }

  std::string material;
  if(_sdf->HasElement("material")){
    material = _sdf->Get<std::string>("material");
  }

  // this->updateConnection = event::Events::ConnectPreRender(std::bind(&DrawTrajectoryPlugin::OnUpdate, this));
  this->updateConnection = event::Events::ConnectRender(std::bind(&DrawTrajectoryPlugin::OnUpdate, this));

  int id;
  if(_sdf->HasElement("id")){
    id = _sdf->Get<int>("id");
  }
  prevPoint = visual->WorldPose().Pos();
  markerMsg.set_ns("visual_draw_trajectory/");
  markerMsg.set_id(id);
  markerMsg.set_action(ignition::msgs::Marker::ADD_MODIFY);
  markerMsg.set_type(ignition::msgs::Marker::LINE_STRIP);
  ignition::msgs::Material *matMsg = markerMsg.mutable_material();
  matMsg->mutable_script()->set_name(material);
  ignition::msgs::Time *timeMsg = markerMsg.mutable_lifetime();
  timeMsg->set_sec(5);
  timeMsg->set_nsec(0);

  nh = new ros::NodeHandle();
}


void DrawTrajectoryPlugin::OnUpdate()
{
  nh->getParamCached("enable_draw", enable_draw);
  if(enable_draw){
    ignition::math::Vector3d point = visual->WorldPose().Pos() + position;
    if(point != ignition::math::Vector3d(0, 0, 0) && point != prevPoint){
      ignition::msgs::Set(markerMsg.add_point(), point);
      if(markerMsg.point_size() >= step){
        node.Request("/marker", markerMsg);
        if(markerMsg.point_size() > length){
          markerMsg.mutable_point()->DeleteSubrange(0, step);
        }
      }
      prevPoint = point;
    }
  }
}
