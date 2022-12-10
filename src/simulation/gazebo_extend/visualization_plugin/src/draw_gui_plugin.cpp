#include "visualization_plugin/draw_gui_plugin.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(DrawGUIPlugin)

DrawGUIPlugin::DrawGUIPlugin() : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
    "QFrame{"
    "  color: white;"
    "  background-color: rgba(100, 100, 100, 255);"
    "}"
  );

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout();

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  button = new QPushButton(tr("Disable Draw"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButtonClicked()));

  // Add the button to the frame's layout
  frameLayout->addWidget(button);
  frameLayout->setContentsMargins(0, 0, 0, 0);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);
  mainFrame->setContentsMargins(0, 0, 0, 0);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(120, 40);

  nh = new ros::NodeHandle();
}


DrawGUIPlugin::~DrawGUIPlugin(){}


void DrawGUIPlugin::OnButtonClicked()
{
  enable_draw = !enable_draw;
  if(enable_draw){
    button->setText("Disable Draw");
  }
  else{
    button->setText("Enable Draw");
  }
  nh->setParam("enable_draw", enable_draw);
}
