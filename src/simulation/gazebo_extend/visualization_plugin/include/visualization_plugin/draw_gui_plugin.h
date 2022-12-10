#ifndef DRAW_GUI_PLUGIN_HH
#define DRAW_GUI_PLUGIN_HH

#include <ros/ros.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

namespace gazebo
{
  class GAZEBO_VISIBLE DrawGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    /// \param[in] _parent Parent widget
    public: DrawGUIPlugin();

    /// \brief Destructor
    public: virtual ~DrawGUIPlugin();

    /// \brief Callback trigged when the button is pressed.
    protected slots: void OnButtonClicked();

    private:
    QPushButton *button;
    bool enable_draw = true;
    ros::NodeHandle* nh;
  };
}
#endif
