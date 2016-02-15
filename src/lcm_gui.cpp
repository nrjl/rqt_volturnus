#include "lcm_gui.h"
#include "ui_lcm_gui.h"
#include <QThread>
//#include <QDateTime>
//#include <QTime>

#include <pluginlib/class_list_macros.h>

namespace rqt_volturnus {

lcm_gui::lcm_gui()
    : rqt_gui_cpp::Plugin(),
      widget_(0)
{
    setObjectName("LCMGUI");
}

void lcm_gui::initPlugin(qt_gui_cpp::PluginContext& context)
{

    // create QWidget
    widget_ = new QWidget();

    // access standalone command line arguments
    QStringList argv = context.argv();

    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    // Subscribers
    nav_sub_ = getNodeHandle().subscribe<lcm_to_ros::STAT_Nav_soln>("/lcm_to_ros/STAT_Nav_soln", 10, &lcm_gui::navMessageCallback, this);

    // CONNECTIONS
    // response message receivers
    connect(this, SIGNAL(navMessageReceived()), this, SLOT( on_nav_message_received()) );
}

void lcm_gui::shutdownPlugin()
{
    // nav_sub_.shutdown();
}

void lcm_gui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void lcm_gui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void lcm_gui::navMessageCallback(const lcm_to_ros::STAT_Nav_soln::ConstPtr& msg)
{
    nav_msg_ = *msg;
    emit navMessageReceived();
}

void lcm_gui::set_OK(double status, QLabel* qlbl, const QString& text)
{
    if (status) {
        qlbl->setText(QString("%1 OK").arg(text));
        qlbl->setStyleSheet("color: black;");
    }
    else {
        qlbl->setText(QString("%1 NOT OK").arg(text));
        qlbl->setStyleSheet("color: red;");
    }
}

void lcm_gui::on_nav_message_received()
{
    ui_.roll_LCD->display(nav_msg_.roll);
    ui_.roll_rate_LCD->display(nav_msg_.roll_rate);
    ui_.pitch_LCD->display(nav_msg_.pitch);
    ui_.pitch_rate_LCD->display(nav_msg_.pitch_rate);
    ui_.heading_LCD->display(nav_msg_.heading);
    ui_.heading_rate_LCD->display(nav_msg_.heading_rate);

    ui_.latitude_LCD->display(nav_msg_.latitude);
    ui_.longitude_LCD->display(nav_msg_.longitude);

    ui_.depth_LCD->display(nav_msg_.depth);
    ui_.depth_rate_LCD->display(nav_msg_.depth_velocity);

    ui_.rel_east_LCD->display(nav_msg_.relative_easting);
    ui_.rel_east_rate_LCD->display(nav_msg_.relative_easting_velocity);
    ui_.rel_north_LCD->display(nav_msg_.relative_northing);
    ui_.rel_north_rate_LCD->display(nav_msg_.relative_northing_velocity);

    ui_.speed_over_ground_LCD->display(nav_msg_.speed_over_ground);
    ui_.course_over_ground_LCD->display(nav_msg_.course_over_ground);

    set_OK(nav_msg_.attitude_OK, ui_.attitude_OK, "Attitude");
    set_OK(nav_msg_.altitude_OK, ui_.altitude_OK, "Altitude");
    set_OK(nav_msg_.depth_OK, ui_.depth_OK, "Depth");
    set_OK(nav_msg_.GPS_OK, ui_.GPS_OK, "GPS");
    set_OK(nav_msg_.relative_position_OK, ui_.rel_pos_OK, "Relative Position");
    set_OK(nav_msg_.absolute_position_OK, ui_.abs_pos_OK, "Absolute Position");
    set_OK(nav_msg_.bottom_lock_OK, ui_.DVL_OK, "DVL Bottom Lock");
}

} //namespace

PLUGINLIB_EXPORT_CLASS(rqt_volturnus::lcm_gui, rqt_gui_cpp::Plugin)

