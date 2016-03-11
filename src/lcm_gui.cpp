#include "lcm_gui.h"
#include "ui_lcm_gui.h"
#include <iostream>
#include <QThread>
#include <QLCDNumber>
#include <iterator>
#include <sys/time.h>
//#include <QDateTime>
//#include <QTime>

#include <pluginlib/class_list_macros.h>

namespace rqt_volturnus {

lcm_gui::lcm_gui()
    : rqt_gui_cpp::Plugin(),
      widget_(0),
      nav_delay_(0),
      dvl_delay_(0),
      pressure_delay_(0),
      compass_delay_(0)
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

    // Make LCDs flat
    QList<QLCDNumber *> allLCDs = widget_->findChildren<QLCDNumber *>();
    foreach(QLCDNumber * item, allLCDs) {
        item->setSegmentStyle(QLCDNumber::Flat);
    }

    // Subscribers
    nav_sub_ = getNodeHandle().subscribe<lcm_to_ros::nav_solution>("/lcm_to_ros/OPENINS_NAV_SOLUTION", 10, &lcm_gui::navMessageCallback, this);
    dvl_sub_ = getNodeHandle().subscribe<lcm_to_ros::dvl_stat>("/lcm_to_ros/OPENINS_DVL_STAT", 10, &lcm_gui::dvlMessageCallback, this);
    compass_sub_ = getNodeHandle().subscribe<lcm_to_ros::compass_stat>("/lcm_to_ros/OPENINS_COMPASS_STAT", 10, &lcm_gui::compassMessageCallback, this);
    pressure_sub_ = getNodeHandle().subscribe<lcm_to_ros::pressure_stat>("/lcm_to_ros/OPENINS_PRESSURE_STAT", 10, &lcm_gui::pressureMessageCallback, this);

    // CONNECTIONS
    // response message receivers
    connect(this, SIGNAL(navMessageReceived()), this, SLOT( on_nav_message_received()) );
    connect(this, SIGNAL(dvlMessageReceived()), this, SLOT( on_dvl_message_received()) );
    connect(this, SIGNAL(pressureMessageReceived()), this, SLOT( on_pressure_message_received()) );
    connect(this, SIGNAL(compassMessageReceived()), this, SLOT( on_compass_message_received()) );
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

void lcm_gui::navMessageCallback(const lcm_to_ros::nav_solution::ConstPtr& msg)
{
    nav_msg_ = *msg;
    emit navMessageReceived();
}

void lcm_gui::dvlMessageCallback(const lcm_to_ros::dvl_stat::ConstPtr& msg)
{
    dvl_msg_ = *msg;
    emit dvlMessageReceived();
}
void lcm_gui::pressureMessageCallback(const lcm_to_ros::pressure_stat::ConstPtr& msg)
{
    pressure_msg_ = *msg;
    emit pressureMessageReceived();
}

void lcm_gui::compassMessageCallback(const lcm_to_ros::compass_stat::ConstPtr& msg)
{
    compass_msg_ = *msg;
    emit compassMessageReceived();
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

    ui_.latitude_LCD->display(nav_msg_.last_lonlat_fix_1);
    ui_.longitude_LCD->display(nav_msg_.last_lonlat_fix_0);

    ui_.depth_LCD->display(nav_msg_.depth);
    ui_.depth_rate_LCD->display(nav_msg_.relative_velocity_z);

    ui_.rel_east_LCD->display(nav_msg_.relative_position_x);
    ui_.rel_east_rate_LCD->display(nav_msg_.relative_velocity_x);
    ui_.rel_north_LCD->display(nav_msg_.relative_position_y);
    ui_.rel_north_rate_LCD->display(nav_msg_.relative_position_y);

    ui_.speed_over_ground_LCD->display(nav_msg_.speed_over_ground);
    ui_.course_over_ground_LCD->display(nav_msg_.course_over_ground);

    set_OK(nav_msg_.attitude_ok, ui_.attitude_OK, "Attitude");
    set_OK(nav_msg_.altitude_ok, ui_.altitude_OK, "Altitude");
    set_OK(nav_msg_.depth_ok, ui_.depth_OK, "Depth");
    set_OK(nav_msg_.absolute_position_ok, ui_.GPS_OK, "GPS");
    set_OK(nav_msg_.relative_position_ok, ui_.rel_pos_OK, "Relative Position");
    set_OK(nav_msg_.absolute_position_ok, ui_.abs_pos_OK, "Absolute Position");
    set_OK(nav_msg_.have_bottom_lock, ui_.DVL_OK, "DVL Bottom Lock");
}

void lcm_gui::on_dvl_message_received()
{
    set_OK(dvl_msg_.btm_channel.lock, ui_.dvl_btm_lock, "LOCK");
    ui_.dvl_btm_velx_LCD->display(dvl_msg_.btm_channel.vel_x);
    ui_.dvl_btm_vely_LCD->display(dvl_msg_.btm_channel.vel_y);
    ui_.dvl_btm_velz_LCD->display(dvl_msg_.btm_channel.vel_z);
    ui_.dvl_btm_alt_LCD->display(dvl_msg_.btm_channel.altitude);

    set_OK(dvl_msg_.ref_channel.lock, ui_.dvl_ref_lock, "LOCK");
    ui_.dvl_ref_velx_LCD->display(dvl_msg_.ref_channel.vel_x);
    ui_.dvl_ref_vely_LCD->display(dvl_msg_.ref_channel.vel_y);
    ui_.dvl_ref_velz_LCD->display(dvl_msg_.ref_channel.vel_z);
    ui_.dvl_ref_alt_LCD->display(dvl_msg_.ref_channel.altitude);
}

void lcm_gui::on_pressure_message_received()
{
    ui_.temperature_LCD->display(pressure_msg_.channels[0].value);
    ui_.pressure_LCD->display(pressure_msg_.pressure_psi);
}

void lcm_gui::on_compass_message_received()
{
    set_OK(compass_msg_.attitude_valid, ui_.compass_att_status, "Attitude valid");
    ui_.compass_roll_LCD->display(compass_msg_.attitude_roll_deg);
    ui_.compass_pitch_LCD->display(compass_msg_.attitude_pitch_deg);
    ui_.compass_heading_LCD->display(compass_msg_.attitude_heading_deg);
    ui_.compass_distortion_LCD->display(compass_msg_.channels[0].value);
}


} //namespace

PLUGINLIB_EXPORT_CLASS(rqt_volturnus::lcm_gui, rqt_gui_cpp::Plugin)

