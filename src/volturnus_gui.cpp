#include "volturnus_gui.h"
#include "ui_volturnus_gui.h"
#include "VolturnusDefines.h"
#include "std_msgs/UInt8.h"

#include <pluginlib/class_list_macros.h>

namespace rqt_volturnus {


volturnus_gui::volturnus_gui()
    : rqt_gui_cpp::Plugin(),
      widget_(0),
      auto_depth_(0), auto_heading_(0), trim_(0), horiz_gain_(1), vert_gain_(1)
{
    setObjectName("VolturnusGUI");
}

void volturnus_gui::initPlugin(qt_gui_cpp::PluginContext& context)
{
    // create QWidget
    widget_ = new QWidget();

    // access standalone command line arguments
    QStringList argv = context.argv();

    // extend the widget with all attributes and children from UI file
    ui_.setupUi(widget_);
    // add widget to the user interface
    context.addWidget(widget_);

    // Set nav parameter display values
    ui_.autodepth_checkBox->setChecked(auto_depth_);
    ui_.autoheading_checkBox->setChecked(auto_heading_);
    ui_.trim_checkBox->setChecked(trim_);
    ui_.vert_gainSlider->setValue(vert_gain_);
    ui_.horiz_gainSlider->setValue(horiz_gain_);


    // Publisher/s
    acc_command_pub_ = getNodeHandle().advertise<volturnus_comms::Accessory>("/volturnus_comms/acc_command", 10);
    request_pub_ = getNodeHandle().advertise<std_msgs::UInt8>("/volturnus_comms/request", 10);

    // Subscribers
    sensor_response_sub_ = getNodeHandle().subscribe<volturnus_comms::SensorResponse>("/volturnus_comms/sensor_response", 10, &volturnus_gui::sensorResponseCallback, this);

    // CONNECTIONS

    // initButton
    connect(ui_.initButton, SIGNAL(released()), this, SLOT( on_initButton_released() ) );

    // lights
    connect(ui_.left_brighterButton, SIGNAL(released()), this, SLOT(on_left_brighterButton_released()) );
    connect(ui_.right_brighterButton, SIGNAL(released()), this, SLOT(on_right_brighterButton_released()) );
    connect(ui_.left_dimmerButton, SIGNAL(released()), this, SLOT(on_left_dimmerButton_released()) );
    connect(ui_.right_dimmerButton, SIGNAL(released()), this, SLOT(on_right_dimmerButton_released()) );

    // tilt
    connect(ui_.tilt_upButton, SIGNAL(pressed()), this, SLOT(on_tilt_upButton_pressed()) );
    connect(ui_.tilt_downButton, SIGNAL(pressed()), this, SLOT(on_tilt_downButton_pressed()) );
    connect(ui_.tilt_upButton, SIGNAL(released()), this, SLOT(on_tilt_upButton_released()) );
    connect(ui_.tilt_downButton, SIGNAL(released()), this, SLOT(on_tilt_upButton_released()) );

    // gripper
    connect(ui_.gripper_openButton, SIGNAL(pressed()), this, SLOT(on_gripper_openButton_pressed()) );
    connect(ui_.gripper_closeButton, SIGNAL(pressed()), this, SLOT(on_gripper_closeButton_pressed()) );
    connect(ui_.gripper_openButton, SIGNAL(released()), this, SLOT(on_gripper_openButton_released()) );
    connect(ui_.gripper_closeButton, SIGNAL(released()), this, SLOT(on_gripper_openButton_released()) );

    // gain sliders
    connect(ui_.vert_gainSlider, SIGNAL(valueChanged(int)), this, SLOT(on_vert_gainSlider_valueChanged(int)) );
    connect(ui_.horiz_gainSlider, SIGNAL(valueChanged(int)), this, SLOT(on_horiz_gainSlider_valueChanged(int)) );

    // update buttons
    connect(ui_.sensors_updateButton, SIGNAL(released()), this, SLOT( on_sensors_updateButton_released() ) );
    connect(ui_.lights_updateButton, SIGNAL(released()), this, SLOT( on_lights_updateButton_released() ) );
    connect(ui_.nav_updateButton, SIGNAL(released()), this, SLOT( on_nav_updateButton_released() ) );

    // auto depth, heading, trim checkboxes
    connect(ui_.autodepth_checkBox, SIGNAL( toggled(bool) ), this, SLOT( on_autodepth_checkBox_toggled(bool)) );
    connect(ui_.autoheading_checkBox, SIGNAL( toggled(bool) ), this, SLOT( on_autoheading_checkBox_toggled(bool)) );
    connect(ui_.trim_checkBox, SIGNAL( toggled(bool) ), this, SLOT( on_trim_checkBox_toggled(bool) ) );

    // response message receivers
    connect(this, SIGNAL(sensorResponseReceived()), this, SLOT( on_sensor_response_received()) );
}

void volturnus_gui::shutdownPlugin()
{
  // TODO unregister all publishers here
    acc_command_pub_.shutdown();
    request_pub_.shutdown();

    sensor_response_sub_.shutdown();
}

void volturnus_gui::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
{
  // TODO save intrinsic configuration, usually using:
  // instance_settings.setValue(k, v)
}

void volturnus_gui::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
{
  // TODO restore intrinsic configuration, usually using:
  // v = instance_settings.value(k)
}

void volturnus_gui::sensorResponseCallback(const volturnus_comms::SensorResponse::ConstPtr& msg)
{
    sensor_response_ = *msg;
    emit sensorResponseReceived();
}

void volturnus_gui::navigationResponseCallback(const volturnus_comms::NavigationResponse::ConstPtr& msg)
{
    nav_response_ = *msg;
    emit navigationResponseReceived();
}

void volturnus_gui::lightsResponseCallback(const volturnus_comms::LightsResponse::ConstPtr& msg)
{
    lights_response_ = *msg;
    emit lightsResponseReceived();
}


void rqt_volturnus::volturnus_gui::sendAccMessage(const std::string& acc, const std::string& comm, const int& num )
{
    acc_mess_.accessory_name = acc;
    acc_mess_.command = comm;
    acc_mess_.accessory_number = num;
    acc_command_pub_.publish(acc_mess_);
}

void rqt_volturnus::volturnus_gui::on_initButton_released()
{
    auto_depth_ = false;
    auto_heading_ = false;
    trim_ = false;
    horiz_gain_ = 5;
    vert_gain_ = 5;

    ui_.autodepth_checkBox->setChecked(auto_depth_);
    ui_.autoheading_checkBox->setChecked(auto_heading_);
    ui_.trim_checkBox->setChecked(trim_);
    ui_.vert_gainSlider->setValue(vert_gain_);
    ui_.horiz_gainSlider->setValue(horiz_gain_);
}

void rqt_volturnus::volturnus_gui::on_sensor_response_received()
{
    ui_.sensors_depthLCD->display(sensor_response_.depth);
    ui_.sensors_headingLCD->display(sensor_response_.heading);
    ui_.sensors_pitchLCD->display(sensor_response_.pitch);
    ui_.sensors_rollLCD->display(sensor_response_.roll);
    ui_.sensors_altitudeLCD->display(sensor_response_.altitude);
    ui_.sensors_turnsLCD->display(sensor_response_.turns);
    ui_.sensors_temp_intLCD->display(sensor_response_.temp_int);
    ui_.sensors_temp_extLCD->display(sensor_response_.temp_ext);
    ui_.sensors_press_intLCD->display(sensor_response_.press_int);
    ui_.sensors_desired_headingLCD->display(sensor_response_.desired_heading);
    ui_.sensors_desired_depthLCD->display(sensor_response_.desired_depth);
    ui_.sensors_desired_altitudeLCD->display(sensor_response_.desired_altitude);
    ui_.sensors_lowV_power_supplyLCD->display(sensor_response_.lowV_power_supply);
    ui_.sensors_hiV_power_supplyLCD->display(sensor_response_.hiV_power_supply);

    if ((sensor_response_.VPS_status & 0x80) > 0)
        ui_.sensors_waterdetect->setText("True");
    else
        ui_.sensors_waterdetect->setText("False");
}

void rqt_volturnus::volturnus_gui::on_navigation_response_received()
{
    ui_.nav_headingPLCD->display(nav_response_.heading_p);
    ui_.nav_headingILCD->display(nav_response_.heading_i);
    ui_.nav_headingDLCD->display(nav_response_.heading_d);
    ui_.nav_depthPLCD->display(nav_response_.depth_alt_p);
    ui_.nav_depthILCD->display(nav_response_.depth_alt_i);
    ui_.nav_depthDLCD->display(nav_response_.depth_alt_d);
    ui_.nav_declinationLCD->display(nav_response_.declination);
    ui_.nav_vpowerLCD->display(nav_response_.vehicle_power_supply);
    ui_.nav_hpowerLCD->display(nav_response_.hotel_power_supply);
    switch (nav_response_.units)
    {
        case 1:
            ui_.nav_units->setText("METRIC SEAWATER");
            break;
        case 2:
            ui_.nav_units->setText("METRIC FRESHWATER");
            break;
        case 3:
            ui_.nav_units->setText("US SEAWATER");
            break;
        case 4:
            ui_.nav_units->setText("US FRESHWATER");
            break;
        default:
            ui_.nav_units->setText("NAV UNITS NOT RECOGNISED!");
    }
}

void rqt_volturnus::volturnus_gui::on_lights_response_received()
{
    ui_.sensors_depthLCD->display(sensor_response_.depth);
    ui_.sensors_headingLCD->display(sensor_response_.heading);
    ui_.sensors_pitchLCD->display(sensor_response_.pitch);
    ui_.sensors_rollLCD->display(sensor_response_.roll);
    ui_.sensors_altitudeLCD->display(sensor_response_.altitude);
    ui_.sensors_turnsLCD->display(sensor_response_.turns);
    ui_.sensors_temp_intLCD->display(sensor_response_.temp_int);
    ui_.sensors_temp_extLCD->display(sensor_response_.temp_ext);
    ui_.sensors_press_intLCD->display(sensor_response_.press_int);
    ui_.sensors_desired_headingLCD->display(sensor_response_.desired_heading);
    ui_.sensors_desired_depthLCD->display(sensor_response_.desired_depth);
    ui_.sensors_desired_altitudeLCD->display(sensor_response_.desired_altitude);
    ui_.sensors_lowV_power_supplyLCD->display(sensor_response_.lowV_power_supply);
    ui_.sensors_hiV_power_supplyLCD->display(sensor_response_.hiV_power_supply);

    if ((sensor_response_.VPS_status & 0x80) > 0)
        ui_.sensors_waterdetect->setText("True");
    else
        ui_.sensors_waterdetect->setText("False");
}


void rqt_volturnus::volturnus_gui::on_left_brighterButton_released()
{
    sendAccMessage("lights", "brighter", LIGHT1_NUMBER);
}

void rqt_volturnus::volturnus_gui::on_right_brighterButton_released()
{
    sendAccMessage("lights", "brighter", LIGHT2_NUMBER);
}

void rqt_volturnus::volturnus_gui::on_left_dimmerButton_released()
{
    sendAccMessage("lights", "dimmer", LIGHT1_NUMBER);
}

void rqt_volturnus::volturnus_gui::on_right_dimmerButton_released()
{
    sendAccMessage("lights", "dimmer", LIGHT2_NUMBER);
}

void rqt_volturnus::volturnus_gui::on_tilt_upButton_pressed()
{
    sendAccMessage("tilt", "up");
}

void rqt_volturnus::volturnus_gui::on_tilt_downButton_pressed()
{
    sendAccMessage("tilt", "down");
}

void rqt_volturnus::volturnus_gui::on_tilt_upButton_released()
{
    sendAccMessage("tilt", "stop");
}

void rqt_volturnus::volturnus_gui::on_gripper_openButton_pressed()
{
    sendAccMessage("gripper", "open");
}

void rqt_volturnus::volturnus_gui::on_gripper_closeButton_pressed()
{
    sendAccMessage("gripper", "close");
}

void rqt_volturnus::volturnus_gui::on_gripper_openButton_released()
{
    sendAccMessage("gripper", "stop");
}

void rqt_volturnus::volturnus_gui::on_vert_gainSlider_valueChanged(int value)
{
    int tvalue = (value >= 10) ? 0 : value;
    sendAccMessage("status", "vert_gain", tvalue);
}

void rqt_volturnus::volturnus_gui::on_horiz_gainSlider_valueChanged(int value)
{
    int tvalue = (value >= 10) ? 0 : value;
    sendAccMessage("status", "horiz_gain", tvalue);
}

void rqt_volturnus::volturnus_gui::on_lights_updateButton_released()
{
    std_msgs::UInt8 msg;
    msg.data = REQUEST_LIGHTS;
    request_pub_.publish(msg);
}

void rqt_volturnus::volturnus_gui::on_sensors_updateButton_released()
{
    std_msgs::UInt8 msg;
    msg.data = REQUEST_SENSOR;
    request_pub_.publish(msg);
}

void rqt_volturnus::volturnus_gui::on_nav_updateButton_released()
{
    std_msgs::UInt8 msg;
    msg.data = REQUEST_NAVIGATION;
    request_pub_.publish(msg);
}

void rqt_volturnus::volturnus_gui::on_autodepth_checkBox_toggled(bool checked)
{
    if (checked)
        sendAccMessage("status", "auto_depth", 1);
    else
        sendAccMessage("status", "auto_depth", 0);
}

void rqt_volturnus::volturnus_gui::on_autoheading_checkBox_toggled(bool checked)
{
    if (checked)
        sendAccMessage("status", "auto_heading", 1);
    else
        sendAccMessage("status", "auto_heading", 0);
}

void rqt_volturnus::volturnus_gui::on_trim_checkBox_toggled(bool checked)
{
    if (checked)
        sendAccMessage("status", "trim", 1);
    else
        sendAccMessage("status", "trim", 0);
}


} //namespace

PLUGINLIB_EXPORT_CLASS(rqt_volturnus::volturnus_gui, rqt_gui_cpp::Plugin)

