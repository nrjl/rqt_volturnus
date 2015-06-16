#include "volturnus_gui.h"
#include "ui_volturnus_gui.h"
#include "VolturnusDefines.h"

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


    // Messages
    p_acc_mess = new volturnus_comms::Accessory;

    // Publisher/s
    acc_command_pub_ = getNodeHandle().advertise<volturnus_comms::Accessory>("/volturnus_comms/acc_command", 10);

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
}

void volturnus_gui::shutdownPlugin()
{
  // TODO unregister all publishers here
    delete p_acc_mess;
    acc_command_pub_.shutdown();
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

void rqt_volturnus::volturnus_gui::sendAccMessage(const std::string& acc, const std::string& comm, const int& num )
{
    p_acc_mess->accessory_name = acc;
    p_acc_mess->command = comm;
    p_acc_mess->accessory_number = num;
    acc_command_pub_.publish(*p_acc_mess);
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

    //sendAccMessage("status", "auto_depth", auto_depth_);
    //sendAccMessage("status", "auto_heading", auto_heading_);
    //sendAccMessage("status", "trim", trim_);
    //sendAccMessage("status", "vert_gain", vert_gain_);
    //sendAccMessage("status", "horiz_gain", horiz_gain_);
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
    sendAccMessage("request", "lights");
}

void rqt_volturnus::volturnus_gui::on_sensors_updateButton_released()
{
    sendAccMessage("request", "sensors");
}

void rqt_volturnus::volturnus_gui::on_nav_updateButton_released()
{
    sendAccMessage("request", "navigation");
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

