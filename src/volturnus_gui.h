#ifndef rqt_volturnus__VOLTURNUS_GUI_H
#define rqt_volturnus__VOLTURNUS_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_volturnus_gui.h>
#include <QWidget>
#include <ros/ros.h>
#include "volturnus_comms/Accessory.h"
#include "volturnus_comms/NavigationResponse.h"
#include "volturnus_comms/SensorResponse.h"
#include "volturnus_comms/LightsResponse.h"
//#include <ros/console.h>


namespace rqt_volturnus {

class volturnus_gui
        : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:
    volturnus_gui();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:
    void on_initButton_released();

    void on_left_brighterButton_released();
    void on_right_brighterButton_released();
    void on_left_dimmerButton_released();
    void on_right_dimmerButton_released();

    void on_tilt_upButton_pressed();
    void on_tilt_downButton_pressed();
    void on_tilt_upButton_released();

    void on_gripper_openButton_pressed();
    void on_gripper_closeButton_pressed();
    void on_gripper_openButton_released();

    void on_vert_gainSlider_valueChanged(int value);
    void on_horiz_gainSlider_valueChanged(int value);

    void on_lights_updateButton_released();

    void on_sensors_updateButton_released();

    void on_nav_updateButton_released();

    void on_autodepth_checkBox_toggled(bool checked);
    void on_autoheading_checkBox_toggled(bool checked);
    void on_trim_checkBox_toggled(bool checked);

    void on_sensor_response_received();
    void on_navigation_response_received();
    void on_lights_response_received();


signals:
    void sensorResponseReceived();
    void navigationResponseReceived();
    void lightsResponseReceived();

private:
    Ui::VolturnusWidget ui_;
    QWidget* widget_;
    volturnus_comms::Accessory acc_mess_;
    volturnus_comms::SensorResponse sensor_response_;
    volturnus_comms::NavigationResponse nav_response_;
    volturnus_comms::LightsResponse lights_response_;
    bool auto_depth_, auto_heading_;
    int trim_, vert_gain_, horiz_gain_;
    void sensorResponseCallback(const volturnus_comms::SensorResponse::ConstPtr& msg);
    void navigationResponseCallback(const volturnus_comms::NavigationResponse::ConstPtr& msg);
    void lightsResponseCallback(const volturnus_comms::LightsResponse::ConstPtr& msg);

protected:
    void sendAccMessage(const std::string& acc, const std::string& comm, const int& num=0);
    ros::Publisher acc_command_pub_, request_pub_;
    ros::Subscriber sensor_response_sub_, nav_response_sub_, lights_response_sub_;
};


} // namespace

#endif // rqt_volturnus__VOLTURNUS_GUI_H
