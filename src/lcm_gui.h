#ifndef rqt_volturnus__LCM_GUI_H
#define rqt_volturnus__LCM_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_lcm_gui.h>
#include <QWidget>
#include <QString>
#include <ros/ros.h>
#include "lcm_to_ros/STAT_Nav_soln.h"
//#include <ros/console.h>


namespace rqt_volturnus {

class lcm_gui
        : public rqt_gui_cpp::Plugin
{

    Q_OBJECT

public:
    lcm_gui();
    virtual void initPlugin(qt_gui_cpp::PluginContext& context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

private slots:
    void on_nav_message_received();

signals:
    void navMessageReceived();

private:
    Ui::VolturnusWidget ui_;
    QWidget* widget_;
    lcm_to_ros::STAT_Nav_soln nav_msg_;
    void navMessageCallback(const lcm_to_ros::STAT_Nav_soln::ConstPtr& msg);
    void set_OK(double status, QLabel *qlbl, const QString& text);

protected:
    ros::Subscriber nav_sub_;
};


} // namespace

#endif // rqt_volturnus__LCM_GUI_H
