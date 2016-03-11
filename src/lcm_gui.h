#ifndef rqt_volturnus__LCM_GUI_H
#define rqt_volturnus__LCM_GUI_H

#include <rqt_gui_cpp/plugin.h>
#include <ui_lcm_gui.h>
#include <QWidget>
#include <QString>
#include <ros/ros.h>
#include "lcm_to_ros/nav_solution.h"
#include "lcm_to_ros/compass_stat.h"
#include "lcm_to_ros/dvl_stat.h"
#include "lcm_to_ros/pressure_stat.h"
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
    void on_dvl_message_received();
    void on_pressure_message_received();
    void on_compass_message_received();

signals:
    void navMessageReceived();
    void dvlMessageReceived();
    void pressureMessageReceived();
    void compassMessageReceived();

private:
    Ui::VolturnusWidget ui_;
    QWidget* widget_;
    lcm_to_ros::nav_solution nav_msg_;
    lcm_to_ros::dvl_stat dvl_msg_;
    lcm_to_ros::pressure_stat pressure_msg_;
    lcm_to_ros::compass_stat compass_msg_;
    void navMessageCallback(const lcm_to_ros::nav_solution::ConstPtr& msg);
    void dvlMessageCallback(const lcm_to_ros::dvl_stat::ConstPtr& msg);
    void pressureMessageCallback(const lcm_to_ros::pressure_stat::ConstPtr& msg);
    void compassMessageCallback(const lcm_to_ros::compass_stat::ConstPtr& msg);
    void set_OK(double status, QLabel *qlbl, const QString& text);

protected:
    ros::Subscriber nav_sub_;
    ros::Subscriber dvl_sub_;
    ros::Subscriber pressure_sub_;
    ros::Subscriber compass_sub_;
    double nav_delay_;
    double dvl_delay_;
    double pressure_delay_;
    double compass_delay_;
};

//class LightWidget : public QWidget
//{
//    Q_OBJECT
//    Q_PROPERTY(bool on READ isOn WRITE setOn)

//public:
//    LightWidget(const QColor &coloron, const QColor &coloroff, QWidget *parent = 0)
//        : QWidget(parent), m_coloron(coloron), m_coloroff(coloroff), m_on(false) {}

//    bool isOn() const
//        { return m_on; }
//    void setOn(bool on)
//    {
//        if (on == m_on)
//            return;
//        m_on = on;
//        update();
//    }

//public slots:
//    void turnOff() { setOn(false); }
//    void turnOn() { setOn(true); }

//protected:
//    virtual void paintEvent(QPaintEvent *)
//    {
//        QPainter painter(this);
//        painter.setRenderHint(QPainter::Antialiasing);
//        if (m_on)
//            painter.setBrush(m_coloron);
//        else
//            painter.setBrush(m_coloroff);
//        painter.drawEllipse(0, 0, width(), height());
//    }

//private:
//    QColor m_coloron, m_coloroff;
//    bool m_on;
//};


} // namespace

#endif // rqt_volturnus__LCM_GUI_H
