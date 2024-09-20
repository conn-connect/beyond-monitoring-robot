#pragma once
#include "tf_mat.h"
#include "xarm/wrapper/xarm_api.h"
#include <cstdlib>
#include <eigen3/Eigen/Core>
#include <iostream>

typedef std::function<void(int)> pCallback_state_changed;

namespace return_codes {
enum ReturnCode {
    SUCCESS = 0,
    UNKNOWN = 1
};
}

namespace xarm_states {
enum XarmState {
    IN_MOTION = 1,
    SLEEPING,
    SUSPENDED,
    STOPPING
};
}

namespace motion_types {
enum MotionType {
    IN_MOTION = 1,
    SLEEPING,
    SUSPENDED,
    STOPPING
};
}

class ARM {
private:
    std::shared_ptr<XArmAPI> m_xarm;

public:
    ARM(std::string ip_address)
    {
        m_xarm = std::make_shared<XArmAPI>(ip_address, true, true, true, true, true, false, true, true, 0);
        // printf("register events\n");
        // m_xarm->register_report_location_callback(report_location_callback);
        // m_xarm->register_connect_changed_callback(connect_changed_callback);
        // m_xarm->register_state_changed_callback(state_changed_callback);
        // m_xarm->register_mode_changed_callback(mode_changed_callback);
        // m_xarm->register_mtable_mtbrake_changed_callback(mtable_mtbrake_changed_callback);
        // m_xarm->register_error_warn_changed_callback(error_warn_changed_callback);
        // m_xarm->register_cmdnum_changed_callback(cmdnum_changed_callback);
    }
    ARM() {};
    ~ARM()
    {
        disconnect();
        // printf("release events\n");
        // m_xarm->release_report_location_callback(report_location_callback);
        // m_xarm->release_connect_changed_callback(connect_changed_callback);
        // m_xarm->release_state_changed_callback(state_changed_callback);
        // m_xarm->release_mode_changed_callback(mode_changed_callback);
        // m_xarm->release_mtable_mtbrake_changed_callback(mtable_mtbrake_changed_callback);
        // m_xarm->release_error_warn_changed_callback(error_warn_changed_callback);
        // m_xarm->release_cmdnum_changed_callback(cmdnum_changed_callback);
        sleep_milliseconds(1000);
    }

    /**
     * @brief Connect to xArm
     *
     * @param port: port name or the ip address
     * @return:
     *   0: success
     *  -1: port is empty
     *  -2: tcp control connect failed
     *  -3: tcp report connect failed
     */

    ////connection
    int connect();
    int disconnect();
    bool isConnected();

    ////move
    // positions
    int getPose(Eigen::VectorXf& pose);
    int movePose(Eigen::VectorXf pose, float speed, float acc, float radius, bool wait = true, int motion_type = 1);
    int movePoseJ(Eigen::VectorXf pose, float speed, float acc, float radius, bool wait = true);

    // set jerk
    void set_tcp_jerk(float jerk)
    {
        m_xarm->set_tcp_jerk(jerk);
    }
    void set_joint_jerk(float jerk)
    {
        m_xarm->set_joint_jerk(jerk);
    }

    // joints
    int getCurrentJoint(Eigen::VectorXf& joints);
    int moveJoint(Eigen::VectorXf& joints, float speed, float acc, float radius, bool wait = true);
    int moveJointLinear(Eigen::VectorXf& joints, float speed, float acc, float radius, bool wait = true);
    int getForwardKinematcis(Eigen::VectorXf& joints, Eigen::VectorXf& pose);

    // transformations
    int getTransformationFlange(tf_mat<float>& tf_base2flange);
    int getTransformationTCP(tf_mat<float>& tf_base2flange, const tf_mat<float>& tf_flange2tcp);
    bool moveTransformationFlange(const tf_mat<float>& tf_base2flange, float speed, float acc, float radius, bool wait, bool joint);
    bool moveTransformationTCP(const tf_mat<float>& tf_base2flange, const tf_mat<float>& tf_flange2tcp, float speed, float acc, float radius, bool wait, bool joint);
    bool checkReachabilityFlange(const tf_mat<float>& tf_base2flange);

    ////utilities
    int goHome(bool wait = false);
    int setCallback(pCallback_state_changed callback);
    void forceStop();
    int setState(int state); // reset state after forceStop

    // 1: in motion
    // 2: sleeping
    // 3: suspended
    // 4: stopping
    int getState();

    int setMode(int mode, int detection_param = 0);

    void close_gripper()
    {
        setAnalogOutput(0, 0);
        // m_xarm->open_lite6_gripper();
    }

    void open_gripper()
    {
        setAnalogOutput(0, 10);
        // m_xarm->close_lite6_gripper();
    }

    void stop_gripper()
    {
        m_xarm->stop_lite6_gripper();
    }

    void cut_gripper()
    {
        setAnalogOutput(0, 10);
    }

    void suction_on()
    {
        setAnalogOutput(0, 10);
    }

    void vacuum_on(float vol = 17)
    {
        setAnalogOutput(1, vol);
    }

    void suction_off()
    {
        setAnalogOutput(0, 0);
    }

    ////IO
    int setDigitalOutput(int id, bool value, bool tcp = true, float delay = 0);
    int setAnalogOutput(int id, float value);
    Eigen::VectorXi getDigitalInput();
};