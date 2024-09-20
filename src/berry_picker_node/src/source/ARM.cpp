#include "ARM.h"
using namespace std;

int ARM::connect()
{
    int connected = m_xarm->connect();
    if (connected == 0) {
        m_xarm->motion_enable(true);
        m_xarm->set_mode(0);
        m_xarm->set_state(0);
        sleep_milliseconds(1000);
        // sleep_milliseconds(100000);

        // m_xarm->default_is_radian = true;

        if (m_xarm->has_error()) {
            cout << "Error: " << m_xarm->error_code << endl;
            cout << "reset error: " << m_xarm->clean_error() << endl;
            cout << "Error: " << m_xarm->error_code << endl;
        }
    }
    return connected;
}

int ARM::disconnect()
{
    if (isConnected() == return_codes::SUCCESS) {
        m_xarm->disconnect();
    }
    return return_codes::UNKNOWN;
}

bool ARM::isConnected()
{
    return m_xarm->is_connected();
}

int ARM::getPose(Eigen::VectorXf& pose)
{
    if (isConnected()) {
        pose.resize(6);
        int ret = m_xarm->get_position(pose.data());
        return return_codes::SUCCESS;
    }
    return return_codes::UNKNOWN;
}

int ARM::movePose(Eigen::VectorXf pose, float speed, float acc, float radius, bool wait, int motion_type)
{
    bool relative = false;
    int ret = m_xarm->set_position(pose.data(), radius, speed, acc, 0, wait, NO_TIMEOUT, relative, motion_type);
    return ret;
}

int ARM::movePoseJ(Eigen::VectorXf pose, float speed, float acc, float radius, bool wait)
{
    bool relative = false;
    float angles[7];
    m_xarm->get_inverse_kinematics(pose.data(), angles);
    // set_servo_angle(fp32 angles[7], fp32 speed = 0, fp32 acc = 0, fp32 mvtime = 0, bool wait = false, fp32 timeout = NO_TIMEOUT, fp32 radius = -1, bool relative = false);
    int ret = m_xarm->set_servo_angle(angles, speed, acc, 0, wait, NO_TIMEOUT, radius, relative);
    return ret;
}

int ARM::getCurrentJoint(Eigen::VectorXf& joints)
{
    if (isConnected()) {
        int n_joints = 7;
        int ret;
        joints.resize(n_joints);
        memcpy(joints.data(), m_xarm->angles, sizeof(float) * 7);
        return return_codes::SUCCESS;
    }
    return return_codes::UNKNOWN;
}

int ARM::moveJoint(Eigen::VectorXf& joints, float speed, float acc, float radius, bool wait)
{
    bool relative = false;
    if (joints.size() != 7)
        return -1;

    int ret = m_xarm->set_servo_angle(joints.data(), speed, acc, 0, wait, NO_TIMEOUT, radius, relative);
    return ret;
}

int ARM::moveJointLinear(Eigen::VectorXf& joints, float speed, float acc, float radius, bool wait)
{
    bool relative = false;
    if (joints.size() != 7)
        return -1;

    float pose[6];
    m_xarm->get_forward_kinematics(joints.data(), pose);
    int ret = m_xarm->set_position(pose, radius, speed, acc, 0, wait, NO_TIMEOUT, relative);
    return ret;
}

int ARM::getForwardKinematcis(Eigen::VectorXf& joints, Eigen::VectorXf& pose)
{
    if (isConnected()) {
        float angles[7];
        memcpy(angles, joints.data(), sizeof(float) * 7);
        float pose_[6];
        int ret = m_xarm->get_forward_kinematics(angles, pose_);
        pose.resize(6);
        memcpy(pose.data(), pose_, sizeof(float) * 6);
        pose[3] = RAD2DEG(pose[3]);
        pose[4] = RAD2DEG(pose[4]);
        pose[5] = RAD2DEG(pose[5]);
        return return_codes::SUCCESS;
    }
    return return_codes::UNKNOWN;
}

int ARM::getTransformationFlange(tf_mat<float>& tf_base2flange)
{
    if (isConnected()) {
        Eigen::VectorXf pose(6);
        int ret = m_xarm->get_position(pose.data());

        tf_base2flange = tf_mat<float>(Eigen::Vector3f(pose[0], pose[1], pose[2]), Eigen::Vector3f(pose[3], pose[4], pose[5]), TypeEuler::RPY);
        return return_codes::SUCCESS;
    }
    return return_codes::UNKNOWN;
}

int ARM::getTransformationTCP(tf_mat<float>& tf_base2tcp, const tf_mat<float>& tf_flange2tcp)
{
    if (isConnected()) {
        Eigen::VectorXf pose(6);
        int ret = m_xarm->get_position(pose.data());

        tf_mat<float> tf_base2flange = tf_mat<float>(Eigen::Vector3f(pose[0], pose[1], pose[2]), Eigen::Vector3f(pose[3], pose[4], pose[5]), TypeEuler::RPY);
        tf_base2tcp = tf_base2flange * tf_flange2tcp;

        return return_codes::SUCCESS;
    }
    return return_codes::UNKNOWN;
}

bool ARM::moveTransformationFlange(const tf_mat<float>& tf_base2flange, float speed, float acc, float radius, bool wait, bool joint)
{
    if (isConnected()) {
        Eigen::Vector3f euler_rpy = tf_base2flange.GetEuler();
        Eigen::VectorXf pose(6);
        pose.segment(0, 3) = tf_base2flange.trans_vec;
        pose.segment(3, 3) = euler_rpy;

        // check if its reachable
        float angles[7];
        int return_code = m_xarm->get_inverse_kinematics(pose.data(), angles);
        // cout<<"inverse angles: "<<RAD2DEG(angles[0])<<" "<<RAD2DEG(angles[1])<<" "<<RAD2DEG(angles[2])<<" "
        //<<RAD2DEG(angles[3])<<" "<<RAD2DEG(angles[4])<<" "<<RAD2DEG(angles[5])<<endl;
        if (return_code > 0) {
            cout << "[moveTransformationFlange] pose is not possible to reach!! returning false! " << endl;
            return false;
        }

        int ret_code;
        if (joint) {
            ret_code = movePose(pose, speed, acc, radius, wait, 1);
        } else {
            ret_code = movePose(pose, speed, acc, radius, wait, 0);
        }

        return true;
    }
    return false;
}

bool ARM::moveTransformationTCP(const tf_mat<float>& tf_base2flange, const tf_mat<float>& tf_flange2tcp, float speed, float acc, float radius, bool wait, bool joint)
{
    if (isConnected()) {
        tf_mat<float> tf_base2tcp = tf_base2flange * tf_flange2tcp.Inversed();

        Eigen::Vector3f euler_rpy = tf_base2tcp.GetEuler();
        Eigen::VectorXf pose(6);
        pose.segment(0, 3) = tf_base2tcp.trans_vec;
        pose.segment(3, 3) = euler_rpy;

        // check if its reachable
        float angles[7];
        int return_code = m_xarm->get_inverse_kinematics(pose.data(), angles);
        // cout<<"inverse angles: "<<RAD2DEG(angles[0])<<" "<<RAD2DEG(angles[1])<<" "<<RAD2DEG(angles[2])<<" "
        //<<RAD2DEG(angles[3])<<" "<<RAD2DEG(angles[4])<<" "<<RAD2DEG(angles[5])<<endl;
        if (return_code > 0) {
            cout << "[moveTransformationFlange] pose is not possible to reach!! returning false! " << endl;
            return false;
        }

        int ret_code;
        if (joint) {
            ret_code = movePose(pose, speed, acc, radius, wait, 1);
        } else {
            ret_code = movePose(pose, speed, acc, radius, wait, 0);
        }

        return true;
    }
    return false;
}

bool ARM::checkReachabilityFlange(const tf_mat<float>& tf_base2flange)
{
    if (isConnected()) {
        Eigen::Vector3f euler_rpy = tf_base2flange.GetEuler();
        Eigen::VectorXf pose(6);
        pose.segment(0, 3) = tf_base2flange.trans_vec;
        pose.segment(3, 3) = euler_rpy;

        // check if its reachable
        float angles[7];
        int return_code = m_xarm->get_inverse_kinematics(pose.data(), angles);
        for (int i = 0; i < 6; i++) {
            // cout<<"angles["<<i<<"]: "<<RAD2DEG(angles[i])<<endl;
        }

        if (return_code > 0) {
            return false;
        }
        return true;
    }
    return false;
}

int ARM::goHome(bool wait)
{
    if (isConnected()) {
        Eigen::VectorXf q_curr;
        getCurrentJoint(q_curr);
        cout << q_curr.norm() << endl;
        if (q_curr.norm() < 1e-4)
            return return_codes::UNKNOWN;

        m_xarm->move_gohome(0, 0, 0, wait);
        return return_codes::SUCCESS;
    }
    return return_codes::UNKNOWN;
}

int ARM::setCallback(pCallback_state_changed callback)
{
    cout << "callback is registered? " << m_xarm->register_state_changed_callback(callback) << endl;
    return return_codes::SUCCESS;
}

void ARM::forceStop()
{
    if (isConnected()) {
        m_xarm->emergency_stop();
    }
}

int ARM::setState(int state)
{
    if (isConnected()) {
        return m_xarm->set_state(state);
    }
    return -1;
}

int ARM::getState()
{
    if (isConnected()) {
        return m_xarm->state;
    }
    return -1;
}

int ARM::setMode(int mode, int detection_param)
{
    if (isConnected()) {
        m_xarm->motion_enable(true);
        m_xarm->set_mode(0);
        m_xarm->set_state(0);
        return m_xarm->set_mode(mode, detection_param);
    }
    return -1;
}

int ARM::setDigitalOutput(int id, bool value, bool tcp, float delay)
{
    if (isConnected()) {
        if (tcp) {
            return m_xarm->set_tgpio_digital(id, value, delay);
        } else {
            return m_xarm->set_cgpio_digital(id, value, delay);
        }
    }
    return 0;
}

int ARM::setAnalogOutput(int id, float value)
{
    if (isConnected()) {
        return m_xarm->set_cgpio_analog(id, value);
    }
    return 0;
}

Eigen::VectorXi ARM::getDigitalInput()
{
    Eigen::VectorXi dio(16);
    dio.setZero();
    if (isConnected()) {
        m_xarm->get_cgpio_digital(dio.data(), dio.data() + 8);
    }
    return dio;
}
