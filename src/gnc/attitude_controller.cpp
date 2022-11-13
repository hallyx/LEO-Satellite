#include "satellite_data.hpp"
#include "gnc/attitude_controller.hpp"

Vector3d attitude_controller(const Vector4d &bodyquat, const Vector3d &bodyw, Vector4d &refquat, Vector3d &refw , Vector3d &ext_torque) {
    Vector3d command_torque;
    Vector3d sigma_mrp; // modified rodrigues parameters
    Vector3d delta_w;
    Vector3d B_wdot_r;
    Vector3d wdot_r;
    Matrix3d wcross; //crossproduct matrix [w[crossproduct]]
    Vector3d w_r;
    Vector3d w;
    Matrix3d MOI;
    
    MOI << spacecraft::MOI_1, 0, 0, 0, spacecraft::MOI_2, 0, 0, 0, spacecraft::MOI_3;

    wcross <<  0, -w(2), w(1), w(2), 0, -w(0), -w(1), w(0), 0;

    command_torque = -attitude::K_K*sigma_mrp - attitude::K_P*delta_w + MOI*(wdot_r - wcross*w_r) + wcross*MOI*w - ext_torque;
    return command_torque;
}