#include "satellite_data.hpp"
#include "gnc/attitude_controller.hpp"

Vector3d attitude_controller(const Vector4d &q_B2O, const Vector4d &q_R2O, const Vector3d &body_w, const Vector3d &body_w_ref , const Vector3d &body_wdot, const Vector3d &body_wdot_ref, const Vector3d &body_ext_torque) {
    Vector3d command_torque;
    Vector3d mrp_B2R; // modified rodrigues parameters
    Vector3d delta_w;
    Vector3d wdot_r;
    Matrix3d wcross; //crossproduct matrix [w[crossproduct]]
    Matrix3d MOI;
    
    Vector4d q_B2R;
    Matrix4d O_q_R;
    // quaternion rotation matrix
    O_q_R << q_R2O[0], -q_R2O[1], -q_R2O[2], -q_R2O[3], 
             q_R2O[1], q_R2O[0], q_R2O[3], -q_R2O[2], 
             q_R2O[2], -q_R2O[3], q_R2O[0], q_R2O[1], 
             q_R2O[3], q_R2O[2], -q_R2O[1], -q_R2O[0];
    //adding q_B2O and q_R2O to get q_B2R to then convert to an MRP for the attitude control law
    q_B2R = O_q_R.inverse()*q_B2O;
    //conversion of q_B2R into the relevant Modified Rodrigues Parameter with a check for shadow MRP

    double s1 = q_B2R[1]/(1+q_B2R[0]);
    double s2 = q_B2R[2]/(1+q_B2R[0]);
    double s3 = q_B2R[3]/(1+q_B2R[0]);
    mrp_B2R << s1,s2,s3;

    if (mrp_B2R.dot(mrp_B2R) > 1) {
        s1 = -q_B2R[1]/(1-q_B2R[0]);
        s2 = -q_B2R[2]/(1-q_B2R[0]);
        s3 = -q_B2R[3]/(1-q_B2R[0]);
        mrp_B2R << s1,s2,s3;
    }

    MOI << spacecraft::MOI_1, 0, 0, 0, spacecraft::MOI_2, 0, 0, 0, spacecraft::MOI_3;

    wcross <<  0, -body_w[2], body_w[1], body_w[2], 0, -body_w[0], -body_w[1], body_w[0], 0;

    delta_w = body_w - body_w_ref;

    command_torque = -attitude::K_K*mrp_B2R - attitude::K_P*delta_w + MOI*(body_wdot_ref - wcross*body_w_ref) + wcross*MOI*body_w - body_ext_torque;
    
    return command_torque;
}