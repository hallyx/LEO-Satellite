#include "Eigen/Dense"
#include <cmath>
#include <iostream>
using namespace Eigen;

/**
 * @brief Computes torque command from a nonlinear Lyapunov stable controller
 *          -credit Boulder ASEN 6010 - Prof. Schaub
 *          - inputs:
 *                      * q_B2O = body frame to Origin frame quaternion
 *                      * q_R2O = reference frame to Origin frame quaternion
 *                      * body_w = body angular velocity in body frame
 *                      * body_w_ref = reference angular velocity in body frame
 *                      * body_wdot = body angular acceleration in body frame
 *                      * body_wdot_ref = reference angular acceleration in body frame
 *       
 *                      * ext_torque = external torque i.e. gravity gradient or external torque
 * 
 */
Vector3d attitude_controller(const Vector4d &q_B2O, const Vector4d &q_R2O, const Vector3d &body_w, const Vector3d &body_w_ref , const Vector3d &body_wdot, const Vector3d &body_wdot_ref, const Vector3d &body_ext_torque);