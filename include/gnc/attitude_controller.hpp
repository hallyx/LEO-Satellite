#include "Eigen/Dense"
#include <cmath>
#include <iostream>
using namespace Eigen;

/**
 * @brief Computes torque command from a nonlinear Lyapunov stable controller
 *          -credit Boulder ASEN 6010 - Prof. Schaub
 *          - inputs:
 *                      * bodyquat = estimated/true body quaternion
 *                      * bodyw = "" body angular velocity
 *                      * refquat = reference quaternion
 *                      * refw = reference angular velocity
 *                      * ext_torque = external torque i.e. gravity gradient or external torque
 * 
 */
Vector3d attitude_controller(const Vector4d &q_B2R, const Vector3d &body_w, const Vector3d &body_w_ref , const Vector3d &body_wdot, const Vector3d &body_wdot_ref, const Vector3d &body_ext_torque);