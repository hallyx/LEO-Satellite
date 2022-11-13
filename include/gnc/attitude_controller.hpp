#include "Eigen/Dense"
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
Vector3d attitude_controller(const Vector4d &bodyquat, const Vector3d &bodyw, Vector4d &refquat, Vector3d &refw , Vector3d &ext_torque);