#include <iostream>
#include "simulator/sim_runner.hpp"
//Matrix<typename, no_rows, no_col> VARNAME;
void sim_runner(Matrix<double, simulator::states_no, 1> Z) {
    //sim loop
    int flight_duration = 100; //seconds

    for (int t = 0; t <= flight_duration; t++) {
        Vector3d torque_command;
        Vector3d body_ext_moment;
        body_ext_moment << 0, 0, 0;
        torque_command = attitude_controller(Z.block<4,1>(0,0), Z.block<4,1>(4,0), 
                        Z.block<3,1>(8,0), Z.block<3,1>(11,0), Z.block<3,1>(14,0), Z.block<3,1>(17,0), body_ext_moment);

    }
}