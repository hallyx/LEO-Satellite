
namespace attitude {
    static constexpr double K_K = 2.3;
    static constexpr double K_P = K_K*400; // in attitude_controller.cpp multiply by identity matrix
}
namespace spacecraft {
    static constexpr double MOI_1 = 9000; //metric units
    static constexpr double MOI_2 = 30000;
    static constexpr double MOI_3 = 35000;
    static constexpr double dt = 1;
}

namespace orbit {
    //static constexpr double mu_body1 = ;
    //static constexpr double mu_body2 = ;
}