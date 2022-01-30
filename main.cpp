#include "IK.h"

/**
 * An example how the inverse kinematics can be used.
 * It should not be required to change this code.
 */
int main()
{

    // initial conditions and desired goal
    vector_t q_start(3);
    q_start << -0.1, -0.1, -0.1;
    std::cout << q_start << std::endl;

    trafo2d_t goal = trafo2d_t::Identity();
    goal.translation()(0) = 1.;
    vector_t q_val(3);
    // q_val = inverse_kinematics(q_start, goal);

    IK ik(q_start, goal);

    return 0;
}
