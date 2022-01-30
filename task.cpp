#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>



typedef Eigen::VectorXd vector_t;
typedef Eigen::MatrixXd matrix_t;
typedef Eigen::Transform<double,2,Eigen::Affine> trafo2d_t;



/**************************************************
 * A function to compute the forward kinematics of
 * a planar 3-link robot.
 *************************************************/
trafo2d_t forward_kinematics(const vector_t& q ) {
    // check that the joint angle vector has the correct size
    assert( q.size() == 3 );

    // define a constant offset between two joints
    trafo2d_t link_offset = trafo2d_t::Identity();
    link_offset.translation()(1) = 1.;

    // define the start pose
    trafo2d_t trafo = trafo2d_t::Identity();

    for(int joint_idx = 0; joint_idx < 3 ; joint_idx++ ) {
        // add the rotation contributed by this joint
        trafo *= Eigen::Rotation2D<double>(q(joint_idx));
        // add the link offset to this position
        trafo = trafo * link_offset;
    }
    return trafo;
}



/*************************************************
 * Task: Complete at least 2 of 3 Parts
 * 
 * Part 1.
 * ^^^^^^^
 * Complete this inverse kinematics function for the robot defined by
 * the forward kinematics function defined above.
 * 
 * It should return the joint angles q for the given goal specified in the
 * corresponding parameter.
 * 
 * Only the translation (not rotation) part of the goal has to match.
 * 
 * Hints:
 * - This is an non-linear optimization problem which can be solved by using
 *   an iterative algorithm.
 * - The algorithm should stop when norm of the error is smaller than 1e-3
 * - The algorithm should also stop when 200 iterations are reached
 *
 * Part 2. 
 * ^^^^^^^
 * Refactor the code to expose the functionality as a library with API for
 * a naiive user and visualize the iterations. If you could not complete the 
 * inverse_kinematics() function, use any "dummy" data for visualization. 
 *
 * Part 3. 
 * ^^^^^^^
 * Explain how this code could be used to control the motion of a robot 
 * end-effector in real-time. 
 * 
 * Are there drawbacks to this approach? 
 * 
 * How would you determine if this ik algorithm can be computed in time 
 * within a control loop? Make any assumptions necessary to draw a 
 * conclusion.
 *
 * How would you design an api to expose this algorithm over middleware, 
 * explain any considerations which may be relevant. Imagine users have
 * varying needs, some would like to use the inverse_kinematics() function
 * for offline applications, some would like to use it for real-time 
 * execution in a middleware. 
 * 
 * This Part can be answered in any form: written explanations, code 
 * examples, hand-drawn diagrams, etc. 
 *
 ************************************************/
vector_t inverse_kinematics(const vector_t& q_start, const trafo2d_t& goal ) {
}



/**
 * An example how the inverse kinematics can be used.
 * It should not be required to change this code.
 */
int main(){

    /** initial conditions and desired goal */
    vector_t q_start(3);
    q_start.setConstant(-0.1);

    trafo2d_t goal = trafo2d_t::Identity();
    goal.translation()(0) = 1.;

    /** find the joint angles \a result to reach the end-effector position \a goal */
    vector_t result = inverse_kinematics(q_start,goal);
    std::cout << result << std::endl;

    return 0;
}
