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
 * 
 * TODO: Complete IK problem; use simple NL with Eigen and then probably improve with LM with a damping factor
 * TODO: Jacobi is rectangular => redundant system  => pseudo inverse 
 * TODO: limiting condition 1e-3 && it < 100 && someDisplayFlag
 * TODO: Refactor the code, probably include logger (GUI) as a friend function or a singleton
 * TODO: Explain the realtimeness of the solution
 * TODO:   How would you determine if this ik algorithm can be computed in time 
 * within a control loop? Make any assumptions necessary to draw a 
 * conclusion.
 * TODO: Encapsulate in an object with RAII and expose important parts of it 
 * TODO: Ex: RTI DDS Connext => IDL generator to generate necessary message type => may be microRTIDDS for small hardware 
 * TODO: More optimization and cost effectiveness => ARM with it's optimizations 
 * TODO: UML class diagram for ROS2 API with the current implementation
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
