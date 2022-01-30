#include <iostream>
#include <chrono>
#include <thread>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>
#include <pangolin/display/display.h>
#include <pangolin/plot/plotter.h>
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
    link_offset.translation().x() = 1.;

    // define the start pose
    trafo2d_t trafo = trafo2d_t::Identity();

    for(int joint_idx = 0; joint_idx < 3 ; joint_idx++ ) {
        // add the rotation contributed by this joint
        trafo *= Eigen::Rotation2D<double>(q(joint_idx));
        // add the link offset to this position
        trafo = trafo * link_offset; // T_end = R(q(1)) * Tx(q(1)) * R((q(2)) * Tx(q(2))  ......  R(q(n))* Tx((q(n)))
    }                                //  T_end -> should give use the joint coordinates x, y, phi
                                    // q(1) , q(2) and q(3) are basically joint angles
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


enum class METHOD { NEWTONR, LAVENBERGM, ANALYTICAL };



/** @fn void computeVectorNorm(const vector_t& q)
 * @brief Function to compute Norm
*  @param q_current:   input joint angles
*  @return double
*/
 double computeVectorNorm(const vector_t& q){
    return sqrt( pow(q(0),2) + pow(q(1),2) + pow(q(2),2));
}




/** @fn void computeJacobi(matrix& J, const vector_t& q_current)
 * @brief A function to compute Jacobian at current joint angles
*  @param J :          input Jacobi.
*  @param q_current:   input joint angles
*  @return : void
*/

void  computeJacobi(matrix_t& J, const vector_t& q_current ){

    assert(J.rows() == 2);
    assert(J.cols() == 3);;
    J(0,0) = cos(q_current(0)) + cos(q_current(0) + q_current(1)) + cos(q_current(0) + q_current(1) + q_current(2));
    J(0,1) =   cos(q_current(0) + q_current(1)) + cos(q_current(0) + q_current(1) + q_current(2));
    J(0,2) =  cos(q_current(0) + q_current(1) + q_current(2));
    J(1,0) = -sin(q_current(0)) - sin(q_current(0) + q_current(1)) - sin(q_current(0) + q_current(1) + q_current(2));
    J(1,1) = sin(q_current(0) + q_current(1)) - sin(q_current(0) + q_current(1) + q_current(2));
    J(1,2) = sin(q_current(0) + q_current(1) + q_current(2));

}


// TODO: include damping factor (Lavenberg Macquardt) 
/** @fn void inverseKinematics(const vector_t& q_start, const trafo2d_t& goal )
 * @brief Non linear least squares optimization IK problem without damping factor
*  @param q_start :    input start joint angles 
*  @param goal :       input goal @type SE(2)
*  @return : vector_t
*/

vector_t inverse_kinematics(const vector_t& q_start, const trafo2d_t& goal ) {
    vector_t q_current = q_start;
    vector_t diff_theta(3);
    vector_t diff_pos(2);
    diff_theta = q_start;
    matrix_t  J(2,3);
    matrix_t pseudo_inv(3,2);
    trafo2d_t t;
    int it =0;


    pangolin::CreateWindowAndBind("Main", 720, 640);
    pangolin::DataLog logger;
    std::vector<std::string> legend;
    legend.emplace_back(std::string("norm"));
    legend.emplace_back(std::string("iteration"));
    logger.SetLabels(legend);
    pangolin::Plotter plotter( &logger, 0.0f, 10.0f,0.0f, 10.0f  );
    plotter.SetBounds(0.0,1.0, 0.0,1.0);
    plotter.Track("$i");
    plotter.AddMarker(pangolin::Marker::Vertical,   -1000, pangolin::Marker::LessThan, pangolin::Colour::Blue().WithAlpha(0.2f) );
    //plotter.AddMarker(pangolin::Marker::Horizontal,   100, pangolin::Marker::GreaterThan, pangolin::Colour::Red().WithAlpha(0.2f) );
    pangolin::DisplayBase().AddDisplay(plotter);

    while(computeVectorNorm(diff_theta) > 1e-3 && !pangolin::ShouldQuit() && it < 100)
    {
        computeJacobi(J, q_current);
        // compute pseudo inverse, this is a redundant system (Jacobi is rectangular)
        pseudo_inv =  J.transpose()*((J*J.transpose()).inverse());
        diff_pos = goal.translation() - forward_kinematics(q_current).translation();
        diff_theta = pseudo_inv* diff_pos;
        q_current = q_current + diff_theta;
        it++;
        std::cout << " === The norm is " << computeVectorNorm(diff_theta) << " ==== iteration is " << it << std::endl;
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        logger.Log(computeVectorNorm(diff_theta));
        // Render graph, Swap frames and Process Events
        pangolin::FinishFrame();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
   }
    std::cout << q_current << "Value" << std::endl;
    return q_current;

}


// TODO:Check if the angle lies in workspace of the given configuration
bool checkWS(const trafo2d_t goal)
{
    // goal has to be definite


}

/**
 * An example how the inverse kinematics can be used.
 * It should not be required to change this code.
 */
int main(){

    // initial conditions and desired goal
    vector_t q_start(3);
    q_start << -0.1, -0.1, -0.1 ;
    std::cout << q_start << std::endl;

    trafo2d_t goal = trafo2d_t::Identity();
    goal.translation()(0) = 1.;
    vector_t q_val(3);
    q_val = inverse_kinematics(q_start, goal);



    return 0;
}
