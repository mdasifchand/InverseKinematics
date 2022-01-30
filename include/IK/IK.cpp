//
// Created by light on 30.01.22.
//

#include "IK.h"

trafo2d_t IK::forward_kinematics(const vector_t& q ){
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


double IK::computeVectorNorm(const vector_t& q){
    return sqrt( pow(q(0),2) + pow(q(1),2) + pow(q(2),2));
}


void  IK::computeJacobi(matrix_t& J, const vector_t& q_current ){

    assert(J.rows() == 2);
    assert(J.cols() == 3);

    J(0,0) = cos(q_current(0)) + cos(q_current(0) + q_current(1)) + cos(q_current(0) + q_current(1) + q_current(2));
    J(0,1) =   cos(q_current(0) + q_current(1)) + cos(q_current(0) + q_current(1) + q_current(2));
    J(0,2) =  cos(q_current(0) + q_current(1) + q_current(2));
    J(1,0) = -sin(q_current(0)) - sin(q_current(0) + q_current(1)) - sin(q_current(0) + q_current(1) + q_current(2));
    J(1,1) = sin(q_current(0) + q_current(1)) - sin(q_current(0) + q_current(1) + q_current(2));
    J(1,2) = sin(q_current(0) + q_current(1) + q_current(2));

}

vector_t IK::inverse_kinematics(){
    vector_t q_current = m_qstart;
    vector_t diff_theta(3);
    vector_t diff_pos(2);
    diff_theta = m_qstart;
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

    while(IK::computeVectorNorm(diff_theta) > 1e-3 && !pangolin::ShouldQuit() && it < 100)
    {
        computeJacobi(J, q_current);
        // compute pseudo inverse, this is a redundant system (Jacobi is rectangular)
        pseudo_inv =  J.transpose()*((J*J.transpose()).inverse());
        diff_pos = m_goal.translation() - forward_kinematics(q_current).translation();
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
// Checks if the value lies in the WS1
bool checkWS(const trafo2d_t goal) {
    //TODO
    return true;
}
