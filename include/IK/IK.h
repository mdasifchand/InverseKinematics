//
// Created by light on 30.01.22.
//

#ifndef _IK_H
#define _IK_H
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <iostream>
#include <pangolin/display/display.h>
#include <pangolin/plot/plotter.h>
#include <thread>
#include <vector>
typedef Eigen::VectorXd vector_t;
typedef Eigen::MatrixXd matrix_t;
typedef Eigen::Transform<double, 2, Eigen::Affine> trafo2d_t;

class IK
{
  public:
    IK(vector_t q_start, trafo2d_t goal) : m_qstart(m_qstart), m_goal(goal)
    {
        std::cout << " Starting the IK solver " << std::endl;
    }

    /**************************************************
     * A function to compute the forward kinematics of
     * a planar 3-link robot.
     *************************************************/
    trafo2d_t forward_kinematics(const vector_t & q);

    /** @fn void computeVectorNorm(const vector_t& q)
     * @brief Function to compute Norm
     *  @param q_current:   input joint angles
     *  @return double
     */
    double computeVectorNorm(const vector_t & q);

    /** @fn void computeJacobi(matrix& J, const vector_t& q_current)
     * @brief A function to compute Jacobian at current joint angles
     *  @param J :          input Jacobi.
     *  @param q_current:   input joint angles
     *  @return : void
     */

    void computeJacobi(matrix_t & J, const vector_t & q_current);

    /** @fn void inverseKinematics(const vector_t& q_start, const trafo2d_t&
     * goal )
     * @brief Non linear least squares optimization IK problem without damping
     * factor
     *  @param q_start :    input start joint angles
     *  @param goal :       input goal @type SE(2)
     *  @return : vector_t
     */

    vector_t inverse_kinematics();
    bool checkWS(const trafo2d_t goal);

  private:
    vector_t m_qstart;
    trafo2d_t m_goal;
};

#endif // IK_H
