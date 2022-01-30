
## Problem statement:

Inverse kinematics of 3R planar robot, determine the joint configuration given the position of end effector. 


 - [x] TODO: Complete IK problem; use Non Linear Least Squares approach and then probably improve with Lavenberg Makquardt with a damping factor later
 - [x] TODO: Jacobi is rectangular => redundant system  => pseudo inverse 
 - [x] TODO: limiting condition 1e-3 && it < 100 && someDisplayFlag - tag 0.0.1 (kind of works!)
 - [x] TODO: Refactor the code, probably include logger (GUI) as a friend function or a singleton
 - [x] TODO: Explain the realtimeness of the solution
 - [ ] TODO:   How would you determine if this ik algorithm can be computed in time 
      - Determine joint velocity differential equation then equate to J(q)*\q
      - Probably solving it over the contraint of time using LM or some non linear solvers or Langrange multipliers etc......
 - [ ] within a control loop? Make any assumptions necessary to draw a conclusion.
      - We can estimate states with the error propagation
      - Then get a better estimate at every iteration or time lapse 
 - [x] TODO: Encapsulate in an object with RAII and expose important parts of it
      - Not fully done  
 - [ ] TODO: Ex: RTI DDS Connext => IDL generator to generate necessary message type => may be microRTIDDS for small hardware 
 - [ ] TODO: More optimization and cost effectiveness
 - [ ] TODO: UML class diagram for ROS2 API with the current implementation
        - friend singleton logger for the GUI
        - trigger a shared_ptr of the object (assuming if it's a closed loop with state estimation)
        - assuming frequency of all the sensors is the same we can fill the queue over some time space
        - Perform necessary opeartion on the callback function


## Project Requirements

- Eigen
- Pangolin
- CERES (not used so far)
- C++14

## Convergence
- Converges in a matter of 120 iterations (Could be a local minima?)

## Images
 ![convergence](https://github.com/mdasifchand/InverseKinematics/blob/master/images/Screenshot%20from%202022-01-30%2022-08-44.png)
 ![iteration](https://github.com/mdasifchand/InverseKinematics/blob/master/images/Screenshot%20from%202022-01-30%2022-09-05.png)

## Important References: 

https://robotacademy.net.au/lesson/velocity-of-3-joint-planar-robot-arm/ -> b/w T and final configuration of EF
https://inst.eecs.berkeley.edu/~cs184/fa09/resources/ik.pdf
https://motion.cs.illinois.edu/RoboticSystems/InverseKinematics.html
https://github.com/ceres-solver/ceres-solver/blob/master/include/ceres/covariance.h -> relates to the estimation of deficiency
http://robots.iit.edu/uploads/2/5/7/1/25715664/mmae_540_-_lecture_2_-_manipulator_kinematics.pdf
https://www.cs.cmu.edu/~15464-s13/lectures/lecture6/iksurvey.pdf -> Damped least squares
https://mathweb.ucsd.edu/~sbuss/ResearchWeb/ikmethods/SdlsPaper.pdf -> Damping Least Squares factor



