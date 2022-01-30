
 - [x] TODO: Complete IK problem; use simple NL with Eigen and then probably improve with LM with a damping factor
 - [x] TODO: Jacobi is rectangular => redundant system  => pseudo inverse 
 - [x] TODO: limiting condition 1e-3 && it < 100 && someDisplayFlag - tag 0.0.1 (kind of works!)
 - [ ] TODO: Refactor the code, probably include logger (GUI) as a friend function or a singleton
 - [ ] TODO: Explain the realtimeness of the solution
 - [ ] TODO:   How would you determine if this ik algorithm can be computed in time 
      - Determine joint velocity differential equation then equate to J(q)*\q
      - Probably solving it over the contraint of time using LM or some non linear solvers
 - [ ] within a control loop? Make any assumptions necessary to draw a conclusion.
      - We can estimate states with the error propagation
      - Then get a better estimate at every iteration or time lapse 
 - [ ] TODO: Encapsulate in an object with RAII and expose important parts of it
      - Not fully done  
 - [ ] TODO: Ex: RTI DDS Connext => IDL generator to generate necessary message type => may be microRTIDDS for small hardware 
 - [ ] TODO: More optimization and cost effectiveness => ARM with it's optimizations 
 - [ ] TODO: UML class diagram for ROS2 API with the current implementation


## Project Requirements

- Eigen
- Pangolin
- CERES (not used so far)
- C++14





