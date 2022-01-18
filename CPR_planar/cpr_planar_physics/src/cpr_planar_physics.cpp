/*!
MIT License

Copyright (c) 2022 Gotelli Andrea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/




#include "cpr_planar_physics/cpr_planar_physics.h"



/*
 *
 *
 * guess [n10x, n10y, m10z, n20x, n20y, m20z, ... ... ..., ni0x, ni0y, mi0z, q1, q2, ..., qi]
 *
 *
 *
 */



namespace physics_core {





void RobotPhysics::InitializeRobotModel(const Eigen::Affine2d &distal_plate_pose,
                                        const std::vector<cpr_planar_geometry::Limb> &robot_limbs,
                                        const std::vector<double> &initial_guess,
                                        std::vector<double> &joint_values)
{
  InitilizeVectorsDimensions( static_cast<unsigned int>(robot_limbs.size()) );

  //  Copy the robot limbs into the local instance
  robot_limbs_.reserve(number_of_limbs_);
  for(const auto& limb : robot_limbs)
    robot_limbs_.push_back( limb );

  //  Avoid continuing if there are no limbs
  if(robot_limbs_.empty()){
    ROS_ERROR_STREAM("The robot is initialized without any limbs, exiting...");
    return;
  }

  //  Initialize the distal plate pose
  distal_plate_pose_ = distal_plate_pose;

  //  Define the useful data in the shape message
  shapes_message_.shapes_to_define = number_of_limbs_;


  //  Reserve memory for the guess vector
  guess_.reserve(guess_size_);


  if(!initial_guess.empty()) {
    //  Initialize with the user defined initial guess
    for(const auto& value : initial_guess)
      guess_.push_back(value);

    //  Also include for distal plate position and orientation
    guess_.push_back( distal_plate_pose.translation().x() );
    guess_.push_back( distal_plate_pose.translation().y() );
    guess_.push_back( Eigen::Rotation2D<double>( distal_plate_pose.linear() ).angle() );
  } else GuessNullInit();


  //  Robot initialization is a IGM problem
  problem_type_ = ProblemType::IGM;


  //  Define the problem linking the correct shooting function
  cost_function_ = new ceres::DynamicNumericDiffCostFunction<RobotPhysics, ceres::NumericDiffMethodType::CENTRAL>(this, ceres::Ownership::DO_NOT_TAKE_OWNERSHIP);
  cost_function_->AddParameterBlock( static_cast<int>(guess_size_) );
  cost_function_->SetNumResiduals( static_cast<int>(residual_vector_size_) );

  //  Add the cost function to the problem
  problem_.AddResidualBlock(cost_function_, nullptr, guess_.data());

  //  Set solver options
  options_.max_num_iterations = 100;
  options_.linear_solver_type = ceres::DENSE_QR;
  options_.minimizer_progress_to_stdout = true;

  //  Print initial guess
  LogGuessWithLabel("Initial guess : ");

  //  Try to solve the assembly with the given guess vector
  if(!TryToSolveAssembly())
    return;

  //  Print solver outputs
  std::cout << summary_.FullReport() << "\n";

  //  Print final guess
  LogGuessWithLabel("Finanl guess : ");

  //  Save joints values
  for(unsigned int i=0; i<number_of_limbs_; i++)
    joint_values.push_back( guess_[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i] );

  //  Eliminate verbosity from now on
  options_.minimizer_progress_to_stdout = false;

  //  Publish the visualization message
  PublishRodsVisual();
}

void RobotPhysics::LogGuessWithLabel(const std::string &label) const
{
  std::cout << std::endl;
  //  Print initial guess
  std::cout << label << std::endl;
  for(unsigned int i=0; i<number_of_limbs_; i++) {
    std::cout << "Limb" << std::to_string(i+1) << ": [q, nx, ny, mz] " << std::endl;
    std::cout << "  " << guess_[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i] << ", " << guess_[i*(VECTOR_DIMENSIONS::LIMB_GUESS-1)+0] << ", " << guess_[i*(VECTOR_DIMENSIONS::LIMB_GUESS-1)+1] << ", " << guess_[i*(VECTOR_DIMENSIONS::LIMB_GUESS-1)+2] << std::endl;
  }
  std::cout << std::endl;
}

bool RobotPhysics::TryToSolveAssembly()
{
  //  Use the non linear solver to find a solution
  ceres::Solve(options_, &problem_, &summary_);

  if(summary_.termination_type == ceres::TerminationType::CONVERGENCE)
    return true;

  //  Else, try with another solution
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  ROS_ERROR_STREAM("Impossible to find solution from given intial guess");
  ROS_INFO_STREAM("Initializing the robot with a null initial guess");
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;

  GuessNullInit();

  //  Use the non linear solver to find a solution
  ceres::Solve(options_, &problem_, &summary_);

  if(summary_.termination_type == ceres::TerminationType::CONVERGENCE)
    return true;

  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << "Null initialization is not good either." << std::endl;
  std::cout << "We will try with random joints values for 5 times at the most" << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;

  const unsigned int max_number_of_tentatives = 5;
  for(unsigned int tentative=0; tentative<max_number_of_tentatives; tentative++) {
    GuessNullInit();

    srand( static_cast<unsigned int>(time(nullptr)) );

    std::mt19937_64 rng( std::random_device{}() );
    std::uniform_real_distribution<> dist(-M_PI, M_PI);

    for(unsigned int i=0; i<number_of_limbs_; i++) {
      guess_[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i] = dist(rng);
    }

    //  Use the non linear solver to find a solution
    ceres::Solve(options_, &problem_, &summary_);

    if(summary_.termination_type == ceres::TerminationType::CONVERGENCE)
      return true;

  }

  //  At this point, the only thing is to terminate the execution
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;
  ROS_ERROR_STREAM("Impossible to find solution for this robot, please give a correct initial guess or check the robot model correctness.");
  std::cout << std::endl;
  std::cout << std::endl;
  std::cout << std::endl;

  return false;

}

void RobotPhysics::GuessNullInit() const
{
  guess_.clear();

  for (unsigned int i=0;i<VECTOR_DIMENSIONS::LIMB_GUESS*number_of_limbs_;i++)
    guess_.push_back(0);

  //  Also include for distal plate position and orientation
  guess_.push_back( distal_plate_pose_.translation().x() );
  guess_.push_back( distal_plate_pose_.translation().y() );
  guess_.push_back( Eigen::Rotation2D<double>( distal_plate_pose_.linear() ).angle() );

}


bool RobotPhysics::operator()(double const* const* guess, double* residual) const
{
  //  Initialize the guess vector from the current vector
  Eigen::VectorXd current_guess(guess_size_);
  for (unsigned int i=0;i<guess_size_;i++) {
    current_guess[i] = guess[0][i];
  }

  //  Obtain current distal plate pose from guess vector
  Eigen::Affine2d current_distal_plate_pose = Eigen::Affine2d::Identity();
  current_distal_plate_pose.translation() = current_guess.segment(current_guess.size()-VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS, 2);
  current_distal_plate_pose.linear() = Eigen::Rotation2D<double>( current_guess[current_guess.size()-1]).toRotationMatrix();

  //  Define a vector to contain the result of a limb numerical integration
  Eigen::VectorXd integration_result;
  //  result << position_error, angle_error, joint_constraint;
  //                0     1         2               3

  //  Numerically integrate each limb
  for(unsigned int i=0;i<number_of_limbs_;i++) {

    //  Define the guess vector relative to the limb
    double joint_value = current_guess[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i];
    Eigen::Vector3d guessed_wrench = current_guess.segment(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1), (VECTOR_DIMENSIONS::LIMB_GUESS-1));
    Eigen::Vector4d limb_guess_vector;
    limb_guess_vector << joint_value, guessed_wrench;

    //  Evaluate the Boundary Conditions given the current guess
    integration_result = robot_limbs_[i].EvaluateBoundaryConditions( current_distal_plate_pose, limb_guess_vector );

    //  Store geometrical residual errors
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+0] = integration_result[0];
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+1] = integration_result[1];
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+2] = integration_result[2];
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+3] = integration_result[3];
  }


  //  Intitialization for the wrench balance
  Eigen::Vector3d wrench_balance(wrench_.fx, wrench_.fy, wrench_.mz);

  //  Sum the contribute of each limb
  for(const auto& limb : robot_limbs_)
    wrench_balance += limb.GetAppliedWrench();

  //  Add the wrench balance in evaluation
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+0] = wrench_balance[0];
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+1] = wrench_balance[1];
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+2] = wrench_balance[2];

      const Eigen::Vector2d dp_position_error = distal_plate_pose_.translation() - current_distal_plate_pose.translation();
      const double dp_angle_error = Eigen::Rotation2D<double>( distal_plate_pose_.linear() ).angle() -
                                    Eigen::Rotation2D<double>( current_distal_plate_pose.linear() ).angle();

        residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS+3+0] = dp_position_error.x();
        residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS+3+1] = dp_position_error.y();
        residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS+3+2] = dp_angle_error;


  if(problem_type_ == ProblemType::DGM){
    //  Define error btw guessed and real joint values (in order to keep given joint values)
    for (unsigned int i=0; i<number_of_limbs_;i++)
      residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + VECTOR_DIMENSIONS::EQUILIBRIUM_SECTION+i] = joints_value_[i] - current_guess[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1)+i];

    //  Fill up the missing components with zeroes
    if(number_of_limbs_ < VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS) {
      for(unsigned int i=0; i<VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS-number_of_limbs_;i++)
        residual[(residual_vector_size_-1)-i] = 0.0;
    }

    distal_plate_pose_ = current_distal_plate_pose;
  }

  if(problem_type_ == ProblemType::IGM){

    //  Define error in position btw guessed and real distal plate (in order to keep same position)
    const Eigen::Vector2d dp_position_error = distal_plate_pose_.translation() - current_distal_plate_pose.translation();
    //  Define error in the angle btw guessed and real distal plate (in order to keep same orientation)
    const double dp_angle_error = Eigen::Rotation2D<double>( distal_plate_pose_.linear() ).angle()        -
                                  Eigen::Rotation2D<double>( current_distal_plate_pose.linear() ).angle() ;

    //  Copy values in the appropriate part of the residual vector
    residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS+VECTOR_DIMENSIONS::EQUILIBRIUM_SECTION+0] = dp_position_error.x();
    residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS+VECTOR_DIMENSIONS::EQUILIBRIUM_SECTION+1] = dp_position_error.y();

    //  Depending on the number of limbs, we also have the angle
    if(number_of_limbs_ < VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS) {
      //  If there are less then 3 limbs, we lose the control on theta
      residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS+VECTOR_DIMENSIONS::EQUILIBRIUM_SECTION+2] = 0.0;
    } else
      residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS+VECTOR_DIMENSIONS::EQUILIBRIUM_SECTION+2] = dp_angle_error;

  }

  return true;
}



void RobotPhysics::SolveIGM(const Eigen::Affine2d &_distal_plate_pose, std::vector<double> &_joint_values, const Wrench &wrench)
{
  problem_type_ = ProblemType::IGM;

  wrench_ = wrench;

  //  Update current distal plate position
  distal_plate_pose_ = _distal_plate_pose;

  //  Set the corresponding values in the guess vector for consistence
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 0] = distal_plate_pose_.translation().x();
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 1] = distal_plate_pose_.translation().y();
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 2] = Eigen::Rotation2D<double>( distal_plate_pose_.linear() ).angle();

  //  Use the non linear solver to find a solution
  ceres::Solve(options_, &problem_, &summary_);
  CheckForSolverIssue();

  _joint_values.clear();
  //  Save joints values
  for(const auto& limb : robot_limbs_)
    _joint_values.push_back( limb.GetJointValue() );

  //  Publish the visualization message
  PublishRodsVisual();
}

void RobotPhysics::SolveDGM(Eigen::Affine2d &_distal_plate_pose, const std::vector<double> &_joint_values, const Wrench &wrench)
{
  problem_type_ = ProblemType::DGM;

  wrench_ = wrench;

  //  Save current joint values
  joints_value_ = _joint_values;

  //  Set the corresponding values in the guess vector for consistence
  for(unsigned int i=0; i<number_of_limbs_; i++)
    guess_[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i] = _joint_values[i];

  //  Use the non linear solver to find a solution
  ceres::Solve(options_, &problem_, &summary_);
  CheckForSolverIssue();

  _distal_plate_pose = distal_plate_pose_;

  //  Publish the visualization message
  PublishRodsVisual();
}

void RobotPhysics::CheckForSolverIssue() const
{
  if(summary_.termination_type == ceres::TerminationType::NO_CONVERGENCE) {
    ROS_ERROR_STREAM(summary_.message);
    return;
  }

  if(summary_.termination_type == ceres::TerminationType::FAILURE) {
    ROS_ERROR_STREAM(summary_.message);
    return;
  }

  if(summary_.termination_type == ceres::TerminationType::CONVERGENCE) {
    if(summary_.final_cost >= 1e-3) {
      std::cout << std::endl;
      ROS_WARN_STREAM("Careful, solution is not precise.");
      ROS_WARN_STREAM(summary_.BriefReport());
      ROS_WARN_STREAM(summary_.message);
      std::cout << std::endl;
    }
  }
}


void RobotPhysics::PublishRodsVisual()
{
  for(unsigned int i=0; i<number_of_limbs_; i++) {
    //  Unpack guess vector
    Eigen::Vector2d n0(guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+0], guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+1]);
    double m0 = guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+2];

    //  Fill the current shape message
    shapes_message_.rod_centerlines[i] = robot_limbs_[i].GetRodVisual(n0, m0);

  }

  //  Publish the visual message
  shapes_publisher_.publish(shapes_message_);
}

bool RobotPhysics::SetGuessVector(std::vector<double> given_guess,
                                  Eigen::Affine2d &distal_plate_pose,
                                  std::vector<float> &joint_values)
{
  if(given_guess.size() != guess_size_) {
    ROS_ERROR_STREAM("The given guess error has a size different from the original one. The guess vector remains unchanged");
    return false;
  }

  guess_ = given_guess;

  std::vector<double> vec(guess_.begin() + number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS, guess_.end());

  Eigen::Map<Eigen::VectorXd> distal_plate_section(vec.data(), 3); // x, y, theta

  distal_plate_pose_.translation() << distal_plate_section.x(),
                                      distal_plate_section.y();
  distal_plate_pose.linear() << Eigen::Rotation2D<double>(distal_plate_section[2]).toRotationMatrix();

  std::cout << "The distal plate has been set to :" << std::endl << distal_plate_pose_.matrix() << std::endl;

  //  Enable output
  options_.minimizer_progress_to_stdout = true;

  //  Print initial guess
  LogGuessWithLabel("Given guess : ");

  //  Solve the problem
  ceres::Solve(options_, &problem_, &summary_);

  //  Print final guess
  LogGuessWithLabel("Final guess : ");

  //  Eliminate verbosity from now on
  options_.minimizer_progress_to_stdout = false;

  if(summary_.termination_type == ceres::TerminationType::FAILURE) {
    ROS_ERROR_STREAM("The solver has failed with the following message :");
    ROS_ERROR_STREAM(summary_.message);
    return false;
  }

  distal_plate_pose = distal_plate_pose_;

  joint_values.resize(number_of_limbs_);
  for(unsigned int i=0; i<number_of_limbs_; i++)
    joint_values[i] = static_cast<float>( guess_[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i] );

  PublishRodsVisual();

  return true;
}




} //  END namespace physics_core
