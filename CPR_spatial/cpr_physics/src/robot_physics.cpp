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




#include "cpr_physics/cpr_physics.h"



/*
 *
 *
 * guess [n10x, n10y, n10z, m10x, m10y, m10z, ... ... ..., ni0x, ni0y, ni0z, mi0x, mi0y, mi0z, q1, ..., qi]
 *
 *
 *
 */



namespace physics_core {


const Eigen::Affine3d RobotPhysics::ToAffine3D(const Eigen::VectorXd &dp_guess_section) const
{
  Eigen::Affine3d distal_plate_pose = Eigen::Affine3d::Identity();

  //  Copy the translational part
  distal_plate_pose.translation().x() = dp_guess_section[0];
  distal_plate_pose.translation().y() = dp_guess_section[1];
  distal_plate_pose.translation().z() = dp_guess_section[2];

  //  Obtain Euler Angles
  Eigen::Vector3d distal_plate_euler_angles;
  distal_plate_euler_angles.x() = dp_guess_section[3];
  distal_plate_euler_angles.y() = dp_guess_section[4];
  distal_plate_euler_angles.z() = dp_guess_section[5];

  //  Convert Euler Angles in Rotation matrix
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(distal_plate_euler_angles.x(), Eigen::Vector3d::UnitX())
      *Eigen::AngleAxisd(distal_plate_euler_angles.y(), Eigen::Vector3d::UnitY())
      *Eigen::AngleAxisd(distal_plate_euler_angles.z(), Eigen::Vector3d::UnitZ());

  //  Copy to the rotation part of the transformation
  distal_plate_pose.linear() = R;

  return distal_plate_pose;
}


/* How to compute the Jacobian
 *
 * Having the guess vector as
 *
 * |  n01_x |
 * |  n01_y |
 * |  n01_z |
 * |  m01_x |
 * |  m01_y |
 * |  m01_z |
 * |    :   |
 * |    :   |
 * |    :   |
 * |  n0n_x |
 * |  n0n_y |
 * |  n0n_z |
 * |  m0n_x |
 * |  m0n_y |
 * |  m0n_z |
 * |   q1   |
 * |    :   |
 * |   qn   |
 * |  dp_x  |
 * |  dp_y  |
 * |  dp_z  |
 * |  dp_ψ  |
 * |  dp_φ  |
 * |  dp_θ  |
 *
 *
 * And the residual vector as
 *        |  e1_x  |
 *        |  e1_y  |
 *        |  e1_z  |
 *        |  e1_ψ  |
 *        |  e1_φ  |
 *        |  e1_θ  |
 *        |  j1c_1 |
 *        |  j1c_2 |
 *        |  j1c_3 |
 *        |    :   |
 *        |    :   |
 *        |    :   |
 *        |  en_x  |
 *        |  en_y  |
 *        |  en_z  |
 *        |  en_ψ  |
 *        |  en_φ  |
 *        |  en_θ  |
 *        |  jnc_1 |
 *        |  jnc_2 |
 *        |  jnc_3 |
 *        |  Σfx   |
 *        |  Σfy   |
 *        |  Σfz   |
 *        |  Σmx   |
 *        |  Σmy   |
 *        |  Σmz   |
 * |  dp_x  |    |  e_jv1 |
 * |  dp_y  |    |  e_jv2 |
 * |  dp_z  |    |    :   |
 * |  dp_ψ  |    |    :   |
 * |  dp_φ  |    |    :   |
 * |  dp_θ  |    |  e_jvn |
 *    IGM            DGM
 */

bool RobotPhysics::operator()(double const* const* guess, double* residual) const
{
  //  Initialize the guess vector from the current vector
  Eigen::VectorXd current_guess(guess_size_);
  for (unsigned int i=0;i<guess_size_;i++) {
    current_guess[i] = guess[0][i];
  }

  //  Obtain current distal plate pose from guess vector
  Eigen::Affine3d current_distal_plate_pose = ToAffine3D( current_guess.segment( number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS, 6) );


  //  Define a vector to contain the result of constrains evaluation
  Eigen::VectorXd limb_residual(static_cast<int>(VECTOR_DIMENSIONS::LIMB_RESIDUAL));

  //  Numerically integrate each limb
  for(unsigned int i=0;i<number_of_limbs_;i++) {

    //  Define the guess vector relative to the limb
    double joint_value = current_guess[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i];

    Eigen::VectorXd guessed_wrench(6);
    guessed_wrench << current_guess.segment(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1), (VECTOR_DIMENSIONS::LIMB_GUESS-1));

    Eigen::VectorXd limb_guess_vector(static_cast<int>(VECTOR_DIMENSIONS::LIMB_GUESS));
    limb_guess_vector << joint_value, guessed_wrench;

    //  Evaluate the Boundary Conditions given the current guess
    limb_residual = robot_limbs_[i].EvaluateBoundaryConditions( current_distal_plate_pose, limb_guess_vector );

    //  Store geometrical residual errors
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+0] = limb_residual[0];  //  ex
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+1] = limb_residual[1];  //  ey
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+2] = limb_residual[2];  //  ez
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+3] = limb_residual[3];  //  eψ
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+4] = limb_residual[4];  //  eφ
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+5] = limb_residual[5];  //  eθ
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+6] = limb_residual[6];  //  jc1
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+7] = limb_residual[7];  //  jc2
    residual[(i*VECTOR_DIMENSIONS::LIMB_RESIDUAL)+8] = limb_residual[8];  //  jc3

  }


  //  Intitialization for the wrench balance
  Eigen::VectorXd wrench_balance(6);
  wrench_balance << initial_wrench_.force, initial_wrench_.torque;
  wrench_balance[0] += wrench_.force[0];
  wrench_balance[1] += wrench_.force[1];
  wrench_balance[2] += wrench_.force[2];
  wrench_balance[3] += wrench_.torque[0];
  wrench_balance[4] += wrench_.torque[1];
  wrench_balance[5] += wrench_.torque[2];

  //  Sum the contribute of each limb
  for(const auto& limb : robot_limbs_)
    wrench_balance += limb.GetAppliedWrench();

  //  Add the wrench balance in evaluation
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+0] = wrench_balance[0];  //  Σfx
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+1] = wrench_balance[1];  //  Σfy
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+2] = wrench_balance[2];  //  Σfz
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+3] = wrench_balance[3];  //  Σmx
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+4] = wrench_balance[4];  //  Σmy
  residual[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL+5] = wrench_balance[5];  //  Σmz


  const unsigned int index = number_of_limbs_*VECTOR_DIMENSIONS::LIMB_RESIDUAL + VECTOR_DIMENSIONS::EQUILIBRIUM_SECTION;


  if(problem_type_ == ProblemType::DGM){
    //  Define error btw guessed and real joint values (in order to keep given joint values)

    for (unsigned int i=0; i<number_of_limbs_;i++)
      residual[index + i] = static_cast<double>(joints_value_[i]) - current_guess[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i];

    //  Fill up the missing components with zeroes
    if(number_of_limbs_ < VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS) {
      for(unsigned int i=0; i<VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS-number_of_limbs_;i++)
        residual[(residual_vector_size_-1)-i] = 0.0;
    }

    distal_plate_pose_ = current_distal_plate_pose;
  }

  if(problem_type_ == ProblemType::IGM){

    //  Define error in position btw guessed and real distal plate (in order to keep same position)
    const Eigen::Vector3d dp_position_error = distal_plate_pose_.translation() - current_distal_plate_pose.translation();

    //  Define error in the angle btw guessed and real distal plate (in order to keep same orientation)
    const Eigen::Vector3d orientation_error = cpr_geometry::inv_hat(distal_plate_pose_.linear().transpose()*current_distal_plate_pose.linear() -
                                                                      distal_plate_pose_.linear()*current_distal_plate_pose.linear().transpose() );

    Eigen::VectorXd geometrical_errors(6);
    geometrical_errors << dp_position_error, orientation_error;


    Eigen::VectorXd corrected_geometrical_errors = underactuation_correction_*geometrical_errors;

    residual[index + 0] = corrected_geometrical_errors[0];
    residual[index + 1] = corrected_geometrical_errors[1];
    residual[index + 2] = corrected_geometrical_errors[2];
    residual[index + 3] = corrected_geometrical_errors[3];
    residual[index + 4] = corrected_geometrical_errors[4];
    residual[index + 5] = corrected_geometrical_errors[5];

  }

  return true;
}





void RobotPhysics::InitializeRobotModel(Eigen::Affine3d &distal_plate_pose,
                                        const std::vector<cpr_geometry::Limb> &robot_limbs,
                                        const std::vector<double> &initial_guess,
                                        std::vector<float> &joint_values,
                                        const Eigen::Matrix<double, 6, 6> &underactuation_correction,
                                        const Wrench &initial_wrench)
{
  InitilizeVectorsDimensions( static_cast<unsigned int>(robot_limbs.size()) );

  initial_wrench_ = initial_wrench;

  underactuation_correction_ = underactuation_correction;

  std::cout << "Underactuation correction matrix : " << std::endl;
  std::cout << underactuation_correction_ << std::endl << std::endl << std::endl;

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
    GuessInitDistalPlateSection();
  } else GuessNullInit();



  //  Robot initialization is a IGM problem
  problem_type_ = ProblemType::IGM;


  //  Define the problem linking the correct shooting function
  cost_function_ = new ceres::DynamicNumericDiffCostFunction<RobotPhysics, ceres::NumericDiffMethodType::CENTRAL>(this, ceres::Ownership::DO_NOT_TAKE_OWNERSHIP);
  cost_function_->AddParameterBlock( static_cast<int>(guess_size_) );
  cost_function_->SetNumResiduals( static_cast<int>(residual_vector_size_) );

//  //  Add the cost function to the problem
  problem_id = problem_.AddResidualBlock(cost_function_, nullptr, guess_.data());


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
  LogGuessWithLabel("Final guess : ");


  //  Save joints values
  for(unsigned int i=0; i<number_of_limbs_; i++)
    joint_values.push_back( static_cast<float>(guess_[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i]) );

  //  Save changes in distal plate pose
  distal_plate_pose = DistalPlatePoseFromParametersVector();

  //  Eliminate verbosity from now on
  options_.minimizer_progress_to_stdout = false;

  //  Publish the visualization message
  PublishRodsVisual();
}




void RobotPhysics::SolveIGM(const Eigen::Affine3d &_distal_plate_pose, std::vector<float> &_joint_values, const Wrench &wrench)
{
  problem_type_ = ProblemType::IGM;

  wrench_ = wrench;

  //  Update current distal plate position
  distal_plate_pose_ = _distal_plate_pose;

  //  Set the corresponding values in the guess vector for consistence
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 0] = distal_plate_pose_.translation().x();
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 1] = distal_plate_pose_.translation().y();
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 2] = distal_plate_pose_.translation().z();
  Eigen::Vector3d euler_angles = distal_plate_pose_.linear().eulerAngles(0, 1, 2);
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 3] = euler_angles[0];
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 4] = euler_angles[1];
  guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 5] = euler_angles[2];

  //  Use the non linear solver to find a solution
  ceres::Solve(options_, &problem_, &summary_);
  CheckForSolverIssue();

  _joint_values.clear();
  //  Save joints values
  for(const auto& limb : robot_limbs_)
    _joint_values.push_back( static_cast<float>(limb.GetJointValue()) );

  //  Publish the visualization message
  PublishRodsVisual();
}





void RobotPhysics::SolveDGM(Eigen::Affine3d &_distal_plate_pose, const std::vector<float> &_joint_values, const Wrench &wrench)
{
  problem_type_ = ProblemType::DGM;

  wrench_ = wrench;

  //  Save current joint values
  joints_value_ = _joint_values;

  //  Set the corresponding values in the guess vector for consistence
  for(unsigned int i=0; i<number_of_limbs_; i++)
    guess_[number_of_limbs_*(VECTOR_DIMENSIONS::LIMB_GUESS-1) + i] = static_cast<double>(_joint_values[i]);

  //  Use the non linear solver to find a solution
  ceres::Solve(options_, &problem_, &summary_);
  CheckForSolverIssue();

  //  Copy the solution
  _distal_plate_pose = distal_plate_pose_;

  //  Publish the visualization message
  PublishRodsVisual();
}




void SaveRodsShapesInYaml(const cpr_visual::ShapeMsg &shapes_message_)
{
  YAML::Emitter emitter;
  emitter << YAML::BeginMap;
  //  First decompose the shape message in some vectors of double
  for(unsigned int i=0; i<shapes_message_.shapes_to_define; i++) {
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    for(const auto& point : shapes_message_.rod_centerlines[i].points) {
      x.push_back( point.x );
      y.push_back( point.y );
      z.push_back( point.z );
    }
    emitter << YAML::Key << "limb" + std::to_string(i+1) + "_shape";
    emitter << YAML::Value << YAML::BeginMap;
      emitter << YAML::Key << "x" << YAML::Value << YAML::Flow << x;
      emitter << YAML::Key << "y" << YAML::Value << YAML::Flow << y;
      emitter << YAML::Key << "z" << YAML::Value << YAML::Flow << z;
    emitter << YAML::EndMap;

  }
  emitter << YAML::EndMap;

  std::ofstream fout("/home/andrea/QtCreator/latest/catkin/src/A-Gazebo-Simulator-for-Continuum-Parallel-Robots/robot_physics/launch/Stewart_Gough_6FFR_Till/rods_shapes.yaml");
  fout << emitter.c_str();

}




void RobotPhysics::PublishRodsVisual()
{
  for(unsigned int i=0; i<number_of_limbs_; i++) {
    //  Unpack guess vector
    Eigen::Vector3d n0(guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+0],
                       guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+1],
                       guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+2]);
    Eigen::Vector3d m0(guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+3],
                       guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+4],
                       guess_[(i*(VECTOR_DIMENSIONS::LIMB_GUESS-1))+5]);

    //  Fill the current shape message
    shapes_message_.rod_centerlines[i] = robot_limbs_[i].GetRodVisual(n0, m0);
  }

  //SaveRodsShapesInYaml(shapes_message_);

  //  Publish the visual message
  shapes_publisher_.publish(shapes_message_);
}


Eigen::Affine3d RobotPhysics::DistalPlatePoseFromParametersVector() const
{
  Eigen::Affine3d distal_plate_pose = Eigen::Affine3d::Identity();
  distal_plate_pose.translation().x() = guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 0];
  distal_plate_pose.translation().y() = guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 1];
  distal_plate_pose.translation().z() = guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 2];

  Eigen::Vector3d distal_plate_euler_angles;
  distal_plate_euler_angles.x() = guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 3];
  distal_plate_euler_angles.y() = guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 4];
  distal_plate_euler_angles.z() = guess_[number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS + 5];

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(distal_plate_euler_angles.x(), Eigen::Vector3d::UnitX())
      *Eigen::AngleAxisd(distal_plate_euler_angles.y(), Eigen::Vector3d::UnitY())
      *Eigen::AngleAxisd(distal_plate_euler_angles.z(), Eigen::Vector3d::UnitZ());
  distal_plate_pose.linear() = R;

  return distal_plate_pose;
}


void RobotPhysics::LogGuessWithLabel(const std::string &label) const
{
  const unsigned int limb_index = VECTOR_DIMENSIONS::LIMB_GUESS-1;
  std::cout << std::endl;
  //  Print initial guess
  std::cout << label << std::endl;
  for(unsigned int i=0; i<number_of_limbs_; i++) {
    std::cout << "Limb" << std::to_string(i+1) << ": [q, nx, ny, nz, mx, my, mz] " << std::endl;
    std::cout << "  " << guess_[number_of_limbs_*(limb_index) + i] << ", ";
    std::cout << guess_[i*(limb_index)+0] << ", " << guess_[i*(limb_index)+1] << ", " << guess_[i*(limb_index)+2] << ", ";
    std::cout << guess_[i*(limb_index)+3] << ", " << guess_[i*(limb_index)+4] << ", " << guess_[i*(limb_index)+5] << std::endl;
  }

  Eigen::Affine3d current_distal_plate_pose = DistalPlatePoseFromParametersVector();

  std::cout << "Distal plate pose : " << std::endl << current_distal_plate_pose.matrix() << std::endl;
  std::cout << std::endl;
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


bool RobotPhysics::SetGuessVector(std::vector<double> given_guess,
                                  Eigen::Affine3d &distal_plate_pose,
                                  std::vector<float> &joint_values)
{
  if(given_guess.size() != guess_size_) {
    ROS_ERROR_STREAM("The given guess error has a size different from the original one. The guess vector remains unchanged");
    return false;
  }

  guess_ = given_guess;

  std::vector<double> vec(guess_.begin() + number_of_limbs_*VECTOR_DIMENSIONS::LIMB_GUESS, guess_.end());

  Eigen::Map<Eigen::VectorXd> distal_plate_section(vec.data(), 6);

  distal_plate_pose_ = ToAffine3D( distal_plate_section );

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


  return false;

}






void RobotPhysics::GuessNullInit() const
{

  guess_.clear();

  for (unsigned int i=0;i<VECTOR_DIMENSIONS::LIMB_GUESS*number_of_limbs_;i++)
    guess_.push_back(0);


  GuessInitDistalPlateSection();

}

void RobotPhysics::GuessInitDistalPlateSection() const
{
  //  Include for distal plate position and orientation
  guess_.push_back( distal_plate_pose_.translation().x() );
  guess_.push_back( distal_plate_pose_.translation().y() );
  guess_.push_back( distal_plate_pose_.translation().z() );
  Eigen::Vector3d euler_angles = distal_plate_pose_.linear().eulerAngles(0, 1, 2);
  guess_.push_back( euler_angles[0] );
  guess_.push_back( euler_angles[1] );
  guess_.push_back( euler_angles[2] );
}



} //  END namespace physics_core
