#include <hlpr_jaco_trajectory_action_server/hlpr_jaco_trajectory_action_server.h>
#include <hlpr_jaco_trajectory_action_server/util.h>

#include <dynamic_reconfigure/Reconfigure.h>
#include <iostream>

using namespace std;

JacoTrajectoryController::JacoTrajectoryController() : pnh("~"),
  smoothTrajectoryServer(pnh, "trajectory", boost::bind(&JacoTrajectoryController::executeSmoothTrajectory, this, _1), false)
{
  pnh.param("max_curvature", maxCurvature, 100.0);

  jointNames.clear();
  jointNames.push_back("j2s7s300_joint_1");
  jointNames.push_back("j2s7s300_joint_2");
  jointNames.push_back("j2s7s300_joint_3");
  jointNames.push_back("j2s7s300_joint_4");
  jointNames.push_back("j2s7s300_joint_5");
  jointNames.push_back("j2s7s300_joint_6");
  jointNames.push_back("j2s7s300_joint_7");


  // Connect to the low-level angular driver from kinova-ros
  angularCmdPublisher = n.advertise<kinova_msgs::JointVelocity>("j2s7s300_driver/in/joint_velocity", 1);

  // Subscribes to the joint states of the robot
  jointStatesSubscriber = n.subscribe("joint_states", 1, &JacoTrajectoryController::jointStateCallback, this);


  // Start the trajectory server
  smoothTrajectoryServer.start();
}

void JacoTrajectoryController::jointStateCallback(const sensor_msgs::JointState &msg)
{

  // Create a new message
  sensor_msgs::JointState arm_msg;
  arm_msg.position.resize(NUM_JACO_JOINTS);
  arm_msg.velocity.resize(NUM_JACO_JOINTS);
  arm_msg.effort.resize(NUM_JACO_JOINTS);
  arm_msg.name.resize(NUM_JACO_JOINTS);

  // Cycle through the number of JACO joints
  for (int joint_id = 0; joint_id < NUM_JACO_JOINTS; joint_id++){

    // Find the location of the joint
    string joint_name = jointNames[joint_id];
    int msg_loc = distance(msg.name.begin(), find(msg.name.begin(), msg.name.end(), joint_name));

    // Pull out joint loc and store
    arm_msg.position[joint_id] = msg.position.at(msg_loc);
    arm_msg.name[joint_id] = msg.name.at(msg_loc);
    arm_msg.velocity[joint_id] = msg.velocity.at(msg_loc);
    arm_msg.effort[joint_id] = msg.effort.at(msg_loc);
  }

  jointStates = arm_msg;
  //cout << "Current names: " << jointStates.name[0] << ", " << jointStates.name[1] << ", " << jointStates.name[2] << ", " << jointStates.name[3] << ", " << jointStates.name[4] << ", " << jointStates.name[5] << jointStates.name[6] << endl;
  //cout << "Current values: " << jointStates.position[0] << ", " << jointStates.position[1] << ", " << jointStates.position[2] << ", " << jointStates.position[3] << ", " << jointStates.position[4] << ", " << jointStates.position[5] << jointStates.position[6] << endl;
}


void JacoTrajectoryController::executeSmoothTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal) {

  std::size_t const numPoints = goal->trajectory.points.size();
  ecl::Array<double> timePoints;
  std::vector<std::unique_ptr<Spline> > splines;
  std::tie(timePoints, splines) = toSplines(goal->trajectory, this->jointNames, this->maxCurvature);

  // // Gather timing corrections for trajectory segments that violate max velocity
  // float correctedTime[numPoints] = { };
  // for (unsigned int i = 1; i < numPoints; i++)
  // {
  //   float maxTime = 0.0;
  //   float vel = 0.0;

  //   float plannedTime = timePoints[i] - timePoints[i-1];
  //   bool validTime = plannedTime > 0;

  //   for (unsigned int j = 0; j < NUM_JACO_JOINTS; j++)
  //   {
  //     float time = fabs(jointPoints[j][i] - jointPoints[j][i-1]);
  //     if (plannedTime > 0)
  //       vel = fabs(jointPoints[j][i] - jointPoints[j][i-1]) / plannedTime;

  //     if (j <= 3)
  //     {
  //       time /= 0.9*LARGE_ACTUATOR_VELOCITY;
  //       if (plannedTime > 0 && vel > 0.9*LARGE_ACTUATOR_VELOCITY)
  //         validTime = false;
  //     }
  //     else
  //     {
  //       time /= 0.9*SMALL_ACTUATOR_VELOCITY;
  //       if (plannedTime > 0 && vel > 0.9*SMALL_ACTUATOR_VELOCITY)
  //         validTime = false;
  //     }

  //     if (time > maxTime)
  //       maxTime = time;
  //   }

  //   if (!validTime)
  //     correctedTime[i] = maxTime;
  // }

  // // Apply timing corrections
  // for (unsigned int i = 1; i < numPoints; i++)
  // {
  //   correctedTime[i] += correctedTime[i-1];
  //   timePoints[i] += correctedTime[i];
  // }

  // // Print warning if time corrections applied
  // if (correctedTime[numPoints-1] > 0)
  // {
  //   ROS_WARN("Warning: Timing of joint trajectory violates max velocities, using computed time"); 
  //   if (ros::service::exists("/move_group/trajectory_execution/set_parameters", false))
  //   {
  //     dynamic_reconfigure::ReconfigureRequest req;
  //     dynamic_reconfigure::ReconfigureResponse resp;

  //     ros::service::call("/move_group/trajectory_execution/set_parameters", req, resp);

  //     for (auto const& it : resp.config.bools)
  //       if (it.name == "execution_duration_monitoring" && it.value)
  //         ROS_WARN("Warning: Execution duration monitoring turned on. This may cause trajectory to be premempted before completion.");
  //   }
  // }

  //control loop
  bool trajectoryComplete = false;
  double startTime = ros::Time::now().toSec();
  double t = 0;
  float error[NUM_JACO_JOINTS];
  float totalError;
  float prevError[NUM_JACO_JOINTS] = {0};
  float currentPoint;
  double current_joint_pos[NUM_JACO_JOINTS];
  kinova_msgs::JointVelocity trajectoryPoint;
  ros::Rate rate(100);
  bool reachedFinalPoint;
  ros::Time finalPointTime;


  // Sending to the real robot
  while (!trajectoryComplete)
  {
    //check for preempt requests from clients
    if (smoothTrajectoryServer.isPreemptRequested())
    {
      //stop gripper control
      trajectoryPoint.joint1 = 0.0;
      trajectoryPoint.joint2 = 0.0;
      trajectoryPoint.joint3 = 0.0;
      trajectoryPoint.joint4 = 0.0;
      trajectoryPoint.joint5 = 0.0;
      trajectoryPoint.joint6 = 0.0;
      trajectoryPoint.joint7 = 0.0;

      angularCmdPublisher.publish(trajectoryPoint);

      //preempt action server
      smoothTrajectoryServer.setPreempted();
      ROS_INFO("Smooth trajectory server preempted by client");

      return;
    }

    //get time for trajectory
    t = ros::Time::now().toSec() - startTime;
    if (t > timePoints.at(timePoints.size() - 1))
    {
      //use final trajectory point as the goal to calculate error until the error
      //is small enough to be considered successful
      /*
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetAngularPosition(position_data);
      }
      */

      if (!reachedFinalPoint)
      {
        reachedFinalPoint = true;
        finalPointTime = ros::Time::now();
      }

      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        current_joint_pos[i] = jointStates.position[i];
      }

      bool jointError = false;
      double maxError = 0;
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);

        // Check if we're using a cubic or a linear spline
        double splineValue = splines[i]->at(timePoints.at(timePoints.size() - 1));
        // Now generate the value
        error[i] = nearest_equivalent(simplify_angle(splineValue),
                                      currentPoint) - currentPoint;
        jointError = jointError || fabs(error[i]) > ERROR_THRESHOLD;
      }

      if (!jointError || ros::Time::now() - finalPointTime >= ros::Duration(3.0))
      {
        cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;
        //stop arm
        trajectoryPoint.joint1 = 0.0;
        trajectoryPoint.joint2 = 0.0;
        trajectoryPoint.joint3 = 0.0;
        trajectoryPoint.joint4 = 0.0;
        trajectoryPoint.joint5 = 0.0;
        trajectoryPoint.joint6 = 0.0;
        trajectoryPoint.joint7 = 0.0;
        angularCmdPublisher.publish(trajectoryPoint);
        trajectoryComplete = true;
        ROS_INFO("Trajectory complete!");
        break;
      }
    }
    else
    {
      //calculate error
      /*
      {
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        GetAngularPosition(position_data);
      }
      */
      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        current_joint_pos[i] = jointStates.position[i];
      }

      for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
      {
        currentPoint = simplify_angle(current_joint_pos[i]);
        // Check if we're using a cubic or a linear spline
        double splineValue = splines[i]->at(t);
        error[i] = nearest_equivalent(simplify_angle(splineValue), currentPoint) - currentPoint;
      }
    }

    //calculate control input
    //populate the velocity command
    trajectoryPoint.joint1 = (KP * error[0] + KV * (error[0] - prevError[0]) * RAD_TO_DEG);
    trajectoryPoint.joint2 = (KP * error[1] + KV * (error[1] - prevError[1]) * RAD_TO_DEG);
    trajectoryPoint.joint3 = (KP * error[2] + KV * (error[2] - prevError[2]) * RAD_TO_DEG);
    trajectoryPoint.joint4 = (KP * error[3] + KV * (error[3] - prevError[3]) * RAD_TO_DEG);
    trajectoryPoint.joint5 = (KP * error[4] + KV * (error[4] - prevError[4]) * RAD_TO_DEG);
    trajectoryPoint.joint6 = (KP * error[5] + KV * (error[5] - prevError[5]) * RAD_TO_DEG);
    trajectoryPoint.joint7 = (KP * error[6] + KV * (error[6] - prevError[6]) * RAD_TO_DEG);

    //for debugging:
    // cout << "Errors: " << error[0] << ", " << error[1] << ", " << error[2] << ", " << error[3] << ", " << error[4] << ", " << error[5] << endl;

    //send the velocity command
    angularCmdPublisher.publish(trajectoryPoint);

    for (unsigned int i = 0; i < NUM_JACO_JOINTS; i++)
    {
      prevError[i] = error[i];
    }

    rate.sleep();
    ros::spinOnce();
  }

  control_msgs::FollowJointTrajectoryResult result;
  result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
  smoothTrajectoryServer.setSucceeded(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hlpr_jaco_trajectory_action_server");

  JacoTrajectoryController jtc;
  ros::spin();
}

