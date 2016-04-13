#include <godel_surface_detection/services/surface_blending_service.h>
#include <godel_msgs/TrajectoryExecution.h>
// Process Execution
#include <godel_msgs/BlendProcessExecution.h>
#include <godel_msgs/KeyenceProcessExecution.h>
// Process Planning
#include <godel_msgs/BlendProcessPlanning.h>
#include <godel_msgs/KeyenceProcessPlanning.h>
// Param server
#include <godel_param_helpers/godel_param_helpers.h>
#include <fstream>
#include <iostream>

//Topic
const static std::string SURFACE_DETECTION_SERVICE = "surface_detection";
const static std::string SELECT_SURFACE_SERVICE = "select_surface";
const static std::string PROCESS_PATH_SERVICE = "process_path";
const static std::string GET_AVAILABLE_MOTION_PLANS_SERVICE = "get_available_motion_plans";
const static std::string SELECT_MOTION_PLAN_SERVICE = "select_motion_plan";
const static std::string SURFACE_BLENDING_PARAMETERS = "surface_blending_parameters";
const static int MAX_SERVICECALL_FAILURE = 20;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_auto_blending");
  ros::NodeHandle node_handle;
  ros::Rate loop_rate(10);

  godel_msgs::SurfaceBlendingParameters param_srv;
  godel_msgs::SurfaceDetection surfaceDetection_srv;
  godel_msgs::SelectSurface::Request selectSurface_req;
  godel_msgs::SelectSurface::Response selectSurface_res;
  godel_msgs::ProcessPlanning process_plan;
  godel_msgs::GetAvailableMotionPlans motionQuery_srv;
  godel_msgs::SelectMotionPlan motion_srv;
  std::vector < std::string > plan_names;
  int serviceCall_failure = 0;

  //ServiceClient
  ros::ServiceClient param_client = node_handle.serviceClient<godel_msgs::SurfaceBlendingParameters>(
      SURFACE_BLENDING_PARAMETERS);
  ros::ServiceClient select_surface_client_ = node_handle.serviceClient<godel_msgs::SelectSurface>(
      SELECT_SURFACE_SERVICE);
  ros::ServiceClient process_plan_client_ = node_handle.serviceClient<godel_msgs::ProcessPlanning>(
      PROCESS_PATH_SERVICE);
  ros::ServiceClient surface_client_ = node_handle.serviceClient<godel_msgs::SurfaceDetection>(
      SURFACE_DETECTION_SERVICE);
  ros::ServiceClient get_motion_plans_client_ = node_handle.serviceClient<godel_msgs::GetAvailableMotionPlans>(
      GET_AVAILABLE_MOTION_PLANS_SERVICE);
  ros::ServiceClient sim_client_ = node_handle.serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);

  while (ros::ok())
  {
    if (serviceCall_failure > MAX_SERVICECALL_FAILURE)
    {
      ROS_ERROR_STREAM("Service Call failures exceed the MAX_SERVICECALL_FAILURE");
      return -1;
    }

    //Get parameters for surface detection
    param_srv.request.action = param_srv.request.GET_CURRENT_PARAMETERS;
    if (!param_client.call(param_srv))
    {
      ROS_WARN_STREAM("Unable to call the "<<SURFACE_BLENDING_PARAMETERS<<" service");
      serviceCall_failure++;
      continue;
    }

    //Scan and detect surface
    surfaceDetection_srv.request.action = 3;
    surfaceDetection_srv.request.use_default_parameters = false;
    surfaceDetection_srv.request.robot_scan = param_srv.response.robot_scan;
    surfaceDetection_srv.request.surface_detection = param_srv.response.surface_detection;
    if (!surface_client_.call(surfaceDetection_srv))
    {
      ROS_WARN_STREAM("Unable to call the "<<SURFACE_DETECTION_SERVICE<<" service");
      serviceCall_failure++;
      continue;
    }

    //Select all surface
    selectSurface_req.action = selectSurface_req.SELECT_ALL;
    if (!select_surface_client_.call(selectSurface_req, selectSurface_res))
    {
      ROS_WARN_STREAM("Unable to call the "<<SELECT_SURFACE_SERVICE<<" service");
      serviceCall_failure++;
      continue;
    }

    //Generate path
    process_plan.request.use_default_parameters = false;
    process_plan.request.params = param_srv.response.blending_plan;
    process_plan.request.scan_params = param_srv.response.scan_plan;
    process_plan.request.action = 2;
    if (!process_plan_client_.call(process_plan))
    {
      ROS_WARN_STREAM("Unable to call the "<<PROCESS_PATH_SERVICE<<" service");
      serviceCall_failure++;
      continue;
    }

    if (!get_motion_plans_client_.call(motionQuery_srv))
    {
      ROS_WARN_STREAM("Unable to call the "<<GET_AVAILABLE_MOTION_PLANS_SERVICE<<" service");
      serviceCall_failure++;
      continue;
    }
    plan_names = motionQuery_srv.response.names;

    //Blending Simulation
    for (std::size_t i = 0; i < plan_names.size(); ++i)
    {
      motion_srv.request.name = plan_names[i];
      motion_srv.request.simulate = true;
      motion_srv.request.wait_for_execution = true;
      if (!sim_client_.call(motion_srv))
      {
        ROS_WARN_STREAM("Unable to call the "<<SELECT_MOTION_PLAN_SERVICE<<" service");
        serviceCall_failure++;
        continue;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
