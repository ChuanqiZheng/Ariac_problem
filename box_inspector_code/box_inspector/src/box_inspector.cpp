//box_inspector.cpp implementation of class/library
#include <box_inspector/box_inspector.h>
//#include "box_inspector_fncs.cpp" //more code, outside this file
#include "box_inspector_fncs.cpp" //more code, outside this file
#define pi 3.1415926
BoxInspector::BoxInspector(ros::NodeHandle* nodehandle) : nh_(*nodehandle) { //constructor
  //set up camera subscriber:
 box_camera_subscriber_ = nh_.subscribe("/ariac/box_camera_1", 1, &BoxInspector::box_camera_callback, this);
   got_new_snapshot_=false; //trigger to get new snapshots

 }

//to request a new snapshot, set need_new_snapshot_ = true, and make sure to give a ros::spinOnce()
 void BoxInspector::box_camera_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg) {
  if (!got_new_snapshot_) {
        box_inspector_image_ = *image_msg;  //copy the current message to a member data var, i.e. freeze the snapshot
        got_new_snapshot_ =  true;
        ROS_INFO_STREAM("received box-camera image of: "<<box_inspector_image_<<endl);
        int n_models = box_inspector_image_.models.size();
        ROS_INFO("%d models seen ",n_models);
      }
    }

//method to request a new snapshot from logical camera; blocks until snapshot is ready,
// then result will be in box_inspector_image_
    void BoxInspector::get_new_snapshot_from_box_cam() {
      got_new_snapshot_= false;
      ROS_INFO("waiting for snapshot from camera");
      while (!got_new_snapshot_) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
      }
      ROS_INFO("got new snapshot");
    }


//here is  the main fnc; provide a list of models, expressed as desired parts w/ poses w/rt box;
//get a box-camera logical image and  parse it
//populate the vectors as follows:
// satisfied_models_wrt_world: vector of models that match shipment specs, including precision location in box
// misplaced_models_wrt_world: vector of models that belong in the shipment, but are imprecisely located in box
// missing_models_wrt_world:  vector of models that are requested in the shipment, but not yet present in the box
// orphan_models_wrt_world: vector of models that are seen in the box, but DO NOT belong in the box
    void BoxInspector::update_inspection(vector<osrf_gear::Model> desired_models_wrt_world,
     vector<osrf_gear::Model> &satisfied_models_wrt_world,
     vector<osrf_gear::Model> &misplaced_models_actual_coords_wrt_world,
     vector<osrf_gear::Model> &misplaced_models_desired_coords_wrt_world,
     vector<osrf_gear::Model> &missing_models_wrt_world,
     vector<osrf_gear::Model> &orphan_models_wrt_world) {
  //WRITE ME!!!
  //ROS_WARN("NEED TO WRITE update_inspection() ");
      vector<osrf_gear::Model> actual_part_pose_wrt_world;
      vector<osrf_gear::Model> pending_desired_models_wrt_world;
      vector<osrf_gear::Model> pending_actual_models_wrt_world;
      vector<osrf_gear::Model> update_desired_models_wrt_world;
      vector<osrf_gear::Model> update_actual_models_wrt_world;
      geometry_msgs::Pose cam_pose, model_pose_wrt_cam;
      geometry_msgs::PoseStamped part_pose_wrt_world;

      bool found_box = false;
      int i_box = 0;

      got_new_snapshot_=false;
      while(!got_new_snapshot_) {
     ros::spinOnce(); // refresh camera image
     ros::Duration(0.5).sleep();
     ROS_INFO("waiting for logical camera image");
   }
   ROS_INFO("got new image");
   int num_parts_seen =  box_inspector_image_.models.size();
   int desired_num_parts = desired_models_wrt_world.size();
   ROS_INFO("update_inspection: box camera saw %d objects (including box)",num_parts_seen);
   ROS_INFO("Order desires %d parts (not including box)",desired_num_parts);
  orphan_models_wrt_world.clear(); //this will be empty, unless something very odd happens
  satisfied_models_wrt_world.clear();  //shipment will be complete when this matches parts/poses specified in shipment
  misplaced_models_actual_coords_wrt_world.clear();
  misplaced_models_desired_coords_wrt_world.clear();
  missing_models_wrt_world.clear();
  pending_desired_models_wrt_world.clear();
  pending_actual_models_wrt_world.clear();
  update_desired_models_wrt_world.clear();
  update_actual_models_wrt_world.clear();

  string box_name("shipping_box");
  osrf_gear::Model model;
  for (int i=0;i<num_parts_seen;i++) {
   model = box_inspector_image_.models[i];
   string model_name(model.type);
   if (model_name == box_name)
   {
     i_box = i;
     found_box = true;
     break;
   }
 }
  if (!found_box) { //to be implemented
    ROS_WARN("model_poses_wrt_box(): did not find box in view");
  }
  cam_pose = box_inspector_image_.pose;
  for (int i = 0; i < num_parts_seen; i++) {
        if (i != i_box) { //if here, have a model NOT the box
          model = box_inspector_image_.models[i];
          model_pose_wrt_cam = model.pose;
          part_pose_wrt_world = compute_stPose(cam_pose, model_pose_wrt_cam);
          model.pose = part_pose_wrt_world.pose;
          actual_part_pose_wrt_world.push_back(model);
        }
      }
      /*
      pending_desired_models_wrt_world = desired_models_wrt_world;
      int num_pending_desired = pending_desired_models_wrt_world.size();
      int id_pending_desired;
      pending_actual_models_wrt_world = actual_part_pose_wrt_world;
      int num_pending_actual = pending_actual_models_wrt_world.size();
      int id_pending_actual;*/
      update_desired_models_wrt_world = desired_models_wrt_world;
      //pending_desired_models_wrt_world = desired_models_wrt_world;
      int num_update_desired = update_desired_models_wrt_world.size();
      int id_update_desired;
      update_actual_models_wrt_world = actual_part_pose_wrt_world;
      pending_actual_models_wrt_world = actual_part_pose_wrt_world;
      int num_update_actual = update_actual_models_wrt_world.size();
      int id_update_actual;
      geometry_msgs::Pose pose_update_desired_wrt_actual;
      Eigen::Affine3d affine_update_desired_models_wrt_world, affine_update_actual_models_wrt_world, affine_update_desired_wrt_actual;
      bool satisfied_model_found = false;
      
      for (int i_desired=0;i_desired<num_update_desired;i_desired++) 
      {
        id_update_desired = mappings[update_desired_models_wrt_world[i_desired].type];
        satisfied_model_found = false;
        update_actual_models_wrt_world.clear();
        update_actual_models_wrt_world = pending_actual_models_wrt_world;
        num_update_actual = update_actual_models_wrt_world.size();
        pending_actual_models_wrt_world.clear();
        for (int i_actual=0;i_actual<num_update_actual;i_actual++)
        {
          id_update_actual = mappings[update_actual_models_wrt_world[i_actual].type];
          if((id_update_desired == id_update_actual)&&(!satisfied_model_found))
          {
            float dx = update_desired_models_wrt_world[i_desired].pose.position.x - update_actual_models_wrt_world[i_actual].pose.position.x;
            float dy = update_desired_models_wrt_world[i_desired].pose.position.y - update_actual_models_wrt_world[i_actual].pose.position.y;
            float dz = update_desired_models_wrt_world[i_desired].pose.position.z - update_actual_models_wrt_world[i_actual].pose.position.z;
            float dist = (float)sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < 0.03)
            {
              affine_update_desired_models_wrt_world = xformUtils_.transformPoseToEigenAffine3d(update_desired_models_wrt_world[i_desired].pose);
              affine_update_actual_models_wrt_world = xformUtils_.transformPoseToEigenAffine3d(update_actual_models_wrt_world[i_actual].pose);
              affine_update_desired_wrt_actual = affine_update_actual_models_wrt_world.inverse() * affine_update_desired_models_wrt_world;
              pose_update_desired_wrt_actual = xformUtils_.transformEigenAffine3dToPose(affine_update_desired_wrt_actual);
              float angle_update_desired_wrt_actual = 2*acos(pose_update_desired_wrt_actual.orientation.w);
              while (angle_update_desired_wrt_actual > pi)
              {
                angle_update_desired_wrt_actual -= 2*pi;
              }
              if ((angle_update_desired_wrt_actual<0.1)&&(angle_update_desired_wrt_actual>-0.1))
              {
                satisfied_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
                satisfied_model_found = true;
              }
              else
              {
                pending_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
              }
            }
            else
            {
              pending_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
            }
          }
          else
          {
            pending_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
          }
        }
        if(!satisfied_model_found)
        {
          pending_desired_models_wrt_world.push_back(update_desired_models_wrt_world[i_desired]);
        }
      }
      //ROS_INFO_STREAM("size of pending_desired_models_wrt_world:"<< pending_desired_models_wrt_world.size() << endl);
      //ROS_INFO_STREAM("size of pending_actual_models_wrt_world:"<< pending_actual_models_wrt_world.size() << endl);
      /*
      update_actual_models_wrt_world.clear();
      update_actual_models_wrt_world = pending_actual_models_wrt_world;
      pending_actual_models_wrt_world.clear(); */
      update_desired_models_wrt_world.clear();
      update_desired_models_wrt_world = pending_desired_models_wrt_world;
      pending_desired_models_wrt_world.clear();
      num_update_desired = update_desired_models_wrt_world.size();

      vector<osrf_gear::Model> pending_misplaced_actual_models_wrt_world;
      pending_misplaced_actual_models_wrt_world.clear();

      for (int i_desired=0;i_desired<num_update_desired;i_desired++) 
      {
        id_update_desired = mappings[update_desired_models_wrt_world[i_desired].type];
        update_actual_models_wrt_world.clear();
        update_actual_models_wrt_world = pending_actual_models_wrt_world;
        num_update_actual = update_actual_models_wrt_world.size();
        pending_actual_models_wrt_world.clear();
        for (int i_actual=0;i_actual<num_update_actual;i_actual++)
        {
          id_update_actual = mappings[update_actual_models_wrt_world[i_actual].type];
          if(id_update_desired == id_update_actual)
          {
            pending_misplaced_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
          }
          else
          {
            pending_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
          }
        }
        int num_misplaced_candidates = pending_misplaced_actual_models_wrt_world.size();
        if (num_misplaced_candidates == 0)
        {
          missing_models_wrt_world.push_back(update_desired_models_wrt_world[i_desired]);
        }
        else if (num_misplaced_candidates == 1)
        {
          misplaced_models_actual_coords_wrt_world.push_back(pending_misplaced_actual_models_wrt_world[0]);
          misplaced_models_desired_coords_wrt_world.push_back(update_desired_models_wrt_world[i_desired]);
          pending_misplaced_actual_models_wrt_world.clear();
        }
        else
        {
          float former_dist = 1000.00;
          int i_success_candidate;
          for(int i_misplaced=0;i_misplaced<num_misplaced_candidates;i_misplaced++)
          {
            float dx = update_desired_models_wrt_world[i_desired].pose.position.x - pending_misplaced_actual_models_wrt_world[i_misplaced].pose.position.x;
            float dy = update_desired_models_wrt_world[i_desired].pose.position.y - pending_misplaced_actual_models_wrt_world[i_misplaced].pose.position.y;
            float dz = update_desired_models_wrt_world[i_desired].pose.position.z - pending_misplaced_actual_models_wrt_world[i_misplaced].pose.position.z;
            float dist = (float)sqrt(dx*dx + dy*dy + dz*dz);
            if(dist < former_dist)
            {
              former_dist = dist;
              i_success_candidate = i_misplaced;
            }
          }
          for(int i_misplaced=0;i_misplaced<num_misplaced_candidates;i_misplaced++)
          {
            if(i_misplaced != i_success_candidate)
            {
              pending_actual_models_wrt_world.push_back(pending_misplaced_actual_models_wrt_world[i_misplaced]);
            }
          }
          misplaced_models_actual_coords_wrt_world.push_back(pending_misplaced_actual_models_wrt_world[i_success_candidate]);
          misplaced_models_desired_coords_wrt_world.push_back(update_desired_models_wrt_world[i_desired]);
          pending_misplaced_actual_models_wrt_world.clear();
        }
      }

      int num_orphan = pending_actual_models_wrt_world.size();
      for(int i_orphan=0;i_orphan<num_orphan;i_orphan++)
      {
        orphan_models_wrt_world.push_back(pending_actual_models_wrt_world[i_orphan]);
      }


  //THIS  IS WRONG...but an example of how to get models from image and sort into category vectors
  /*
  for (int i=0;i<num_parts_seen;i++) {
     orphan_models_wrt_world.push_back(box_inspector_image_.models[i]);
  }*/


   }

//intent of this function is, get a snapshot from the box-inspection camera;
//parse the image data to see if a shipping box is present
//if not, return false
//if seen, transform box pose to box pose w/rt world frame,
//    copy data to reference arg box_pose_wrt_world
//    and return "true"

   bool BoxInspector::get_box_pose_wrt_world(geometry_msgs::PoseStamped &box_pose_wrt_world) {
    geometry_msgs::Pose cam_pose, box_pose; //cam_pose is w/rt world, but box_pose is w/rt camera

    //get a new snapshot of the box-inspection camera:
    get_new_snapshot_from_box_cam();

    //ROS_INFO("got box-inspection camera snapshot");
    //look for box in model list:
    int num_models = box_inspector_image_.models.size(); //how many models did the camera see?
    if (num_models == 0) return false;
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose; //camera wrt world
    ROS_INFO("box cam sees %d models", num_models);
    for (int imodel = 0; imodel < num_models; imodel++) {
      model = box_inspector_image_.models[imodel];
      string model_name(model.type);
      if (model_name == box_name) {
        box_pose = model.pose;
        ROS_INFO_STREAM("get_box_pose_wrt_world(): found box at pose " << box_pose << endl);
            //ROS_WARN("USE THIS INFO TO COMPUTE BOX POSE WRT WORLD AND  POPULATE box_pose_wrt_world");
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose); //box wrt world
            ROS_INFO_STREAM("box_pose_wrt_world: " << box_pose_wrt_world << endl);
            return true;
          }
        }
    //if reach here, did not find shipping_box
        ROS_WARN("get_box_pose_wrt_world(): shipping-box not seen!");
        return false;
      }

//helper  function:
//given a camera pose and a part-pose (or box-pose) w/rt camera, compute part pose w/rt world
//xform_utils library should help here

      geometry_msgs::PoseStamped BoxInspector::compute_stPose(geometry_msgs::Pose cam_pose, geometry_msgs::Pose part_pose) {

        geometry_msgs::PoseStamped stPose_part_wrt_world;
    //compute part-pose w/rt world and return as a pose-stamped message object
        Eigen::Affine3d cam_wrt_world, part_wrt_cam, part_wrt_world;

        cam_wrt_world = xformUtils_.transformPoseToEigenAffine3d(cam_pose);
        part_wrt_cam = xformUtils_.transformPoseToEigenAffine3d(part_pose);
        part_wrt_world = cam_wrt_world*part_wrt_cam;
        geometry_msgs::Pose pose_part_wrt_world = xformUtils_.transformEigenAffine3dToPose(part_wrt_world);
        geometry_msgs::PoseStamped part_pose_stamped;
        part_pose_stamped.header.stamp = ros::Time::now();
        part_pose_stamped.header.frame_id = "world";
        part_pose_stamped.pose = pose_part_wrt_world;
        return part_pose_stamped;
      }


//rosmsg show osrf_gear/LogicalCameraImage: 
/*
osrf_gear/Model[] models
  string type
  geometry_msgs/Pose pose
    geometry_msgs/Point position
      float64 x
      float64 y
      float64 z
    geometry_msgs/Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w
*/
