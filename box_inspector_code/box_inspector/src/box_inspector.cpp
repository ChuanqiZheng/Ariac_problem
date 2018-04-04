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

      vector<osrf_gear::Model> actual_part_pose_wrt_world;//use this and desired_models_wrt_world, to classify models
      vector<osrf_gear::Model> pending_desired_models_wrt_world;
      vector<osrf_gear::Model> pending_actual_models_wrt_world;//need pending lists to record unclassified items
      vector<osrf_gear::Model> update_desired_models_wrt_world;
      vector<osrf_gear::Model> update_actual_models_wrt_world;//need these to update pending lists
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

  //compute actual_part_pose_wrt_world, based on most recent box_inspector_image_
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

      update_desired_models_wrt_world = desired_models_wrt_world;
      int num_update_desired = update_desired_models_wrt_world.size();
      int id_update_desired;
      update_actual_models_wrt_world = actual_part_pose_wrt_world;
      pending_actual_models_wrt_world = actual_part_pose_wrt_world;
      int num_update_actual = update_actual_models_wrt_world.size();
      int id_update_actual;
      geometry_msgs::Pose pose_update_desired_wrt_actual;
      Eigen::Affine3d affine_update_desired_models_wrt_world, affine_update_actual_models_wrt_world, affine_update_desired_wrt_actual;
      bool satisfied_desired_model_found = false;
      bool satisfied_actual_model_found = false;
      
      //First step: find perfectly matched items, push to satisfied_models_wrt_world, rest go to pending lists.
      for (int i_desired=0;i_desired<num_update_desired;i_desired++) //check all desired items
      {
        id_update_desired = mappings[update_desired_models_wrt_world[i_desired].type];
        satisfied_desired_model_found = false;
        update_actual_models_wrt_world.clear();
        update_actual_models_wrt_world = pending_actual_models_wrt_world;
        num_update_actual = update_actual_models_wrt_world.size();
        pending_actual_models_wrt_world.clear();//push "pending" list to "update" list and clear "pending" list. Now "pending" list is ready to be updated again.
        for (int i_actual=0;i_actual<num_update_actual;i_actual++)//check all actual items
        {
          satisfied_actual_model_found = false;
          id_update_actual = mappings[update_actual_models_wrt_world[i_actual].type];
          if((id_update_desired == id_update_actual)&&(!satisfied_desired_model_found))
          {
            float dx = update_desired_models_wrt_world[i_desired].pose.position.x - update_actual_models_wrt_world[i_actual].pose.position.x;
            float dy = update_desired_models_wrt_world[i_desired].pose.position.y - update_actual_models_wrt_world[i_actual].pose.position.y;
            float dz = update_desired_models_wrt_world[i_desired].pose.position.z - update_actual_models_wrt_world[i_actual].pose.position.z;
            float dist = (float)sqrt(dx*dx + dy*dy + dz*dz);
            if (dist < 0.03)//tolerance in position
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
              if ((angle_update_desired_wrt_actual<0.1)&&(angle_update_desired_wrt_actual>-0.1))//tolerance in orientation
              {
                satisfied_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);//satisfied match found, push to satisfied_models_wrt_world
                satisfied_desired_model_found = true;//a reminder whether to push corresponding desired item to pending list
                satisfied_actual_model_found = true;//a reminder whether to push corresponding actual item to pending list
              }
            }
          }
          if(!satisfied_actual_model_found)
          {
            pending_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
          }
        }
        if(!satisfied_desired_model_found)
        {
          pending_desired_models_wrt_world.push_back(update_desired_models_wrt_world[i_desired]);
        }
      }
      
      update_desired_models_wrt_world.clear();
      update_desired_models_wrt_world = pending_desired_models_wrt_world;
      pending_desired_models_wrt_world.clear();//push "pending" list to "update" list and clear "pending" list.
      num_update_desired = update_desired_models_wrt_world.size();

      vector<osrf_gear::Model> pending_misplaced_actual_models_wrt_world;//need this to compute and compare distances between misplaced pairs
      pending_misplaced_actual_models_wrt_world.clear();

      //Step two: check pending list of desired items, they should be either "misplaced" or "missing".
      //Misplaced match is based on minimum distance between items with the same id.
      //Unclassified actual items are pushed to third step, which should be all "orphan"s.
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
          {//all actual items with the right id are qualified candidates
            pending_misplaced_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
          }
          else
          {//actual items with wrong name go to pending list
            pending_actual_models_wrt_world.push_back(update_actual_models_wrt_world[i_actual]);
          }
        }
        int num_misplaced_candidates = pending_misplaced_actual_models_wrt_world.size();
        if (num_misplaced_candidates == 0)
        {//no candidates: this desired item is missing
          missing_models_wrt_world.push_back(update_desired_models_wrt_world[i_desired]);
        }
        else if (num_misplaced_candidates == 1)
        {//only one candidate: push to "misplaced" list
          misplaced_models_actual_coords_wrt_world.push_back(pending_misplaced_actual_models_wrt_world[0]);
          misplaced_models_desired_coords_wrt_world.push_back(update_desired_models_wrt_world[i_desired]);
          pending_misplaced_actual_models_wrt_world.clear();
        }
        else
        {//more than one candidate: compute distance and compare
          float former_dist = 1000.00;//one impossible long distance
          int i_success_candidate;
          for(int i_misplaced=0;i_misplaced<num_misplaced_candidates;i_misplaced++)
          {
            float dx = update_desired_models_wrt_world[i_desired].pose.position.x - pending_misplaced_actual_models_wrt_world[i_misplaced].pose.position.x;
            float dy = update_desired_models_wrt_world[i_desired].pose.position.y - pending_misplaced_actual_models_wrt_world[i_misplaced].pose.position.y;
            float dz = update_desired_models_wrt_world[i_desired].pose.position.z - pending_misplaced_actual_models_wrt_world[i_misplaced].pose.position.z;
            float dist = (float)sqrt(dx*dx + dy*dy + dz*dz);
            if(dist < former_dist)
            {//update shortest distance and corresponding actual item
              former_dist = dist;
              i_success_candidate = i_misplaced;
            }
          }
          for(int i_misplaced=0;i_misplaced<num_misplaced_candidates;i_misplaced++)
          {//push all the failed candidates to pending list
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
      
      //Step 3: All the actual items on pending list go to "orphan" list.
      int num_orphan = pending_actual_models_wrt_world.size();
      for(int i_orphan=0;i_orphan<num_orphan;i_orphan++)
      {
        orphan_models_wrt_world.push_back(pending_actual_models_wrt_world[i_orphan]);
      }
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
