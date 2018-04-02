


//given an image, compute all models w/rt box and return in a "Shipment" object
bool BoxInspector::model_poses_wrt_box(osrf_gear::Shipment &shipment_status) {
    ROS_INFO("model_poses_wrt_box()");
    geometry_msgs::Pose cam_pose, box_pose_wrt_cam, model_pose_wrt_cam, part_pose_wrt_box;
    geometry_msgs::PoseStamped box_pose_wrt_world, part_pose_wrt_world;
    Eigen::Affine3d affine_cam_wrt_world, affine_part_wrt_cam, affine_part_wrt_box,
            affine_box_pose_wrt_world, affine_part_wrt_world;

    get_new_snapshot_from_box_cam();
    bool found_box = false;
    int i_box = 0;
    int num_models;

    num_models = box_inspector_image_.models.size();
    //parse the  image and compute model poses w/rt box; ignore  the box itself
    if (num_models == 0) {
        ROS_WARN("model_poses_wrt_box(): image has zero models");
        return false;
    }
    ROS_INFO("box cam sees %d models", num_models);

    //look for the box:
    string box_name("shipping_box"); //does a model match this name?
    osrf_gear::Model model;
    cam_pose = box_inspector_image_.pose;

    for (int imodel = 0; imodel < num_models; imodel++) {
        model = box_inspector_image_.models[imodel];
        string model_name(model.type);
        if (model_name == box_name) {
            box_pose_wrt_cam = model.pose;
            ROS_INFO_STREAM("box pose w/rt camera: " << box_pose_wrt_cam << endl);
            found_box = true;
            i_box = imodel; //remember where box is in the list of models
            box_pose_wrt_world = compute_stPose(cam_pose, box_pose_wrt_cam);
            affine_box_pose_wrt_world = xformUtils_.transformPoseToEigenAffine3d(box_pose_wrt_world);
            break;
        }
    }
    if (!found_box) {
        ROS_WARN("model_poses_wrt_box(): did not find box in view");
        return false;
    }

    osrf_gear::Product product;
    //if here, image contains box, and box pose w/rt world is in affine_box_pose_wrt_world
    for (int imodel = 0; imodel < num_models; imodel++) {
        if (imodel != i_box) { //if here, have a model NOT the box
            model = box_inspector_image_.models[imodel];
            string model_name(model.type);
            ROS_INFO_STREAM("model: " << model << endl);
            model_pose_wrt_cam = model.pose;
            part_pose_wrt_world = compute_stPose(cam_pose, model_pose_wrt_cam);
            //g_actual_part_pose_wrt_world_[igmodel].pose = part_pose_wrt_world.pose;
            //g_actual_part_pose_wrt_world_[igmodel].type = model.type;
            //igmodel ++;
            affine_part_wrt_world = xformUtils_.transformPoseToEigenAffine3d(part_pose_wrt_world);
            ROS_INFO_STREAM("part pose wrt world: " << part_pose_wrt_world << endl);
            
            //ROS_WARN("model_poses_wrt_box(): FINISH ME!  compute part_pose_wrt_box");
            //MISSING LINES HERE...
            affine_part_wrt_box = affine_box_pose_wrt_world.inverse() * affine_part_wrt_world;
            part_pose_wrt_box = xformUtils_.transformEigenAffine3dToPose(affine_part_wrt_box);

            //put this into "shipment"  object:
            //string shipment_type
            //box_inspector/Product[] products
            //  string type
            //  geometry_msgs/Pose pose    
            product.type = model.type;
            product.pose = part_pose_wrt_box;
            shipment_status.products.push_back(product);
        }
    }
    ROS_INFO_STREAM("resulting part poses w/rt box: " << shipment_status << endl);

}



//given a shipment description that specifies desired parts and  poses with respect to box,
//convert this to poses of parts w/rt world;
//robot needs to know current and desired part poses  w/rt world

void BoxInspector::compute_shipment_poses_wrt_world(osrf_gear::Shipment shipment_wrt_box,
        geometry_msgs::PoseStamped box_pose_wrt_world,
        vector<osrf_gear::Model> &desired_models_wrt_world) {
    int num_models = 0;
    osrf_gear::Model model;
    geometry_msgs::Pose model_wrt_box_pose;   //first jigsaw
    geometry_msgs::Pose box_wrt_world_pose = box_pose_wrt_world.pose;  //second jigsaw
    geometry_msgs::PoseStamped desired_model_wrt_world;   //need this type to compute stPose
    //osrf_gear::Model model;

    num_models = shipment_wrt_box.products.size(); //number of products in this desired shipment
    if (num_models == 0) //no product required in the box
    {
        ROS_INFO("Empty box required");
    }
    else
    {
        ROS_INFO("%d models required in the box", num_models);
        for(int i_model = 0; i_model < num_models; i_model++)
        {
            model_wrt_box_pose = shipment_wrt_box.products[i_model].pose;
            desired_model_wrt_world = compute_stPose(box_wrt_world_pose,model_wrt_box_pose);
            //model.pose = desired_model_wrt_world.pose;
            //model.type = shipment_wrt_box.products[i_model].type;
            model.type = shipment_wrt_box.products[i_model].type;
            model.pose = desired_model_wrt_world.pose;
            desired_models_wrt_world.push_back(model);
            //desired_models_wrt_world[i_model].type = shipment_wrt_box.products[i_model].type;
            //desired_models_wrt_world[i_model].pose = desired_model_wrt_world.pose;
        }
    }
    ROS_INFO("compute_shipment_poses_wrt_world complete!");
    
}


