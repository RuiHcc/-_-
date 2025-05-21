
#include <Cluster.h>

FrontierClusterInfo::FrontierClusterInfo(ros::NodeHandle& n):nh(n){
    nh.param("FOV_R", sensor_range, 5.0);
    nh.param("cluster/is_viewpoint", is_viewpoint, false);
    nh.param("cluster/cluster_dp_max", cluster_dp_max, 1.0);
    nh.param("cluster/cluster_dv_max", cluster_dv_max, 1.0);
    nh.param("cluster/cluster_normal_thr", cluster_normal_thr, 1.0);
    nh.param("cluster/k_nearest", k_nearest, 10);
    nh.param("is_on_car", is_on_car, false);
    nh.param("car_odom_z", car_odom_z, 0.1);
    nh.param("car_id", car_id, 0);

    dp_ratio_dv = cluster_dp_max / cluster_dv_max;

    nh.param("cluster/candidate_dphi", candidate_dphi_, -1.0);
    nh.param("cluster/candidate_rmax", candidate_rmax_, -1.0);
    nh.param("cluster/candidate_rmin", candidate_rmin_, -1.0);
    nh.param("cluster/candidate_rnum", candidate_rnum_, -1);
    nh.param("cluster/candidate_phinum", candidate_phinum_, -1);
    nh.param("cluster/viewpoint_gd_dis_thr", viewpoint_gd_dis_thr, -1.0);
    nh.param("cluster/viewpoint_gd_cos_thr", viewpoint_gd_cos_thr, -1.0);

    nh.param("cluster/fmesh_in_hull_dis_thr", this->fmesh_in_hull_dis_thr, 1.0);

    nh.param("cluster/svp_dis", this->svp_dis, 1.0);
    nh.param("cluster/svp_arrive_dis", this->svp_arrive_dis, 1.0);
    nh.param("cluster/svp_refine_r", this->svp_refine_r, 1.0);
    nh.param("cluster/svp_s_thr", this->svp_s_thr, 1.0);

    nh.param("freespace/freespace_dis", freespace_dis, 3.0);
    nh.param("freespace/frontier_gen_frenq", frontier_gen_frenq, 1);
    nh.param("freespace/ftr_dis_thr_from_sense_range", ftr_dis_thr_from_sense_range, 2.0);
    nh.param("frontierq/drone_num", drone_num, -1);
    nh.param("client/robot_id", robot_id, -1);

    update_last_start_time = std::chrono::high_resolution_clock::now();

    frontier_cluster_pub = nh.advertise<visualization_msgs::Marker>("/global_frontier_cluster", 1);
    frontier_center_pub = nh.advertise<visualization_msgs::MarkerArray>("/global_frontier_cluster_center", 1);
    svp_center_pub = nh.advertise<visualization_msgs::MarkerArray>("/global_svp_center", 1);
    frontier_oval_pub = nh.advertise<visualization_msgs::MarkerArray>("/global_frontier_cluster_oval", 1);

    drone_pos_sub = nh.subscribe("grid_map/odom", 1, &FrontierClusterInfo::odomCallBack, this);

    vis_ptr = std::make_shared<visualization::Visualization>(nh);

    for(int i = 0; i < 1000; i++){
        id_pool.push_back(0);
        id_vp_pool.push_back(0);
    }
    
    free_space_fac_.reset(new FreeSpaceFactory(nh, Ptr(this)));

    // frontier_space_timer = nh.createTimer(ros::Duration(0.01), &FrontierClusterInfo::updateFrontierSpace, this);
    surfacepc_kdtree_.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>());

    vector<Vector3d> inpoints;
    vector<Vector3d> outpoints;
    vector<Vector4d> indis;
    vis_ptr->visualize_pointcloud(vvv,"/Hit");
    vis_ptr->visualize_pointcloud(vvvv,"/Avis");
    vis_ptr->visualize_pointcloud(v5,"/ASampleV");



    vis_ptr->visualize_pointcloud(inpoints, "/debug_inpoint");
    vis_ptr->visualize_pointcloud(outpoints, "/debug_outpoint");
    vis_ptr->visualize_texts(indis, 0.02, "/debug_dis2mesh");
}

void FrontierClusterInfo::odomCallBack(const nav_msgs::Odometry& msg){
    drone_pose(0) = msg.pose.pose.position.x;
    drone_pose(1) = msg.pose.pose.position.y;
    drone_pose(2) = msg.pose.pose.position.z;
    if(is_on_car){
        drone_pose(2) = car_odom_z;
    }

    drone_vel(0) = msg.twist.twist.linear.x;
    drone_vel(1) = msg.twist.twist.linear.y;
    drone_vel(2) = msg.twist.twist.linear.z;

}

void FrontierClusterInfo::updateFrontierSpace()
{
    // ROS_ERROR(" updateFrontierSpace event");
    
    ros::Time rostime = ros::Time::now();
    vector<Vector3d> vec;

    // cout<<"updateFrontierSpace "<<grid_map_->recieve_cnt<<", " << grid_inf_map_->recieve_cnt<<","<<free_space_fac_->grid_map_->recieve_cnt << endl;

    if(free_space_fac_->grid_map_->recieve_cnt > 5){
        // ROS_ERROR(" into switch1");
        bool is_arrive = isArriandDeletePass();
        // if(free_space_info.key_pose_list->size() <= 0 || isArriandDeletePass()){
        if(free_space_info.posegraph->getSize() <= 0 || (last_key_pose - drone_pose).norm() > freespace_dis || is_arrive ||
            std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - update_last_start_time).count() > 10000) {
            update_last_start_time = std::chrono::high_resolution_clock::now();
            ROS_INFO("\033[1;35mFreeSapce&Frontier Start \033[0m");  //紫

            double time0 = ros::Time::now().toSec();

            free_space_fac_->map_size = grid_map_->global_map_size;

            FreeSpace fs;
            fs = free_space_fac_->genFreeSpcePoints(drone_pose); 

            for(size_t i = 0; i < fs.convex_points->size(); i++){
                vec.emplace_back(fs.convex_points->points[i].x, fs.convex_points->points[i].y, fs.convex_points->points[i].z);
            }
            vis_ptr->visualize_pointcloud(vec, "/AAA");
            vis_ptr->visualize_pointcloud(vec, "AAA");
            
            std::vector<Vector3d> v;
            std::vector<size_t> inxs;

            fs.create_convexhull(v, inxs);
            // testMeshTable(fs);

            pcl::PointXYZ pt(drone_pose(0), drone_pose(1), drone_pose(2));

            free_space_info.free_space_list.push_back(fs);
            free_space_info.posegraph->push_back(pt);
            temp_num++;

            CellList frontier_cell_set;
            free_space_fac_->LableFrontierSurface(fs, v, inxs, drone_pose, frontier_cell_set);

            free_space_fac_->publishStarCvx(fs, drone_pose, rostime,"world");
            free_space_fac_->publishStarCvxFrontier(fs, drone_pose, rostime,"world");
            double time1 = ros::Time::now().toSec();
            
            // The kdtree include all keypose To find nearby freespace (Only used for current frame)
            free_space_info.posegraph->renewKdtree();
            //! 在这之前的frontier的mesh都是以keypose为原点
            frontier_cluster_list.clear();
            erased_svp_list.clear();

            // return;
            
            // checkFrontierSet(frontier_cell_set, fs);
            v5.clear();
            frontierCluster(frontier_cell_set, fs);
            // ROS_WARN_STREAM("v5: "<< v5.size());
            vis_ptr->visualize_pointcloud(v5,"/ASampleV");
            vis_ptr->visualize_pointcloud(vvv,"/Hit");
            vis_ptr->visualize_pointcloud(vvv,"Hit");


            Vector3d cur_keypose(0,0,0);
            Vector3d cur_svp_pose(0,0,0);
            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
            arrows_pair_list.emplace_back(cur_keypose, cur_svp_pose);
            vis_ptr->visualize_pairline(arrows_pair_list, "AAline", 0.1, visualization::Color::red);
            vis_ptr->visualize_pairline(arrows_pair_list, "AAline_after", 0.1, visualization::Color::green);


            // ROS_INFO("\033[1;35mfrontierCluster Time-Consume: %f ms\033[0m",(ros::Time::now().toSec()-time1) * 1000.0);  //紫
            

            // cout<<"pg size: "<<free_space_info.posegraph->getSize()<<endl;
            // cout<<"pg cnt size: "<<free_space_info.posegraph->getSize()-temp_num<<endl;

            if(free_space_info.free_space_list.size() > frontier_gen_frenq && temp_num % frontier_gen_frenq == 0){
                for(int k = 0; k < frontier_gen_frenq; k++){
                    free_space_info.free_space_list.pop_back();
                    free_space_info.posegraph->erase_back();
                }
            
                free_space_info.free_space_list.push_back(fs);
                free_space_info.posegraph->push_back(pt);
                temp_num = 0;
                addPoseEdge();
                // cout<<"[Erase]pg size: "<<free_space_info.posegraph->getSize()<<endl;
                // cout<<"pg cnt size: "<<free_space_info.posegraph->getSize()-temp_num<<endl;
                // vector<Vector3d> pathtoHome;
                // getPathtoHome(drone_pose, pathtoHome);
                // vis_ptr->visualize_path(pathtoHome, "/path2Home");
            }
            addSuperPoint();


            last_key_pose = drone_pose;
            key_pose_cnt++;
            // ROS_INFO("\033[1;35mFreeSapce&Frontier Generation Done Time-Consume: %f ms\033[0m",(ros::Time::now().toSec()-time0) * 1000.0);  //紫

        }else{
            

        }
        // cout<<"start pub"<<endl;
        publishFrontierClusterSurface(rostime);
        // cout<<"publishFrontierClusterSurface pub"<<endl;

        publishFrontierClusterCenterPCL(rostime);
        publishSVPCenterPCL(rostime);
        // publishFrontierClusterOval(rostime);
        if(is_viewpoint) pubViewPoints(rostime);
        // cout<<"pubViewPoints pub"<<endl;

        pubPoseEdges(rostime);
        // cout<<"pubPoseEdges pub"<<endl;

        // free_space_fac_->publishStarCvxAll(free_space_info, drone_pose, rostime,"world");
        // ROS_INFO("\033[1;35mFreeSapce&Frontier pub Done \033[0m");  //紫
    }else{
        vis_ptr->visualize_pointcloud(vec, "AAA");
    }
    // ROS_INFO("\033[1;35mFreeSapce&Frontier all Done \033[0m");  //紫
}

bool FrontierClusterInfo::isArriandDeletePass(){
    bool is_arrive = false;
    for(list<SuperViewPoint>::iterator it = svp_list.begin(); it != svp_list.end(); ){
        if(!is_on_car){
            if((Vector3d(it->super_vp.x, it->super_vp.y, it->super_vp.z) - drone_pose).norm() < svp_arrive_dis){
                it = svp_list.erase(it);
                is_arrive = true;
            }else{
                it++;
            }
        }else{
            if((Vector3d(it->super_vp.x, it->super_vp.y, drone_pose(2)) - drone_pose).norm() < svp_arrive_dis){
                it = svp_list.erase(it);
                is_arrive = true;
            }else{
                it++;
            } 
        }

    }
    return is_arrive;
}

void FrontierClusterInfo::frontierCluster(CellList& frontier_cell_set, FreeSpace& fs){
    // cout<<"frontierCluster"<<endl;
    //! 1. 当前frontier在历史freespace中剪裁    
    if(free_space_info.posegraph->getSize() >= 1){

        // The kdtree include all keypose To find nearby freespace
        // pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_keypose;
        for(CellList::iterator j_it = frontier_cell_set.begin(); j_it != frontier_cell_set.end(); ){
            bool deleted = false;
            pcl::PointXYZ searchPoint(j_it->_center(0), j_it->_center(1), j_it->_center(2));
            std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
            std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方
            if ( free_space_info.posegraph->kdtree_keypose->radiusSearch (searchPoint, 1.5*free_space_fac_->sensor_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                for (size_t k = 0; k < pointIdxRadiusSearch.size (); ++k){
                    if(pointIdxRadiusSearch[k] == free_space_info.posegraph->getSize()-1) continue;
                    if(isMeshInFreeSapce(free_space_info.free_space_list[pointIdxRadiusSearch[k]], *j_it)){
                        j_it = frontier_cell_set.erase(j_it);
                        deleted = true;
                        break;
                    }
                }
            }
            if(!deleted){
                j_it++;
            }
        }
    }


    //! 2. 会被当前freespace影响的fc，剪裁
    // TODO is_affected judge is not fine enough
    for(list<SuperViewPoint>::iterator it = svp_list.begin(); it != svp_list.end(); ){
        bool is_affected = false;
        for(vector<FrontierCluster>::iterator iter = it->fc_list.begin(); iter != it->fc_list.end(); iter++){
            if((iter->center - fs.key_pose).norm() < free_space_fac_->sensor_range + 0.1 &&
               isVisible(iter->center, fs.key_pose, 0.0)){
                is_affected = true;
                break;
            }
        }
        if(!is_affected){
            it++;
            continue;
        }
        
        for(vector<FrontierCluster>::iterator iter = it->fc_list.begin(); iter != it->fc_list.end(); ){
            for(CellList::iterator j_it = iter->cell_list.begin(); j_it != iter->cell_list.end(); j_it++)
            {
                if(!isMeshInFreeSapce(fs, *j_it))
                {
                    if (j_it->_is_large_view_angle)
                    {
                        frontier_cell_set.push_back(*j_it);
                    }
                    else if (free_space_fac_->checkMeshOcc(*j_it))
                    {
                        frontier_cell_set.push_back(*j_it);
                    }
                }
            }
            id_pool[iter->id] = 0;
            iter = it->fc_list.erase(iter);
            
        }
        id_vp_pool[it->id] = 0;
        it = eraseSuperPoint(it);
    }


    //! 3. 得到frontierset，聚类后加入fclist
    double time1 = ros::Time::now().toSec();

    // 注意，frontier_cluster_list中的mesh和freespace的mesh是不一样的，frontier_cluster_list中的mesh有center，在addcluster之前要计算center
    this->findCluster(frontier_cell_set);

    // ROS_INFO("\033[1;35mfindCluster Time-Consume: %f ms\033[0m",(ros::Time::now().toSec()-time1) * 1000.0);  //紫

}


void FrontierClusterInfo::findCluster(CellList& frontier_cell_set_){
    // cout<<"in findCluster"<<endl;
    // cout<<"frontierset size:" <<frontier_cell_set_.size()<<endl;
    if(frontier_cell_set_.size() == 0){
        return;
    }

    FrontierCluster cluster0;
    for(CellList::iterator it = frontier_cell_set_.begin(); it != frontier_cell_set_.end(); it++){
        cluster0.addCell(*it);
    }
    cluster0.update();
    if((cluster0.d_vertical_max < cluster_dv_max && cluster0.d_para_max < cluster_dp_max && cluster0.normal.norm() > cluster_normal_thr) ||
         cluster0.cell_list.size() == 1){
        PCA(cluster0);
        if(is_viewpoint) sampleViewpoints(cluster0);
        addCluster(cluster0);
        return;
    }


    FrontierCluster cluster1, cluster2;
    spectralCluster(cluster1, cluster2, frontier_cell_set_, 2);


    // publishFrontierClusterSurface(ros::Time::now(), frontier_cell_set_,100);
    // int m;
    // cin>>m;

    // publishFrontierClusterCenterPCL(ros::Time::now());
    // publishFrontierClusterSurface(ros::Time::now(),cluster1.cell_list,10);
    // publishFrontierClusterSurface(ros::Time::now(),cluster2.cell_list,1);
    // int m;
    // cin>>m;


    if((cluster1.d_vertical_max < cluster_dv_max && cluster1.d_para_max < cluster_dp_max && cluster1.normal.norm() > cluster_normal_thr) ||
         cluster1.cell_list.size() == 1){
        PCA(cluster1);
        if(is_viewpoint) sampleViewpoints(cluster1);
        addCluster(cluster1);
    }else{
        findCluster(cluster1.cell_list);
    }

    if((cluster2.d_vertical_max < cluster_dv_max && cluster2.d_para_max < cluster_dp_max && cluster2.normal.norm() > cluster_normal_thr) ||
         cluster2.cell_list.size() == 1){
        PCA(cluster2);
        if(is_viewpoint) sampleViewpoints(cluster2);
        addCluster(cluster2);
    }else{
        findCluster(cluster2.cell_list);
    }
    return;
}

void FrontierClusterInfo::addCluster(FrontierCluster& fc){
    // if(fc.d_para_max < 0.5) return; // too small omit //! ATT 
    int chosen_id = -1;
    for(size_t i = 0; i < id_pool.size(); i++){
        if(id_pool[i] == 0){
            chosen_id = i;
            id_pool[i] = 1;
            break;
        }
    }
    fc.id = chosen_id;
    // fc.id = id_cnt++;
    fc.intensity = rand() % 255;
    frontier_cluster_list.push_back(fc);
}

list<FrontierCluster>::iterator FrontierClusterInfo::eraseCluster(list<FrontierCluster>::iterator iter){
    id_pool[iter->id] = 0;
    return frontier_cluster_list.erase(iter);
}


bool FrontierClusterInfo::isMeshInFreeSapce(FreeSpace& fs, Triangle& mesh){
    // bool mesh_in_free = (fs.is_in_freesapce(mesh._center) > -1e-6);

    bool mesh_in_free = ((mesh._center - fs.key_pose).norm() < sensor_range - ftr_dis_thr_from_sense_range && 
                         fs.is_in_freesapce(mesh._center) > fmesh_in_hull_dis_thr);

    // bool mesh_in_free = (fs.is_in_freesapce(mesh._a) > 0.0) && 
    //                      (fs.is_in_freesapce(mesh._b) > 0.0) &&
    //                      (fs.is_in_freesapce(mesh._c) > 0.0); 


    // bool mesh_in_free = (fs.is_in_freesapce(mesh._center)) &&
    //     (fs.is_in_freesapce(mesh._a)) &&
    //     (fs.is_in_freesapce(mesh._b))&&
    //     (fs.is_in_freesapce(mesh._c));
    return mesh_in_free;
}

FrontierCluster FrontierClusterInfo::getFC(int id){
    for(list<FrontierCluster>::iterator iter = frontier_cluster_list.begin(); iter != frontier_cluster_list.end(); iter++){
        if(iter->id == id){
            return (*iter);
        }
    }
    FrontierCluster fc;
    fc.id = -1;
    return fc;
}

SuperViewPoint FrontierClusterInfo::getSVP(int id){
    for(list<SuperViewPoint>::iterator iter = svp_list.begin(); iter != svp_list.end(); iter++){
        if(iter->id == id){
            return (*iter);
        }
    }
    SuperViewPoint fc;
    fc.id = -1;
    return fc;
}

//! pose graph
void FrontierClusterInfo::addPoseEdge(){
    map<int, double> nearby_v_set;
    int inx2 = free_space_info.posegraph->getSize()-1;
    pcl::PointXYZ cur = free_space_info.posegraph->getCor(inx2);
    Vector3d cur_p(cur.x, cur.y, cur.z);
    free_space_info.posegraph->getPotenConnectSetSelf(inx2, nearby_v_set, 0.5*free_space_fac_->sensor_range, 2.0*free_space_fac_->sensor_range);
    for(map<int, double>::iterator it = nearby_v_set.begin(); it != nearby_v_set.end(); it++){
        pcl::PointXYZ endp = free_space_info.posegraph->getCor(it->first);
        Vector3d end_p(endp.x, endp.y, endp.z);
        if(isVisible(cur_p, end_p, 0.0)){
            free_space_info.posegraph->addPoseEdge(it->first, inx2, it->second);
        }
    }
}


/******************************************谱聚类***********************************************************/
void FrontierClusterInfo::spectralCluster(FrontierCluster& c1, FrontierCluster& c2, CellList& frontier_cell_set, int k){
    MatrixXd W;
    MatrixXd ED;
    MatrixXd D;
    MatrixXd L;
    MatrixXd U;
    double time1 = ros::Time::now().toSec();

    calSimilarMatrix(W, ED, frontier_cell_set);
    // ROS_INFO("\033[1;35mcalSimilarMatrix Time-Consume: %f ms\033[0m",(ros::Time::now().toSec()-time1) * 1000.0);  //紫

    
    vector<Vector4d> textv;
    for(size_t i = 0; i < frontier_cell_set.size(); i++){
        textv.emplace_back(frontier_cell_set[i]._center(0), frontier_cell_set[i]._center(1), frontier_cell_set[i]._center(2), i);
    }
    vis_ptr->visualize_texts(textv, 0.2, "/cell_no", visualization::blue);
    time1 = ros::Time::now().toSec();
    calDegreeMatrix(W, D);
    calLaplaceMatrix(W, D, L);
    // ROS_INFO("\033[1;35mcalLaplaceMatrix Time-Consume: %f ms\033[0m",(ros::Time::now().toSec()-time1) * 1000.0);  //紫
    time1 = ros::Time::now().toSec();
    // if(L.rows() <= 3){
    getEigenLaplace(L, U, k);
    // cout<<U<<endl;
    // cout<<"-----------------------------"<<endl;
    // // }else{
    //     getEigenLaplaceSpectra(L, U, k);
    // // }
    // cout<<U<<endl;
    // ROS_INFO("\033[1;35mgetEigenLaplace Time-Consume: %f ms\033[0m",(ros::Time::now().toSec()-time1) * 1000.0);  //紫

    // publishFrontierClusterSurface(ros::Time::now(),frontier_cell_set,10);
    vector<size_t> inx1, inx2;
    time1 = ros::Time::now().toSec();

    biKMeans(ED, U, inx1, inx2);
    // ROS_INFO("\033[1;35mbiKMeansTime-Consume: %f ms\033[0m",(ros::Time::now().toSec()-time1) * 1000.0);  //紫

    for(size_t i = 0; i < inx1.size(); i++){
        c1.addCell(frontier_cell_set[inx1[i]]);
    }
    c1.update();
    for(size_t i = 0; i < inx2.size(); i++){
        c2.addCell(frontier_cell_set[inx2[i]]);
    }
    c2.update();
}

void FrontierClusterInfo::calSimilarMatrix(MatrixXd& W, MatrixXd& ED, const CellList& frontier_cell_set){
    int N = frontier_cell_set.size();
    W = MatrixXd::Zero(N, N);
    ED = MatrixXd::Zero(N, N);

    MatrixXd W_display = MatrixXd::Zero(N, N);
    for(int i = 0; i < N-1; i++){
        for(int j = i+1; j < N; j++){
            ED(i,j) = calEulerDis(frontier_cell_set[i], frontier_cell_set[j]);
            ED(j,i) = ED(i,j);
        }
    }
    for(int i = 0; i < N-1; i++){
        for(int j = i+1; j < N; j++){
            if(isKNearest(ED, i, j) || isKNearest(ED, j, i)){
                double res = calClusterDis(frontier_cell_set[i], frontier_cell_set[j]);
                W(i,j) = exp(-(res*res)/(2*sigmasq));
                W(j,i) = W(i,j); 
                // W_display(i,j) = res;
                // W_display(j,i) = W_display(i,j); 
            }
        }
    }
    // cout<<"W_display:"<<W_display<<endl;
}

// Is Cell i the belong to the k-nearest of j
bool FrontierClusterInfo::isKNearest(const MatrixXd& dis_matrix, const int iq, const int jq){
    double dis = dis_matrix(iq,jq);
    int cnt = 0;
    for(int i = 0; i < dis_matrix.rows(); i++){
        if(i != iq && dis_matrix(i,jq) < dis){
            cnt++;
            if(cnt > k_nearest) return false;
        }
    }
    return true;
}

double FrontierClusterInfo::calClusterDis(const Triangle t1, const Triangle t2){
    double res;
    double dv = abs((t1._center-t2._center).dot(t2._normal.normalized()));
    double dp = ((t1._center-t2._center).cross(t2._normal.normalized())).norm();
    double n_diff = (t1._normal - t2._normal).norm()/2.0;
    res = dv * dp_ratio_dv + dp + n_diff * cluster_dp_max; //TODO
    // res = exp(-(res*res)/(2*sigmasq));
    return res;
}

double FrontierClusterInfo::calEulerDis(const Triangle t1, const Triangle t2){
    return (t1._center-t2._center).norm();
}

void FrontierClusterInfo::calDegreeMatrix(const MatrixXd& W, MatrixXd& D){
    int N = W.rows();
    D = MatrixXd::Zero(N, N);
    for(int i = 0; i < N; i++){
        double sum = 0;
        for (int j = 0; j < N; j++){
            sum += W(i,j);
        }
        D(i,i) = sum;
    }
}

void FrontierClusterInfo::calLaplaceMatrix(const MatrixXd& W, const MatrixXd& D, MatrixXd& L){
    int N = W.rows();
    MatrixXd D_n = D;
    for(int i = 0; i < N; i++){
        if(abs(D_n(i,i)) < 1e-10) continue;
        D_n(i,i) = 1/sqrt(D_n(i,i));
    }
    L = MatrixXd::Identity(N, N) - D_n * W * D_n;
}

void FrontierClusterInfo::getEigenLaplace(MatrixXd& L, MatrixXd& U, int k){
    const int N = L.rows();

    //TODO SelfAdjointEigenSolver精度和速度
    SelfAdjointEigenSolver<MatrixXd> es(L);
    MatrixXcd evecs = es.eigenvectors();
    MatrixXcd evals = es.eigenvalues();//获取矩阵特征值 4*1
    MatrixXd evalsReal;//注意这里定义的MatrixXd里没有c
    evalsReal=evals.real();//获取特征值实数部分
    // cout<<"indx, evalsReal "<<endl;
    // for(int i = 0; i < N; i++){
    //     cout<<i<<", "<<evalsReal(i)<<endl;
    // }

    MatrixXd evecsReal;
    evecsReal = evecs.real();


    multimap<double, int> mm;
    for(int i = 0; i < (int)evalsReal.size(); i++){
        mm.insert(pair<double, int>(evalsReal(i), i));
    }
    int i = 0;
    U.resize(N, k);
    for(auto n:mm){
        if(i >= k) break;
        U.col(i) = evecsReal.col(n.second);
        i++;
        // cout<<n.first<<endl;
    }
    // cout<<"eval:"<<endl;
    // for(auto n:mm){
    //     cout<<n.first<<endl;
    // }

    // cout<<"U: "<<endl;
    // cout<<U<<endl;
    for(int i = 0; i < N; i++){
        if(U.row(i).norm() < 1e-5){
            U.row(i) = VectorXd::Zero(k).transpose();
        }else{
            U.row(i).normalize();
        }
    }
  
}



void FrontierClusterInfo::biKMeans(const MatrixXd& ED, const MatrixXd& Y, vector<size_t>& inx1, vector<size_t>& inx2){
    int N = Y.rows();
    int k = Y.cols();
    Vector2d c1, c2;
    Vector2d pre_c1, pre_c2;
    vector<Vector2d> cluster1, cluster2;

    double max_d = 0;
    int max_ind;
    double dis;
    for(int i = 1; i < N; i++){
        dis = (Y.row(0)-Y.row(i)).norm();
        if(dis > max_d){
            max_d = dis;
            max_ind = i;
        }
    }
    c1 = Y.row(0);
    c2 = Y.row(max_ind);

    do{
        pre_c1 = c1;
        pre_c2 = c2;
        cluster1.clear();
        cluster2.clear();
        inx1.clear();
        inx2.clear();

        for(int i = 0; i < N; i++){
            double dis1 = (c1.transpose() - Y.row(i)).norm();
            double dis2 = (c2.transpose() - Y.row(i)).norm();

            VectorXd x = Y.row(i);

            if(dis1 < dis2){
                cluster1.push_back(VectorXd(Y.row(i)));
                inx1.push_back(i);
            }else{
                cluster2.push_back(VectorXd(Y.row(i)));
                inx2.push_back(i);
            }
        }
        c1 = VectorXd::Zero(k);
        for(size_t i = 0; i < cluster1.size(); i++){
            c1 = c1 + cluster1[i];
        }

        c1 = c1 / cluster1.size();

        c2 = VectorXd::Zero(k);
        for(size_t i = 0; i < cluster2.size(); i++){
            c2 = c2 + cluster2[i];
        }
        c2 = c2 / cluster2.size();

        if(cluster1.size() < 1 || cluster2.size() < 1){
            // ROS_ERROR("Frontier Cluster size == 0");
            while(1){}
        } 
    }while(c1 != pre_c1 || c2 != pre_c2);
}


/************************************视点操作***************************************************************/

void FrontierClusterInfo::sampleViewpoints(FrontierCluster& fc){
    // cout<<"in sampleViewpoints"<<endl;
    // Evaluate sample viewpoints on circles, find ones that cover most cells
    // sampleViewpointsOnGround(fc);
    // ROS_WARN_STREAM("fc.viewpoints.size(): "<<fc.viewpoints.size());
    if(fc.viewpoints.size() <= 0){
        // cout<<"in sampleViewpoints"<<endl;
        sampleViewpointsInAir(fc);
    }
    // ROS_WARN_STREAM("fc.viewpoints.size(): "<<fc.viewpoints.size());
    // if (fc.viewpoints.size() <= 0)
        // ROS_WARN_STREAM("fc.center: "<< fc.center.transpose());
}

void FrontierClusterInfo::sampleViewpointsInAir(FrontierCluster& fc){
   for (double rc = candidate_rmax_, dr = (candidate_rmax_ - candidate_rmin_) / candidate_rnum_; rc >= candidate_rmin_ - 1e-3; rc -= dr){
        double phi_s = atan2(fc.normal(1), fc.normal(0));
        double phi_range = candidate_phinum_*candidate_dphi_;
        int step = 1;
        vector<double> phi_vec;
        for (double phi = phi_s; abs(phi - phi_s) < phi_range + 1e-3; phi += candidate_dphi_*step, step = -(step+1)) { // 越到后面偏得越多
            phi_vec.push_back(phi);
        }

        for(size_t i = 0; i < phi_vec.size(); i++){
            Vector3d sample_pos = fc.center + rc * Vector3d(cos(phi_vec[i]), sin(phi_vec[i]), 0);
            //TODO magic number! lock viewpoint z height
            // sample_pos.z() = 0.4;
            // if (sample_pos.z() < 0.5) sample_pos.z() = 0.5;
            // if (sample_pos.z() > 1.0) sample_pos.z() = 1.0;
            // Qualified viewpoint is in bounding box and in safe region
            if ((drone_pose - sample_pos).norm() > free_space_fac_->sensor_range) continue;
            if(checkViewPoint(sample_pos, fc))
            {
                v5.push_back(sample_pos);
                bool isinfree = free_space_info.isPointInFreeSpace(sample_pos, 1.5*free_space_fac_->sensor_range);
                if(isinfree){ 
                    double yaw = atan2((fc.center-sample_pos)(1), (fc.center-sample_pos)(0));
                    Viewpoint vp = { sample_pos, yaw, AIR };
                    fc.viewpoints.push_back(vp);
                    break;
                }
            }
        }
        //TODO 这里只找了唯一一个viewpoint
        if(fc.viewpoints.size() > 0){
            break;
        }
    }
}


void FrontierClusterInfo::sampleViewpointsOnGround(FrontierCluster& fc){
    Viewpoint vp;
    map<double, Vector3d> candidate_viewpoints_map;
    // double phi_range = candidate_phinum_*candidate_dphi_;
    // double viewpoint_cos_thr = cos(phi_range);

    Vector3d C = fc.center; 
    pcl::PointXYZ C_p;
    C_p.x = C(0);
    C_p.y = C(1);
    C_p.z = C(2);
    Vector3d A1 = fc.center + fc.d_para_max*fc.normal_nd.normalized();
    Vector3d A2 = fc.center - fc.d_para_max*fc.normal_nd.normalized(); 

    if (grid_map_->pc_surface->points.empty()) return;

    surfacepc_kdtree_->setInputCloud(grid_map_->pc_surface);
    std::vector<int> neighbor_indices;
    std::vector<float> neighbor_sqdist;
    surfacepc_kdtree_->radiusSearch(C_p, viewpoint_gd_dis_thr, neighbor_indices, neighbor_sqdist);
    for (const auto& idx : neighbor_indices)
    {   
        pcl::PointXYZ pv = grid_map_->pc_surface->points[idx];
        Vector3d V(pv.x, pv.y, pv.z);  
        Vector3d CV = V - C;  
        if(CV.norm() < candidate_rmin_) continue;
        if(cosBetweenTwoVectors(CV, fc.normal) < viewpoint_gd_cos_thr) continue; 
        // Vector3d CA1 = A1 - C;  
        // if(cosBetweenTwoVectors(CA1, fc.normal) < viewpoint_gd_cos_thr) continue;
        // Vector3d CA2 = A2 - C;  
        // if(cosBetweenTwoVectors(CA2, fc.normal) < viewpoint_gd_cos_thr) continue;
        if(checkViewPoint(V+Vector3d(0,0,1.0), fc) == false) continue;
        double score = -cosBetweenTwoVectors(CV, fc.normal) + abs(CV.norm()-candidate_rmax_)/candidate_rmax_;
        candidate_viewpoints_map.insert({score, V});
    }
    if(candidate_viewpoints_map.size() > 0){
        vp.pos_ = candidate_viewpoints_map.begin()->second;
        vp.yaw_ = atan2((fc.center-vp.pos_)(1), (fc.center-vp.pos_)(0));
        vp.state_ = GROUND;
        // ROS_WARN_STREAM("check viewpoint: "<<vp.pos_.transpose()<<" -> fc: "<<fc.center.transpose()<<" check:"<<checkViewPoint(vp.pos_+Vector3d(0,0,1.0), fc));
        // if(checkViewPoint(vp.pos_+Vector3d(0,0,1.0), fc))
        // {
            fc.viewpoints.push_back(vp);
        // }
    }
}

double FrontierClusterInfo::cosBetweenTwoVectors(Vector3d v1, Vector3d v2){
    return v1.dot(v2)/(v1.norm()*v2.norm());
}

bool FrontierClusterInfo::checkViewPoint(Vector3d sample_p, FrontierCluster& fc)
{
    if (grid_inf_map_->getInflateOccupancy(sample_p))
        return false;

    //TODO viewpoint occ clearance
    double res = grid_inf_map_->getResolution();
    for (double dx = -res; dx <= res; dx += res)
        for (double dy = -res; dy <= res; dy += res)
    {
        Vector3d pc = sample_p;
        pc.x() = pc.x() + dx;
        pc.y() = pc.y() + dy;
        if (grid_inf_map_->getInflateOccupancy(pc))
            return false;
    }

    Eigen::Vector3d dir = sample_p - fc.center;
    if (dir.norm() <= res)
    {
        return true;
    } 
    else
    {
        Eigen::Vector3d fc_check_p = fc.center + dir.normalized() * res;
        return (isVisible(sample_p, fc_check_p, res));
    }
}

bool FrontierClusterInfo::isVisible(Vector3d p1, Vector3d p2, double check_res = 0.0){
    Vector3d p1_2_p2 = p2-p1;
    Vector3d dir = p1_2_p2.normalized();
    double dis = p1_2_p2.norm();
    

    if(check_res < grid_map_->getResolution()) check_res = grid_map_->getResolution();
    if(check_res >= dis) check_res = dis/2.0;

    // Vector3d pt = p1;
    // for(;(pt-p1).norm() < dis; pt += check_res*dir){
    //     vvvv.push_back(pt);
    //     if(grid_map_->getInflateOccupancy(pt)){
    //         return false;
    //     }
    // }
    // return true;
    Eigen::Vector3d res_p;
    return (grid_map_->isVisible(p1, p2, check_res, 1, res_p) == mapping::VISIBLE);
}




void FrontierClusterInfo::addSuperPoint(){ 
    //Add current fc that has viewpoint to superpoint
    // frontier_cluster_list is the current frame new or re-new ftr clusters
    // cout<<"addSuperPoint"<<endl;
    // ROS_WARN_STREAM("frontier_cluster_list size: "<<frontier_cluster_list.size());
    vector<int> label_picked(frontier_cluster_list.size(), 0);
    int cnt = 0;
    for(FrontierClusterList::iterator it = frontier_cluster_list.begin(); it != frontier_cluster_list.end(); cnt++, it++){
        if(it->viewpoints.size() > 0 && label_picked[cnt]==0){
            // ROS_WARN_STREAM("has viewp, pick it");
            label_picked[cnt] = 1;
            SuperViewPoint sp;

            sp.add((*it));
            Vector3d vp_t = it->viewpoints[0].pos_;
            int cntj = 0;
            // Find the other cluster whose viewpoints are nearby
            for(FrontierClusterList::iterator itj = frontier_cluster_list.begin(); itj != frontier_cluster_list.end(); cntj++, itj++){
                if(label_picked[cntj]==0 && itj->viewpoints.size()>0 && 
                   (itj->viewpoints[0].pos_- sp.getPoseActual()).norm() < this->svp_dis)
                {
                    label_picked[cntj] = 1;
                    sp.add((*itj));
                }
            }

            if(sp.id == -1){
                int chosen_id = -1;
                for(size_t i = 0; i < id_vp_pool.size(); i++){
                    if(id_vp_pool[i] == 0){
                        chosen_id = i;
                        id_vp_pool[i] = 1;
                        break;
                    }
                }
                sp.id = chosen_id;
                // sp.id = id_vp_cnt++;
            }

            // avoid recluster change the connected keypose
            sp.keypose_inx = free_space_info.posegraph->getSize() - 1 - temp_num;
            ROS_ASSERT(sp.keypose_inx<100000);

            // ROS_WARN_STREAM("now sp id: "<< sp.id<<", keyinx before change: "<<sp.keypose_inx << 
            // ", pos: " << sp.getPoseActual().transpose());
            
            // -------------- display
            // cout<<"sp.keypose_inx: "<<sp.keypose_inx<<"="<<free_space_info.posegraph->getSize()<<"-1-"<<temp_num<<endl;
            Vector3d cur_svp_pose = sp.getPoseForPlan();
            pcl::PointXYZ cur_keypose_pcl = free_space_info.posegraph->getCor(sp.keypose_inx);
            Vector3d cur_keypose(cur_keypose_pcl.x, cur_keypose_pcl.y, cur_keypose_pcl.z);

            // ROS_WARN_STREAM("isVisible? "<<isVisible(cur_keypose, cur_svp_pose, grid_map_->getResolution()));

            vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
            arrows_pair_list.emplace_back(cur_keypose, cur_svp_pose);
            vis_ptr->visualize_pairline(arrows_pair_list, "AAline", 0.1, visualization::Color::red);

            vector<Vector3d> ooo;
            ooo.push_back(cur_svp_pose);
            vis_ptr->visualize_pointcloud(ooo, "/AAAsvppose");

            ooo.clear();
            ooo.push_back(sp.fc_list[0].center);
            vis_ptr->visualize_pointcloud(ooo, "/AAAfc");
            // -------------- display

            // if cur_keypose & cur_svp_pose is not visible OR distance between them is too far -> 
            // find a new keypose (find from the erased svps' keyposes) for this svp to attach
            bool isfail = false;
            if((cur_keypose-cur_svp_pose).norm() > 1.5*free_space_fac_->sensor_range || 
                !isVisible(cur_keypose, cur_svp_pose, grid_map_->getResolution())){
                ROS_WARN_STREAM("not visible");
                SuperViewPoint closest_svp;
                double min_dis = 9999; 
                for(size_t i = 0; i < erased_svp_list.size(); i++){
                    Vector3d erased_svp = erased_svp_list[i].getPoseForPlan();
                    if((cur_svp_pose-erased_svp).norm() < min_dis){
                        min_dis = (cur_svp_pose-erased_svp).norm();
                        ROS_ASSERT(erased_svp_list[i].keypose_inx<100000);
                        closest_svp = erased_svp_list[i];
                    }
                }
                ROS_WARN_STREAM("min dis: "<<min_dis<<"| ("<<cur_svp_pose.transpose()<<" -> "<<closest_svp.getPoseForPlan()<<")");
                vvvv.clear();
                if(min_dis < 9999){ // 在重新聚类之后svp位置可能会变化
                    pcl::PointXYZ close_keypose_pcl = free_space_info.posegraph->getCor(closest_svp.keypose_inx);
                    Vector3d closest_svp_keypose(close_keypose_pcl.x, close_keypose_pcl.y, close_keypose_pcl.z);
                    sp.keypose_inx = closest_svp.keypose_inx; // find a new keypose
                    ROS_ASSERT(sp.keypose_inx<1000);
                    ROS_WARN_STREAM("keyinx after change:"<<sp.keypose_inx);

                    if(!isVisible(closest_svp_keypose, cur_svp_pose, grid_map_->getResolution()) && 
                        (cur_keypose-cur_svp_pose).norm() <= 1.5*free_space_fac_->sensor_range)
                    {
                        vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
                        arrows_pair_list.emplace_back(cur_keypose, cur_svp_pose);
                        vis_ptr->visualize_pairline(arrows_pair_list, "AAline_after", 0.1, visualization::Color::green);
                        
                        vvvv.clear();
                        isVisible(cur_keypose, cur_svp_pose, grid_map_->getResolution());
                        vis_ptr->visualize_pointcloud(vvvv,"/Avis");
                        ROS_ERROR_STREAM("svp ERROR!!!!!!!");
                        isfail = true;
                        // ROS_ASSERT(false);
                    }

                }
                // sp.id = 9999;
            }
            if(!isfail && checkSVP(sp)){
                svp_list.push_back(sp);
            }
            else if(checkSVP(sp)){
                sp.keypose_inx = free_space_info.posegraph->getSize() - 1 - temp_num; // just use the current keypose, because it is sensened at this keypose
                svp_list.push_back(sp);
            }
        }
    }
    ROS_WARN_STREAM("svp: "<<svp_list.size());
    // cout<<"addSuperPoint end"<<endl;

}

bool FrontierClusterInfo::checkSVP(SuperViewPoint& sp){
    double s_sum = 0;
    for(auto& fc:sp.fc_list){
        for(auto& mesh:fc.cell_list){
            Vector3d B = mesh._b - mesh._a;
            Vector3d C = mesh._c - mesh._a;
            double S = B.cross(C).norm() * 0.5;
            s_sum += S;
        }
    }
    if(s_sum < svp_s_thr){
        return false;
    }else{
        return true;
    }
}

void FrontierClusterInfo::refineSuperPoint(SuperViewPoint& sp){
    Vector3d c(sp.super_vp.x, sp.super_vp.y, sp.super_vp.z);
    if(true||drone_vel.norm() < 0.2){
        sp.refined_svp = c;
        return;
    }
    
    Vector3d p = drone_pose;
    Vector3d p2c = c-p;
    Vector3d vel_dir = drone_vel.normalized();
    double D = p2c.cross(vel_dir).norm();
    double v_dis = sqrt(p2c.squaredNorm()-D*D);
    Vector3d k = p + vel_dir*v_dis;
    Vector3d k2c = c-k;
    sp.refined_svp = k + k2c.normalized()*(D - svp_refine_r);
    sp.refined_svp(2) = sp.super_vp.z;
    if(grid_map_->getInflateOccupancyRoof(sp.refined_svp)){
        sp.refined_svp = c;
    }
}

list<SuperViewPoint>::iterator FrontierClusterInfo::eraseSuperPoint(list<SuperViewPoint>::iterator it){
    erased_svp_list.push_back(*it);
    // id_vp_pool[it->id] = 0;
    ROS_ASSERT(erased_svp_list.back().keypose_inx<1000);
    return svp_list.erase(it);
}

/****************************************可视化*******************************************************/

void FrontierClusterInfo::testMeshTable(FreeSpace& fs){
    vector<Vector3d> inpoints;
    vector<Vector3d> outpoints;
    vector<Vector4d> indis;

    for(int i = 0; i < 100; i++){
        for(int j =0; j < 100; j++){
            Vector3d p(-20+rand()%40, -20+rand()%40, 1.0);
            double dis = fs.is_in_freesapce(p);
            if(dis>-1e-6){
                inpoints.push_back(p);
                Vector4d textp;
                textp << p, dis; 
                indis.push_back(textp);
            }else{
                outpoints.push_back(p);
                Vector4d textp;
                textp << p, dis; 
                indis.push_back(textp);
            }
        }
    }

    // for(int i = 0; i < 100; i++){
    //     for(int j =0; j < 100; j++){
    //         Vector3d p(-20+rand()%40, -20+rand()%40, 1.0);

    //         if(!grid_map_->getInflateOccupancy(p)){
    //             inpoints.push_back(p);
    //             Vector4d textp;
    //             // textp << p, dis; 
    //             // indis.push_back(textp);
    //         }else{
    //             outpoints.push_back(p);
    //             // Vector4d textp;
    //             // textp << p, dis; 
    //             // indis.push_back(textp);
    //         }
    //     }
    // }

    vis_ptr->visualize_pointcloud(inpoints, "/debug_inpoint");
    vis_ptr->visualize_pointcloud(outpoints, "/debug_outpoint");
    // vis_ptr->visualize_texts(indis, 0.1, "/debug_dis2mesh");
    
}

//! Below is for visulization----------------------------------------------------------------------------------------
// color has 3 digital, represent RGB respectively
void FrontierClusterInfo::publishFrontierClusterSurface(const ros::Time& rostime, CellList& cell_list, int color){
    size_t octomapSize = frontier_cluster_list.size();
    // cout<<"frontier_cluster_list_size: "<<octomapSize<<endl;
    if (octomapSize < 1){
        // ROS_WARN("Nothing to publish, frontier_cluster list is empty");
        // return;
    }
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
    arrows_pair_list.clear();
    // show plane
    visualization_msgs::Marker component;
    component.header.frame_id = "world";
    component.header.stamp = rostime;
    component.ns = "xg_planes";
    component.lifetime = ros::Duration();
    component.frame_locked = true;
    component.type = visualization_msgs::Marker::TRIANGLE_LIST;
    component.action = visualization_msgs::Marker::ADD;
    component.color.r = (int)(color/100);
    component.color.g = (int)(color%100/10);
    component.color.b = (int)(color%10);
    if((int)(color/100)){
        component.ns = "xg_planes1";
    }else if((int)(color%100/10)){
        component.ns = "xg_planes1";
    }else{
        component.ns = "xg_planes2";
    }
    component.color.a = 0.5f;
    component.scale.x = 1.0;
    component.scale.y = 1.0;
    component.scale.z = 1.0;
    component.pose.position.x = 0;
    component.pose.position.y = 0;
    component.pose.position.z = 0;
    component.pose.orientation.x = 0.0;
    component.pose.orientation.y = 0.0;
    component.pose.orientation.z = 0.0;
    component.pose.orientation.w = 1.0;

        for(CellList::iterator j_it = cell_list.begin(); j_it != cell_list.end(); j_it++){
            geometry_msgs::Point p1;
            p1.x = j_it->_a(0);
            p1.y = j_it->_a(1);
            p1.z = j_it->_a(2);
            component.points.push_back(p1);
            geometry_msgs::Point p2;
            p2.x = j_it->_b(0);
            p2.y = j_it->_b(1);
            p2.z = j_it->_b(2);
            component.points.push_back(p2);
            geometry_msgs::Point p3;
            p3.x = j_it->_c(0);
            p3.y = j_it->_c(1);
            p3.z = j_it->_c(2);
            component.points.push_back(p3);
            Vector3d c_pos = j_it->_center;
            Vector3d t_pos;
            t_pos(0) = c_pos(0) + j_it->_normal(0)*0.2;
            t_pos(1) = c_pos(1) + j_it->_normal(1)*0.2;
            t_pos(2) = c_pos(2) + j_it->_normal(2)*0.2;

            arrows_pair_list.emplace_back(c_pos, t_pos);
        }

    
    frontier_cluster_pub.publish(component);
    vis_ptr->visualize_arrows(arrows_pair_list, "fc_normal",visualization::Color::red);
}

void FrontierClusterInfo::publishFrontierClusterSurface(const ros::Time& rostime){
    size_t octomapSize = svp_list.size();
    // cout<<"svp_list size: "<<octomapSize<<endl;
    if (octomapSize < 1){
        // ROS_WARN("Nothing to publish, frontier_cluster list is empty");
        return;
    }
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
    arrows_pair_list.clear();
    vector<Vector4d> textvec;
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> belong_keypose;
    
    // show plane
    visualization_msgs::Marker component;
    component.header.frame_id = "world";
    component.header.stamp = rostime;
    component.ns = "xg_planes";
    component.lifetime = ros::Duration();
    component.frame_locked = true;
    component.type = visualization_msgs::Marker::TRIANGLE_LIST;
    component.action = visualization_msgs::Marker::ADD;
    component.color.r = 0.0f;
    component.color.g = 0.0f;
    component.color.b = 1.0f;
    component.color.a = 0.5f;
    component.scale.x = 1.0;
    component.scale.y = 1.0;
    component.scale.z = 1.0;
    component.pose.position.x = 0;
    component.pose.position.y = 0;
    component.pose.position.z = 0;
    component.pose.orientation.x = 0.0;
    component.pose.orientation.y = 0.0;
    component.pose.orientation.z = 0.0;
    component.pose.orientation.w = 1.0;
    for(list<SuperViewPoint>::iterator iter = svp_list.begin(); iter != svp_list.end(); iter++){
        for(vector<FrontierCluster>::iterator it = iter->fc_list.begin(); it != iter->fc_list.end(); it++){
            for(CellList::iterator j_it = it->cell_list.begin(); j_it != it->cell_list.end(); j_it++){
                geometry_msgs::Point p1;
                p1.x = j_it->_a(0);
                p1.y = j_it->_a(1);
                p1.z = j_it->_a(2);
                component.points.push_back(p1);
                geometry_msgs::Point p2;
                p2.x = j_it->_b(0);
                p2.y = j_it->_b(1);
                p2.z = j_it->_b(2);
                component.points.push_back(p2);
                geometry_msgs::Point p3;
                p3.x = j_it->_c(0);
                p3.y = j_it->_c(1);
                p3.z = j_it->_c(2);
                component.points.push_back(p3);

                // Vector3d c_pos = j_it->_center;
                // Vector3d t_pos;
                // t_pos(0) = c_pos(0) + j_it->_normal(0)*0.2;
                // t_pos(1) = c_pos(1) + j_it->_normal(1)*0.2;
                // t_pos(2) = c_pos(2) + j_it->_normal(2)*0.2;

                // arrows_pair_list.emplace_back(c_pos, t_pos);
                // textvec.emplace_back(t_pos(0), t_pos(1), t_pos(2), j_it->_normal(2));
            }

        }
        Vector3d p1(iter->super_vp.x, iter->super_vp.y, iter->super_vp.z);
        pcl::PointXYZ p2_pcl =  free_space_info.posegraph->getCor(iter->keypose_inx);
        Vector3d p2(p2_pcl.x, p2_pcl.y, p2_pcl.z);
        belong_keypose.emplace_back(p1, p2);
    }

    frontier_cluster_pub.publish(component);
    // vis_ptr->visualize_arrows(arrows_pair_list, "mesh_normal",visualization::Color::red);
    // vis_ptr->visualize_texts(textvec, 0.2, "/debug_normal2");
    vis_ptr->visualize_pairline(belong_keypose, "belong_keypose", 0.05, visualization::Color::blue);
}

void FrontierClusterInfo::publishFrontierClusterCenterPCL(const ros::Time& rostime)
{
    size_t octomapSize = svp_list.size();
    if (octomapSize < 1){
        // ROS_WARN("Nothing to publish, frontier_cluster_center list is empty");
        return;
    }

    int marker_size = 500;

    visualization_msgs::MarkerArray landMark_array_msg;
    landMark_array_msg.markers.clear();
    frontier_center_pub.publish(landMark_array_msg);
    landMark_array_msg.markers.resize(marker_size);

    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
    arrows_pair_list.clear();
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list1;
    arrows_pair_list1.clear();
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list2;
    arrows_pair_list2.clear();
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list3;
    arrows_pair_list3.clear();
    int i = 0;
    for(list<SuperViewPoint>::iterator it = svp_list.begin(); it != svp_list.end(); it++){
        for(vector<FrontierCluster>::iterator iter = it->fc_list.begin(); iter != it->fc_list.end(); iter++,i++)
        {
            landMark_array_msg.markers[i].header.frame_id = "world";
            landMark_array_msg.markers[i].header.stamp = rostime;
            landMark_array_msg.markers[i].ns = "fc";
            landMark_array_msg.markers[i].id = i;
            // landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
            landMark_array_msg.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;

            landMark_array_msg.markers[i].pose.position.x = iter->center(0);
            landMark_array_msg.markers[i].pose.position.y = iter->center(1);
            landMark_array_msg.markers[i].pose.position.z = iter->center(2);
            landMark_array_msg.markers[i].pose.orientation.x = 0.0;
            landMark_array_msg.markers[i].pose.orientation.y = 0.0;
            landMark_array_msg.markers[i].pose.orientation.z = 0.0;
            landMark_array_msg.markers[i].pose.orientation.w = 1.0;
            landMark_array_msg.markers[i].scale.x = 1.2;
            landMark_array_msg.markers[i].scale.y = 1.2;
            // landMark_array_msg.markers[i].scale.z = 0.2;
            // landMark_array_msg.markers[i].color.a = 0.4; // Don't forget to set the alpha!
            landMark_array_msg.markers[i].scale.z = 1.8;
            landMark_array_msg.markers[i].color.a = 0.9;
            landMark_array_msg.markers[i].color.r = 1.0;
            landMark_array_msg.markers[i].color.g = 0.0;
            landMark_array_msg.markers[i].color.b = 0.0;
            ostringstream str;
            str<<iter->id;
            landMark_array_msg.markers[i].text = str.str();

            Vector3d c_pos = iter->center;
            for(CellList::iterator jit = iter->cell_list.begin(); jit != iter->cell_list.end(); jit++){
                Vector3d t_pos;
                t_pos = jit->_center;
                arrows_pair_list.emplace_back(c_pos, t_pos);
            }
            Vector3d t_pos;
            t_pos = c_pos + iter->normal.normalized()*0.8;
            arrows_pair_list1.emplace_back(c_pos, t_pos);
            t_pos = c_pos + iter->normal_nd*0.4;
            arrows_pair_list2.emplace_back(c_pos, t_pos);
            t_pos = c_pos + iter->normal_rd*0.16;
            arrows_pair_list2.emplace_back(c_pos, t_pos);
        }
    }




    for(; i<marker_size; i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "world";
        landMark_array_msg.markers[i].header.stamp = rostime;
        landMark_array_msg.markers[i].ns = "fc";
        landMark_array_msg.markers[i].id = i;
        // landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::DELETE;

    }
    frontier_center_pub.publish(landMark_array_msg);
    vis_ptr->visualize_pairline(arrows_pair_list, "fc_cenection",0.02, visualization::Color::blue);
    vis_ptr->visualize_arrows(arrows_pair_list1, "fc_normal", visualization::Color::blue);
    vis_ptr->visualize_arrows(arrows_pair_list2, "fc_normal_nd", visualization::Color::blue);
    vis_ptr->visualize_arrows(arrows_pair_list3, "fc_normal_rd", visualization::Color::blue);



}


void FrontierClusterInfo::publishSVPCenterPCL(const ros::Time& rostime)
{
    int marker_size = 200;

    visualization_msgs::MarkerArray landMark_array_msg;
    landMark_array_msg.markers.clear();
    svp_center_pub.publish(landMark_array_msg);
    landMark_array_msg.markers.resize(marker_size);

    int i = 0;
    for(list<SuperViewPoint>::iterator it = svp_list.begin(); it != svp_list.end(); it++,i++){
            landMark_array_msg.markers[i].header.frame_id = "world";
            landMark_array_msg.markers[i].header.stamp = rostime;
            landMark_array_msg.markers[i].ns = "svp";
            landMark_array_msg.markers[i].id = i;
            // landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
            landMark_array_msg.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;

            landMark_array_msg.markers[i].pose.position.x = it->super_vp.x;
            landMark_array_msg.markers[i].pose.position.y = it->super_vp.y;
            landMark_array_msg.markers[i].pose.position.z = it->super_vp.z;
            landMark_array_msg.markers[i].pose.orientation.x = 0.0;
            landMark_array_msg.markers[i].pose.orientation.y = 0.0;
            landMark_array_msg.markers[i].pose.orientation.z = 0.0;
            landMark_array_msg.markers[i].pose.orientation.w = 1.0;
            if(car_id == 0) {
                landMark_array_msg.markers[i].scale.x = 0.7;
                landMark_array_msg.markers[i].scale.y = 0.7;
                // landMark_array_msg.markers[i].scale.z = 0.2;
                // landMark_array_msg.markers[i].color.a = 0.4; // Don't forget to set the alpha!
                landMark_array_msg.markers[i].scale.z = 0.9;
                landMark_array_msg.markers[i].color.a = 0.9;
                landMark_array_msg.markers[i].color.r = 0.0;
                landMark_array_msg.markers[i].color.g = 1.0;
                landMark_array_msg.markers[i].color.b = 0.2;
            } else if(car_id == 1) {
                landMark_array_msg.markers[i].scale.x = 0.7;
                landMark_array_msg.markers[i].scale.y = 0.7;
                // landMark_array_msg.markers[i].scale.z = 0.2;
                // landMark_array_msg.markers[i].color.a = 0.4; // Don't forget to set the alpha!
                landMark_array_msg.markers[i].scale.z = 0.9;
                landMark_array_msg.markers[i].color.a = 0.9;
                landMark_array_msg.markers[i].color.r = 1.0; 
                landMark_array_msg.markers[i].color.g = 0.2; 
                landMark_array_msg.markers[i].color.b = 0.0; 
            }

            ostringstream str;
            str<<it->id;
            landMark_array_msg.markers[i].text = str.str();
    }




    for(; i<marker_size; i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "world";
        landMark_array_msg.markers[i].header.stamp = rostime;
        landMark_array_msg.markers[i].ns = "svp";
        landMark_array_msg.markers[i].id = i;
        // landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::DELETE;

    }
    svp_center_pub.publish(landMark_array_msg);

}


void FrontierClusterInfo::publishFrontierClusterOval(const ros::Time& rostime)
{
    size_t octomapSize = frontier_cluster_list.size();
    if (octomapSize < 1){
        // ROS_WARN("Nothing to publish, frontier_cluster_center list is empty");
        // return;
    }

    int marker_size = 200;

    visualization_msgs::MarkerArray landMark_array_msg;
    landMark_array_msg.markers.clear();
    frontier_oval_pub.publish(landMark_array_msg);
    landMark_array_msg.markers.resize(marker_size);

    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
    arrows_pair_list.clear();

    int i = 0;
    for(list<FrontierCluster>::iterator iter = frontier_cluster_list.begin(); iter != frontier_cluster_list.end(); iter++,i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "world";
        landMark_array_msg.markers[i].header.stamp = rostime;
        landMark_array_msg.markers[i].ns = "fcoval";
        landMark_array_msg.markers[i].id = i;
        // landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::ADD;

        landMark_array_msg.markers[i].pose.position.x = iter->center(0);
        landMark_array_msg.markers[i].pose.position.y = iter->center(1);
        landMark_array_msg.markers[i].pose.position.z = iter->center(2);

        Matrix3d R;
        R << iter->normal_nd.normalized(), iter->normal_rd.normalized(), iter->normal.normalized();
        
        Quaterniond q;
        q = R;
        q = q.normalized();

        landMark_array_msg.markers[i].pose.orientation.x = q.x();
        landMark_array_msg.markers[i].pose.orientation.y = q.y();
        landMark_array_msg.markers[i].pose.orientation.z = q.z();
        landMark_array_msg.markers[i].pose.orientation.w = q.w();

        landMark_array_msg.markers[i].scale.x = iter->d_para_max * 2.0;
        landMark_array_msg.markers[i].scale.y = iter->d_para_max/iter->normal_nd.norm() * 2.0;
        // landMark_array_msg.markers[i].scale.z = 0.2;
        // landMark_array_msg.markers[i].color.a = 0.4; // Don't forget to set the alpha!
        landMark_array_msg.markers[i].scale.z = 0.02;
        landMark_array_msg.markers[i].color.a = 0.4;
        landMark_array_msg.markers[i].color.r = 1.0;
        landMark_array_msg.markers[i].color.g = 0.0;
        landMark_array_msg.markers[i].color.b = 0.0;

        Vector3d c_pos = iter->center;
        for(CellList::iterator jit = iter->cell_list.begin(); jit != iter->cell_list.end(); jit++){
            Vector3d t_pos;
            t_pos = jit->_center;
            arrows_pair_list.emplace_back(c_pos, t_pos);
        }
    }

    for(int i=frontier_cluster_list.size(); i<marker_size; i++)
    {
        landMark_array_msg.markers[i].header.frame_id = "world";
        landMark_array_msg.markers[i].header.stamp = rostime;
        landMark_array_msg.markers[i].ns = "fcoval";
        landMark_array_msg.markers[i].id = i;
        // landMark_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
        landMark_array_msg.markers[i].type = visualization_msgs::Marker::CYLINDER;
        landMark_array_msg.markers[i].action = visualization_msgs::Marker::DELETE;

    }
    frontier_oval_pub.publish(landMark_array_msg);
    // vis_ptr->visualize_pairline(arrows_pair_list, "fc_cenection",0.02, visualization::Color::red);

}


void FrontierClusterInfo::pubViewPoints(const ros::Time& rostime){
    if(svp_list.size() < 1) return;
    vector<Vector4d> svp_vec;
    vector<Vector4d> viewpoints;
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> arrows_pair_list;
    for(auto svp:svp_list){
        if(svp.state == GROUND)
        {
            svp_vec.emplace_back(svp.super_vp.x, svp.super_vp.y, svp.super_vp.z, 0);
        }else
        {
            svp_vec.emplace_back(svp.super_vp.x, svp.super_vp.y, svp.super_vp.z, 255);
        }
        for(auto fc:svp.fc_list){
            if(fc.viewpoints.empty()) continue;
            Viewpoint& vp = fc.viewpoints[0];
            if(vp.state_ == GROUND){
                viewpoints.emplace_back(vp.pos_(0), vp.pos_(1), vp.pos_(2), 0);
            }
            else{
                viewpoints.emplace_back(vp.pos_(0), vp.pos_(1), vp.pos_(2), 255);
            }
            arrows_pair_list.emplace_back(fc.center, fc.viewpoints[0].pos_);
        }
    }

    if(viewpoints.size() > 0){
        vis_ptr->visualize_pointcloud_intensity(viewpoints, "viewpoints");
        vis_ptr->visualize_pairline(arrows_pair_list, "viewpoints_line", 0.04, visualization::Color::blue);
    }
    vis_ptr->visualize_pointcloud_intensity(svp_vec, "svp");
}

void FrontierClusterInfo::pubPoseEdges(const ros::Time& rostime){
    vector<Vector3d> poses;
    vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> edges;
    for(int i = 0; i < free_space_info.posegraph->getSize(); i++){
        Vector3d startp(free_space_info.posegraph->getCor(i).x, free_space_info.posegraph->getCor(i).y, free_space_info.posegraph->getCor(i).z);
        poses.push_back(startp);

        Vector3d endp;
        pcl::PointXYZ p_res;
        if(i > 0){
            p_res = free_space_info.posegraph->getCor(i-1);
            endp << p_res.x, p_res.y, p_res.z;
            edges.emplace_back(startp, endp);
        }
        if(i < free_space_info.posegraph->getSize()-1){
            p_res = free_space_info.posegraph->getCor(i+1);
            endp << p_res.x, p_res.y, p_res.z;
            edges.emplace_back(startp, endp);
        }

        set<Edge> res_set;
        free_space_info.posegraph->getEdge(i, res_set);
        for(auto edge:res_set){
            p_res = free_space_info.posegraph->getCor(edge.v_inx);
            endp << p_res.x, p_res.y, p_res.z;
            edges.emplace_back(startp, endp);
        }
    }
    vis_ptr->visualize_pointcloud(poses, "keyposes");
    vis_ptr->visualize_pairline(edges, "keyedges", 0.06);
}