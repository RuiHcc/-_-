#include "FreeSpace.h"
#include "Cluster.h"

/****************FreeSpace的成员函数********************************************************************************/
FreeSpace::FreeSpace(){
    convex_points.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void FreeSpace::createMeshTable(){
    mesh_table.createTable(meshes);
    is_convex_calculated = true;
}

void FreeSpace::create_convexhull(std::vector<Vector3d>& v, std::vector<size_t>& inxs){
    if(is_convex_calculated){
        ROS_ERROR("error! multicreate starconvex");
    }
	FastLab::Corridor builder(convex_points, 500.0, 0.02);
	pcl::PointXYZ cur_origin;
    cur_origin.x = key_pose(0);
    cur_origin.y = key_pose(1);
    cur_origin.z = key_pose(2);
    // std::vector<pcl::PointXYZ> v;
    v.clear();
	builder.build(meshes,v, inxs, cur_origin, 1); // 1 - starvex; 0 - convex

    meshes.clear();
    convex_points->clear();
    for(auto& vp:v){
        pcl::PointXYZ p;
        p.x = vp(0);
        p.y = vp(1);
        p.z = vp(2);
        convex_points->push_back(p);
    }

    v.clear();
    inxs.clear();
	FastLab::Corridor builder2(convex_points, 500.0, 0.02);
	builder2.build(meshes,v, inxs, cur_origin, 1); // 1 - starvex; 0 - convex

	for(size_t i = 0; i < meshes.size(); i++){
        meshes[i]._a.x -= key_pose(0);
        meshes[i]._a.y -= key_pose(1);
        meshes[i]._a.z -= key_pose(2);
        meshes[i]._b.x -= key_pose(0);
        meshes[i]._b.y -= key_pose(1);
        meshes[i]._b.z -= key_pose(2);
        meshes[i]._c.x -= key_pose(0);
        meshes[i]._c.y -= key_pose(1);
        meshes[i]._c.z -= key_pose(2);
		meshes[i].calcuNormal();
	}
    mesh_table.createTable(meshes);
    is_convex_calculated = true;
}

void FreeSpace::get_convexhull(std::vector<Vector3d>& v, std::vector<size_t>& inxs){
    if(is_convex_calculated) return;
	FastLab::Corridor builder(convex_points, 500.0, 0.02);
	pcl::PointXYZ cur_origin;
    cur_origin.x = key_pose(0);
    cur_origin.y = key_pose(1);
    cur_origin.z = key_pose(2);
    // std::vector<pcl::PointXYZ> v;
	builder.build(meshes,v, inxs, cur_origin, 1); // 1 - starvex; 0 - convex
	for(size_t i = 0; i < meshes.size(); i++){
        meshes[i]._a.x -= key_pose(0);
        meshes[i]._a.y -= key_pose(1);
        meshes[i]._a.z -= key_pose(2);
        meshes[i]._b.x -= key_pose(0);
        meshes[i]._b.y -= key_pose(1);
        meshes[i]._b.z -= key_pose(2);
        meshes[i]._c.x -= key_pose(0);
        meshes[i]._c.y -= key_pose(1);
        meshes[i]._c.z -= key_pose(2);
		meshes[i].calcuNormal();
	}
    mesh_table.createTable(meshes);
    is_convex_calculated = true;
}

double FreeSpace::is_in_freesapce(Vector3d p_query){
    if(!is_convex_calculated){
        std::vector<Vector3d> v;
        std::vector<size_t> inxs;
        get_convexhull(v, inxs);
        is_convex_calculated = true;
    }
	pcl::PointXYZ cur_origin;
    cur_origin.x = key_pose(0);
    cur_origin.y = key_pose(1);
    cur_origin.z = key_pose(2);
    return mesh_table.isInStarHull(meshes, cur_origin, p_query-key_pose);
}


/****************FreeSpaceInfo的成员函数********************************************************************************/

bool FreeSpaceInfo::isPointInFreeSpace(Vector3d& searchPoint, double search_R){
    pcl::PointXYZ pq;
    pq.x = searchPoint(0);
    pq.y = searchPoint(1);
    pq.z = searchPoint(2);
    bool isinfree = false;
    std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
    std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方
    if ( posegraph->kdtree_keypose->radiusSearch(pq, search_R, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (size_t k = 0; k < pointIdxRadiusSearch.size(); ++k){
            if(free_space_list[pointIdxRadiusSearch[k]].is_in_freesapce(searchPoint) > 0.0 - 1e-1){
                isinfree = true;
                break;
            }
        }
    }
    return isinfree;
}

bool FreeSpaceInfo::isPointsInFreeSpace(vector<Vector3d>& searchPoints, double search_R){
    pcl::PointXYZ pq;
    pq.x = searchPoints[0](0);
    pq.y = searchPoints[0](1);
    pq.z = searchPoints[0](2);
    vector<bool> isinfree;
    isinfree.resize(searchPoints.size(), false);
    std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
    std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方
    if ( posegraph->kdtree_keypose->radiusSearch(pq, search_R, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (size_t k = 0; k < pointIdxRadiusSearch.size(); ++k){
            for(size_t i = 0; i < searchPoints.size(); i++){
                if(free_space_list[pointIdxRadiusSearch[k]].is_in_freesapce(searchPoints[i]) > 0.0 - 1e-1){
                    isinfree[i] = true;
                }
            }

        }
    }
    bool allfree = false;
    for(size_t i = 0; i < searchPoints.size(); i++){
        if(isinfree[i] == false){
            allfree = false;
            break;
        }
    }
    return allfree;
}

/****************FreeSpaceFactory的成员函数********************************************************************************/

FreeSpaceFactory::FreeSpaceFactory(ros::NodeHandle& nh, 
    std::shared_ptr<FrontierClusterInfo> fc_info_ptr):nh(nh), fc_info_ptr_(fc_info_ptr){
    
    nh.param("FOV_R", sensor_range, 5.0);
    nh.param("freespace/d_theta",d_theta, 0.5);
    nh.param("freespace/res", res, 0.1);
    nh.param("freespace/res_z", res_z, 0.1);
    nh.param("frontierq/drone_id", drone_id, -1);
    nh.param("frontierq/drone_num", drone_num, -1);
    nh.param("freespace/d_thr", d_thr, 2.0);
    nh.param("freespace/ftr_dis_thr_from_sense_range", ftr_dis_thr_from_sense_range, 2.0);
    nh.param("freespace/theta_thr", theta_thr, 30.0/180.0*M_PI);
    nh.param("freespace/frontier_roof", frontier_roof, 2.0);
    nh.param("freespace/frontier_floor", frontier_floor, 0.1);
    nh.param("freespace/frontier_delta_roof", frontier_delta_roof, 2.0);
    nh.param("freespace/frontier_delta_floor", frontier_delta_floor, 0.1);
    nh.param("freespace/lable_frontier_normal_thr", lable_frontier_normal_thr, 0.8);

    nh.param("fmesh/fmesh_scale_thr", this->fmesh_scale_thr, 1.0);
    nh.param("fmesh/fmesh_area_thr", this->fmesh_area_thr, 1.0);
    nh.param("fmesh/fmesh_view_angle_thr", this->fmesh_view_angle_thr, 1.0);
    nh.param("fmesh/frontier_margin", this->frontier_margin, 1.0);
    nh.param("fmesh/fmesh_raycast_dis", this->fmesh_raycast_dis, 1.0);

	star_cvx_plane_pub_ = nh.advertise<visualization_msgs::Marker>("/star_cvx/polyhedron_plane", 1);
	star_cvx_edge_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/star_cvx/polyhedron_edge", 1);
	star_cvx_edge_all_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/star_cvx/polyhedron_edge_all", 1);

	star_cvx_f_plane_pub_ = nh.advertise<visualization_msgs::Marker>("/star_cvx/Frontier/polyhedron_plane", 1);
	star_cvx_f_edge_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/star_cvx/Frontier/polyhedron_edge", 1);

    free_space_pub = nh.advertise<frontier_space::FreeSpaceInfoMsg>("/free_space_info", 1);
}

// TODO Can do better!
FreeSpace FreeSpaceFactory::genFreeSpcePoints(Vector3d key_pose){
    FreeSpace fs;
    fs.key_pose = key_pose;
    
    vector<Vector3d> convex_points_vec;
    convex_points_vec.clear();

    vector<double> set_R;
    set_R.push_back(sensor_range);
    set_R.push_back(sensor_range/2.0);
    set_R.push_back(sensor_range/4.0);
    // set_R.push_back(sensor_range/8.0);

    vector<int> before_size;
    before_size.push_back(0);
    for(size_t i = 0; i < set_R.size(); i++){
        genConvexPointOnR(convex_points_vec, set_R[i], before_size[i], key_pose);
        before_size.push_back(convex_points_vec.size());
    }

    double max_z = -9999;
    double min_z = 9999;
    for(size_t i = 0; i < convex_points_vec.size(); i++){
        bool iscon = false;
        for(size_t j = 1; j < before_size.size(); j++){
            if(i >= before_size[j] && (convex_points_vec[i]-key_pose).norm() > set_R[j]-0.5){
                iscon = true;
                break;
            }
        }
        if(iscon) continue;
        pcl::PointXYZ tp;
        tp.x = convex_points_vec[i](0);
        tp.y = convex_points_vec[i](1);
        tp.z = convex_points_vec[i](2);
        fs.convex_points->push_back(tp);
        if(tp.z > max_z) max_z = tp.z;
        if(tp.z < min_z) min_z = tp.z;
    }

    // Add an up & buttom point
    pcl::PointXYZ p1;
    p1.x = key_pose(0);
    p1.y = key_pose(1);
    p1.z = max((key_pose(2) + 1e-3), max_z);
    pcl::PointXYZ p2;
    p2.x = key_pose(0);
    p2.y = key_pose(1);
    p2.z = min((key_pose(2) - 1e-3), min_z);
    fs.convex_points->push_back(p1);
    fs.convex_points->push_back(p2);

    return fs;
}

void FreeSpaceFactory::genConvexPointOnR(vector<Vector3d>& convex_points_vec, double set_R, int before_size, Vector3d key_pose){
    Vector3d p1 = key_pose;
    double floor_thr = std::max(key_pose(2) + frontier_delta_floor, frontier_floor);
    double roof_thr  = std::min(key_pose(2) + frontier_delta_roof,  frontier_roof);

    // ROS_INFO_STREAM("frontier_floor: " << frontier_floor << ", frontier_roof: " << frontier_roof);
    // ROS_INFO_STREAM("floor_thr: " << floor_thr << ", roof_thr: " << roof_thr);
    // ROS_INFO_STREAM("getMinBound: " << grid_map_->getMinBound().transpose() << ", getMaxBound: " << grid_map_->getMaxBound().transpose());
    // ROS_INFO_STREAM("getMinBound: " << grid_inf_map_->getMinBound().transpose() << ", getMaxBound: " << grid_inf_map_->getMaxBound().transpose());

    for(double z = floor_thr; z <= roof_thr; z+=res_z){
        for(double theta = 0.0; theta < 2*M_PI; theta += d_theta){
            Vector3d res_p;
            Vector3d p2;
            p2(0) = key_pose(0) + cos(theta)*set_R;
            p2(1) = key_pose(1) + sin(theta)*set_R;
            p2(2) = z;
            res_p = p2;

            // Vector3d p1_2_p2 = p2-p1;
            // Vector3d dir = p1_2_p2.normalized();
            // Vector3d pt = p1 + res*dir;
            // for(;(pt-p1).norm() < set_R; pt += res*dir){
                //     if(grid_map_->getInflateOccupancy(pt)){
                    //         res_p = pt;
            //         break;
            //     }
            // }
            if (grid_map_->isVisible(p1, p2, res, 1, res_p) == mapping::VISIBLE)
            {
                res_p = p2;
            }
            
            double dis2key = (res_p-key_pose).norm(); // res_p is the obstancle point or maxrange point
            if(dis2key < 0.3) continue;

            if(convex_points_vec.size() > before_size){
                extendPoints(convex_points_vec, convex_points_vec.back(), res_p, key_pose, set_R);
            }else{
                convex_points_vec.push_back(res_p);
            }
        }
    }
}

void FreeSpaceFactory::extendPoints(vector<Vector3d>& point_vec, Vector3d p1, Vector3d p2, Vector3d keypose, double sensorrange){
    double r1 = (p1 - keypose).norm();
    double r2 = (p2 - keypose).norm();
    double r_max = max(r1, r2);
    double r_min = min(r1, r2);
    double dis12 = (p1-p2).norm();
    
    if(dis12 < d_thr){
        point_vec.push_back(p2);
        return;
    }
    
    double cosangle = (p1-keypose).dot(p2-keypose)/((p1-keypose).norm()*(p2-keypose).norm());
    double angle = acos(cosangle);

    if(sin(angle)*(r_min/1.0)/dis12 < sin(theta_thr)){
        point_vec.push_back(p2);
        return;
    }
    Vector3d dir = ((p1-keypose).normalized()+(p2-keypose).normalized()).normalized();
    Vector3d pt = keypose + res*dir;
    Vector3d p_max = keypose + sensorrange*dir;
    Vector3d res_p = p_max;
    // for(;(pt-keypose).norm() < sensorrange; pt += res*dir){
        //     if(grid_map_->getInflateOccupancy(pt)){
            //         res_p = pt;
            //         break;
        //     }
    // }
    if (grid_map_->isVisible(pt, p_max, res, 1, res_p) == mapping::VISIBLE)
    {
        res_p = p_max;
    }
    extendPoints(point_vec, p1, res_p, keypose, sensorrange);
    extendPoints(point_vec, res_p, p2, keypose, sensorrange);
}


void FreeSpaceFactory::deleteInvalidMeshes(FreeSpaceInfo& free_space_info_input, FreeSpace& fs, Vector3d& keypose){
    if(!free_space_info_input.isPointInFreeSpace(keypose, 1.5*sensor_range)) return;
    for(auto it = fs.meshes.begin(); it != fs.meshes.end();){
        Vector3d pa(it->_a.x,it->_a.y,it->_a.z);
        pa = pa + keypose;
        Vector3d pb(it->_b.x,it->_b.y,it->_b.z);
        pb = pb + keypose;
        Vector3d pc(it->_c.x,it->_c.y,it->_c.z);
        pc = pc + keypose;
        vector<Vector3d> points_q;
        points_q.push_back(pa);
        points_q.push_back(pb);
        points_q.push_back(pc);
        if(free_space_info_input.isPointsInFreeSpace(points_q, sensor_range)){
            it = fs.meshes.erase(it);
        }else{
            it++;
        }
    }
}



struct hash_pair{
    template <class T1, class T2>
    size_t operator()(const pair<T1, T2>& p) const{
        auto hash1 = hash<T1>{}(p.first);
        auto hash2 = hash<T2>{}(p.second);
        return hash1^hash2;
    }
};

void FreeSpaceFactory::LableFrontierSurface(FreeSpace& fs, std::vector<Vector3d>& v, std::vector<size_t>& inxs, Vector3d& keypose,
                                            CellList& frontier_cell_set)
{
    // ROS_ERROR("in LableFrontierSurface");
    cout << "v size: " << fs.meshes.size() << endl;
    int ii = 0, i3 = 0, i4 = 0, i5 = 0;
    
    //! 0. build edge and unordered_map
    // unordered_map<pair<size_t,size_t>, pair<size_t, bool>, hash_pair> um;
    fs.mesh_is_frontier.resize(fs.meshes.size(), false);
    for(size_t m = 0; m < fs.meshes.size(); m++){
        auto mesh_cur = fs.meshes[m]; 
        // if the mesh is connected to robot -> not frontier
        if(abs(mesh_cur._a.x)+abs(mesh_cur._a.y) < 1e-2 || 
            abs(mesh_cur._b.x)+abs(mesh_cur._b.y) < 1e-2 || 
            abs(mesh_cur._c.x)+abs(mesh_cur._c.y) < 1e-2 )
        {
            ii++;
            fs.mesh_is_frontier[m] = false;
            continue;
        }
        // if the mesh is pron to z axis -> not frontier
        if(abs(mesh_cur._normal.normalized()(2)) > lable_frontier_normal_thr){
            i3++;
            fs.mesh_is_frontier[m] = false;
            continue; 
        }
        // if the mesh is out of map -> not frontier
        Vector3d pa(mesh_cur._a.x,mesh_cur._a.y,mesh_cur._a.z);
        pa = pa + keypose;
        Vector3d pb(mesh_cur._b.x,mesh_cur._b.y,mesh_cur._b.z);
        pb = pb + keypose;
        Vector3d pc(mesh_cur._c.x,mesh_cur._c.y,mesh_cur._c.z);
        pc = pc + keypose;

        // ROS_ERROR_STREAM("mesh: " << m << " pa: " << pa << " pb: " << pb << " pc: " << pc << endl);
        // ROS_ERROR_STREAM("grid_map_->isInGlobalMap(pa): " << grid_map_->isInGlobalMap(pa) << endl);
        // ROS_ERROR_STREAM("grid_map_->isInGlobalMap(pb): " << grid_map_->isInGlobalMap(pb) << endl);
        // ROS_ERROR_STREAM("grid_map_->isInGlobalMap(pc): " << grid_map_->isInGlobalMap(pc) << endl);
        // ROS_ERROR_STREAM("grid_map_->map_max_boundary_: " << grid_map_->map_max_boundary_ << endl);
        // ROS_ERROR_STREAM("grid_map_->map_min_boundary_: " << grid_map_->map_min_boundary_ << endl);


         if(!grid_map_->isInGlobalMap(pa) || 
            !grid_map_->isInGlobalMap(pb) ||
            !grid_map_->isInGlobalMap(pc) )
         {
             i4++;
             fs.mesh_is_frontier[m] = false;
             continue;
         }

        // if one of the mesh edge is FrontierEdge (one point of the edge is close to sensor range) -> frontier
        pair<size_t, size_t> edge1(inxs[m*3], inxs[m*3 + 1]);
        pair<size_t, size_t> edge2(inxs[m*3], inxs[m*3 + 2]);
        pair<size_t, size_t> edge3(inxs[m*3 + 1], inxs[m*3 + 2]);

        if(isFrontierEdge(v[edge1.first], v[edge1.second], fs.key_pose)){
            fs.mesh_is_frontier[m] = true;
            // continue; 
        }
        if(isFrontierEdge(v[edge2.first], v[edge2.second], fs.key_pose)){
            fs.mesh_is_frontier[m] = true;
            // continue;
        }
        if(isFrontierEdge(v[edge3.first], v[edge3.second], fs.key_pose)){
            fs.mesh_is_frontier[m] = true;
            // continue; 
        }

        // will calculate the center and normal of the mesh
        // and trasit it's position to the world frame
        Triangle tri(mesh_cur, fs.key_pose, this->fmesh_view_angle_thr); 

        // if the view angle of the mesh is large -> frontier
        if (tri._is_large_view_angle)
        {
            i5++;
            fs.mesh_is_frontier[m] = true;
        }
        // if the view angle of the mesh is small (face the mesh) -> check if the ftr mesh in occ
        else if (fs.mesh_is_frontier[m])
        {
            if (!checkMeshOcc(tri)){
            
                fs.mesh_is_frontier[m] = false;
            }
        }

        // Check the size
        if (fs.mesh_is_frontier[m])
        {
            // ii++;
            if (checkMeshSize(tri))
            {
                frontier_cell_set.push_back(tri);
                fs.mesh_is_frontier[m] = true;
            }
            else
            {
                fs.mesh_is_frontier[m] = false;
            }
        }
    }
    // cout << "frontier_cell_set size: " << frontier_cell_set.size() << endl;
    // cout << "mesh_is_frontier size: " << ii << ";" << i3 << ";" << i4 << ";" << i5 << ";" << endl;
    // cout << "fmesh_area_thr: " << fmesh_area_thr << endl;
    // ROS_ERROR("in LableFrontierSurface done");

}

bool FreeSpaceFactory::checkMeshSize(Triangle& tri)
{
    if((tri._a-tri._center).norm() < this->fmesh_scale_thr || 
       (tri._b-tri._center).norm() < this->fmesh_scale_thr || 
       (tri._c-tri._center).norm() < this->fmesh_scale_thr)
    {
        return false;
    }
    // 
    if(tri._center(2) <= this->frontier_margin) return false;
    if(tri._center(2) >= map_size(2) - this->frontier_margin) return false;

    Vector3d B = tri._b - tri._a;
    Vector3d C = tri._c - tri._a;
    double S = B.cross(C).norm() * 0.5;
    if(S < this->fmesh_area_thr){
        return false;
    }
    return true;
}

bool FreeSpaceFactory::checkMeshOcc(Triangle& tri)
{
    //查询normal反方向的localmap是否是障碍
    bool notocc  = true;
    for(double query_len = 0.0; query_len <= this->fmesh_raycast_dis; query_len += grid_map_->getResolution()){
        Vector3d p_q = tri._center - query_len*tri._normal;
        if(grid_map_->getInflateOccupancy(p_q)){
            notocc = false;
            break;
        }
    }
    return notocc;
}

bool FreeSpaceFactory::isFrontierEdge(Vector3d &ep1, Vector3d &ep2,Vector3d &keypose){
    double r1 = (ep1 - keypose).norm();
    double r2 = (ep2 - keypose).norm();
    double r_max = max(r1, r2);
    double r_min = min(r1, r2);
    double dis12 = (ep1-ep2).norm();
    // cout<<"sensor_range*0.9"<<sensor_range*0.9<<endl;
    if(r_max >= sensor_range - ftr_dis_thr_from_sense_range) return true;
    if(dis12 > d_thr) return true;
    return false;
}


/****************可视化函数********************************************************************************/
void FreeSpaceFactory::publishStarCvx( FreeSpace &fs,Vector3d key_pose,
                     ros::Time msg_time, std::string msg_frame_id )
{
  // show plane
  std::vector<FastLab::Triangle> &meshes_in = fs.meshes;
  visualization_msgs::Marker component;
  component.header.frame_id = msg_frame_id;
  component.header.stamp = msg_time;
  component.ns = "xg_planes";
  component.lifetime = ros::Duration();
  component.frame_locked = true;
  component.type = visualization_msgs::Marker::TRIANGLE_LIST;
  component.action = visualization_msgs::Marker::ADD;
  component.color.r = 1.0f;
  component.color.g = 0.0f;
  component.color.b = 0.0f;
  component.color.a = 0.46f;
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

  // show point and edges
  visualization_msgs::MarkerArray markerArray;
  //  nodes
  visualization_msgs::Marker markerNode;
  markerNode.header.frame_id = msg_frame_id;
  markerNode.header.stamp = msg_time;
  markerNode.action = visualization_msgs::Marker::ADD;
  markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNode.ns = "xg_nodes";
  markerNode.id = 0;
  markerNode.pose.orientation.w = 1;
  markerNode.scale.x = 0.05; markerNode.scale.y = 0.05; markerNode.scale.z = 0.05; 
  markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
  markerNode.color.a = 1;
  //  edges
  visualization_msgs::Marker markerEdge;
  markerEdge.header.frame_id = msg_frame_id;
  markerEdge.header.stamp = msg_time;
  markerEdge.action = visualization_msgs::Marker::ADD;
  markerEdge.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge.ns = "xg_edges";
  markerEdge.id = 1;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.01; markerEdge.scale.y = 0.01; markerEdge.scale.z = 0.01;
  markerEdge.color.r = 1.0f; markerEdge.color.g = 0.0; markerEdge.color.b = 0.0;
  markerEdge.color.a = 0.6;

  // push point
  for (unsigned int i = 0; i<meshes_in.size(); i++) 
  {
    // if(fs.mesh_is_frontier[i]) continue;
    geometry_msgs::Point p1;
    p1.x = meshes_in[i]._a.x + key_pose(0);
    p1.y = meshes_in[i]._a.y + key_pose(1);
    p1.z = meshes_in[i]._a.z + key_pose(2);
    component.points.push_back(p1);
    geometry_msgs::Point p2;
    p2.x = meshes_in[i]._b.x + key_pose(0);
    p2.y = meshes_in[i]._b.y + key_pose(1);
    p2.z = meshes_in[i]._b.z + key_pose(2);
    component.points.push_back(p2);
    geometry_msgs::Point p3;
    p3.x = meshes_in[i]._c.x + key_pose(0);
    p3.y = meshes_in[i]._c.y + key_pose(1);
    p3.z = meshes_in[i]._c.z + key_pose(2);
    component.points.push_back(p3);

    markerNode.points.push_back(p1);
    markerNode.points.push_back(p2);
    markerNode.points.push_back(p3);

    markerEdge.points.push_back(p1);
    markerEdge.points.push_back(p2);

    markerEdge.points.push_back(p2);
    markerEdge.points.push_back(p3);

    markerEdge.points.push_back(p3);
    markerEdge.points.push_back(p1);

    // std::cout << p1 << std::endl;
    // std::cout << p2 << std::endl;
    // std::cout << p3 << std::endl;
    // std::cout << "*********************" << std::endl;
  }

//! for debug-----------------------------------
  //  edges
  visualization_msgs::Marker markerFEdge;
  markerFEdge.header.frame_id = msg_frame_id;
  markerFEdge.header.stamp = msg_time;
  markerFEdge.action = visualization_msgs::Marker::ADD;
  markerFEdge.type = visualization_msgs::Marker::LINE_LIST;
  markerFEdge.ns = "xg_fedges";
  markerFEdge.id = 1;
  markerFEdge.pose.orientation.w = 1;
  markerFEdge.scale.x = 0.01; markerFEdge.scale.y = 0.01; markerFEdge.scale.z = 0.01;
  markerFEdge.color.r = 0.0f; markerFEdge.color.g = 1.0; markerFEdge.color.b = 0.0;
  markerFEdge.color.a = 1;

  // push point
  for (unsigned int i = 0; i<meshes_in.size(); i++) 
  {
    // if(fs.mesh_is_frontier[i]) continue;
    geometry_msgs::Point p1;
    p1.x = meshes_in[i]._a.x + key_pose(0);
    p1.y = meshes_in[i]._a.y + key_pose(1);
    p1.z = meshes_in[i]._a.z + key_pose(2);
    geometry_msgs::Point p2;
    p2.x = meshes_in[i]._b.x + key_pose(0);
    p2.y = meshes_in[i]._b.y + key_pose(1);
    p2.z = meshes_in[i]._b.z + key_pose(2);
    geometry_msgs::Point p3;
    p3.x = meshes_in[i]._c.x + key_pose(0);
    p3.y = meshes_in[i]._c.y + key_pose(1);
    p3.z = meshes_in[i]._c.z + key_pose(2);

    Vector3d p11(meshes_in[i]._a.x, meshes_in[i]._a.y,meshes_in[i]._a.z);
    Vector3d p22(meshes_in[i]._b.x, meshes_in[i]._b.y,meshes_in[i]._b.z);
    Vector3d p33(meshes_in[i]._c.x, meshes_in[i]._c.y,meshes_in[i]._c.z);


    if(isFrontierEdge(p11, p22, key_pose)){
        markerFEdge.points.push_back(p1);
        markerFEdge.points.push_back(p2);
    }
    if(isFrontierEdge(p22, p33, key_pose)){
        markerFEdge.points.push_back(p2);
        markerFEdge.points.push_back(p3);
    }
    if(isFrontierEdge(p33, p11, key_pose)){
        markerFEdge.points.push_back(p3);
        markerFEdge.points.push_back(p1);
    }
  }
//! for debug-----------------------------------------------


  star_cvx_plane_pub_.publish(component);

  markerArray.markers.push_back(markerNode);
  markerArray.markers.push_back(markerEdge);
  // markerArray.markers.push_back(markerFEdge);

  star_cvx_edge_pub_.publish(markerArray);
}

void FreeSpaceFactory::publishStarCvxFrontier( FreeSpace &fs,Vector3d key_pose,
                     ros::Time msg_time, std::string msg_frame_id )
{
  // show plane
  std::vector<FastLab::Triangle> &meshes_in = fs.meshes;
  visualization_msgs::Marker component;
  component.header.frame_id = msg_frame_id;
  component.header.stamp = msg_time;
  component.ns = "xg_planes";
  component.lifetime = ros::Duration();
  component.frame_locked = true;
  component.type = visualization_msgs::Marker::TRIANGLE_LIST;
  component.action = visualization_msgs::Marker::ADD;
  component.color.r = 0.0f;
  component.color.g = 1.0f;
  component.color.b = 0.0f;
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

  // show point and edges
  visualization_msgs::MarkerArray markerArray;
  //  nodes
  visualization_msgs::Marker markerNode;
  markerNode.header.frame_id = msg_frame_id;
  markerNode.header.stamp = msg_time;
  markerNode.action = visualization_msgs::Marker::ADD;
  markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
  markerNode.ns = "xg_nodes";
  markerNode.id = 0;
  markerNode.pose.orientation.w = 1;
  markerNode.scale.x = 0.05; markerNode.scale.y = 0.05; markerNode.scale.z = 0.05; 
  markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
  markerNode.color.a = 1;
  //  edges
  visualization_msgs::Marker markerEdge;
  markerEdge.header.frame_id = msg_frame_id;
  markerEdge.header.stamp = msg_time;
  markerEdge.action = visualization_msgs::Marker::ADD;
  markerEdge.type = visualization_msgs::Marker::LINE_LIST;
  markerEdge.ns = "xg_edges";
  markerEdge.id = 1;
  markerEdge.pose.orientation.w = 1;
  markerEdge.scale.x = 0.01; markerEdge.scale.y = 0.01; markerEdge.scale.z = 0.01;
  markerEdge.color.r = 1.0f; markerEdge.color.g = 0.0; markerEdge.color.b = 0.0;
  markerEdge.color.a = 1;

  // push point
  for (unsigned int i = 0; i<meshes_in.size(); i++) 
  {
    if(!fs.mesh_is_frontier[i]) continue;
    geometry_msgs::Point p1;
    p1.x = meshes_in[i]._a.x + key_pose(0);
    p1.y = meshes_in[i]._a.y + key_pose(1);
    p1.z = meshes_in[i]._a.z + key_pose(2);
    component.points.push_back(p1);
    geometry_msgs::Point p2;
    p2.x = meshes_in[i]._b.x + key_pose(0);
    p2.y = meshes_in[i]._b.y + key_pose(1);
    p2.z = meshes_in[i]._b.z + key_pose(2);
    component.points.push_back(p2);
    geometry_msgs::Point p3;
    p3.x = meshes_in[i]._c.x + key_pose(0);
    p3.y = meshes_in[i]._c.y + key_pose(1);
    p3.z = meshes_in[i]._c.z + key_pose(2);
    component.points.push_back(p3);

    markerNode.points.push_back(p1);
    markerNode.points.push_back(p2);
    markerNode.points.push_back(p3);

    markerEdge.points.push_back(p1);
    markerEdge.points.push_back(p2);

    markerEdge.points.push_back(p2);
    markerEdge.points.push_back(p3);

    markerEdge.points.push_back(p3);
    markerEdge.points.push_back(p1);

    // std::cout << p1 << std::endl;
    // std::cout << p2 << std::endl;
    // std::cout << p3 << std::endl;
    // std::cout << "*********************" << std::endl;
  }

  star_cvx_f_plane_pub_.publish(component);

  markerArray.markers.push_back(markerNode);
  markerArray.markers.push_back(markerEdge);
  star_cvx_f_edge_pub_.publish(markerArray);
}



void FreeSpaceFactory::publishStarCvxAll( FreeSpaceInfo& freespace_info_in,Vector3d key_pose,
                     ros::Time msg_time, std::string msg_frame_id )
{

  // show point and edges
  visualization_msgs::MarkerArray markerArray;

  int id = 0;
  for(size_t k = 0; k < freespace_info_in.free_space_list.size(); k++){
    // show plane
    FreeSpace fs = freespace_info_in.free_space_list[k];
    std::vector<FastLab::Triangle> &meshes_in = fs.meshes;
    visualization_msgs::Marker component;
    component.header.frame_id = msg_frame_id;
    component.header.stamp = msg_time;
    component.ns = "xg_planes";
    component.lifetime = ros::Duration();
    component.frame_locked = true;
    component.type = visualization_msgs::Marker::TRIANGLE_LIST;
    component.action = visualization_msgs::Marker::ADD;
    component.color.r = 1.0f;
    component.color.g = 0.0f;
    component.color.b = 0.0f;
    component.color.a = 0.2f;
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


    //  nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = msg_frame_id;
    markerNode.header.stamp = msg_time;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "xg_nodes";
    markerNode.id = id++;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.05; markerNode.scale.y = 0.05; markerNode.scale.z = 0.05; 
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    //  edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = msg_frame_id;
    markerEdge.header.stamp = msg_time;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "xg_edges";
    markerEdge.id = id++;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.01; markerEdge.scale.y = 0.01; markerEdge.scale.z = 0.01;
    markerEdge.color.r = 1.0f; markerEdge.color.g = 0.0; markerEdge.color.b = 0.0;
    markerEdge.color.a = 1;

    // push point
    pcl::PointXYZ key_pose_pcl = freespace_info_in.posegraph->getCor(k);
    Vector3d key_pose(key_pose_pcl.x, key_pose_pcl.y, key_pose_pcl.z);

    for (unsigned int i = 0; i<meshes_in.size(); i++) 
    {
      // if(fs.mesh_is_frontier[i]) continue;
      geometry_msgs::Point p1;
      p1.x = meshes_in[i]._a.x + key_pose(0);
      p1.y = meshes_in[i]._a.y + key_pose(1);
      p1.z = meshes_in[i]._a.z + key_pose(2);
      component.points.push_back(p1);
      geometry_msgs::Point p2;
      p2.x = meshes_in[i]._b.x + key_pose(0);
      p2.y = meshes_in[i]._b.y + key_pose(1);
      p2.z = meshes_in[i]._b.z + key_pose(2);
      component.points.push_back(p2);
      geometry_msgs::Point p3;
      p3.x = meshes_in[i]._c.x + key_pose(0);
      p3.y = meshes_in[i]._c.y + key_pose(1);
      p3.z = meshes_in[i]._c.z + key_pose(2);
      component.points.push_back(p3);

      markerNode.points.push_back(p1);
      markerNode.points.push_back(p2);
      markerNode.points.push_back(p3);

      markerEdge.points.push_back(p1);
      markerEdge.points.push_back(p2);

      markerEdge.points.push_back(p2);
      markerEdge.points.push_back(p3);

      markerEdge.points.push_back(p3);
      markerEdge.points.push_back(p1);

      // std::cout << p1 << std::endl;
      // std::cout << p2 << std::endl;
      // std::cout << p3 << std::endl;
      // std::cout << "*********************" << std::endl;
    }
    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
  }
  star_cvx_edge_all_pub_.publish(markerArray);
}

