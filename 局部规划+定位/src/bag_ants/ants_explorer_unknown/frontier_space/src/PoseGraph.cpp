#include <PoseGraph.h>
/**
 * @brief Add bi-direction edge to the poseedge
 * 
 * @param inx_s edge start vertex inx
 * @param inx_e edge end vertex inx
 * @param dis euler distance
 */
void PoseGraph::addPoseEdge(int inx_s, int inx_e, double dis){
    addVertex(inx_s);
    addVertex(inx_e);
    pose_edge[inx_s].insert(Edge(inx_e, dis));
    pose_edge[inx_e].insert(Edge(inx_s, dis));
}

/**
 * @brief Please run renewKdtree() first!!!
 * Get the potential connectted vertex
 * (close in euler dis and far in sequential dis)
 * 
 * @param inx2 the query inx of self posegraph
 * @param res_set result map<inx,euler-distance>
 * @param connect_eulerdis_range close euler distance threshold
 * @param connect_seqendis_range far sequential distance threshold
 */
void PoseGraph::getPotenConnectSetSelf(int inx2, map<int, double>& res_set, double connect_eulerdis_range, double connect_seqendis_range){
    res_set.clear();
    pcl::PointXYZ cur = key_pose_list->points[inx2];
    Vector3d cur_p(cur.x, cur.y, cur.z);

    std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
    std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方
    if ( kdtree_keypose->radiusSearch(cur, connect_eulerdis_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (size_t k = 0; k < pointIdxRadiusSearch.size(); ++k){
            int inx1 = pointIdxRadiusSearch[k];
            pcl::PointXYZ endp = key_pose_list->points[inx1];
            Vector3d end_p(endp.x, endp.y, endp.z);

            double graph_dis = calSeqentialDis(inx1, inx2);
            if(graph_dis > connect_seqendis_range + 1e-3){
                res_set.insert({inx1, (cur_p-end_p).norm()});
            }

        }
    }
}

/**
 * @brief Please run renewKdtree() first!!!
 * Get the potential connectted vertex
 * (close in euler dis and far in sequential dis)
 * 
 * @param cur the query point
 * @param res_set result map<inx,euler-distance>
 * @param connect_eulerdis_range close euler distance threshold
 * @param connect_seqendis_range far sequential distance threshold
 */
void PoseGraph::getPotenConnectSetB(pcl::PointXYZ cur, map<int, double>& res_set, double connect_eulerdis_range, double connect_seqendis_range){
    cout<<"getPotenConnectSetB"<<endl;
    res_set.clear();
    if(kdtree_keypose->getInputCloud() == nullptr) return;
    Vector3d cur_p(cur.x, cur.y, cur.z);

    std::vector<int> pointIdxRadiusSearch;           //存储近邻索引
    std::vector<float> pointRadiusSquaredDistance;   //存储近邻对应距离的平方
    cout<<"size: "<<kdtree_keypose->getInputCloud()->size()<<endl;
    if ( kdtree_keypose->radiusSearch(cur, connect_eulerdis_range, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
        for (size_t k = 0; k < pointIdxRadiusSearch.size(); ++k){
            int inx1 = pointIdxRadiusSearch[k];
            pcl::PointXYZ endp = key_pose_list->points[inx1];
            Vector3d end_p(endp.x, endp.y, endp.z);
            res_set.insert({inx1, (cur_p-end_p).norm()});
        }
    }
    cout<<"getPotenConnectSetB Done"<<endl;

}

double PoseGraph::calSeqentialDis(int inx1, int inx2){
    double dis_sum = 0;
    for(int i = min(inx1,inx2); i < max(inx1, inx2); i++){
        pcl::PointXYZ p1 = key_pose_list->points[i];
        pcl::PointXYZ p2 = key_pose_list->points[i+1];
        dis_sum += sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) +(p1.z-p2.z)*(p1.z-p2.z));
    }
    return dis_sum;
}

void PoseGraph::renewKdtree(int inx_thr){
    kdtree_keypose.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    cloud = key_pose_list->makeShared();
    cout<<key_pose_list->size()<<endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudnew;
    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cout<<inx_thr<<", "<<cloud->size()<<endl;
    assert(inx_thr < cloud->size());
    for(size_t i = inx_thr; i < cloud->size(); i++){
        cloudnew->push_back(cloud->points[i]);
    }
    kdtree_keypose->setInputCloud(cloudnew);
}

void PoseGraph::deepCopy(PoseGraph::Ptr pg_tar, int copy_size){
    pg_tar->clear();
    ROS_ASSERT(getSize() >= copy_size);
    for(int i = 0; i < copy_size; i++){
        pg_tar->push_back(this->getCor(i));
    }
    pg_tar->pose_edge = this->pose_edge;
}

void PoseGraph::deepCopy(PoseGraph& pg_tar, int copy_size){
    pg_tar.clear();
    // cout<<"copy size: "<<copy_size<<endl;
    ROS_ASSERT(getSize() >= copy_size);
    for(int i = 0; i < copy_size; i++){
        pg_tar.push_back(this->getCor(i));
    }
    pg_tar.pose_edge = this->pose_edge;
}


void PoseGraph::dispPoseEdges(){
    cout<<"PoseEdges: "<<endl;
    for(auto it = pose_edge.begin(); it != pose_edge.end(); it++){
        for(auto edge:it->second){
            cout<<it->first<<"-"<<edge.v_inx<<"("<<edge.weight<<"), ";
        }
    }
    cout<<endl;
}

int MultiPoseGraph::getEdgeSize(int ps, int pe){
    // map<int, set<Edge> > res;
    auto res = table.find(make_pair(ps, pe));
    if(res == table.end()) return 0;
    int s = 0;
    for(auto it = res->second.begin(); it!=res->second.end(); it++){
        s += it->second.size();
    }
    return s;
}

void MultiPoseGraph::clear(){
    posegraphes.clear();
    table.clear();
}

void MultiPoseGraph::dispEdgeTable(){
    int i = 0;
    for(auto it = table.begin(); it != table.end(); it++){
        PoseEdge& edges = it->second;
        for(auto iter = edges.begin(); iter!=edges.end(); iter++){
            set<Edge>& edge_set = iter->second;
            for(auto edge:edge_set){
                i++;
                cout<<"(<"<<it->first.first<<"->"<<it->first.second<<">:"<<iter->first<<"->"<<edge.v_inx<<"="<<edge.weight<<"); ";
            }
        }
    }
    cout<<endl;
    cout<<"Edge Num: "<<i<<endl;
}

bool MultiPoseGraph::contain(pair<int, int> p, int inx) const{
    auto it = table.find(p);
    return it != table.end() && it->second.find(inx) != it->second.end();
}

bool MultiPoseGraph::containPg(int inx) const{
    auto it = posegraphes.find(inx);
    return it != posegraphes.end();
}

void MultiPoseGraph::addVertex(pair<int, int> p, int inx){
    auto it = table.find(p);
    if(it == table.end()){
        map<int, set<Edge>> ma;
        table[p] = ma;
    }
    else if(it->second.find(inx) == it->second.end())
    {   
		set<Edge> edge_list;
		table[p][inx] = edge_list;
    }
}

void MultiPoseGraph::addEdge(int ps, int pe, int inx_s, int inx_e, double dis){
    addVertex(make_pair(ps, pe), inx_s);
    addVertex(make_pair(pe, ps), inx_e);

    table[make_pair(ps, pe)][inx_s].insert(Edge(inx_e, dis));
    table[make_pair(pe, ps)][inx_e].insert(Edge(inx_s, dis));
}

void MultiPoseGraph::getEdge(int ps, int pe, map<int, set<Edge> >& res){
    res.clear();
    if(table.find(make_pair(ps, pe)) != table.end()){
        res = table[make_pair(ps, pe)];
    }
}

void MultiPoseGraph::getEdge(int ps, int pe, int vs, set<Edge>& res){
    res.clear();
    if(table.find(make_pair(ps, pe)) != table.end()){
        map<int, set<Edge> >& edge_map = table[make_pair(ps, pe)];
        auto it = edge_map.find(vs);
        if(it != edge_map.end()){
            res = it->second;
        }
    }
}


int MultiPoseGraph::getPoseGraphSize(int pg_id){
    return posegraphes[pg_id].key_pose_list->size();
}

double MultiPoseGraph::calHeris(pair<int, int> cur, pair<int, int> aim){
    pcl::PointXYZ p1 = posegraphes[cur.first].getCor(cur.second);
    pcl::PointXYZ p2 = posegraphes[aim.first].getCor(aim.second);
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) +(p1.z-p2.z)*(p1.z-p2.z));
}

void MultiPoseGraph::getNeighbor(pair<int, int> p, multimap<double, pair<int,int>>& neighbor){
    neighbor.clear();
    int cur_pgid = p.first;
    int cur_vinx = p.second;
    // self edges
    if(cur_vinx > 0){
        neighbor.insert({posegraphes[cur_pgid].calSeqentialDis(cur_vinx, cur_vinx-1), pair<int, int>(cur_pgid, cur_vinx-1)});
    }
    if(cur_vinx < getPoseGraphSize(cur_pgid)-1){
        neighbor.insert({posegraphes[cur_pgid].calSeqentialDis(cur_vinx, cur_vinx+1), pair<int, int>(cur_pgid, cur_vinx+1)});
    }
    set<Edge> self_edges;
    posegraphes[cur_pgid].getEdge(cur_vinx, self_edges);
    for(auto edge:self_edges){
        neighbor.insert({edge.weight, pair<int, int>(cur_pgid, edge.v_inx)});
    }
    // edges between posegraphes
    for(auto it = posegraphes.begin(); it != posegraphes.end(); it++){
        if(it->first == cur_pgid) continue;
        set<Edge> edge_set;
        getEdge(cur_pgid, it->first, cur_vinx, edge_set);
        for(auto edge:edge_set){
            neighbor.insert({edge.weight, pair<int, int>(it->first, edge.v_inx)});    
        }
    }
}

// The result path only contains the end points, but without start points
double MultiPoseGraph::calGraphPath(pair<int, int> cur, pair<int, int> aim, vector<Vector3d>& path){
    // cout<<"In MultiPoseGraph::calGraphPath"<<endl;
    // cout<<"keypose: "<<endl;
    // for(int i = 0; i < posegraphes[0].getSize(); i++){
    //     pcl::PointXYZI p_pcl = posegraphes[0].getCor(i);
    //     cout<<p_pcl.x<<", "<<p_pcl.y<<", "<<p_pcl.z<<"-"<<p_pcl.intensity<< endl;
    // }
    // cout<<"aim: ("<<aim.first<<", "<<aim.second<<")"<<endl;
    path.clear();
    if(cur == aim){
        pcl::PointXYZ p_pcl = posegraphes[aim.first].getCor(aim.second);
        path.emplace_back(p_pcl.x, p_pcl.y, p_pcl.z);
        return 0.0;
    }

    vector<int> size_table;
    int i = 0;
    for(auto it = posegraphes.begin(); it != posegraphes.end(); it++, i++){
        // cout<<"PG_"<<it->first<<endl;
        // it->second.dispPoseEdges();
        if(i == 0)
            size_table.push_back(0);
        else
            size_table.push_back(getPoseGraphSize(i-1)+size_table[i-1]);
    }
    size_table.push_back(getPoseGraphSize(i-1)+size_table[i-1]);
    // cout<<"size_table: ";
    // for(size_t i = 0; i < size_table.size(); i++){
    //     cout<<size_table[i]<<endl;
    // }
    // dispEdgeTable();


    auto pair2list = [size_table](pair<int, int> p) -> int{return size_table[p.first]+p.second;};

    vector<bool> is_in_close_list;
    vector<double> g_list;
    vector<pair<int, int>> parent_list;

    is_in_close_list.resize(size_table.back());
    g_list.resize(size_table.back());
    parent_list.resize(size_table.back());
    for(size_t i = 0; i < is_in_close_list.size(); i++){
        is_in_close_list[i] = false;
        g_list[i] = 9999999;
        parent_list[i] = pair<int, int>(-1,-1);
    }

    multimap<double, pair<int,int>> open_list;
    open_list.insert({calHeris(cur, aim), cur});
    g_list[pair2list(cur)] = 0;

    // loop
    bool success_flag = false;
    while(!open_list.empty()){
        multimap<double, pair<int,int>>::iterator it;
        it = open_list.begin();
        pair<int,int> inx_now = it->second;
        open_list.erase(it);
        is_in_close_list[pair2list(inx_now)] = true;
        // cout<<"current Node: <"<<inx_now.first<<","<<inx_now.second<<">"<<endl;
        // get aim
        if(inx_now == aim){
            success_flag = true;
            break;
        }

        multimap<double, pair<int,int>> neighbor;
        getNeighbor(inx_now, neighbor);
        // cout<<"Neighbor: "<<endl;
        for(multimap<double, pair<int,int>>::iterator it = neighbor.begin(); it != neighbor.end(); it++){
            pair<int,int> nei_inx = it->second;
            // cout<<"<"<<nei_inx.first<<","<<nei_inx.second<<">: ";

            if(is_in_close_list[pair2list(nei_inx)]){
                // cout<<"in close list"<<endl;
                continue;
            }
            if(g_list[pair2list(nei_inx)] == 9999999){
                g_list[pair2list(nei_inx)] = g_list[pair2list(nei_inx)] + it->first;
                open_list.insert({g_list[pair2list(nei_inx)]+calHeris(aim, nei_inx), nei_inx});
                parent_list[pair2list(nei_inx)] = inx_now;
                // cout<<"add in openlist"<<endl;
            }
            else if(g_list[pair2list(nei_inx)] > g_list[pair2list(nei_inx)] + it->first){
                double old_f = g_list[pair2list(nei_inx)] + calHeris(aim, nei_inx);
                auto it = open_list.find(old_f);
                bool deleteit = false;
                if(it != open_list.end()){
                    if(it->second == nei_inx){
                        open_list.erase(it);
                        deleteit = true;
                    }
                }
                if(!deleteit){
                    assert(false);
                }
                g_list[pair2list(nei_inx)] = g_list[pair2list(inx_now)] + it->first;
                open_list.insert({g_list[pair2list(nei_inx)]+calHeris(aim, nei_inx), nei_inx});
                parent_list[pair2list(nei_inx)] = inx_now;
                // cout<<"update in openlist"<<endl;
            }
        }
    }

    if(success_flag)
    {
        // ROS_ERROR("Graph A* search success");
        double dis_sum = 0.0;
        pair<int,int> inx_now = aim;

        // for(size_t i = 0; i < parent_list.size(); i++){
        //     cout<<parent_list[i].first<<"," <<parent_list[i].second <<endl;
        // }

        // path.push_back(aim_svp_p);
        while(inx_now != pair<int,int>(-1,-1)){
            // cout<<"inx: "<<inx_now.second<<endl;
            pcl::PointXYZ p_pcl = posegraphes[inx_now.first].getCor(inx_now.second);
            path.emplace_back(p_pcl.x, p_pcl.y, p_pcl.z);
            // cout<<path.back().transpose()<<" - ";
            if(path.size() >= 2){
                dis_sum += (path.back() - path[path.size()-2]).norm();
                // cout<<(path.back().block(0,0,3,1) - path[path.size()-2].block(0,0,3,1) ).norm()<<endl;
            }
            inx_now = parent_list[pair2list(inx_now)];
        }
        // path.push_back(drone_p);
        // dis_sum += (path.back().block(0,0,3,1) - path[path.size()-2].block(0,0,3,1)).norm();
        reverse(path.begin(), path.end());
        // ROS_ERROR_STREAM("Dis: " << dis_sum);
        return dis_sum;
    }
    else
    {
        ROS_ERROR("Graph A* search ERROR");
        throw "Graph A* search ERROR";
        return -1;
    }
}



void MultiPoseGraphFactory::mergeotherPoseGraph(MultiPoseGraph::Ptr merged_mpg, int start_inx){
    // ROS_WARN_STREAM("mergeotherPoseGraph");
    auto selfpgit = merged_mpg->posegraphes.find(merged_mpg->main_inx);
    if(selfpgit != merged_mpg->posegraphes.end()){
        PoseGraph& selfpg = selfpgit->second;
        if(start_inx >= selfpg.getSize()) return;
        for(int i = start_inx; i < selfpg.getSize(); i++){ //for each new keypose in selfposegraph, search in others' posegraph 
            pcl::PointXYZ point_vi = selfpg.getCor(i);
            for(auto otherpg = merged_mpg->posegraphes.begin(); otherpg != merged_mpg->posegraphes.end(); otherpg++){
                if(otherpg->first == merged_mpg->main_inx || otherpg->second.getSize() <= 0) continue;
                map<int, double> res_set;
                otherpg->second.getPotenConnectSetB(point_vi, res_set, connect_eulerdis_range, connect_seqendis_range);
                for(map<int, double>::iterator iter = res_set.begin(); iter != res_set.end(); iter++){
                    merged_mpg->addEdge(merged_mpg->main_inx,otherpg->first,i, iter->first, iter->second);
                    cout<<"mergeotherPoseGraph addEdge: (<"<<merged_mpg->main_inx<< "->"<<otherpg->first<<">,"<<i<<","<< iter->first<<")"<<endl;
                    merged_mpg->dispEdgeTable();
                }
            }
        }
    }
    // ROS_WARN_STREAM("mergeotherPoseGraph Done");

}


void MultiPoseGraphFactory::mergeotherPoseGraph(map<int, MultiPoseGraph>& allmpg, MultiPoseGraph::Ptr merged_mpg){
    //! 1. Get the main posegraph of each mulitipul posegraph
    merged_mpg->clear();
    for(auto it = allmpg.begin(); it != allmpg.end(); it++){
        int id = it->first;
        merged_mpg->posegraphes[id] = allmpg[id].posegraphes[id];
        cout<<"in merged_mpg: "<<"robot_"<<id<<": "<<merged_mpg->posegraphes[id].getSize()<<endl;
    }
    cout<<"aa"<<endl;

    //! 2. low method
    for(auto it = merged_mpg->posegraphes.begin(); it != merged_mpg->posegraphes.end(); it++){
        it->second.renewKdtree();
    }

    for(auto it = merged_mpg->posegraphes.begin(); it != merged_mpg->posegraphes.end(); it++){
        for(auto jt = merged_mpg->posegraphes.begin(); jt != merged_mpg->posegraphes.end(); jt++){
            int i = it->first;
            int j = jt->first;
            if(j<=i) continue;
            PoseGraph& pgi = it->second;
            PoseGraph& pgj = jt->second;
            // pgj.renewKdtree();
            for(int k = 0; k < pgi.getSize(); k++){
                pcl::PointXYZ q_p = pgi.getCor(k);
                map<int, double> res_set;
                pgj.getPotenConnectSetB(q_p, res_set, connect_eulerdis_range, connect_seqendis_range);
                for(map<int, double>::iterator iter = res_set.begin(); iter != res_set.end(); iter++){
                    merged_mpg->addEdge(i,j,k, iter->first, iter->second);
                }
            }
        }
    }

    //! 2. connect new part with old part
    // for(auto it =  allmpg.begin(); it != allmpg.end(); it++){
    //     for(auto jt =  allmpg.begin(); jt != allmpg.end(); jt++){



            
    //         int i = it->first;
    //         int j = jt->first;
    //         if(j<=i) continue;
    //         cout<<"multipg update between robot_("<<i<<", "<<j<<")"<<endl;
    //         map<int, set<Edge> > In_i_edgeij, In_j_edgeji;
    //         allmpg[i].getEdge(i,j,In_i_edgeij);
    //         cout<<"in Robot_"<<i<<" edge between "<<i<<","<<j<<" size: "<<In_i_edgeij.size()<<endl;
    //         allmpg[j].getEdge(j,i,In_j_edgeji);
    //         cout<<"in Robot_"<<j<<" edge between "<<j<<","<<i<<" size: "<< In_j_edgeji.size()<<endl;

    //         int i_newinx_thr, j_newinx_thr;
    //         i_newinx_thr = allmpg[j].getPoseGraphSize(i);
    //         j_newinx_thr = allmpg[i].getPoseGraphSize(j);
    //         int i_inx_size, j_inx_size;
    //         i_inx_size = allmpg[i].getPoseGraphSize(i);
    //         j_inx_size = allmpg[j].getPoseGraphSize(j);
    //         bool i_has_new, j_has_new;
    //         if(i_newinx_thr < i_inx_size){
    //             // merged_mpg->posegraphes[i].renewKdtree(i_newinx_thr);
    //             i_has_new = true;
    //             cout<<i_inx_size<<", "<<i_newinx_thr<<endl;
    //             cout<<"robot_"<<i<<" has new part of itself: "<< i_inx_size-i_newinx_thr <<endl;
    //         }
    //         if(j_newinx_thr < j_inx_size){
    //             // merged_mpg->posegraphes[j].renewKdtree(j_newinx_thr);
    //             j_has_new = true;
    //             cout<<j_inx_size<<", "<<j_newinx_thr<<endl;
    //             cout<<"robot_"<<j<<" has new part of itself"<< j_inx_size-j_newinx_thr <<endl;
    //         }

    //         // choose i as the old part
    //         for(map<int, set<Edge> >::iterator it = In_i_edgeij.begin(); it != In_i_edgeij.end(); it++){
    //             int v_in_i = it->first;
    //             // the old part in i
    //             if(v_in_i < i_newinx_thr){
    //                 set<Edge>& v_in_j_set = it->second;
    //                 for(set<Edge>::iterator iter = v_in_j_set.begin(); iter != v_in_j_set.end(); iter++){
    //                     merged_mpg->addEdge(i,j,it->first,iter->v_inx,iter->weight);
    //                     cout<<it->first<<"->"<<iter->v_inx<<endl;
    //                 }
    //                 //TODO PG Check the visible of the new connections 
    //                 if(j_has_new){
    //                     merged_mpg->posegraphes[j].renewKdtree(j_newinx_thr);
    //                     pcl::PointXYZ point_vi = merged_mpg->posegraphes[i].getCor(v_in_i);
    //                     map<int, double> res_set;
    //                     merged_mpg->posegraphes[j].getPotenConnectSetB(point_vi, res_set, connect_eulerdis_range, connect_seqendis_range);
    //                     for(map<int, double>::iterator iter = res_set.begin(); iter != res_set.end(); iter++){
    //                         merged_mpg->addEdge(i,j,it->first, iter->first, iter->second);
    //                         cout<<it->first<<"->"<<iter->first<<endl;
    //                     }
    //                 }
    //             }
    //             // the new part in i
    //             else{
    //                 // new part of i to old part of j
    //                 set<Edge>& v_in_j_set = it->second;
    //                 for(set<Edge>::iterator iter = v_in_j_set.begin(); iter != v_in_j_set.end(); iter++){
    //                     merged_mpg->addEdge(i,j,it->first,iter->v_inx,iter->weight);
    //                     cout<<it->first<<"->"<<iter->v_inx<<endl;
    //                 }
    //                 // new part of i to new part of j
    //                 //TODO PG Check the visible of the new connections 
    //                 if(j_has_new){
    //                     merged_mpg->posegraphes[j].renewKdtree(j_newinx_thr);
    //                     pcl::PointXYZ point_vi = merged_mpg->posegraphes[i].getCor(v_in_i);
    //                     map<int, double> res_set;
    //                     merged_mpg->posegraphes[j].getPotenConnectSetB(point_vi, res_set, connect_eulerdis_range, connect_seqendis_range);
    //                     for(map<int, double>::iterator iter = res_set.begin(); iter != res_set.end(); iter++){
    //                         merged_mpg->addEdge(i,j,it->first, iter->first, iter->second);
    //                         cout<<it->first<<"->"<<iter->first<<endl;
    //                     }
    //                 }
    //             }
    //         }
    //         cout<<"======multipg update between robot_("<<i<<", "<<j<<") Done"<<endl;
    //     }
    // }
    cout<<"aa  DDDDDDDDDDDDDD"<<endl;

}