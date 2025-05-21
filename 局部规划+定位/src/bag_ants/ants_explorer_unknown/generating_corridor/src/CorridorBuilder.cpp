#include <assert.h>
#include <ctime>
#include <cmath>

#include <Eigen/Eigen>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

#include "geoutils.hpp"
#include "CorridorBuilder.h"
#include "quickhull.hpp"
#include "tic_toc.h"
#include <set>

namespace FastLab
{

Corridor::Corridor(pcl::PointCloud<pcl::PointXYZ>::Ptr data, float radius, float precision)
{
    _data_ptr = data;
    _r = radius;
    _precision_qh = precision;
}

//    pcl::PointXYZ Corridor::flipMap(pcl::PointXYZ input) { // stereographic projection
//        pcl::PointXYZ p;
//        float norm2 = input.x*input.x + input.y*input.y + input.z*input.z;
//        p.x = 2*input.x/(norm2+1);
//        p.y = 2*input.y/(norm2+1);
//        p.z = 2*input.z/(norm2+1);
//        return p;
//    }

//    pcl::PointXYZ Corridor::inverseMap(pcl::PointXYZ input) { // TODO: find index directly from quick hull
//        pcl::PointXYZ p;
//        float norm2 = input.x*input.x + input.y*input.y + input.z*input.z;
//        float ori_norm2 = (2-norm2+std::sqrt(norm2*norm2+4-3*norm2))/(norm2);
//        p.x = 0.5*(ori_norm2+1)*input.x;
//        p.y = 0.5*(ori_norm2+1)*input.y;
//        p.z = 0.5*(ori_norm2+1)*input.z;
//        return p;
//    }

pcl::PointXYZ Corridor::flipMap(pcl::PointXYZ input)
{ // sphere flipping projection
    pcl::PointXYZ p;
    float norm2 = std::sqrt(input.x * input.x + input.y * input.y + input.z * input.z);
    if (norm2 == 0)
        return pcl::PointXYZ(0, 0, 0);
    p.x = input.x + 2 * (_r - norm2) * input.x / norm2;
    p.y = input.y + 2 * (_r - norm2) * input.y / norm2;
    p.z = input.z + 2 * (_r - norm2) * input.z / norm2;
    return p;
}

pcl::PointXYZ Corridor::inverseMap(pcl::PointXYZ input)
{ // TODO: find index directly from quick hull
    pcl::PointXYZ p;
    float norm2 = std::sqrt(input.x * input.x + input.y * input.y + input.z * input.z);
    if (norm2 == 0)
        return pcl::PointXYZ(0, 0, 0);
    float d = 2 * _r - norm2;
    p.x = input.x * d / norm2;
    p.y = input.y * d / norm2;
    p.z = input.z * d / norm2;
    return p;
}

double Corridor::distanceP2P(Eigen::Vector3d &normal, pcl::PointXYZ p_on_plane, pcl::PointXYZ p_query)
{
    Eigen::Vector3d start(p_on_plane.x, p_on_plane.y, p_on_plane.z);
    Eigen::Vector3d end(p_query.x, p_query.y, p_query.z);
    Eigen::Vector3d v = end - start;
    double distance = v.transpose() * normal;
    return distance;
}

bool Corridor::isIntersectTriangle(pcl::PointXYZ p, pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c)
{
    double d = a.x * (b.y * c.z - c.y * b.z) - a.y * (b.x * c.z - c.x * b.z) + a.z * (b.x * c.y - c.x * b.y);
    if (d == 0)
        return false;
    double m11 = (b.y * c.z - c.y * b.z) / d;
    double m12 = (c.x * b.z - b.x * c.z) / d;
    double m13 = (b.x * c.y - c.x * b.y) / d;
    if (p.x * m11 + p.y * m12 + p.z * m13 < 0)
        return false;
    double m21 = (c.y * a.z - a.y * c.z) / d;
    double m22 = (a.x * c.z - c.x * a.z) / d;
    double m23 = (a.y * c.x - a.x * c.y) / d;
    if (p.x * m21 + p.y * m22 + p.z * m23 < 0)
        return false;
    double m31 = (a.y * b.z - b.y * a.z) / d;
    double m32 = (b.x * a.z - a.x * b.z) / d;
    double m33 = (a.x * b.y - a.y * b.x) / d;
    return p.x * m31 + p.y * m32 + p.z * m33 > 0;
}

//     bool Corridor::isIntersectTriangle(pcl::PointXYZ p, pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c) {
//        if (p.x*p.x < 0.01) return false;
//        if (a.x == 0 || b.x == 0 || c.x == 0) return false;
//        Eigen::Matrix3d m;
//        m << a.x , b.x , c.x,
//             a.y , b.y , c.y,
//             a.z , b.z , c.z;
//        Eigen::Vector3d v(p.x, p.y, p.z);
//        Eigen::Vector3d result = m.inverse() * v;
//        if (result(0) < 0 || result(1) < 0 || result(2) < 0) return false;
//        return true;
//    }

void Corridor::H2V(std::vector<Plane> &planes, std::vector<pcl::PointXYZ> &vertices)
{
    Eigen::MatrixXd hPoly(6, planes.size()), vPoly;
    for (int i = 0; i < hPoly.cols(); i++)
    {
        hPoly.col(i) << -planes[i]._normal[0], -planes[i]._normal[1], -planes[i]._normal[2],
            planes[i]._p.x, planes[i]._p.y, planes[i]._p.z;
    }
    geoutils::enumerateVs(hPoly, vPoly);
    for (int i = 0; i < vPoly.cols(); i++)
    {
        vertices.emplace_back(vPoly(0, i), vPoly(1, i), vPoly(2, i));
    }
}

void Corridor::build(std::vector<Triangle> &meshes, std::vector<Eigen::Vector3d> &v, std::vector<size_t> &inxs, pcl::PointXYZ query_point, bool starconvex)
{
    if (_data_ptr->empty())
        return;

    std::vector<quickhull::Vector3<float>> flipcloud;
    flipcloud.push_back(quickhull::Vector3<float>(0, 0, 0));
    bool x_p_flag = true;
    bool x_n_flag = true;
    bool y_p_flag = true;
    bool y_n_flag = true;
    bool z_p_flag = true;
    bool z_n_flag = true;
    for (unsigned int i = 0; i < _data_ptr->points.size(); i++)
    {
        float x_check = _data_ptr->points[i].x - query_point.x;
        float y_check = _data_ptr->points[i].y - query_point.y;
        float z_check = _data_ptr->points[i].z - query_point.z;
        if (x_check > 0)
            x_p_flag = false;
        if (x_check < 0)
            x_n_flag = false;
        if (y_check > 0)
            y_p_flag = false;
        if (y_check < 0)
            y_n_flag = false;
        if (z_check > 0)
            z_p_flag = false;
        if (z_check < 0)
            z_n_flag = false;
        pcl::PointXYZ p = flipMap(pcl::PointXYZ(x_check, y_check, z_check));
        flipcloud.push_back(quickhull::Vector3<float>(p.x, p.y, p.z));
    }
    if (x_p_flag)
        flipcloud.push_back(quickhull::Vector3<float>(2 * _r - 0.1, 0, 0));
    if (x_n_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0.1 - 2 * _r, 0, 0));
    if (y_p_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 2 * _r - 0.1, 0));
    if (y_n_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 0.1 - 2 * _r, 0));
    if (z_p_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 0, 2 * _r - 0.1));
    if (z_n_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 0, 0.1 - 2 * _r));

    quickhull::QuickHull<float> qh;
    TicToc t;
    t.tic();
    auto hull = qh.getConvexHull(flipcloud, true, true, _precision_qh);
    // std::cout << t.toc() << "ms quick hull" << std::endl;

    auto vertexBuffer = hull.getVertexBuffer();
    inxs = hull.getIndexBuffer();

    if (starconvex)
    {
        // std::cout << "starconvex output!" << std::endl;
        // std::cout<<"vertexBuffer size: "<<vertexBuffer.size()<<std::endl;
        // std::cout<<"inxs size: "<<inxs.size()<<std::endl;

        // for (unsigned int m = 0; m < vertexBuffer.size(); m++){
        //     pcl::PointXYZ p1(vertexBuffer[m].x, vertexBuffer[m].y, vertexBuffer[m].z);
        //     Eigen::Vector3d op1(inverseMap(p1).x + query_point.x, inverseMap(p1).y + query_point.y, inverseMap(p1).z + query_point.z);
        //     v.push_back(op1);
        // }

        std::set<size_t> inx_set;
        for (unsigned int m = 0; m < inxs.size() / 3; m++)
        { // star convex
            pcl::PointXYZ p1(vertexBuffer[inxs[m * 3]].x, vertexBuffer[inxs[m * 3]].y, vertexBuffer[inxs[m * 3]].z);
            pcl::PointXYZ p2(vertexBuffer[inxs[m * 3 + 1]].x, vertexBuffer[inxs[m * 3 + 1]].y, vertexBuffer[inxs[m * 3 + 1]].z);
            pcl::PointXYZ p3(vertexBuffer[inxs[m * 3 + 2]].x, vertexBuffer[inxs[m * 3 + 2]].y, vertexBuffer[inxs[m * 3 + 2]].z);
            pcl::PointXYZ op1(inverseMap(p1).x + query_point.x, inverseMap(p1).y + query_point.y, inverseMap(p1).z + query_point.z);
            pcl::PointXYZ op2(inverseMap(p2).x + query_point.x, inverseMap(p2).y + query_point.y, inverseMap(p2).z + query_point.z);
            pcl::PointXYZ op3(inverseMap(p3).x + query_point.x, inverseMap(p3).y + query_point.y, inverseMap(p3).z + query_point.z);
            Triangle mesh(op1, op2, op3);
            inx_set.insert(inxs[m * 3]);
            inx_set.insert(inxs[m * 3+1]);
            inx_set.insert(inxs[m * 3+2]);

            mesh.calcuNormal();
            meshes.push_back(mesh);
        }
        for(auto& inx_i:inx_set){
            pcl::PointXYZ p1(vertexBuffer[inx_i].x, vertexBuffer[inx_i].y, vertexBuffer[inx_i].z);
            Eigen::Vector3d op1(inverseMap(p1).x + query_point.x, inverseMap(p1).y + query_point.y, inverseMap(p1).z + query_point.z);
            v.push_back(op1);
        }
        return;
    }

    std::vector<quickhull::Vector3<float>> ori_vertices;
    for (unsigned int n = 0; n < vertexBuffer.size(); n++)
    {
        pcl::PointXYZ mp(vertexBuffer[n].x, vertexBuffer[n].y, vertexBuffer[n].z);
        pcl::PointXYZ ori_p = inverseMap(mp);
        ori_vertices.push_back(quickhull::Vector3<float>(ori_p.x, ori_p.y, ori_p.z));
    }

    auto ori_hull = qh.getConvexHull(ori_vertices, true, false, 0.001);
    auto ori_vertexBuffer = ori_hull.getVertexBuffer();
    auto ori_indexBuffer = ori_hull.getIndexBuffer();

    std::vector<Triangle> ori_meshes;
    for (unsigned int m = 0; m < ori_indexBuffer.size() / 3; m++)
    {
        pcl::PointXYZ p1(ori_vertexBuffer[ori_indexBuffer[m * 3]].x, ori_vertexBuffer[ori_indexBuffer[m * 3]].y, ori_vertexBuffer[ori_indexBuffer[m * 3]].z);
        pcl::PointXYZ p2(ori_vertexBuffer[ori_indexBuffer[m * 3 + 1]].x, ori_vertexBuffer[ori_indexBuffer[m * 3 + 1]].y, ori_vertexBuffer[ori_indexBuffer[m * 3 + 1]].z);
        pcl::PointXYZ p3(ori_vertexBuffer[ori_indexBuffer[m * 3 + 2]].x, ori_vertexBuffer[ori_indexBuffer[m * 3 + 2]].y, ori_vertexBuffer[ori_indexBuffer[m * 3 + 2]].z);
        Triangle mesh(p1, p2, p3);
        mesh.calcuNormal();
        ori_meshes.push_back(mesh);
    }

    std::vector<float> distances(ori_meshes.size(), -1);
    std::vector<Plane> planes(ori_meshes.size());

    TicToc t2;
    t2.tic();
    for (unsigned int i = 0; i < ori_vertices.size(); i++)
    {
        if (ori_vertices[i].x == 0 || ori_vertices[i].y == 0 || ori_vertices[i].z == 0)
            continue;
        pcl::PointXYZ ap = pcl::PointXYZ(ori_vertices[i].x, ori_vertices[i].y, ori_vertices[i].z);
        for (unsigned int j = 0; j < ori_meshes.size(); j++)
        {
            if (isIntersectTriangle(ap, ori_meshes[j]._a, ori_meshes[j]._b, ori_meshes[j]._c))
            {
                double d = distanceP2P(ori_meshes[j]._normal, ori_meshes[j]._c, ap);
                if (d > distances[j])
                {
                    distances[j] = d;
                    planes[j]._p = ap;
                    planes[j]._normal = ori_meshes[j]._normal;
                }
            }
        }
    }

    std::vector<Plane> constraints;
    for (unsigned int n = 0; n < distances.size(); n++)
    {
        if (distances[n] > 0)
        {
            constraints.push_back(planes[n]);
        }
        else
        {
            constraints.push_back(Plane(ori_meshes[n]._c, ori_meshes[n]._normal));
        }
    }
    std::cout << t2.toc() << " ms convex modification part1" << std::endl;
    TicToc t3;
    t3.tic();
    std::vector<pcl::PointXYZ> vertexs;
    H2V(constraints, vertexs);
    std::cout << t3.toc() << " ms convex modification part2" << std::endl;
    // ugly method, just for visualization
    std::vector<quickhull::Vector3<float>> q_ori_vertices;
    for (auto vert : vertexs)
    {
        q_ori_vertices.push_back(quickhull::Vector3<float>(vert.x, vert.y, vert.z));
    }
    auto final_hull = qh.getConvexHull(q_ori_vertices, true, false, 0.01);
    auto final_vertexBuffer = final_hull.getVertexBuffer();
    auto final_indexBuffer = final_hull.getIndexBuffer();

    for (unsigned int m = 0; m < final_indexBuffer.size() / 3; m++)
    {
        pcl::PointXYZ p1(final_vertexBuffer[final_indexBuffer[m * 3]].x + query_point.x,
                         final_vertexBuffer[final_indexBuffer[m * 3]].y + query_point.y,
                         final_vertexBuffer[final_indexBuffer[m * 3]].z + query_point.z);
        pcl::PointXYZ p2(final_vertexBuffer[final_indexBuffer[m * 3 + 1]].x + query_point.x,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 1]].y + query_point.y,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 1]].z + query_point.z);
        pcl::PointXYZ p3(final_vertexBuffer[final_indexBuffer[m * 3 + 2]].x + query_point.x,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 2]].y + query_point.y,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 2]].z + query_point.z);
        Triangle mesh(p1, p2, p3);
        meshes.push_back(mesh);
    }
    //    std::cout << "mesh : " << meshes.size()  << std::endl;
}



void Corridor::build(std::vector<Triangle> &meshes, std::vector<Eigen::Vector3d> &v, pcl::PointXYZ query_point, bool starconvex)
{
    if (_data_ptr->empty())
        return;

    std::vector<quickhull::Vector3<float>> flipcloud;
    flipcloud.push_back(quickhull::Vector3<float>(0, 0, 0));
    bool x_p_flag = true;
    bool x_n_flag = true;
    bool y_p_flag = true;
    bool y_n_flag = true;
    bool z_p_flag = true;
    bool z_n_flag = true;
    for (unsigned int i = 0; i < _data_ptr->points.size(); i++)
    {
        float x_check = _data_ptr->points[i].x - query_point.x;
        float y_check = _data_ptr->points[i].y - query_point.y;
        float z_check = _data_ptr->points[i].z - query_point.z;
        if (x_check > 0)
            x_p_flag = false;
        if (x_check < 0)
            x_n_flag = false;
        if (y_check > 0)
            y_p_flag = false;
        if (y_check < 0)
            y_n_flag = false;
        if (z_check > 0)
            z_p_flag = false;
        if (z_check < 0)
            z_n_flag = false;
        pcl::PointXYZ p = flipMap(pcl::PointXYZ(x_check, y_check, z_check));
        flipcloud.push_back(quickhull::Vector3<float>(p.x, p.y, p.z));
    }
    if (x_p_flag)
        flipcloud.push_back(quickhull::Vector3<float>(2 * _r - 0.1, 0, 0));
    if (x_n_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0.1 - 2 * _r, 0, 0));
    if (y_p_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 2 * _r - 0.1, 0));
    if (y_n_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 0.1 - 2 * _r, 0));
    if (z_p_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 0, 2 * _r - 0.1));
    if (z_n_flag)
        flipcloud.push_back(quickhull::Vector3<float>(0, 0, 0.1 - 2 * _r));

    quickhull::QuickHull<float> qh;
    TicToc t;
    t.tic();
    auto hull = qh.getConvexHull(flipcloud, true, true, _precision_qh);
    std::cout << t.toc() << "ms quick hull" << std::endl;

    auto vertexBuffer = hull.getVertexBuffer();
    auto inxs = hull.getIndexBuffer();

    if (starconvex)
    {
        std::cout << "starconvex output!" << std::endl;
        std::cout<<"vertexBuffer size: "<<vertexBuffer.size()<<std::endl;
        std::cout<<"inxs size: "<<inxs.size()<<std::endl;

        for (unsigned int m = 0; m < vertexBuffer.size(); m++){
            pcl::PointXYZ p1(vertexBuffer[m].x, vertexBuffer[m].y, vertexBuffer[m].z);
            Eigen::Vector3d op1(inverseMap(p1).x + query_point.x, inverseMap(p1).y + query_point.y, inverseMap(p1).z + query_point.z);
            v.push_back(op1);
        }

        for (unsigned int m = 0; m < inxs.size() / 3; m++)
        { // star convex
            pcl::PointXYZ p1(vertexBuffer[inxs[m * 3]].x, vertexBuffer[inxs[m * 3]].y, vertexBuffer[inxs[m * 3]].z);
            pcl::PointXYZ p2(vertexBuffer[inxs[m * 3 + 1]].x, vertexBuffer[inxs[m * 3 + 1]].y, vertexBuffer[inxs[m * 3 + 1]].z);
            pcl::PointXYZ p3(vertexBuffer[inxs[m * 3 + 2]].x, vertexBuffer[inxs[m * 3 + 2]].y, vertexBuffer[inxs[m * 3 + 2]].z);
            pcl::PointXYZ op1(inverseMap(p1).x + query_point.x, inverseMap(p1).y + query_point.y, inverseMap(p1).z + query_point.z);
            pcl::PointXYZ op2(inverseMap(p2).x + query_point.x, inverseMap(p2).y + query_point.y, inverseMap(p2).z + query_point.z);
            pcl::PointXYZ op3(inverseMap(p3).x + query_point.x, inverseMap(p3).y + query_point.y, inverseMap(p3).z + query_point.z);
            Triangle mesh(op1, op2, op3);
            mesh.calcuNormal();
            meshes.push_back(mesh);
        }
        return;
    }

    std::vector<quickhull::Vector3<float>> ori_vertices;
    for (unsigned int n = 0; n < vertexBuffer.size(); n++)
    {
        pcl::PointXYZ mp(vertexBuffer[n].x, vertexBuffer[n].y, vertexBuffer[n].z);
        pcl::PointXYZ ori_p = inverseMap(mp);
        ori_vertices.push_back(quickhull::Vector3<float>(ori_p.x, ori_p.y, ori_p.z));
    }

    auto ori_hull = qh.getConvexHull(ori_vertices, true, false, 0.001);
    auto ori_vertexBuffer = ori_hull.getVertexBuffer();
    auto ori_indexBuffer = ori_hull.getIndexBuffer();

    std::vector<Triangle> ori_meshes;
    for (unsigned int m = 0; m < ori_indexBuffer.size() / 3; m++)
    {
        pcl::PointXYZ p1(ori_vertexBuffer[ori_indexBuffer[m * 3]].x, ori_vertexBuffer[ori_indexBuffer[m * 3]].y, ori_vertexBuffer[ori_indexBuffer[m * 3]].z);
        pcl::PointXYZ p2(ori_vertexBuffer[ori_indexBuffer[m * 3 + 1]].x, ori_vertexBuffer[ori_indexBuffer[m * 3 + 1]].y, ori_vertexBuffer[ori_indexBuffer[m * 3 + 1]].z);
        pcl::PointXYZ p3(ori_vertexBuffer[ori_indexBuffer[m * 3 + 2]].x, ori_vertexBuffer[ori_indexBuffer[m * 3 + 2]].y, ori_vertexBuffer[ori_indexBuffer[m * 3 + 2]].z);
        Triangle mesh(p1, p2, p3);
        mesh.calcuNormal();
        ori_meshes.push_back(mesh);
    }

    std::vector<float> distances(ori_meshes.size(), -1);
    std::vector<Plane> planes(ori_meshes.size());

    TicToc t2;
    t2.tic();
    for (unsigned int i = 0; i < ori_vertices.size(); i++)
    {
        if (ori_vertices[i].x == 0 || ori_vertices[i].y == 0 || ori_vertices[i].z == 0)
            continue;
        pcl::PointXYZ ap = pcl::PointXYZ(ori_vertices[i].x, ori_vertices[i].y, ori_vertices[i].z);
        for (unsigned int j = 0; j < ori_meshes.size(); j++)
        {
            if (isIntersectTriangle(ap, ori_meshes[j]._a, ori_meshes[j]._b, ori_meshes[j]._c))
            {
                double d = distanceP2P(ori_meshes[j]._normal, ori_meshes[j]._c, ap);
                if (d > distances[j])
                {
                    distances[j] = d;
                    planes[j]._p = ap;
                    planes[j]._normal = ori_meshes[j]._normal;
                }
            }
        }
    }

    std::vector<Plane> constraints;
    for (unsigned int n = 0; n < distances.size(); n++)
    {
        if (distances[n] > 0)
        {
            constraints.push_back(planes[n]);
        }
        else
        {
            constraints.push_back(Plane(ori_meshes[n]._c, ori_meshes[n]._normal));
        }
    }
    std::cout << t2.toc() << " ms convex modification part1" << std::endl;
    TicToc t3;
    t3.tic();
    std::vector<pcl::PointXYZ> vertexs;
    H2V(constraints, vertexs);
    std::cout << t3.toc() << " ms convex modification part2" << std::endl;
    // ugly method, just for visualization
    std::vector<quickhull::Vector3<float>> q_ori_vertices;
    for (auto vert : vertexs)
    {
        q_ori_vertices.push_back(quickhull::Vector3<float>(vert.x, vert.y, vert.z));
    }
    auto final_hull = qh.getConvexHull(q_ori_vertices, true, false, 0.01);
    auto final_vertexBuffer = final_hull.getVertexBuffer();
    auto final_indexBuffer = final_hull.getIndexBuffer();

    for (unsigned int m = 0; m < final_indexBuffer.size() / 3; m++)
    {
        pcl::PointXYZ p1(final_vertexBuffer[final_indexBuffer[m * 3]].x + query_point.x,
                         final_vertexBuffer[final_indexBuffer[m * 3]].y + query_point.y,
                         final_vertexBuffer[final_indexBuffer[m * 3]].z + query_point.z);
        pcl::PointXYZ p2(final_vertexBuffer[final_indexBuffer[m * 3 + 1]].x + query_point.x,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 1]].y + query_point.y,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 1]].z + query_point.z);
        pcl::PointXYZ p3(final_vertexBuffer[final_indexBuffer[m * 3 + 2]].x + query_point.x,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 2]].y + query_point.y,
                         final_vertexBuffer[final_indexBuffer[m * 3 + 2]].z + query_point.z);
        Triangle mesh(p1, p2, p3);
        meshes.push_back(mesh);
    }
    //    std::cout << "mesh : " << meshes.size()  << std::endl;
}

} // namespace FastLab
