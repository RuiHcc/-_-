#ifndef CORRIDOR_BUILDER_H
#define CORRIDOR_BUILDER_H

#include <cmath>

#include <Eigen/Dense>
#include <Eigen/Core>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/surface/convex_hull.h>

namespace FastLab {

    struct Triangle {
        Triangle(){};
        Triangle(pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c): _a(a), _b(b), _c(c){};
        pcl::PointXYZ _a;
        pcl::PointXYZ _b;
        pcl::PointXYZ _c;
        Eigen::Vector3d _normal;
        
        void calcuNormal() { //Point to origin(0,0,0)
            float x1 = _a.x - _c.x;
            float y1 = _a.y - _c.y;
            float z1 = _a.z - _c.z;
            float x2 = _b.x - _c.x;
            float y2 = _b.y - _c.y;
            float z2 = _b.z - _c.z;
            _normal[0] = y1*z2 - z1*y2;
            _normal[1] = z1*x2 - x1*z2;
            _normal[2] = x1*y2 - y1*x2;
            _normal.normalize();
            float dot_product = -_c.x*_normal[0] - _c.y*_normal[1] - _c.z*_normal[2];
            if (dot_product<0) _normal = - _normal;
        }
    };

    struct Plane {
        Plane() {}
        Plane(pcl::PointXYZ p, Eigen::Vector3d& normal): _p(p), _normal(normal){};
        
        pcl::PointXYZ _p;
        Eigen::Vector3d _normal;
    };

    class Corridor {
    public:
        Corridor(pcl::PointCloud<pcl::PointXYZ>::Ptr data, float radius, float precision);
        ~Corridor() {};
        void build(std::vector<Triangle> &meshes, std::vector<Eigen::Vector3d> &vertexs, std::vector<size_t> &inxs, pcl::PointXYZ query_point, bool starconvex = false);
        void build(std::vector<Triangle> &meshes, std::vector<Eigen::Vector3d> &vertexs, pcl::PointXYZ query_point, bool starconvex=false);
    private:
        pcl::PointCloud<pcl::PointXYZ>::Ptr _data_ptr;
        float _r;
        float _precision_qh;
        pcl::PointXYZ flipMap(pcl::PointXYZ input);
        pcl::PointXYZ inverseMap(pcl::PointXYZ input);
        double distanceP2P(Eigen::Vector3d& normal, pcl::PointXYZ p_on_plane, pcl::PointXYZ p_query); // respect to P2P
        bool isIntersectTriangle(pcl::PointXYZ p, pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c);
        void H2V(std::vector<Plane>& planes, std::vector<pcl::PointXYZ>& vertices);
    };



}


#endif //CORRIDOR_BUILDER_H