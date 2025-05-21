#ifndef MESH_TABLE_HPP_
#define MESH_TABLE_HPP_

#include <vector>
#include <set>
#include <unordered_map>
#include <fstream>
#include <memory>
#include <Eigen/Eigen>
#include "CorridorBuilder.h"

using namespace Eigen;
using namespace std;

namespace FastLab {

	template <typename T>
	struct matrix_hash : std::unary_function<T, size_t> {
	std::size_t operator()(T const& matrix) const {
		size_t seed = 0;
		for (size_t i = 0; i < matrix.size(); ++i) {
		auto elem = *(matrix.data() + i);
		seed ^= std::hash<typename T::Scalar>()(elem) + 0x9e3779b9 + (seed << 6) +
				(seed >> 2);
		}
		return seed;
	}
	};

	class MeshTable{
	private:
		Vector2i table_num_;
		double theta_reso_;
		double phi_reso_;
		std::vector<std::vector<int>> data_;
		double D_thr = 0.1;
		// std::unordered_multimap<Eigen::Vector2i, int, matrix_hash<Eigen::Vector2i>> data_;

	public:
		MeshTable(const int theta_size = 18, const int phi_size = 36){
			setTableNum(theta_size, phi_size);
			data_ = std::vector<std::vector<int>>(theta_size*phi_size);
		};
		void setTableNum(const int theta_size = 18, const int phi_size = 36){
			table_num_(0) = theta_size;
			table_num_(1) = phi_size;
			theta_reso_ = 3.141593 / theta_size;//M_PI / theta_size;
			phi_reso_ = 2*3.141593 / phi_size;//2*M_PI / phi_size;
		}

		// Get mesh's AABB, and Assign the AABB correspond sphere grid with mesh index, normal must be calculated
		void createTable(const std::vector<FastLab::Triangle>& meshes){
			for(int i = 0; i < (int)meshes.size(); i++){
				std::vector<Vector2i> min_cor, max_cor;
				// cout<<endl;
				// The mesh connected to the robot don't generate into mesh table
				if(abs(meshes[i]._a.x)+abs(meshes[i]._a.y)+abs(meshes[i]._a.z) < 1e-2) continue;
				if(abs(meshes[i]._b.x)+abs(meshes[i]._b.y)+abs(meshes[i]._b.z) < 1e-2) continue;
				if(abs(meshes[i]._c.x)+abs(meshes[i]._c.y)+abs(meshes[i]._c.z) < 1e-2) continue;

				getAABB(meshes[i], min_cor, max_cor);
				
				// cout<<"mesh_a: "<<meshes[i]._a.x<<", "<<meshes[i]._a.y<<", "<<meshes[i]._a.z<<endl;
				// cout<<"mesh_b: "<<meshes[i]._b.x<<", "<<meshes[i]._b.y<<", "<<meshes[i]._b.z<<endl;
				// cout<<"mesh_c: "<<meshes[i]._c.x<<", "<<meshes[i]._c.y<<", "<<meshes[i]._c.z<<endl;


				for(size_t j = 0; j < min_cor.size(); j++){
					Vector2i min_cor_t = min_cor[j];
					Vector2i max_cor_t = max_cor[j];
					// cout<<"min_cor: "<<min_cor_t.transpose()<<endl;
					// cout<<"max_cor: "<<max_cor_t.transpose()<<endl;
					min_cor_t(0) = max(0, min_cor_t(0)-1);
					min_cor_t(1) = max(0, min_cor_t(1)-1);
					max_cor_t(0) = min(table_num_(0)-1, max_cor_t(0)+1);
					max_cor_t(1) = min(table_num_(1)-1, max_cor_t(1)+1);


					for(int inx_t = min_cor_t(0); inx_t <= max_cor_t(0); inx_t++){
						for(int inx_p = min_cor_t(1); inx_p <= max_cor_t(1); inx_p++){
							data_[toAddress(Vector2i(inx_t,inx_p))].push_back(i);
						}
					}
				}

			}
			// for(int i = 0; i < table_num_(0); i++){
			// 	for(int j = 0; j < table_num_(1); j++){
			// 		cout<<"["<<i<<","<<j<<"]: "<<data_[toAddress(Vector2i(i,j))].size()<<endl;
			// 	}
			// }
		}

		// Query the point correspond to which mesh, return mesh index
		void getMeshInx(const pcl::PointXYZ query_p, std::set<int>& res){
			res.clear();
			Vector2i p_sphinx = carToSphInx(query_p);
			// cout<<"p_sphinx: "<<p_sphinx.transpose()<<endl;
			std::vector<int> t_res = data_[toAddress(p_sphinx)];
			// std::cout<<"res_size: "<<t_res.size()<<std::endl;
			for(auto element:t_res){
				res.insert(element);
			}
		}

		void getAABB(const FastLab::Triangle& mesh, std::vector<Vector2i>& min_cor, std::vector<Vector2i>& max_cor){
			std::vector<Vector3d> p_vec;
			p_vec.emplace_back(mesh._a.x, mesh._a.y, mesh._a.z);
			p_vec.emplace_back(mesh._b.x, mesh._b.y, mesh._b.z);
			p_vec.emplace_back(mesh._c.x, mesh._c.y, mesh._c.z);

			p_vec.push_back((p_vec[0]+p_vec[1])/2.0);
			p_vec.push_back((p_vec[1]+p_vec[2])/2.0);
			p_vec.push_back((p_vec[0]+p_vec[2])/2.0);


			Vector2i min_cor_t(999, 999);
			Vector2i max_cor_t(0, 0);
			for(size_t i = 0; i < p_vec.size(); i++){
				Vector2i inx = carToSphInx(p_vec[i]);
				for(int j = 0; j < 2; j++){
					if(inx(j)<min_cor_t(j)) min_cor_t(j) = inx(j);
					if(inx(j)>max_cor_t(j)) max_cor_t(j) = inx(j);
				}
			}

			Vector2i inx_a = carToSphInx(mesh._a);
			Vector2i inx_b = carToSphInx(mesh._b);
			Vector2i inx_c = carToSphInx(mesh._c);
			// cout<<"inx_a: "<<inx_a.transpose()<<endl;
			// cout<<"inx_b: "<<inx_b.transpose()<<endl;
			// cout<<"inx_c: "<<inx_c.transpose()<<endl;


			// for(int i = 0; i < 2; i++){
			// 	if(inx_a(i) < min_cor_t(i)) min_cor_t(i) = inx_a(i);
			// 	if(inx_b(i) < min_cor_t(i)) min_cor_t(i) = inx_b(i);
			// 	if(inx_c(i) < min_cor_t(i)) min_cor_t(i) = inx_c(i);

			// 	if(inx_a(i) > max_cor_t(i)) max_cor_t(i) = inx_a(i);
			// 	if(inx_b(i) > max_cor_t(i)) max_cor_t(i) = inx_b(i);
			// 	if(inx_c(i) > max_cor_t(i)) max_cor_t(i) = inx_c(i);
			// }

			if(ifCrossZ(mesh)){
				if(mesh._normal(2) < 0){ // located on the top
					// cout<<"top"<<endl;
					min_cor_t << 0, 0;
					max_cor_t << max_cor_t(0), table_num_(1)-1;
					min_cor.push_back(min_cor_t);
					max_cor.push_back(max_cor_t);
					return;
				}else if((mesh._normal(2) > 0)){// located on the buttom
					// cout<<"buttom"<<endl;
					min_cor_t << min_cor_t(0), 0;
					max_cor_t << table_num_(0)-1, table_num_(1)-1;
					min_cor.push_back(min_cor_t);
					max_cor.push_back(max_cor_t);
					return;
				}
			}

			if(ifCrossX(mesh)){
				// cout<<"x cross, normal: "<<mesh._normal(0)<<endl;
				// if(mesh._normal(0) < 0){ //on the x positive direction
					// cout<<"x positive"<<endl;
				
					int min_left_phi = 999, max_right_phi = 0;
					if(inx_a(1) > table_num_(1)/2 && inx_a(1) < min_left_phi) min_left_phi = inx_a(1);
					if(inx_b(1) > table_num_(1)/2 && inx_b(1) < min_left_phi) min_left_phi = inx_b(1);
					if(inx_c(1) > table_num_(1)/2 && inx_c(1) < min_left_phi) min_left_phi = inx_c(1);
					if(inx_a(1) < table_num_(1)/2 && inx_a(1) > max_right_phi) max_right_phi = inx_a(1);
					if(inx_b(1) < table_num_(1)/2 && inx_b(1) > max_right_phi) max_right_phi = inx_b(1);
					if(inx_c(1) < table_num_(1)/2 && inx_c(1) > max_right_phi) max_right_phi = inx_c(1);
					if(min_left_phi - max_right_phi > table_num_(1)/2){ // if located on the both sides of the x-z plane
						min_cor.emplace_back(min_cor_t(0), 0);
						max_cor.emplace_back(max_cor_t(0), max_right_phi);
						min_cor.emplace_back(min_cor_t(0), min_left_phi);
						max_cor.emplace_back(max_cor_t(0), table_num_(1)-1);
						return;
					}

				// }
			}
			min_cor.push_back(min_cor_t);
			max_cor.push_back(max_cor_t);
			return;

		}

		Vector2i carToSphInx(pcl::PointXYZ car){
			Vector2i SphInx;
			double r, theta, phi;
			r = sqrt(car.x*car.x + car.y*car.y + car.z*car.z);
			theta = acos(car.z/r); //theta in [0,pi]
			phi = atan2(car.y, car.x); //[-pi,pi]
			// cout<<"(x,y,z): "<<car.x<<", "<<car.y<<", "<<car.z<<endl;
			// cout<<"(theta,phi): "<<theta<<", "<<phi<<endl;
			if(phi < 0) phi += 2*M_PI; //phi in [0,2pi]
			SphInx(0) = floor(theta/theta_reso_);
			SphInx(1) = floor(phi/phi_reso_);
			return SphInx;
		}
		Vector2i carToSphInx(Vector3d car){
			Vector2i SphInx;
			double r, theta, phi;
			r = sqrt(car.x()*car.x() + car.y()*car.y() + car.z()*car.z());
			theta = acos(car.z()/r); //theta in [0,pi]
			phi = atan2(car.y(), car.x()); //[-pi,pi]
			// cout<<"(x,y,z): "<<car.x()<<", "<<car.y()<<", "<<car.z()<<endl;
			// cout<<"(theta,phi): "<<theta<<", "<<phi<<endl;
			if(phi < 0) phi += 2*M_PI; //phi in [0,2pi]
			SphInx(0) = floor(theta/theta_reso_);
			SphInx(1) = floor(phi/phi_reso_);
			return SphInx;
		}

		int toAddress(Vector2i sphinx){
			return sphinx(0)*table_num_(1) + sphinx(1);
		}

		bool ifCrossZ(const FastLab::Triangle& mesh){
			Vector2d v0 = Vector2d(mesh._c.x, mesh._c.y) - Vector2d(mesh._a.x, mesh._a.y);
			Vector2d v1 = Vector2d(mesh._b.x, mesh._b.y) - Vector2d(mesh._a.x, mesh._a.y);
			Vector2d v2 = Vector2d(0, 0) - Vector2d(mesh._a.x, mesh._a.y);
			double inv = 1/(v0.dot(v0) * v1.dot(v1) - v0.dot(v1 * v1.dot(v0)));
			double u = (v1.dot(v1)*v2.dot(v0) - v1.dot(v0)*v2.dot(v1)) * inv;
			double v = (v0.dot(v0)*v2.dot(v1) - v0.dot(v1)*v2.dot(v0)) * inv;
			return (u>=0 && v>=0 && (u+v)<=1); //don‘t choose equal!
		}

		bool ifCrossX(const FastLab::Triangle& mesh){
			int y_pos = 0;
			int y_neg = 0;
			(mesh._a.y>=0) ? y_pos++ : y_neg++;
			(mesh._b.y>=0) ? y_pos++ : y_neg++;
			(mesh._c.y>=0) ? y_pos++ : y_neg++;
			if(y_pos == 0 || y_neg == 0) return false;
			return true;
		}


		double distanceP2P(const Eigen::Vector3d& normal, const pcl::PointXYZ& p_on_plane, const pcl::PointXYZ& p_query) {
			Eigen::Vector3d start(p_on_plane.x, p_on_plane.y, p_on_plane.z);
			Eigen::Vector3d end(p_query.x, p_query.y, p_query.z);
			Eigen::Vector3d v = end - start;
			double distance = v.transpose()*normal;
			return distance;
		}

		Vector3d projectToMesh(const FastLab::Triangle& mesh, const pcl::PointXYZ& p_query){
			Vector3d p_res;
			Vector3d c(mesh._c.x, mesh._c.y, mesh._c.z);
			Vector3d p(p_query.x, p_query.y, p_query.z);
			Vector3d pc = c-p;
			Vector3d n = mesh._normal.normalized();
			Vector3d p2p_res;
			p2p_res = pc.dot(n)*n;
			p_res = p2p_res + p;
			return p_res;
		}

		bool isPointInTriangle(const FastLab::Triangle& mesh, const Vector3d& p_project){
			Vector3d P = p_project;
			Vector3d A(mesh._a.x, mesh._a.y, mesh._a.z);
			Vector3d B(mesh._b.x, mesh._b.y, mesh._b.z);
			Vector3d C(mesh._c.x, mesh._c.y, mesh._c.z);
			Vector3d PA = A-P;
			Vector3d PB = B-P;
			Vector3d PC = C-P;
			Vector3d C1 = PA.cross(PB);
			Vector3d C2 = PB.cross(PC);
			Vector3d C3 = PC.cross(PA);
			return C1.dot(C2) >= 0 && C2.dot(C3) >= 0 && C1.dot(C3) >= 0;
		}

		bool isPointInExtendPolygen(const FastLab::Triangle& mesh, const pcl::PointXYZ& p_query){
			Vector3d P(p_query.x, p_query.y, p_query.z);
			Vector3d p_project = projectToMesh(mesh, p_query);
			double D = (p_project - P).norm();
			return (D < D_thr) && isPointInTriangle(mesh, p_project);
		}

		bool isIntersectTriangle(pcl::PointXYZ p, pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c)
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

		// Query if a Point inside the triPsyramid generated by the mesh
		double isInMesh(const FastLab::Triangle& mesh, const pcl::PointXYZ& cur_pos, const pcl::PointXYZ& p_query){
			if(isIntersectTriangle(p_query, mesh._a, mesh._b, mesh._c)){
				double distance = distanceP2P(mesh._normal, mesh._c, p_query);
				if(distance > -1e-6){
					return 1;
				}
				// else if(isPointInExtendPolygen(mesh, p_query))
				// {
				// 	return 1;
				// }
				else{
					return -1;
				}
			}else{
				return -1;
			}
		}
		
		// Query if a Point inside the meshes indexed by the mesh_inx
		double isInMeshes(const std::vector<FastLab::Triangle>& meshes, const std::set<int>& mesh_inx,
						const pcl::PointXYZ cur_pos, const pcl::PointXYZ p_query){
			bool isIn = false;
			for(std::set<int>::iterator it = mesh_inx.begin(); it != mesh_inx.end(); it++){
				// Elimate the mesh with any vertex is robot position
				if(abs(meshes[*it]._a.x)+abs(meshes[*it]._a.y)+abs(meshes[*it]._a.z) < 1e-2) continue;
				if(abs(meshes[*it]._b.x)+abs(meshes[*it]._b.y)+abs(meshes[*it]._b.z) < 1e-2) continue;
				if(abs(meshes[*it]._c.x)+abs(meshes[*it]._c.y)+abs(meshes[*it]._c.z) < 1e-2) continue;
				double dis = isInMesh(meshes[*it], cur_pos, p_query);
				// cout<<"mesh_check: "<<meshes[*it]._a.x<<", "<<meshes[*it]._a.y<<", "<<meshes[*it]._a.z<<" -- "<<t<<endl;
				// cout<<"mesh_check: "<<meshes[*it]._b.x<<", "<<meshes[*it]._b.y<<", "<<meshes[*it]._b.z<<" -- "<<t<<endl;
				// cout<<"mesh_check: "<<meshes[*it]._c.x<<", "<<meshes[*it]._c.y<<", "<<meshes[*it]._c.z<<" -- "<<t<<endl;

				isIn = isIn | (dis > -1e-6);
				if(isIn) return dis;
			}
			return -1;
		}

		double isInStarHull(const std::vector<FastLab::Triangle>& meshes, const pcl::PointXYZ cur_pos, const pcl::PointXYZ p_query){
			std::set<int> res;
			pcl::PointXYZ p_query_tran;
			p_query_tran.x = p_query.x;// - cur_pos.x;
			p_query_tran.y = p_query.y;// - cur_pos.y;
			p_query_tran.z = p_query.z;// - cur_pos.z;
			
			getMeshInx(p_query_tran, res);
			return isInMeshes(meshes, res, cur_pos, p_query_tran);
		}
		double isInStarHull(const std::vector<FastLab::Triangle>& meshes, const pcl::PointXYZ cur_pos, Vector3d p_query){
			pcl::PointXYZ p;
			p.x = p_query(0);
			p.y = p_query(1);
			p.z = p_query(2);
			return isInStarHull(meshes, cur_pos, p);
		}
	};






}

#endif /* CONVEXHULL_UTILS_HPP_ */
