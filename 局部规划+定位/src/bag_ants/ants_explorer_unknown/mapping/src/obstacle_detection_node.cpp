#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/distances.h>
#include <pcl/common/copy_point.h>

#include <Eigen/Dense>

geometry_msgs::Point robot_pos_; // 机器人当前位置
bool z_mode = true;

class ObstacleDetectionNode {
public:
    ObstacleDetectionNode() : nh_("~") {
        // 订阅雷达点云数据
        cloud_sub_ = nh_.subscribe("/map_generator/global_cloud", 1, &ObstacleDetectionNode::cloudCallback, this);

        // 发布障碍物点云数据
        obstacle_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle/cloud", 1);
        
        // 获取机器人当前位置的话题
        // robot_pos_sub_ = nh_.subscribe("/robot_position", 1, &ObstacleDetectionNode::robotPositionCallback, this);

        // 读取参数：R的值
        nh_.param<double>("R", R_, 15); // 默认半径R为5米
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
        ROS_WARN("running");
        // 将ROS的点云消息转换为PCL点云数据类型
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *cloud);

        // 进行地面分割
        pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // groundSegmentation(cloud, obstacle_cloud, 0.6, 1.6, 0.4, 200);
        groundSegmentation(cloud, obstacle_cloud, -1, 11, 0.4, 200);
        // obstacle_cloud = cloud;

        // 使用半径滤波器进行过滤
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloudR(new pcl::PointCloud<pcl::PointXYZ>);
        radiusFilterPointCloud(obstacle_cloud, filtered_cloudR);

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointXYZ ref_point;
        ref_point.x = robot_pos_.x;
        ref_point.y = robot_pos_.y;
        ref_point.z = robot_pos_.z;
        // preprocessPointCloud(filtered_cloudR, filtered_cloud, 8, 2.0, 
        //                     ref_point, 2, 0.2, 0.5);
        preprocessPointCloud(filtered_cloudR, filtered_cloud, 18, 22.0, 
                            ref_point, 2, 5, 0.01, 0.3, 0.6);
        // std::vector<float> thresholds_vec;
        // thresholds_vec = {1, 2, 3, 4, 5, 6, 7, 8, 9};
        // std::vector<float> leaf_vec;
        // leaf_vec = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1};
        // preprocessPointCloud(filtered_cloudR, filtered_cloud, 5, 5.0, 
        //                     ref_point, thresholds_vec, leaf_vec);

        // filtered_cloud = filtered_cloudR;
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // segmentAndExtractPlane(filtered_cloud, plane_cloud);
        plane_cloud = filtered_cloud;

        // 发布过滤后的障碍物点云数据
        sensor_msgs::PointCloud2 obstacle_cloud_msg;
        pcl::toROSMsg(*plane_cloud, obstacle_cloud_msg);
        obstacle_cloud_msg.header = cloud_msg->header;
        obstacle_pub_.publish(obstacle_cloud_msg);
    }

    // void robotPositionCallback(const geometry_msgs::PointStampedConstPtr& pos_msg) {
    //     // 更新机器人当前位置
    //     robot_pos_.x = pos_msg->point.x;
    //     robot_pos_.y = pos_msg->point.y;
    //     robot_pos_.z = pos_msg->point.z;
    //     robot_pos_.x = 0.0;
    //     robot_pos_.y = 0.0;
    //     robot_pos_.z = 0.0;
    // }

    void groundSegmentation(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& obstacle_cloud, 
                            double height_min, double height_max, 
                            double distance_threshold, int max_iterations) {
        // // 创建一个分割器
        // pcl::SACSegmentation<pcl::PointXYZ> seg;
        // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        // seg.setOptimizeCoefficients(true);
        // seg.setModelType(pcl::SACMODEL_PLANE);
        // seg.setMethodType(pcl::SAC_RANSAC);
        // seg.setDistanceThreshold(0.1); // 设定RANSAC算法的距离阈值，用于确定地面点

        // seg.setInputCloud(input_cloud);
        // seg.segment(*inliers, *coefficients);

        // // 根据地面平面方程移除地面点云
        // pcl::ExtractIndices<pcl::PointXYZ> extract;
        // extract.setInputCloud(input_cloud);
        // extract.setIndices(inliers);
        // extract.setNegative(true); // 保留非地面点
        // extract.filter(*obstacle_cloud);

        // 创建条件滤波器
        if(z_mode) {
            pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
            range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::GT, height_min)));
            range_condition->addComparison(pcl::FieldComparison<pcl::PointXYZ>::ConstPtr(
                new pcl::FieldComparison<pcl::PointXYZ>("z", pcl::ComparisonOps::LT, height_max)));

            pcl::ConditionalRemoval<pcl::PointXYZ> cond_removal;
            cond_removal.setInputCloud(input_cloud);
            cond_removal.setCondition(range_condition);
            cond_removal.setKeepOrganized(true); // 保持点云的有序性
            cond_removal.filter(*obstacle_cloud);
        } else {
            // 创建分割器
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            
            // 设置分割器参数
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setMaxIterations(max_iterations);
            seg.setDistanceThreshold(distance_threshold);
            
            // 执行地面分割
            seg.setInputCloud(input_cloud);
            seg.segment(*inliers, *coefficients);
            
            // 提取地面和障碍物点云
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(input_cloud);
            extract.setIndices(inliers);
            // extract.setNegative(false); // 提取地面点云
            // extract.filter(*ground_cloud);
            
            extract.setNegative(true); // 提取非地面点云（障碍物）
            extract.filter(*obstacle_cloud);
        }
    }

    void radiusFilterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
        // // 创建半径滤波器
        // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // outrem.setInputCloud(input_cloud);
        // outrem.setRadiusSearch(R_);
        // outrem.setMinNeighborsInRadius(10); // 可调整的参数，表示半径内至少有多少个邻近点才保留当前点
        // outrem.filter(*output_cloud);

        // // 创建一个滤波器，仅保留半径为R内的点
        // pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // for (const auto& point : input_cloud->points) {
        //     double distance = std::sqrt(std::pow(point.x - 0, 2) +
        //                                 std::pow(point.y - 0, 2));
        //     if (distance <= R_) {
        //         temp_cloud->points.push_back(point);
        //     }
        // }
        // *output_cloud = *temp_cloud;

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(input_cloud);

        output_cloud->clear();

        // 定义半径和最小邻近点数
        float radius = R_;
        int min_neighbors = 10;

        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        pcl::PointXYZ search_point; 
        search_point.x = 11;
        search_point.y = 8;
        search_point.z = 0;


        if (kdtree.radiusSearch(search_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
            for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
                if (pointIdxRadiusSearch[i] >= 0 && pointIdxRadiusSearch[i] < input_cloud->size()) {
                    output_cloud->points.push_back(input_cloud->points[pointIdxRadiusSearch[i]]);
                }
            }
        }

        output_cloud->width = output_cloud->points.size();
        output_cloud->height = 1;
        output_cloud->is_dense = true;
    }

    void preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& processed_cloud,
                            int mean_k, double std_dev, 
                            const pcl::PointXYZ& reference_point,  float max_distance, float min_distance, float leaf_size_base) {
        // 去除离群点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(mean_k);
        sor.setStddevMulThresh(std_dev);
        sor.filter(*cloud_filtered);

        // // Voxel Grid降采样
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::VoxelGrid<pcl::PointXYZ> vg;
        // vg.setInputCloud(cloud_filtered);
        // vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        // vg.filter(*cloud_downsampled);

        // // 返回处理后的点云
        // processed_cloud = cloud_downsampled;


        pcl::PointCloud<pcl::PointXYZI>::Ptr weighted_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            weighted_cloud->resize(cloud_filtered->size());

            // 计算每个点的权重
            for (size_t i = 0; i < cloud_filtered->size(); ++i) {
                // float distance = pcl::geometry::distance(cloud_filtered->points[i], reference_point);
                Eigen::Vector3d dis_3d(cloud_filtered->points[i].x - reference_point.x, 
                                       cloud_filtered->points[i].y - reference_point.y, 
                                       cloud_filtered->points[i].z - reference_point.z);
                float distance = dis_3d.norm();

                float weight;
                if (distance <= min_distance) {
                    weight = 1.0f;
                } else if (distance >= max_distance) {
                    weight = 0.0f;
                } else {
                    // 使用高斯函数作为权重函数
                    float sigma = (max_distance - min_distance) / 3.0f;
                    weight = std::exp(-0.5f * std::pow((distance - min_distance) / sigma, 2));
                }

                weighted_cloud->points[i].x = cloud_filtered->points[i].x ;
                weighted_cloud->points[i].y = cloud_filtered->points[i].y ;
                weighted_cloud->points[i].z = cloud_filtered->points[i].z ;
                weighted_cloud->points[i].intensity = weight;
            }

            // 根据权重进行体素滤波
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::VoxelGrid<pcl::PointXYZI> vg;
            vg.setInputCloud(weighted_cloud);
            
            // 根据权重调整 leaf size
            float leaf_size = leaf_size_base * (max_distance - min_distance);
            vg.setLeafSize(leaf_size, leaf_size, leaf_size);

            vg.filter(*cloud_filtered2);

            // processed_cloud = cloud_filtered2;
            pcl::copyPointCloud(*cloud_filtered2, *processed_cloud);
    }

    void preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& processed_cloud,
                            int mean_k, double std_dev, 
                            const pcl::PointXYZ& reference_point, 
                            float threshold1, float threshold2,
                            float leaf_size1, float leaf_size2, float leaf_size3) {
        // 去除离群点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud);
        sor.setMeanK(mean_k);
        sor.setStddevMulThresh(std_dev);
        sor.filter(*cloud_filtered);

        // // Voxel Grid降采样
        // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        // pcl::VoxelGrid<pcl::PointXYZ> vg;
        // vg.setInputCloud(cloud_filtered);
        // vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        // vg.filter(*cloud_downsampled);
        // // 返回处理后的点云
        // processed_cloud = cloud_downsampled;

        // 初始化三个点云以存储不同距离范围的点
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_near(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mid(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_far(new pcl::PointCloud<pcl::PointXYZ>);

        // 遍历输入点云，根据点到 reference_point 的距离将点分类到三个点云之一
        for (const auto& point : *cloud_filtered) {
                Eigen::Vector3d dis_3d(point.x - reference_point.x, 
                                       point.y - reference_point.y, 
                                       point.z - reference_point.z);
                float distance = dis_3d.norm();
            if (distance < threshold1) {
                cloud_near->push_back(point);
            } else if (distance < threshold2) {
                cloud_mid->push_back(point);
            } else {
                cloud_far->push_back(point);
            }
        }

        // 对每个点云应用体素滤波，使用不同的 leaf_size
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZ>);

        // 近距离点云滤波
        vg.setInputCloud(cloud_near);
        vg.setLeafSize(leaf_size1, leaf_size1, leaf_size1);
        vg.filter(*cloud_filtered2);
        *processed_cloud += *cloud_filtered2;

        // 中距离点云滤波
        cloud_filtered2->clear(); // 清空过滤后的点云以复用
        vg.setInputCloud(cloud_mid);
        vg.setLeafSize(leaf_size2, leaf_size2, leaf_size2);
        vg.filter(*cloud_filtered2);
        *processed_cloud += *cloud_filtered2;

        // 远距离点云滤波
        cloud_filtered2->clear(); // 清空过滤后的点云以复用
        vg.setInputCloud(cloud_far);
        vg.setLeafSize(leaf_size3, leaf_size3, leaf_size3);
        vg.filter(*cloud_filtered2);
        *processed_cloud += *cloud_filtered2;
    }
    
    void preprocessPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud2,
                            pcl::PointCloud<pcl::PointXYZ>::Ptr& downsampled_cloud,
                            int mean_k, double std_dev, 
                            const pcl::PointXYZ& reference_point, 
                            const std::vector<float>& thresholds, // N-1 个距离阈值
                            const std::vector<float>& leaf_sizes) {  // N 个 leaf_size
        // 去除离群点
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(input_cloud2);
        sor.setMeanK(mean_k);
        sor.setStddevMulThresh(std_dev);
        sor.filter(*input_cloud);
        // input_cloud = input_cloud2;

        if (thresholds.size() + 1 != leaf_sizes.size()) {
            // 错误处理: 阈值和 leaf_sizes 的数量不匹配
            std::cerr << "Error: The number of thresholds plus one must equal the number of leaf_sizes." << std::endl;
            return;
        }

        // 初始化 N 个点云以存储不同距离范围的点
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds(thresholds.size() + 1);
        for (auto& cloud : clouds) {
            cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        }

        // 遍历输入点云，根据点到 reference_point 的距离将点分配到相应的点云中
        for (const auto& point : *input_cloud) {
            Eigen::Vector3d dis_3d(point.x - reference_point.x, 
                                    point.y - reference_point.y, 
                                    0);
            float distance = dis_3d.norm();
            // 找到第一个大于当前距离的阈值的位置，以确定点属于哪个距离范围
            auto it = std::lower_bound(thresholds.begin(), thresholds.end(), distance);
            int idx = std::distance(thresholds.begin(), it);
            clouds[idx]->push_back(point);
        }

        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

        // 对每个距离范围的点云应用体素滤波
        for (size_t i = 0; i < clouds.size(); ++i) {
            cloud_filtered->clear(); // 清空过滤后的点云以复用
            vg.setInputCloud(clouds[i]);
            vg.setLeafSize(leaf_sizes[i], leaf_sizes[i], leaf_sizes[i]);
            vg.filter(*cloud_filtered);
            *downsampled_cloud += *cloud_filtered;
        }

    }

    void segmentAndExtractPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud) {
        // 平面分割
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(input_cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.size() == 0) {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
            // 处理失败情况
        }

        // 提取平面点云
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());
        extract.setInputCloud(input_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloud_plane);

        output_cloud = cloud_plane;
    }


private:
    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Subscriber robot_pos_sub_;
    ros::Publisher obstacle_pub_;
    double R_; // 圆的半径

};



int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detection_node");

    ObstacleDetectionNode node;
    robot_pos_.x = 11.0;
    robot_pos_.y = 8.0;
    robot_pos_.z = 0.0;

    ros::spin();

    return 0;
}
