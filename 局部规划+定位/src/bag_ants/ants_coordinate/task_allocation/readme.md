<!--
 * @Author: Chenyang Zhang && 1727326672@qq.com
 * @Date: 2024-03-24 13:46:57
 * @LastEditors: Chenyang Zhang && 1727326672@qq.com
 * @LastEditTime: 2024-03-25 19:53:28
 * @FilePath: /bit_zcy_001/src/bag_ants/ants_coordinate/task_allocation/readme.md
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
-->
# 1、本功能包的目的是通过H网格对多车探索任务进行划分

# 2、为了实现该目的，小步策略
## 2.1、第一步：写一个节点调用地图，接入仿真系统，观察地图生成的正确性
* 同时研究一下该包的多地图策略

## 2.2、第二步：节点中加入H网格类，对地图划分
1、接入雷达数据，创建地图；
2、创建H网格；
3、编写一个利用H网格的函数，findGlobalTour，在该函数中引入TSP求解器，求解网格访问顺序；
    3.1、接受超级视点列表，填充前沿视点
4、可视化网格和网格访问顺序。

## 2.3、第三步：考虑对H网格类修改，能接入前沿探索
* 难点1：多面体空间和地图的融合
* 难点2：H网格如何兼容
## 2.4、第四步：在全局网格划分的基础上，加入单车的全局TSP规划
## 2.5、第五步：加入成对优化策略，进行任务的划分