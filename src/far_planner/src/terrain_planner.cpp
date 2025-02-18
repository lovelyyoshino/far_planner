/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */

 #include "far_planner/terrain_planner.h"

 /***************************************************************************************/
 
 /**
  * @brief 初始化地形规划器
  * 
  * 此函数用于初始化TerrainPlanner，设置相关参数，并为地形网格分配内存。
  * 
  * @param nh ROS节点句柄
  * @param params 地形规划参数
  */
 void TerrainPlanner::Init(const ros::NodeHandle& nh, const TerrainPlannerParams& params) {
     nh_ = nh; // 保存节点句柄
     tp_params_ = params; // 保存参数
     // 根据局部规划范围和体素大小计算行列数
     row_num_ = std::ceil((FARUtil::kLocalPlanRange + tp_params_.radius) * 2.0f / tp_params_.voxel_size);
     col_num_ = row_num_; // 行数与列数相同
 
     TerrainNodePtr init_terrain_node_ptr = NULL;
     // 设置网格大小、原点及分辨率
     Eigen::Vector3i grid_size(row_num_, col_num_, 1);
     Eigen::Vector3d grid_origin(0,0,0);
     Eigen::Vector3d grid_resolution(tp_params_.voxel_size, tp_params_.voxel_size, tp_params_.voxel_size);
     
     // 创建地形网格
     terrain_grids_ = std::make_unique<grid_ns::Grid<TerrainNodePtr>>(grid_size, init_terrain_node_ptr, grid_origin, grid_resolution, 3);
     this->AllocateGridNodes(); // 分配网格节点
     viz_path_stack_.clear(); // 清空可视化路径栈
 
     // 定义ROS话题发布者
     local_path_pub_   = nh_.advertise<Marker>("/local_terrain_path_debug", 5);
     terrain_map_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("/local_terrain_map_debug", 5);
 }
 
 /**
  * @brief 更新中心节点
  * 
  * 将传入的导航节点设为新的中心节点，并重置网格位置和占用状态。
  * 
  * @param node_ptr 当前的导航节点指针
  */
 void TerrainPlanner::UpdateCenterNode(const NavNodePtr& node_ptr) {
     if (node_ptr == NULL) return; // 如果传入节点为空则返回
     center_node_prt_ = node_ptr, center_pos_ = node_ptr->position; // 更新中心节点及其位置
     
     // 计算新网格原点
     Eigen::Vector3d grid_origin;
     grid_origin.x() = center_pos_.x - (tp_params_.voxel_size * row_num_) / 2.0f;
     grid_origin.y() = center_pos_.y - (tp_params_.voxel_size * col_num_) / 2.0f;
     grid_origin.z() = center_pos_.z - (tp_params_.voxel_size * 1.0f) / 2.0f;
     
     terrain_grids_->SetOrigin(grid_origin); // 设置网格原点
     is_grids_init_ = true; // 标记网格已初始化
     this->ResetGridsPositions(); // 重置网格位置
     this->ResetGridsOccupancy(); // 重置网格占用状态
 }
 
 /**
  * @brief 设置局部地形障碍物点云
  * 
  * 输入点云并将其加入到地形网格中。会根据膨胀大小更新网格的占用状态。
  * 
  * @param obsCloudIn 输入的障碍物点云
  */
 void TerrainPlanner::SetLocalTerrainObsCloud(const PointCloudPtr& obsCloudIn) {
     if (!is_grids_init_ || obsCloudIn->empty()) return; // 检查是否已初始化以及点云是否为空
     const int N_IF = tp_params_.inflate_size; // 膨胀大小
     for (const auto& point : obsCloudIn->points) { // 遍历点云中的每个点
         Eigen::Vector3i c_sub = terrain_grids_->Pos2Sub(Eigen::Vector3d(point.x, point.y, center_pos_.z)); // 获取当前点在网格中的索引
         for (int i = -N_IF; i <= N_IF; i++) { // 按照膨胀大小更新周围网格的占用状态
             for (int j = -N_IF; j <= N_IF; j++) {
                 Eigen::Vector3i sub;
                 sub.x() = c_sub.x() + i, sub.y() = c_sub.y() + j, sub.z() = 0;
                 if (terrain_grids_->InRange(sub)) { // 检查子索引是否在有效范围内
                     const int ind = terrain_grids_->Sub2Ind(sub); // 转换成线性索引
                     terrain_grids_->GetCell(ind)->is_occupied = true; // 设置网格单元为占用状态
                 }
             }
         }
     }
     /* 可视化地形规划地图和路径 */
     this->GridVisualCloud();
 }
 
 /**
  * @brief 可视化地形网格
  * 
  * 该函数生成一个包含所有自由空间的点云消息，用于显示地形网格的状态。
  */
 void TerrainPlanner::GridVisualCloud() {
     if (!is_grids_init_) return; // 确保网格已初始化
     PointCloudPtr temp_cloud_ptr(new pcl::PointCloud<PCLPoint>());
     const int N = terrain_grids_->GetCellNumber(); // 获取网格单元数量
     for (int ind=0; ind<N; ind++) {
         if (!terrain_grids_->GetCell(ind)->is_occupied) { // 判断是否是自由空间
             temp_cloud_ptr->points.push_back(this->Ind2PCLPoint(ind)); // 添加点信息至临时点云
         }
     }
     sensor_msgs::PointCloud2 msg_pc;
     pcl::toROSMsg(*temp_cloud_ptr, msg_pc); // 转换为ROS消息
     msg_pc.header.frame_id = tp_params_.world_frame; // 设置坐标帧ID
     msg_pc.header.stamp = ros::Time::now(); // 设置时间戳
     terrain_map_pub_.publish(msg_pc); // 发布可视化消息
 }
 
 /**
  * @brief 从一个点到另一个点进行路径规划
  * 
  * 使用A*算法在地形网格中从起点（from_p）到终点（to_p）进行路径规划。
  * 
  * @param from_p 起始点
  * @param to_p 目标点
  * @param path 存储生成的路径点堆栈
  * @return bool 路径规划成功返回true，否则返回false
  */
 bool TerrainPlanner::PlanPathFromPToP(const Point3D& from_p, const Point3D& to_p, PointStack& path) {
     path.clear(); // 清除已有路径
     if (!is_grids_init_) return false; // 检查网格是否已初始化
     
     this->ResetGridsPlanStatus(); // 重置网格计划状态
 
     const Eigen::Vector3d start_pos(from_p.x, from_p.y, center_pos_.z); // 起点位置
     const Eigen::Vector3d end_pos(to_p.x, to_p.y, center_pos_.z); // 终点位置
     const Eigen::Vector3i start_sub = terrain_grids_->Pos2Sub(start_pos); // 获取起点索引
     const Eigen::Vector3i end_sub   = terrain_grids_->Pos2Sub(end_pos); // 获取终点索引
     
     
     // 检查起点和终点是否在有效范围内
     if (!terrain_grids_->InRange(start_sub) || !terrain_grids_->InRange(end_sub)) {
         if (FARUtil::IsDebug) ROS_WARN("TP: two interval navigation nodes are not in terrain planning range.");
         return false; // 返回失败
     }
 
     const TerrainNodePtr start_node_ptr = terrain_grids_->GetCell(terrain_grids_->Sub2Ind(start_sub)); // 获取起点节点
     const TerrainNodePtr end_node_ptr = terrain_grids_->GetCell(terrain_grids_->Sub2Ind(end_sub)); // 获取终点节点
     const Point3D unit_axial = (to_p - from_p).normalize(); // 归一化方向向量
     const float ndist = (to_p - from_p).norm(); // 两点间距离
 
     // Lambda函数定义：检测节点是否在柱状区域内
     auto InCylinder = [&](const TerrainNodePtr& tnode_ptr) {
         const Point3D vec = tnode_ptr->position - from_p; // 节点与起点的向量
         float proj_scalar = vec * unit_axial; // 投影标量
         if (proj_scalar < - FARUtil::kNavClearDist || proj_scalar > ndist + FARUtil::kNavClearDist) {
             return false; // 不在范围内
         }
         const Point3D vec_axial = unit_axial * proj_scalar; 
         if ((vec - vec_axial).norm() > tp_params_.radius) {
             return false; // 超出半径范围
         }
         return true; // 在柱状范围内
     };
 
     // 定义邻居方向数组
     std::array<int, 8> dx = {1, 1, 0,-1,-1, 0, 1,-1};
     std::array<int, 8> dy = {0, 1, 1, 0, 1,-1,-1,-1};
 
     // A*算法初始化
     start_node_ptr->gscore = 0.0; // G值初始化
     std::unordered_set<int> open_set; // 开放集
     std::priority_queue<TerrainNodePtr, std::vector<TerrainNodePtr>, TNodeptr_fcomp> open_queue; // 优先队列
     std::unordered_set<int> close_set; // 闭合集
 
     // 将起点添加到开放集中
     open_queue.push(start_node_ptr);
     open_set.insert(start_node_ptr->id);
 
     while (true) {
         if (open_set.empty()) { // 如果开放集为空，则停止
             break;
         }
         const TerrainNodePtr current = open_queue.top(); // 找到最低F值的节点
         if (current == end_node_ptr) { // 如果找到终点
             this->ExtractPath(end_node_ptr, path); // 提取路径
             break;
         }
         open_queue.pop(); // 移除当前节点
         open_set.erase(current->id); // 更新开放集
         close_set.insert(current->id); // 加入闭合集
         
         // 遍历相邻节点
         for (int i=0; i<8; i++) {
             Eigen::Vector3i csub = terrain_grids_->Ind2Sub(current->id);
             csub.x() += dx[i], csub.y() += dy[i], csub.z() = 0; // 更新相邻节点索引
             
             if (!terrain_grids_->InRange(csub)) continue; // 检查是否越界
             const TerrainNodePtr neighbor = terrain_grids_->GetCell(terrain_grids_->Sub2Ind(csub)); // 获取邻居节点
             
             // 检查闭合集和占用状态
             if (close_set.count(neighbor->id) || (neighbor->is_occupied && neighbor != end_node_ptr) || !InCylinder(neighbor)) continue;
 
             // 更新G值
             const float temp_gscore = current->gscore + this->EulerCost(current, neighbor);
             if (temp_gscore < neighbor->gscore) { // 如果找到更优路径
                 neighbor->parent = current; // 记录父节点
                 neighbor->gscore = temp_gscore; // 更新G值
                 neighbor->fscore = temp_gscore + this->EulerCost(neighbor, end_node_ptr); // 更新F值
                 
                 // 如果邻居不在开放集中，就添加进去
                 if (!open_set.count(neighbor->id)) {
                     open_queue.push(neighbor);
                     open_set.insert(neighbor->id);
                 } 
             }
         }
     }
     if (!path.empty()) { // 如果路径非空
         viz_path_stack_.push_back(path); // 将路径存入可视化路径栈
     }
     return !path.empty(); // 返回是否成功得到了路径
 }
 
 /**
  * @brief 从结束节点提取路径
  * 
  * 从给定的结束节点往回追溯，形成完整的路径。
  * 
  * @param end_ptr 结束节点
  * @param path 存储生成的路径点堆栈
  */
 void TerrainPlanner::ExtractPath(const TerrainNodePtr& end_ptr, PointStack& path) {
     path.clear(); // 清空路径
     TerrainNodePtr cur_ptr = end_ptr; // 当前指针指向结束节点
     while (true) {
         path.push_back(cur_ptr->position); // 将当前位置加入路径
         if (cur_ptr->parent != NULL) { // 如果有父节点
             cur_ptr = cur_ptr->parent; // 向上追溯
         } else {
             break; // 到达起始节点
         }
     }
     std::reverse(path.begin(), path.end()); // 翻转路径，变为从起点到终点
 }
 
 /**
  * @brief 可视化生成的路径
  * 
  * 该函数使用Marker类型的数据生成可视化路线图，将多个路径以Orange颜色绘制出来。
  */
 void TerrainPlanner::VisualPaths() {
     Marker terrain_paths_marker; // 创建路径标记对象
     terrain_paths_marker.type = Marker::LINE_LIST; // 设置标记类型
     DPVisualizer::SetMarker(VizColor::ORANGE, "terrain_path", 0.3f, 0.85f, terrain_paths_marker); // 设置颜色和其他属性
     
     auto DrawPath = [&](const PointStack& path) {
         if (path.size() < 2) return; // 小于两点无效
         geometry_msgs::Point last_p = FARUtil::Point3DToGeoMsgPoint(path[0]); // 获取第一个点
         for (int i=1; i<path.size(); i++) { // 遍历路径中的所有点
             terrain_paths_marker.points.push_back(last_p); // 将上一个点加入列表
             last_p = FARUtil::Point3DToGeoMsgPoint(path[i]); // 更新最后一点
             terrain_paths_marker.points.push_back(last_p); // 将当前点加入列表
         }
     };
 
     for (const auto& tpath : viz_path_stack_) { // 遍历所有路径
         DrawPath(tpath); // 绘制路径
     }
     local_path_pub_.publish(terrain_paths_marker); // 发布路径可视化信息
     viz_path_stack_.clear(); // 清空可视化路径栈
 } 