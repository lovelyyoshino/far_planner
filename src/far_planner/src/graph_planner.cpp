/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



 #include "far_planner/graph_planner.h"

 /***************************************************************************************/
 
 const char INIT_BIT = char(0); // 初始化状态位，0000
 const char OBS_BIT  = char(1); // 障碍物状态位，0001
 const char FREE_BIT = char(2); // 可通行状态位，0010
 
 // 初始化图规划器
 void GraphPlanner::Init(const ros::NodeHandle& nh, const GraphPlannerParams& params) {
     nh_ = nh; // 保存节点句柄
     gp_params_ = params; // 保存参数配置
     is_goal_init_ = false; // 确定目标是否初始化
     current_graph_.clear(); // 清空当前图形
     // 尝试规划监听者
     attemptable_sub_ = nh_.subscribe("/planning_attemptable", 5, &GraphPlanner::AttemptStatusCallBack, this);
     
     // 初始化地形网格
     const int col_num = std::ceil(gp_params_.adjust_radius * 2.0f / FARUtil::kLeafSize); // 列数量
     Eigen::Vector3i grid_size(col_num, col_num, 1); // 网格大小定义
     Eigen::Vector3d grid_origin(0,0,0); // 网格原点
     Eigen::Vector3d grid_resolution(FARUtil::kLeafSize, FARUtil::kLeafSize, FARUtil::kLeafSize); // 分辨率
     free_terrain_grid_ = std::make_unique<grid_ns::Grid<char>>(grid_size, INIT_BIT, grid_origin, grid_resolution, 3); // 创建自由地形网格
 }
 
 // 更新图中可遍历性，从当前位置（odom_node_ptr）开始更新所有导航节点的 fscore
 void GraphPlanner::UpdateGraphTraverability(const NavNodePtr& odom_node_ptr, const NavNodePtr& goal_ptr) {
     if (odom_node_ptr == NULL || current_graph_.empty()) { // 检查输入有效性
         ROS_ERROR("GP: Update global graph traversablity fails.");
         return;
     }
 
     odom_node_ptr_ = odom_node_ptr; // 当前里程计位置设置
     this->InitNodesStates(current_graph_); // 初始化节点状态
     odom_node_ptr_->gscore = 0.0; // 设置起始节点 gscore 为 0
     IdxSet open_set; 
     std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_gcomp> open_queue; // 基于 gscore 大小排列的优先队列
     IdxSet close_set; 
 
     // 从里程计节点扩展到所有可达的导航节点
     open_queue.push(odom_node_ptr_);
     open_set.insert(odom_node_ptr_->id);
 
     while (!open_set.empty()) {
         const NavNodePtr current = open_queue.top(); // 获取当前节点
         open_queue.pop();
         open_set.erase(current->id);
         close_set.insert(current->id); // 将当前节点加入闭合集
         current->is_traversable = true; // 当前节点可以遍历
 
         for (const auto& neighbor : current->connect_nodes) {
             // 跳过已经在闭合集中或者不合法的邻居节点
             if (close_set.count(neighbor->id) || this->IsInvalidBoundary(current, neighbor)) continue;
 
             float edist = this->EulerCost(current, neighbor); // 计算两点间的实际距离
             
             // 检测是否多层遍历成本，如果是连接目标节点且满足条件进行处理
             if (neighbor == goal_ptr && edist > FARUtil::kEpsilon && !FARUtil::IsAtSameLayer(neighbor, current)) {
                 const Point3D diff_p = neighbor->position - current->position;
                 float factor = std::hypotf(diff_p.x, diff_p.y) / edist;
                 if (factor > FARUtil::kEpsilon) {
                     edist /= factor; // 调整edist
                 } else {
                     continue;
                 }
             }
 
             const float temp_gscore = current->gscore + edist; // 临时gscore计算
             if (temp_gscore < neighbor->gscore) { // 如果新的gscore更优
                 neighbor->parent = current; // 更新父节点
                 neighbor->gscore = temp_gscore; // 更新gscore
                 if (!open_set.count(neighbor->id)) { // 如果未在开启集合中，则添加
                     open_queue.push(neighbor);
                     open_set.insert(neighbor->id);
                 }
             }
         }
     }
     std::priority_queue<NavNodePtr, NodePtrStack, nodeptr_fgcomp> fopen_queue;
     IdxSet fopen_set;
     close_set.clear();
     // Expansion from odom node to all covered navigation node
     odom_node_ptr_->fgscore = 0.0;
     fopen_queue.push(odom_node_ptr_);
     fopen_set.insert(odom_node_ptr_->id);
     while (!fopen_set.empty()) {
         const NavNodePtr current = fopen_queue.top();
         fopen_queue.pop();
         fopen_set.erase(current->id);
         close_set.insert(current->id);
         current->is_free_traversable = true; // reachable from current position
         for (const auto& neighbor : current->connect_nodes) {
             if (!neighbor->is_covered || close_set.count(neighbor->id) || this->IsInvalidBoundary(current, neighbor)) continue;
             const float e_dist = this->EulerCost(current, neighbor);
             if (neighbor == goal_ptr && (!is_goal_in_freespace_ || e_dist > FARUtil::kTerrainRange)) continue;
             const float temp_fgscore = current->fgscore + e_dist;
             if (temp_fgscore < neighbor->fgscore) {
                 neighbor->free_parent = current;
                 neighbor->fgscore = temp_fgscore;
                 if (!fopen_set.count(neighbor->id)) {
                     fopen_queue.push(neighbor);
                     fopen_set.insert(neighbor->id);
                 } 
             }
         }
     }
 }
 
 // 更新目标导航节点的连接情况
 void GraphPlanner::UpdateGoalNavNodeConnects(const NavNodePtr& goal_ptr) {
     if (goal_ptr == NULL || is_use_internav_goal_) return; // 检查有效性和是否使用内部导航目标
     for (const auto& node_ptr : current_graph_) {
         if (node_ptr == goal_ptr) continue; // 避免自身连通检查
         
         // 根据判断模型记录投票或删除投票
         if (this->IsValidConnectToGoal(node_ptr, goal_ptr)) {
             const bool is_directly_connect = node_ptr->is_odom ? true : false;
             DynamicGraph::RecordPolygonVote(node_ptr, goal_ptr, gp_params_.votes_size, is_directly_connect); 
         } else {
             DynamicGraph::DeletePolygonVote(node_ptr, goal_ptr, gp_params_.votes_size);
         }
 
         const auto it = goal_ptr->edge_votes.find(node_ptr->id);
         // 根据边投票决定是否添加/删除边
         if (it != goal_ptr->edge_votes.end() && FARUtil::IsVoteTrue(it->second, false)) {
             DynamicGraph::AddPolyEdge(node_ptr, goal_ptr), DynamicGraph::AddEdge(node_ptr, goal_ptr);
             node_ptr->is_block_to_goal = false; // 表示该节点未阻塞目标
         } else {
             DynamicGraph::ErasePolyEdge(node_ptr, goal_ptr), DynamicGraph::EraseEdge(node_ptr, goal_ptr);
             node_ptr->is_block_to_goal = true; // 标记为阻塞目标
         }
     }
 }
 
 // 验证与目标的有效连接关系
 bool GraphPlanner::IsValidConnectToGoal(const NavNodePtr& node_ptr, const NavNodePtr& goal_node_ptr) {
     // 校验节点是否可遍历以及其阻塞状态
     if (node_ptr->is_traversable && (!node_ptr->is_block_to_goal || IsResetBlockStatus(node_ptr, goal_node_ptr))) {
         const Point3D diff_p = goal_node_ptr->position - node_ptr->position;
         // 判断是否存在凸性连结并校验方向性
         if (DynamicGraph::IsConvexConnect(node_ptr, goal_node_ptr) &&
             (!FARUtil::IsAtSameLayer(node_ptr, goal_node_ptr) || FARUtil::IsOutReducedDirs(diff_p, node_ptr)) &&
             ContourGraph::IsNavToGoalConnectFreePolygon(node_ptr, goal_node_ptr)) 
         {
             return true; // 有效连接返回真
         }
     }
     return false; // 否则返回假
 }
 
 // 获取到目标的路径，并执行必要动作，如成功、失败标志等
 bool GraphPlanner::PathToGoal(const NavNodePtr& goal_ptr,
                               NodePtrStack& global_path,
                               NavNodePtr& _nav_node_ptr,
                               Point3D& _goal_p,
                               bool& _is_fail,
                               bool& _is_succeed,
                               bool& _is_free_nav) {
     if (!is_goal_init_) return false; // 检查目标是否初始化
     
     // 检查基本前提条件
     if (odom_node_ptr_ == NULL || goal_ptr == NULL || current_graph_.empty()) {
         ROS_ERROR("GP: Graph or Goal is not initialized correctly.");
         return false;
     }
 
     _is_fail = false, _is_succeed = false;
     global_path.clear(); // 清空全局路径
     _goal_p = goal_ptr->position; // 设置目标位置
     
     // 特殊情况处理
     if (current_graph_.size() == 1) {
         global_path.push_back(odom_node_ptr_); // 更新全局路径
         global_path.push_back(goal_ptr);
         _nav_node_ptr = this->NextNavWaypointFromPath(global_path, goal_ptr);
         _is_free_nav = is_free_nav_goal_;
         return true;       
     }
     
     // 达成目标处理
     if ((odom_node_ptr_->position - _goal_p).norm() < gp_params_.converge_dist ||
         (odom_node_ptr_->position - origin_goal_pos_).norm() < gp_params_.converge_dist) 
     {
         if (FARUtil::IsDebug) ROS_INFO("GP: *********** Goal Reached! ***********");
         global_path.push_back(odom_node_ptr_);
         
         // 如果还未达到设定界限，则重置目标位置
         if ((odom_node_ptr_->position - _goal_p).norm() > gp_params_.converge_dist) {
             _goal_p = origin_goal_pos_; 
             goal_ptr->position = _goal_p;   
         }
 
         _is_succeed = true; // 成功标志
         global_path.push_back(goal_ptr);
         _nav_node_ptr = goal_ptr; // 更新导航节点
         _is_free_nav = is_free_nav_goal_;
         this->GoalReset(); // 重置目标
         is_goal_init_ = false;
         return true;
     }
 
     // 检查自由导航命令是否生效
     if (!command_is_free_nav_) is_free_nav_goal_ = false;
     else if (goal_ptr->is_free_traversable) is_free_nav_goal_ = true;
     
     // 自动切换模型
     if (gp_params_.is_autoswitch && command_is_free_nav_) {
         if (goal_ptr->is_free_traversable || (is_free_nav_goal_ && is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred)) {
             is_free_nav_goal_ = true; // 若符合条件保持自由导航模式
         } else {
             is_free_nav_goal_ = false; // 否则切换至其他模式
         }
     }   
     
     _is_free_nav = is_free_nav_goal_; // 更新自由导航标志
     // 查找能到达的上一个导航节点
     const NavNodePtr reach_nav_node = is_free_nav_goal_ ? goal_ptr->free_parent : goal_ptr->parent;
 
     // 找到有效路径
     if (reach_nav_node != NULL) {
         // 动量导航处理
         if (is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred && 
             last_waypoint_dist_ > gp_params_.adjust_radius && reach_nav_node != odom_node_ptr_)
         {   
             const float cur_waypoint_dist = (odom_node_ptr_->position - next_waypoint_).norm();
             if (cur_waypoint_dist > gp_params_.adjust_radius) {
                 if ((odom_node_ptr_->position - last_planning_odom_).norm() < gp_params_.momentum_dist) { // 移动动量验证
                     global_path = recorded_path_; // 使用记录下来的路径
                     _nav_node_ptr = this->NextNavWaypointFromPath(global_path, goal_ptr);
                     path_momentum_counter_ ++;
                     if (FARUtil::IsDebug) ROS_INFO_STREAM("Momentum path counter: " << path_momentum_counter_ << " Over max: "<< gp_params_.momentum_thred);
                     return true; // 返回成立
                 }
             }
         }
 
         NodePtrStack cur_path; // 存储当前路径
         if (this->ReconstructPath(goal_ptr, is_free_nav_goal_, cur_path)) {
             _nav_node_ptr = this->NextNavWaypointFromPath(cur_path, goal_ptr);
             // 再次进行动量导航处理
             if (is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred) {
                 const float cur_waypoint_dist = (odom_node_ptr_->position - _nav_node_ptr->position).norm();
                 // 是否动力学一致性
                 if (last_waypoint_dist_ > gp_params_.adjust_radius && cur_waypoint_dist > gp_params_.adjust_radius) {
                     const float heading_dot = (next_waypoint_ - last_planning_odom_).norm_dot(_nav_node_ptr->position - odom_node_ptr_->position);
                     if (heading_dot < 0.0f) { // 一致的航向动量
                         global_path = recorded_path_;
                         _nav_node_ptr = this->NextNavWaypointFromPath(global_path, goal_ptr);
                         path_momentum_counter_ ++;
 
                         if (FARUtil::IsDebug) ROS_INFO_STREAM("Momentum path counter: " << path_momentum_counter_ << "; Over max: "<< gp_params_.momentum_thred);
                         return true;
                     }
                 }
             } 
             // 计划到目标的新路径
             global_path = cur_path; 
             this->RecordPathInfo(global_path); // 记录路径信息
             return true;
         }
     } else { // 未找到有效路径的情况下
         if (is_global_path_init_ && path_momentum_counter_ < gp_params_.momentum_thred) { // 向前的动量继续前进
             global_path = recorded_path_;
             _nav_node_ptr = this->NextNavWaypointFromPath(global_path, goal_ptr);
             path_momentum_counter_ ++;
             if (FARUtil::IsDebug) ROS_INFO_STREAM("Momentum path counter: " << path_momentum_counter_ << "; Over max: "<< gp_params_.momentum_thred);
             return true;
         } else {
             if (gp_params_.is_autoswitch && is_free_nav_goal_) { // 在自由导航失败后自动切换
                 if (FARUtil::IsDebug) ROS_WARN("GP: free navigation fails, auto switching to attemptable navigation...");
                 if (is_global_path_init_) {
                     global_path = recorded_path_; // 回退到之前的路径
                     _nav_node_ptr = this->NextNavWaypointFromPath(global_path, goal_ptr);
                 } else {
                     _nav_node_ptr = goal_ptr; // 此时直接指向目标
                 }
                 is_free_nav_goal_ = false; // 关闭自由导航状态
                 return true;
             }
             if (FARUtil::IsDebug) ROS_ERROR("****************** FAIL TO REACH GOAL ******************");
             this->GoalReset(); // 目标重置
             is_goal_init_ = false, _is_fail = true; // 标识失败
             return false;
         }
     }
     if (FARUtil::IsDebug) ROS_ERROR("GP: unexpected error happened within planning, navigation to goal fails.");
     this->GoalReset(); // 出现意外错误则重置目标
     is_goal_init_ = false, _is_fail = true; // 标识失败
     return false;
 }
 
 // 重构从目标节点到起始节点的路径
 bool GraphPlanner::ReconstructPath(const NavNodePtr& goal_node_ptr,
                                    const bool& is_free_nav,
                                    NodePtrStack& global_path) {
     if (goal_node_ptr == NULL || (!is_free_nav && goal_node_ptr->parent == NULL) || (is_free_nav && goal_node_ptr->free_parent == NULL)) {
         ROS_ERROR("GP: Critical! reconstruct path error: goal node or its parent equals to NULL."); // 检查输入有效性
         return false;
     }
 
     global_path.clear(); // 清空输出路径栈
     NavNodePtr check_ptr = goal_node_ptr; // 用来迭代检查目标节点
     global_path.push_back(check_ptr);
 
     if (is_free_nav) {
         // 若是在自由导航模式下
         while (true) {
             const NavNodePtr parent_ptr = check_ptr->free_parent; // 获取自由父节点
             if (parent_ptr->free_direct != NodeFreeDirect::CONCAVE) {
                 global_path.push_back(parent_ptr); // 不在凹陷区则纳入路径
             }
             if (parent_ptr->free_parent == NULL) break; // 无法再回溯了
             check_ptr = parent_ptr; // 转到下一层级
         }
     } else {
         // 若不是自由导航模式
         while (true) {
             const NavNodePtr parent_ptr = check_ptr->parent; // 获取普通父节点
             if (parent_ptr->free_direct != NodeFreeDirect::CONCAVE) {
                 global_path.push_back(parent_ptr); // 同理只要不在凹陷区就是合规
             }
             if (parent_ptr->parent == NULL) break; // 无法再回溯了
             check_ptr = parent_ptr; // 转到另一层级
         } 
     }
     std::reverse(global_path.begin(), global_path.end()); // 翻转路径，使之从开头显示
     return true;
 }
 // 从全局路径中获取下一个导航航点
 NavNodePtr GraphPlanner::NextNavWaypointFromPath(const NodePtrStack& global_path, const NavNodePtr goal_ptr) {
     // 检查全局路径的大小是否小于2
     if (global_path.size() < 2) {
         ROS_ERROR("GP: global path size less than 2.");
         return goal_ptr;  // 返回目标指针
     }
 
     NavNodePtr nav_point_ptr;  
     const std::size_t path_size = global_path.size(); // 获取路径的大小
     std::size_t nav_idx = 1;  // 设置第一个导航索引
 
     nav_point_ptr = global_path[nav_idx]; // 指向路径中的第一个可导航点
     float dist = (nav_point_ptr->position - odom_node_ptr_->position).norm(); // 计算当前位置与导航点之间的距离
 
     // 当与目标点的距离小于收敛距离时，继续移动到下一个导航点
     while (dist < gp_params_.converge_dist) { 
         nav_idx++;
         if (nav_idx < path_size) { // 如果导航索引依然在范围内
             nav_point_ptr = global_path[nav_idx]; // 更新导航点
             dist = (nav_point_ptr->position - odom_node_ptr_->position).norm(); // 重新计算距离
         } else break; // 超出范围则退出循环
     }
     return nav_point_ptr; // 返回选择的导航点
 }
 
 // 更新当前目标位置
 void GraphPlanner::UpdateGoal(const Point3D& goal) {
     this->GoalReset(); // 重置目标状态
     is_use_internav_goal_ = false; // 标记不使用内部导航目标
     float min_dist = FARUtil::kNearDist; // 初始化最小距离
 
     // 遍历当前图中的所有节点
     for (const auto& node_ptr : current_graph_) {
         node_ptr->is_block_to_goal = false; // 将节点标记为可以到达目标
 
         if (node_ptr->is_navpoint) { // 如果该节点为导航点
             const float cur_dist = (node_ptr->position - goal).norm(); // 计算目标与导航点之间的距离
             if (cur_dist < min_dist) { // 如果发现更近的导航点
                 is_use_internav_goal_   = true; // 标记为使用内部导航目标
                 goal_node_ptr_          = node_ptr; // 更新目标节点指针
                 min_dist                = cur_dist; // 更新最小距离
                 goal_node_ptr_->is_goal = true; // 标记此节点为目标
             }
         }
     }
 
     // 若没有合适的导航点，则从给定目标创建新节点
     if (!is_use_internav_goal_) {
         DynamicGraph::CreateNavNodeFromPoint(goal, goal_node_ptr_, false, false, true); // 创建新的导航点
         DynamicGraph::AddNodeToGraph(goal_node_ptr_); // 将其添加到图中
     }
 
     // 调试信息输出
     if (FARUtil::IsDebug) ROS_INFO("GP: *********** new goal updated ***********");
     
     // 更新状态初始化标志
     is_goal_init_          = true; 
     is_global_path_init_   = false;
     is_terrain_associated_ = false;
     origin_goal_pos_       = goal_node_ptr_->position; // 原始目标位置
     is_free_nav_goal_      = command_is_free_nav_; // 自由导航目标标志
     next_waypoint_         = Point3D(0,0,0); // 下一个航点初始化
     last_waypoint_dist_    = 0.0f; // 上一个航点距离初始化
     last_planning_odom_    = Point3D(0,0,0); // 上一计划位置初始化
 
     path_momentum_counter_ = 0; // 路径动量计数器重置
     recorded_path_.clear(); // 清空记录的路径
     
     // 若非多层设置，依据地形调整目标高度
     if (!FARUtil::IsMultiLayer) {
         goal_node_ptr_->position.z = MapHandler::NearestTerrainHeightofNavPoint(origin_goal_pos_, is_terrain_associated_) + FARUtil::vehicle_height; 
     }
 
     this->ResetFreeTerrainGridOrigin(goal_node_ptr_->position); // 重置自由地形网格原点
 }
 
 // 重新评估目标位置
 void GraphPlanner::ReEvaluateGoalPosition(const NavNodePtr& goal_ptr, const bool& is_adjust_height)
 {
     if (is_use_internav_goal_) return; // 若正在使用现有内部导航节点做目标，则返回
 
     // 根据路径调整目标高度（若允许）
     if (is_adjust_height && is_global_path_init_ && recorded_path_.size() > 1) { 
         const auto it = recorded_path_.end() - 2; 
         if (!is_terrain_associated_) {
             goal_ptr->position.z = (*it)->position.z; // 使用路径的 z 坐标作为目标z坐标
         } else if ((*it)->is_odom) {
             goal_ptr->position.z = (*it)->position.z; // 使用里程计的 z 坐标
         }
         
     }
 
     // 确认目标位置是否是可通行区域
     const Eigen::Vector3i ori_sub = free_terrain_grid_->Pos2Sub(origin_goal_pos_.x, origin_goal_pos_.y, grid_center_.z);
     const Point3D ori_pos_height(origin_goal_pos_.x, origin_goal_pos_.y, goal_ptr->position.z);
 
     // 检查起始位置是否能够连接至无障碍区域
     const bool is_origin_free = ContourGraph::IsPoint3DConnectFreePolygon(ori_pos_height, odom_node_ptr_->position);
     if (is_origin_free) {
         goal_ptr->position = ori_pos_height; // 如果起始位置是自由空间，直接赋值
     } else { // 否则，寻找附近的自由区进行重新投影
         std::array<int, 4> dx = {-1, 0, 1, 0}; // 四个方向偏移
         std::array<int, 4> dy = { 0, 1, 0,-1};
         std::deque<int> q; // 队列用于 BFS 遍历
         std::unordered_set<int> visited_set; // 访问过的节点集合
         q.push_back(free_terrain_grid_->Sub2Ind(ori_sub)); // 入队当前坐标
         visited_set.insert(free_terrain_grid_->Sub2Ind(ori_sub));
 
         int valid_idx = -1; // 有效索引初始化
         while (!q.empty()) {
             const int cur_id = q.front();
             q.pop_front();
 
             // 找到第一个无障碍单元
             if (free_terrain_grid_->GetCell(cur_id) == FREE_BIT) {
                 valid_idx = cur_id; // 更新有效索引
                 break;
             }
             
             // 扩展搜索邻居节点
             for (int i=0; i<4; i++) {
                 Eigen::Vector3i csub = free_terrain_grid_->Ind2Sub(cur_id);
                 csub.x() += dx[i], csub.y() += dy[i], csub.z() = 0; // 更新坐标
                 if (!free_terrain_grid_->InRange(csub)) continue; // 检查边界
                 
                 const int cidx = free_terrain_grid_->Sub2Ind(csub);
                 if (!visited_set.count(cidx)) { // 未被访问过
                     q.push_back(cidx); // 入队
                     visited_set.insert(cidx); // 标记为已访问
                 }
             }
         }
 
         // 如果找到了有效的位置且在区域内
         if (valid_idx != -1 && free_terrain_grid_->InRange(valid_idx)) {
             Point3D new_p = Point3D(free_terrain_grid_->Ind2Pos(valid_idx)); // 新的随机点
             new_p.z = goal_ptr->position.z;
 
             const float pred = (goal_ptr->position - ori_pos_height).norm(); // 预测距离
             const float curd = (new_p - ori_pos_height).norm(); // 当前找到的新距离
             
             // 偏差大于阈值并且不发生碰撞则更新目标位置
             if (abs(curd - pred) > FARUtil::kLeafSize && !ContourGraph::IsEdgeCollideBoundary(goal_ptr->position, new_p)) { 
                 if (FARUtil::IsDebug) ROS_INFO_THROTTLE(1.0, "GP: adjusting goal into free space.");
                 goal_ptr->position = new_p; // 更新目标位置
             }
         } 
         
         // 检查是否在本地范围之内
         if (FARUtil::IsNodeInLocalRange(goal_ptr)) {
             Point3D current_goal_pos = goal_ptr->position;
             if (ContourGraph::ReprojectPointOutsidePolygons(current_goal_pos, FARUtil::kNearDist)) {
                 if (FARUtil::IsDebug) {
                     const float reproject_dist = (current_goal_pos - origin_goal_pos_).norm_flat();
                     ROS_WARN_THROTTLE(1.0, "GP: current goal is inside polygon, reproject goal position distance to origin goal: %f.", reproject_dist);
                 }
                 goal_ptr->position = current_goal_pos; // 更新目标位置
             }
         }
     }
 }
 
 // 尝试状态回调方法
 void GraphPlanner::AttemptStatusCallBack(const std_msgs::Bool& msg) {
     // 如果当前命令是自由导航而消息数据为真，即表示当前目标不可尝试
     if (command_is_free_nav_ && msg.data) { 
         if (FARUtil::IsDebug) ROS_WARN("GP: switch to attemptable planning mode."); 
         command_is_free_nav_ = false; // 切换到可尝试规划模式
         path_momentum_counter_ = gp_params_.momentum_thred; // 恢复动量阈值
     } 
 
     // 如果当前不可尝试且消息数据显示为假，表示进入当前可尝试规划
     if (!command_is_free_nav_ && !msg.data) { 
         if (FARUtil::IsDebug) ROS_WARN("GP: planning without attempting."); 
         command_is_free_nav_ = true; // 切换到自由导航
         path_momentum_counter_ = gp_params_.momentum_thred; // 恢复动量阈值
     }
 }
 
 // 更新自由地形网格
 void GraphPlanner::UpdateFreeTerrainGrid(const Point3D& center,
                                          const PointCloudPtr& obsCloudIn, 
                                          const PointCloudPtr& freeCloudIn) 
 {
     // 重置网格
     const Point3D origin_center(origin_goal_pos_.x, origin_goal_pos_.y, center.z);
     this->ResetFreeTerrainGridOrigin(origin_center); // 重置原点角色
     free_terrain_grid_->ReInitGrid(INIT_BIT); // 重新初始化网格
     
     // 评估目标的自由空间状态
     is_goal_in_freespace_ = false;
     if (!freeCloudIn->empty() || !obsCloudIn->empty()) { // 准备处理地形云
         const int C_IF = FARUtil::kObsInflate; // 障碍物膨胀因子
         
         // 遍历观察云，将障碍物设置在自由地形网格中
         for (const auto& point : obsCloudIn->points) { 
             Eigen::Vector3i c_sub = free_terrain_grid_->Pos2Sub(point.x, point.y, grid_center_.z);
             for (int i = -C_IF; i <= C_IF; i++) {
                 for (int j = -C_IF; j <= C_IF; j++) {
                     Eigen::Vector3i sub = c_sub;
                     sub.x() += i, sub.y() += j, sub.z() = 0; // 获取周围障碍物位置
                     if (free_terrain_grid_->InRange(sub)) { // 检查是否超出边界
                         const int ind = free_terrain_grid_->Sub2Ind(sub);
                         free_terrain_grid_->GetCell(ind) = free_terrain_grid_->GetCell(ind) | OBS_BIT; // 设置为障碍物
                     }
                 }
             }
         }
 
         // 遍历自由云，将自由空间设置在自由地形网格中
         for (const auto& point : freeCloudIn->points) { 
             Eigen::Vector3i c_sub = free_terrain_grid_->Pos2Sub(point.x, point.y, grid_center_.z);
             for (int i = -1; i <= 1; i++) {
                 for (int j = -1; j <= 1; j++) {
                     Eigen::Vector3i sub = c_sub;
                     sub.x() += i, sub.y() += j, sub.z() = 0; // 获取周围自由位置
                     if (free_terrain_grid_->InRange(sub)) {
                         const int ind = free_terrain_grid_->Sub2Ind(sub);
                         free_terrain_grid_->GetCell(ind) = free_terrain_grid_->GetCell(ind) | FREE_BIT; // 设置为空闲区域
                         is_goal_in_freespace_ = true; // 表示最近目标可能位于无人区
                     }
                 }
             }
         }
     }
 
     // 如果附近没有地形云，则检查其他自由空间状态
     if (!is_goal_in_freespace_) { 
         float min_dist = FARUtil::kINF; // 初始化最小距离
 
         // 检查当前图上的所有节点
         for (const auto& node_ptr : current_graph_) {
             if (!node_ptr->is_navpoint) continue; // 仅处理导航点
             
             const float cur_dist = (node_ptr->position - center).norm_flat(); // 计算与中心的距离
             
             // 在局部规划范围内，且同一层上，更新 Z 位置
             if (cur_dist < FARUtil::kLocalPlanRange && FARUtil::IsAtSameLayer(node_ptr, goal_node_ptr_)) {
                 if (cur_dist < min_dist) {
                     goal_node_ptr_->position.z = node_ptr->position.z; // 更新目标的Z坐标
                     min_dist = cur_dist; // 更新最小距离
                 }
                 is_goal_in_freespace_ = true; // 标记目标在自由空间
             }
         }
     }
 }
 