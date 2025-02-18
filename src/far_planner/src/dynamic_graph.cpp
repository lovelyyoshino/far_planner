/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



 #include "far_planner/dynamic_graph.h"

 /***************************************************************************************/
 
 /**
  * @brief 初始化动态图
  * 
  * 该函数用于初始化动态图的参数，包括连接角度的余弦值、噪声角度的余弦值、ID 跟踪器等，
  * 并初始化地形规划器。
  * 
  * @param nh 一个 ros::NodeHandle 类型的引用，用于与 ROS 系统进行通信
  * @param params 一个 DynamicGraphParams 类型的引用，包含动态图的参数
  */
 void DynamicGraph::Init(const ros::NodeHandle& nh, const DynamicGraphParams& params) {
     // 将传入的参数赋值给类的成员变量 dg_params_
     dg_params_ = params;
     // 计算连接角度阈值的余弦值，并赋值给常量 CONNECT_ANGLE_COS
     CONNECT_ANGLE_COS = cos(dg_params_.kConnectAngleThred);
     // 计算噪声角度的余弦值，并赋值给常量 NOISE_ANGLE_COS
     NOISE_ANGLE_COS = cos(FARUtil::kAngleNoise);
     // 初始化 ID 跟踪器，从 1 开始
     id_tracker_     = 1;
     // 初始化最后连接位置为原点
     last_connect_pos_ = Point3D(0,0,0);
     /* Initialize Terrian Planner */
     // 设置地形规划器的世界坐标系为全局的世界坐标系
     tp_params_.world_frame  = FARUtil::worldFrameId;
     // 设置地形规划器的体素大小为全局的叶子大小
     tp_params_.voxel_size   = FARUtil::kLeafSize;
     // 设置地形规划器的半径为全局近邻距离的两倍
     tp_params_.radius       = FARUtil::kNearDist * 2.0f;
     // 设置地形规划器的膨胀大小为全局的障碍物膨胀大小
     tp_params_.inflate_size = FARUtil::kObsInflate;
     // 调用地形规划器的初始化函数，传入节点句柄和地形规划器参数
     terrain_planner_.Init(nh, tp_params_);
 }
 
 /**
  * @brief 更新机器人的位置
  * 
  * 该函数用于更新机器人的位置信息，同时更新地形规划器的局部地形障碍物云。
  * 如果里程计节点尚未初始化，则创建一个新的里程计节点并添加到图中；
  * 否则，更新现有里程计节点的位置。最后，更新全局里程计位置并可视化路径。
  * 
  * @param robot_pos 一个 Point3D 类型的引用，表示机器人的当前位置
  */
 void DynamicGraph::UpdateRobotPosition(const Point3D& robot_pos) {
     // 将传入的机器人位置赋值给类的成员变量 robot_pos_
     robot_pos_ = robot_pos;
     // 调用地形规划器的 SetLocalTerrainObsCloud 方法，设置局部地形障碍物云
     terrain_planner_.SetLocalTerrainObsCloud(FARUtil::local_terrain_obs_);
     // 检查里程计节点指针是否为 NULL
     if (odom_node_ptr_ == NULL) {
         // 如果里程计节点指针为 NULL，调用 CreateNavNodeFromPoint 方法创建一个新的里程计节点
         this->CreateNavNodeFromPoint(robot_pos_, odom_node_ptr_, true);
         // 将新创建的里程计节点添加到图中
         this->AddNodeToGraph(odom_node_ptr_);
         // 如果处于调试模式，输出信息表示里程计节点已初始化
         if (FARUtil::IsDebug) ROS_INFO("DG: Odom node has been initilaized.");
     } else {
         // 如果里程计节点指针不为 NULL，调用 UpdateNodePosition 方法更新里程计节点的位置
         this->UpdateNodePosition(odom_node_ptr_, robot_pos_);
     }
     // 将里程计节点的位置赋值给全局的里程计位置
     FARUtil::odom_pos = odom_node_ptr_->position;
     // 调用地形规划器的 VisualPaths 方法，可视化路径
     terrain_planner_.VisualPaths();
 }
 
 /**
  * @brief 判断是否需要创建中间导航点
  * 
  * 该函数用于判断是否需要在当前的导航图中创建中间导航点。它会考虑当前中间导航点指针的状态、
  * 里程计节点与当前中间导航点的连接情况、中间导航点与最后连接位置的距离等因素。
  * 
  * @return 如果需要创建中间导航点，返回 true；否则返回 false。
  */
 bool DynamicGraph::IsInterNavpointNecessary() {
     // 如果当前中间导航点指针为空，说明需要创建最近的导航点
     if (cur_internav_ptr_ == NULL ) { 
         // 更新最后连接位置为当前自由里程计位置
         last_connect_pos_ = FARUtil::free_odom_p;
         // 返回 true 表示需要创建中间导航点
         return true;
     }
     // 在里程计节点的边投票中查找当前中间导航点的 ID
     const auto it = odom_node_ptr_->edge_votes.find(cur_internav_ptr_->id);
     // 如果满足以下条件之一：
     // 1. 处于桥接中间导航点状态
     // 2. 里程计节点的边投票中未找到当前中间导航点的 ID
     // 3. 当前中间导航点不在有效范围内
     if (is_bridge_internav_ || it == odom_node_ptr_->edge_votes.end() || !this->IsInternavInRange(cur_internav_ptr_)) {
         // 初始化最小距离为无穷大
         float min_dist = FARUtil::kINF;
         // 遍历所有靠近的中间导航点
         for (const auto& internav_ptr : internav_near_nodes_) {
             // 计算当前中间导航点与最后连接位置的距离
             const float cur_dist = (internav_ptr->position - last_connect_pos_).norm();
             // 如果当前距离小于最小距离，则更新最小距离
             if (cur_dist < min_dist) min_dist = cur_dist;
         }
         // 如果最小距离大于导航清除距离，则需要创建中间导航点
         if (min_dist > FARUtil::kNavClearDist) return true;
     } 
     // 如果当前自由里程计位置与最后连接位置的距离大于近邻距离
     // 或者里程计节点的边投票中存在当前中间导航点的 ID 且最后一次投票为 1
     if ((FARUtil::free_odom_p - last_connect_pos_).norm() > FARUtil::kNearDist || 
         (it != odom_node_ptr_->edge_votes.end() && it->second.back() == 1)) 
     {
         // 更新最后连接位置为当前自由里程计位置
         last_connect_pos_ = FARUtil::free_odom_p;
     }
     // 不需要创建中间导航点
     return false;
 }
 
 /**
  * @brief 从新的轮廓节点中提取图节点
  * 
  * 该函数用于从传入的新轮廓节点栈中提取图节点。它会检查是否需要创建中间导航点，
  * 并遍历所有新的轮廓节点，判断是否为有效的新节点，若是则创建新的导航节点并添加到新节点栈中。
  * 
  * @param new_ctnodes 一个 CTNodeStack 类型的引用，表示新的轮廓节点栈
  * @return 如果成功提取到新节点，返回 true；否则返回 false
  */
 bool DynamicGraph::ExtractGraphNodes(const CTNodeStack& new_ctnodes) {
     // 如果新的轮廓节点栈为空，直接返回 false
     if (new_ctnodes.empty()) return false;
     // 初始化新节点指针为 NULL
     NavNodePtr new_node_ptr = NULL;
     // 清空新节点栈
     new_nodes_.clear();
     // 检查是否需要创建中间导航点
     if (this->IsInterNavpointNecessary()) { 
         // 如果处于调试模式，输出信息表示已创建一个轨迹节点
         if (FARUtil::IsDebug) ROS_INFO("DG: One trajectory node has been created.");
         // 从最后连接位置创建一个新的导航节点
         this->CreateNavNodeFromPoint(last_connect_pos_, new_node_ptr, false, true);
         // 将新节点添加到新节点栈中
         new_nodes_.push_back(new_node_ptr);
         // 更新最后连接位置为当前自由里程计位置
         last_connect_pos_ = FARUtil::free_odom_p;
         // 如果处于桥接中间导航点状态，将其置为 false
         if (is_bridge_internav_) is_bridge_internav_ = false;
     }
     // 遍历所有新的轮廓节点
     for (const auto& ctnode_ptr : new_ctnodes) {
         // 标记是否靠近新节点
         bool is_near_new = false;
         // 检查当前轮廓节点是否为有效的新节点
         if (this->IsAValidNewNode(ctnode_ptr, is_near_new)) {
             // 从轮廓节点创建一个新的导航节点
             this->CreateNewNavNodeFromContour(ctnode_ptr, new_node_ptr);
             // 如果不靠近新节点，将其标记为阻塞前沿节点
             if (!is_near_new) {
                 new_node_ptr->is_block_frontier = true;
             }
             // 将新节点添加到新节点栈中
             new_nodes_.push_back(new_node_ptr);
         }
     }
     // 如果新节点栈为空，返回 false；否则返回 true
     if (new_nodes_.empty()) return false;
     else return true;
 }
 
 /**
  * @brief 更新导航图
  * 
  * 该函数用于更新导航图，包括清除误报的节点检测、重新评估轨迹边、清除合并的节点、
  * 添加匹配的边缘节点到近节点和宽近节点中、检查并添加与里程计节点的连接、添加新节点、
  * 连接超出范围的轮廓节点、重新连接近节点之间的连接、更新超出范围的断开节点连接以及分析前沿节点。
  * 
  * @param new_nodes 一个 NodePtrStack 类型的引用，表示新的节点栈
  * @param is_freeze_vgraph 一个布尔类型的引用，表示是否冻结可视化图
  * @param clear_node 一个 NodePtrStack 类型的引用，表示需要清除的节点栈
  */
 void DynamicGraph::UpdateNavGraph(const NodePtrStack& new_nodes,
                                   const bool& is_freeze_vgraph,
                                   NodePtrStack& clear_node) 
 {
     // 清除误报的节点检测
     clear_node.clear();
     if (!is_freeze_vgraph) {
         // 遍历扩展匹配节点
         for (const auto& node_ptr : extend_match_nodes_) {
             // 如果是静态节点或当前中间导航点，则跳过
             if (FARUtil::IsStaticNode(node_ptr) || node_ptr == cur_internav_ptr_) continue;
             // 重新评估节点的角点
             if (!this->ReEvaluateCorner(node_ptr)) {
                 // 如果需要将节点设置为清除状态
                 if (this->SetNodeToClear(node_ptr)) {
                     // 将节点添加到清除节点栈中
                     clear_node.push_back(node_ptr);
                 }
             } else {
                 // 减少节点的计数器
                 this->ReduceDumperCounter(node_ptr);
             }
         }
         // 如果不是静态环境且当前中间导航点不为空
         if (!FARUtil::IsStaticEnv && cur_internav_ptr_ != NULL) {
             // 获取周围的中间导航节点
             NodePtrStack internav_check_nodes = surround_internav_nodes_;
             // 如果当前中间导航点不在检查节点列表中，则添加
             if (!FARUtil::IsTypeInStack(cur_internav_ptr_, internav_check_nodes)) {
                 internav_check_nodes.push_back(cur_internav_ptr_);
             }
             // 遍历周围的中间导航节点
             for (const auto& sur_internav_ptr : internav_check_nodes) {
                 // 获取轨迹连接节点的副本
                 const NodePtrStack copy_traj_connects = sur_internav_ptr->trajectory_connects;
                 // 遍历轨迹连接节点
                 for (const auto& tnode_ptr : copy_traj_connects) {
                     // 重新评估轨迹连接
                     if (this->ReEvaluateConnectUsingTerrian(sur_internav_ptr, tnode_ptr)) {
                         // 记录有效的轨迹边
                         this->RecordValidTrajEdge(sur_internav_ptr, tnode_ptr);
                     } else {
                         // 移除无效的轨迹边
                         this->RemoveInValidTrajEdge(sur_internav_ptr, tnode_ptr);
                     }
                 }
             }
         }
     }
     // 清除图中合并的节点
     this->ClearMergedNodesInGraph();
     // 将匹配的边缘节点添加到近节点和宽近节点中
     this->UpdateNearNodesWithMatchedMarginNodes(margin_near_nodes_, near_nav_nodes_, wide_near_nodes_);
     // 检查并添加与里程计节点的连接，使用宽近节点
     NodePtrStack codom_check_list = wide_near_nodes_;
     // 将新节点添加到检查列表中
     codom_check_list.insert(codom_check_list.end(), new_nodes.begin(), new_nodes.end()); 
     // 遍历检查列表中的节点
     for (const auto& conode_ptr : codom_check_list) {
         // 如果是里程计节点，则跳过
         if (conode_ptr->is_odom) continue;
         // 检查连接是否有效
         if (this->IsValidConnect(odom_node_ptr_, conode_ptr, false)) {
             // 添加多边形边和边
             this->AddPolyEdge(odom_node_ptr_, conode_ptr), this->AddEdge(odom_node_ptr_, conode_ptr);
         } else {
             // 移除多边形边和边
             this->ErasePolyEdge(odom_node_ptr_, conode_ptr), this->EraseEdge(conode_ptr, odom_node_ptr_);
         }
     }
     if (!is_freeze_vgraph) {
         // 将新节点添加到近节点栈中
         for (const auto& new_node_ptr : new_nodes) {
             // 将新节点添加到图中
             this->AddNodeToGraph(new_node_ptr);
             // 标记新节点为近节点
             new_node_ptr->is_near_nodes = true;
             // 将新节点添加到近导航节点栈中
             near_nav_nodes_.push_back(new_node_ptr);
             // 如果是导航点，则更新当前中间导航节点
             if (new_node_ptr->is_navpoint) this->UpdateCurInterNavNode(new_node_ptr);
             // 如果新节点有轮廓节点
             if (new_node_ptr->ctnode != NULL) {
                 // 匹配轮廓节点和导航节点
                 ContourGraph::MatchCTNodeWithNavNode(new_node_ptr->ctnode, new_node_ptr);
             }
         }
         // 连接超出范围的轮廓节点
         for (const auto& out_node_ptr : out_contour_nodes_) {
             // 匹配超出范围的节点和近导航节点
             const NavNodePtr matched_node = ContourGraph::MatchOutrangeNodeWithCTNode(out_node_ptr, near_nav_nodes_);
             // 在超出范围的轮廓节点映射中查找该节点
             const auto it = out_contour_nodes_map_.find(out_node_ptr);
             // 如果匹配到节点
             if (matched_node != NULL) {
                 // 记录轮廓投票
                 this->RecordContourVote(out_node_ptr, matched_node);
                 // 将匹配节点添加到已到达节点集合中
                 it->second.second.insert(matched_node);
             }
             // 遍历已到达节点集合
             for (const auto& reached_node_ptr : it->second.second) {
                 // 如果不是匹配节点
                 if (reached_node_ptr != matched_node) {
                     // 删除轮廓投票
                     this->DeleteContourVote(out_node_ptr, reached_node_ptr);
                 }
             }
         }
         // 重新连接近节点之间的连接
         NodePtrStack outside_break_nodes;
         // 遍历近导航节点
         for (std::size_t i=0; i<near_nav_nodes_.size(); i++) {
             const NavNodePtr nav_ptr1 = near_nav_nodes_[i];
             // 如果是里程计节点，则跳过
             if (nav_ptr1->is_odom) continue;
             // 获取连接节点的副本
             const NodePtrStack copy_connect_nodes = nav_ptr1->connect_nodes;
             // 遍历连接节点
             for (const auto& cnode : copy_connect_nodes) {
                 // 如果是里程计节点、近节点、超出目标范围或轮廓连接节点，则跳过
                 if (cnode->is_odom || cnode->is_near_nodes || FARUtil::IsOutsideGoal(cnode) || FARUtil::IsTypeInStack(cnode, nav_ptr1->contour_connects)) continue;
                 // 检查连接是否有效
                 if (this->IsValidConnect(nav_ptr1, cnode, false)) {
                     // 添加多边形边和边
                     this->AddPolyEdge(nav_ptr1, cnode), this->AddEdge(nav_ptr1, cnode);
                 } else {
                     // 移除多边形边和边
                     this->ErasePolyEdge(nav_ptr1, cnode) ,this->EraseEdge(nav_ptr1, cnode);
                     // 将断开连接的节点添加到断开节点栈中
                     outside_break_nodes.push_back(cnode);
                 } 
             }
             // 遍历近导航节点
             for (std::size_t j=0; j<near_nav_nodes_.size(); j++) {
                 const NavNodePtr nav_ptr2 = near_nav_nodes_[j];
                 // 如果是同一个节点、索引大于当前节点或里程计节点，则跳过
                 if (i == j || j > i || nav_ptr2->is_odom) continue;
                 // 检查连接是否有效
                 if (this->IsValidConnect(nav_ptr1, nav_ptr2, true)) {
                     // 添加多边形边和边
                     this->AddPolyEdge(nav_ptr1, nav_ptr2), this->AddEdge(nav_ptr1, nav_ptr2);
                 } else {
                     // 移除多边形边和边
                     this->ErasePolyEdge(nav_ptr1, nav_ptr2), this->EraseEdge(nav_ptr1, nav_ptr2);
                 }
             }
             // 遍历所有超出轮廓范围的节点
             for (const auto& oc_node_ptr : out_contour_nodes_) {
                 // 如果超出轮廓范围的节点或当前导航节点 1 没有匹配的轮廓，则跳过此次循环
                 if (!oc_node_ptr->is_contour_match || !nav_ptr1->is_contour_match) continue;
                 // 检查当前导航节点 1 和超出轮廓范围的节点是否通过轮廓相连
                 if (ContourGraph::IsNavNodesConnectFromContour(nav_ptr1, oc_node_ptr)) {
                     // 如果相连，则记录轮廓投票
                     this->RecordContourVote(nav_ptr1, oc_node_ptr);
                 } else {
                     // 如果不相连，则删除轮廓投票
                     this->DeleteContourVote(nav_ptr1, oc_node_ptr);
                 }
             }
             // 处理当前导航节点 1 的前两个轮廓连接
             this->TopTwoContourConnector(nav_ptr1);
         }
         // 更新超出范围的断开节点的连接
         for (const auto& node_ptr : near_nav_nodes_) {
             for (const auto& ob_node_ptr : outside_break_nodes) {
                 // 检查当前近导航节点和超出范围的断开节点之间的连接是否有效
                 if (this->IsValidConnect(node_ptr, ob_node_ptr, false)) {
                     // 如果有效，则添加多边形边和普通边
                     this->AddPolyEdge(node_ptr, ob_node_ptr), this->AddEdge(node_ptr, ob_node_ptr);
                 } else {
                     // 如果无效，则删除多边形边和普通边
                     this->ErasePolyEdge(node_ptr, ob_node_ptr), this->EraseEdge(node_ptr, ob_node_ptr);
                 }
             }
         }
         // 分析前沿节点
         for (const auto& node_ptr : near_nav_nodes_) {
             // 检查当前近导航节点是否被完全覆盖
             if (this->IsNodeFullyCovered(node_ptr)) {
                 // 如果被完全覆盖，则标记为已覆盖
                 node_ptr->is_covered = true;
             } else {
                 // 如果未被完全覆盖，则标记为未覆盖
                 node_ptr->is_covered = false;
             }
             // 检查当前近导航节点是否为前沿节点
             if (this->IsFrontierNode(node_ptr)) {
                 // 如果是前沿节点，则标记为前沿节点
                 node_ptr->is_frontier = true;
             } else {
                 // 如果不是前沿节点，则标记为非前沿节点
                 node_ptr->is_frontier = false;
             }
         }
     }
 }
 
 /**
  * @brief 检查两个导航节点之间的连接是否有效
  * 
  * 该函数用于检查两个导航节点之间的连接是否有效，考虑了多种连接条件，
  * 包括距离、轮廓连接、多边形连接、轨迹连接等。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param is_check_contour 是否检查轮廓连接的标志
  * @return 如果连接有效返回 true，否则返回 false
  */
 bool DynamicGraph::IsValidConnect(const NavNodePtr& node_ptr1, 
                                   const NavNodePtr& node_ptr2,
                                   const bool& is_check_contour) 
 {
     // 计算两个节点之间的距离
     const float dist = (node_ptr1->position - node_ptr2->position).norm();
     // 如果两个节点几乎重叠，则认为连接有效
     if (dist < FARUtil::kEpsilon) return true;
     // 如果其中一个节点是里程计节点，另一个是导航点节点，且距离小于导航清除距离，则认为连接有效
     if ((node_ptr1->is_odom || node_ptr2->is_odom) && (node_ptr1->is_navpoint || node_ptr2->is_navpoint)) {
         if (dist < FARUtil::kNavClearDist) return true; 
     } 
     /* check contour connection from node1 to node2 */
     // 如果需要检查轮廓连接
     if (is_check_contour) {
         // 如果两个节点之间是边界连接，或者通过轮廓相连且地形连接有效，则记录轮廓投票
         if (this->IsBoundaryConnect(node_ptr1, node_ptr2) || (ContourGraph::IsNavNodesConnectFromContour(node_ptr1, node_ptr2) && IsOnTerrainConnect(node_ptr1, node_ptr2, true))) {
             this->RecordContourVote(node_ptr1, node_ptr2);
         } 
         // 如果两个节点都有匹配的轮廓，但连接无效，则删除轮廓投票
         else if (node_ptr1->is_contour_match && node_ptr2->is_contour_match) {
             this->DeleteContourVote(node_ptr1, node_ptr2);
         }
     }
     // 初始化连接标志为 false
     bool is_connect = false;
     /* check polygon connections */
     // 根据节点是否为里程计节点，确定投票队列的大小
     const int vote_queue_size = (node_ptr1->is_odom || node_ptr2->is_odom) ? std::ceil(dg_params_.votes_size / 3.0f) : dg_params_.votes_size;
     // 如果两个节点是凸连接、在方向约束内、通过自由多边形相连且地形连接有效
     if (IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2) && ContourGraph::IsNavNodesConnectFreePolygon(node_ptr1, node_ptr2) && IsOnTerrainConnect(node_ptr1, node_ptr2, false)) {
         // 如果多边形匹配用于连接，则记录多边形投票
         if (this->IsPolyMatchedForConnect(node_ptr1, node_ptr2)) {
             RecordPolygonVote(node_ptr1, node_ptr2, vote_queue_size);
         }
     } 
     // 否则，删除多边形投票
     else {
         DeletePolygonVote(node_ptr1, node_ptr2, vote_queue_size);
     }
     // 如果多边形边投票为真，且没有相似的连接方向，则认为连接有效
     if (this->IsPolygonEdgeVoteTrue(node_ptr1, node_ptr2)) {
         if (!this->IsSimilarConnectInDiection(node_ptr1, node_ptr2)) is_connect = true;
     } 
     // 如果其中一个节点是里程计节点，清除边投票和潜在连接
     else if (node_ptr1->is_odom || node_ptr2->is_odom) {
         node_ptr1->edge_votes.erase(node_ptr2->id);
         node_ptr2->edge_votes.erase(node_ptr1->id);
         // clear potential connections
         FARUtil::EraseNodeFromStack(node_ptr2, node_ptr1->potential_edges);
         FARUtil::EraseNodeFromStack(node_ptr1, node_ptr2->potential_edges);
     }
     /* check if exsiting trajectory connection exist */
     // 如果前面的条件都不满足，检查是否存在现有的轨迹连接
     if (!is_connect) {
         // 如果节点 1 在节点 2 的轨迹连接列表中，则认为连接有效
         if (FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) is_connect = true;
         // 如果其中一个节点是里程计节点，且当前中间导航节点不为空
         if ((node_ptr1->is_odom || node_ptr2->is_odom) && cur_internav_ptr_ != NULL) {
             if (node_ptr1->is_odom && FARUtil::IsTypeInStack(node_ptr2, cur_internav_ptr_->trajectory_connects)) {
                 // 如果节点 2 在以当前中间导航节点和里程计节点为轴的圆柱体内，则认为连接有效
                 if (FARUtil::IsInCylinder(cur_internav_ptr_->position, node_ptr2->position, node_ptr1->position, FARUtil::kNearDist)) {
                     is_connect = true;
                 }   
             } else if (node_ptr2->is_odom && FARUtil::IsTypeInStack(node_ptr1, cur_internav_ptr_->trajectory_connects)) {
                 // 如果节点 1 在以当前中间导航节点和里程计节点为轴的圆柱体内，则认为连接有效
                 if (FARUtil::IsInCylinder(cur_internav_ptr_->position, node_ptr1->position, node_ptr2->position, FARUtil::kNearDist)) {
                     is_connect = true;
                 }
             }
         }
     }
     /* check for additional contour connection through tight area from current robot position */
     // 如果前面的条件都不满足，检查是否存在通过狭窄区域的额外轮廓连接
     if (!is_connect && (node_ptr1->is_odom || node_ptr2->is_odom) && IsConvexConnect(node_ptr1, node_ptr2) && this->IsInDirectConstraint(node_ptr1, node_ptr2)) {
         if (node_ptr1->is_odom && !node_ptr2->contour_connects.empty()) {
             // 遍历节点 2 的轮廓连接节点
             for (const auto& ctnode_ptr : node_ptr2->contour_connects) {
                 // 如果轮廓连接节点在以节点 2 和里程计节点为轴的圆柱体内，则认为连接有效
                 if (FARUtil::IsInCylinder(ctnode_ptr->position, node_ptr2->position, node_ptr1->position, FARUtil::kNavClearDist)) {
                     is_connect = true;
                 }
             }
         } else if (node_ptr2->is_odom && !node_ptr1->contour_connects.empty()) {
             // 遍历节点 1 的轮廓连接节点
             for (const auto& ctnode_ptr : node_ptr1->contour_connects) {
                 // 如果轮廓连接节点在以节点 1 和里程计节点为轴的圆柱体内，则认为连接有效
                 if (FARUtil::IsInCylinder(ctnode_ptr->position, node_ptr1->position, node_ptr2->position, FARUtil::kNavClearDist)) {
                     is_connect = true;
                 }
             }
         }
     }
     // 返回连接是否有效的标志
     return is_connect;
 }
 
 /**
  * @brief 检查两个导航节点之间是否存在有效的地形连接
  * 
  * 该函数用于检查两个导航节点之间是否存在有效的地形连接，考虑了节点的活性、坡度、高度差等因素。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param is_contour 是否为轮廓连接的标志
  * @return 如果存在有效的地形连接返回 true，否则返回 false
  */
 bool DynamicGraph::IsOnTerrainConnect(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const bool& is_contour) {
     // 如果任一节点不活跃，则认为存在连接
     if (!node_ptr1->is_active || !node_ptr2->is_active) return true;
     // 计算两个节点的中点
     Point3D mid_p = (node_ptr1->position + node_ptr2->position) / 2.0f;
     // 计算两个节点之间的差值向量
     const Point3D diff_p = node_ptr2->position - node_ptr1->position;
     // 如果两点距离大于匹配距离且坡度大于45度，则移除无效连接并返回 false
     if (diff_p.norm() > FARUtil::kMatchDist && abs(diff_p.z) / std::hypotf(diff_p.x, diff_p.y) > 1) {
         // 如果不是轮廓连接，则移除无效的地形连接
         if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
         return false; // slope is too steep > 45 degree
     } 
     // 如果是轮廓连接且已记录轮廓地形连接，则认为存在连接
     if (is_contour && node_ptr1->contour_votes.find(node_ptr2->id) != node_ptr1->contour_votes.end()) { 
         return true;
     }
     // 用于判断是否匹配到高度信息
     bool is_match;
     // 记录最小高度
     float minH;
     // 记录最大高度
     float maxH;
     // 获取中点周围半径内的最近高度
     const float avg_h = MapHandler::NearestHeightOfRadius(mid_p, FARUtil::kMatchDist, minH, maxH, is_match);
     // 如果未匹配到高度信息且满足一定条件，则移除无效连接并返回 false
     if (!is_match && (is_contour || !node_ptr1->is_frontier || !node_ptr2->is_frontier)) {
         // 如果不是轮廓连接，则移除无效的地形连接
         if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
         return false;
     } 
     // 如果匹配到高度信息但高度差过大或中点高度偏差过大，则移除无效连接并返回 false
     if (is_match && (maxH - minH > FARUtil::kMarginHeight || abs(minH + FARUtil::vehicle_height - mid_p.z) > FARUtil::kTolerZ / 2.0f)) {
         // 如果不是轮廓连接，则移除无效的地形连接
         if (!is_contour) RemoveInvaildTerrainConnect(node_ptr1, node_ptr2);
         return false;
     }
     // 如果不是轮廓连接，进行额外处理
     if (!is_contour) {
         // 如果匹配到高度信息，则记录有效的地形连接
         if (is_match) RecordVaildTerrainConnect(node_ptr1, node_ptr2);
         // 查找节点1的地形投票记录
         const auto it = node_ptr1->terrain_votes.find(node_ptr2->id);
         // 如果找到投票记录且投票数超过阈值，则返回 false
         if (it != node_ptr1->terrain_votes.end() && it->second > dg_params_.finalize_thred) {
             return false;
         }
     }
     // 所有条件都满足，则认为存在有效的地形连接
     return true;
 }
 
 /**
  * @brief 检查一个导航节点是否被完全覆盖
  * 
  * 该函数用于检查给定的导航节点是否被完全覆盖。它考虑了节点的自由状态、是否已经被标记为覆盖，
  * 以及与附近里程计节点的距离和连接投票情况。
  * 
  * @param node_ptr 要检查的导航节点的指针
  * @return 如果节点被完全覆盖返回 true，否则返回 false
  */
 bool DynamicGraph::IsNodeFullyCovered(const NavNodePtr& node_ptr) {
     // 如果节点是自由导航节点或者已经被标记为覆盖，则认为节点被完全覆盖
     if (FARUtil::IsFreeNavNode(node_ptr) || node_ptr->is_covered) return true;
     // 创建一个检查里程计节点的列表，包含中间导航附近节点和里程计节点
     NodePtrStack check_odom_list = internav_near_nodes_;
     check_odom_list.push_back(odom_node_ptr_);
     // 遍历检查里程计节点列表
     for (const auto& near_optr : check_odom_list) {
         // 计算当前节点与附近里程计节点的距离
         const float cur_dist = (node_ptr->position - near_optr->position).norm();
         // 如果距离小于匹配距离，则认为节点被完全覆盖
         if (cur_dist < FARUtil::kMatchDist) return true;
         // 如果节点的自由方向不是柱子类型
         if (node_ptr->free_direct != NodeFreeDirect::PILLAR) {
             // TODO: 凹节点根据当前实现不会被标记为覆盖
             // 在附近里程计节点的边投票中查找当前节点的投票记录
             const auto it = near_optr->edge_votes.find(node_ptr->id);
             // 如果找到投票记录且投票结果为真
             if (it != near_optr->edge_votes.end() && FARUtil::IsVoteTrue(it->second)) {
                 // 计算附近里程计节点与当前节点的差值向量
                 const Point3D diff_p = near_optr->position - node_ptr->position;
                 // 如果差值向量在覆盖方向对中，则认为节点被完全覆盖
                 if (FARUtil::IsInCoverageDirPairs(diff_p, node_ptr)) {
                     return true;
                 }
             }
         }
     }
     // 所有条件都不满足，则认为节点未被完全覆盖
     return false;
 }
 
 /**
  * @brief 检查一个导航节点是否为前沿节点
  * 
  * 该函数用于检查给定的导航节点是否为前沿节点。它考虑了节点的轮廓匹配状态、是否被覆盖、自由方向、
  * 多边形周长以及节点位置是否在有效范围内等因素，并通过投票机制来确定节点是否为前沿节点。
  * 
  * @param node_ptr 要检查的导航节点的指针
  * @return 如果节点是前沿节点返回 true，否则返回 false
  */
 bool DynamicGraph::IsFrontierNode(const NavNodePtr& node_ptr) {
     // 如果节点有匹配的轮廓
     if (node_ptr->is_contour_match) {
         // 如果节点是块前沿、已被覆盖、自由方向不是凸的或者多边形周长小于前沿周长阈值
         if (node_ptr->is_block_frontier || node_ptr->is_covered || node_ptr->free_direct != NodeFreeDirect::CONVEX ||
             node_ptr->ctnode->poly_ptr->perimeter < dg_params_.frontier_perimeter_thred) 
         {
             // 标记为非凸前沿，投票为 0
             node_ptr->frontier_votes.push_back(0); 
         } else {
             // 标记为凸前沿，投票为 1
             node_ptr->frontier_votes.push_back(1); 
         }
     } 
     // 如果节点位置不在有效范围内
     else if (!FARUtil::IsPointInMarginRange(node_ptr->position)) { 
         // 标记为非凸前沿，投票为 0
         node_ptr->frontier_votes.push_back(0); 
     }
     // 如果投票队列的大小超过最终确定阈值
     if (node_ptr->frontier_votes.size() > dg_params_.finalize_thred) {
         // 移除最早的投票
         node_ptr->frontier_votes.pop_front();
     }
     // 根据投票结果判断节点是否为前沿节点
     bool is_frontier = FARUtil::IsVoteTrue(node_ptr->frontier_votes);
     // 如果节点之前不是前沿节点，现在投票结果为是前沿节点，且投票队列大小达到最终确定阈值
     if (!node_ptr->is_frontier && is_frontier && node_ptr->frontier_votes.size() == dg_params_.finalize_thred) {
         // 如果节点位置不在新点附近
         if (!FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
             // 标记为非前沿节点
             is_frontier = false;
         }
     }
     // 返回节点是否为前沿节点的结果
     return is_frontier;
 }
 
 /**
  * @brief 检查两个导航节点之间的连接方向是否相似
  * 
  * 该函数用于检查从一个导航节点到另一个导航节点的连接方向是否与其他连接方向相似。
  * 它考虑了节点是否为里程计节点、是否为轮廓连接，以及是否存在更短的连接方向。
  * 
  * @param node_ptr_from 起始导航节点的指针
  * @param node_ptr_to 目标导航节点的指针
  * @return 如果连接方向相似返回 true，否则返回 false
  */
 bool DynamicGraph::IsSimilarConnectInDiection(const NavNodePtr& node_ptr_from,
                                               const NavNodePtr& node_ptr_to)
 {
     // TODO: 检查连接丢失情况
     // 如果起始节点或目标节点是里程计节点，则认为连接方向不相似
     if (node_ptr_from->is_odom || node_ptr_to->is_odom) return false;
     // 如果目标节点在起始节点的轮廓连接列表中，则释放轮廓连接检查，认为连接方向不相似
     if (FARUtil::IsTypeInStack(node_ptr_to, node_ptr_from->contour_connects)) { 
         return false;
     }
     // 检查从起始节点到目标节点的连接是否存在更短的连接方向
     if (this->IsAShorterConnectInDir(node_ptr_from, node_ptr_to)) {
         return true;
     }
     // 检查从目标节点到起始节点的连接是否存在更短的连接方向
     if (this->IsAShorterConnectInDir(node_ptr_to, node_ptr_from)) {
         return true;
     }
     // 如果以上条件都不满足，则认为连接方向不相似
     return false;
 }
 
 /**
  * @brief 检查两个导航节点之间的连接是否在方向约束内
  * 
  * 该函数用于检查两个导航节点之间的连接是否满足方向约束条件。它考虑了节点是否为里程计节点、前沿节点，
  * 以及节点的自由方向和表面方向。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @return 如果连接在方向约束内返回 true，否则返回 false
  */
 bool DynamicGraph::IsInDirectConstraint(const NavNodePtr& node_ptr1,
                                         const NavNodePtr& node_ptr2) 
 {
     // 检查是否为里程计节点到前沿节点的连接
     // 如果其中一个节点是里程计节点，另一个是前沿节点，则认为连接在方向约束内
     if ((node_ptr1->is_odom && node_ptr2->is_frontier) || (node_ptr2->is_odom && node_ptr1->is_frontier)) return true;
     // 检查从节点1到节点2的连接是否在方向约束内
     if (node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
         // 计算从节点1到节点2的向量
         Point3D diff_1to2 = (node_ptr2->position - node_ptr1->position);
         // 如果该向量不在节点1的表面方向的约束范围内，则认为连接不在方向约束内
         if (!FARUtil::IsOutReducedDirs(diff_1to2, node_ptr1->surf_dirs)) {
             return false;
         }
     }
     // 检查从节点2到节点1的连接是否在方向约束内
     if (node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
         // 计算从节点2到节点1的向量
         Point3D diff_2to1 = (node_ptr1->position - node_ptr2->position);
         // 如果该向量不在节点2的表面方向的约束范围内，则认为连接不在方向约束内
         if (!FARUtil::IsOutReducedDirs(diff_2to1, node_ptr2->surf_dirs)) {
             return false;
         }
     }
     // 所有条件都满足，则认为连接在方向约束内
     return true;
 }
 
 /**
  * @brief 检查两个导航节点之间的连接是否在轮廓方向约束内
  * 
  * 该函数用于检查两个导航节点之间的连接是否满足轮廓方向约束条件。它考虑了节点是否为自由导航节点、
  * 节点是否已最终确定以及节点的自由方向和表面方向。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @return 如果连接在轮廓方向约束内返回 true，否则返回 false
  */
 bool DynamicGraph::IsInContourDirConstraint(const NavNodePtr& node_ptr1,
                                             const NavNodePtr& node_ptr2) 
 {
     // 如果任一节点是自由导航节点，则认为连接不在轮廓方向约束内
     if (FARUtil::IsFreeNavNode(node_ptr1) || FARUtil::IsFreeNavNode(node_ptr2)) return false;
     // 检查从节点1到节点2的连接是否在轮廓方向约束内
     if (node_ptr1->is_finalized && node_ptr1->free_direct != NodeFreeDirect::PILLAR) {
         // 计算从节点1到节点2的向量
         const Point3D diff_1to2 = node_ptr2->position - node_ptr1->position;
         // 如果该向量不在节点1的轮廓方向对范围内
         if (!FARUtil::IsInContourDirPairs(diff_1to2, node_ptr1->surf_dirs)) {
             // 如果节点1的轮廓连接数量小于2
             if (node_ptr1->contour_connects.size() < 2) {
                 // 重置节点1的滤波器
                 this->ResetNodeFilters(node_ptr1);
             } 
             // 认为连接不在轮廓方向约束内
             return false;
         }
     }
     // 检查从节点2到节点1的连接是否在轮廓方向约束内
     if (node_ptr2->is_finalized && node_ptr2->free_direct != NodeFreeDirect::PILLAR) {
         // 计算从节点2到节点1的向量
         const Point3D diff_2to1 = node_ptr1->position - node_ptr2->position;
         // 如果该向量不在节点2的轮廓方向对范围内
         if (!FARUtil::IsInContourDirPairs(diff_2to1, node_ptr2->surf_dirs)) {
             // 如果节点2的轮廓连接数量小于2
             if (node_ptr2->contour_connects.size() < 2) {
                 // 重置节点2的滤波器
                 this->ResetNodeFilters(node_ptr2);
             }
             // 认为连接不在轮廓方向约束内
             return false;
         }
     }
     // 所有条件都满足，则认为连接在轮廓方向约束内
     return true;
 }
 
 /**
  * @brief 检查从一个导航节点到另一个导航节点的连接是否存在更短的连接方向
  * 
  * 该函数用于检查从起始导航节点到目标导航节点的连接方向上，是否存在从起始节点到其他连接节点的更短连接。
  * 它考虑了节点是否为导航点、是否被覆盖，以及连接的角度和距离。
  * 
  * @param node_ptr_from 起始导航节点的指针
  * @param node_ptr_to 目标导航节点的指针
  * @return 如果存在更短的连接方向返回 true，否则返回 false
  */
 bool DynamicGraph::IsAShorterConnectInDir(const NavNodePtr& node_ptr_from, const NavNodePtr& node_ptr_to) {
     // 标记是否为导航点连接
     bool is_nav_connect = false;
     // 标记是否为覆盖连接
     bool is_cover_connect = false;
     // 如果起始节点和目标节点都是导航点，则标记为导航点连接
     if (node_ptr_from->is_navpoint && node_ptr_to->is_navpoint) is_nav_connect = true;
     // 如果起始节点和目标节点都被覆盖，则标记为覆盖连接
     if (node_ptr_from->is_covered && node_ptr_to->is_covered) is_cover_connect = true;
     // 如果起始节点没有连接节点，则返回 false
     if (node_ptr_from->connect_nodes.empty()) return false;
     // 参考方向向量
     Point3D ref_dir;
     // 参考差值向量
     Point3D ref_diff;
     // 计算从起始节点到目标节点的差值向量
     const Point3D diff_p = node_ptr_to->position - node_ptr_from->position;
     // 计算从起始节点到目标节点的连接方向向量
     const Point3D connect_dir = diff_p.normalize();
     // 计算从起始节点到目标节点的距离
     const float dist = diff_p.norm();
     // 遍历起始节点的所有连接节点
     for (const auto& cnode : node_ptr_from->connect_nodes) {
         // 如果是导航点连接，但当前连接节点不是导航点，则跳过
         if (is_nav_connect && !cnode->is_navpoint) continue;
         // 如果是覆盖连接，但当前连接节点未被覆盖，则跳过
         if (is_cover_connect && !cnode->is_covered) continue;
         // 如果当前连接节点在起始节点的轮廓连接列表中，则跳过
         if (FARUtil::IsTypeInStack(cnode, node_ptr_from->contour_connects)) continue;
         // 计算从起始节点到当前连接节点的差值向量
         ref_diff = cnode->position - node_ptr_from->position;
         // 如果当前连接节点是里程计节点，或者距离小于极小值，则跳过
         if (cnode->is_odom || ref_diff.norm() < FARUtil::kEpsilon) continue;
         // 计算从起始节点到当前连接节点的参考方向向量
         ref_dir = ref_diff.normalize();
         // 如果连接方向向量和参考方向向量的点积大于指定的角度余弦值，且到目标节点的距离大于到当前连接节点的距离
         if ((connect_dir * ref_dir) > CONNECT_ANGLE_COS && dist > ref_diff.norm()) {
             // 则认为存在更短的连接方向，返回 true
             return true;
         }
     }
     // 遍历完所有连接节点都没有找到更短的连接方向，返回 false
     return false;
 }
 
 /**
  * @brief 更新导航节点的位置
  * 
  * 该函数用于更新导航节点的位置，使用 RANSAC 算法计算节点的平均位置。
  * 如果节点是自由导航节点，则直接初始化节点位置。
  * 如果节点已经最终确定，则不进行更新。
  * 
  * @param node_ptr 要更新位置的导航节点的指针
  * @param new_pos 新的节点位置
  * @return 如果节点位置更新成功返回 true，否则返回 false
  */
 bool DynamicGraph::UpdateNodePosition(const NavNodePtr& node_ptr,
                                       const Point3D& new_pos) 
 {
     // 如果节点是自由导航节点，则初始化节点位置
     if (FARUtil::IsFreeNavNode(node_ptr)) {
         this->InitNodePosition(node_ptr, new_pos);
         return true;
     }
     // 如果节点已经最终确定，则不进行更新
     if (node_ptr->is_finalized) return true; 
     // 将新的位置添加到位置滤波向量中
     node_ptr->pos_filter_vec.push_back(new_pos);
     // 如果位置滤波向量的大小超过池大小，则移除最早的位置
     if (node_ptr->pos_filter_vec.size() > dg_params_.pool_size) {
         node_ptr->pos_filter_vec.pop_front();
     }
     // 使用 RANSAC 算法计算导航节点的平均位置
     std::size_t inlier_size = 0;
     // 调用 RANSACPoisiton 函数计算平均位置
     Point3D mean_p = FARUtil::RANSACPoisiton(node_ptr->pos_filter_vec, dg_params_.filter_pos_margin, inlier_size);
     // 如果位置滤波向量的大小大于 1，则保持 z 值不变
     if (node_ptr->pos_filter_vec.size() > 1) mean_p.z = node_ptr->position.z; 
     // 更新节点的位置
     node_ptr->position = mean_p;
     // 如果内点数量超过最终确定阈值，则认为节点位置更新成功
     if (inlier_size > dg_params_.finalize_thred) {
         return true;
     }
     // 否则，认为节点位置更新失败
     return false;
 }
 
 /**
  * @brief 初始化导航节点的位置
  * 
  * 该函数用于初始化导航节点的位置，将位置滤波向量清空，并将新的位置赋值给节点的位置，
  * 同时将新的位置添加到位置滤波向量中。
  * 
  * @param node_ptr 要初始化位置的导航节点的指针
  * @param new_pos 新的节点位置
  */
 void DynamicGraph::InitNodePosition(const NavNodePtr& node_ptr, const Point3D& new_pos) {
     // 清空位置滤波向量
     node_ptr->pos_filter_vec.clear();
     // 将新的位置赋值给节点的位置
     node_ptr->position = new_pos;
     // 将新的位置添加到位置滤波向量中
     node_ptr->pos_filter_vec.push_back(new_pos);
 }
 
 /**
  * @brief 更新导航节点的表面方向
  * 
  * 该函数用于更新导航节点的表面方向，使用 RANSAC 算法计算节点的平均表面方向。
  * 如果节点是自由导航节点，则将表面方向设置为默认值，并将自由方向设置为柱子类型。
  * 如果节点已经最终确定，则不进行更新。
  * 
  * @param node_ptr 要更新表面方向的导航节点的指针
  * @param cur_dirs 当前的表面方向
  * @return 如果节点表面方向更新成功返回 true，否则返回 false
  */
 bool DynamicGraph::UpdateNodeSurfDirs(const NavNodePtr& node_ptr, PointPair cur_dirs)
 {
     // 如果节点是自由导航节点，则将表面方向设置为默认值，并将自由方向设置为柱子类型
     if (FARUtil::IsFreeNavNode(node_ptr)) {
         node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
         node_ptr->free_direct = NodeFreeDirect::PILLAR;
         return true;
     }
     // 如果节点已经最终确定，则不进行更新
     if (node_ptr->is_finalized) return true; 
     // 修正表面方向的顺序
     FARUtil::CorrectDirectOrder(node_ptr->surf_dirs, cur_dirs);
     // 将当前的表面方向添加到表面方向向量中
     node_ptr->surf_dirs_vec.push_back(cur_dirs);
     // 如果表面方向向量的大小超过池大小，则移除最早的表面方向
     if (node_ptr->surf_dirs_vec.size() > dg_params_.pool_size) {
         node_ptr->surf_dirs_vec.pop_front();
     }
     // 使用 RANSAC 算法计算导航节点的平均表面方向
     std::size_t inlier_size = 0;
     // 调用 RANSACSurfDirs 函数计算平均表面方向
     const PointPair mean_dir = FARUtil::RANSACSurfDirs(node_ptr->surf_dirs_vec, dg_params_.filter_dirs_margin, inlier_size);
     // 如果平均表面方向为默认值，则将表面方向设置为默认值，并将自由方向设置为柱子类型
     if (mean_dir.first == Point3D(0,0,-1) || mean_dir.second == Point3D(0,0,-1)) {
         node_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
         node_ptr->free_direct = NodeFreeDirect::PILLAR;
     } else {
         // 否则，将平均表面方向赋值给节点的表面方向
         node_ptr->surf_dirs = mean_dir;
         // 重新评估节点的凸性
         this->ReEvaluateConvexity(node_ptr);
     }
     // 如果内点数量超过最终确定阈值，则认为节点表面方向更新成功
     if (inlier_size > dg_params_.finalize_thred) {
         return true;
     }
     // 否则，认为节点表面方向更新失败
     return false;       
 }
 
 /**
  * @brief 重新评估导航节点的凸性
  * 
  * 该函数用于重新评估导航节点的凸性，并根据评估结果更新节点的自由方向。
  * 如果节点不是轮廓匹配节点，或者其对应的轮廓节点的多边形是柱子类型，则不进行评估。
  * 
  * @param node_ptr 要重新评估凸性的导航节点的指针
  */
 void DynamicGraph::ReEvaluateConvexity(const NavNodePtr& node_ptr) {
     // 如果节点不是轮廓匹配节点，或者其对应的轮廓节点的多边形是柱子类型，则不进行评估
     if (!node_ptr->is_contour_match || node_ptr->ctnode->poly_ptr->is_pillar) return;
     // 标记是否为墙壁
     bool is_wall = false;
     // 计算表面拓扑方向
     const Point3D topo_dir = FARUtil::SurfTopoDirect(node_ptr->surf_dirs, is_wall);
     // 如果不是墙壁
     if (!is_wall) {
         // 获取轮廓节点的位置
         const Point3D ctnode_p = node_ptr->ctnode->position;
         // 计算评估点的位置，评估点在轮廓节点位置沿着拓扑方向偏移一定距离
         const Point3D ev_p = ctnode_p + topo_dir * FARUtil::kLeafSize;
         // 检查评估点是否为凸点
         if (FARUtil::IsConvexPoint(node_ptr->ctnode->poly_ptr, ev_p)) {
             // 如果是凸点，则将节点的自由方向设置为凸
             node_ptr->free_direct = NodeFreeDirect::CONVEX;
         } else {
             // 如果不是凸点，则将节点的自由方向设置为凹
             node_ptr->free_direct = NodeFreeDirect::CONCAVE;
         }
     }
 }
 
 /**
  * @brief 为指定节点找出前两个轮廓连接节点
  * 
  * 该函数用于为给定的导航节点找出轮廓投票数最高的前两个节点，并建立连接。
  * 同时，移除不符合条件的轮廓连接。
  * 
  * @param node_ptr 要处理的导航节点的指针
  */
 void DynamicGraph::TopTwoContourConnector(const NavNodePtr& node_ptr) {
     // 存储所有有效的轮廓投票总数
     std::vector<int> votesc;
     // 遍历节点的所有轮廓投票
     for (const auto& vote : node_ptr->contour_votes) {
         // 检查投票是否有效
         if (FARUtil::IsVoteTrue(vote.second, false)) {
             // 计算该投票的总数，并添加到 votesc 向量中
             votesc.push_back(std::accumulate(vote.second.begin(), vote.second.end(), 0));
         }
     }
     // 对投票总数进行降序排序
     std::sort(votesc.begin(), votesc.end(), std::greater<int>());
     // 遍历节点的所有潜在轮廓连接节点
     for (const auto& cnode_ptr : node_ptr->potential_contours) {
         // 查找该潜在轮廓连接节点的投票记录
         const auto it = node_ptr->contour_votes.find(cnode_ptr->id);
         // DEBUG: 如果未找到投票记录，输出错误信息
         if (it == node_ptr->contour_votes.end()) ROS_ERROR("DG: contour potential node matching error");
         // 计算该投票的总数
         const int itc = std::accumulate(it->second.begin(), it->second.end(), 0);
         // 检查该投票总数是否在前两名，并且投票是否有效
         if (FARUtil::VoteRankInVotes(itc, votesc) < 2 && FARUtil::IsVoteTrue(it->second, false)) {
             // 建立轮廓连接
             DynamicGraph::AddContourConnect(node_ptr, cnode_ptr);
             // 添加边到图中
             this->AddEdge(node_ptr, cnode_ptr);
         } 
         // 如果移除轮廓连接成功，并且该节点不在多边形连接列表中
         else if (DynamicGraph::DeleteContourConnect(node_ptr, cnode_ptr) && !FARUtil::IsTypeInStack(cnode_ptr, node_ptr->poly_connects)) {
             // 从图中移除边
             this->EraseEdge(node_ptr, cnode_ptr);
         }
     }
 }
 
 /**
  * @brief 记录两个导航节点之间的轮廓连接投票信息
  * 
  * 该函数用于记录两个导航节点之间的轮廓连接投票信息。如果两个节点相同，则不进行记录。
  * 若投票队列不存在，则初始化投票队列；若存在，则更新投票队列。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  */
 void DynamicGraph::RecordContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
     // 如果两个节点相同，则不进行投票记录
     if (node_ptr1 == node_ptr2) return;
     // 在节点1的轮廓投票记录中查找节点2的记录
     const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
     // 在节点2的轮廓投票记录中查找节点1的记录
     const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
     // 如果处于调试模式
     if (FARUtil::IsDebug) {
         // 检查两个节点的轮廓投票记录是否一个存在一个不存在
         if ((it1 == node_ptr1->contour_votes.end()) != (it2 == node_ptr2->contour_votes.end())) {
             // 若存在不一致，输出错误信息
             ROS_ERROR_THROTTLE(1.0, "DG: Critical! Contour edge votes queue error.");
         }
     }
     // 如果两个节点的轮廓投票记录至少有一个不存在
     if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
         // 初始化轮廓连接投票队列
         std::deque<int> vote_queue1, vote_queue2;
         // 向两个投票队列添加初始投票值1
         vote_queue1.push_back(1), vote_queue2.push_back(1);
         // 将节点2的投票队列插入节点1的轮廓投票记录中
         node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
         // 将节点1的投票队列插入节点2的轮廓投票记录中
         node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
         // 检查两个节点是否不在对方的潜在轮廓节点列表中
         if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) {
             // 将节点2添加到节点1的潜在轮廓节点列表中
             node_ptr1->potential_contours.push_back(node_ptr2);
             // 将节点1添加到节点2的潜在轮廓节点列表中
             node_ptr2->potential_contours.push_back(node_ptr1);
         }
     } else {
         // 如果处于调试模式
         if (FARUtil::IsDebug) {
             // 检查两个节点的投票队列大小是否不一致
             if (it1->second.size() != it2->second.size()) {
                 // 若不一致，输出错误信息
                 ROS_ERROR_THROTTLE(1.0, "DG: contour connection votes are not equal.");
             }
         }
         // 向两个节点的投票队列添加新的投票值1
         it1->second.push_back(1), it2->second.push_back(1);
         // 如果节点1的投票队列大小超过预设的投票大小
         if (it1->second.size() > dg_params_.votes_size) {
             // 移除节点1和节点2投票队列的第一个元素
             it1->second.pop_front(), it2->second.pop_front();
         }
     }
 }
 
 /**
  * @brief 记录两个导航节点之间的多边形连接投票信息
  * 
  * 该函数用于记录两个导航节点之间的多边形连接投票信息。如果两个节点相同，则不进行记录。
  * 如果边缘投票队列不存在则初始化，如果已经存在，根据参数决定是否重置并更新投票队列。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param queue_size 投票队列的最大大小
  * @param is_reset 是否重置投票队列
  */
 void DynamicGraph::RecordPolygonVote(const NavNodePtr& node_ptr1, 
                                      const NavNodePtr& node_ptr2,
                                      const int& queue_size, 
                                      const bool& is_reset) 
 {
     // 如果两个节点相同，则不进行投票记录
     if (node_ptr1 == node_ptr2) return;
 
     // 在节点1的边缘投票记录中查找节点2的记录
     const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
     
     // 在节点2的边缘投票记录中查找节点1的记录
     const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
 
     // 如果处于调试模式
     if (FARUtil::IsDebug) {
         // 检查两个节点的边缘投票记录是否一个存在一个不存在
         if ((it1 == node_ptr1->edge_votes.end()) != (it2 == node_ptr2->edge_votes.end())) {
             // 若存在不一致，输出错误信息
             ROS_ERROR_THROTTLE(1.0, "DG: Critical! Polygon edge votes queue error.");
         }
     }
 
     // 如果至少有一个节点的边缘投票记录不存在
     if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
         // 初始化多边形边缘投票队列
         std::deque<int> vote_queue1, vote_queue2;
         
         // 向两个边缘投票队列添加初始投票值1
         vote_queue1.push_back(1), vote_queue2.push_back(1);
         
         // 将节点2的边缘投票队列插入节点1的边缘投票记录中
         node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
         
         // 将节点1的边缘投票队列插入节点2的边缘投票记录中
         node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
 
         // 检查两个节点是否不在对方的潜在边缘节点列表中
         if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) {
             // 将节点2添加到节点1的潜在边缘节点列表中
             node_ptr1->potential_edges.push_back(node_ptr2);
             // 将节点1添加到节点2的潜在边缘节点列表中
             node_ptr2->potential_edges.push_back(node_ptr1);
         }
     } else {
         // 如果处于调试模式
         if (FARUtil::IsDebug) {
             // 检查两个节点的投票队列大小是否不一致
             if (it1->second.size() != it2->second.size())
                 ROS_ERROR_THROTTLE(1.0, "DG: Polygon edge votes are not equal.");
         }
 
         // 根据is_reset标志决定是否清空当前投票队列
         if (is_reset) it1->second.clear(), it2->second.clear();
         
         // 向两个节点的投票队列添加新的投票值1
         it1->second.push_back(1), it2->second.push_back(1);
 
         // 如果投票队列大小超过预设的限制
         if (it1->second.size() > queue_size) {
             // 移除投票队列的第一个元素
             it1->second.pop_front(), it2->second.pop_front();
         }
     }
 }
 
 /**
  * @brief 填充两个导航节点之间的多边形边缘连接信息
  * 
  * 如果节点间没有投票记录，初始化其边缘投票结构。如若已有，则继续填充该连接信息。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param queue_size 投票队列的大小
  */
 void DynamicGraph::FillPolygonEdgeConnect(const NavNodePtr& node_ptr1,
                                         const NavNodePtr& node_ptr2,
                                         const int& queue_size)
 {
     // 如果两个节点相同，则不进行操作
     if (node_ptr1 == node_ptr2) return;
 
     // 在节点1的边缘投票记录中查找节点2的记录
     const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
     
     // 在节点2的边缘投票记录中查找节点1的记录
     const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
 
     // 如果至少有一个节点的边缘投票记录不存在
     if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) {
         // 初始化边缘投票队列
         std::deque<int> vote_queue1(queue_size, 1);
         std::deque<int> vote_queue2(queue_size, 1);
 
         // 将节点2的边缘投票队列插入节点1的边缘投票记录中
         node_ptr1->edge_votes.insert({node_ptr2->id, vote_queue1});
         
         // 将节点1的边缘投票队列插入节点2的边缘投票记录中
         node_ptr2->edge_votes.insert({node_ptr1->id, vote_queue2});
 
         // 检查两个节点是否未在彼此的潜在边缘节点列表中
         if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_edges) && 
             !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_edges)) 
         {
             // 两者互相添加为潜在边缘节点
             node_ptr1->potential_edges.push_back(node_ptr2);
             node_ptr2->potential_edges.push_back(node_ptr1);
         }
 
         // 添加链接
         if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->poly_connects) &&
             !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->poly_connects)) 
         {
             // 增加两个节点的多边形连接关系
             node_ptr1->poly_connects.push_back(node_ptr2);
             node_ptr2->poly_connects.push_back(node_ptr1);
         }
     }
 }
 
 /**
  * @brief 填充两个导航节点之间的轮廓连接信息
  * 
  * 如果目前没有轮廓投票及连接信息，就初始化其轮廓投票结构，并增添相关连接信息。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param queue_size 投票队列的大小
  */
 void DynamicGraph::FillContourConnect(const NavNodePtr& node_ptr1,
                                     const NavNodePtr& node_ptr2,
                                     const int& queue_size)
 {
     // 如果两个节点相同，则不进行操作
     if (node_ptr1 == node_ptr2) return;
 
     // 查找节点间的投票记录
     const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
     const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
     
     // 初始化轮廓投票队列
     std::deque<int> vote_queue1(queue_size, 1);
     std::deque<int> vote_queue2(queue_size, 1);
 
     // 如果两节点的轮廓投票记录至少有一个不存在
     if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) {
         // 将节点2的轮廓投票队列插入节点1的轮廓投票记录中
         node_ptr1->contour_votes.insert({node_ptr2->id, vote_queue1});
         
         // 将节点1的轮廓投票队列插入节点2的轮廓投票记录中
         node_ptr2->contour_votes.insert({node_ptr1->id, vote_queue2});
         
         // 检查两个节点是否未在彼此的潜在轮廓节点列表中
         if (!FARUtil::IsTypeInStack(node_ptr1, node_ptr2->potential_contours) && 
             !FARUtil::IsTypeInStack(node_ptr2, node_ptr1->potential_contours)) 
         {
             // 互相添加为潜在轮廓节点
             node_ptr1->potential_contours.push_back(node_ptr2);
             node_ptr2->potential_contours.push_back(node_ptr1);
         }
 
         // 增加轮廓连接
         DynamicGraph::AddContourConnect(node_ptr1, node_ptr2);
     }
 }
 
 /**
  * @brief 填充两个导航节点之间的轨迹连接信息
  * 
  * 此方法用于确保两个节点之间的轨迹推测连接成立。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  */
 void DynamicGraph::FillTrajConnect(const NavNodePtr& node_ptr1,
                                  const NavNodePtr& node_ptr2)
 {
     // 如果两个节点相同，则不进行操作
     if (node_ptr1 == node_ptr2) return;
 
     // 在节点1的轨迹投票记录中查找节点2的记录
     const auto it1 = node_ptr1->trajectory_votes.find(node_ptr2->id);
     
     // 在节点2的轨迹投票记录中查找节点1的记录
     const auto it2 = node_ptr2->trajectory_votes.find(node_ptr1->id);
 
     // 如果至少有一个节点的轨迹投票记录不存在
     if (it1 == node_ptr1->trajectory_votes.end() || it2 == node_ptr2->trajectory_votes.end()) {
         // 初始化轨迹投票，使用0表示未连接
         node_ptr1->trajectory_votes.insert({node_ptr2->id, 0});
         node_ptr2->trajectory_votes.insert({node_ptr1->id, 0});
 
         // 检查两个节点是否未在彼此的轨迹连接列表中
         if (!FARUtil::IsTypeInStack(node_ptr2, node_ptr1->trajectory_connects) &&
             !FARUtil::IsTypeInStack(node_ptr1, node_ptr2->trajectory_connects)) 
         {   
             // 互相添加为轨迹连接节点
             node_ptr1->trajectory_connects.push_back(node_ptr2);
             node_ptr2->trajectory_connects.push_back(node_ptr1);
         }
     }
 }
 
 /**
  * @brief 删除两个导航节点之间的多边形连接投票信息
  *
  * 用于减少或清理节点间的连接，依据传进来的布尔参数决定是否重置。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param queue_size 投票队列的最大大小
  * @param is_reset 是否重置投票数组
  */
 void DynamicGraph::DeletePolygonVote(const NavNodePtr& node_ptr1, 
                                      const NavNodePtr& node_ptr2,
                                      const int& queue_size,
                                      const bool& is_reset) 
 {
     // 在节点1的边缘投票记录中查找节点2的记录
     const auto it1 = node_ptr1->edge_votes.find(node_ptr2->id);
     
     // 在节点2的边缘投票记录中查找节点1的记录
     const auto it2 = node_ptr2->edge_votes.find(node_ptr1->id);
     
     // 如果任一记录不存在，则返回
     if (it1 == node_ptr1->edge_votes.end() || it2 == node_ptr2->edge_votes.end()) return;
 
     // 如果是重置模式，则清空投票数据
     if (is_reset) it1->second.clear(), it2->second.clear();
     
     // 否则向投票队列中添加0值，表示反对的投票
     it1->second.push_back(0), it2->second.push_back(0);
     
     // 如果投票队列长度超过规定的大小，移除最旧的投票
     if (it1->second.size() > queue_size) {
         it1->second.pop_front(), it2->second.pop_front();
     }
 }
 
 /**
  * @brief 删除两个导航节点之间的轮廓连接投票信息
  *
  * 用于减少或清理节点间的轮廓连接，通常是在识别出无效连接后执行。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  */
 void DynamicGraph::DeleteContourVote(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
     // 在节点1的轮廓投票记录中查找节点2的记录
     const auto it1 = node_ptr1->contour_votes.find(node_ptr2->id);
     
     // 在节点2的轮廓投票记录中查找节点1的记录
     const auto it2 = node_ptr2->contour_votes.find(node_ptr1->id);
 
     // 如果任一记录不存在，则返回
     if (it1 == node_ptr1->contour_votes.end() || it2 == node_ptr2->contour_votes.end()) return; 
 
     // 否则向投票队列中添加0值，表示反对的投票
     it1->second.push_back(0), it2->second.push_back(0);
     
     // 如果投票队列数量超过预设大小，移除最旧的投票
     if (it1->second.size() > dg_params_.votes_size) {
         it1->second.pop_front(), it2->second.pop_front();
     }
 }
 /**
  * @brief 确定导航节点是否处于激活状态
  *
  * 该函数检查给定的导航节点是否已经被标记为激活。如果未激活，则根据不同条件评估它并激活。如果条件满足，返回true，否则返回false。
  * 
  * @param node_ptr 导航节点的指针
  * @return true 如果节点处于激活状态
  * @return false 如果节点不再激活状态
  */
 bool DynamicGraph::IsActivateNavNode(const NavNodePtr& node_ptr) {
     // 如果节点已活跃, 则返回true
     if (node_ptr->is_active) return true;
     
     // 检查当前环境中是否有新的点接近该节点
     if (FARUtil::IsPointNearNewPoints(node_ptr->position, true)) {
         node_ptr->is_active = true; // 激活节点
         return true;
     }
     
     // 检查此导航节点是否为空闲
     if (FARUtil::IsFreeNavNode(node_ptr)) {
         // 判断节点到里程计节点之间的距离
         const bool is_nearby = (node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kNearDist ? true : false;
         
         // 如果最近，则激活节点
         if (is_nearby) {
             node_ptr->is_active = true;
             return true;
         }
         
         // 检查此节点是否在与里程计节点的连接中
         if (FARUtil::IsTypeInStack(node_ptr, odom_node_ptr_->connect_nodes)) {
             node_ptr->is_active = true;
             return true;
         }
         
         // 检查连接节点是否都是激活状态
         bool is_connects_activate = true;
         for (const auto& cnode_ptr : node_ptr->connect_nodes) {
             if (!cnode_ptr->is_active) {
                 is_connects_activate = false; // 如果其中任何一个未激活则设置为 false
                 break;
             }
         }
         
         // 如果所有连接节点均激活且连接列表非空，则激活当前节点
         if ((is_connects_activate && !node_ptr->connect_nodes.empty())) {
             node_ptr->is_active = true;
             return true;
         }
     }
     return false; // 未激活状态
 }
 
 /**
  * @brief 更新全局附近节点信息
  * 
  * 该函数清除当前存储的多个邻近节点集合，并重新填充这些集合以便能更好地进行路径规划与决策制定。
  */
 void DynamicGraph::UpdateGlobalNearNodes() {
     /* 清空当前所有邻近节点的集合 --> near_nav_nodes_ */
     near_nav_nodes_.clear(), wide_near_nodes_.clear(), extend_match_nodes_.clear();
     margin_near_nodes_.clear(); internav_near_nodes_.clear(), surround_internav_nodes_.clear();
     
     // 遍历全局图中的每个节点
     for (const auto& node_ptr : globalGraphNodes_) {
         node_ptr->is_near_nodes = false; // 初始化标志位
         node_ptr->is_wide_near  = false;
 
         // 检测节点是否在扩展匹配范围内，以及其激活状态或是否处于边界上
         if (FARUtil::IsNodeInExtendMatchRange(node_ptr) && (!node_ptr->is_active || MapHandler::IsNavPointOnTerrainNeighbor(node_ptr->position, true))) {
             if (FARUtil::IsOutsideGoal(node_ptr)) continue; // 如果节点在目标之外则跳过
             
             // 如果节点激活或是边界节点，加入扩展匹配节点集
             if (this->IsActivateNavNode(node_ptr) || node_ptr->is_boundary)
                 extend_match_nodes_.push_back(node_ptr);
             
             // 检查节点是否在本地可用范围内且位置符合条件
             if (FARUtil::IsNodeInLocalRange(node_ptr) && IsPointOnTerrain(node_ptr->position)) {
                 wide_near_nodes_.push_back(node_ptr); // 加入宽阔重叠节点集合
                 node_ptr->is_wide_near = true; // 设置广泛邻近的标志
                 
                 // 增加紧密邻近集合
                 if (node_ptr->is_active || node_ptr->is_boundary) {
                     near_nav_nodes_.push_back(node_ptr);
                     node_ptr->is_near_nodes = true; // 标记为邻近
 
                     // 若为导航点，更新亮度值并加入内部导航节点集合
                     if (node_ptr->is_navpoint) {
                         node_ptr->position.intensity = node_ptr->fgscore; 
                         internav_near_nodes_.push_back(node_ptr);
 
                         // 检查是否在局部规划范围以内
                         if ((node_ptr->position - odom_node_ptr_->position).norm() < FARUtil::kLocalPlanRange / 2.0f) {
                             surround_internav_nodes_.push_back(node_ptr); // 将节点加入周围内部层集合
                         }
                     }
                 }
             } else if (node_ptr->is_active || node_ptr->is_boundary) {
                 margin_near_nodes_.push_back(node_ptr); // 否则加入边缘临近节点记录
             }
         }
     }
 
     // 添加额外的车辆里的连接（odometry connections）到宽敞的邻近堆栈
     for (const auto& cnode_ptr : odom_node_ptr_->connect_nodes) { 
         if (FARUtil::IsOutsideGoal(cnode_ptr)) continue; // 跳过超出目标的连接
         
         // 如果没有标记为广泛邻近则添加
         if (!cnode_ptr->is_wide_near) {
             wide_near_nodes_.push_back(cnode_ptr);
             cnode_ptr->is_wide_near = true;
         }
 
         // 遍历其连接节点进行同样处理
         for (const auto& c2node_ptr : cnode_ptr->connect_nodes) {
             if (!c2node_ptr->is_wide_near && !FARUtil::IsOutsideGoal(c2node_ptr)) {
                 wide_near_nodes_.push_back(c2node_ptr);
                 c2node_ptr->is_wide_near = true;
             }
         }
     }
 
     // 如果存在任一内部导航节点，从中选择靠近odom的那一个
     if (!internav_near_nodes_.empty()) { 
         std::sort(internav_near_nodes_.begin(), internav_near_nodes_.end(), nodeptr_icomp()); // 排序内部导航节点
 
         for (std::size_t i=0; i<internav_near_nodes_.size(); i++) {
             const NavNodePtr temp_internav_ptr = internav_near_nodes_[i];
 
             // 检查是否是可能的连通性
             if (FARUtil::IsTypeInStack(temp_internav_ptr, odom_node_ptr_->potential_edges) && this->IsInternavInRange(temp_internav_ptr)) {
                 if (cur_internav_ptr_ == NULL || temp_internav_ptr == cur_internav_ptr_ || 
                    (temp_internav_ptr->position - cur_internav_ptr_->position).norm() < FARUtil::kNearDist ||
                     FARUtil::IsTypeInStack(temp_internav_ptr, cur_internav_ptr_->connect_nodes)) 
                 {   
                     this->UpdateCurInterNavNode(temp_internav_ptr);  
                 } else {
                     is_bridge_internav_ = true; // 表示存在桥接的内部导航节点
                 }
                 break; // 找到合适的后可以退出循环
             }
         }
     }
 }
 
 /**
  * @brief 重新评估角落节点
  *
  * 此函数用于复审特定导航节点是否需要改变状态。依据各种条件如边界、导航点及其周围新创建的地点等决定其激活情况。
  *
  * @param node_ptr 要评估的导航节点指针
  * @return true 如果节点需要被维护或者状态正常
  * @return false 如果节点的不活动
  */
 bool DynamicGraph::ReEvaluateCorner(const NavNodePtr node_ptr) {
     // 如果节点是边界上的话直接返回true
     if (node_ptr->is_boundary) return true;
 
     // 如果节点是导航点
     if (node_ptr->is_navpoint) {
         // 如果在周围内部导航节点中，并且该节点占物体空间
         if (FARUtil::IsTypeInStack(node_ptr, surround_internav_nodes_) && this->IsNodeInTerrainOccupy(node_ptr)) {
             return false; // 不需要变化
         }
         return true; // 保持活动状态
     }
 
     // 检查环境中是否有新变化发生
     const bool is_near_new = FARUtil::IsPointNearNewPoints(node_ptr->position, false);
     if (is_near_new) { // 如果发现附近环境变动；
         this->ResetNodeFilters(node_ptr); // 重置相关过滤器
         if (!node_ptr->is_contour_match) this->ResetNodeConnectVotes(node_ptr); // 重新约定节点投票
     }
 
     // 如果不是轮廓匹配，且在边界或有新点接近，则保持不激活；否则启用
     if (!node_ptr->is_contour_match) {
         if (FARUtil::IsPointInMarginRange(node_ptr->position) || is_near_new) return false;
         return true;
     }
     
     // 对于轮廓必要判断完成；
     if (node_ptr->is_finalized) return true;
 
     bool is_pos_cov  = false; // 是否已覆盖位置
     bool is_dirs_cov = false; // 是否已覆蓋方向
     
     // 如果这是个轮廓节点
     if (node_ptr->is_contour_match) {
         // 更新节点位置和方向，以获得最新结果
         is_pos_cov  = this->UpdateNodePosition(node_ptr, node_ptr->ctnode->position);
         is_dirs_cov = this->UpdateNodeSurfDirs(node_ptr, node_ptr->ctnode->surf_dirs);
         
         if (FARUtil::IsDebug) ROS_ERROR_COND(node_ptr->free_direct == NodeFreeDirect::UNKNOW, "DG: node free space is unknown.");
     }
     
     // 如果位置和方向都更新成功，则将其最终化
     if (is_pos_cov && is_dirs_cov) node_ptr->is_finalized = true;
 
     return true; // 返回标准
 }
 
 /**
  * @brief 根据地形重新评估两个节点的连接关系
  *
  * 通过对从第一个节点到第二个节点的路径进行地形规划来确认作为有效连接。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @return true 如果有有效的连接
  * @return false 如果无有效连接
  */
 bool DynamicGraph::ReEvaluateConnectUsingTerrian(const NavNodePtr& node_ptr1, const NavNodePtr node_ptr2) {
     PointStack terrain_path; // 定义路径栈，用于保存路径
     // 调用地形规划函数，如果成功找到路径
     if (terrain_planner_.PlanPathFromNodeToNode(node_ptr1, node_ptr2, terrain_path)) {
         return true; // 如果路径计划成功则返回 true
     }
     return false; // 如果失败则返回 false
 }
 