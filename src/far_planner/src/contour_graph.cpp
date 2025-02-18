/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */

 #include "far_planner/contour_graph.h"
 #include "far_planner/intersection.h"
 
 /***************************************************************************************/
 
 // 初始化轮廓图的参数和状态
 void ContourGraph::Init(const ContourGraphParams& params) {
     ctgraph_params_ = params; // 设置参数
     ContourGraph::contour_graph_.clear(); // 清空轮廓图
     ContourGraph::contour_polygons_.clear(); // 清空轮廓多边形
     ALIGN_ANGLE_COS = cos(M_PI - FARUtil::kAcceptAlign / 2.0f); // 计算角度对齐余弦值
     is_robot_inside_poly_ = false; // 标记机器人是否在多边形内部
     ContourGraph::global_contour_set_.clear(); // 清空全局轮廓集合
     ContourGraph::boundary_contour_set_.clear(); // 清空边界轮廓集合
 }
 
 // 更新轮廓图的方法，根据传入的导航节点和过滤后的轮廓点
 void ContourGraph::UpdateContourGraph(const NavNodePtr& odom_node_ptr,
                                       const std::vector<std::vector<Point3D>>& filtered_contours) {
     odom_node_ptr_ = odom_node_ptr; // 保存当前导航节点指针
     this->ClearContourGraph(); // 清除现有的轮廓图
 
     // 遍历每个过滤后的多边形，创建并添加到轮廓多边形中
     for (const auto& poly : filtered_contours) {
         PolygonPtr new_poly_ptr = NULL; // 新多边形指针
         this->CreatePolygon(poly, new_poly_ptr); // 创建多边形
         this->AddPolyToContourPolygon(new_poly_ptr); // 添加多边形
     }
     
     // 更新未障碍物的机器人位置
     ContourGraph::UpdateOdomFreePosition(odom_node_ptr_, FARUtil::free_odom_p);
     
     // 遍历所有轮廓多边形进行处理
     for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
         poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_ptr->vertices, FARUtil::free_odom_p); // 判断机器人是否在多边形内
         CTNodePtr new_ctnode_ptr = NULL;
         
         if (poly_ptr->is_pillar) { // 如果是柱子类型的多边形
             Point3D mean_p = FARUtil::AveragePoints(poly_ptr->vertices); // 计算平均点作为新CT节点的位置
             this->CreateCTNode(mean_p, new_ctnode_ptr, poly_ptr, true); // 创建CT节点
             this->AddCTNodeToGraph(new_ctnode_ptr); // 添加到图中
         } else {            
             CTNodeStack ctnode_stack; // CT节点栈
             ctnode_stack.clear();
             const int N = poly_ptr->vertices.size(); // 多边形顶点数量
             
             // 为每个顶点创建CT节点并推入栈中
             for (std::size_t idx=0; idx<N; idx++) {
                 this->CreateCTNode(poly_ptr->vertices[idx], new_ctnode_ptr, poly_ptr, false);
                 ctnode_stack.push_back(new_ctnode_ptr);
             }
 
             // 在邻接的CT节点间建立连结
             for (int idx=0; idx<N; idx++) {
                 int ref_idx = FARUtil::Mod(idx-1, N);
                 ctnode_stack[idx]->front = ctnode_stack[ref_idx]; // 前向连接
                 ref_idx = FARUtil::Mod(idx+1, N);
                 ctnode_stack[idx]->back = ctnode_stack[ref_idx]; // 后向连接
                 this->AddCTNodeToGraph(ctnode_stack[idx]); // 添加到图中
             }
 
             // 将每个多边形的首个CT节点压入多边形CT节点栈
             if (!ctnode_stack.empty()) ContourGraph::polys_ctnodes_.push_back(ctnode_stack.front());
         }
     }
     
     // 分析运动表面角度和凸性特征
     this->AnalysisSurfAngleAndConvexity(ContourGraph::contour_graph_);
 }
 
 /* 匹配当前轮廓与全局导航节点 */
 void ContourGraph::MatchContourWithNavGraph(const NodePtrStack& global_nodes, const NodePtrStack& near_nodes, CTNodeStack& new_convex_vertices) {
     // 重置匹配标志
     for (const auto& node_ptr : global_nodes) {
         node_ptr->is_contour_match = false; // 全局导航节点不匹配
         node_ptr->ctnode = NULL; // 指定的CT节点为空
     }
 
     // 遍历各个CT节点进行距离匹配
     for (const auto& ctnode_ptr : ContourGraph::contour_graph_) {
         ctnode_ptr->is_global_match = false; // 重置为未匹配
         ctnode_ptr->nav_node_id = 0; // 导航节点ID重置
         if (ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
             const NavNodePtr matched_node = this->NearestNavNodeForCTNode(ctnode_ptr, near_nodes); // 获取最近导航节点
             if (matched_node != NULL && IsCTMatchLineFreePolygon(ctnode_ptr, matched_node, false)) {
                 this->MatchCTNodeWithNavNode(ctnode_ptr, matched_node); // 对应CT节点与导航节点匹配
             }   
         }
     }
     
     // 检查封闭多边形
     this->EnclosePolygonsCheck();
     new_convex_vertices.clear();
 
     // 获取新的凸包顶点
     for (const auto& ctnode_ptr : ContourGraph::contour_graph_) {
         if (!ctnode_ptr->is_global_match && ctnode_ptr->free_direct != NodeFreeDirect::UNKNOW) {
             if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) { // 检查墙体轮廓
                 const float dot_value = ctnode_ptr->surf_dirs.first * ctnode_ptr->surf_dirs.second; 
                 if (dot_value < ALIGN_ANGLE_COS) continue; // 发现墙体
             }
             new_convex_vertices.push_back(ctnode_ptr); // 加入新生成的凸顶点栈
         }
     }
 }
 
 // 检查两个导航节点之间的连接是否没有障碍物
 bool ContourGraph::IsNavNodesConnectFreePolygon(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
     if (node_ptr1->is_navpoint || node_ptr2->is_navpoint) {
         if ((node_ptr1->position - node_ptr2->position).norm() < FARUtil::kNavClearDist) { // 连接至内部导航点
             return true; // 确认节点之间可以连接
         }
     }
     const bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr1->position, node_ptr2->position); // 是否需要全局检查
     ConnectPair cedge = ContourGraph::ReprojectEdge(node_ptr1, node_ptr2, FARUtil::kProjectDist, is_global_check); // 从第一个节点到第二个节点的边缘重投影
     if (node_ptr1->is_odom) {
         cedge.start_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y); // 若是里程计则开始点更新
     } else if (node_ptr2->is_odom) {
         cedge.end_p = cv::Point2f(FARUtil::free_odom_p.x, FARUtil::free_odom_p.y); // 若是里程计则结束点更新
     }
     ConnectPair bd_cedge = cedge; // 边界边缘副本
     const HeightPair h_pair(node_ptr1->position, node_ptr2->position); // 高度成对
     if (!node_ptr1->is_boundary) bd_cedge.start_p = cv::Point2f(node_ptr1->position.x, node_ptr1->position.y);
     if (!node_ptr2->is_boundary) bd_cedge.end_p = cv::Point2f(node_ptr2->position.x, node_ptr2->position.y);
     return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check); // 检查这些点之间的连接是否无障碍
 }
 
 // 检查三维点p1与p2之间的连接是否没有障碍物
 bool ContourGraph::IsPoint3DConnectFreePolygon(const Point3D& p1, const Point3D& p2) {
     const bool is_global_check = ContourGraph::IsNeedGlobalCheck(p1, p2); // 是否需要全局检查
     const ConnectPair ori_cedge(p1, p2); // 原始连接边缘对
     const ConnectPair cedge = ori_cedge; 
     const HeightPair h_pair(p1, p2); // 高度对
     return ContourGraph::IsPointsConnectFreePolygon(cedge, ori_cedge, h_pair, is_global_check); // 检查这些点之间的连接是否无障碍
 }
 
 // 检查两点所形成的边缘是否与边界发生碰撞
 bool ContourGraph::IsEdgeCollideBoundary(const Point3D& p1, const Point3D& p2) {
     if (ContourGraph::boundary_contour_.empty()) return false; // 如果没有边界轮廓，直接返回false
     const ConnectPair edge = ConnectPair(p1, p2); // 当前边缘
     for (const auto& contour : ContourGraph::boundary_contour_) {
         if (ContourGraph::IsEdgeCollideSegment(contour, edge)) {return true;} // 和某条边界相交则返回true
     }
     return false;
 }
 
 // 检查导航到目标节点之间的连接是否没有障碍物
 bool ContourGraph::IsNavToGoalConnectFreePolygon(const NavNodePtr& node_ptr, const NavNodePtr& goal_ptr) {
     // 检查当前节点和目标节点之间的欧几里得距离是否小于安全距离，如果是，则认为路径可通行
     if ((node_ptr->position - goal_ptr->position).norm() < FARUtil::kNavClearDist) return true;
 
     // 创建高度对，存储起始点和目标点的高度信息
     HeightPair h_pair(node_ptr->position, goal_ptr->position);
     
     // 判断是否需要全局检查
     bool is_global_check = ContourGraph::IsNeedGlobalCheck(node_ptr->position, goal_ptr->position);
 
     // 如果为多层环境，执行额外检查
     if (FARUtil::IsMultiLayer) {
         // 如果节点不在同一层且不是前沿节点，则返回不可通行
         if (!FARUtil::IsAtSameLayer(node_ptr, goal_ptr) && !node_ptr->is_frontier) return false;
         h_pair = HeightPair(goal_ptr->position.z, goal_ptr->position.z);  // 更新高度对只包含z值
         is_global_check = true;  // 强制开启全局检查
     }
 
     // 重投影边界检测
     const ConnectPair cedge = ContourGraph::ReprojectEdge(node_ptr, goal_ptr, FARUtil::kProjectDist, is_global_check);
     ConnectPair bd_cedge = cedge;  // 拷贝重投影边界
 
     // 根据是否为边界点调整开始和结束点
     if (!node_ptr->is_boundary) bd_cedge.start_p = cv::Point2f(node_ptr->position.x, node_ptr->position.y);
     if (!goal_ptr->is_boundary) bd_cedge.end_p = cv::Point2f(goal_ptr->position.x, goal_ptr->position.y);
 
     // 检查连接两者的边界/多边形是否可以通行
     return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
 }
 
 bool ContourGraph::IsCTMatchLineFreePolygon(const CTNodePtr& matched_ctnode, const NavNodePtr& matched_navnode, const bool& is_global_check) {
     // 检查匹配的轮廓节点与导航节点之间的距离
     if ((matched_ctnode->position - matched_navnode->position).norm() < FARUtil::kNavClearDist) return true;
 
     // 创建高度对用于碰撞检测
     const HeightPair h_pair(matched_ctnode->position, matched_navnode->position);
     const ConnectPair bd_cedge = ConnectPair(matched_ctnode->position, matched_navnode->position);
     const ConnectPair cedge = ContourGraph::ReprojectEdge(matched_ctnode, matched_navnode, FARUtil::kProjectDist);
 
     // 调用函数判断边界/多边形是否可以通行
     return ContourGraph::IsPointsConnectFreePolygon(cedge, bd_cedge, h_pair, is_global_check);
 }
 
 // 检查两点之间的连线是否可以自由穿过多边形和轮廓，不受障碍物阻挡。
 bool ContourGraph::IsPointsConnectFreePolygon(const ConnectPair& cedge,
                                               const ConnectPair& bd_cedge,
                                               const HeightPair h_pair,
                                               const bool& is_global_check)
 {
     // 检查各个边界上的线段是否相交
     for (const auto& contour : ContourGraph::boundary_contour_) {
         // 检查高度范围内的边界是否有重叠
         if (!ContourGraph::IsEdgeOverlapInHeight(h_pair, HeightPair(contour.first, contour.second))) continue;
         
         // 检查是否有碰撞发生
         if (ContourGraph::IsEdgeCollideSegment(contour, bd_cedge)) {
             return false;
         }
     }
 
     // 如果没有进行全局检查
     if (!is_global_check) {
         // 检查本地范围内的多边形
         const Point3D center_p = Point3D((cedge.start_p.x + cedge.end_p.x) / 2.0f,
                                          (cedge.start_p.y + cedge.end_p.y) / 2.0f,
                                          0.0f);
         for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
             if (poly_ptr->is_pillar) continue;  // 跳过支柱
             
             // 检测点是否在多边形内部，及边缘是否有碰撞
             if ((poly_ptr->is_robot_inside != FARUtil::PointInsideAPoly(poly_ptr->vertices, center_p)) || 
                 ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) 
             {
                 return false;
             }
         }
 
         // 检查未匹配的本地轮廓
         for (const auto& contour : ContourGraph::unmatched_contour_) {
             if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                 return false;
             }
         }
 
         // 检查任何非活跃的本地轮廓
         for (const auto& contour : ContourGraph::inactive_contour_) {
             if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                 return false;
             }
         }
     } else {  // 进行全局检查
         for (const auto& contour : ContourGraph::global_contour_) {
             if (!ContourGraph::IsEdgeOverlapInHeight(h_pair, HeightPair(contour.first, contour.second))) continue;
             
             // 检测全局轮廓的边界和节点e都有冲突
             if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
                 return false;
             }
         }
         for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
             if (poly_ptr->is_pillar) continue;  // 忽略支柱
             if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
                 return false;
             }
         }
     }
     return true;  // 所有检查通过，最终返回可通行状态
 }
 
 //  检查两个导航节点是否可以通过轮廓图连通
 bool ContourGraph::IsNavNodesConnectFromContour(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
     //如果其中一个节点是odom类型则返回不可连通
     if (node_ptr1->is_odom || node_ptr2->is_odom) return false;
 
     // 获取节点对应的CTNode
     const CTNodePtr ctnode1 = node_ptr1->ctnode;
     const CTNodePtr ctnode2 = node_ptr2->ctnode;
 
     // 空指针检查，以及两个CTNode 是否相同
     if (ctnode1 == NULL || ctnode2 == NULL || ctnode1 == ctnode2) return false;
 
     // 检查这两个CT节点之间是否可连通
     return ContourGraph::IsCTNodesConnectFromContour(ctnode1, ctnode2);
 }
 
 //检查两个 CTNode 是否可以通过轮廓图连通
 bool ContourGraph::IsCTNodesConnectFromContour(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2) {
     // 如果 CTNodes 是同一个或者属于不同的多边形，返回不可连通
     if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
 
     // 检查边界碰撞
     const ConnectPair cedge = ConnectPair(ctnode1->position, ctnode2->position);
     for (const auto& contour : ContourGraph::boundary_contour_) {
         // 若边界有碰撞则返回不可连通
         if (ContourGraph::IsEdgeCollideSegment(contour, cedge)) {
             return false;
         }
     }
 
     // 前向搜索看看能不能到达目标CTNode
     CTNodePtr next_ctnode = ctnode1->front; 
     while (next_ctnode != NULL && next_ctnode != ctnode1) {
         if (next_ctnode == ctnode2) {
             return true;  // 找到了目标节点
         }
         // 防止错误的全局匹配节点并且确保下一个节点在合理范围内
         if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) 
         {
             break;  // 达到边界退出
         } else {
             next_ctnode = next_ctnode->front;  // 尝试前进
         }
     }
     
     // 向后搜索看看还能否达到目标CTNode
     next_ctnode = ctnode1->back;
     while (next_ctnode != NULL && next_ctnode != ctnode1) {
         if (next_ctnode == ctnode2) {
             return true;  // 成功找到了目标节点
         }
         // 同样处理全局匹配条件
         if (next_ctnode->is_global_match || !FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) 
         {
             break;  // 达到边界时退出
         } else {
             next_ctnode = next_ctnode->back;  // 尝试后退
         }
     }
     return false;  // 搜索完成仍然找不到目标节点
 }
 
 /**
  * @brief 从给定的 CTNode 开始，查找第一个具有全局匹配标志的 CTNode
  * 
  * 该函数会从传入的 CTNode 开始，沿着其前向连接方向遍历，直到找到第一个具有全局匹配标志的 CTNode，
  * 若遍历一圈仍未找到，则返回空指针。
  * 
  * @param ctnode_ptr 起始的 CTNode 指针
  * @return CTNodePtr 第一个具有全局匹配标志的 CTNode 指针，若未找到则返回 NULL
  */
 CTNodePtr ContourGraph::FirstMatchedCTNode(const CTNodePtr& ctnode_ptr) {
     // 如果当前节点已经是全球匹配，那么直接返回
     if (ctnode_ptr->is_global_match) return ctnode_ptr;
 
     // 从当前节点的前一个节点开始遍历
     CTNodePtr cur_ctnode_ptr = ctnode_ptr->front;
     // 遍历寻找第一个全球匹配的 CTNode
     while (cur_ctnode_ptr != ctnode_ptr) {
         // 找到第一个全球匹配的 CTNode 则返回
         if (cur_ctnode_ptr->is_global_match) return cur_ctnode_ptr;  
         // 移动到下一个 CTNode
         cur_ctnode_ptr = cur_ctnode_ptr->front;  
     }
     // 如果都找不到就返回空指针
     return NULL;  
 }
 
 /**
  * @brief 将超出范围的导航节点与 CT 节点进行匹配
  * 
  * 该函数尝试将一个超出范围的导航节点与临近的导航节点进行匹配，
  * 临近节点需已经与 CT 节点匹配。函数会计算超出范围节点到临近节点与其匹配的 CT 节点连线的垂直距离，
  * 并返回垂直距离最小且在安全阈值之内的临近节点。
  * 
  * @param out_node_ptr 超出范围的导航节点指针
  * @param near_nodes 临近导航节点的指针栈
  * @return NavNodePtr 最佳匹配的临近导航节点指针，如果没有合适的匹配则返回 NULL
  */
 NavNodePtr ContourGraph::MatchOutrangeNodeWithCTNode(const NavNodePtr& out_node_ptr, const NodePtrStack& near_nodes) {
     // 如果附近节点集合为空，返回空指针
     if (near_nodes.empty()) return NULL;
 
     float min_dist = FARUtil::kINF;  // 初始化最小距离为无穷大
     NavNodePtr min_matched_node = NULL;  // 存储最近的匹配节点
     
     // 遍历所有临近节点
     for (const auto& node_ptr : near_nodes) {
         if (!node_ptr->is_contour_match) continue;  // 跳过不匹配的节点
 
         CTNodePtr matched_ctnode = NULL;
         // 查看是否与当前节点形成匹配
         if (IsContourLineMatch(node_ptr, out_node_ptr, matched_ctnode)) {
             // 计算从该节点到out_node的垂直距离
             const float dist = FARUtil::VerticalDistToLine2D(node_ptr->position, matched_ctnode->position, out_node_ptr->position);
             if (dist < min_dist) {  // 如果这个距离更小
                 min_dist = dist;      // 更新最小距离
                 min_matched_node = node_ptr;  // 保存此节点作为最佳匹配
             }
         }
     }
     // 返回距离在安全阈值之内的最佳匹配节点，否则返回空指针
     if (min_dist < FARUtil::kNavClearDist) {
         return min_matched_node;
     }
     return NULL;
 }
 
 /**
  * @brief 检查轮廓线是否匹配
  * 
  * 该函数用于检查两个导航节点之间的连线是否与轮廓节点的连线匹配。
  * 它会分别向前和向后遍历轮廓节点，检查连线的匹配百分比和路径是否畅通。
  * 
  * @param inNode_ptr 内部导航节点的指针
  * @param outNode_ptr 外部导航节点的指针
  * @param matched_ctnode 匹配的轮廓节点指针，若匹配成功则存储该节点，否则为 NULL
  * @return bool 如果匹配成功则返回 true，否则返回 false
  */
 bool ContourGraph::IsContourLineMatch(const NavNodePtr& inNode_ptr, const NavNodePtr& outNode_ptr, CTNodePtr& matched_ctnode) {
     // 获取内部导航节点对应的轮廓节点
     const CTNodePtr ctnode_ptr = inNode_ptr->ctnode;
     // 初始化匹配的轮廓节点为 NULL
     matched_ctnode = NULL;
     // 如果轮廓节点为空或者其对应的多边形是柱子类型，则直接返回 false
     if (ctnode_ptr == NULL || ctnode_ptr->poly_ptr->is_pillar) return false;
 
     // 检查向前方向
     // 定义内部导航节点和外部导航节点之间的连线
     const PointPair line1(inNode_ptr->position, outNode_ptr->position);
     // 获取当前轮廓节点的前一个节点
     CTNodePtr next_ctnode = ctnode_ptr->front;
     // 记录前一个轮廓节点
     CTNodePtr prev_ctnode = ctnode_ptr;
     // 循环遍历前向节点，直到遇到全局匹配节点、回到当前节点或者超出范围
     while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
            FARUtil::IsInCylinder(ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) 
     {
         // 检查节点是否不在边缘范围内且距离大于匹配距离
         if (!FARUtil::IsPointInMarginRange(next_ctnode->position) &&
             (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) 
         {
             // 定义当前轮廓节点和下一个轮廓节点之间的连线
             const PointPair line2(ctnode_ptr->position, next_ctnode->position);
             // 检查两条线的匹配百分比是否大于 0.99
             if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                 // 检查从下一个轮廓节点到外部导航节点的连线是否畅通
                 if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                     // 若畅通，则记录匹配的轮廓节点并返回 true
                     matched_ctnode = next_ctnode;
                     return true;
                 }
             }
         }
         // 更新前一个节点
         prev_ctnode = next_ctnode;
         // 移动到下一个前向节点
         next_ctnode = next_ctnode->front;
     }
 
     // 检查向后方向
     // 获取当前轮廓节点的后一个节点
     next_ctnode = ctnode_ptr->back;
     // 记录前一个轮廓节点
     prev_ctnode = ctnode_ptr;
     // 循环遍历后向节点，直到遇到全局匹配节点、回到当前节点或者超出范围
     while (!next_ctnode->is_global_match && next_ctnode != ctnode_ptr &&
            FARUtil::IsInCylinder(ctnode_ptr->position, next_ctnode->position, prev_ctnode->position, FARUtil::kNearDist, true)) 
     {
         // 检查节点是否不在边缘范围内且距离大于匹配距离
         if (!FARUtil::IsPointInMarginRange(next_ctnode->position) && 
             (ctnode_ptr->position - next_ctnode->position).norm_flat() > FARUtil::kMatchDist) 
         {
             // 定义当前轮廓节点和下一个轮廓节点之间的连线
             const PointPair line2(ctnode_ptr->position, next_ctnode->position);
             // 检查两条线的匹配百分比是否大于 0.99
             if (FARUtil::LineMatchPercentage(line1, line2) > 0.99f) {
                 // 检查从下一个轮廓节点到外部导航节点的连线是否畅通
                 if (IsCTMatchLineFreePolygon(next_ctnode, outNode_ptr, true)) {
                     // 若畅通，则记录匹配的轮廓节点并返回 true
                     matched_ctnode = next_ctnode;
                     return true;
                 }
             }
         }
         // 更新前一个节点
         prev_ctnode = next_ctnode;
         // 移动到下一个后向节点
         next_ctnode = next_ctnode->back;
     }
     // 若未找到匹配的节点，则返回 false
     return false;
 }
 
 /**
  * @brief 检查两个 CTNode 节点是否按照顺序相连
  * 
  * 该函数用于检查两个 CTNode 节点是否按照顺序相连，即从 ctnode1 开始，沿着其前向连接方向，
  * 是否能在不超出指定圆柱范围的情况下到达 ctnode2。如果在遍历过程中遇到超出范围的节点，
  * 则将该节点记录为阻挡顶点，并返回 false；如果能顺利到达 ctnode2，则返回 true。
  * 
  * @param ctnode1 起始的 CTNode 指针
  * @param ctnode2 目标的 CTNode 指针
  * @param block_vertex 若存在阻挡顶点，则存储该节点的指针；否则为 NULL
  * @return bool 如果两个节点按照顺序相连，则返回 true；否则返回 false
  */
 bool ContourGraph::IsCTNodesConnectWithinOrder(const CTNodePtr& ctnode1, const CTNodePtr& ctnode2, CTNodePtr& block_vertex) {
     // 初始化阻挡顶点为 NULL
     block_vertex = NULL;
     // 如果两个节点相同或者属于不同的多边形，则认为它们不相连，直接返回 false
     if (ctnode1 == ctnode2 || ctnode1->poly_ptr != ctnode2->poly_ptr) return false;
     // 从 ctnode1 的前一个节点开始进行前向搜索
     CTNodePtr next_ctnode = ctnode1->front; 
     // 循环遍历，直到找到目标节点或者遇到 NULL
     while (next_ctnode != NULL && next_ctnode != ctnode2) {
         // 检查当前节点是否在指定的圆柱范围内
         if (!FARUtil::IsInCylinder(ctnode1->position, ctnode2->position, next_ctnode->position, FARUtil::kNearDist, true)) {
             // 如果不在范围内，记录该节点为阻挡顶点
             block_vertex = next_ctnode;
             // 返回 false 表示两个节点不相连
             return false;
         }
         // 移动到下一个前向节点
         next_ctnode = next_ctnode->front;
     }
     // 如果能顺利到达目标节点，返回 true 表示两个节点相连
     return true;
 }
 
 /**
  * @brief 检查多边形的封闭性
  * 
  * 该函数遍历所有多边形的 CTNode，对于每个非柱子类型的多边形，找到第一个具有全局匹配标志的 CTNode，
  * 然后从前向连接方向遍历该多边形的 CTNode。在遍历过程中，检查相邻的具有全局匹配标志的 CTNode 是否按照顺序相连，
  * 如果不相连且存在阻挡顶点，并且该阻挡顶点与地面相关联且在边缘范围内，则将该阻挡顶点标记为轮廓必要节点。
  */
 void ContourGraph::EnclosePolygonsCheck() {
     // 遍历所有多边形的 CTNode
     for (const auto& ctnode_ptr : ContourGraph::polys_ctnodes_) { 
         // 如果当前 CTNode 所属的多边形是柱子类型，则跳过
         if (ctnode_ptr->poly_ptr->is_pillar) continue;
         // 找到当前多边形中第一个具有全局匹配标志的 CTNode
         const CTNodePtr start_ctnode_ptr = FirstMatchedCTNode(ctnode_ptr);
         // 如果未找到具有全局匹配标志的 CTNode，则跳过
         if (start_ctnode_ptr == NULL) continue;
         // 初始化前一个 CTNode 为起始 CTNode
         CTNodePtr pre_ctnode_ptr = start_ctnode_ptr;
         // 初始化当前 CTNode 为起始 CTNode 的前一个节点
         CTNodePtr cur_ctnode_ptr = start_ctnode_ptr->front;
         // 循环遍历，直到回到起始 CTNode
         while (cur_ctnode_ptr != start_ctnode_ptr) {
             // 如果当前 CTNode 没有全局匹配标志，则继续下一个节点
             if (!cur_ctnode_ptr->is_global_match) {
                 cur_ctnode_ptr = cur_ctnode_ptr->front;
                 continue;
             }
             // 用于存储可能的阻挡顶点
             CTNodePtr block_vertex = NULL;
             // 检查前一个 CTNode 和当前 CTNode 是否按照顺序相连
             if (!IsCTNodesConnectWithinOrder(pre_ctnode_ptr, cur_ctnode_ptr, block_vertex) && block_vertex != NULL) {
                 // 如果阻挡顶点与地面相关联且在边缘范围内
                 if (block_vertex->is_ground_associate && FARUtil::IsPointInMarginRange(block_vertex->position)) {
                     // 将该阻挡顶点标记为轮廓必要节点
                     block_vertex->is_contour_necessary = true;
                 }
             }
             // 更新前一个 CTNode 为当前 CTNode
             pre_ctnode_ptr = cur_ctnode_ptr;
             // 移动到下一个前向节点
             cur_ctnode_ptr = cur_ctnode_ptr->front;
         }
     }
 }
 
 /**
  * @brief 创建一个 CTNode 对象
  * 
  * 该函数用于创建一个 CTNode 对象，并对其成员变量进行初始化。
  * 
  * @param pos CTNode 的位置
  * @param ctnode_ptr 用于存储新创建的 CTNode 的智能指针
  * @param poly_ptr CTNode 所属的多边形的智能指针
  * @param is_pillar 指示该 CTNode 是否属于柱子类型的多边形
  */
 void ContourGraph::CreateCTNode(const Point3D& pos, CTNodePtr& ctnode_ptr, const PolygonPtr& poly_ptr, const bool& is_pillar) {
     // 创建一个新的 CTNode 对象，并将其智能指针赋值给 ctnode_ptr
     ctnode_ptr = std::make_shared<CTNode>();
     // 设置 CTNode 的位置
     ctnode_ptr->position = pos;
     // 初始化前向指针为 NULL
     ctnode_ptr->front = NULL;
     // 初始化后向指针为 NULL
     ctnode_ptr->back  = NULL;
     // 初始化全局匹配标志为 false
     ctnode_ptr->is_global_match = false;
     // 初始化轮廓必要标志为 false
     ctnode_ptr->is_contour_necessary = false;
     // 初始化地面关联标志为 false
     ctnode_ptr->is_ground_associate = false;
     // 初始化导航节点 ID 为 0
     ctnode_ptr->nav_node_id = 0;
     // 设置 CTNode 所属的多边形
     ctnode_ptr->poly_ptr = poly_ptr;
     // 根据是否为柱子类型的多边形，设置自由方向
     ctnode_ptr->free_direct = is_pillar ? NodeFreeDirect::PILLAR : NodeFreeDirect::UNKNOW;
     // 清空连接节点列表
     ctnode_ptr->connect_nodes.clear();
 }
 
 /**
  * @brief 创建一个多边形对象
  * 
  * 该函数用于创建一个多边形对象，并对其成员变量进行初始化。
  * 
  * @param poly_points 多边形的顶点集合
  * @param poly_ptr 用于存储新创建的多边形的智能指针
  */
 void ContourGraph::CreatePolygon(const PointStack& poly_points, PolygonPtr& poly_ptr) {
     // 创建一个新的多边形对象，并将其智能指针赋值给 poly_ptr
     poly_ptr = std::make_shared<Polygon>();
     // 设置多边形的顶点数量
     poly_ptr->N = poly_points.size();
     // 设置多边形的顶点集合
     poly_ptr->vertices = poly_points;
     // 检查机器人是否在多边形内部
     poly_ptr->is_robot_inside = FARUtil::PointInsideAPoly(poly_points, odom_node_ptr_->position);
     // 初始化多边形的周长为 0
     float perimeter = 0.0f;
     // 判断多边形是否为柱子类型的多边形，并计算周长
     poly_ptr->is_pillar = this->IsAPillarPolygon(poly_points, perimeter);
     // 设置多边形的周长
     poly_ptr->perimeter = perimeter;
 }
 
 /**
  * @brief 为 CTNode 查找最近的导航节点
  * 
  * 该函数用于在给定的临近导航节点集合中，为指定的 CTNode 查找最近的导航节点。
  * 它会考虑节点的类型、高度匹配、自由方向以及方向得分等因素，以确定最佳匹配的导航节点。
  * 
  * @param ctnode_ptr 要查找最近导航节点的 CTNode 指针
  * @param near_nodes 临近导航节点的指针栈
  * @return NavNodePtr 最近的导航节点指针，如果没有合适的匹配则返回 NULL
  */
 NavNodePtr ContourGraph::NearestNavNodeForCTNode(const CTNodePtr& ctnode_ptr, const NodePtrStack& near_nodes) {
     // 初始化最近距离为无穷大
     float nearest_dist = FARUtil::kINF;
     // 初始化最近的导航节点为 NULL
     NavNodePtr nearest_node = NULL;
     // 初始化最小欧式距离为无穷大
     float min_edist = FARUtil::kINF;
     // 方向阈值，用于计算方向得分
     const float dir_thred = 0.5f; //cos(pi/3);
     // 遍历所有临近节点
     for (const auto& node_ptr : near_nodes) {
         // 跳过无效节点：里程计节点、导航点节点、外部目标节点或高度不匹配的节点
         if (node_ptr->is_odom || node_ptr->is_navpoint || FARUtil::IsOutsideGoal(node_ptr) || !IsInMatchHeight(ctnode_ptr, node_ptr)) continue;
         // 跳过柱子和非柱子类型不匹配的节点
         if ((node_ptr->free_direct == NodeFreeDirect::PILLAR && ctnode_ptr->free_direct != NodeFreeDirect::PILLAR) ||
             (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::PILLAR)) 
         {
             continue;
         }
         // 初始化距离阈值
         float dist_thred = FARUtil::kMatchDist;
         // 初始化方向得分
         float dir_score = 0.0f;
         // 如果 CTNode 和导航节点都不是柱子类型，且导航节点不是未知类型
         if (ctnode_ptr->free_direct != NodeFreeDirect::PILLAR && node_ptr->free_direct != NodeFreeDirect::UNKNOW && node_ptr->free_direct != NodeFreeDirect::PILLAR) {
             // 如果 CTNode 和导航节点的自由方向相同
             if (ctnode_ptr->free_direct == node_ptr->free_direct) {
                 // 计算导航节点的拓扑方向
                 const Point3D topo_dir1 = FARUtil::SurfTopoDirect(node_ptr->surf_dirs);
                 // 计算 CTNode 的拓扑方向
                 const Point3D topo_dir2 = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs);
                 // 计算方向得分
                 dir_score = (topo_dir1 * topo_dir2 - dir_thred) / (1.0f - dir_thred);
             }
         } 
         // 如果 CTNode 和导航节点都是柱子类型
         else if (node_ptr->free_direct == NodeFreeDirect::PILLAR && ctnode_ptr->free_direct == NodeFreeDirect::PILLAR) {
             // 设置方向得分
             dir_score = 0.5f;
         }
         // 根据方向得分调整距离阈值
         dist_thred *= dir_score;
         // 计算导航节点和 CTNode 之间的欧式距离
         const float edist = (node_ptr->position - ctnode_ptr->position).norm_flat();
         // 如果欧式距离小于距离阈值且小于最小欧式距离
         if (edist < dist_thred && edist < min_edist) {
             // 更新最近的导航节点
             nearest_node = node_ptr;
             // 更新最小欧式距离
             min_edist = edist;
         }
     }
     // 如果找到了最近的导航节点，并且该节点已经有轮廓匹配
     if (nearest_node != NULL && nearest_node->is_contour_match) {
         // 计算该节点与之前匹配的 CTNode 之间的距离
         const float pre_dist = (nearest_node->position - nearest_node->ctnode->position).norm_flat();
         // 如果新的距离小于之前的距离
         if (min_edist < pre_dist) {
             // 移除之前的匹配
             RemoveMatchWithNavNode(nearest_node);
         } 
         // 如果新的距离不小于之前的距离
         else {
             // 不更新最近的导航节点
             nearest_node = NULL;
         }
     }
     // 返回最近的导航节点
     return nearest_node;
 }
 
 
 /**
  * @brief 分析轮廓图中每个 CTNode 的表面角度和凸性
  * 
  * 该函数遍历轮廓图中的每个 CTNode，根据其前后方向的连接情况计算表面方向，并分析其凸性。
  * 如果节点被判定为柱子类型，则将其表面方向设置为 {Point3D(0,0,-1), Point3D(0,0,-1)}。
  * 
  * @param contour_graph 轮廓图，由 CTNode 指针的栈表示
  */
 void ContourGraph::AnalysisSurfAngleAndConvexity(const CTNodeStack& contour_graph) {
     // 遍历轮廓图中的每个 CTNode
     for (const auto& ctnode_ptr : contour_graph) {
         // 如果 CTNode 是柱子类型或其所属多边形是柱子类型
         if (ctnode_ptr->free_direct == NodeFreeDirect::PILLAR || ctnode_ptr->poly_ptr->is_pillar) {
             // 将表面方向设置为 {Point3D(0,0,-1), Point3D(0,0,-1)}
             ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
             // 标记该多边形为柱子类型
             ctnode_ptr->poly_ptr->is_pillar = true;
             // 设置 CTNode 的自由方向为柱子类型
             ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
         } 
         // 非柱子类型的 CTNode
         else {
             // 定义下一个 CTNode 指针
             CTNodePtr next_ctnode;
             // 前向方向分析
             // 获取当前 CTNode 的前一个节点
             next_ctnode = ctnode_ptr->front;
             // 记录起始点为当前 CTNode 的位置
             Point3D start_p = ctnode_ptr->position;
             // 记录结束点为下一个 CTNode 的位置
             Point3D end_p = next_ctnode->position;
             // 计算结束点与当前 CTNode 位置的欧式距离
             float edist = (end_p - ctnode_ptr->position).norm_flat();
             // 循环遍历前向节点，直到遇到 NULL、回到当前节点或距离超过导航安全距离
             while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < FARUtil::kNavClearDist) {
                 // 移动到下一个前向节点
                 next_ctnode = next_ctnode->front;
                 // 更新起始点为上一个结束点
                 start_p = end_p;
                 // 更新结束点为当前节点的位置
                 end_p = next_ctnode->position;
                 // 重新计算结束点与当前 CTNode 位置的欧式距离
                 edist = (end_p - ctnode_ptr->position).norm_flat();
             }
             // 如果距离小于导航安全距离，认为该节点是柱子类型
             if (edist < FARUtil::kNavClearDist) { 
                 // 将表面方向设置为 {Point3D(0,0,-1), Point3D(0,0,-1)}
                 ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)};
                 // 标记该多边形为柱子类型
                 ctnode_ptr->poly_ptr->is_pillar = true;
                 // 设置 CTNode 的自由方向为柱子类型
                 ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                 // 跳过本次循环，继续处理下一个节点
                 continue;
             } 
             // 距离大于等于导航安全距离，计算前向表面方向
             else {
                 // 计算前向表面方向并存储在 surf_dirs 的第一个元素中
                 ctnode_ptr->surf_dirs.first = FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
             }
 
             // 后向方向分析
             // 获取当前 CTNode 的后一个节点
             next_ctnode = ctnode_ptr->back;
             // 记录起始点为当前 CTNode 的位置
             start_p = ctnode_ptr->position;
             // 记录结束点为下一个 CTNode 的位置
             end_p   = next_ctnode->position;
             // 计算结束点与当前 CTNode 位置的欧式距离
             edist = (end_p - ctnode_ptr->position).norm_flat();
             // 循环遍历后向节点，直到遇到 NULL、回到当前节点或距离超过导航安全距离
             while (next_ctnode != NULL && next_ctnode != ctnode_ptr && edist < FARUtil::kNavClearDist) {
                 // 移动到下一个后向节点
                 next_ctnode = next_ctnode->back;
                 // 更新起始点为上一个结束点
                 start_p = end_p;
                 // 更新结束点为当前节点的位置
                 end_p = next_ctnode->position;
                 // 重新计算结束点与当前 CTNode 位置的欧式距离
                 edist = (end_p - ctnode_ptr->position).norm_flat();
             }
             // 如果距离小于导航安全距离，认为该节点是柱子类型
             if (edist < FARUtil::kNavClearDist) { 
                 // 将表面方向设置为 {Point3D(0,0,-1), Point3D(0,0,-1)}
                 ctnode_ptr->surf_dirs = {Point3D(0,0,-1), Point3D(0,0,-1)}; 
                 // 标记该多边形为柱子类型
                 ctnode_ptr->poly_ptr->is_pillar = true;
                 // 设置 CTNode 的自由方向为柱子类型
                 ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
                 // 跳过本次循环，继续处理下一个节点
                 continue;
             } 
             // 距离大于等于导航安全距离，计算后向表面方向
             else {
                 // 计算后向表面方向并存储在 surf_dirs 的第二个元素中
                 ctnode_ptr->surf_dirs.second = FARUtil::ContourSurfDirs(end_p, start_p, ctnode_ptr->position, FARUtil::kNavClearDist);
             }
         }
         // 分析 CTNode 的凸性（除柱子类型外）
         this->AnalysisConvexityOfCTNode(ctnode_ptr);
     }
 }
 
 /**
  * @brief 判断给定顶点的多边形是否为柱子类型的多边形
  * 
  * 该函数会计算多边形的周长，并根据周长与预设的柱子周长阈值进行比较，
  * 以判断该多边形是否为柱子类型的多边形。
  * 
  * @param vertex_points 多边形的顶点集合
  * @param perimeter 输出参数，用于存储计算得到的多边形周长
  * @return bool 如果多边形是柱子类型，则返回 true；否则返回 false
  */
 bool ContourGraph::IsAPillarPolygon(const PointStack& vertex_points, float& perimeter) {
     // 初始化周长为 0
     perimeter = 0.0f;
     // 如果顶点数量少于 3，认为是柱子类型
     if (vertex_points.size() < 3) return true;
     // 初始化前一个点为第一个顶点
     Point3D prev_p(vertex_points[0]);
     // 遍历顶点集合，计算周长
     for (std::size_t i=1; i<vertex_points.size(); i++) {
         // 获取当前顶点
         const Point3D cur_p(vertex_points[i]);
         // 计算当前顶点与前一个顶点之间的距离
         const float dist = std::hypotf(cur_p.x - prev_p.x, cur_p.y - prev_p.y);
         // 累加距离到周长
         perimeter += dist;
         // 更新前一个点为当前点
         prev_p = cur_p;
     }
     // 如果周长大于柱子周长阈值，则不是柱子类型；否则是柱子类型
     return perimeter > ctgraph_params_.kPillarPerimeter ? false : true;
 }
 
 
 /**
  * @brief 检查两条线段是否相交
  * 
  * 该函数将传入的 PointPair 类型的 line 和 ConnectPair 类型的 edge 转换为 cv::Point2f 类型的点，
  * 然后调用 POLYOPS::doIntersect 函数来判断这两条线段是否相交。
  * 
  * @param line 第一条线段，由 PointPair 类型表示
  * @param edge 第二条线段，由 ConnectPair 类型表示
  * @return bool 如果两条线段相交，则返回 true；否则返回 false
  */
 bool ContourGraph::IsEdgeCollideSegment(const PointPair& line, const ConnectPair& edge) {
     // 将 line 的起点和终点转换为 cv::Point2f 类型的点
     const cv::Point2f start_p(line.first.x, line.first.y);
     const cv::Point2f end_p(line.second.x, line.second.y);
     // 调用 POLYOPS::doIntersect 函数来判断这两条线段是否相交
     if (POLYOPS::doIntersect(start_p, end_p, edge.start_p, edge.end_p)) {
         return true;
     }
     return false;
 }
 
 /**
  * @brief 检查给定的线段是否与多边形的任何一条边相交
  * 
  * 该函数会遍历多边形的每一条边，调用 `IsEdgeCollideSegment` 函数检查线段是否与多边形的某条边相交。
  * 如果相交，则返回 `true`；否则返回 `false`。
  * 
  * @param poly 多边形的顶点集合
  * @param edge 需要检查的线段，由 `ConnectPair` 类型表示
  * @return bool 如果线段与多边形的某条边相交，则返回 `true`；否则返回 `false`
  */
 bool ContourGraph::IsEdgeCollidePoly(const PointStack& poly, const ConnectPair& edge) {
     // 获取多边形的顶点数量
     const int N = poly.size();
     // 如果顶点数量少于 3，输出警告信息
     if (N < 3) std::cout << "Poly vertex size less than 3." << std::endl;
     // 遍历多边形的每一条边
     for (int i = 0; i < N; i++) {
         // 构建当前边的线段
         const PointPair line(poly[i], poly[FARUtil::Mod(i + 1, N)]);
         // 检查当前边是否与给定的线段相交
         if (ContourGraph::IsEdgeCollideSegment(line, edge)) {
             // 如果相交，返回 true
             return true;
         }
     }
     // 如果没有相交，返回 false
     return false;
 }
 
 /**
  * @brief 分析 CTNode 的凸性
  * 
  * 该函数用于分析给定 CTNode 的凸性，并根据分析结果设置其自由方向。
  * 如果 CTNode 的表面方向为特定值或其所属多边形为柱子类型，则将其标记为柱子类型。
  * 否则，根据拓扑方向和点的凸性判断其自由方向为凸点或凹点。
  * 
  * @param ctnode_ptr 要分析凸性的 CTNode 指针
  */
 void ContourGraph::AnalysisConvexityOfCTNode(const CTNodePtr& ctnode_ptr) {
     // 如果 CTNode 的前向表面方向为特定值、后向表面方向为特定值或其所属多边形为柱子类型
     if (ctnode_ptr->surf_dirs.first  == Point3D(0,0,-1) || ctnode_ptr->surf_dirs.second == Point3D(0,0,-1) || ctnode_ptr->poly_ptr->is_pillar) {
         // 将前向和后向表面方向都设置为特定值
         ctnode_ptr->surf_dirs.first = Point3D(0,0,-1), ctnode_ptr->surf_dirs.second == Point3D(0,0,-1);
         // 标记该多边形为柱子类型
         ctnode_ptr->poly_ptr->is_pillar = true;
         // 设置 CTNode 的自由方向为柱子类型
         ctnode_ptr->free_direct = NodeFreeDirect::PILLAR;
         // 直接返回，不再进行后续分析
         return;
     }
     // 标记是否为墙壁
     bool is_wall = false;
     // 计算 CTNode 的拓扑方向
     const Point3D topo_dir = FARUtil::SurfTopoDirect(ctnode_ptr->surf_dirs, is_wall);
     // 如果是墙壁
     if (is_wall) {
         // 设置 CTNode 的自由方向为未知
         ctnode_ptr->free_direct = NodeFreeDirect::UNKNOW;
         // 直接返回，不再进行后续分析
         return;
     }
     // 计算一个评估点，用于判断凸性
     const Point3D ev_p = ctnode_ptr->position + topo_dir * FARUtil::kLeafSize;
     // 如果评估点是凸点
     if (FARUtil::IsConvexPoint(ctnode_ptr->poly_ptr, ev_p)) {
         // 设置 CTNode 的自由方向为凸点
         ctnode_ptr->free_direct = NodeFreeDirect::CONVEX;
     } else {
         // 设置 CTNode 的自由方向为凹点
         ctnode_ptr->free_direct = NodeFreeDirect::CONCAVE;
     }
 }
 
 /**
  * @brief 将点投影到多边形外部
  * 
  * 该函数用于检查给定点是否在轮廓图中的某个非柱子多边形内部，如果是，则将该点重新投影到多边形外部。
  * 
  * @param point 要检查和重新投影的点
  * @param free_radius 投影时使用的自由半径
  * @return bool 如果点原本在多边形内部，则返回 true；否则返回 false
  */
 bool ContourGraph::ReprojectPointOutsidePolygons(Point3D& point, const float& free_radius) {
     // 初始化包含点的多边形指针为 NULL
     PolygonPtr inside_poly_ptr = NULL;
     // 标记点是否在多边形内部
     bool is_inside_poly = false;
     // 遍历所有轮廓多边形
     for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
         // 如果是柱子类型的多边形，跳过
         if (poly_ptr->is_pillar) continue;
         // 检查点是否在多边形内部，并且不在自由里程计点所在的多边形内部
         if (FARUtil::PointInsideAPoly(poly_ptr->vertices, point) && !FARUtil::PointInsideAPoly(poly_ptr->vertices, FARUtil::free_odom_p)) {
             // 记录包含点的多边形指针
             inside_poly_ptr = poly_ptr;
             // 标记点在多边形内部
             is_inside_poly = true;
             // 找到包含点的多边形后，跳出循环
             break;
         }
     }
     // 如果点在多边形内部
     if (is_inside_poly) {
         // 初始化最近距离为无穷大
         float near_dist = FARUtil::kINF;
         // 初始化重新投影的点为原始点
         Point3D reproject_p = point;
         // 初始化自由方向
         Point3D free_dir(0,0,-1);
         // 获取包含点的多边形的顶点数量
         const int N = inside_poly_ptr->vertices.size();
         // 遍历多边形的所有顶点
         for (int idx=0; idx<N; idx++) {
             // 获取当前顶点
             const Point3D vertex = inside_poly_ptr->vertices[idx];
             // 计算当前顶点到点的距离
             const float temp_dist = (vertex - point).norm_flat();
             // 如果当前距离小于最近距离
             if (temp_dist < near_dist) {
                 // 计算前一个顶点到当前顶点的归一化方向
                 const Point3D dir1 = (inside_poly_ptr->vertices[FARUtil::Mod(idx-1, N)] - vertex).normalize_flat();
                 // 计算后一个顶点到当前顶点的归一化方向
                 const Point3D dir2 = (inside_poly_ptr->vertices[FARUtil::Mod(idx+1, N)] - vertex).normalize_flat();
                 // 计算两个方向的和并归一化
                 const Point3D dir = (dir1 + dir2).normalize_flat();
                 // 检查顶点沿着该方向移动一个小距离后是否仍在多边形内部
                 if (FARUtil::PointInsideAPoly(inside_poly_ptr->vertices, vertex + dir * FARUtil::kLeafSize)) { // 凸点
                     // 更新重新投影的点为当前顶点
                     reproject_p = vertex;
                     // 更新最近距离
                     near_dist = temp_dist;
                     // 更新自由方向
                     free_dir = dir;
                 }
             }
         }
         // 记录原始点的 z 坐标
         const float origin_z = point.z;
         // 将点重新投影到多边形外部，距离为自由半径
         point = reproject_p - free_dir * free_radius;
         // 恢复点的原始 z 坐标
         point.z = origin_z;
     }
     // 返回点是否原本在多边形内部的标记
     return is_inside_poly;
 }
 
 /**
  * @brief 将导航节点对添加到轮廓集合中
  * 
  * 该函数用于将两个导航节点对添加到全局轮廓集合和边界轮廓集合中。
  * 确保节点对的顺序是按照节点 ID 从小到大排列的，以避免重复添加。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  */
 void ContourGraph::AddContourToSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
     // 创建一个 NavEdge 对象，用于表示两个导航节点之间的边
     NavEdge edge(node_ptr1, node_ptr2);
     // 强制将节点对的顺序调整为 id1 < id2，以确保集合中的元素唯一
     if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);
 
     // 将边添加到全局轮廓集合中
     ContourGraph::global_contour_set_.insert(edge);
     // 如果两个节点都是边界节点，则将边添加到边界轮廓集合中
     if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
         ContourGraph::boundary_contour_set_.insert(edge);
     }
 }
 
 /**
  * @brief 从轮廓集合中删除指定的导航节点对
  * 
  * 该函数用于从全局轮廓集合和边界轮廓集合中删除由两个导航节点组成的边。
  * 确保节点对的顺序是按照节点 ID 从小到大排列的，以正确匹配集合中的元素。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  */
 void ContourGraph::DeleteContourFromSets(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2) {
     // 创建一个 NavEdge 对象，用于表示两个导航节点之间的边
     NavEdge edge(node_ptr1, node_ptr2);
     // 强制将节点对的顺序调整为 id1 < id2，以确保能正确匹配集合中的元素
     if (node_ptr1->id > node_ptr2->id) edge = NavEdge(node_ptr2, node_ptr1);
 
     // 从全局轮廓集合中删除该边
     ContourGraph::global_contour_set_.erase(edge);
     // 如果两个节点都是边界节点，则从边界轮廓集合中删除该边
     if (node_ptr1->is_boundary && node_ptr2->is_boundary) {
         ContourGraph::boundary_contour_set_.erase(edge);
     }
 }
 
 /**
  * @brief 提取全局轮廓信息
  * 
  * 该函数用于从全局轮廓集合和边界轮廓集合中提取轮廓信息，并将其分类存储到不同的轮廓容器中。
  * 同时，会检查边是否在局部范围内，以及边是否有效，并对无效边界进行标记。
  */
 void ContourGraph::ExtractGlobalContours() {
     // 清空全局轮廓容器
     ContourGraph::global_contour_.clear();
     // 清空非活动轮廓容器
     ContourGraph::inactive_contour_.clear();
     // 清空未匹配轮廓容器
     ContourGraph::unmatched_contour_.clear();
     // 清空边界轮廓容器
     ContourGraph::boundary_contour_.clear();
     // 清空局部边界容器
     ContourGraph::local_boundary_.clear();
     // 遍历全局轮廓集合
     for (const auto& edge : ContourGraph::global_contour_set_) {
         // 将边的两个节点位置添加到全局轮廓容器中
         ContourGraph::global_contour_.push_back({edge.first->position, edge.second->position});
         // 检查边是否在局部范围内
         if (IsEdgeInLocalRange(edge.first, edge.second)) {
             // 检查边是否为非活动边
             if (!this->IsActiveEdge(edge.first, edge.second)) {
                 // 将非活动边的两个节点位置添加到非活动轮廓容器中
                 ContourGraph::inactive_contour_.push_back({edge.first->position, edge.second->position});
             } 
             // 检查边的两个节点是否有临近节点
             else if (!edge.first->is_near_nodes || !edge.second->is_near_nodes) {
                 // 创建一个未匹配的点对
                 PointPair unmatched_pair = std::make_pair(edge.first->position, edge.second->position);
                 // 如果第一个节点有轮廓匹配
                 if (edge.first->is_contour_match) {
                     // 将未匹配点对的第一个点更新为第一个节点匹配的 CTNode 的位置
                     unmatched_pair.first = edge.first->ctnode->position;
                 } 
                 // 如果第二个节点有轮廓匹配
                 else if (edge.second->is_contour_match) {
                     // 将未匹配点对的第二个点更新为第二个节点匹配的 CTNode 的位置
                     unmatched_pair.second = edge.second->ctnode->position;
                 } 
                 // 将未匹配点对添加到未匹配轮廓容器中
                 ContourGraph::unmatched_contour_.push_back(unmatched_pair);
             }
         }
     }
     // 遍历边界轮廓集合
     for (const auto& edge : ContourGraph::boundary_contour_set_) {
         // 将边的两个节点位置添加到边界轮廓容器中
         ContourGraph::boundary_contour_.push_back({edge.first->position, edge.second->position});
         // 检查边是否在局部范围内
         if (IsEdgeInLocalRange(edge.first, edge.second)) {
             // 将边的两个节点位置添加到局部边界容器中
             ContourGraph::local_boundary_.push_back({edge.first->position, edge.second->position});
             // 标记是否为新的无效边界
             bool is_new_invalid = false;
             // 检查边是否为有效边界
             if (!IsValidBoundary(edge.first, edge.second, is_new_invalid) && is_new_invalid) {
                 // 将第二个节点的 ID 添加到第一个节点的无效边界集合中
                 edge.first->invalid_boundary.insert(edge.second->id);
                 // 将第一个节点的 ID 添加到第二个节点的无效边界集合中
                 edge.second->invalid_boundary.insert(edge.first->id);
             }
         }
     }
 }
 
 /**
  * @brief 检查两个导航节点之间的边界是否有效
  * 
  * 该函数用于检查由两个导航节点组成的边界是否有效。它首先检查该边界是否已经被标记为无效，
  * 然后检查该边界是否与局部多边形发生碰撞。如果边界已经无效或与多边形碰撞，则边界无效。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param is_new 引用参数，用于标记该无效边界是否为新发现的
  * @return bool 如果边界有效，则返回 true；否则返回 false
  */
 bool ContourGraph::IsValidBoundary(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, bool& is_new) {
     // 初始标记为新的无效边界
     is_new = true;
     // 检查该边界是否已经被标记为无效
     if (node_ptr1->invalid_boundary.find(node_ptr2->id) != node_ptr1->invalid_boundary.end()) { 
         // 如果已经无效，标记为非新的无效边界
         is_new = false;
         // 返回无效
         return false;
     }
     // 创建一个 ConnectPair 对象，表示两个导航节点之间的边
     const ConnectPair cedge = ConnectPair(node_ptr1->position, node_ptr2->position);
     // 遍历所有轮廓多边形
     for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
         // 如果是柱子类型的多边形，跳过
         if (poly_ptr->is_pillar) continue;
         // 检查边是否与多边形的任何一条边相交
         if (ContourGraph::IsEdgeCollidePoly(poly_ptr->vertices, cedge)) {
             // 如果相交，返回无效
             return false;
         }
     }
     // 如果没有相交，返回有效
     return true;
 }
 
 /**
  * @brief 更新里程计的自由位置
  * 
  * 该函数根据当前里程计节点的位置和轮廓多边形信息，尝试找到一个自由位置。
  * 如果当前位置在非柱子类型的多边形内部，则在其周围采样点，
  * 检查采样点是否在多边形外部，若找到则更新全局自由位置。
  * 
  * @param odom_ptr 里程计节点的指针
  * @param global_free_p 引用参数，用于存储全局自由位置
  */
 void ContourGraph::UpdateOdomFreePosition(const NavNodePtr& odom_ptr, Point3D& global_free_p) {
     // 初始化自由位置为里程计节点的位置
     Point3D free_p = odom_ptr->position;
     // 标记当前位置是否为自由位置
     bool is_free_p = true;
     // 存储自由采样点的容器
     PointStack free_sample_points;
     // 遍历所有轮廓多边形
     for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
         // 如果多边形不是柱子类型且机器人在多边形内部
         if (!poly_ptr->is_pillar && poly_ptr->is_robot_inside) {
             // 标记当前位置不是自由位置
             is_free_p = false;
             // 在当前位置周围创建采样点
             FARUtil::CreatePointsAroundCenter(free_p, FARUtil::kNavClearDist, FARUtil::kLeafSize, free_sample_points);
             // 找到符合条件的多边形后，跳出循环
             break;
         }
     }
     // 如果当前位置是自由位置，标记机器人不在多边形内部
     if (is_free_p) is_robot_inside_poly_ = false;
     // 将全局自由位置初始化为当前位置
     global_free_p = free_p;
     // 如果当前位置不是自由位置且机器人之前不在多边形内部
     if (!is_free_p && !is_robot_inside_poly_) {
         // 标记是否找到自由位置
         bool is_free_pos_found = false;
         // 遍历所有采样点
         for (const auto& p : free_sample_points) {
             // 标记当前采样点是否为自由位置
             bool is_sample_free = true;
             // 遍历所有轮廓多边形
             for (const auto& poly_ptr : ContourGraph::contour_polygons_) {
                 // 如果多边形不是柱子类型且采样点在多边形内部
                 if (!poly_ptr->is_pillar && FARUtil::PointInsideAPoly(poly_ptr->vertices, p)) {
                     // 标记当前采样点不是自由位置
                     is_sample_free = false;
                     // 找到采样点在多边形内部后，跳出循环
                     break;
                 }
             }
             // 如果当前采样点是自由位置
             if (is_sample_free) {
                 // 更新全局自由位置为当前采样点
                 global_free_p = p;
                 // 标记已找到自由位置
                 is_free_pos_found = true;
                 // 找到自由位置后，跳出循环
                 break;
             }
         }
         // 如果没有找到自由位置，标记机器人在多边形内部
         if (!is_free_pos_found) is_robot_inside_poly_ = true;
     }
 }
 
 /**
  * @brief 重新投影由 CTNode 和 NavNode 组成的边
  * 
  * 该函数用于重新投影由一个 CTNode 和一个 NavNode 组成的边。
  * 它会计算两个节点之间的距离，并根据这个距离计算参考距离。
  * 然后，使用参考距离对两个节点进行投影，并将投影后的点作为新边的起点和终点。
  * 
  * @param ctnode_ptr1 第一个节点，类型为 CTNodePtr
  * @param node_ptr2 第二个节点，类型为 NavNodePtr
  * @param dist 最大允许的投影距离
  * @return ConnectPair 重新投影后的边，由两个投影点组成
  */
 ConnectPair ContourGraph::ReprojectEdge(const CTNodePtr& ctnode_ptr1, const NavNodePtr& node_ptr2, const float& dist) {
     // 创建一个 ConnectPair 对象，用于存储重新投影后的边
     ConnectPair edgeOut;
     // 计算两个节点之间的欧几里得距离
     const float ndist = (ctnode_ptr1->position - node_ptr2->position).norm_flat();
     // 计算参考距离，取 ndist 的 0.4 倍和 dist 中的较小值
     const float ref_dist = std::min(ndist*0.4f, dist);
 
     // 对第一个节点进行投影，并将投影后的点作为新边的起点
     edgeOut.start_p = ProjectNode(ctnode_ptr1, ref_dist); // node 1
     // 对第二个节点进行投影，并将投影后的点作为新边的终点
     edgeOut.end_p   = ProjectNode(node_ptr2, ref_dist);   // node 2
 
     // 返回重新投影后的边
     return edgeOut;
 }
 
 /**
  * @brief 重新投影由两个 NavNode 组成的边
  * 
  * 该函数用于重新投影由两个 NavNode 组成的边。它会计算两个节点之间的距离，
  * 并根据这个距离计算参考距离。然后根据是否进行全局检查、节点是否有轮廓匹配
  * 以及节点和其匹配的 CTNode 的自由方向是否一致，决定是对节点本身还是对其匹配的 CTNode 进行投影。
  * 
  * @param node_ptr1 第一个导航节点的指针
  * @param node_ptr2 第二个导航节点的指针
  * @param dist 最大允许的投影距离
  * @param is_global_check 是否进行全局检查的标志
  * @return ConnectPair 重新投影后的边，由两个投影点组成
  */
 ConnectPair ContourGraph::ReprojectEdge(const NavNodePtr& node_ptr1, const NavNodePtr& node_ptr2, const float& dist, const bool& is_global_check) {
     // 创建一个 ConnectPair 对象，用于存储重新投影后的边
     ConnectPair edgeOut;
     // 计算两个节点之间的欧几里得距离
     const float ndist = (node_ptr1->position - node_ptr2->position).norm_flat();
     // 计算参考距离，取 ndist 的 0.4 倍和 dist 中的较小值
     const float ref_dist = std::min(ndist*0.4f, dist);
     // 处理第一个节点
     // 如果不进行全局检查，且第一个节点有轮廓匹配，且第一个节点匹配的 CTNode 的自由方向和该节点的自由方向一致
     if (!is_global_check && node_ptr1->is_contour_match && node_ptr1->ctnode->free_direct == node_ptr1->free_direct) { 
         // 获取第一个节点匹配的 CTNode
         const auto ctnode1 = node_ptr1->ctnode;
         // 对第一个节点匹配的 CTNode 进行投影，并将投影后的点作为新边的起点
         edgeOut.start_p = ProjectNode(ctnode1, ref_dist); 
     } else {
         // 对第一个节点进行投影，并将投影后的点作为新边的起点
         edgeOut.start_p = ProjectNode(node_ptr1, ref_dist); 
     }
     // 处理第二个节点
     // 如果不进行全局检查，且第二个节点有轮廓匹配，且第二个节点匹配的 CTNode 的自由方向和该节点的自由方向一致
     if (!is_global_check && node_ptr2->is_contour_match && node_ptr2->ctnode->free_direct == node_ptr2->free_direct) { 
         // 获取第二个节点匹配的 CTNode
         const auto ctnode2 = node_ptr2->ctnode;
         // 对第二个节点匹配的 CTNode 进行投影，并将投影后的点作为新边的终点
         edgeOut.end_p = ProjectNode(ctnode2, ref_dist); 
     } else {
         // 对第二个节点进行投影，并将投影后的点作为新边的终点
         edgeOut.end_p = ProjectNode(node_ptr2, ref_dist);
     }
     // 返回重新投影后的边
     return edgeOut;
 }
 
 /**
  * @brief 重置当前轮廓图的相关信息
  * 
  * 该函数用于重置当前轮廓图的状态，包括清除轮廓图、清空轮廓集合、
  * 重置里程计节点指针和机器人是否在多边形内部的标志。
  */
 void ContourGraph::ResetCurrentContour() {
     // 调用 ClearContourGraph 函数清除当前的轮廓图
     this->ClearContourGraph();
     // 清空全局轮廓集合
     ContourGraph::global_contour_set_.clear();
     // 清空边界轮廓集合
     ContourGraph::boundary_contour_set_.clear();
 
     // 将里程计节点指针置为 NULL
     odom_node_ptr_ = NULL;
     // 将机器人是否在多边形内部的标志置为 false
     is_robot_inside_poly_ = false;
 }   
 
 
 
 