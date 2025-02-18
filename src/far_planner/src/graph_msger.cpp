/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



 #include "far_planner/graph_msger.h"

 /***************************************************************************************/
 
 /**
  * @brief 初始化 GraphMsger 类的成员变量和 ROS 相关的发布者、订阅者
  * 
  * 该函数用于初始化 GraphMsger 类的成员变量，包括 ROS 节点句柄、参数、消息发布者和订阅者，
  * 同时清空全局图，初始化点云指针和 KD 树，并设置 KD 树的排序结果为不排序。
  * 
  * @param nh ROS 节点句柄，用于与 ROS 系统进行通信
  * @param params GraphMsger 类的参数结构体，包含配置信息
  */
 void GraphMsger::Init(const ros::NodeHandle& nh, const GraphMsgerParams& params) {
     // 将传入的 ROS 节点句柄赋值给成员变量 nh_
     nh_ = nh;
     // 将传入的参数结构体赋值给成员变量 gm_params_
     gm_params_ = params;
     // 使用节点句柄 nh_ 创建一个消息发布者 graph_pub_，用于发布类型为 visibility_graph_msg::Graph 的消息
     // 消息主题为 "/robot_vgraph"，队列长度为 5
     graph_pub_ = nh_.advertise<visibility_graph_msg::Graph>("/robot_vgraph", 5);
     // 使用节点句柄 nh_ 创建一个消息订阅者 graph_sub_，用于订阅类型为 visibility_graph_msg::Graph 的消息
     // 消息主题为 "/decoded_vgraph"，队列长度为 5，回调函数为 GraphMsger::GraphCallBack，回调对象为当前对象 this
     graph_sub_ = nh_.subscribe("/decoded_vgraph", 5, &GraphMsger::GraphCallBack, this);
 
     // 清空全局图 global_graph_
     global_graph_.clear();
     // 初始化点云指针 nodes_cloud_ptr_，使用 new 关键字创建一个 pcl::PointCloud<PCLPoint> 对象
     nodes_cloud_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
     // 初始化 KD 树指针 kdtree_graph_cloud_，使用 new 关键字创建一个 pcl::KdTreeFLANN<PCLPoint> 对象
     kdtree_graph_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
     // 设置 KD 树的排序结果为不排序
     kdtree_graph_cloud_->setSortedResults(false);
 }
 
 /**
  * @brief 将图结构编码为消息格式
  * 
  * 该函数将输入的图结构编码为 visibility_graph_msg::Graph 消息类型，以便通过 ROS 进行发布。
  * 只对满足编码条件的节点及其连接进行编码。
  * 
  * @param graphIn 输入的图结构，由节点指针栈表示
  * @param graphOut 输出的图消息，用于存储编码后的图信息
  */
 void GraphMsger::EncodeGraph(const NodePtrStack& graphIn, visibility_graph_msg::Graph& graphOut) {
     // 清空输出图消息的节点列表
     graphOut.nodes.clear();
     // 获取输出图消息的坐标系 ID
     const std::string frame_id = graphOut.header.frame_id;
     // 遍历输入图中的每个节点
     for (const auto& node_ptr : graphIn) {
         // 如果节点的连接节点列表为空或者节点不满足编码类型，则跳过该节点
         if (node_ptr->connect_nodes.empty() || !IsEncodeType(node_ptr)) continue;
         // 创建一个新的节点消息对象
         visibility_graph_msg::Node msg_node;
         // 设置节点消息的坐标系 ID
         msg_node.header.frame_id = frame_id;
         // 将节点的位置转换为几何消息点并赋值给节点消息的位置
         msg_node.position    = FARUtil::Point3DToGeoMsgPoint(node_ptr->position);
         // 设置节点消息的 ID
         msg_node.id          = node_ptr->id;
         // 将节点的自由方向类型转换为整数并赋值给节点消息的 FreeType
         msg_node.FreeType    = static_cast<int>(node_ptr->free_direct);
         // 设置节点消息的覆盖状态
         msg_node.is_covered  = node_ptr->is_covered;
         // 设置节点消息的前沿状态
         msg_node.is_frontier = node_ptr->is_frontier;
         // 设置节点消息的导航点状态
         msg_node.is_navpoint = node_ptr->is_navpoint;
         // 设置节点消息的边界状态
         msg_node.is_boundary = node_ptr->is_boundary;
         // 清空节点消息的表面方向列表
         msg_node.surface_dirs.clear();
         // 将节点的第一个表面方向转换为几何消息点并添加到节点消息的表面方向列表中
         msg_node.surface_dirs.push_back(FARUtil::Point3DToGeoMsgPoint(node_ptr->surf_dirs.first));
         // 将节点的第二个表面方向转换为几何消息点并添加到节点消息的表面方向列表中
         msg_node.surface_dirs.push_back(FARUtil::Point3DToGeoMsgPoint(node_ptr->surf_dirs.second));
 
         // 编码节点的连接信息
         // 清空节点消息的连接节点列表
         msg_node.connect_nodes.clear();
         // 遍历节点的连接节点列表
         for (const auto& cnode_ptr : node_ptr->connect_nodes) {
             // 如果连接节点不满足编码类型，则跳过该连接节点
             if (!IsEncodeType(cnode_ptr)) continue;
             // 将连接节点的 ID 添加到节点消息的连接节点列表中
             msg_node.connect_nodes.push_back(cnode_ptr->id);
         }
         // 清空节点消息的多边形连接列表
         msg_node.poly_connects.clear();
         // 遍历节点的多边形连接节点列表
         for (const auto& cnode_ptr : node_ptr->poly_connects) {
             // 如果多边形连接节点不满足编码类型，则跳过该节点
             if (!IsEncodeType(cnode_ptr)) continue;
             // 将多边形连接节点的 ID 添加到节点消息的多边形连接列表中
             msg_node.poly_connects.push_back(cnode_ptr->id);
         }
         // 清空节点消息的轮廓连接列表
         msg_node.contour_connects.clear();
         // 遍历节点的轮廓连接节点列表
         for (const auto& cnode_ptr : node_ptr->contour_connects) {
             // 如果轮廓连接节点不满足编码类型，则跳过该节点
             if (!IsEncodeType(cnode_ptr)) continue;
             // 将轮廓连接节点的 ID 添加到节点消息的轮廓连接列表中
             msg_node.contour_connects.push_back(cnode_ptr->id);
         }
         // 清空节点消息的轨迹连接列表
         msg_node.trajectory_connects.clear();
         // 遍历节点的轨迹连接节点列表
         for (const auto& cnode_ptr : node_ptr->trajectory_connects) {
             // 如果轨迹连接节点不满足编码类型，则跳过该节点
             if (!IsEncodeType(cnode_ptr)) continue;
             // 将轨迹连接节点的 ID 添加到节点消息的轨迹连接列表中
             msg_node.trajectory_connects.push_back(cnode_ptr->id);
         }
         // 将编码后的节点消息添加到输出图消息的节点列表中
         graphOut.nodes.push_back(msg_node);
     }
     // 设置输出图消息的节点数量
     graphOut.size = graphOut.nodes.size();
 }
 
 /**
  * @brief 更新全局图信息
  * 
  * 该函数用于更新全局图信息，包括更新图的节点信息和 KD 树，
  * 并在更新完成后发布全局图消息。
  * 
  * @param graph 输入的图结构，由节点指针栈表示
  */
 void GraphMsger::UpdateGlobalGraph(const NodePtrStack& graph) {
     // 将输入的图结构赋值给全局图
     global_graph_ = graph;
     // Update Graph Node KdTree
     // 如果全局图为空
     if (global_graph_.empty()) {
         // 清空点云指针和 KD 树
         FARUtil::ClearKdTree(nodes_cloud_ptr_, kdtree_graph_cloud_);
     } else {
         // 调整点云指针的大小以适应全局图的节点数量
         nodes_cloud_ptr_->resize(global_graph_.size());
         // 初始化索引为 0
         std::size_t idx = 0;
         // 遍历全局图中的每个节点
         for (const auto node_ptr : global_graph_) {
             // 如果节点是里程计节点或者在目标范围外，则跳过该节点
             if (node_ptr->is_odom || FARUtil::IsOutsideGoal(node_ptr)) continue;
             // 将节点的位置转换为 PCL 点
             PCLPoint pcl_p = FARUtil::Point3DToPCLPoint(node_ptr->position);
             // 设置 PCL 点的强度为节点的 ID
             pcl_p.intensity = node_ptr->id;
             // 将 PCL 点存储到点云指针的相应位置
             nodes_cloud_ptr_->points[idx] = pcl_p;
             // 索引加 1
             idx ++;
         }
         // 设置 KD 树的输入点云为更新后的点云指针
         kdtree_graph_cloud_->setInputCloud(nodes_cloud_ptr_);
     }
     // 发布更新后的全局图消息
     this->PublishGlobalGraph(global_graph_);
 }
 
 /**
  * @brief 发布全局图消息
  * 
  * 该函数将输入的图结构编码为 ROS 消息格式，并通过 ROS 发布者发布出去。
  * 
  * @param graphIn 输入的图结构，由节点指针栈表示
  */
 void GraphMsger::PublishGlobalGraph(const NodePtrStack& graphIn) {
     // 创建一个 visibility_graph_msg::Graph 类型的消息对象，用于存储要发布的图信息
     visibility_graph_msg::Graph graph_msg;
     // 设置消息的坐标系 ID，从类的成员变量 gm_params_ 中获取
     graph_msg.header.frame_id = gm_params_.frame_id;
     // 设置消息的机器人 ID，从类的成员变量 gm_params_ 中获取
     graph_msg.robot_id = gm_params_.robot_id;
     // 调用 EncodeGraph 函数将输入的图结构编码到 graph_msg 中
     this->EncodeGraph(graphIn, graph_msg);
     // 使用类的成员变量 graph_pub_ 发布编码后的图消息
     graph_pub_.publish(graph_msg);
 }
 
 /**
  * @brief 处理接收到的图消息的回调函数
  * 
  * 该函数用于处理接收到的 visibility_graph_msg::Graph 类型的消息。
  * 它会将消息中的节点解码为导航节点，并将其添加到全局图中。
  * 同时，它会为这些节点分配连接信息，包括图连接、多边形连接、轮廓连接和轨迹连接。
  * 
  * @param msg 接收到的图消息的常量指针
  */
 void GraphMsger::GraphCallBack(const visibility_graph_msg::GraphConstPtr& msg) {
     // 如果接收到的消息中节点列表为空，则直接返回
     if (msg->nodes.empty()) return;
     // 用于存储解码后的导航节点指针栈
     NodePtrStack decoded_nodes;
     // 获取消息中的机器人 ID
     const std::size_t robot_id = msg->robot_id;
     // 复制消息内容到 graph_msg
     const visibility_graph_msg::Graph graph_msg = *msg;
     // 用于存储节点 ID 到索引的映射
     IdxMap nodeIdx_idx_map;
     // 清空解码节点栈
     decoded_nodes.clear();
     // 遍历消息中的每个节点
     for (std::size_t i = 0; i < graph_msg.nodes.size(); i++) {
         // 获取当前节点
         const auto node = graph_msg.nodes[i];
         // 将节点的位置信息转换为 Point3D 类型
         const Point3D node_p = Point3D(node.position.x, node.position.y, node.position.z);
         // 在全局图中查找距离当前节点最近的导航节点
         NavNodePtr nearest_node_ptr = NearestNodePtrOnGraph(node_p, gm_params_.dist_margin);
         // 如果未找到最近节点，或者最近节点与当前节点的自由类型不匹配
         if (nearest_node_ptr == NULL || IsMismatchFreeNode(nearest_node_ptr, node)) {
             // 创建一个新的解码导航节点
             CreateDecodedNavNode(node, nearest_node_ptr);
             // 将新节点添加到全局图中
             DynamicGraph::AddNodeToGraph(nearest_node_ptr);
         }
         // 将处理后的节点添加到解码节点栈中
         decoded_nodes.push_back(nearest_node_ptr);
         // 将节点 ID 及其索引插入到映射中
         nodeIdx_idx_map.insert({node.id, i});
     }
     // 用于存储不同类型连接的节点 ID 列表
     std::vector<std::size_t> connect_idxs, poly_idxs, contour_idxs, traj_idxs;
     // 再次遍历消息中的每个节点
     for (std::size_t i = 0; i < graph_msg.nodes.size(); i++) {
         // 获取当前节点
         const auto node = graph_msg.nodes[i];
         // 获取解码后的节点指针
         const NavNodePtr node_ptr = decoded_nodes[i];
         // 从当前节点中提取不同类型的连接节点 ID
         ExtractConnectIdxs(node, connect_idxs, poly_idxs, contour_idxs, traj_idxs);
         // 处理图连接
         NavNodePtr cnode_ptr = NULL;
         // 遍历图连接的节点 ID 列表
         for (const auto& cid : connect_idxs) {
             // 根据节点 ID 和映射关系找到对应的节点指针
             cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
             // 如果找到对应节点，并且当前节点或连接节点未激活，或者两者都是边界节点
             if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                 // 在两个节点之间添加边连接
                 DynamicGraph::AddEdge(node_ptr, cnode_ptr);
             }
         }
         // 处理多边形连接
         for (const auto& cid : poly_idxs) {
             // 根据节点 ID 和映射关系找到对应的节点指针
             cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
             // 如果找到对应节点，并且当前节点或连接节点未激活，或者两者都是边界节点
             if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                 // 填充多边形边连接信息
                 DynamicGraph::FillPolygonEdgeConnect(node_ptr, cnode_ptr, gm_params_.votes_size);
             }
         }
         // 处理轮廓连接
         for (const auto& cid : contour_idxs) {
             // 根据节点 ID 和映射关系找到对应的节点指针
             cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
             // 如果找到对应节点，并且当前节点或连接节点未激活，或者两者都是边界节点
             if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                 // 填充轮廓连接信息
                 DynamicGraph::FillContourConnect(node_ptr, cnode_ptr, gm_params_.votes_size);
             }
         }
         // 处理轨迹连接
         for (const auto& cid : traj_idxs) {
             // 根据节点 ID 和映射关系找到对应的节点指针
             cnode_ptr = IdToNodePtr(cid, nodeIdx_idx_map, decoded_nodes);
             // 如果找到对应节点，并且当前节点或连接节点未激活，或者两者都是边界节点
             if (cnode_ptr != NULL && (!node_ptr->is_active || !cnode_ptr->is_active || (node_ptr->is_boundary && cnode_ptr->is_boundary))) {
                 // 填充轨迹连接信息
                 DynamicGraph::FillTrajConnect(node_ptr, cnode_ptr);
             }
         }
     }
 }
 
 /**
  * @brief 在全局图中查找距离给定点最近的导航节点
  * 
  * 该函数通过 KD 树在全局图中查找距离输入点最近的节点，并检查该节点是否在指定半径范围内。
  * 如果找到符合条件的节点，则返回该节点的指针；否则返回 NULL。
  * 
  * @param p 输入的点，类型为 Point3D
  * @param radius 查找半径，用于判断最近节点是否有效
  * @return NavNodePtr 距离输入点最近的导航节点指针，如果未找到则返回 NULL
  */
 NavNodePtr GraphMsger::NearestNodePtrOnGraph(const Point3D p, const float radius) {
     // 如果全局图为空，直接返回 NULL
     if (global_graph_.empty()) return NULL;
     // Find the nearest node in graph
     // 用于存储最近邻节点的索引
     std::vector<int> pIdxK(1);
     // 用于存储最近邻节点的距离
     std::vector<float> pdDistK(1);
     // 将输入的 Point3D 类型的点转换为 PCL 点
     PCLPoint pcl_p = FARUtil::Point3DToPCLPoint(p);
     // 使用 KD 树查找距离输入点最近的一个节点
     if (kdtree_graph_cloud_->nearestKSearch(pcl_p, 1, pIdxK, pdDistK) > 0) {
         // 如果找到的最近节点的距离小于指定半径
         if (pdDistK[0] < radius) {
             // 获取全局图中最近节点的 PCL 点
             const PCLPoint graph_p = nodes_cloud_ptr_->points[pIdxK[0]];
             // 从 PCL 点的强度信息中提取节点 ID
             const std::size_t node_id = static_cast<std::size_t>(graph_p.intensity);
             // 根据节点 ID 从动态图中获取对应的导航节点指针
             return DynamicGraph::MappedNavNodeFromId(node_id);
         }
     }
     // 如果未找到符合条件的节点，返回 NULL
     return NULL;
 }
 
 /**
  * @brief 根据接收到的消息节点创建解码后的导航节点
  * 
  * 该函数根据接收到的 visibility_graph_msg::Node 类型的消息节点信息，创建并初始化一个导航节点。
  * 它会从消息节点中提取位置、覆盖状态、前沿状态、导航点状态和边界状态等信息，
  * 并使用这些信息创建一个新的导航节点，同时对节点的其他属性进行初始化。
  * 
  * @param vnode 接收到的消息节点，包含节点的各种属性信息
  * @param node_ptr 引用参数，用于存储创建的导航节点指针
  */
 void GraphMsger::CreateDecodedNavNode(const visibility_graph_msg::Node& vnode, NavNodePtr& node_ptr) {
     // 从消息节点中提取位置信息，创建一个 Point3D 类型的点
     const Point3D p = Point3D(vnode.position.x, vnode.position.y, vnode.position.z);
     // 根据消息节点的覆盖状态信息，将其转换为布尔类型
     const bool is_covered  = vnode.is_covered  == 0 ? false : true;
     // 根据消息节点的前沿状态信息，将其转换为布尔类型
     const bool is_frontier = vnode.is_frontier == 0 ? false : true;
     // 根据消息节点的导航点状态信息，将其转换为布尔类型
     const bool is_navpoint = vnode.is_navpoint == 0 ? false : true;
     // 根据消息节点的边界状态信息，将其转换为布尔类型
     const bool is_boundary = vnode.is_boundary == 0 ? false : true;
     // 使用提取的位置信息和状态信息，调用 DynamicGraph 类的 CreateNavNodeFromPoint 函数创建一个导航节点
     DynamicGraph::CreateNavNodeFromPoint(p, node_ptr, false, is_navpoint, false, is_boundary);
     // 将新创建的导航节点设置为未激活状态
     node_ptr->is_active = false;
     /* Assign relative values */
     // 将节点的覆盖状态赋值给新创建的导航节点
     node_ptr->is_covered  = is_covered;
     // 将节点的前沿状态赋值给新创建的导航节点
     node_ptr->is_frontier = is_frontier;
     // 根据节点的前沿状态，调用 DynamicGraph 类的 FillFrontierVotes 函数填充前沿投票信息
     DynamicGraph::FillFrontierVotes(node_ptr, is_frontier);
     // positions
     // 将节点标记为已完成状态
     node_ptr->is_finalized = true;
     // 创建一个大小为 gm_params_.pool_size 的双端队列，队列中的元素初始化为节点的位置 p
     const std::deque<Point3D> pos_queue(gm_params_.pool_size, p);
     // 将位置队列赋值给节点的位置过滤器向量
     node_ptr->pos_filter_vec = pos_queue;
     // 从消息节点中提取表面方向信息，创建一个 PointPair 类型的表面方向对
     const PointPair surf_pair = {Point3D(vnode.surface_dirs[0].x, vnode.surface_dirs[0].y, vnode.surface_dirs[0].z),
                                  Point3D(vnode.surface_dirs[1].x, vnode.surface_dirs[1].y, vnode.surface_dirs[1].z)};
     // surf directions
     // 将消息节点的自由方向类型转换为 NodeFreeDirect 枚举类型，并赋值给新创建的导航节点
     node_ptr->free_direct = static_cast<NodeFreeDirect>(vnode.FreeType);
     // 将表面方向对赋值给新创建的导航节点
     node_ptr->surf_dirs = surf_pair;
     // 创建一个大小为 gm_params_.pool_size 的双端队列，队列中的元素初始化为表面方向对 surf_pair
     const std::deque<PointPair> surf_queue(gm_params_.pool_size, surf_pair);
     // 将表面方向队列赋值给节点的表面方向向量
     node_ptr->surf_dirs_vec = surf_queue;
 }
 
 /**
  * @brief 从消息节点中提取不同类型的连接节点 ID
  * 
  * 该函数用于从传入的 visibility_graph_msg::Node 类型的消息节点中提取图连接、多边形连接、轮廓连接和轨迹连接的节点 ID，
  * 并将这些 ID 分别存储到对应的索引栈中。在提取之前，会清空所有的索引栈。
  * 
  * @param node 输入的消息节点，包含各种连接信息
  * @param connect_idxs 输出参数，用于存储图连接的节点 ID
  * @param poly_idxs 输出参数，用于存储多边形连接的节点 ID
  * @param contour_idxs 输出参数，用于存储轮廓连接的节点 ID
  * @param traj_idxs 输出参数，用于存储轨迹连接的节点 ID
  */
 void GraphMsger::ExtractConnectIdxs(const visibility_graph_msg::Node& node,
                                     IdxStack& connect_idxs,
                                     IdxStack& poly_idxs,
                                     IdxStack& contour_idxs,
                                     IdxStack& traj_idxs)
 {
     // 清空所有的索引栈，确保每次提取前栈为空
     connect_idxs.clear(), poly_idxs.clear(), contour_idxs.clear(), traj_idxs.clear();
     // 遍历消息节点的图连接节点列表
     for (const auto& cid : node.connect_nodes) {
         // 将图连接节点的 ID 转换为 std::size_t 类型，并添加到 connect_idxs 栈中
         connect_idxs.push_back((std::size_t)cid);
     }
     // 遍历消息节点的多边形连接节点列表
     for (const auto& cid : node.poly_connects) {
         // 将多边形连接节点的 ID 转换为 std::size_t 类型，并添加到 poly_idxs 栈中
         poly_idxs.push_back((std::size_t)cid);
     }    
     // 遍历消息节点的轮廓连接节点列表
     for (const auto& cid : node.contour_connects) {
         // 将轮廓连接节点的 ID 转换为 std::size_t 类型，并添加到 contour_idxs 栈中
         contour_idxs.push_back((std::size_t)cid);
     }
     // 遍历消息节点的轨迹连接节点列表
     for (const auto& cid : node.trajectory_connects) {
         // 将轨迹连接节点的 ID 转换为 std::size_t 类型，并添加到 traj_idxs 栈中
         traj_idxs.push_back((std::size_t)cid);
     }
 }
 
 