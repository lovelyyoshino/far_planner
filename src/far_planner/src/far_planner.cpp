/*
 * FAR Planner
 * Copyright (C) 2021 Fan Yang - All rights reserved
 * fanyang2@andrew.cmu.edu,   
 */



 #include "far_planner/far_planner.h"

 /***************************************************************************************/
 
 /**
  * @brief 初始化 FARMaster 类的成员和订阅发布者
  * 
  * 此函数负责初始化 ROS 订阅者、发布者、定时器和各种处理对象。
  * 它还设置了内部参数，分配了内存，并初始化了 TF 监听器。
  */
 void FARMaster::Init() {
   /* initialize subscriber and publisher */
   // 订阅重置可见性图的话题，回调函数为 ResetGraphCallBack
   reset_graph_sub_    = nh.subscribe("/reset_visibility_graph", 5, &FARMaster::ResetGraphCallBack, this);
   // 订阅里程计话题，回调函数为 OdomCallBack
   odom_sub_           = nh.subscribe("/odom_world", 5, &FARMaster::OdomCallBack, this);
   // 订阅地形点云话题，回调函数为 TerrainCallBack
   terrain_sub_        = nh.subscribe("/terrain_cloud", 1, &FARMaster::TerrainCallBack, this);
   // 订阅扫描点云话题，回调函数为 ScanCallBack
   scan_sub_           = nh.subscribe("/scan_cloud", 5, &FARMaster::ScanCallBack, this);
   // 订阅目标点话题，回调函数为 WaypointCallBack
   waypoint_sub_       = nh.subscribe("/goal_point", 1, &FARMaster::WaypointCallBack, this);
   // 订阅局部地形点云话题，回调函数为 TerrainLocalCallBack
   terrain_local_sub_  = nh.subscribe("/terrain_local_cloud", 1, &FARMaster::TerrainLocalCallBack, this);
   // 订阅手柄命令话题，回调函数为 JoyCommandCallBack
   joy_command_sub_    = nh.subscribe("/joy", 5, &FARMaster::JoyCommandCallBack, this);
   // 订阅更新可见性图的命令话题，回调函数为 UpdateCommandCallBack
   update_command_sub_ = nh.subscribe("/update_visibility_graph", 5, &FARMaster::UpdateCommandCallBack, this);
   // 发布路径点消息
   goal_pub_           = nh.advertise<geometry_msgs::PointStamped>("/way_point",5);
   // 发布导航边界消息
   boundary_pub_       = nh.advertise<geometry_msgs::PolygonStamped>("/navigation_boundary",5);
   // Timers
   // 发布运行时间消息
   runtime_pub_        = nh.advertise<std_msgs::Float32>("/runtime",1);
   // 发布规划时间消息
   planning_time_pub_  = nh.advertise<std_msgs::Float32>("/planning_time",1);
   // 发布遍历时间消息
   traverse_time_pub_  = nh.advertise<std_msgs::Float32>("/far_traverse_time", 5);
   // planning status publisher
   // 发布是否到达目标的状态消息
   reach_goal_pub_     = nh.advertise<std_msgs::Bool>("/far_reach_goal_status", 5);
   // Terminal formatting subscriber
   // 订阅读取文件目录的命令话题，回调函数为 ReadFileCommand
   read_command_sub_   = nh.subscribe("/read_file_dir", 1, &FARMaster::ReadFileCommand, this);
   // 订阅保存文件目录的命令话题，回调函数为 SaveFileCommand
   save_command_sub_   = nh.subscribe("/save_file_dir", 1, &FARMaster::SaveFileCommand, this);
   // DEBUG Publisher
   // 发布动态障碍物调试点云消息
   dynamic_obs_pub_     = nh.advertise<sensor_msgs::PointCloud2>("/FAR_dynamic_obs_debug",1);
   // 发布周围自由空间调试点云消息
   surround_free_debug_ = nh.advertise<sensor_msgs::PointCloud2>("/FAR_free_debug",1);
   // 发布周围障碍物调试点云消息
   surround_obs_debug_  = nh.advertise<sensor_msgs::PointCloud2>("/FAR_obs_debug",1);
   // 发布扫描网格调试点云消息
   scan_grid_debug_     = nh.advertise<sensor_msgs::PointCloud2>("/FAR_scanGrid_debug",1);
   // 发布新点云调试消息
   new_PCL_pub_         = nh.advertise<sensor_msgs::PointCloud2>("/FAR_new_debug",1);
   // 发布地形高度调试点云消息
   terrain_height_pub_  = nh.advertise<sensor_msgs::PointCloud2>("/FAR_terrain_height_debug",1);
 
   // 加载 ROS 参数
   this->LoadROSParams();
 
   /*init path generation thred callback*/
   // 计算定时器的持续时间
   const float duration_time = 0.99f / master_params_.main_run_freq;
   // 创建一个定时器，定期调用 PlanningCallBack 函数
   planning_event_ = nh.createTimer(ros::Duration(duration_time), &FARMaster::PlanningCallBack, this);
 
   /* init Dynamic Planner Processing Objects */
   // 初始化轮廓检测器
   contour_detector_.Init(cdetect_params_);
   // 初始化图管理器
   graph_manager_.Init(nh, graph_params_);
   // 初始化图规划器
   graph_planner_.Init(nh, gp_params_);
   // 初始化轮廓图
   contour_graph_.Init(cg_params_);
   // 初始化规划可视化器
   planner_viz_.Init(nh);
   // 初始化地图处理器
   map_handler_.Init(map_params_);
   // 初始化扫描处理器
   scan_handler_.Init(scan_params_);
   // 初始化图消息器
   graph_msger_.Init(nh, msger_parmas_);
 
   /* init internal params */
   // 初始化里程计节点指针为空
   odom_node_ptr_      = NULL;
   // 标记点云是否初始化
   is_cloud_init_      = false;
   // 标记里程计是否初始化
   is_odom_init_       = false;
   // 标记扫描是否初始化
   is_scan_init_       = false;
   // 标记规划器是否正在运行
   is_planner_running_ = false;
   // 标记图是否初始化
   is_graph_init_      = false;
   // 标记是否重置环境
   is_reset_env_       = false;
   // 标记是否停止更新
   is_stop_update_     = false;
 
   // allocate memory to pointers
   // 为新顶点点云分配内存
   new_vertices_ptr_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为临时障碍物点云分配内存
   temp_obs_ptr_         = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为临时自由空间点云分配内存
   temp_free_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为临时点云分配内存
   temp_cloud_ptr_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为扫描网格点云分配内存
   scan_grid_ptr_        = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为局部地形点云分配内存
   local_terrain_ptr_    = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为地形高度点云分配内存
   terrain_height_ptr_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为视点周围点云分配内存
   viewpoint_around_ptr_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
   // 为视点障碍物点云的 kdtree 分配内存
   kdtree_viewpoint_obs_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>());
 
   // set kdtree sorted value
   // 设置新点云的 kdtree 不进行排序
   FARUtil::kdtree_new_cloud_->setSortedResults(false);
   // 设置过滤点云的 kdtree 不进行排序
   FARUtil::kdtree_filter_cloud_->setSortedResults(false);
   // 设置视点障碍物点云的 kdtree 不进行排序
   kdtree_viewpoint_obs_cloud_->setSortedResults(false);
 
   // init global utility cloud
   // 清空全局新点云栈
   FARUtil::stack_new_cloud_->clear();
   // 清空全局动态障碍物点云栈
   FARUtil::stack_dyobs_cloud_->clear();
 
   // init TF listener
   // 初始化 TF 监听器
   tf_listener_ = new tf::TransformListener();
 
   // clear temp vectors and memory
   // 清除临时内存
   this->ClearTempMemory();
   // 初始化机器人位置
   FARUtil::robot_pos = Point3D(0,0,0);
   // 设置自由里的初始位置
   FARUtil::free_odom_p = Point3D(0,0,0);
 
   robot_pos_   = Point3D(0,0,0); // 添加机器人的位置信息
   nav_heading_ = Point3D(0,0,0); // 初始朝向
   goal_waypoint_stamped_.header.frame_id = master_params_.world_frame; // 设置世界坐标系作为参考框架
   printf("\033[2J"), printf("\033[0;0H"); // 清屏
   std::cout<<std::endl;
   
   if (master_params_.is_static_env) { // 判断是静态环境还是动态环境
     std::cout<<"\033[1;33m **************** STATIC ENV PLANNING **************** \033[0m\n"<<std::endl;
   } else {
     std::cout<< "\033[1;33m **************** DYNAMIC ENV PLANNING **************** \033[0m\n" << std::endl;
   }
   std::cout<<"\n"<<std::endl; // 打印换行
 }
 
 /**
  * @brief 重置环境和图的状态
  * 
  * 此函数用于重置FARMaster类的内部状态、图管理器、地图处理器、图规划器和轮廓图。
  * 同时，它会清空所有的点云数据，并停止机器人的移动。
  */
 void FARMaster::ResetEnvironmentAndGraph() {
   // 调用 ResetInternalValues 函数重置内部值
   this->ResetInternalValues();
   // 如果不是调试模式，清理终端输出并打印图重置信息
   if (!FARUtil::IsDebug) { // Terminal Output
     // 光标上移两行
     printf("\033[A"), printf("\033[A");
     // 清除当前行内容
     printf("\033[2K");
     // 打印图重置信息
     std::cout<< "\033[1;31m V-Graph Resetting...\033[0m\n" << std::endl;
   }
   // 调用 graph_manager_ 的 ResetCurrentGraph 函数重置当前图
   graph_manager_.ResetCurrentGraph();
   // 调用 map_handler_ 的 ResetGripMapCloud 函数重置网格地图点云
   map_handler_.ResetGripMapCloud();
   // 调用 graph_planner_ 的 ResetPlannerInternalValues 函数重置规划器的内部值
   graph_planner_.ResetPlannerInternalValues();
   // 调用 contour_graph_ 的 ResetCurrentContour 函数重置当前轮廓
   contour_graph_.ResetCurrentContour();
   /* Reset clouds */
   // 清空周围障碍物点云
   FARUtil::surround_obs_cloud_->clear();
   // 清空周围自由空间点云
   FARUtil::surround_free_cloud_->clear();
   // 清空新点云栈
   FARUtil::stack_new_cloud_->clear();
   // 清空动态障碍物点云栈
   FARUtil::stack_dyobs_cloud_->clear();
   // 清空当前新点云
   FARUtil::cur_new_cloud_->clear();
   // 清空当前动态障碍物点云
   FARUtil::cur_dyobs_cloud_->clear();
   /* Stop the robot if it is moving */
   // 更新目标点消息的时间戳
   goal_waypoint_stamped_.header.stamp = ros::Time::now();
   // 设置目标点为机器人当前位置
   goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
   // 发布目标点消息以停止机器人
   goal_pub_.publish(goal_waypoint_stamped_);
   // 创建一个空的路径栈
   NodePtrStack empty_path;
   // 可视化空路径，以清除之前的路径可视化
   planner_viz_.VizPath(empty_path);
 }
 
 void FARMaster::Loop() {
    // 创建一个ros::Rate对象，用于控制循环的频率，频率由master_params_.main_run_freq指定
   ros::Rate loop_rate(master_params_.main_run_freq);
   // 进入主循环，只要ROS节点正常运行就持续执行
   while (ros::ok()) {
     // 检查是否需要重置环境和图
     if (is_reset_env_) {
       // 调用ResetEnvironmentAndGraph函数重置环境和图
       this->ResetEnvironmentAndGraph(); 
       // 重置重置标志
       is_reset_env_ = false;
       // 如果处于调试模式，输出警告信息
       if (FARUtil::IsDebug) ROS_WARN("****************** Graph and Env Reset ******************");
       // 休眠以跳过当前迭代
       loop_rate.sleep(); 
       // 继续下一次循环
       continue;
     }
     /* Process callback functions */
     // 处理ROS回调函数
     ros::spinOnce(); 
     // 检查前置条件是否满足
     if (!this->PreconditionCheck()) {
       // 休眠
       loop_rate.sleep();
       // 继续下一次循环
       continue;
     }
     /* add main process after this line */
     // 更新图管理器中的机器人位置
     graph_manager_.UpdateRobotPosition(robot_pos_);
     // 获取里程计节点指针
     odom_node_ptr_ = graph_manager_.GetOdomNode();
     // 检查里程计节点指针是否为空
     if (odom_node_ptr_ == NULL) {
       // 输出警告信息，提示等待里程计数据
       ROS_WARN("FAR: Waiting for Odometry...");
       // 休眠
       loop_rate.sleep();
       // 继续下一次循环
       continue;
     }
     /* Extract Vertices and new nodes */
     // 开始记录总V图更新的时间
     FARUtil::Timer.start_time("Total V-Graph Update");
     // 构建地形图像并提取轮廓
     contour_detector_.BuildTerrainImgAndExtractContour(odom_node_ptr_, FARUtil::surround_obs_cloud_, realworld_contour_);
     // 更新轮廓图
     contour_graph_.UpdateContourGraph(odom_node_ptr_, realworld_contour_);
     // 检查图是否已经初始化
     if (is_graph_init_) {
       // 如果不是调试模式，清理终端当前行输出
       if (!FARUtil::IsDebug) printf("\033[2K");
       // 输出局部V图更新信息，包括局部顶点数量
       std::cout<<"    "<<"Local V-Graph Updated. Number of local vertices: "<<ContourGraph::contour_graph_.size()<<std::endl;
     }
     /* Adjust heights with terrain */
     // 调整轮廓图节点的高度
     map_handler_.AdjustCTNodeHeight(ContourGraph::contour_graph_);
     // 调整导航图节点的高度
     map_handler_.AdjustNodesHeight(nav_graph_);
     // 截断局部范围内的节点
     graph_manager_.UpdateGlobalNearNodes();
     // 获取扩展的局部节点
     near_nav_graph_ = graph_manager_.GetExtendLocalNode();
     // 匹配近导航节点和轮廓
     contour_graph_.MatchContourWithNavGraph(nav_graph_, near_nav_graph_, new_ctnodes_);
     // 检查是否开启OpenCV可视化
     if (master_params_.is_visual_opencv) {
       // 将新的轮廓节点转换为点云
       FARUtil::ConvertCTNodeStackToPCL(new_ctnodes_, new_vertices_ptr_);
       // 获取云图像的OpenCV矩阵
       cv::Mat cloud_img = contour_detector_.GetCloudImgMat();
       // 显示带有角点的图像
       contour_detector_.ShowCornerImage(cloud_img, new_vertices_ptr_);
     }
     /* update planner graph */
     // 清空新节点列表
     new_nodes_.clear();
     // 检查是否停止更新，并且尝试提取图节点
     if (!is_stop_update_ && graph_manager_.ExtractGraphNodes(new_ctnodes_)) {
       // 获取新的节点
       new_nodes_ = graph_manager_.GetNewNodes();
     }
     // 检查图是否已经初始化
     if (is_graph_init_) {
       // 如果不是调试模式，清理终端当前行输出
       if (!FARUtil::IsDebug) printf("\033[2K");
       // 输出添加到全局V图的新顶点数量
       std::cout<<"    "<< "Number of new vertices adding to global V-Graph: "<< new_nodes_.size()<<std::endl;
     }
     /* Graph Updating */
     // 更新导航图
     graph_manager_.UpdateNavGraph(new_nodes_, is_stop_update_, clear_nodes_);
     // 结束记录总V图更新的时间，并将结果转换为秒
     runtimer_.data = FARUtil::Timer.end_time("Total V-Graph Update", is_graph_init_) / 1000.f; // Unit: second
     // 发布运行时间
     runtime_pub_.publish(runtimer_);
     /* Update v-graph in other modules */
     // 获取更新后的导航图
     nav_graph_ = graph_manager_.GetNavGraph();
     // 检查图是否已经初始化
     if (is_graph_init_) {
       // 如果不是调试模式，清理终端当前行输出
       if (!FARUtil::IsDebug) printf("\033[2K");
       // 输出全局V图更新信息，包括全局顶点数量
       std::cout<<"    "<<"Global V-Graph Updated. Number of global vertices: "<<nav_graph_.size()<<std::endl;
     }
     // 提取全局轮廓
     contour_graph_.ExtractGlobalContours();      // Global Polygon Update
     // 更新图规划器中的图
     graph_planner_.UpdaetVGraph(nav_graph_);     // Graph Planner Update
     // 更新图消息器中的图
     graph_msger_.UpdateGlobalGraph(nav_graph_);  // Graph Messager Update
 
     /* Publish local boundary to lower level local planner */
     // 发布局部边界信息到下层局部规划器
     this->LocalBoundaryHandler(ContourGraph::local_boundary_);
 
     /* Viz Navigation Graph */
     // 获取最后一个内部导航节点指针
     const NavNodePtr last_internav_ptr = graph_manager_.GetLastInterNavNode();
     // 检查最后一个内部导航节点指针是否为空
     if (last_internav_ptr != NULL) {
       // 可视化最后一个内部导航节点
       planner_viz_.VizPoint3D(last_internav_ptr->position, "last_nav_node", VizColor::MAGNA, 1.0);
     }
     // 可视化要清除的节点
     planner_viz_.VizNodes(clear_nodes_, "clear_nodes", VizColor::ORANGE);
     // 可视化轮廓外的节点
     planner_viz_.VizNodes(graph_manager_.GetOutContourNodes(), "out_contour", VizColor::YELLOW);
     // 可视化自由里程计位置
     planner_viz_.VizPoint3D(FARUtil::free_odom_p, "free_odom_position", VizColor::ORANGE, 1.0);
     // 可视化导航图
     planner_viz_.VizGraph(nav_graph_);
     // 可视化轮廓图
     planner_viz_.VizContourGraph(ContourGraph::contour_graph_);
     // 可视化全局多边形
     planner_viz_.VizGlobalPolygons(ContourGraph::global_contour_, ContourGraph::unmatched_contour_);
 
     // 检查图是否已经初始化
     if (is_graph_init_) { 
       // 如果处于调试模式，输出分隔线
       if (FARUtil::IsDebug) {
         std::cout<<" ========================================================== "<<std::endl;
       } else { // cleanup outputs in terminal
         // 清理终端输出
         for (int i = 0; i < 6; i++) {
           printf("\033[A");
         }
       }
     }
 
     // 检查图是否未初始化且导航图不为空
     if (!is_graph_init_ && !nav_graph_.empty()) {
       // 标记图已初始化
       is_graph_init_ = true;
       // 清理终端输出
       printf("\033[A"), printf("\033[A"), printf("\033[2K");
       // 输出图初始化信息
       std::cout<< "\033[1;32m V-Graph Initialized \033[0m\n" << std::endl;
     }
     // 休眠以控制循环频率
     loop_rate.sleep();
   }
 }
 
 /**
  * @brief 规划回调函数，处理路径规划相关逻辑
  * 
  * 该函数在每次定时器触发时被调用，根据图是否初始化以及目标节点是否存在，执行不同的路径规划操作。
  * 如果图未初始化，函数将直接返回。如果目标节点不存在，将更新图的可遍历性；如果目标节点存在，
  * 则更新目标位置、添加目标到图中、更新图的可遍历性、构建路径并发布航点信息。
  * 
  * @param event 定时器事件，包含定时器触发的时间信息
  */
 void FARMaster::PlanningCallBack(const ros::TimerEvent& event) {
   // 如果图未初始化，直接返回
   if (!is_graph_init_) return;
   // 获取目标节点指针
   const NavNodePtr goal_ptr = graph_planner_.GetGoalNodePtr();
   // 如果目标节点指针为空
   if (goal_ptr == NULL) {
     /* Graph Traversablity Update */
     // 如果不是调试模式，清理终端当前行输出
     if (!FARUtil::IsDebug) printf("\033[2K");
     // 输出添加目标到V图的信息，时间为0ms
     std::cout<<"    "<<"Adding Goal to V-Graph "<<"Time: "<<0.f<<"ms"<<std::endl;
     // 更新图的可遍历性，传入里程计节点指针和空指针
     graph_planner_.UpdateGraphTraverability(odom_node_ptr_, NULL);
     // 如果不是调试模式，清理终端当前行输出
     if (!FARUtil::IsDebug) printf("\033[2K");
     // 输出路径搜索的信息，时间为0ms
     std::cout<<"    "<<"Path Search "<<"Time: "<<0.f<<"ms"<<std::endl;
   } else { 
     // 更新目标位置，结合附近的地形点云
     // 获取起始节点的位置
     const Point3D ori_p = graph_planner_.GetOriginNodePos(true);
     // 创建目标障碍物点云指针
     PointCloudPtr goal_obs(new pcl::PointCloud<PCLPoint>());
     // 创建目标自由空间点云指针
     PointCloudPtr goal_free(new pcl::PointCloud<PCLPoint>());
     // 获取起始点周围的障碍物点云
     map_handler_.GetCloudOfPoint(ori_p, goal_obs, CloudType::OBS_CLOUD, true);
     // 获取起始点周围的自由空间点云
     map_handler_.GetCloudOfPoint(ori_p, goal_free, CloudType::FREE_CLOUD, true);
     // 更新自由地形网格
     graph_planner_.UpdateFreeTerrainGrid(ori_p, goal_obs, goal_free);
     // 重新评估目标位置
     graph_planner_.ReEvaluateGoalPosition(goal_ptr, !master_params_.is_multi_layer);
 
     // 将目标添加到V图中
     // 开始记录添加目标到V图的时间
     FARUtil::Timer.start_time("Adding Goal to V-Graph");
     // 更新目标导航节点的连接
     graph_planner_.UpdateGoalNavNodeConnects(goal_ptr); 
     // 更新图规划器中的图
     graph_planner_.UpdaetVGraph(graph_manager_.GetNavGraph());
     // 如果不是调试模式，清理终端当前行输出
     if (!FARUtil::IsDebug) printf("\033[2K");
     // 结束记录添加目标到V图的时间
     FARUtil::Timer.end_time("Adding Goal to V-Graph");
 
     // 更新V图的可遍历性
     // 开始记录路径搜索的时间
     FARUtil::Timer.start_time("Path Search");
     // 更新图的可遍历性，传入里程计节点指针和目标节点指针
     graph_planner_.UpdateGraphTraverability(odom_node_ptr_, goal_ptr);
 
     // 构建到目标的路径并发布航点
     // 创建全局路径栈
     NodePtrStack global_path;
     // 当前自由目标点
     Point3D current_free_goal;
     // 最后一个导航节点指针
     NavNodePtr last_nav_ptr = nav_node_ptr_;
     // 标记规划是否失败
     bool is_planning_fails = false;
     // 更新目标航点消息的时间戳
     goal_waypoint_stamped_.header.stamp = ros::Time::now();
     // 标记当前是否为自由导航
     bool is_current_free_nav = false;
     // 标记是否到达目标
     bool is_reach_goal = false;
     // 如果路径规划成功且当前导航节点指针不为空
     if (graph_planner_.PathToGoal(goal_ptr, global_path, nav_node_ptr_, current_free_goal, is_planning_fails, is_reach_goal, is_current_free_nav) && nav_node_ptr_ != NULL) {
       // 航点初始化为当前导航节点的位置
       Point3D waypoint = nav_node_ptr_->position;
       // 如果当前导航节点不是目标节点
       if (nav_node_ptr_ != goal_ptr) {
         // 投影导航航点
         waypoint = this->ProjectNavWaypoint(nav_node_ptr_, last_nav_ptr);
       } else if (master_params_.is_viewpoint_extend) {
         // 可视化目标点扩展
         planner_viz_.VizViewpointExtend(goal_ptr, goal_ptr->position);
       }
       // 将航点转换为几何消息点
       goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(waypoint);
       // 发布目标航点消息
       goal_pub_.publish(goal_waypoint_stamped_);
       // 标记规划器正在运行
       is_planner_running_ = true;
       // 可视化航点
       planner_viz_.VizPoint3D(waypoint, "waypoint", VizColor::MAGNA, 1.5);
       // 可视化当前自由目标点
       planner_viz_.VizPoint3D(current_free_goal, "free_goal", VizColor::GREEN, 1.5);
       // 可视化全局路径
       planner_viz_.VizPath(global_path, is_current_free_nav);
     } else if (is_planner_running_) {
       // 停止机器人
       // 清空全局路径栈
       global_path.clear();
       // 可视化空路径
       planner_viz_.VizPath(global_path);
       // 标记规划器停止运行
       is_planner_running_ = false;
       // 重置导航朝向
       nav_heading_ = Point3D(0,0,0);
       // 如果规划失败
       if (is_planning_fails) { 
         // 设置目标航点为机器人当前位置
         goal_waypoint_stamped_.point = FARUtil::Point3DToGeoMsgPoint(robot_pos_);
         // 发布目标航点消息
         goal_pub_.publish(goal_waypoint_stamped_);
       }
     }
     // 如果不是调试模式，清理终端当前行输出
     if (!FARUtil::IsDebug) printf("\033[2K");
 
     // 发布规划器状态和计时器信息
     // 创建到达目标消息
     std_msgs::Bool reach_goal_msg;
     // 设置到达目标消息的数据
     reach_goal_msg.data = is_reach_goal;
     // 发布到达目标消息
     reach_goal_pub_.publish(reach_goal_msg);
     // 创建遍历计时器消息
     std_msgs::Float32 traverse_timer;
     // 设置遍历计时器消息的数据
     traverse_timer.data = FARUtil::Timer.record_time("Overall_executing");
     // 发布遍历计时器消息
     traverse_time_pub_.publish(traverse_timer);
     // 如果到达目标
     if (is_reach_goal) {
       // 结束记录整体执行时间
       FARUtil::Timer.end_time("Overall_executing", false);
     }
     // 记录路径搜索时间
     plan_timer_.data = FARUtil::Timer.end_time("Path Search");
     // 发布规划时间消息
     planning_time_pub_.publish(plan_timer_);
   }
 }
 
 /**
  * @brief 处理并发布局部边界信息到下层局部规划器
  * 
  * 该函数接收一个局部边界点对的向量，根据配置参数和距离条件对边界进行筛选和排序，
  * 然后将筛选后的边界转换为几何多边形消息并发布。
  * 
  * @param local_boundary 局部边界点对的向量，每个点对表示一条边界线段
  */
 void FARMaster::LocalBoundaryHandler(const std::vector<PointPair>& local_boundary) {
   // 如果不发布边界信息或者局部边界为空，则直接返回
   if (!master_params_.is_pub_boundary || local_boundary.empty()) return;
   // 创建一个多边形消息对象
   geometry_msgs::PolygonStamped boundary_poly;
   // 设置多边形消息的坐标系
   boundary_poly.header.frame_id = master_params_.world_frame;
   // 设置多边形消息的时间戳
   boundary_poly.header.stamp = ros::Time::now();
   // 获取机器人当前位置的z坐标
   float index_z = robot_pos_.z;
   // 用于存储筛选后的边界点对
   std::vector<PointPair> sorted_boundary;
   // 遍历所有边界点对
   for (const auto& edge : local_boundary) {
     // 如果边界点对到机器人的距离超过局部规划范围，则跳过该边界点对
     if (FARUtil::DistanceToLineSeg2D(robot_pos_, edge) > master_params_.local_planner_range) continue;
     // 将符合条件的边界点对添加到筛选后的列表中
     sorted_boundary.push_back(edge);
   }
   // 对筛选后的边界点对进行顺时针排序，仅用于更好的rviz可视化
   FARUtil::SortEdgesClockWise(robot_pos_, sorted_boundary); /* For better rviz visualization purpose only! */ 
   // 遍历筛选并排序后的边界点对
   for (const auto& edge : sorted_boundary) {
     // 创建两个几何点对象
     geometry_msgs::Point32 geo_p1, geo_p2;
     // 设置第一个点的坐标
     geo_p1.x = edge.first.x,  geo_p1.y = edge.first.y,  geo_p1.z = index_z;
     // 设置第二个点的坐标
     geo_p2.x = edge.second.x, geo_p2.y = edge.second.y, geo_p2.z = index_z;
     // 将两个点添加到多边形消息的点列表中
     boundary_poly.polygon.points.push_back(geo_p1), boundary_poly.polygon.points.push_back(geo_p2);
     // 增加z坐标，用于分隔多边形的线条
     index_z += 0.001f; // seperate polygon lines
   }
   // 发布多边形消息
   boundary_pub_.publish(boundary_poly);
 }
 
 
 /**
  * @brief 投影导航航点
  * 
  * 该函数根据当前导航节点和上一个导航节点的位置，计算并投影出一个新的导航航点。
  * 考虑了动量、视点扩展、方向变化等因素，以确保机器人能够平滑地移动到目标位置。
  * 
  * @param nav_node_ptr 当前导航节点的指针
  * @param last_point_ptr 上一个导航节点的指针
  * @return Point3D 投影后的导航航点位置
  */
 Point3D FARMaster::ProjectNavWaypoint(const NavNodePtr& nav_node_ptr, const NavNodePtr& last_point_ptr) {
   // 标记是否具有动量
   bool is_momentum = false;
   // 如果上一个导航节点指针等于当前导航节点指针，或者上一个导航节点指针不为空且两点之间的距离小于近邻距离
   if (last_point_ptr == nav_node_ptr || (last_point_ptr != NULL && (last_point_ptr->position - nav_node_ptr_->position).norm() < FARUtil::kNearDist)) {
     // 标记具有动量
     is_momentum = true;
   }
   // 初始化航点为当前导航节点的位置
   Point3D waypoint = nav_node_ptr->position;
   // 初始化自由距离为局部规划范围
   float free_dist = master_params_.local_planner_range;
   // 调用 ExtendViewpointOnObsCloud 函数扩展视点，获取扩展后的点
   const Point3D extend_p = this->ExtendViewpointOnObsCloud(nav_node_ptr_, FARUtil::surround_obs_cloud_, free_dist);
   // 确保自由距离不小于机器人尺寸的2.5倍
   free_dist = std::max(free_dist, master_params_.robot_dim * 2.5f);
   // 如果启用了视点扩展
   if (master_params_.is_viewpoint_extend) {
     // 更新航点为扩展后的点
     waypoint = extend_p;
     // 可视化视点扩展
     planner_viz_.VizViewpointExtend(nav_node_ptr_, waypoint);
   }
   // 计算航点与机器人位置的差值向量
   const Point3D diff_p = waypoint - robot_pos_;
   // 初始化新的航向向量
   Point3D new_heading;
   // 如果具有动量且当前导航航向向量的模大于一个极小值
   if (is_momentum && nav_heading_.norm() > FARUtil::kEpsilon) {
     // 计算半自由距离
     const float hdist = free_dist / 2.0f;
     // 计算比例因子，确保不超过半自由距离
     const float ratio = std::min(hdist, diff_p.norm()) / hdist;
     // 根据比例因子和当前导航航向向量计算新的航向向量
     new_heading = diff_p.normalize() * ratio + nav_heading_ * (1.0f - ratio);
   } else {
     // 否则，新的航向向量为差值向量的单位向量
     new_heading = diff_p.normalize();
   }
   // 如果当前导航航向向量的模大于一个极小值，且新的航向向量与当前导航航向向量的点积小于0（方向相反）
   if (nav_heading_.norm() > FARUtil::kEpsilon && new_heading.norm_dot(nav_heading_) < 0.0f) { 
     // 创建一个临时航向向量，垂直于当前导航航向向量
     Point3D temp_heading(nav_heading_.y, -nav_heading_.x, nav_heading_.z);
     // 如果临时航向向量与新的航向向量的点积小于0
     if (temp_heading.norm_dot(new_heading) < 0.0f) {
       // 反转临时航向向量的方向
       temp_heading.x = -temp_heading.x, temp_heading.y = -temp_heading.y;
     }
     // 更新新的航向向量为临时航向向量
     new_heading = temp_heading;
   }
   // 将新的航向向量归一化
   nav_heading_ = new_heading.normalize();
   // 如果差值向量的模小于自由距离
   if (diff_p.norm() < free_dist) {
     // 沿着新的航向向量扩展航点，使其距离机器人为自由距离
     waypoint = waypoint + nav_heading_ * (free_dist - diff_p.norm());
   }
   // 返回投影后的导航航点
   return waypoint;
 }
 
 /**
  * @brief 在障碍物点云上扩展视点
  * 
  * 该函数根据给定的导航节点指针和障碍物点云，在障碍物点云上扩展视点，以找到一个合适的导航航点。
  * 考虑了导航节点的自由方向、障碍物的分布以及最大扩展距离等因素。
  * 
  * @param nav_node_ptr 当前导航节点的指针
  * @param obsCloudIn 障碍物点云的指针
  * @param free_dist 自由距离，函数可能会修改该值
  * @return Point3D 扩展后的视点位置
  */
 Point3D FARMaster::ExtendViewpointOnObsCloud(const NavNodePtr& nav_node_ptr, const PointCloudPtr& obsCloudIn, float& free_dist) {
   // 如果导航节点的自由方向不是凸的，或者障碍物点云为空，则直接返回导航节点的位置
   if (nav_node_ptr_->free_direct != NodeFreeDirect::CONVEX || obsCloudIn->empty()) return nav_node_ptr_->position;
   // 裁剪障碍物点云，获取导航节点周围一定范围内的点云
   FARUtil::CropPCLCloud(obsCloudIn, viewpoint_around_ptr_, nav_node_ptr_->position, free_dist + FARUtil::kNearDist);
   // 计算最大扩展距离，取导航节点到机器人的距离和自由距离的最小值减去近邻距离
   float maxR = std::min((nav_node_ptr_->position - robot_pos_).norm(), free_dist) - FARUtil::kNearDist;
   // 确保最大扩展距离不小于0
   maxR = std::max(maxR, 0.0f);
   // 标记是否为墙
   bool is_wall = false;
   // 计算表面拓扑方向的反方向
   const Point3D direct = -FARUtil::SurfTopoDirect(nav_node_ptr_->surf_dirs, is_wall);
   // 如果不是墙
   if (!is_wall) {
     // 初始化航点为导航节点的位置
     Point3D waypoint = nav_node_ptr_->position;
     // 如果导航节点周围的点云为空
     if (viewpoint_around_ptr_->empty()) {
       // 沿着表面拓扑反方向扩展航点，扩展距离为最大扩展距离
       waypoint = waypoint + direct * maxR;
     } else {
       // 设置用于查找的kdtree的输入点云
       kdtree_viewpoint_obs_cloud_->setInputCloud(viewpoint_around_ptr_);
       // 计算点计数阈值，用于判断点是否被占用
       const int N_Thred = (int)std::floor(FARUtil::kNearDist / FARUtil::kLeafSize);
       // 计算查找半径
       const float R = FARUtil::kNearDist / 2.0f + FARUtil::kLeafSize;
       // 射线追踪起始点，从导航节点位置沿着表面拓扑反方向扩展近邻距离
       Point3D start_p = waypoint + direct * FARUtil::kNearDist;
       // 射线追踪距离初始化为近邻距离
       float ray_dist  = FARUtil::kNearDist; 
       // 判断起始点是否被占用
       bool is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred;
       // 初始化航点为起始点
       waypoint = start_p;
       // 当起始点未被占用且射线追踪距离小于自由距离时，继续射线追踪
       while (!is_occupied && ray_dist < free_dist) {
         // 沿着表面拓扑反方向扩展起始点，扩展距离为近邻距离
         start_p = start_p + direct * FARUtil::kNearDist;
         // 增加射线追踪距离
         ray_dist += FARUtil::kNearDist;
         // 判断新的起始点是否被占用
         is_occupied = FARUtil::PointInXCounter(start_p, R, kdtree_viewpoint_obs_cloud_) > N_Thred;
         // 如果射线追踪距离小于最大扩展距离，更新航点为新的起始点
         if (ray_dist < maxR) {
           waypoint = start_p;
         }
       }
       // 如果起始点被占用
       if (is_occupied) {
         // 计算新的航点，取导航节点位置和当前航点的中点，并减去近邻距离
         waypoint = (nav_node_ptr_->position + waypoint - direct * FARUtil::kNearDist) / 2.0f;
         // 设置航点的z坐标为导航节点的z坐标
         waypoint.z = nav_node_ptr_->position.z;
         // 更新自由距离，减去近邻距离
         free_dist = ray_dist - FARUtil::kNearDist;
       }
       // 返回扩展后的航点
       return waypoint;
     }
   }
   // 如果是墙或者其他情况，返回导航节点的位置
   return nav_node_ptr_->position;
 }
 
 
 
 /**
  * @brief 加载ROS参数配置
  * 
  * 该函数从ROS参数服务器中加载配置信息，并填充到相关的结构体和全局变量中，它为系统运行提供所需的各种初始化参数。
  */
 void FARMaster::LoadROSParams() {
   const std::string master_prefix   = "/far_planner/"; // 主前缀
   const std::string map_prefix      = master_prefix + "MapHandler/"; // 地图处理相关的前缀
   const std::string scan_prefix     = master_prefix + "ScanHandler/"; // 扫描处理相关的前缀
   const std::string cdetect_prefix  = master_prefix + "CDetector/"; // 轮廓检测器相关的前缀
   const std::string graph_prefix    = master_prefix + "Graph/"; // 图形管理相关的前缀
   const std::string viz_prefix      = master_prefix + "Viz/"; // 可视化相关的前缀
   const std::string utility_prefix  = master_prefix + "Util/"; // 工具类相关的前缀
   const std::string planner_prefix  = master_prefix + "GPlanner/"; // 规划器相关的前缀
   const std::string contour_prefix  = master_prefix + "ContourGraph/"; // 轮廓图相关的前缀
   const std::string msger_prefix    = master_prefix + "GraphMsger/"; // 图消息相关的前缀
 
   // master params
   nh.param<float>(master_prefix + "main_run_freq",         master_params_.main_run_freq, 5.0); // 获取主线程运行频率
   nh.param<float>(master_prefix + "voxel_dim",             master_params_.voxel_dim, 0.2); // 获取每个体素的大小
   nh.param<float>(master_prefix + "robot_dim",             master_params_.robot_dim, 0.8); // 获取机器人尺寸
   nh.param<float>(master_prefix + "vehicle_height",        master_params_.vehicle_height, 0.75); // 获取车辆高度
   nh.param<float>(master_prefix + "sensor_range",          master_params_.sensor_range, 50.0); // 获取传感器范围
   nh.param<float>(master_prefix + "terrain_range",         master_params_.terrain_range, 15.0); // 获取地形探测范围
   nh.param<float>(master_prefix + "local_planner_range",   master_params_.local_planner_range, 5.0); // 获取局部规划器范围
   nh.param<float>(master_prefix + "visualize_ratio",       master_params_.viz_ratio, 1.0); // 获取可视化比例
   nh.param<bool>(master_prefix  + "is_viewpoint_extend",   master_params_.is_viewpoint_extend, true); // 是否启用观点扩展
   nh.param<bool>(master_prefix  + "is_multi_layer",        master_params_.is_multi_layer, false); // 是否启用多层绘制
   nh.param<bool>(master_prefix  + "is_opencv_visual",      master_params_.is_visual_opencv, true); // 是否使用OpenCV可视化
   nh.param<bool>(master_prefix  + "is_static_env",         master_params_.is_static_env, true); // 是否取静态环境
   nh.param<bool>(master_prefix  + "is_pub_boundary",       master_params_.is_pub_boundary, true); // 是否出版边界值
   nh.param<bool>(master_prefix  + "is_debug_output",       master_params_.is_debug_output, false); // 是否是调试输出模式
   nh.param<bool>(master_prefix  + "is_attempt_autoswitch", master_params_.is_attempt_autoswitch, true); // 是否尝试自动切换
   nh.param<std::string>(master_prefix + "world_frame",     master_params_.world_frame, "map"); // 设置世界框架名称
   
   master_params_.terrain_range = std::min(master_params_.terrain_range, master_params_.sensor_range); // 确保地形范围小于或等于传感器范围
   
   // map handler params
   nh.param<float>(map_prefix + "floor_height",        map_params_.floor_height, 2.0); // 地板高度
   nh.param<float>(map_prefix + "cell_length",         map_params_.cell_length, 5.0); // 每个网格单元的长度
   nh.param<float>(map_prefix + "map_grid_max_length", map_params_.grid_max_length, 5000.0); // 地图最大网格长度
   nh.param<float>(map_prefix + "map_grad_max_height", map_params_.grid_max_height, 100.0); // 最大地图高度
   map_params_.height_voxel_dim = master_params_.voxel_dim * 2.0f; // 高度体素维度设置
   map_params_.cell_height      = map_params_.floor_height / 2.5f; // 网格单元高度
   map_params_.sensor_range     = master_params_.sensor_range; // 更新地图处理器中的传感器范围
 
   // utility params
   nh.param<float>(utility_prefix + "angle_noise",            FARUtil::kAngleNoise, 15.0); // 角度噪音阈值
   nh.param<float>(utility_prefix + "accept_max_align_angle", FARUtil::kAcceptAlign, 15.0); // 接受的对齐最大角度
   nh.param<float>(utility_prefix + "new_intensity_thred",    FARUtil::kNewPIThred, 2.0); // 新强度阈值
   nh.param<float>(utility_prefix + "nav_clear_dist",         FARUtil::kNavClearDist, 0.5); // 导航清除距离
   nh.param<float>(utility_prefix + "terrain_free_Z",         FARUtil::kFreeZ, 0.1); // 自由Z轴阈值
   nh.param<int>(utility_prefix   + "dyosb_update_thred",     FARUtil::kDyObsThred, 4); // 动态障碍物更新阈值
   nh.param<int>(utility_prefix   + "new_point_counter",      FARUtil::KNewPointC, 10); // 新点计数器
   nh.param<float>(utility_prefix + "dynamic_obs_dacay_time", FARUtil::kObsDecayTime, 10.0); // 动态障碍物衰减时间
   nh.param<float>(utility_prefix + "new_points_decay_time",  FARUtil::kNewDecayTime, 2.0); // 新点衰减时间
   nh.param<int>(utility_prefix   + "obs_inflate_size",       FARUtil::kObsInflate, 2); // 障碍物膨胀大小
   FARUtil::kLeafSize       = master_params_.voxel_dim; // 定义KD树节点叶子的尺寸
   FARUtil::kNearDist       = master_params_.robot_dim; // 定义临近点的距离
   FARUtil::kHeightVoxel    = map_params_.height_voxel_dim; // 获取高度体素维度
   FARUtil::kMatchDist      = master_params_.robot_dim * 2.0f + FARUtil::kLeafSize; // 匹配距离计算
   FARUtil::kNavClearDist   = master_params_.robot_dim / 2.0f + FARUtil::kLeafSize; // 导航清空距离定义
   FARUtil::kProjectDist    = master_params_.voxel_dim; // 投影距离获得
   FARUtil::worldFrameId    = master_params_.world_frame; // 获得世界帧ID
   FARUtil::kVizRatio       = master_params_.viz_ratio; // 可视化比例
   FARUtil::kTolerZ         = map_params_.floor_height - FARUtil::kHeightVoxel; // 容忍高差定义
   FARUtil::kCellLength     = map_params_.cell_length; // 单元长度设定
   FARUtil::kCellHeight     = map_params_.cell_height; // 单元高度设定
   FARUtil::kAcceptAlign    = FARUtil::kAcceptAlign / 180.0f * M_PI; // 将接受的对齐角度转换为弧度
   FARUtil::kAngleNoise     = FARUtil::kAngleNoise  / 180.0f * M_PI; // 将角度噪声转为弧度
   FARUtil::robot_dim       = master_params_.robot_dim; // 机器人的维度
   FARUtil::IsStaticEnv     = master_params_.is_static_env; // 静态环境标记
   FARUtil::IsDebug         = master_params_.is_debug_output; // 调试输出是否开启
   FARUtil::IsMultiLayer    = master_params_.is_multi_layer; // 多层渲染标记
   FARUtil::vehicle_height  = master_params_.vehicle_height; // 车辆高度信息
   FARUtil::kSensorRange    = master_params_.sensor_range; // 传感器范围
   FARUtil::kMarginDist     = master_params_.sensor_range - FARUtil::kMatchDist; // 边距栅栏
   FARUtil::kMarginHeight   = FARUtil::kTolerZ - FARUtil::kCellHeight / 2.0f; // 高度容错边际
   FARUtil::kTerrainRange   = master_params_.terrain_range; // 地形探测区间
   FARUtil::kLocalPlanRange = master_params_.local_planner_range; // 本地计划范围 
 
 
   // graph planner params
   nh.param<float>(planner_prefix + "converge_distance",    gp_params_.converge_dist, 1.0);
   nh.param<float>(planner_prefix + "goal_adjust_radius",   gp_params_.adjust_radius, 10.0);
   nh.param<int>(planner_prefix   + "free_counter_thred",   gp_params_.free_thred, 5);
   nh.param<int>(planner_prefix   + "reach_goal_vote_size", gp_params_.votes_size, 5);
   nh.param<int>(planner_prefix   + "path_momentum_thred",  gp_params_.momentum_thred, 5);
   gp_params_.momentum_dist = master_params_.robot_dim / 2.0f;
   gp_params_.is_autoswitch = master_params_.is_attempt_autoswitch;
 
   // contour graph params
   cg_params_.kPillarPerimeter = master_params_.robot_dim * 4.0f;
 
   // dynamic graph params
   nh.param<int>(graph_prefix    + "connect_votes_size",        graph_params_.votes_size, 10);
   nh.param<int>(graph_prefix    + "clear_dumper_thred",        graph_params_.dumper_thred, 3);
   nh.param<int>(graph_prefix    + "node_finalize_thred",       graph_params_.finalize_thred, 3);
   nh.param<int>(graph_prefix    + "filter_pool_size",          graph_params_.pool_size, 12);
   nh.param<float>(graph_prefix  + "connect_angle_thred",       graph_params_.kConnectAngleThred, 10.0);
   nh.param<float>(graph_prefix  + "dirs_filter_margin",        graph_params_.filter_dirs_margin, 10.0);
   graph_params_.filter_pos_margin        = FARUtil::kNavClearDist;
   graph_params_.filter_dirs_margin       = FARUtil::kAngleNoise;
   graph_params_.kConnectAngleThred       = FARUtil::kAcceptAlign;
   graph_params_.frontier_perimeter_thred = FARUtil::kMatchDist * 4.0f;
 
   // graph messager params
   nh.param<int>(msger_prefix + "robot_id", msger_parmas_.robot_id, 0);
   msger_parmas_.frame_id    = master_params_.world_frame;
   msger_parmas_.votes_size  = graph_params_.votes_size;
   msger_parmas_.pool_size   = graph_params_.pool_size;
   msger_parmas_.dist_margin = graph_params_.filter_pos_margin;
 
   // scan handler params
   scan_params_.terrain_range = master_params_.terrain_range;
   scan_params_.voxel_size    = master_params_.voxel_dim;
   scan_params_.ceil_height   = map_params_.floor_height;
 
   // contour detector params
   nh.param<float>(cdetect_prefix       + "resize_ratio",       cdetect_params_.kRatio, 5.0);
   nh.param<int>(cdetect_prefix         + "filter_count_value", cdetect_params_.kThredValue, 5);
   nh.param<bool>(cdetect_prefix        + "is_save_img",        cdetect_params_.is_save_img, false);
   nh.param<std::string>(cdetect_prefix + "img_folder_path",    cdetect_params_.img_path, "");
   cdetect_params_.kBlurSize    = (int)std::round(FARUtil::kNavClearDist / master_params_.voxel_dim);
   cdetect_params_.sensor_range = master_params_.sensor_range;
   cdetect_params_.voxel_dim    = master_params_.voxel_dim;
 }
 
 /**
  * @brief 处理里程计消息的回调函数
  * 
  * 该函数接收里程计消息，将里程计坐标系下的位姿转换到世界坐标系下，
  * 并更新机器人的位置和航向信息。同时，在首次接收到里程计消息时，
  * 初始化系统启动时间和地图原点。
  * 
  * @param msg 里程计消息的常量指针
  */
 void FARMaster::OdomCallBack(const nav_msgs::OdometryConstPtr& msg) {
   // transform from odom frame to mapping frame
   // 获取里程计消息的坐标系
   std::string odom_frame = msg->header.frame_id;
   // 用于存储里程计位姿的tf::Pose对象
   tf::Pose tf_odom_pose;
   // 将ROS消息中的位姿转换为tf::Pose对象
   tf::poseMsgToTF(msg->pose.pose, tf_odom_pose);
   // 检查里程计坐标系与世界坐标系是否一致
   if (!FARUtil::IsSameFrameID(odom_frame, master_params_.world_frame)) {
     // 如果不一致，且开启了调试模式，输出警告信息
     if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: odom frame does NOT match with world frame!");
     // 用于存储从里程计坐标系到世界坐标系的变换
     tf::StampedTransform odom_to_world_tf_stamp;
     try
     {
       // 等待从世界坐标系到里程计坐标系的变换可用，最多等待1秒
       tf_listener_->waitForTransform(master_params_.world_frame, odom_frame, ros::Time(0), ros::Duration(1.0));
       // 查找从世界坐标系到里程计坐标系的变换
       tf_listener_->lookupTransform(master_params_.world_frame, odom_frame, ros::Time(0), odom_to_world_tf_stamp);
       // 将里程计位姿转换到世界坐标系下
       tf_odom_pose = odom_to_world_tf_stamp * tf_odom_pose;
     }
     catch (tf::TransformException ex){
       // 如果查找变换时出现异常，输出错误信息并返回
       ROS_ERROR("Tracking odom TF lookup: %s",ex.what());
       return;
     }
   }
   // 将转换后的位置信息赋值给机器人的位置
   robot_pos_.x = tf_odom_pose.getOrigin().getX(); 
   robot_pos_.y = tf_odom_pose.getOrigin().getY();
   robot_pos_.z = tf_odom_pose.getOrigin().getZ();
   // extract robot heading
   // 更新全局的机器人位置信息
   FARUtil::robot_pos = robot_pos_;
   // 用于存储机器人的欧拉角
   double roll, pitch, yaw;
   // 从tf::Pose对象中提取欧拉角
   tf_odom_pose.getBasis().getRPY(roll, pitch, yaw);
   // 根据偏航角计算机器人的航向向量
   robot_heading_ = Point3D(cos(yaw), sin(yaw), 0);
 
   // 检查是否是首次接收到里程计消息
   if (!is_odom_init_) {
     // system start time
     // 记录系统启动时间
     FARUtil::systemStartTime = ros::Time::now().toSec();
     // 记录地图原点为当前机器人位置
     FARUtil::map_origin = robot_pos_;
     // 更新地图处理器中的机器人位置
     map_handler_.UpdateRobotPosition(robot_pos_);
   }
 
   // 标记里程计信息已初始化
   is_odom_init_ = true;
 }
 
 /**
  * @brief 处理点云数据的函数
  * 
  * 该函数接收ROS格式的点云消息，将其转换为PCL格式的点云，并进行滤波、去除无效点和坐标转换等处理。
  * 
  * @param pc 输入的ROS格式点云消息的常量指针
  * @param cloudOut 输出的PCL格式点云的指针
  */
 void FARMaster::PrcocessCloud(const sensor_msgs::PointCloud2ConstPtr& pc,
                              const PointCloudPtr& cloudOut) 
 {
   // 创建一个临时的PCL点云对象
   pcl::PointCloud<PCLPoint> temp_cloud;
   // 将ROS格式的点云消息转换为PCL格式的点云
   pcl::fromROSMsg(*pc, temp_cloud);
   // 清空输出点云，并将临时点云赋值给输出点云
   cloudOut->clear(), *cloudOut = temp_cloud;
   // 如果输出点云为空，直接返回
   if (cloudOut->empty()) return;
   // 对输出点云进行滤波处理，使用指定的体素尺寸
   FARUtil::FilterCloud(cloudOut, master_params_.voxel_dim);
   // 获取点云的坐标系
   std::string cloud_frame = pc->header.frame_id;
   // 从点云中移除NaN和Inf点
   FARUtil::RemoveNanInfPoints(cloudOut);
   // 检查点云的坐标系与世界坐标系是否一致
   if (!FARUtil::IsSameFrameID(cloud_frame, master_params_.world_frame)) {
     // 如果不一致，且开启了调试模式，输出警告信息
     if (FARUtil::IsDebug) ROS_WARN_ONCE("FARMaster: cloud frame does NOT match with world frame!");
     try
     {
       // 将点云的坐标系转换为世界坐标系
       FARUtil::TransformPCLFrame(cloud_frame, 
                                 master_params_.world_frame, 
                                 tf_listener_,
                                 cloudOut);
     }
     catch(tf::TransformException ex)
     {
       // 如果转换过程中出现异常，输出错误信息并返回
       ROS_ERROR("Tracking cloud TF lookup: %s",ex.what());
       return;
     }
   }
 }
 
 /**
  * @brief 扫描点云数据回调函数
  * 
  * 该函数用于处理传入的扫描点云数据。根据环境是否静态和里程计是否初始化来决定是否进行处理。
  */
 void FARMaster::ScanCallBack(const sensor_msgs::PointCloud2ConstPtr& scan_pc) {
   if (master_params_.is_static_env || !is_odom_init_) return; // 如果环境是静态或者里程计未初始化，则返回
   this->PrcocessCloud(scan_pc, FARUtil::cur_scan_cloud_); // 处理点云并存储当前扫描云
   scan_handler_.UpdateRobotPosition(robot_pos_); // 更新机器人位置
 }
 
 /**
  * @brief 地形局部点云数据回调函数
  * 
  * 该函数处理来自地形的点云数据。只有在环境不是静态时才会处理点云。
  */
 void FARMaster::TerrainLocalCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
   if (master_params_.is_static_env) return; // 如果环境为静态，直接返回
   this->PrcocessCloud(pc, local_terrain_ptr_); // 处理局部地形点云数据
   FARUtil::ExtractFreeAndObsCloud(local_terrain_ptr_, FARUtil::local_terrain_free_, FARUtil::local_terrain_obs_); // 提取可通行和障碍物云
 }
 
 /**
  * @brief 地形点云数据回调函数
  * 
  * 此函数处理完整的地形点云，对于姿态初始化后的状态执行必要的操作以更新地图网格及其内容。
  */
 void FARMaster::TerrainCallBack(const sensor_msgs::PointCloud2ConstPtr& pc) {
   if (!is_odom_init_) return; // 确保里程计已初始化
   // 更新地图网格中机器人的位置
   map_handler_.UpdateRobotPosition(FARUtil::robot_pos);
   
   if (!is_stop_update_) { // 持续更新环境
     this->PrcocessCloud(pc, temp_cloud_ptr_); // 处理收到的点云数据
     FARUtil::CropBoxCloud(temp_cloud_ptr_, robot_pos_, Point3D(master_params_.terrain_range,
                                                                master_params_.terrain_range,
                                                                FARUtil::kTolerZ)); // 裁剪云
     FARUtil::ExtractFreeAndObsCloud(temp_cloud_ptr_, temp_free_ptr_, temp_obs_ptr_); // 提取可通行和障碍物云
     
     if (!master_params_.is_static_env) { // 非静态环境下
       FARUtil::RemoveOverlapCloud(temp_obs_ptr_, FARUtil::stack_dyobs_cloud_, true); // 移除重叠的动态障碍物云
     }
     
     // 更新观察到的障碍物和可通行云至地图网格
     map_handler_.UpdateObsCloudGrid(temp_obs_ptr_);
     map_handler_.UpdateFreeCloudGrid(temp_free_ptr_);
     
     // 提取新出现的点
     FARUtil::ExtractNewObsPointCloud(temp_obs_ptr_,
                                      FARUtil::surround_obs_cloud_,
                                      FARUtil::cur_new_cloud_);
   } else { // 暂停环境更新
     temp_cloud_ptr_->clear(); // 清空临时 cloud
     FARUtil::cur_new_cloud_->clear(); // 清空当前新的点云
   }
 
   // 提取周围的自由点云并更新地形高度
   map_handler_.GetSurroundFreeCloud(FARUtil::surround_free_cloud_);
   map_handler_.UpdateTerrainHeightGrid(FARUtil::surround_free_cloud_, terrain_height_ptr_);
   
   // 更新周围的障碍物点云
   map_handler_.GetSurroundObsCloud(FARUtil::surround_obs_cloud_);
   
   // 提取动态障碍物
   FARUtil::cur_dyobs_cloud_->clear();
   if (!master_params_.is_static_env && !is_stop_update_) {
     this->ExtractDynamicObsFromScan(FARUtil::cur_scan_cloud_, 
                                     FARUtil::surround_obs_cloud_, 
                                     FARUtil::surround_free_cloud_, 
                                     FARUtil::cur_dyobs_cloud_);
     
     if (FARUtil::cur_dyobs_cloud_->size() > FARUtil::kDyObsThred) { // 检测到动态障碍模式
       if (FARUtil::IsDebug) ROS_WARN("FARMaster: dynamic obstacle detected, size: %ld", FARUtil::cur_dyobs_cloud_->size());
       
       FARUtil::InflateCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim, 1, true); // 膨胀动态障碍物云
       map_handler_.RemoveObsCloudFromGrid(FARUtil::cur_dyobs_cloud_); // 从网格中移除这些云
       FARUtil::RemoveOverlapCloud(FARUtil::surround_obs_cloud_, FARUtil::cur_dyobs_cloud_); // 去除重叠点
       FARUtil::FilterCloud(FARUtil::cur_dyobs_cloud_, master_params_.voxel_dim); // 滤波
       
       // 更新新的云数据
       *FARUtil::cur_new_cloud_ += *FARUtil::cur_dyobs_cloud_;
       FARUtil::FilterCloud(FARUtil::cur_new_cloud_, master_params_.voxel_dim); // 再次滤波
     }
     
     // 更新世界中的动态障碍物
     FARUtil::StackCloudByTime(FARUtil::cur_dyobs_cloud_, FARUtil::stack_dyobs_cloud_, FARUtil::kObsDecayTime);
   }
   
   // 创建和更新kd树
   FARUtil::StackCloudByTime(FARUtil::cur_new_cloud_, FARUtil::stack_new_cloud_, FARUtil::kNewDecayTime);
   FARUtil::UpdateKdTrees(FARUtil::stack_new_cloud_);
 
   // 如果有周围的障碍物产生过则标记点云已初始化
   if (!FARUtil::surround_obs_cloud_->empty()) is_cloud_init_ = true;
 
   /* 可视化云数据 */
   planner_viz_.VizPointCloud(new_PCL_pub_, FARUtil::stack_new_cloud_); // 可视化最新云
   planner_viz_.VizPointCloud(dynamic_obs_pub_, FARUtil::cur_dyobs_cloud_); // 可视化动态障碍物
   planner_viz_.VizPointCloud(surround_free_debug_, FARUtil::surround_free_cloud_); // 可视化周围自由空间
   planner_viz_.VizPointCloud(surround_obs_debug_,  FARUtil::surround_obs_cloud_); // 可视化周围障碍物
   planner_viz_.VizPointCloud(terrain_height_pub_, terrain_height_ptr_); // 可视化地形高度
   
   // 可视化地图网格
   PointStack neighbor_centers, occupancy_centers;
   map_handler_.GetNeighborCeilsCenters(neighbor_centers); // 获取邻居单元中心
   map_handler_.GetOccupancyCeilsCenters(occupancy_centers); // 获取占用单元中心
   planner_viz_.VizMapGrids(neighbor_centers, occupancy_centers, map_params_.cell_length, map_params_.cell_height); // 可视化所有单元
 
   // DEBUG可视化射线投射网格
   if (!master_params_.is_static_env) {
     scan_handler_.GridVisualCloud(scan_grid_ptr_, GridStatus::RAY); // 创建视觉网格
     planner_viz_.VizPointCloud(scan_grid_debug_, scan_grid_ptr_); // 可视化光栅网格
   }
 }
 
 /**
  * @brief 从扫描中提取动态障碍物
  * 
  * 旨在从给定的扫描、观察云和自由云中提取出动态障碍物。
  * 
  * @param scanCloudIn 输入的扫描点云
  * @param obsCloudIn 图中的障碍物点云
  * @param freeCloudIn 可通行的点云
  * @param dyObsCloudOut 输出的动态障碍物点云
  */
 void FARMaster::ExtractDynamicObsFromScan(const PointCloudPtr& scanCloudIn, 
                                           const PointCloudPtr& obsCloudIn,
                                           const PointCloudPtr& freeCloudIn,
                                           const PointCloudPtr& dyObsCloudOut)
 {
   scan_handler_.ReInitGrids(); // 重新初始化网格
   scan_handler_.SetCurrentScanCloud(scanCloudIn, freeCloudIn); // 设置当前的扫描云和自由区间
   scan_handler_.ExtractDyObsCloud(obsCloudIn, dyObsCloudOut); // 提取动态障碍物云
 }
 
 /**
  * @brief 目标回调函数
  * 
  * 此函数用于接收路径规划的目标，并在图形初始化后将其更新。确保目标在正确的坐标框架中，如果不在，则进行转换。
  * 
  * @param route_goal 输入的目标点，包含x、y、z及相关帧信息
  */
 void FARMaster::WaypointCallBack(const geometry_msgs::PointStamped& route_goal) {
   // 检查图是否已初始化
   if (!is_graph_init_) {
     if (FARUtil::IsDebug) ROS_WARN("FARMaster: wait for v-graph to init before sending any goals");
     return; // 如果未初始化则返回
   }
 
   // 创建目标点
   Point3D goal_p(route_goal.point.x, route_goal.point.y, route_goal.point.z);
   const std::string goal_frame = route_goal.header.frame_id; // 获取目标帧ID
 
   // 确保目标位于正确的世界框架，如果不在，则转换目标坐标系
   if (!FARUtil::IsSameFrameID(goal_frame, master_params_.world_frame)) {
     if (FARUtil::IsDebug) ROS_WARN_THROTTLE(1.0, "FARMaster: waypoint published is not on world frame!");
     FARUtil::TransformPoint3DFrame(goal_frame, master_params_.world_frame, tf_listener_, goal_p); // 坐标转换
   }
   
   graph_planner_.UpdateGoal(goal_p); // 更新目标至图形规划器
   FARUtil::Timer.start_time("Overall_executing", true); // 开始计时记录整体执行时间
   
   // 可视化原始目标点
   planner_viz_.VizPoint3D(goal_p, "original_goal", VizColor::RED, 1.5); // 将目标点可视化为红色
 }
 
 /* 为静态工具类点云指针分配内存 */
 PointCloudPtr  FARUtil::surround_obs_cloud_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 周围障碍物点云
 PointCloudPtr  FARUtil::surround_free_cloud_ = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 周围自由区域点云
 PointCloudPtr  FARUtil::stack_new_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 堆叠的新点云
 PointCloudPtr  FARUtil::cur_new_cloud_       = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 当前的新点云
 PointCloudPtr  FARUtil::cur_dyobs_cloud_     = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 当前动态障碍物点云
 PointCloudPtr  FARUtil::stack_dyobs_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 堆叠的动态障碍物点云
 PointCloudPtr  FARUtil::cur_scan_cloud_      = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 当前扫描点云
 PointCloudPtr  FARUtil::local_terrain_obs_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 局部地形障碍物点云
 PointCloudPtr  FARUtil::local_terrain_free_  = PointCloudPtr(new pcl::PointCloud<PCLPoint>()); // 局部地形自由空间点云
 PointKdTreePtr FARUtil::kdtree_new_cloud_    = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>()); // 新点云的kd树
 PointKdTreePtr FARUtil::kdtree_filter_cloud_ = PointKdTreePtr(new pcl::KdTreeFLANN<PCLPoint>()); // 滤波用kd树
 
 /* 初始化静态工具值 */
 const float FARUtil::kEpsilon = 1e-7; // 很小的容忍误差常量
 const float FARUtil::kINF     = std::numeric_limits<float>::max(); // 浮点数的最大值
 std::string FARUtil::worldFrameId; // 世界框架ID
 float   FARUtil::kAngleNoise; // 角度噪声
 Point3D FARUtil::robot_pos; // 机器人当前的位置
 Point3D FARUtil::odom_pos; // 里程计位置
 Point3D FARUtil::map_origin; // 地图的原点
 Point3D FARUtil::free_odom_p; // 可通行的里程计位置
 float   FARUtil::robot_dim; // 机器人的尺寸
 float   FARUtil::vehicle_height; // 车辆的高度
 float   FARUtil::kLeafSize; // kd树的叶子大小
 float   FARUtil::kHeightVoxel; // 高度体素
 float   FARUtil::kNavClearDist; // 导航清空距离
 float   FARUtil::kCellLength; // 网格单元长度
 float   FARUtil::kCellHeight; // 网格单元高度
 float   FARUtil::kNewPIThred; // 新产生点的阈值
 float   FARUtil::kSensorRange; // 传感器范围
 float   FARUtil::kMarginDist; // 边缘距离
 float   FARUtil::kMarginHeight; // 边缘高度
 float   FARUtil::kTerrainRange; // 地形范围
 float   FARUtil::kLocalPlanRange; // 本地规划范围
 float   FARUtil::kFreeZ; // 自由空间Z值
 float   FARUtil::kVizRatio; // 可视化比率
 double  FARUtil::systemStartTime; // 系统启动时间
 float   FARUtil::kObsDecayTime; // 障碍物衰减时间
 float   FARUtil::kNewDecayTime; // 新点云衰减时间
 float   FARUtil::kNearDist; // 最近距离
 float   FARUtil::kMatchDist; // 匹配距离
 float   FARUtil::kProjectDist; // 投影距离
 int     FARUtil::kDyObsThred; // 动态障碍物阈值
 int     FARUtil::KNewPointC; // 新出现点数量
 int     FARUtil::kObsInflate; // 障碍物膨胀量
 float   FARUtil::kTolerZ; // Z方向上的公差
 float   FARUtil::kAcceptAlign; // 接受对齐距离
 bool    FARUtil::IsStaticEnv; // 是否静态环境标识
 bool    FARUtil::IsDebug; // 调试模式标志
 bool    FARUtil::IsMultiLayer; // 多层标记
 TimeMeasure FARUtil::Timer; // 时间测量工具
 
 /* 全局图形 */
 DynamicGraphParams DynamicGraph::dg_params_; 
 NodePtrStack DynamicGraph::globalGraphNodes_; 
 std::size_t  DynamicGraph::id_tracker_; 
 std::unordered_map<std::size_t, NavNodePtr> DynamicGraph::idx_node_map_; 
 std::unordered_map<NavNodePtr, std::pair<int, std::unordered_set<NavNodePtr>>> DynamicGraph::out_contour_nodes_map_; 
 
 /* 初始化静态轮廓图变量 */
 CTNodeStack ContourGraph::polys_ctnodes_; 
 CTNodeStack ContourGraph::contour_graph_; 
 PolygonStack ContourGraph::contour_polygons_; 
 std::vector<PointPair> ContourGraph::global_contour_; 
 std::vector<PointPair> ContourGraph::unmatched_contour_; 
 std::vector<PointPair> ContourGraph::inactive_contour_; 
 std::vector<PointPair> ContourGraph::boundary_contour_; 
 std::vector<PointPair> ContourGraph::local_boundary_; 
 std::unordered_set<NavEdge, navedge_hash> ContourGraph::global_contour_set_; 
 std::unordered_set<NavEdge, navedge_hash> ContourGraph::boundary_contour_set_; 
 
 /* 初始化地形地图变量 */
 PointKdTreePtr MapHandler::kdtree_terrain_clould_; 
 std::vector<int> MapHandler::terrain_grid_occupy_list_; 
 std::vector<int> MapHandler::terrain_grid_traverse_list_; 
 std::unordered_set<int> MapHandler::neighbor_obs_indices_; 
 std::unordered_set<int> MapHandler::extend_obs_indices_; 
 std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_free_cloud_grid_; 
 std::unique_ptr<grid_ns::Grid<PointCloudPtr>> MapHandler::world_obs_cloud_grid_;
 std::unique_ptr<grid_ns::Grid<std::vector<float>>> MapHandler::terrain_height_grid_; 
 
 
 int main(int argc, char** argv) {
   ros::init(argc, argv, "far_planner_node"); // 初始化ROS节点
   FARMaster dp_node; // 创建FARMaster实例
   dp_node.Init(); // 初始化节点设置
   dp_node.Loop(); // 启动主循环
 }
 