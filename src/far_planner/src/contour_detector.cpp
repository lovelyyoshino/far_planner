/*
 * 完整的轮廓检测流程，包括点云数据处理、图像生成、轮廓提取和过滤等功能。这些功能可以用于机器人导航、环境感知等领域，帮助机器人识别障碍物的轮廓信息。
 */

 #include "far_planner/contour_detector.h"

 // const static int BLUR_SIZE = 10;
 
 /***************************************************************************************/
 
 /** 
  * @brief 初始化轮廓检测器的参数和变量。
  *
  * @param params ContourDetectParams 类型的结构体参数，包含传感器范围、体素维度等设置。
  */
 void ContourDetector::Init(const ContourDetectParams& params) {
     cd_params_ = params; // 保存结构体参数到成员变量
     /* 分配点云指针内存 */
     new_corners_cloud_   = PointCloudPtr(new pcl::PointCloud<PCLPoint>());
     
     // 初始化投影 cv Mat 的尺寸
     MAT_SIZE = std::ceil(cd_params_.sensor_range * 2.0f / cd_params_.voxel_dim);
     if (MAT_SIZE % 2 == 0) MAT_SIZE ++; // 确保为奇数
     MAT_RESIZE = MAT_SIZE * (int)cd_params_.kRatio;
     CMAT = MAT_SIZE / 2, CMAT_RESIZE = MAT_RESIZE / 2; // 中心位置
     img_mat_ = cv::Mat::zeros(MAT_SIZE, MAT_SIZE, CV_32FC1); // 初始化图像矩阵为零
 
     img_counter_ = 0; // 初始图像计数器
     odom_node_ptr_ = NULL; // 里程计节点指针初始化为空
     refined_contours_.clear(), refined_hierarchy_.clear(); // 清空精细化轮廓和层次信息
 
     DIST_LIMIT = cd_params_.kRatio * 1.5f; // 设置距离限制
     ALIGN_ANGLE_COS = cos(FARUtil::kAcceptAlign / 2.0f); // 接受对齐角的余弦值
     VOXEL_DIM_INV = 1.0f / cd_params_.voxel_dim; // 体素维度的倒数
 }
 
 /** 
  * @brief 构建地形图并提取轮廓信息。
  *
  * @param odom_node_ptr 里程计节点指针，用于更新位置信息。
  * @param surround_cloud 环境点云数据，提供周围障碍物的信息。
  * @param realworld_contour 存放提取出来的真实世界轮廓。
  */
 void ContourDetector::BuildTerrainImgAndExtractContour(const NavNodePtr& odom_node_ptr,
                                                        const PointCloudPtr& surround_cloud,
                                                        std::vector<PointStack>& realworl_contour) {
     CVPointStack cv_corners; // 用于存放经过处理的轮廓点
     PointStack corner_vec; // 向量用于后续处理
     this->UpdateOdom(odom_node_ptr); // 更新位置
     this->ResetImgMat(img_mat_); // 重置图像矩阵
     this->UpdateImgMatWithCloud(surround_cloud, img_mat_); // 使用点云更新图像矩阵
     this->ExtractContourFromImg(img_mat_, refined_contours_, realworl_contour); // 从图像中提取轮廓
 }
 
 /** 
  * @brief 用给定的点云数据更新图像矩阵。
  *
  * @param pc 输入的点云数据。
  * @param img_mat 要更新的图像矩阵。
  */
 void ContourDetector::UpdateImgMatWithCloud(const PointCloudPtr& pc, cv::Mat& img_mat) {
     int row_idx, col_idx, inf_row, inf_col; // 行列索引
     const std::vector<int> inflate_vec{-1, 0, 1}; // 膨胀操作使用的偏移向量
     for (const auto& pcl_p : pc->points) { // 遍历所有点云中的点
         this->PointToImgSub(pcl_p, odom_pos_, row_idx, col_idx, false, false); // 将3D点转为图像坐标
         if (!this->IsIdxesInImg(row_idx, col_idx)) continue; // 检查是否在图像范围内
         for (const auto& dr : inflate_vec) { // 对每个点进行膨胀处理
             for (const auto& dc : inflate_vec) {
                 inf_row = row_idx + dr, inf_col = col_idx + dc; // 邻域的行列索引
                 if (this->IsIdxesInImg(inf_row, inf_col)) { // 再次检查邻域是否在图像范围内
                     img_mat.at<float>(inf_row, inf_col) += 1.0; // 增加对应单元格的亮度值
                 }
             }
         }
     }
     if (!FARUtil::IsStaticEnv) { // 如果不是静态环境，则应用阈值处理
         cv::threshold(img_mat, img_mat, cd_params_.kThredValue, 1.0, cv::ThresholdTypes::THRESH_BINARY);
     }
     if (cd_params_.is_save_img) this->SaveCurrentImg(img_mat); // 可选保存当前图像
 }
 
 /** 
  * @brief 调整并模糊化输入图像。
  *
  * @param img 输入图像。
  * @param Rimg 输出调整后的图像。
  */
 void ContourDetector::ResizeAndBlurImg(const cv::Mat& img, cv::Mat& Rimg) {
     img.convertTo(Rimg, CV_8UC1, 255); // 转换图像格式并归一化
     cv::resize(Rimg, Rimg, cv::Size(), cd_params_.kRatio, cd_params_.kRatio, 
                cv::InterpolationFlags::INTER_LINEAR); // 按比例缩放图像
     //cv::morphologyEx(Rimg, Rimg, cv::MORPH_OPEN, getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3))); // 开运算（可选）
     cv::boxFilter(Rimg, Rimg, -1, cv::Size(cd_params_.kBlurSize, cd_params_.kBlurSize), cv::Point2i(-1, -1), false); // 执行均值模糊
     //cv::morphologyEx(Rimg, Rimg, cv::MORPH_CLOSE, getStructuringElement(cv::MORPH_RECT, cv::Size(cd_params_.kBlurSize+2, cd_params_.kBlurSize+2))); // 闭运算（可选）  
 }
 
 /** 
  * @brief 从图像中提取轮廓并转换为现实世界坐标。
  *
  * @param img 输入图像。
  * @param img_contours 输出找到的轮廓。
  * @param realworld_contour 转换得到的真实世界轮廓。
  */
 void ContourDetector::ExtractContourFromImg(const cv::Mat& img,
                                             std::vector<CVPointStack>& img_contours, 
                                             std::vector<PointStack>& realworld_contour)
 {
     cv::Mat Rimg; // 调整后的图像容器
     this->ResizeAndBlurImg(img, Rimg); // 调整大小并模糊处理图像
     this->ExtractRefinedContours(Rimg, img_contours); // 提取精细化轮廓
     this->ConvertContoursToRealWorld(img_contours, realworld_contour); // 转换为真实世界坐标
 }
 
 /** 
  * @brief 将轮廓从图像坐标系转换到真实世界坐标系。
  *
  * @param ori_contours 原始轮廓。
  * @param realWorld_contours 转换后的真实世界轮廓。
  */
 void ContourDetector::ConvertContoursToRealWorld(const std::vector<CVPointStack>& ori_contours,
                                                  std::vector<PointStack>& realWorld_contours)
 {
     const std::size_t C_N = ori_contours.size(); // 梯度数量
     realWorld_contours.clear(), realWorld_contours.resize(C_N); // 清除并调整输出轮廓大小
     for (std::size_t i=0; i<C_N; i++) { // 循环遍历每个轮廓
         const CVPointStack cv_contour = ori_contours[i]; // 获取当前轮廓
         this->ConvertCVToPoint3DVector(cv_contour, realWorld_contours[i], true); // 转换为3D点集合
     }
 }
 
 /** 
  * @brief 显示带有轮廓点的图像。
  *
  * @param img_mat 要显示的图像矩阵。
  * @param pc 点云数据，用于绘制额外的信息。
  */
 void ContourDetector::ShowCornerImage(const cv::Mat& img_mat,
                                      const PointCloudPtr& pc) {
     cv::Mat dst = cv::Mat::zeros(MAT_RESIZE, MAT_RESIZE, CV_8UC3); // 创建一个少量初始项的目标图像
     const int circle_size = (int)(cd_params_.kRatio*1.5); // 圆圈大小
     for (std::size_t i=0; i<pc->size(); i++) { // 遍历点云，将其绘制到图像上
         cv::Point2f cv_p = this->ConvertPoint3DToCVPoint(pc->points[i], odom_pos_, true); // 从3D转换成2D
         cv::circle(dst, cv_p, circle_size, cv::Scalar(128,128,128), -1); // 在相应位置画圆
     }
     // 显示自由里程计的位置
     cv::circle(dst, free_odom_resized_, circle_size, cv::Scalar(0,0,255), -1);
     std::vector<std::vector<cv::Point2i>> round_contours; // 圆润的轮廓列表
     this->RoundContours(refined_contours_, round_contours); // 使轮廓变圆滑
     for(std::size_t idx=0; idx<round_contours.size(); idx++) {
         cv::Scalar color(rand()&255, rand()&255, rand()&255 ); // 随机颜色
         cv::drawContours(dst, round_contours, idx, color, cv::LineTypes::LINE_4); // 绘制轮廓
     }
     cv::imshow("Obstacle Cloud Image", dst); // 显示由障碍云生成的图像
     cv::waitKey(30); // 等待键盘事件以便查看图像
 }
 
 /** 
  * @brief 从输入图像中提取精细化轮廓。
  *
  * @param imgIn 输入图像。
  * @param refined_contours 输出的精细化轮廓。
  */
 void ContourDetector::ExtractRefinedContours(const cv::Mat& imgIn,
                                             std::vector<CVPointStack>& refined_contours) 
 { 
     std::vector<std::vector<cv::Point2i>> raw_contours; // 原始轮廓列表
     refined_contours.clear(), refined_hierarchy_.clear(); // 清空现有轮廓和层次
     cv::findContours(imgIn, raw_contours, refined_hierarchy_, // 找到边界轮廓
                      cv::RetrievalModes::RETR_TREE, 
                      cv::ContourApproximationModes::CHAIN_APPROX_TC89_L1);
                      
     refined_contours.resize(raw_contours.size()); // 配合原始轮廓数量调整输出
     for (std::size_t i=0; i<raw_contours.size(); i++) {
         // 使用Ramer-Douglas-Peucker算法简化轮廓
         cv::approxPolyDP(raw_contours[i], refined_contours[i], DIST_LIMIT, true); 
     }
     this->TopoFilterContours(refined_contours); // 顶部过滤轮廓
     this->AdjecentDistanceFilter(refined_contours); // 相邻距离过滤
 }
 
 /** 
  * @brief 对轮廓进行相邻距离过滤，以删除相邻重叠部分。
  *
  * @param contoursInOut 输入和输出的轮廓集合。
  */
 void ContourDetector::AdjecentDistanceFilter(std::vector<CVPointStack>& contoursInOut) {
     /* 过滤掉与邻近轮廓重合的顶点 */
     std::unordered_set<int> remove_idxs; // 需删除的索引集合
     for (std::size_t i=0; i<contoursInOut.size(); i++) { 
         const auto c = contoursInOut[i]; // 当前轮廓
         const std::size_t c_size = c.size(); // 当前轮廓的大小
         std::size_t refined_idx = 0; // 缩减后的顶点索引
         for (std::size_t j=0; j<c_size; j++) {
             cv::Point2f p = c[j]; // 当前顶点
             if (refined_idx < 1 || FARUtil::PixelDistance(contoursInOut[i][refined_idx-1], p) > DIST_LIMIT) {
                 /** 减少墙体节点 */ 
                 RemoveWallConnection(contoursInOut[i], p, refined_idx); // 删除多余连接
                 contoursInOut[i][refined_idx] = p; // 存储当前顶点
                 refined_idx++; // 增加有效顶点计数
             }
         }
         /** 减少墙体节点 */
         RemoveWallConnection(contoursInOut[i], contoursInOut[i][0], refined_idx); // 在闭合处再次处理
         contoursInOut[i].resize(refined_idx); // 调整轮廓大小
         if (refined_idx > 1 && FARUtil::PixelDistance(contoursInOut[i].front(), contoursInOut[i].back()) < DIST_LIMIT) {
             contoursInOut[i].pop_back(); // 移除尾部重叠
         }
         if (contoursInOut[i].size() < 3) remove_idxs.insert(i); // 小于3的轮廓将被删除
     }
     if (!remove_idxs.empty()) { // 如果存在要删除的区域
         std::vector<CVPointStack> temp_contours = contoursInOut; // 临时存储当前轮廓
         contoursInOut.clear(); 
         for (int i=0; i<temp_contours.size(); i++) {
             if (remove_idxs.find(i) != remove_idxs.end()) continue; // 跳过需要删除的索引
             contoursInOut.push_back(temp_contours[i]); // 否则添加到结果集中
         }
     }
 }
 
 /** 
  * @brief 执行拓扑过滤，从而去除内部轮廓。
  *
  * @param contoursInOut 输入和输出的轮廓集合。
  */
 void ContourDetector::TopoFilterContours(std::vector<CVPointStack>& contoursInOut) {
     std::unordered_set<int> remove_idxs; // 需删除的索引集合
     for (int i=0; i<contoursInOut.size(); i++) {
         if (remove_idxs.find(i) != remove_idxs.end()) continue; // 跳过已标记为删除的轮廓
         const auto poly = contoursInOut[i]; // 获取轮廓
         if (poly.size() < 3) { // 若轮廓小于3则无效
             remove_idxs.insert(i);
         } else if (!FARUtil::PointInsideAPoly(poly, free_odom_resized_)) { // 检查是否在免费空间内
             InternalContoursIdxs(refined_hierarchy_, i, remove_idxs); // 获取内部轮廓索引
         }
     }
     if (!remove_idxs.empty()) { // 类似前面的逻辑：如果需要删除
         std::vector<CVPointStack> temp_contours = contoursInOut; // 临时存储
         contoursInOut.clear();
         for (int i=0; i<temp_contours.size(); i++) {
             if (remove_idxs.find(i) != remove_idxs.end()) continue; // 跳过需要删除的
             contoursInOut.push_back(temp_contours[i]); // 添加到结果集
         }
     }
 }