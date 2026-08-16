// ============================================================================
// 文件名：main.cpp
// 用途：节点入口：初始化 ROS 节点，构造 FastLioLocalizationScQn 主对象，
//       用多线程 AsyncSpinner 驱动回调后阻塞等待退出。
// 结构：main() 单函数，流程为 init -> 构造主对象 -> spinner 旋转。
// ============================================================================

#include "fast_lio_localization_sc_qn.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fast_lio_localization_sc_qn_node");
    ros::NodeHandle nh_private("~");

    // 构造主对象：读取参数、加载离线地图、建立订阅/发布与匹配定时器
    FastLioLocalizationScQn FastLioLocalizationScQn_(nh_private);
    FastLioLocalizationScQn FastLioLocalizationScQn_(nh_private);

    // 3 线程异步旋转：订阅回调与匹配定时器可并行执行
    ros::AsyncSpinner spinner(3); // Use multi threads
    ros::AsyncSpinner spinner(3); // Use multi threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
