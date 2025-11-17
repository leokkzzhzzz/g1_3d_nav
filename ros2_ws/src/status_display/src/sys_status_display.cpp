#include <QApplication>
#include <QLabel>
#include <QString>
#include <sstream>  // 补充stringstream头文件
#include "rclcpp/rclcpp.hpp"
#include "status_interfaces/msg/system_status.hpp"

using SystemStatus = status_interfaces::msg::SystemStatus;

class SysStatusDisplay : public rclcpp::Node {
public:
    SysStatusDisplay() : Node("sys_status_display") {
        // 创建订阅者
        subscription_ = this->create_subscription<SystemStatus>(
            "sys_status", 10, 
            [this](const SystemStatus::SharedPtr msg) -> void {
                label1_->setText(get_gstr_from_msg(msg));
            }
        );
        
        // 初始化标签并显示
        label1_ = new QLabel(get_gstr_from_msg(std::make_shared<SystemStatus>()));
        label1_->setWindowTitle("系统状态监控");  // 增加窗口标题
        label1_->resize(400, 300);  // 调整窗口大小
        label1_->show();
    }

private:
    // 将消息转换为显示字符串（修正为类的成员函数）
    QString get_gstr_from_msg(const SystemStatus::SharedPtr msg) {
        std::stringstream show_str;
        show_str << "===================== 系统状态可视化显示工具 =====================\n"
                 << "数据时间:\t" << msg->stamp.sec << "." << msg->stamp.nanosec << "\n"  // 补充纳秒
                 << "主机名:\t" << msg->host_name << "\n"  // 修正为"主机名"（原"用户名"不准确）
                 << "CPU使用率:\t" << msg->cpu_percent << "%\n"
                 << "内存使用率:\t" << msg->memory_percent << "%\n"
                 << "内存总大小:\t" << msg->memory_total << " MB\n"
                 << "剩余有效内存:\t" << msg->memory_available << " MB\n"
                 << "网络发送量:\t" << msg->net_sent << " MB\n"
                 << "网络接收量:\t" << msg->net_recv << " MB\n"
                 << "-------------------------------------------------------------";
        return QString::fromStdString(show_str.str());
    }

    rclcpp::Subscription<SystemStatus>::SharedPtr subscription_;
    QLabel* label1_;  // 显示标签
};

int main(int argc, char* argv[]) {
    // 初始化ROS 2和Qt
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    // 创建节点并启动spin线程
    auto node = std::make_shared<SysStatusDisplay>();
    std::thread spin_thread([&node]() { 
        rclcpp::spin(node); 
    });
    spin_thread.detach();  // 分离线程，避免阻塞Qt事件循环

    // 运行Qt应用
    int result = app.exec();

    // 清理资源
    rclcpp::shutdown();
    return result;
}
