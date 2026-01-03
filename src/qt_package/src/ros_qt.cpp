#include <QApplication>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/parameter_client.hpp"

using namespace std::chrono_literals;

class TurtleQtNode : public rclcpp::Node
{
public:
    TurtleQtNode() : Node("turtle_qt_node")
    {
        client_ = std::make_shared<rclcpp::AsyncParametersClient>(
            this, "turtlesim");

        timer_ = this->create_wall_timer(
            1s, std::bind(&TurtleQtNode::set_blue_background, this));
    }

private:
    void set_blue_background()
    {
        if (!client_->service_is_ready()) {
            RCLCPP_INFO(this->get_logger(), "Waiting for turtlesim...");
            return;
        }

        client_->set_parameters({
            rclcpp::Parameter("background_r", 0),
            rclcpp::Parameter("background_g", 0),
            rclcpp::Parameter("background_b", 255)
        });

        RCLCPP_INFO(this->get_logger(), "Blue background set");
        timer_->cancel();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<rclcpp::AsyncParametersClient> client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<TurtleQtNode>();

    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [&]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10);

    return app.exec();
}
