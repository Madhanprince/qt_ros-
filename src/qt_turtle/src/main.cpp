#include <QApplication>
#include <QWidget>
#include <QPainter>
#include <QTimer>

#include "rclcpp/rclcpp.hpp"

class TurtleWindow : public QWidget
{
public:
    TurtleWindow()
    {
        setWindowTitle("My Turtle");
        resize(500, 500);
    }

protected:
    void paintEvent(QPaintEvent *) override
    {
        QPainter painter(this);

        // Blue background
        painter.fillRect(rect(), QColor(0, 0, 255));

        // Draw turtle image
        QPixmap turtle(":/turtle.png");
        int x = (width() - turtle.width()) / 2;
        int y = (height() - turtle.height()) / 2;
        painter.drawPixmap(x, y, turtle);
    }
};

class RosNode : public rclcpp::Node
{
public:
    RosNode() : Node("qt_turtle_node") {}
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<RosNode>();

    TurtleWindow window;
    window.show();

    QTimer ros_timer;
    QObject::connect(&ros_timer, &QTimer::timeout, [&]() {
        rclcpp::spin_some(node);
    });
    ros_timer.start(10);

    int ret = app.exec();
    rclcpp::shutdown();
    return ret;
}
