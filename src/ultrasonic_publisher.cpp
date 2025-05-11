#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <gpiod.h>

using namespace std::chrono_literals;

#define CONSUMER "ultrasonic_publisher"
#define TRIG_CHIP "/dev/gpiochip0"
#define TRIG_LINE 18
#define ECHO_LINE 21
#define MAX_DIST_M 3.0
#define DT 50ms

class UltrasonicPublisher : public rclcpp::Node {
public:
    UltrasonicPublisher()
    : Node("ultrasonic_publisher") {
        header_pub_ = this->create_publisher<std_msgs::msg::Header>("distance_header", 10);
        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);

        chip = gpiod_chip_open(TRIG_CHIP);
        trig = gpiod_chip_get_line(chip, TRIG_LINE);
        echo = gpiod_chip_get_line(chip, ECHO_LINE);

        gpiod_line_request_output(trig, CONSUMER, 0);
        gpiod_line_request_input(echo, CONSUMER);

        timer_ = this->create_wall_timer(DT, std::bind(&UltrasonicPublisher::publish_distance, this));
    }

    ~UltrasonicPublisher() {
        gpiod_line_release(trig);
        gpiod_line_release(echo);
        gpiod_chip_close(chip);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;

    struct gpiod_chip *chip;
    struct gpiod_line *trig;
    struct gpiod_line *echo;

    void publish_distance() {
        // Trigger pulse: 10us HIGH
        gpiod_line_set_value(trig, 0);
        rclcpp::sleep_for(2us);
        gpiod_line_set_value(trig, 1);
        rclcpp::sleep_for(10us);
        gpiod_line_set_value(trig, 0);

        // Wait for ECHO to go HIGH
        auto start = std::chrono::high_resolution_clock::now();
        while (gpiod_line_get_value(echo) == 0) {
            if (timeout(start, 5ms)) return;
        }

        // Measure ECHO HIGH duration
        auto pulse_start = std::chrono::high_resolution_clock::now();
        while (gpiod_line_get_value(echo) == 1) {
            if (timeout(pulse_start, 25ms)) return;  // beyond 4m
        }
        auto pulse_end = std::chrono::high_resolution_clock::now();

        double duration = std::chrono::duration<double>(pulse_end - pulse_start).count(); // seconds
        float distance = static_cast<float>(duration * 343.0 / 2.0); // meters

        auto header = std_msgs::msg::Header();
        header.stamp = this->get_clock()->now();

        auto msg = std_msgs::msg::Float32();
        msg.data = distance;

        header_pub_->publish(header);
        dist_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Distance: %.3f m", distance);
    }

    bool timeout(std::chrono::high_resolution_clock::time_point start, std::chrono::milliseconds limit) {
        return std::chrono::high_resolution_clock::now() - start > limit;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UltrasonicPublisher>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

