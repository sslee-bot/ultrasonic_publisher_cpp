#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/header.hpp>
#include <gpiod.h>
#include <time.h>

#define CONSUMER "ultrasonic_publisher"
#define TRIG_CHIP "/dev/gpiochip0"
#define TRIG_LINE 18
#define ECHO_LINE 21
#define MAX_DISTANCE_M 4.0
#define MEASUREMENT_INTERVAL 100ms

using namespace std::chrono_literals;

class UltrasonicPublisher : public rclcpp::Node {
public:
    UltrasonicPublisher()
    : Node("ultrasonic_publisher") {
        header_pub_ = this->create_publisher<std_msgs::msg::Header>("distance_header", 10);
        dist_pub_ = this->create_publisher<std_msgs::msg::Float32>("distance", 10);

        chip_ = gpiod_chip_open(TRIG_CHIP);
        trig_line_ = gpiod_chip_get_line(chip_, TRIG_LINE);
        echo_line_ = gpiod_chip_get_line(chip_, ECHO_LINE);

        gpiod_line_request_output(trig_line_, CONSUMER, 0);

	struct gpiod_line_request_config echo_config = {};
	echo_config.consumer = CONSUMER;
	echo_config.request_type = GPIOD_LINE_REQUEST_EVENT_BOTH_EDGES;
	echo_config.flags = 0;

        gpiod_line_request(echo_line_, &echo_config, 0);

        timer_ = this->create_wall_timer(MEASUREMENT_INTERVAL, std::bind(&UltrasonicPublisher::measure_and_publish, this));
    }

    ~UltrasonicPublisher() {
        gpiod_line_release(trig_line_);
        gpiod_line_release(echo_line_);
        gpiod_chip_close(chip_);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr header_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr dist_pub_;

    struct gpiod_chip *chip_;
    struct gpiod_line *trig_line_;
    struct gpiod_line *echo_line_;

    void measure_and_publish() {
        // Trigger pulse: 10us HIGH
        gpiod_line_set_value(trig_line_, 0);
        rclcpp::sleep_for(2us);
        gpiod_line_set_value(trig_line_, 1);
        rclcpp::sleep_for(10us);
        gpiod_line_set_value(trig_line_, 0);

        // Wait for rising and falling edge
        struct gpiod_line_event echo_rise, echo_fall;
        timespec timeout = {0, 300000000}; // 300ms timeout

	// Check for rising edge timeout or failure
	if (gpiod_line_event_wait(echo_line_, &timeout) <= 0 ||
	    gpiod_line_event_read(echo_line_, &echo_rise) != 0 ||
	    echo_rise.event_type != GPIOD_LINE_EVENT_RISING_EDGE) {
	    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Timeout or invalid rising edge");
	    return;
	}

	// Check for falling edge timeout or failure
	if (gpiod_line_event_wait(echo_line_, &timeout) <= 0 ||
	    gpiod_line_event_read(echo_line_, &echo_fall) != 0 ||
	    echo_fall.event_type != GPIOD_LINE_EVENT_FALLING_EDGE) {
	    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Timeout or invalid falling edge");
	    return;
	}
	
	double duration = time_diff_sec(echo_rise.ts, echo_fall.ts);
        float distance = static_cast<float>(duration * 343.0 / 2.0);  // meter

	// Out-of-range check
	if (distance > MAX_DISTANCE_M) {
	    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Distance out of range: %.3f m", distance);
	    return;
	}

        auto header = std_msgs::msg::Header();
        header.stamp = this->get_clock()->now();

        auto msg = std_msgs::msg::Float32();
        msg.data = distance;

        header_pub_->publish(header);
        dist_pub_->publish(msg);

        // RCLCPP_INFO(this->get_logger(), "Distance: %.3f m", distance);
    }

    double time_diff_sec(const timespec& start, const timespec& end) {
        return static_cast<double>(end.tv_sec - start.tv_sec) +
               static_cast<double>(end.tv_nsec - start.tv_nsec) * 1e-9;
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UltrasonicPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

