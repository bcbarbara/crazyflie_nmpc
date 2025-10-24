/*
 * MIT License
 *
 * Copyright (c) 2025 Barbara Barros Carlos, Tommaso Sartor
 *
 * This file is part of the crazyflie_nmpc project.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the “Software”), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <array>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

class CrazyflieImu : public rclcpp::Node {
  public:
    CrazyflieImu()
      : Node("crazyflie_imu") {
        RCLCPP_INFO(get_logger(), "Crazyflie IMU Node started");

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/crazyflie/imu", 10, std::bind(&CrazyflieImu::imuCallback, this, std::placeholders::_1));
    }

  private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        gyro_  = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        accel_ = {msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z};
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    std::array<double, 3> gyro_, accel_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CrazyflieImu>());
    rclcpp::shutdown();
    return 0;
}
