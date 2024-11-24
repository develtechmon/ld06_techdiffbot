#!/usr/bin/env python3
import rclpy.time
import smbus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu

# MPU6050 Registers
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
DEVICE_ADDRESS = 0x68

class MPU6050_Driver(Node):
    def __init__(self):
        super().__init__("mpu6050_driver")

        # I2C Interface
        self.is_connected_ = False
        self.init_i2c()

        # Define custom QoS profile with RELIABLE reliability
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # ROS 2 Interface
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile)
        self.imu_msg_ = Imu()
        #self.imu_msg_.header.frame_id = "base_footprint"

        # Publish frequency (in seconds)
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

    def timerCallback(self):
        try:
            if not self.is_connected_:
                self.init_i2c()
                if not self.is_connected_:
                    self.get_logger().warn("I2C connection not established. Skipping this iteration.")
                    return

            # Read accelerometer raw values
            acc_x = self.read_raw_data(ACCEL_XOUT_H)
            acc_y = self.read_raw_data(ACCEL_YOUT_H)
            acc_z = self.read_raw_data(ACCEL_ZOUT_H)

            # Read gyroscope raw values
            gyro_x = self.read_raw_data(GYRO_XOUT_H)
            gyro_y = self.read_raw_data(GYRO_YOUT_H)
            gyro_z = self.read_raw_data(GYRO_ZOUT_H)

            # Convert raw values to scaled values
            self.imu_msg_.linear_acceleration.x = acc_x / 1670.13
            self.imu_msg_.linear_acceleration.y = acc_y / 1670.13
            self.imu_msg_.linear_acceleration.z = acc_z / 1670.13
            self.imu_msg_.angular_velocity.x = gyro_x / 7509.55
            self.imu_msg_.angular_velocity.y = gyro_y / 7509.55
            self.imu_msg_.angular_velocity.z = gyro_z / 7509.55

            # Set timestamp
            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()

            # Publish IMU data
            self.imu_pub_.publish(self.imu_msg_)
            #self.get_logger().info("Published IMU data.")

        except OSError as e:
            self.is_connected_ = False
            self.get_logger().error(f"I2C communication error: {e}")

    def init_i2c(self):
        try:
            self.bus_ = smbus.SMBus(1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, SMPLRT_DIV, 7)
            self.bus_.write_byte_data(DEVICE_ADDRESS, PWR_MGMT_1, 1)
            self.bus_.write_byte_data(DEVICE_ADDRESS, CONFIG, 0)
            self.bus_.write_byte_data(DEVICE_ADDRESS, GYRO_CONFIG, 24)
            self.bus_.write_byte_data(DEVICE_ADDRESS, INT_ENABLE, 1)
            self.is_connected_ = True
            self.get_logger().info("I2C connection established.")
        except OSError as e:
            self.is_connected_ = False
            self.get_logger().error(f"Failed to initialize I2C: {e}")

    def read_raw_data(self, addr):
        try:
            # Read high and low bytes from the sensor
            high = self.bus_.read_byte_data(DEVICE_ADDRESS, addr)
            low = self.bus_.read_byte_data(DEVICE_ADDRESS, addr + 1)

            # Concatenate high and low bytes
            value = ((high << 8) | low)

            # Convert to signed value
            if value > 32768:
                value -= 65536
            return value
        except OSError as e:
            self.get_logger().error(f"Failed to read data from address {addr}: {e}")
            raise

def main():
    rclpy.init()
    mpu6050_driver = MPU6050_Driver()
    try:
        rclpy.spin(mpu6050_driver)
    except KeyboardInterrupt:
        pass
    finally:
        mpu6050_driver.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

