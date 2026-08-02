// ros2_control SystemInterface for the UGV (ugv_beast and siblings).
//
// The ESP32 sub-controller speaks newline-delimited JSON over a single UART. It
// accepts *body* velocity commands ({"T":"13","X":<v>,"Z":<w>}) and streams a
// telemetry frame ({"T":1001, ...}) carrying IMU, magnetometer, wheel encoders
// and battery voltage. Because that one link is single-reader, this component is
// the sole owner of the port: it replaces both ugv_driver.py and ugv_bringup.py.
//
//   * Drive wheels  -> velocity command + position/velocity state (diff_drive_controller)
//   * IMU           -> orientation/ang_vel/lin_accel state         (imu_sensor_broadcaster)
//   * mag / voltage -> published directly by an internal aux node on the legacy topics
//   * pan-tilt / LED-> the aux node subscribes the legacy command topics and queues
//                      the corresponding JSON onto the shared serial write path.
#ifndef UGV_HARDWARE__UGV_SYSTEM_HPP_
#define UGV_HARDWARE__UGV_SYSTEM_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ugv_hardware/serial_port.hpp"

namespace ugv_hardware
{

// Latest decoded telemetry frame (raw firmware units, exactly as the legacy
// Python bridge consumed them). Guarded by UgvSystemHardware::telem_mutex_.
struct Telemetry
{
  double ax{0.0}, ay{0.0}, az{0.0};  // accelerometer raw counts
  double gx{0.0}, gy{0.0}, gz{0.0};  // gyroscope raw counts
  double mx{0.0}, my{0.0}, mz{0.0};  // magnetometer raw counts
  double odl{0.0}, odr{0.0};         // cumulative wheel travel (firmware: metres * 100)
  double voltage{0.0};               // battery (firmware: volts * 100)
  bool valid{false};                 // at least one T:1001 frame seen
};

class UgvSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UgvSystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // ---- interface storage (addresses handed to ros2_control must stay stable) ----
  std::vector<std::pair<std::string, std::string>> cmd_ifaces_;    // (component, interface)
  std::vector<double> cmd_values_;
  std::vector<std::pair<std::string, std::string>> state_ifaces_;
  std::vector<double> state_values_;
  std::unordered_map<std::string, std::size_t> cmd_index_;         // "comp/iface" -> idx
  std::unordered_map<std::string, std::size_t> state_index_;

  bool has_state(const std::string & key) const;
  bool has_cmd(const std::string & key) const;
  double & state_ref(const std::string & key);
  double & cmd_ref(const std::string & key);

  // ---- serial link, shared write queue and reader thread ----
  SerialPort serial_;
  std::thread reader_thread_;
  std::thread writer_thread_;
  std::atomic<bool> running_{false};

  // Link-liveness tracking so a dead serial link propagates to ros2_control.
  // last_telem_ns_: steady_clock nanoseconds of the last decoded T:1001 frame
  // (seeded at on_activate to grant a startup grace window). write_failed_: set
  // by writer_loop when ::write reports a dead fd. Both are polled by
  // read()/write(), which return ERROR so the controller_manager deactivates.
  std::atomic<int64_t> last_telem_ns_{0};
  std::atomic<bool> write_failed_{false};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // for throttled fault logging

  std::mutex telem_mutex_;
  Telemetry telem_;

  std::mutex write_mutex_;
  std::condition_variable write_cv_;
  std::deque<std::string> write_queue_;  // raw lines incl. trailing '\n'

  void reader_loop();
  void writer_loop();
  void enqueue_line(const std::string & line);  // thread-safe; appends '\n'

  // ---- aux ROS node (mag/voltage publish + pan-tilt/LED subscribe) ----
  class AuxNode;
  std::shared_ptr<AuxNode> aux_node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr aux_executor_;
  std::thread aux_thread_;

  // ---- parameters (from the <hardware> block, with sane defaults) ----
  std::string serial_device_;
  int baud_{115200};
  double serial_timeout_{0.5};        // seconds without telemetry -> link dead
  double wheel_radius_{0.025};       // metres  (wheel diameter 0.05)
  double wheel_separation_{0.175};   // metres
  std::vector<std::string> left_wheel_joints_;
  std::vector<std::string> right_wheel_joints_;

  // ---- gyro zero-rate-offset (ZRO) calibration ----
  // The ESP32 gyro has a small constant bias (~0.02 rad/s on z). Left uncorrected
  // it integrates into an endless slow yaw drift in the EKF, since the downstream
  // complementary filter only de-biases orientation, not the angular_velocity we
  // publish. On activation we average a window of samples while the robot is held
  // still, then subtract the mean from every reading. The robot MUST be stationary
  // for gyro_cal_samples_ reads (~4 s at the 50 Hz control rate) after bringup.
  bool gyro_calibration_{true};       // <hardware> param: enable ZRO calibration
  int gyro_cal_samples_{200};         // <hardware> param: samples to average
  bool gyro_calibrated_{false};       // true once the bias has been captured
  int gyro_cal_count_{0};             // samples accumulated so far
  double gyro_bias_x_{0.0};           // measured bias (raw counts), subtracted pre-scale
  double gyro_bias_y_{0.0};
  double gyro_bias_z_{0.0};
  double gyro_cal_sum_x_{0.0};        // running sums during the calibration window
  double gyro_cal_sum_y_{0.0};
  double gyro_cal_sum_z_{0.0};
  double gyro_cal_sq_z_{0.0};         // sum of squares of z, for a movement sanity check

  // ---- bookkeeping for velocity estimation from cumulative encoder travel ----
  bool first_read_{true};
  double prev_left_pos_{0.0};
  double prev_right_pos_{0.0};
};

}  // namespace ugv_hardware

#endif  // UGV_HARDWARE__UGV_SYSTEM_HPP_
