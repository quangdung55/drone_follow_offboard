# Hướng Dẫn Chuyển Từ ROS2 Sang Qt

## 📋 Tổng Quan

Dự án hiện tại của bạn có **kiến trúc lý tưởng** để chuyển sang Qt vì đã tách biệt:
- **Core Library** (`drone_offboard_core_lib`): Thuật toán thuần túy, CHỈ phụ thuộc Eigen
- **ROS2 Wrapper** (`drone_offboard_wrapper_lib`): Lớp giao tiếp ROS2

### ✅ Ưu Điểm Kiến Trúc Hiện Tại
```
┌─────────────────────────────────────────┐
│  ROS2 Node (smart_follow_node.cpp)      │  ← Chỉ lo pub/sub
├─────────────────────────────────────────┤
│  Core Algorithms (KHÔNG phụ thuộc ROS)  │  ← Giữ nguyên được!
│  - TargetEstimator                      │
│  - FollowController                     │
│  - YawController                        │
│  - Safety Validators                    │
└─────────────────────────────────────────┘
```

---

## 🎯 Chiến Lược Chuyển Đổi

### Phương Án 1: Tích Hợp Qt Với ROS2 (Khuyến Nghị)
**Tình huống**: Vẫn cần ROS2 cho giao tiếp drone, nhưng muốn GUI Qt
- ✅ Giữ nguyên kiến trúc hiện tại
- ✅ Thêm Qt GUI để giám sát và điều khiển
- ✅ Ít rủi ro nhất, triển khai nhanh

### Phương Án 2: Thay Thế Hoàn Toàn ROS2 Bằng Qt
**Tình huống**: Không cần ROS2 nữa, muốn app độc lập
- ⚠️ Phải viết lại lớp giao tiếp MAVLink/UDP
- ⚠️ Mất khả năng tương tác với ROS ecosystem
- ✅ Ứng dụng nhẹ hơn, dễ phân phối

---

## 🚀 PHƯƠNG ÁN 1: Tích Hợp Qt + ROS2 (KHUYẾN NGHỊ)

### Bước 1: Cài Đặt Qt

```bash
# Ubuntu/Debian
sudo apt install qt6-base-dev qt6-charts-dev qt6-multimedia-dev

# Hoặc cài Qt Creator
wget https://download.qt.io/official_releases/online_installers/qt-unified-linux-x64-online.run
chmod +x qt-unified-linux-x64-online.run
./qt-unified-linux-x64-online.run
```

### Bước 2: Cập Nhật CMakeLists.txt

Thêm vào `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(drone_offboard)

# ... existing ROS2 packages ...

# Thêm Qt6
find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# ==============================================================================
# 1. CORE LIBRARY (Không thay đổi - đã tách biệt ROS)
# ==============================================================================
add_library(${PROJECT_NAME}_core_lib SHARED
  src/core/control/ap_control.cpp
  src/core/control/ap_follow.cpp
  src/core/control/follow_controller.cpp
  src/core/control/yaw_controller.cpp
  src/core/estimator/target_estimator.cpp
  src/core/math/JitterCorrection.cpp
  src/core/safety/safety.cpp
)

target_include_directories(${PROJECT_NAME}_core_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(${PROJECT_NAME}_core_lib PUBLIC Eigen3::Eigen)

# ==============================================================================
# 2. ROS2 WRAPPER (Giữ nguyên cho ROS node)
# ==============================================================================
add_library(${PROJECT_NAME}_wrapper_lib SHARED
  src/ros2/smart_follow_node.cpp
)

target_include_directories(${PROJECT_NAME}_wrapper_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}_wrapper_lib ${PROJECT_NAME}_core_lib)

ament_target_dependencies(${PROJECT_NAME}_wrapper_lib
  rclcpp sensor_msgs geometry_msgs geographic_msgs
  std_msgs diagnostic_msgs tf2 tf2_geometry_msgs ardupilot_msgs
)

# ==============================================================================
# 3. QT ADAPTER (NEW! - Giao tiếp với Qt GUI)
# ==============================================================================
add_library(${PROJECT_NAME}_qt_adapter SHARED
  src/qt/qt_ros_bridge.cpp
  src/qt/drone_telemetry.cpp
)

target_include_directories(${PROJECT_NAME}_qt_adapter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link Qt Adapter với Core và ROS Wrapper
target_link_libraries(${PROJECT_NAME}_qt_adapter
  ${PROJECT_NAME}_core_lib
  ${PROJECT_NAME}_wrapper_lib
  Qt6::Core
)

ament_target_dependencies(${PROJECT_NAME}_qt_adapter
  rclcpp sensor_msgs geometry_msgs
)

# ==============================================================================
# 4. QT GUI APPLICATION (NEW!)
# ==============================================================================
set(GUI_SOURCES
  src/qt/main_window.cpp
  src/qt/main_window.ui
  src/qt/main.cpp
)

add_executable(drone_gui ${GUI_SOURCES})

target_link_libraries(drone_gui
  ${PROJECT_NAME}_qt_adapter
  Qt6::Widgets
  Qt6::Charts
)

# ==============================================================================
# 5. ROS2 NODE (Giữ nguyên)
# ==============================================================================
add_executable(smart_follow_node src/smart_follow_main.cpp)
target_link_libraries(smart_follow_node ${PROJECT_NAME}_wrapper_lib)

# ==============================================================================
# 6. INSTALL
# ==============================================================================
install(TARGETS smart_follow_node drone_gui
  DESTINATION lib/${PROJECT_NAME}
)
```

### Bước 3: Tạo Qt-ROS Bridge

Tạo file `include/drone_offboard/qt/qt_ros_bridge.hpp`:

```cpp
#pragma once

#include <QObject>
#include <QTimer>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "drone_offboard/core/types/drone_state.hpp"
#include "drone_offboard/core/types/target_state.hpp"

namespace drone_follow {
namespace qt {

/**
 * @brief Bridge giữa Qt GUI và ROS2 Node
 * 
 * Nhiệm vụ:
 * - Chuyển ROS messages thành Qt signals
 * - Chuyển Qt commands thành ROS messages
 * - Quản lý ROS node trong Qt event loop
 */
class QtRosBridge : public QObject {
    Q_OBJECT

public:
    explicit QtRosBridge(QObject* parent = nullptr);
    ~QtRosBridge();

    void init(int argc, char** argv);
    void start();
    void stop();

signals:
    // Signals gửi tới Qt GUI
    void droneStateUpdated(const drone_follow::core::DroneState& state);
    void targetStateUpdated(const drone_follow::core::TargetState& state);
    void followStatusChanged(const QString& status);
    void errorOccurred(const QString& error);

public slots:
    // Slots nhận commands từ Qt GUI
    void startFollow();
    void stopFollow();
    void setFollowDistance(double distance);
    void setFollowHeight(double height);
    void emergencyStop();

private:
    void spin_ros();
    
    std::shared_ptr<rclcpp::Node> node_;
    std::shared_ptr<QTimer> ros_timer_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_drone_gps_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_drone_pose_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_velocity_cmd_;
    
    // Callbacks
    void drone_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
    void drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};

} // namespace qt
} // namespace drone_follow
```

Tạo file `src/qt/qt_ros_bridge.cpp`:

```cpp
#include "drone_offboard/qt/qt_ros_bridge.hpp"

using namespace drone_follow::qt;
using namespace std::chrono_literals;

QtRosBridge::QtRosBridge(QObject* parent) 
    : QObject(parent) {
}

QtRosBridge::~QtRosBridge() {
    stop();
}

void QtRosBridge::init(int argc, char** argv) {
    rclcpp::init(argc, argv);
    node_ = std::make_shared<rclcpp::Node>("qt_ros_bridge");
    
    // Setup subscribers
    sub_drone_gps_ = node_->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/ap/gps_global_origin/raw",
        rclcpp::SensorDataQoS(),
        std::bind(&QtRosBridge::drone_gps_callback, this, std::placeholders::_1)
    );
    
    sub_drone_pose_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/ap/pose/filtered",
        10,
        std::bind(&QtRosBridge::drone_pose_callback, this, std::placeholders::_1)
    );
    
    // Setup publishers
    pub_velocity_cmd_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/ap/cmd_vel",
        10
    );
}

void QtRosBridge::start() {
    // Spin ROS trong Qt timer để không block GUI
    ros_timer_ = std::make_shared<QTimer>(this);
    connect(ros_timer_.get(), &QTimer::timeout, this, &QtRosBridge::spin_ros);
    ros_timer_->start(10); // 100Hz
}

void QtRosBridge::stop() {
    if (ros_timer_) {
        ros_timer_->stop();
    }
    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}

void QtRosBridge::spin_ros() {
    if (rclcpp::ok()) {
        rclcpp::spin_some(node_);
    }
}

void QtRosBridge::drone_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg) {
    // Convert ROS message to core type
    drone_follow::core::DroneState state;
    state.gps.latitude = msg->latitude;
    state.gps.longitude = msg->longitude;
    state.gps.altitude = msg->altitude;
    
    // Emit Qt signal
    emit droneStateUpdated(state);
}

void QtRosBridge::drone_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Handle pose update
    emit followStatusChanged("Following...");
}

void QtRosBridge::startFollow() {
    // Gửi lệnh start follow tới ROS
    emit followStatusChanged("Follow Started");
}

void QtRosBridge::stopFollow() {
    // Gửi lệnh stop follow tới ROS
    emit followStatusChanged("Follow Stopped");
}

void QtRosBridge::setFollowDistance(double distance) {
    // Gọi ROS service để set parameter
}

void QtRosBridge::emergencyStop() {
    geometry_msgs::msg::TwistStamped stop_cmd;
    stop_cmd.twist.linear.x = 0;
    stop_cmd.twist.linear.y = 0;
    stop_cmd.twist.linear.z = 0;
    pub_velocity_cmd_->publish(stop_cmd);
    
    emit followStatusChanged("Emergency Stop!");
}
```

### Bước 4: Tạo Qt Main Window

Tạo file `src/qt/main_window.hpp`:

```cpp
#pragma once

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QtCharts>
#include "drone_offboard/qt/qt_ros_bridge.hpp"

namespace drone_follow {
namespace qt {

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void on_start_clicked();
    void on_stop_clicked();
    void on_emergency_clicked();
    void update_drone_state(const drone_follow::core::DroneState& state);
    void update_status(const QString& status);

private:
    void setup_ui();
    void setup_connections();
    
    // Qt Widgets
    QPushButton* btn_start_;
    QPushButton* btn_stop_;
    QPushButton* btn_emergency_;
    QLabel* lbl_status_;
    QLabel* lbl_gps_;
    QLabel* lbl_altitude_;
    QLineEdit* edit_follow_dist_;
    
    // Charts
    QChartView* chart_view_;
    QLineSeries* series_altitude_;
    
    // ROS Bridge
    std::unique_ptr<QtRosBridge> ros_bridge_;
};

} // namespace qt
} // namespace drone_follow
```

Tạo file `src/qt/main_window.cpp`:

```cpp
#include "main_window.hpp"

using namespace drone_follow::qt;

MainWindow::MainWindow(QWidget* parent) 
    : QMainWindow(parent) {
    
    setup_ui();
    
    // Khởi tạo ROS bridge
    ros_bridge_ = std::make_unique<QtRosBridge>(this);
    
    setup_connections();
}

MainWindow::~MainWindow() = default;

void MainWindow::setup_ui() {
    setWindowTitle("Drone Follow Control");
    resize(800, 600);
    
    QWidget* central = new QWidget(this);
    QVBoxLayout* layout = new QVBoxLayout(central);
    
    // Status labels
    lbl_status_ = new QLabel("Status: Idle", this);
    lbl_gps_ = new QLabel("GPS: N/A", this);
    lbl_altitude_ = new QLabel("Altitude: N/A", this);
    
    layout->addWidget(lbl_status_);
    layout->addWidget(lbl_gps_);
    layout->addWidget(lbl_altitude_);
    
    // Control buttons
    QHBoxLayout* btn_layout = new QHBoxLayout();
    btn_start_ = new QPushButton("Start Follow", this);
    btn_stop_ = new QPushButton("Stop Follow", this);
    btn_emergency_ = new QPushButton("EMERGENCY STOP", this);
    
    btn_emergency_->setStyleSheet("background-color: red; color: white; font-weight: bold;");
    
    btn_layout->addWidget(btn_start_);
    btn_layout->addWidget(btn_stop_);
    btn_layout->addWidget(btn_emergency_);
    layout->addLayout(btn_layout);
    
    // Follow distance input
    QHBoxLayout* param_layout = new QHBoxLayout();
    param_layout->addWidget(new QLabel("Follow Distance (m):", this));
    edit_follow_dist_ = new QLineEdit("5.0", this);
    param_layout->addWidget(edit_follow_dist_);
    layout->addLayout(param_layout);
    
    // Chart
    QChart* chart = new QChart();
    chart->setTitle("Altitude History");
    series_altitude_ = new QLineSeries();
    chart->addSeries(series_altitude_);
    chart->createDefaultAxes();
    
    chart_view_ = new QChartView(chart, this);
    layout->addWidget(chart_view_);
    
    setCentralWidget(central);
}

void MainWindow::setup_connections() {
    // Button signals
    connect(btn_start_, &QPushButton::clicked, this, &MainWindow::on_start_clicked);
    connect(btn_stop_, &QPushButton::clicked, this, &MainWindow::on_stop_clicked);
    connect(btn_emergency_, &QPushButton::clicked, this, &MainWindow::on_emergency_clicked);
    
    // ROS signals
    connect(ros_bridge_.get(), &QtRosBridge::droneStateUpdated, 
            this, &MainWindow::update_drone_state);
    connect(ros_bridge_.get(), &QtRosBridge::followStatusChanged,
            this, &MainWindow::update_status);
}

void MainWindow::on_start_clicked() {
    double distance = edit_follow_dist_->text().toDouble();
    ros_bridge_->setFollowDistance(distance);
    ros_bridge_->startFollow();
}

void MainWindow::on_stop_clicked() {
    ros_bridge_->stopFollow();
}

void MainWindow::on_emergency_clicked() {
    ros_bridge_->emergencyStop();
}

void MainWindow::update_drone_state(const drone_follow::core::DroneState& state) {
    lbl_gps_->setText(QString("GPS: %1, %2")
        .arg(state.gps.latitude, 0, 'f', 6)
        .arg(state.gps.longitude, 0, 'f', 6));
    
    lbl_altitude_->setText(QString("Altitude: %1 m")
        .arg(state.gps.altitude, 0, 'f', 2));
    
    // Update chart
    series_altitude_->append(QDateTime::currentMSecsSinceEpoch(), state.gps.altitude);
}

void MainWindow::update_status(const QString& status) {
    lbl_status_->setText("Status: " + status);
}
```

Tạo file `src/qt/main.cpp`:

```cpp
#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "main_window.hpp"

int main(int argc, char** argv) {
    // Khởi tạo Qt
    QApplication app(argc, argv);
    
    // Khởi tạo ROS2
    rclcpp::init(argc, argv);
    
    // Tạo và hiển thị GUI
    drone_follow::qt::MainWindow window;
    window.show();
    
    // Chạy Qt event loop
    int result = app.exec();
    
    // Cleanup ROS2
    rclcpp::shutdown();
    
    return result;
}
```

### Bước 5: Build & Run

```bash
cd /home/xb/ardu_ws
colcon build --packages-select drone_offboard
source install/setup.bash

# Chạy ROS2 node (backend)
ros2 run drone_offboard smart_follow_node

# Chạy Qt GUI (terminal khác)
ros2 run drone_offboard drone_gui
```

---

## 🔄 PHƯƠNG ÁN 2: Thay Thế Hoàn Toàn ROS2

### Bước 1: Tạo Qt + MAVLink Project

```cmake
cmake_minimum_required(VERSION 3.8)
project(drone_qt_app)

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Charts Network)
find_package(Eigen3 REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

# CORE LIBRARY (copy từ ROS project)
add_library(drone_core SHARED
  src/core/control/ap_control.cpp
  src/core/control/follow_controller.cpp
  # ... all core files
)

target_link_libraries(drone_core PUBLIC Eigen3::Eigen)

# MAVLink Communication Layer (NEW!)
add_library(mavlink_comm SHARED
  src/mavlink/mavlink_connection.cpp
  src/mavlink/mavlink_parser.cpp
)

target_link_libraries(mavlink_comm Qt6::Network)

# Qt Application
add_executable(drone_qt_app
  src/main.cpp
  src/main_window.cpp
  src/drone_controller.cpp
)

target_link_libraries(drone_qt_app
  drone_core
  mavlink_comm
  Qt6::Widgets
  Qt6::Charts
)
```

### Bước 2: Tạo MAVLink Connection

```cpp
#pragma once
#include <QObject>
#include <QUdpSocket>
#include <QTimer>

class MAVLinkConnection : public QObject {
    Q_OBJECT

public:
    explicit MAVLinkConnection(QObject* parent = nullptr);
    
    void connectToVehicle(const QString& host, quint16 port);
    void sendVelocityCommand(float vx, float vy, float vz, float yaw_rate);

signals:
    void gpsReceived(double lat, double lon, double alt);
    void attitudeReceived(float roll, float pitch, float yaw);

private slots:
    void readPendingDatagrams();
    void sendHeartbeat();

private:
    QUdpSocket* socket_;
    QTimer* heartbeat_timer_;
    QString target_host_;
    quint16 target_port_;
    
    void parseMAVLinkMessage(const QByteArray& data);
};
```

---

## 📊 So Sánh 2 Phương Án

| Tiêu Chí | Phương Án 1 (Qt + ROS2) | Phương Án 2 (Qt Only) |
|----------|-------------------------|----------------------|
| **Độ phức tạp** | Thấp (giữ nguyên ROS) | Cao (viết lại MAVLink) |
| **Thời gian** | 2-3 ngày | 1-2 tuần |
| **Tương thích ROS** | ✅ Có | ❌ Không |
| **Kích thước app** | Lớn (cần ROS runtime) | Nhỏ |
| **Phù hợp** | Dev/Testing với ROS | Production standalone |

---

## 🎓 Khuyến Nghị

### Bắt Đầu Với Phương Án 1
1. ✅ **Nhanh nhất** - Giữ nguyên core đã stable
2. ✅ **Ít rủi ro** - Không phải viết lại giao tiếp
3. ✅ **Linh hoạt** - Có thể chuyển sang Phương án 2 sau

### Khi Nào Chuyển Sang Phương Án 2?
- Cần deploy app độc lập không cần ROS
- Muốn giảm kích thước package
- Target platform không hỗ trợ ROS tốt (Windows, mobile)

---

## 📚 Tài Liệu Tham Khảo

- [Qt Documentation](https://doc.qt.io/)
- [Qt Charts](https://doc.qt.io/qt-6/qtcharts-index.html)
- [MAVLink Developer Guide](https://mavlink.io/en/)
- [ROS2 + Qt Integration](https://docs.ros.org/en/humble/How-To-Guides/Using-Python-Qt-with-ROS-2.html)

---

## 💡 Tips

1. **Sử dụng Qt Designer** để thiết kế UI nhanh hơn
2. **QSettings** để lưu parameters thay thế ROS params
3. **QThread** để chạy control loop không block GUI
4. **Qt WebSockets** để xây dựng web dashboard
5. **Qt Quick/QML** cho UI hiện đại hơn

---

## ❓ Còn thắc mắc?

Hỏi tôi về:
- Tích hợp Qt Designer
- Xây dựng 3D visualization với Qt3D
- Deploy Qt app ra Windows/mobile
- Performance tuning Qt + ROS2
