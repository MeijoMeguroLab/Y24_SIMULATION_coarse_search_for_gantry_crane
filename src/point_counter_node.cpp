#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <mutex>

class PointCounterNode : public rclcpp::Node
{
public:
    PointCounterNode() : Node("point_counter_node")
    {
        // 閾値を設定
        this->declare_parameter<int>("point_threshold", 300);
        this->get_parameter("point_threshold", point_threshold_);

        // 高いz値の閾値を設定
        this->declare_parameter<double>("z_high_threshold", 3.0); // 例として5.0メートル以上を高いz値とする
        this->get_parameter("z_high_threshold", z_high_threshold_);

        // 最小距離の閾値を設定（近すぎる点を除外するため）
        this->declare_parameter<double>("min_distance_threshold", 5.0); // 例として1.0メートル以内を除外
        this->get_parameter("min_distance_threshold", min_distance_threshold_);

        // クリッピング範囲を設定（map座標系での四角形の座標）
        clipping_region_ = {
            {-154161.0, 38311.0},  // 左下
            {-154155.0, 38205.0},  // 右下
            {-154136.0, 38209.0},  // 右上
            {-154134.0, 38312.0}   // 左上
        };

        // 最新のPoseを初期化
        latest_pose_.pose.orientation.w = 1.0;

        // サブスクライバの設定
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            // "/ekf_localization/current_pose", 10,
            "/sensing/gnss/awsim_pose", 10,
            std::bind(&PointCounterNode::pose_callback, this, std::placeholders::_1));

        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/sensing/lidar/top/awsim_pointcloud_raw_ex", 10,
            std::bind(&PointCounterNode::pointcloud_callback, this, std::placeholders::_1));

        // 重心位置のパブリッシャを追加
        centroid_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "centroid_position", 10);

        // フラグの初期化
        flag_ = false;
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        latest_pose_ = *msg;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        geometry_msgs::msg::PoseStamped pose;
        {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            pose = latest_pose_;
        }

        // クォータニオンから回転行列を作成
        tf2::Quaternion q(
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w);

        tf2::Matrix3x3 rotation_matrix(q);

        // 平行移動ベクトル
        tf2::Vector3 translation(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z);

        // 変換行列の作成
        tf2::Transform transform(rotation_matrix, translation);

        // PointCloud2からポイントを取得し、map座標系に変換
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        std::vector<tf2::Vector3> high_z_points;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            // 車両座標系の点
            tf2::Vector3 point_vehicle_frame(*iter_x, *iter_y, *iter_z);

            // センサからの距離を計算
            double distance = point_vehicle_frame.length();

            // 最小距離の閾値を超えているかチェック（近すぎる点を除外）
            if (distance < min_distance_threshold_)
            {
                continue; // この点は除外
            }

            // map座標系の点に変換
            tf2::Vector3 point_map_frame = transform * point_vehicle_frame;

            // クリッピング範囲内かどうかをチェック
            if (is_point_in_polygon(point_map_frame.x(), point_map_frame.y()))
            {
                // z座標が閾値以上かどうかをチェック
                if (*iter_z >= z_high_threshold_)
                {
                    high_z_points.push_back(point_vehicle_frame);
                }
            }
        }

        // デバッグ情報をターミナルに表示
        RCLCPP_INFO(this->get_logger(), "High z-value point count in clipping region: %zu", high_z_points.size());

        // 閾値と比較してフラグを設定
        bool previous_flag = flag_;  // 前回のフラグ状態を保存
        if (high_z_points.size() >= static_cast<size_t>(point_threshold_))
        {
            // 重心を計算
            tf2::Vector3 centroid_vehicle_frame(0.0, 0.0, 0.0);
            for (const auto& point : high_z_points)
            {
                centroid_vehicle_frame += point;
            }
            centroid_vehicle_frame /= static_cast<double>(high_z_points.size());

            // 重心をマップ座標系に変換
            tf2::Vector3 centroid_map_frame = transform * centroid_vehicle_frame;

            // 重心の位置をターミナルに表示（自車座標系）
            RCLCPP_INFO(this->get_logger(), "Centroid of high z-value points in vehicle frame: x=%.2f, y=%.2f, z=%.2f",
                        centroid_vehicle_frame.x(), centroid_vehicle_frame.y(), centroid_vehicle_frame.z());

            // 重心の位置をターミナルに表示（マップ座標系）
            RCLCPP_INFO(this->get_logger(), "Centroid of high z-value points in map frame: x=%.2f, y=%.2f, z=%.2f",
                        centroid_map_frame.x(), centroid_map_frame.y(), centroid_map_frame.z());

            // 重心位置をパブリッシュ
            geometry_msgs::msg::Point point_msg;
            point_msg.x = centroid_vehicle_frame.x();
            point_msg.y = centroid_vehicle_frame.y();
            //point_msg.z = 0.0; // zは不要な場合0に設定
            centroid_pub_->publish(point_msg);

            flag_ = true;
        }
        else
        {
            flag_ = false;
        }

        // フラグの状態が変化した場合のみ表示
        if (flag_ != previous_flag)
        {
            if (flag_)
            {
                RCLCPP_INFO(this->get_logger(), "Flag set to TRUE");
                // 必要に応じてここでフラグが立った際の処理を追加
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Flag set to FALSE");
                // 必要に応じてここでフラグが下がった際の処理を追加
            }
        }
        else
        {
            // デバッグのためにフラグの状態を表示
            RCLCPP_INFO(this->get_logger(), "Flag remains %s", flag_ ? "TRUE" : "FALSE");
        }
    }

    // 点が多角形内にあるかを判定（射影法）
    bool is_point_in_polygon(double x, double y)
    {
        bool inside = false;
        size_t num_points = clipping_region_.size();
        for (size_t i = 0, j = num_points - 1; i < num_points; j = i++)
        {
            double xi = clipping_region_[i].first, yi = clipping_region_[i].second;
            double xj = clipping_region_[j].first, yj = clipping_region_[j].second;

            bool intersect = ((yi > y) != (yj > y)) &&
                             (x < (xj - xi) * (y - yi) / (yj - yi + 1e-10) + xi);
            if (intersect)
                inside = !inside;
        }
        return inside;
    }

    // メンバ変数
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_pub_;

    geometry_msgs::msg::PoseStamped latest_pose_;
    std::mutex pose_mutex_;

    std::vector<std::pair<double, double>> clipping_region_;
    int point_threshold_;
    double z_high_threshold_;
    double min_distance_threshold_;
    bool flag_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCounterNode>());
    rclcpp::shutdown();
    return 0;
}