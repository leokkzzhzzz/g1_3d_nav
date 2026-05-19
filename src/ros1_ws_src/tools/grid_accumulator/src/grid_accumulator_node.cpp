#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float32.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <signal.h>
#include <fstream>

// Grid accumulates occupancy: 0=FREE, 100=OCCUPIED, -1=UNKNOWN
// Once a cell is OCCUPIED, it stays OCCUPIED (ground can't erase it).

class GridAccumulator {
  using Lock = std::lock_guard<std::mutex>;

  // params
  double resolution_ = 0.05;
  double map_length_x_ = 100.0, map_length_y_ = 100.0;
  double origin_x_ = -50.0, origin_y_ = -50.0;  // centre at (0,0) default
  int    grid_w_ = 2000, grid_h_ = 2000;
  double ground_z_thresh_ = 0.15;   // in body frame, z<this => ground
  double obstacle_z_thresh_ = 0.25; // in body frame, z>this => obstacle
  bool   auto_resize_ = true;
  double publish_rate_ = 2.0;
  std::string cloud_topic_{"/cloud_registered_body_1"};
  std::string grid_topic_{"/accumulated_grid"};
  std::string body_frame_{"body"};
  std::string map_frame_{"map"};

  // grid
  std::vector<int8_t> grid_;        // row-major
  mutable std::mutex   grid_mutex_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // state
  size_t point_count_ = 0, obstacle_count_ = 0, ground_count_ = 0;
  bool   map_saved_ = false;

public:
  GridAccumulator(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    : tf_listener_(tf_buffer_)
  {
    // read params
    pnh.param("resolution", resolution_, 0.05);
    pnh.param("map_length_x", map_length_x_, 100.0);
    pnh.param("map_length_y", map_length_y_, 100.0);
    pnh.param("ground_z_thresh", ground_z_thresh_, 0.15);
    pnh.param("obstacle_z_thresh", obstacle_z_thresh_, 0.25);
    pnh.param("auto_resize", auto_resize_, true);
    pnh.param("publish_rate", publish_rate_, 2.0);
    pnh.param("cloud_topic", cloud_topic_, std::string("/cloud_registered_body_1"));
    pnh.param("grid_topic", grid_topic_, std::string("/accumulated_grid"));
    pnh.param("body_frame", body_frame_, std::string("body"));
    pnh.param("map_frame", map_frame_, std::string("map"));

    // init grid from params
    grid_w_ = static_cast<int>(std::ceil(map_length_x_ / resolution_));
    grid_h_ = static_cast<int>(std::ceil(map_length_y_ / resolution_));
    grid_.assign(grid_w_ * grid_h_, static_cast<int8_t>(-1));

    origin_x_ = -map_length_x_ / 2.0;
    origin_y_ = -map_length_y_ / 2.0;

    ROS_INFO("Grid: %dx%d cells, %.2f m/cell, [%.1f,%.1f] x [%.1f,%.1f]",
             grid_w_, grid_h_, resolution_, origin_x_, origin_x_ + map_length_x_,
             origin_y_, origin_y_ + map_length_y_);
  }

  // ── point cloud callback ──────────────────────────────────
  void onCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                       msg->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException &e) {
      ROS_WARN_THROTTLE(5.0, "TF lookup failed: %s", e.what());
      return;
    }

    Lock lock(grid_mutex_);

    // Pre‑compute transform matrix
    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    double tx = t.x, ty = t.y, tz = t.z;
    double qx = q.x, qy = q.y, qz = q.z, qw = q.w;
    // rotation matrix for (qx,qy,qz,qw)
    double R00 = 1. - 2.*(qy*qy + qz*qz);
    double R01 = 2.*(qx*qy - qz*qw);
    double R02 = 2.*(qx*qz + qy*qw);
    double R10 = 2.*(qx*qy + qz*qw);
    double R11 = 1. - 2.*(qx*qx + qz*qz);
    double R12 = 2.*(qy*qz - qx*qw);
    // R20, R21, R22 not needed for Z if we only need Z for classification
    double R20 = 2.*(qx*qz - qy*qw);
    double R21 = 2.*(qy*qz + qx*qw);
    double R22 = 1. - 2.*(qx*qx + qy*qy);

    point_count_ += msg->width * msg->height;

    sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");

    int added = 0;
    for (; x_it != x_it.end(); ++x_it, ++y_it, ++z_it) {
      float px = *x_it, py = *y_it, pz = *z_it;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

      // Classify in body frame (robot‑local)
      bool is_ground   = (pz < ground_z_thresh_);
      bool is_obstacle = (pz > obstacle_z_thresh_);
      if (!is_ground && !is_obstacle) continue;

      // Transform to map frame
      double mx = R00*px + R01*py + R02*pz + tx;
      double my = R10*px + R11*py + R12*pz + ty;

      // Grid index
      int col = static_cast<int>(std::round((mx - origin_x_) / resolution_));
      int row = static_cast<int>(std::round((my - origin_y_) / resolution_));
      if (col < 0 || col >= grid_w_ || row < 0 || row >= grid_h_) {
        if (!auto_resize_) continue;
        // auto‑grow
        int new_w = grid_w_, new_h = grid_h_;
        double new_ox = origin_x_, new_oy = origin_y_;
        if (col < 0) { new_ox -= (grid_w_ / 2) * resolution_; new_w += grid_w_ / 2; }
        if (col >= grid_w_) { new_w += grid_w_ / 2; }
        if (row < 0) { new_oy -= (grid_h_ / 2) * resolution_; new_h += grid_h_ / 2; }
        if (row >= grid_h_) { new_h += grid_h_ / 2; }

        std::vector<int8_t> new_grid(new_w * new_h, static_cast<int8_t>(-1));
        int dx = static_cast<int>(std::round((origin_x_ - new_ox) / resolution_));
        int dy = static_cast<int>(std::round((origin_y_ - new_oy) / resolution_));
        for (int r = 0; r < grid_h_; ++r)
          for (int c = 0; c < grid_w_; ++c)
            if (grid_[r * grid_w_ + c] >= 0)
              new_grid[(r + dy) * new_w + (c + dx)] = grid_[r * grid_w_ + c];

        origin_x_ = new_ox; origin_y_ = new_oy;
        grid_w_ = new_w; grid_h_ = new_h;
        grid_.swap(new_grid);
        col += dx; row += dy;

        ROS_INFO("Grid resized: %dx%d origin=[%.1f,%.1f]",
                 grid_w_, grid_h_, origin_x_, origin_y_);
        if (col < 0 || col >= grid_w_ || row < 0 || row >= grid_h_) continue;
      }

      int idx = row * grid_w_ + col;
      if (is_obstacle) {
        grid_[idx] = 100;
        ++obstacle_count_;
        ++added;
      } else if (is_ground && grid_[idx] == -1) {
        grid_[idx] = 0;
        ++ground_count_;
        ++added;
      }
    }
  }

  // ── publish loop ──────────────────────────────────────────
  void publishLoop(const ros::Publisher &pub) {
    ros::Rate rate(publish_rate_);
    while (ros::ok()) {
      {
        Lock lock(grid_mutex_);
        nav_msgs::OccupancyGrid msg;
        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = map_frame_;
        msg.info.resolution = resolution_;
        msg.info.width = grid_w_;
        msg.info.height = grid_h_;
        msg.info.origin.position.x = origin_x_;
        msg.info.origin.position.y = origin_y_;
        msg.info.origin.position.z = 0;
        msg.info.origin.orientation.w = 1;
        msg.data.assign(grid_.begin(), grid_.end());
        pub.publish(msg);
      }
      rate.sleep();
    }
  }

  // ── save PGM + YAML ───────────────────────────────────────
  bool save(const std::string &pgm_path, const std::string &yaml_path) {
    Lock lock(grid_mutex_);
    int occ = 0, free = 0;
    for (auto v : grid_) { if (v >= 100) ++occ; else if (v == 0) ++free; }

    std::ofstream pgm(pgm_path, std::ios::binary);
    pgm << "P5\n" << grid_w_ << " " << grid_h_ << "\n255\n";
    for (auto v : grid_) {
      uint8_t c = (v >= 100) ? 0 : (v == 0 ? 254 : 205);
      pgm.write(reinterpret_cast<const char*>(&c), 1);
    }
    pgm.close();

    std::ofstream yml(yaml_path);
    yml << "image: " << pgm_path.substr(pgm_path.find_last_of('/') + 1) << "\n"
        << "resolution: " << resolution_ << "\n"
        << "origin: [" << origin_x_ << ", " << origin_y_ << ", 0.0]\n"
        << "negate: 0\noccupied_thresh: 0.45\nfree_thresh: 0.196\n";
    yml.close();

    ROS_INFO("Saved %dx%d PGM, occupied=%d free=%d", grid_w_, grid_h_, occ, free);
    map_saved_ = true;
    return true;
  }

  // ── stats ─────────────────────────────────────────────────
  void stats() {
    Lock lock(grid_mutex_);
    int occ = 0, free_c = 0;
    for (auto v : grid_) { if (v >= 100) ++occ; else if (v == 0) ++free_c; }
    ROS_INFO("Points: %zu processed, %zu obstacle, %zu ground | Grid: %d occ, %d free, %d cells",
             point_count_, obstacle_count_, ground_count_, occ, free_c, grid_w_ * grid_h_);
  }

  bool saved() const { return map_saved_; }
};

// ── global for signal handler ───────────────────────────────
static GridAccumulator *g_accum = nullptr;

void signalHandler(int) {
  if (g_accum) {
    ROS_INFO("Saving map on shutdown...");
    g_accum->save("/root/maps/accumulated_grid.pgm", "/root/maps/accumulated_grid.yaml");
    g_accum->stats();
  }
  ros::shutdown();
}

// ── main ────────────────────────────────────────────────────
int main(int argc, char **argv) {
  ros::init(argc, argv, "grid_accumulator_node");
  ros::NodeHandle nh, pnh("~");
  signal(SIGINT, signalHandler);

  GridAccumulator accum(nh, pnh);
  g_accum = &accum;

  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>(
      pnh.param<std::string>("grid_topic", "/accumulated_grid"), 1, true);

  ros::Subscriber sub = nh.subscribe(
      pnh.param<std::string>("cloud_topic", "/cloud_registered_body_1"),
      10, &GridAccumulator::onCloud, &accum);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  accum.publishLoop(pub);
  spinner.stop();

  // final save
  if (!accum.saved())
    accum.save("/root/maps/accumulated_grid.pgm", "/root/maps/accumulated_grid.yaml");
  accum.stats();

  return 0;
}
