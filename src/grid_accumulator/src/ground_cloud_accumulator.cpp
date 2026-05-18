#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <cmath>
#include <vector>
#include <mutex>
#include <signal.h>
#include <fstream>

// Per‑frame: classify points in body frame (z<ground_thresh→FREE, z>obstacle_thresh→OCCUPIED)
//            transform to map frame, project to 2D grid, accumulate (obstacle wins)
// Publish OccupancyGrid. Ctrl‑C → PGM+YAML.

class GridAccumulatorV2 {
  using Lock = std::lock_guard<std::mutex>;

  // ── params ──────────────────────────────────────────────
  double resolution_         = 0.05;
  double ground_z_thresh_    = 0.15;
  double obstacle_z_thresh_  = 0.25;
  double publish_rate_       = 2.0;
  bool   auto_resize_        = true;
  std::string cloud_in_      = "/cloud_registered_body_1";
  std::string grid_out_      = "/accumulated_grid";
  std::string body_frame_    = "body";
  std::string map_frame_     = "map";

  // ── grid state ──────────────────────────────────────────
  std::vector<int8_t> grid_;
  int    grid_w_ = 0, grid_h_ = 0;          // 0 = uninitialised
  double origin_x_ = 0.0, origin_y_ = 0.0;
  bool   origin_set_ = false;
  mutable std::mutex mutex_;

  // ── TF ──────────────────────────────────────────────────
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // ── stats ───────────────────────────────────────────────
  size_t processed_  = 0;
  size_t ground_pts_ = 0;
  size_t obs_pts_    = 0;

public:
  GridAccumulatorV2(ros::NodeHandle &pnh)
    : tf_listener_(tf_buffer_)
  {
    pnh.param("resolution",         resolution_,         0.05);
    pnh.param("ground_z_thresh",    ground_z_thresh_,    0.15);
    pnh.param("obstacle_z_thresh",  obstacle_z_thresh_,  0.25);
    pnh.param("publish_rate",       publish_rate_,       2.0);
    pnh.param("auto_resize",        auto_resize_,        true);
    pnh.param("cloud_in",           cloud_in_,  std::string("/cloud_registered_body_1"));
    pnh.param("grid_out",           grid_out_,  std::string("/accumulated_grid"));
    pnh.param("body_frame",         body_frame_, std::string("body"));
    pnh.param("map_frame",          map_frame_,  std::string("map"));

    grid_.assign(grid_w_ * grid_h_, static_cast<int8_t>(-1));
    ROS_INFO("GridV2: %dx%d %.2f m/px, ground<%.2f obstacle>%.2f",
             grid_w_, grid_h_, resolution_, ground_z_thresh_, obstacle_z_thresh_);
  }

  // ── cloud callback ──────────────────────────────────────
  void onCloud(const sensor_msgs::PointCloud2ConstPtr &msg) {
    geometry_msgs::TransformStamped tf;
    try {
      tf = tf_buffer_.lookupTransform(map_frame_, msg->header.frame_id,
                                       msg->header.stamp, ros::Duration(0.1));
    } catch (tf2::TransformException &e) {
      ROS_WARN_THROTTLE(5.0, "TF %s→%s fail: %s", map_frame_.c_str(), msg->header.frame_id.c_str(), e.what());
      return;
    }

    const auto &t = tf.transform.translation;
    const auto &q = tf.transform.rotation;
    double tx=t.x, ty=t.y, tz=t.z;
    double qx=q.x, qy=q.y, qz=q.z, qw=q.w;
    double R00=1.-2.*(qy*qy+qz*qz), R01=2.*(qx*qy-qz*qw), R02=2.*(qx*qz+qy*qw);
    double R10=2.*(qx*qy+qz*qw), R11=1.-2.*(qx*qx+qz*qz), R12=2.*(qy*qz-qx*qw);

    sensor_msgs::PointCloud2ConstIterator<float> x_it(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y_it(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z_it(*msg, "z");

    // ── lazy initialise grid from first frame map bounds ────
    if (!origin_set_) {
      double min_x = 1e9, max_x = -1e9, min_y = 1e9, max_y = -1e9;
      size_t n = msg->width * msg->height;
      for (size_t i = 0; i < n; ++i, ++x_it, ++y_it, ++z_it) {
        float px = *x_it, py = *y_it, pz = *z_it;
        if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;
        if (!(pz < ground_z_thresh_ && pz > -1.0f) && !(pz > obstacle_z_thresh_)) continue;
        double mx = R00*px + R01*py + R02*pz + tx;
        double my = R10*px + R11*py + R12*pz + ty;
        if (mx < min_x) min_x = mx; if (mx > max_x) max_x = mx;
        if (my < min_y) min_y = my; if (my > max_y) max_y = my;
      }
      double margin = 5.0;
      origin_x_ = min_x - margin;
      origin_y_ = min_y - margin;
      grid_w_ = static_cast<int>(std::ceil((max_x - min_x + 2*margin) / resolution_));
      grid_h_ = static_cast<int>(std::ceil((max_y - min_y + 2*margin) / resolution_));
      grid_.assign(grid_w_ * grid_h_, static_cast<int8_t>(-1));
      origin_set_ = true;
      ROS_INFO("Grid init from data: %dx%d [%.1f,%.1f]",
               grid_w_, grid_h_, origin_x_, origin_y_);
      // reset iterators for the actual processing loop
      x_it = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "x");
      y_it = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "y");
      z_it = sensor_msgs::PointCloud2ConstIterator<float>(*msg, "z");
    }

    Lock lock(mutex_);
    size_t n = msg->width * msg->height;
    for (size_t i = 0; i < n; ++i, ++x_it, ++y_it, ++z_it) {
      float px = *x_it, py = *y_it, pz = *z_it;
      if (!std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;

      ++processed_;
      bool is_ground   = (pz < ground_z_thresh_ && pz > -1.0f);
      bool is_obstacle = (pz > obstacle_z_thresh_);
      if (!is_ground && !is_obstacle) continue;  // transition band, skip

      // TF body → map
      double mx = R00*px + R01*py + R02*pz + tx;
      double my = R10*px + R11*py + R12*pz + ty;

      int col = static_cast<int>(std::round((mx - origin_x_) / resolution_));
      int row = static_cast<int>(std::round((my - origin_y_) / resolution_));

      if (col < 0 || col >= grid_w_ || row < 0 || row >= grid_h_) {
        if (!auto_resize_) continue;
        resize(col, row);
        col = static_cast<int>(std::round((mx - origin_x_) / resolution_));
        row = static_cast<int>(std::round((my - origin_y_) / resolution_));
        if (col < 0 || col >= grid_w_ || row < 0 || row >= grid_h_) continue;
      }

      int idx = row * grid_w_ + col;
      if (is_obstacle) {
        grid_[idx] = 100;   // obstacle always wins
        ++obs_pts_;
      } else if (is_ground && grid_[idx] == -1) {
        grid_[idx] = 0;     // free only if previously unknown
        ++ground_pts_;
      }
    }
  }

  // ── publish loop ─────────────────────────────────────────
  void publishLoop(ros::Publisher &pub) {
    ros::Rate rate(publish_rate_);
    while (ros::ok()) {
      rate.sleep();
      Lock lock(mutex_);
      nav_msgs::OccupancyGrid msg;
      msg.header.stamp = ros::Time::now();
      msg.header.frame_id = map_frame_;
      msg.info.resolution = resolution_;
      msg.info.width  = grid_w_;
      msg.info.height = grid_h_;
      msg.info.origin.position.x = origin_x_;
      msg.info.origin.position.y = origin_y_;
      msg.info.origin.position.z = 0;
      msg.info.origin.orientation.w = 1;
      msg.data.assign(grid_.begin(), grid_.end());
      pub.publish(msg);
    }
  }

  // ── save PGM + YAML ─────────────────────────────────────
  void save(const std::string &pgm_path, const std::string &yaml_path) {
    Lock lock(mutex_);
    int occ=0, free_c=0;
    for (auto v : grid_) { if(v>=100) ++occ; else if(v==0) ++free_c; }

    std::ofstream pgm(pgm_path, std::ios::binary);
    pgm << "P5\n" << grid_w_ << " " << grid_h_ << "\n255\n";
    for (int r = grid_h_ - 1; r >= 0; --r) {
      const int8_t *row = &grid_[r * grid_w_];
      for (int c = 0; c < grid_w_; ++c) {
        uint8_t v = (row[c] >= 100) ? 0 : (row[c] == 0 ? 254 : 205);
        pgm.write(reinterpret_cast<const char*>(&v), 1);
      }
    }
    pgm.close();

    std::ofstream yml(yaml_path);
    yml << "image: " << pgm_path.substr(pgm_path.find_last_of('/')+1) << "\n"
        << "resolution: " << resolution_ << "\n"
        << "origin: [" << origin_x_ << ", " << origin_y_ << ", 0.0]\n"
        << "negate: 0\noccupied_thresh: 0.45\nfree_thresh: 0.196\n";
    yml.close();
    ROS_INFO("Saved %dx%d PGM occ=%d free=%d orig=[%.1f,%.1f]",
             grid_w_, grid_h_, occ, free_c, origin_x_, origin_y_);
  }

  void stats() {
    Lock lock(mutex_);
    int occ=0, free_c=0;
    for (auto v : grid_) { if(v>=100) ++occ; else if(v==0) ++free_c; }
    ROS_INFO("Proc:%zu ground:%zu obs:%zu | grid:%d occ %d free %d cells",
             processed_, ground_pts_, obs_pts_, occ, free_c, grid_w_*grid_h_);
  }

private:
  void resize(int col, int row) {
    int new_w = grid_w_, new_h = grid_h_;
    double ox = origin_x_, oy = origin_y_;
    double grow = std::max(grid_w_, grid_h_) / 2.0 * resolution_;
    if (col < 0)      { ox -= grow; new_w += static_cast<int>(grow / resolution_); }
    if (col >= grid_w_) { new_w += static_cast<int>(grow / resolution_); }
    if (row < 0)      { oy -= grow; new_h += static_cast<int>(grow / resolution_); }
    if (row >= grid_h_) { new_h += static_cast<int>(grow / resolution_); }

    std::vector<int8_t> ng(new_w * new_h, static_cast<int8_t>(-1));
    int dx = static_cast<int>(std::round((origin_x_ - ox) / resolution_));
    int dy = static_cast<int>(std::round((origin_y_ - oy) / resolution_));
    for (int r=0; r<grid_h_; ++r)
      for (int c=0; c<grid_w_; ++c)
        if (grid_[r*grid_w_+c] >= 0)
          ng[(r+dy)*new_w+(c+dx)] = grid_[r*grid_w_+c];
    grid_.swap(ng);
    origin_x_=ox; origin_y_=oy; grid_w_=new_w; grid_h_=new_h;
    ROS_INFO("Grid auto‑resized to %dx%d [%.1f,%.1f]", grid_w_, grid_h_, origin_x_, origin_y_);
  }
};

static GridAccumulatorV2 *g_node = nullptr;

void sigHandler(int) {
  if (g_node) {
    g_node->save("/root/maps/accumulated_grid.pgm", "/root/maps/accumulated_grid.yaml");
    g_node->stats();
  }
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "grid_accumulator_v2");
  ros::NodeHandle pnh("~");
  signal(SIGINT, sigHandler);

  GridAccumulatorV2 node(pnh);
  g_node = &node;

  ros::Subscriber sub = ros::NodeHandle().subscribe(
      pnh.param<std::string>("cloud_in", "/cloud_registered_body_1"),
      10, &GridAccumulatorV2::onCloud, &node);
  ros::Publisher pub = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>(
      pnh.param<std::string>("grid_out", "/accumulated_grid"), 1, true);

  ros::AsyncSpinner spinner(2);
  spinner.start();
  node.publishLoop(pub);
  spinner.stop();

  node.save("/root/maps/accumulated_grid.pgm", "/root/maps/accumulated_grid.yaml");
  node.stats();
  return 0;
}
