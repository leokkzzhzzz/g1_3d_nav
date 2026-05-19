#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/point_cloud.h>

struct GridParams {
    float position_x = 0, position_y = 0, cell_size = 0.05;
    float length_x = 80, length_y = 80;
    float intensity_factor = 0.2, height_factor = 1.0;
    std::string cloud_in = "/cloud_registered_body_1";
    std::string mapi_topic = "/lidargrid_i";
    std::string maph_topic = "/lidargrid_h";
    float topleft_x, topleft_y, bottomright_x, bottomright_y;
    int cell_num_x, cell_num_y;

    void refresh() {
        topleft_x = position_x + length_x / 2;
        bottomright_x = position_x - length_x / 2;
        topleft_y = position_y + length_y / 2;
        bottomright_y = position_y - length_y / 2;
        cell_num_x = int(length_x / cell_size);
        cell_num_y = int(length_y / cell_size);
    }
};

GridParams g;

nav_msgs::OccupancyGridPtr intensity_grid(new nav_msgs::OccupancyGrid);
nav_msgs::OccupancyGridPtr height_grid(new nav_msgs::OccupancyGrid);

void initGrid(nav_msgs::OccupancyGridPtr grid, std::string frame_id) {
    grid->header.frame_id = frame_id;
    grid->info.origin.position.x = g.position_x + g.length_x / 2;
    grid->info.origin.position.y = g.position_y + g.length_y / 2;
    grid->info.origin.position.z = 0;
    grid->info.origin.orientation.w = 1.0;
    grid->info.width = g.cell_num_x;
    grid->info.height = g.cell_num_y;
    grid->info.resolution = g.cell_size;
}

void cloudCallback(const pcl::PCLPointCloud2 &msg) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromPCLPointCloud2(msg, cloud);
    initGrid(intensity_grid, msg.header.frame_id);
    initGrid(height_grid, msg.header.frame_id);
    
    std::vector<signed char> hpoints(g.cell_num_x * g.cell_num_y, -128);
    std::vector<signed char> ipoints(g.cell_num_x * g.cell_num_y, -128);

    for (auto &pt : cloud) {
        if (pt.x > g.bottomright_x && pt.x < g.topleft_x &&
            pt.y > g.bottomright_y && pt.y < g.topleft_y) {
            int cx = int(fabs(pt.x - g.topleft_x) / g.cell_size);
            int cy = int(fabs(pt.y - g.topleft_y) / g.cell_size);
            if (cx < g.cell_num_x && cy < g.cell_num_y) {
                int idx = cy * g.cell_num_x + cx;
                ipoints[idx] = pt.intensity * g.intensity_factor;
                hpoints[idx] = pt.z * g.height_factor;
            }
        }
    }

    intensity_grid->header.stamp = ros::Time::now();
    intensity_grid->data = ipoints;
    height_grid->header.stamp = ros::Time::now();
    height_grid->data = hpoints;

    static ros::Publisher pub_i = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>(g.mapi_topic, 1, true);
    static ros::Publisher pub_h = ros::NodeHandle().advertise<nav_msgs::OccupancyGrid>(g.maph_topic, 1, true);
    pub_i.publish(intensity_grid);
    pub_h.publish(height_grid);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "pointcloud_to_grid_node");
    ros::NodeHandle nh("~");
    nh.param("cell_size", g.cell_size, 0.05f);
    nh.param("length_x", g.length_x, 80.0f);
    nh.param("length_y", g.length_y, 80.0f);
    nh.param("height_factor", g.height_factor, 1.0f);
    nh.param("cloud_in", g.cloud_in, std::string("/cloud_registered_body_1"));
    nh.param("maph_topic", g.maph_topic, std::string("/lidargrid_h"));
    g.refresh();
    ROS_INFO("Grid: %dx%d cells, %.2fm/px", g.cell_num_x, g.cell_num_y, g.cell_size);
    
    ros::Subscriber sub = nh.subscribe(g.cloud_in, 1, cloudCallback);
    ros::spin();
    return 0;
}
