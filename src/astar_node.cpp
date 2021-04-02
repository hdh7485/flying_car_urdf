#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <flying_car_urdf/astar.h>

class ROSAstar{
public:
    ROSAstar(){
        generator.setHeuristic(AStar::Heuristic::euclidean);
        generator.setDiagonalMovement(true);
        sub_ = n_.subscribe("/virtual_grid", 1, &ROSAstar::gridCallback, this);
        pub_ = n_.advertise<nav_msgs::Path>("/astar_path", 1);
    }

    void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
        generator.setWorldSize({msg->info.width, msg->info.height});
        for (auto i=0; i<msg->data.size(); i++){
            if (msg->data[i] > 90){
                ROS_INFO("x:%d, y:%d, value:%d", 
                    i%msg->info.width, i/msg->info.width, msg->data[i]);
                generator.addCollision({i%msg->info.width, i/msg->info.width});
            }
        }
        ROS_INFO("Generate path ... \n");
        auto path = generator.findPath({0, 0}, {5, 5});
        nav_msgs::Path astarPath;
        astarPath.header = msg->header;
        for(auto& coordinate : path) {
            geometry_msgs::PoseStamped astar_pose;
            astar_pose.header = astarPath.header;
            astar_pose.pose.position.x = coordinate.x;
            astar_pose.pose.position.y = coordinate.y;
            astarPath.poses.push_back(astar_pose);
            // std::cout << coordinate.x << " " << coordinate.y << "\n";
        }
        pub_.publish(astarPath);
    }
private:
    AStar::Generator generator;
    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};

int main(int argc, char** argv){
    ros::init(argc, argv, "astar_node");
    ROSAstar rosAstar;
    ros::spin();

    return 0;
}
