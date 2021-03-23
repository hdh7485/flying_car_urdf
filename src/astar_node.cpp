#include <ros/ros.h>
#include <flying_car_urdf/astar.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "astar_node");
    AStar::Generator generator;
    generator.setWorldSize({25, 25});
    generator.setHeuristic(AStar::Heuristic::euclidean);
    generator.setDiagonalMovement(true);

    std::cout << "Generate path ... \n";
    auto path = generator.findPath({0, 0}, {20, 20});

    for(auto& coordinate : path) {
        std::cout << coordinate.x << " " << coordinate.y << "\n";
    }
    return 0;
}
