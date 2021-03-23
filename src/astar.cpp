#include <flying_car_urdf/astar.h>
#include <algorithm>

using namespace std::placeholders;

bool AStar::Vec2i::operator == (const Vec2i& coordinates_)
{
    return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator + (const AStar::Vec2i& left_, const AStar::Vec2i& right_)
{
    return{ left_.x + right_.x, left_.y + right_.y };
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_)
{
    parent = parent_;
    coordinates = coordinates_;
    G = H = 0;
}

AStar::uint AStar::Node::getScore()
{
    return G + H;
}

AStar::Generator::Generator()
{
    setDiagonalMovement(false);
    setHeuristic(&Heuristic::manhattan);
    direction = {
        { 0, 1 }, { 1, 0 }, { 0, -1 }, { -1, 0 },
        { -1, -1 }, { 1, 1 }, { -1, 1 }, { 1, -1 }
    };
}

void AStar::Generator::setWorldSize(Vec2i worldSize_)
{
    worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_)
{
    directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_)
{
    heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_)
{
    walls.push_back(coordinates_);
}

void AStar::Generator::removeCollision(Vec2i coordinates_)
{
    auto it = std::find(walls.begin(), walls.end(), coordinates_);
    if (it != walls.end()) {
        walls.erase(it);
    }
}

void AStar::Generator::clearCollisions()
{
    walls.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_)
{
    Node *current = nullptr;
    NodeSet openSet, closedSet;
    openSet.reserve(100);
    closedSet.reserve(100);
    openSet.push_back(new Node(source_));

    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;

        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->getScore() <= current->getScore()) {
                current = node;
                current_it = it;
            }
        }

        if (current->coordinates == target_) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        for (uint i = 0; i < directions; ++i) {
            Vec2i newCoordinates(current->coordinates + direction[i]);
            if (detectCollision(newCoordinates) ||
                findNodeOnList(closedSet, newCoordinates)) {
                continue;
            }

            uint totalCost = current->G + ((i < 4) ? 10 : 14);

            Node *successor = findNodeOnList(openSet, newCoordinates);
            if (successor == nullptr) {
                successor = new Node(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.push_back(successor);
            }
            else if (totalCost < successor->G) {
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    CoordinateList path;
    while (current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_, Vec2i coordinates_)
{
    for (auto node : nodes_) {
        if (node->coordinates == coordinates_) {
            return node;
        }
    }
    return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_)
{
    for (auto it = nodes_.begin(); it != nodes_.end();) {
        delete *it;
        it = nodes_.erase(it);
    }
}

bool AStar::Generator::detectCollision(Vec2i coordinates_)
{
    if (coordinates_.x < 0 || coordinates_.x >= worldSize.x ||
        coordinates_.y < 0 || coordinates_.y >= worldSize.y ||
        std::find(walls.begin(), walls.end(), coordinates_) != walls.end()) {
        return true;
    }
    return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_)
{
    return{ abs(source_.x - target_.x),  abs(source_.y - target_.y) };
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_)
{
    auto delta = std::move(getDelta(source_, target_));
    return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}

// #include "astar.h"

// Astar::Astar(p_ii start, p_ii goal, p_ii mapSize, std::vector<int> map){
//     start_ = start;
//     goal_ = goal;
//     mapSize_ = mapSize;
//     map_ = map;
// }

// std::vector<p_ii> Astar::plan(){
//     Nodeset openSet;
//     Nodeset closedSet;
//     Node startNode;
//     startNode.G = 0;
//     startNode.H = 0;
//     startNode.coordinates = start_;
//     startNode.parent = nullptr;
//     openSet.pushback(startNode);

//     bool success = false;

//     while (!openSet.empty()){
//         Nodeset::iterator current_it = openSet.begin();
//         currentNode = *current_it;

//         for (Nodeset::iterator it=openSet.begin(); it!= openSet.end(); it++){
//             Node node = *it;
//             if (node.G + node.H < currentNode.G + currentNode.H)
//             {
//                 currentNode = node;
//                 current_it = it;
//             }
//         }
//         if (currentNode.coordinates == goal_){
//             break;
//         }
//         closedSet.push_back(currentNode);
//         openSet.erase(current_it);
        
//         int index = currentNode.coordinates.first 
//                         + currentNode.coordinates.second*mapSize_.first;
//         p_ii newCoordinate = {0, 0};
//         //12
//         bool isBeing = false;
//         newCoordinate = {currentNode.coordinates.first,
//                          ,(currentNode.coordinates.second+1)*mapSize_.first}

//         for (auto node : closedSet){
//             if (node.coordinates == newCoordinate)
//                 isBeing = true;
//         }
//         if (currentNode.coordinates.first < mapSize_.first 
//             || !isBeing) {
//             float newCost = currentNode.G + (1 + map_[index + mapSize_.first]);

//             for (auto it=openSet.begin(); it!=openSet.end(); it++){
//                 if (node.coordinates == newNode.coordinates){
//                     if(newCost < node.G){
//                         it->parent = *currentNode;
//                         it->G = newCost;
//                     }
//                 }
//             }
            
//         }
//         for (auto node : openSet){
//             if(node.coordinates == )
//         }
//         //3
//         if (currentNode.coordinates.second > 0) {
//             Node.G = Node.G + (1 + map_[index + 1]);
//             Node.H = abs(goal.first - currentNode.first) 
//                       + abs(goal.second - currentNode.second);
//         }
//         //6
//         if (currentNode.coordinates.second < mapSize_.second) {
//             Node.G = Node.G + (1 + map_[index - mapSize_.first]);
//             Node.H = abs(goal.first - currentNode.first) 
//                       + abs(goal.second - currentNode.second);
//         }
//         //9
//         if (currentNode.coordinates.second > 0) {
//             Node.G = Node.G + (1 + map_[index - 1]);
//             Node.H = abs(goal.first - currentNode.first) 
//                       + abs(goal.second - currentNode.second);
//         }

//         Node *updateNode = 
//     }
//     return 0;
// }
