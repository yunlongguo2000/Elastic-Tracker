#pragma once

#include <mapping/mapping.h>
#include <ros/ros.h>

#include <Eigen/Core>
#include <queue>

namespace prediction {

struct Node {
  Eigen::Vector3d p, v, a;
  double t;
  double score;
  double h;
  Node* parent = nullptr;
};
typedef Node* NodePtr;
class NodeComparator {
 public:
  bool operator()(NodePtr& lhs, NodePtr& rhs) { //用于比较不同预测路径的好坏，分数越小的路径预测效果越好？
    return lhs->score + lhs->h > rhs->score + rhs->h;
  }
};
struct Predict {
 private:
  static constexpr int MAX_MEMORY = 1 << 22;
  // searching

  double dt;
  double pre_dur;
  double rho_a;
  double car_z, vmax;
  mapping::OccGridMap map;
  NodePtr data[MAX_MEMORY];
  int stack_top;

  inline bool isValid(const Eigen::Vector3d& p, const Eigen::Vector3d& v) const { //检查预测点的速度是否超过速度上限，以及预测点是否在障碍物内
    return (v.norm() < vmax) && (!map.isOccupied(p));
  }

 public:
  inline Predict(ros::NodeHandle& nh) {
    nh.getParam("tracking_dur", pre_dur);
    nh.getParam("tracking_dt", dt);
    nh.getParam("prediction/rho_a", rho_a);
    nh.getParam("prediction/vmax", vmax);
    for (int i = 0; i < MAX_MEMORY; ++i) {
      data[i] = new Node;
    }
  }
  inline void setMap(const mapping::OccGridMap& _map) {
    map = _map;
    // map.inflate_last();
  }

  inline bool predict(const Eigen::Vector3d& target_p,
                      const Eigen::Vector3d& target_v,
                      std::vector<Eigen::Vector3d>& target_predcit,
                      const double& max_time = 0.1) {
    auto score = [&](const NodePtr& ptr) -> double { //用于计算预测点的分数，预测点的加速度越小分数越小，应该是路径规划中f(n)=g(n)+h(n)的g(n)，即代价函数
      return rho_a * ptr->a.norm();
    };
    Eigen::Vector3d end_p = target_p + target_v * pre_dur; //假设在预测总时间内目标作匀速运动其最终的位置
    auto calH = [&](const NodePtr& ptr) -> double { //用于计算预测点与end_p之间的距离，应该是路径规划中f(n)=g(n)+h(n)的h(n)，即启发性函数
      return 0.001 * (ptr->p - end_p).norm();
    };
    ros::Time t_start = ros::Time::now();
    std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator> open_set;

    Eigen::Vector3d input(0, 0, 0);

    stack_top = 0; //堆栈指针
    NodePtr curPtr = data[stack_top++];
    curPtr->p = target_p;
    curPtr->v = target_v;
    curPtr->a.setZero();
    curPtr->parent = nullptr;
    curPtr->score = 0;
    curPtr->h = 0;
    curPtr->t = 0;
    double dt2_2 = dt * dt / 2;
    while (curPtr->t < pre_dur) {
      for (input.x() = -3; input.x() <= 3; input.x() += 3)
        for (input.y() = -3; input.y() <= 3; input.y() += 3) { //假设目标在dt的时间间隔内作匀加速运动，遍历9种可能的加速度得到9个预测点
          Eigen::Vector3d p = curPtr->p + curPtr->v * dt + input * dt2_2;
          Eigen::Vector3d v = curPtr->v + input * dt;
          if (!isValid(p, v)) {
            continue;
          }
          if (stack_top == MAX_MEMORY) {
            std::cout << "[prediction] out of memory!" << std::endl;
            return false;
          }
          double t_cost = (ros::Time::now() - t_start).toSec();
          if (t_cost > max_time) {
            std::cout << "[prediction] too slow!" << std::endl;
            return false;
          }
          NodePtr ptr = data[stack_top++];
          ptr->p = p;
          ptr->v = v;
          ptr->a = input;
          ptr->parent = curPtr; //有点像链表的感觉，一个一个parent相连最终形成目标的预测轨迹
          ptr->t = curPtr->t + dt;
          ptr->score = curPtr->score + score(ptr);
          ptr->h = calH(ptr);
          open_set.push(ptr);
          // std::cout << "open set push: " << state.transpose() << std::endl;
        }
      if (open_set.empty()) {
        std::cout << "[prediction] no way!" << std::endl;
        return false;
      }
      curPtr = open_set.top(); //open_set是一个先入先出的队列，每次都以队列的第一个预测点重新作为curPtr参与计算，类似广度优先搜索，最终得到的是在不同加速度序列下，在end_pt周围的许多不同预测点
      open_set.pop();
    }
    target_predcit.clear();
    while (curPtr != nullptr) {
      target_predcit.push_back(curPtr->p); //从最远预测点开始沿着parent指针向前倒退，存入target_predcit的尾部，直至预测的起始点
      curPtr = curPtr->parent;
    }
    std::reverse(target_predcit.begin(), target_predcit.end()); //颠倒一下target_predcit，让第一个预测点在第一个位置
    return true;
  }
};

}  // namespace prediction
