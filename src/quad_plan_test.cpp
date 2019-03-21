#include "quad_rrt_planner.h"
#include "ros/ros.h"
#include <gtest/gtest.h>
#include <thread>
#include <chrono>

ros::NodeHandle nh;

class MyTestSuite : public ::testing::Test {
  public:
    MyTestSuite() {
    }
    ~MyTestSuite() {}
};

TEST_F(MyTestSuite, lowValue) {  
  int value = 5;
  ASSERT_EQ(value, 5) << "Value should be it's initial value plus 5";
}

TEST_F(MyTestSuite, highValue) {
  int value = 0;
  ASSERT_EQ(value, 0) << "Value should be 0";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "quad_unit_test");
    testing::InitGoogleTest(&argc, argv);
    // ros::NodeHandle nh;
    std::thread t([]{while(ros::ok()) ros::spin();});
    auto res = RUN_ALL_TESTS();
    ros::shutdown();
    return res;
}