#include "quad_rrt_planner.h"
#include "ros/ros.h"
#include <gtest/gtest.h>


class MyTestSuite : public ::testing::Test {
  public:
 // ros::NodeHandle nh;   
    MyTestSuite() {
      
    }
    ~MyTestSuite() {}
};

TEST_F(MyTestSuite, lowValue) {  
  //planner plan_ob(nh);
  int value = 1;
  ASSERT_EQ(value, 5) << "Value should be it's initial value plus 5";
}

TEST_F(MyTestSuite, highValue) {
  int value = 0;
  ASSERT_EQ(value, 0) << "Value should be 0";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_path_planning");
    testing::InitGoogleTest(&argc, argv);
    ros::NodeHandle nh;
    planner plan_ob(nh);
    auto res = RUN_ALL_TESTS();
    ros::shutdown();
    return res;
}