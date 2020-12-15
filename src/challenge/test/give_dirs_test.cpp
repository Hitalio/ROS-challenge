#include <gtest/gtest.h>
#include "ros/ros.h"

TEST(processInputTester, passingTest){

	float returnedFloat = process_input_coord("1");
	EXPECT_EQ(returnedFloat,1);

	returnedFloat = process_input_coord("5.6");
	EXPECT_EQ(returnedFloat,5.6);

	returnedFloat = process_input_coord("10");
	EXPECT_EQ(returnedFloat,10);

	returnedFloat = process_input_coord("1.34");
	EXPECT_EQ(returnedFloat,1.34);

	returnedFloat = process_input_coord("9");
	EXPECT_EQ(returnedFloat,9);

	returnedFloat = process_input_coord("9.4");
	EXPECT_EQ(returnedFloat,9.4);
}



int main(int argc, char** argv){
  testing::InitGoogleTest(&argc, argv);

  ros::init(argc, argv, "tester");
  ros::NodeHandle nh;

  return RUN_ALL_TESTS();
}
