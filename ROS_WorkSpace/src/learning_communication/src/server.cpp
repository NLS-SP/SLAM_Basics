#include "ros/ros.h"
#include "learning_communication/AddTwoInts.h"

bool add(learning_communication::AddTwoInts::Request &req,
         learning_communication::AddTwoInts::Response &res){
  res.sum = req.a + req.b;
  ROS_INFO("request x=%ld, y = %ld", (long int)req.a, (long int)req.b);
  ROS_INFO("Sending back response: [%ld]", (long int)res.sum);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "add_two_ints_server");

  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);

  ROS_INFO("Ready to add tow ints");
  ros::spin();

  return 0;
}
