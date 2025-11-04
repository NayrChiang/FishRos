#include "motion_control_system/motion_control_interface.hpp"
#include <pluginlib/class_loader.hpp>

int main(int argc, char **argv) {
  // Check if the number of arguments is valid
  if (argc != 2)
    return 0;
  // Select the plugin to load through command line arguments, argv[0] is the executable name, argv[1] represents the parameter name
  std::string controller_name = argv[1];
  // 1. Create a controller loader by package name and base class name
  pluginlib::ClassLoader<motion_control_system::MotionController>
      controller_loader("motion_control_system",
                        "motion_control_system::MotionController");
  // 2. Use the loader to load the plugin with the specified name, returns a pointer to an object of the specified plugin class
  auto controller = controller_loader.createSharedInstance(controller_name);
  // 3. Call the plugin's methods
  controller->start();
  controller->stop();
  return 0;
}