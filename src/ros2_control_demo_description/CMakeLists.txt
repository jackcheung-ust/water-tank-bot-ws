cmake_minimum_required(VERSION 3.5)
project(ros2_control_demo_description)

find_package(ament_cmake REQUIRED)

# install(
#   DIRECTORY rrbot/urdf rrbot/rviz
#   DESTINATION share/${PROJECT_NAME}/rrbot
# )

install(
  DIRECTORY diffbot/urdf diffbot/rviz
  DESTINATION share/${PROJECT_NAME}/diffbot
)

# install(
#   DIRECTORY carlikebot/urdf carlikebot/rviz
#   DESTINATION share/${PROJECT_NAME}/carlikebot
# )

# install(
#   DIRECTORY r3bot/urdf
#   DESTINATION share/${PROJECT_NAME}/r3bot
# )

# install(
#   DIRECTORY r6bot/meshes r6bot/srdf r6bot/urdf r6bot/rviz
#   DESTINATION share/${PROJECT_NAME}/r6bot
# )

ament_package()
