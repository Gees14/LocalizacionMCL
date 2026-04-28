cmake_minimum_required(VERSION 3.8)
project(puzzlebot_gz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  worlds
  config
  urdf
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Regenerate SDF from URDF whenever it is stale
# Run manually: gz sdf -p urdf/puzzlebot_gz.urdf > urdf/puzzlebot_gz.sdf

ament_package()
