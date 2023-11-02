find_package(xbot2 REQUIRED)
find_package(gazebo_ocs2_ros_interfaces REQUIRED)

set(CMAKE_CXX_STANDARD 17)

find_package(tf REQUIRED)

add_library(visualizer
#    ${catkin_LIBRARIES}
#    ${ocs2_centauro_LIBRARIES}
    ../visualization/LeggedRobotVisualizer.cpp
)

target_link_libraries(visualizer
    ${catkin_LIBRARIES}
    )

add_xbot2_plugin(mrt_plugin
    mrtplugin.cpp)

target_link_libraries(mrt_plugin
    PRIVATE
    ${catkin_LIBRARIES}
    visualizer
    )

install(
    TARGETS mrt_plugin
    DESTINATION lib
    )
