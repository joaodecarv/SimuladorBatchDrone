cmake_minimum_required(VERSION 3.8)
project(Meu_nohEVTOL)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --------------------------------------------------------------------------
# Dependências
# --------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp       REQUIRED)
find_package(px4_msgs     REQUIRED)
find_package(custom_msgs  REQUIRED)
find_package(std_msgs     REQUIRED)
find_package(Eigen3       REQUIRED)

# --------------------------------------------------------------------------
# Biblioteca compartilhada: DroneSimNode + InterfaceFSM
# Reutilizável por outros nós ou testes
# --------------------------------------------------------------------------
add_library(evtol_sim_lib SHARED
  src/drone_sim_node.cpp
  src/interfaceFSM.cpp
)

target_include_directories(evtol_sim_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(evtol_sim_lib
  rclcpp
  px4_msgs
  custom_msgs
  std_msgs
)

# --------------------------------------------------------------------------
# Executável: simulador em lote (nova arquitetura — FSM real da sae_2025)
#   Fontes: batch_simulator.cpp (nó ROS2) + my_fsm.cpp (troque pelo seu faseX.cpp)
#   Não depende de px4_msgs — usa Drone drop-in (include/drone/Drone.hpp)
# --------------------------------------------------------------------------
add_executable(batch_simulator
  src/batch_simulator.cpp
  src/my_fsm.cpp
)

target_include_directories(batch_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(batch_simulator Eigen3::Eigen)

ament_target_dependencies(batch_simulator rclcpp)

# --------------------------------------------------------------------------
# Instalação
# --------------------------------------------------------------------------
install(TARGETS
  evtol_sim_lib
  batch_simulator
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  OPTIONAL
)

# --------------------------------------------------------------------------
# Testes
# --------------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
