cmake_minimum_required(VERSION 3.15)
project(Simulador1drone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Headers de outros módulos do projeto
set(CORE_DIR     ${CMAKE_SOURCE_DIR}/../Simulador_Core)
set(RENDERER_DIR ${CMAKE_SOURCE_DIR}/../Renderizador)

add_executable(sim1drone
    main.cpp
    ${CORE_DIR}/SimuladorCore.cpp
    ${RENDERER_DIR}/Renderer.cpp
)

target_include_directories(sim1drone PRIVATE
    ${CORE_DIR}
    ${RENDERER_DIR}
    /usr/local/include
    /usr/include/glm
)

find_package(raylib QUIET)

if(raylib_FOUND)
    target_link_libraries(sim1drone PRIVATE raylib)
else()
    target_link_libraries(sim1drone PRIVATE
        ${CMAKE_SOURCE_DIR}/../libs/raylib/lib/libraylib.a
    )
    target_include_directories(sim1drone PRIVATE
        ${CMAKE_SOURCE_DIR}/../libs/raylib/include
    )
endif()

target_link_libraries(sim1drone PRIVATE
    GL m pthread dl rt X11
)
