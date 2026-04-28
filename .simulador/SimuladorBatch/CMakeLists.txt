cmake_minimum_required(VERSION 3.10)
project(SimuladorEVTOL_Batch)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Localizar apenas dependências de lógica e Python
find_package(pybind11 REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development.Embed)

# 2. Incluir diretórios de headers
include_directories(/usr/include/glm)
include_directories(${Python3_INCLUDE_DIRS})

# 3. Definir apenas o executável de Batch (Sem interface gráfica)
add_executable(batch_sim 
    SimuladorBatch.cpp 
    SimuladorCore.cpp
)

# 4. Vincular bibliotecas necessárias para a FSM e Física
target_link_libraries(batch_sim 
    pybind11::embed
    ${Python3_LIBRARIES}
    pthread 
    m
)

# 5. Simulador gráfico (com Raylib)
find_package(raylib QUIET)

add_executable(sim_grafico
    main.cpp
    Renderer.cpp
    SimuladorCore.cpp
)

target_include_directories(sim_grafico PRIVATE
    /usr/local/include
    /usr/include/glm
    ${Python3_INCLUDE_DIRS}
)

if(raylib_FOUND)
    target_link_libraries(sim_grafico PRIVATE raylib)
else()
    target_link_libraries(sim_grafico PRIVATE
        ${CMAKE_SOURCE_DIR}/../libs/raylib/lib/libraylib.a
    )
    target_include_directories(sim_grafico PRIVATE
        ${CMAKE_SOURCE_DIR}/../libs/raylib/include
    )
endif()

target_link_libraries(sim_grafico PRIVATE
    GL m pthread dl rt X11
)