cmake_minimum_required(VERSION 3.10)
project(RobotVisionCpp)

# Устанавливаем стандарт C++17 (требуется для Open3D)
set(CMAKE_CXX_STANDARD 17)

# Пытаемся найти установленную на компьютере библиотеку Open3D
find_package(Open3D REQUIRED)

# Говорим создать исполняемый файл из нашего кода
add_executable(robot_vision main.cpp)

# Привязываем библиотеку Open3D к нашему коду
target_link_libraries(robot_vision Open3D::Open3D)