cmake_minimum_required(VERSION 3.26)

project(couplvecs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(couplvecs main.cpp
               stl/coupled_vectors.h)

include(GNUInstallDirs)
install(TARGETS couplvecs
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
