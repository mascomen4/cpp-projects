cmake_minimum_required(VERSION 3.19)
project(EAGLE_TextureMapping)

set(CMAKE_CXX_STANDARD 14)

include_directories(.)
include_directories(lib)
include_directories(rayint)
include_directories(rayint/acc)
include_directories(rayint/math)

add_executable(EAGLE_TextureMapping
        lib/Eagle_Utils.cpp
        lib/Eagle_Utils.h
        rayint/acc/acceleration.h
        rayint/acc/bvh_tree.h
        rayint/acc/defines.h
        rayint/acc/kd_tree.h
        rayint/acc/primitives.h
        rayint/math/algo.h
        rayint/math/defines.h
        rayint/math/vector.h
        getalignresults.cpp
        getalignresults.h
        main.cpp
        settings.h)
