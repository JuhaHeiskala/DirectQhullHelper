cmake_minimum_required(VERSION 3.15)

project(DirectQhullHelper C)

add_library (DirectQhullHelper SHARED src/DirectQhullHelper.c)

target_include_directories (DirectQhullHelper PUBLIC ${QHULL_JLL_INCLUDE})

install(TARGETS DirectQhullHelper)
