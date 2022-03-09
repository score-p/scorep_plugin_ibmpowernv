cmake_minimum_required(VERSION 3.15)
project(scorep_plugin_ibmpowernv)

# there are some weird quirks in the regex compilation when using shared libraries
# on some system (including our test systems)
# this prevents this
add_compile_definitions(_GLIBCXX_USE_CXX11_ABI=0)

set(NITRO_POSITION_INDEPENDENT_CODE ON CACHE INTERNAL "")

find_package(Threads REQUIRED)

add_subdirectory(lib/scorep_plugin_cxx_wrapper)

add_library(ibmpowernv_plugin MODULE
    src/plugin.cpp
    include/plugin.hpp
    src/occ_sensor_t.cpp
    include/occ_sensor_t.hpp
    src/occ.cpp
    include/occ.hpp
    src/occ_util.cpp
    include/occ_util.hpp
    src/util.cpp
    include/util.hpp
)

add_library(ibmpowernv_sync_plugin MODULE
    src/plugin_sync.cpp
    include/plugin_sync.hpp
    src/occ_sensor_t.cpp
    include/occ_sensor_t.hpp
    src/occ.cpp
    include/occ.hpp
    src/occ_util.cpp
    include/occ_util.hpp
    src/util.cpp
    include/util.hpp
)

target_include_directories(ibmpowernv_plugin PUBLIC include)
target_include_directories(ibmpowernv_sync_plugin PUBLIC include)
target_compile_features(ibmpowernv_plugin PUBLIC cxx_std_17)
target_compile_features(ibmpowernv_sync_plugin PUBLIC cxx_std_17)
target_link_libraries(ibmpowernv_plugin PUBLIC Scorep::scorep-plugin-cxx Threads::Threads)
target_link_libraries(ibmpowernv_sync_plugin PUBLIC Scorep::scorep-plugin-cxx Threads::Threads)
install(
    TARGETS ibmpowernv_plugin ibmpowernv_sync_plugin
    LIBRARY DESTINATION lib
)
