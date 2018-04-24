project(libowl VERSION 0.1 LANGUAGES CXX)

add_library(owl SHARED
        src/color/color_names.cpp
        src/color/gamma_correction.cpp
        src/image/image_io.cpp
        src/io/off.cpp
        src/io/ply.cpp
        src/utils/base64.cpp
        src/utils/buffer.cpp
        src/utils/file_utils.cpp
        src/utils/progress.cpp
        src/utils/signal.cpp
        src/utils/uuid.cpp
        src/utils/stop_watch.cpp
        include/owl/color/channel_traits.hpp
        include/owl/color/color.hpp
        include/owl/color/color_conversion.hpp
        include/owl/color/color_names.hpp
        include/owl/color/color_maps.hpp
        include/owl/color/gamma_correction.hpp
        include/owl/image/image.hpp
        include/owl/image/image_io.hpp
        include/owl/io/off.hpp
        include/owl/io/ply.hpp
        include/owl/math/angle.hpp
        include/owl/math/approx.hpp
        include/owl/math/constants.hpp
        include/owl/math/geometry/euler_angles.hpp
        include/owl/math/geometry/interval.hpp
        include/owl/math/geometry/line_segment.hpp
        include/owl/math/matrix.hpp
        include/owl/math/geometry/mesh.hpp
        include/owl/math/geometry/mesh_io.hpp
        include/owl/math/geometry/mesh_primitives.hpp
        include/owl/math/geometry/mesh_triangulation.hpp
        include/owl/math/geometry/nplane.hpp
        include/owl/math/geometry/physical_properties.hpp
        include/owl/math/geometry/quaternion.hpp
        include/owl/math/geometry/ray.hpp
        include/owl/math/geometry/trafos.hpp
        include/owl/math/geometry/triangle.hpp
        include/owl/math/utils.hpp
        include/owl/math/geometry/point_utils.hpp
        include/owl/utils/adjacent_iterator.hpp
        include/owl/utils/base64.hpp
        include/owl/utils/binary_serialization.hpp
        include/owl/utils/buffer.hpp
        include/owl/utils/connected_components.hpp
        include/owl/utils/container_utils.hpp
        include/owl/utils/count_iterator.hpp
        include/owl/utils/dynamic_properties.hpp
        include/owl/utils/file_utils.hpp
        include/owl/utils/filter_iterator.hpp
        include/owl/utils/handle.hpp
        include/owl/utils/hash_utils.hpp
        include/owl/utils/iterator_range.hpp
        include/owl/utils/lin_space.hpp
        include/owl/utils/linear_index.hpp
        include/owl/utils/map_iterator.hpp
        include/owl/utils/non_copyable.hpp
        include/owl/utils/progress.hpp
        include/owl/utils/random_utils.hpp
        include/owl/utils/range_algorithm.hpp
        include/owl/utils/step_iterator.hpp
        include/owl/utils/stop_watch.hpp
        include/owl/utils/template_utils.hpp
        include/owl/utils/uuid.hpp
        include/owl/export.hpp
        include/owl/optional.hpp
        include/owl/variant.hpp
        include/owl/math/geometry/aabb_tree.hpp
        include/owl/math/geometry/primitive_traits.hpp
        include/owl/math/geometry/distance.hpp
        include/owl/math/geometry/ray_segment.hpp
        include/owl/math/geometry/intersection.hpp)

target_include_directories(owl PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        PRIVATE src
        PRIVATE ../libs/include)


# Depend on a library that we defined in the top-level file
#target_link_libraries(owl
   #     boost)

# 'make install' to the correct locations (provided by GNUInstallDirs).
install(TARGETS owl EXPORT OwlConfig
        ARCHIVE  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        LIBRARY  DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME  DESTINATION ${CMAKE_INSTALL_BINDIR})  # This is for Windows
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# This makes the project importable from the install directory
# Put config file in per-project dir (name MUST match), can also
# just go into 'cmake'.
install(EXPORT OwlConfig DESTINATION share/OwlConfig/cmake)

# This makes the project importable from the build directory
export(TARGETS owl FILE OwlConfig.cmake)

# Every library has unit tests, of course
add_executable(testowl
        test/main.cpp
        test/color/color.cpp
        test/color/color_maps.cpp
        test/image/image.cpp
        test/math/angle.cpp
        test/math/interval.cpp
        test/math/matrix.cpp
        test/math/mesh.cpp
        test/math/nplane.cpp
        test/math/quaternion.cpp
        test/math/ray.cpp
        test/utils/binary_serialization.cpp
        test/utils/template_utils.cpp
        test/utils/container_traits.cpp
        test/utils/step_iterator.cpp
        test/utils/uuid.cpp
        test/utils/signal.cpp
        test/utils/base64.cpp
        test/utils/connected_components.cpp
        test/utils/progress.cpp
        test/math/physical_properties.cpp
        test/math/aabb_tree.cpp
        test/math/distance.cpp
        test/math/intersection.cpp)

target_link_libraries(testowl
        owl)
target_include_directories(testowl
        PRIVATE ../libs/include)

add_custom_command(
        TARGET testowl PRE_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory ${PROJECT_SOURCE_DIR}/test/data/images
        ${EXECUTABLE_OUTPUT_PATH}/images
        COMMENT "Copying image test data."
)

add_test(testowl testowl)


