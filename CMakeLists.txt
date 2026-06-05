cmake_minimum_required(VERSION 3.10)
project(TRAC_IK VERSION 1.0.0 LANGUAGES CXX)

# 通过 -DBUILD_SHARED_LIBS=ON 切换动态库/静态库构建
option(BUILD_SHARED_LIBS "构建动态库（OFF 则构建静态库）" OFF)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_INSTALL_DIR)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install)
endif ()


if(WIN32)
    # 自动检测vcpkg工具链文件
    if(DEFINED ENV{VCPKG_ROOT} AND NOT DEFINED CMAKE_TOOLCHAIN_FILE)
        set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake"
                CACHE STRING "")
    endif()

    # 处理Windows特有的库命名规则
    set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
    set(CMAKE_DEBUG_POSTFIX "_d")
endif()

# ---------- 源码文件 ----------
set(LIB_SOURCES
    src/kdl_tl.cpp
    src/nlopt_ik.cpp
    src/trac_ik.cpp
)
set(LIB_HEADERS
    include/kdl_tl.h
    include/nlopt_ik.h
    include/trac_ik.h
)

# 1. 创建库 trac_ik（由 BUILD_SHARED_LIBS 决定静态/动态）
add_library(trac_ik SHARED ${LIB_SOURCES} ${LIB_HEADERS})


# 2. 包含 trac_ik 自己的头文件目录
target_include_directories(trac_ik
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

# 3. 查找 vcpkg 安装的第三方库
find_package(nlopt REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# 4. 链接库到 trac_ik
target_link_libraries(trac_ik
    PUBLIC
        NLopt::nlopt
        orocos-kdl
        Eigen3::Eigen
)

# 5. 可执行文件 trac_ik_example
add_executable(trac_ik_example examples/example.cpp)



# 6. 链接 trac_ik 以及第三方库到可执行文件
target_link_libraries(trac_ik_example
    PRIVATE
        trac_ik
)

add_executable(irb2600_example examples/irb2600_test.cpp)

target_link_libraries(irb2600_example
        PRIVATE
        trac_ik
)

# ---------- 安装规则（可选）----------
include(GNUInstallDirs)
install(TARGETS trac_ik
        EXPORT TRAC_IKTargets
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS trac_ik_example
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# 安装头文件
install(FILES ${LIB_HEADERS} DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# ---------- 生成 find_package 配置文件 ----------
include(CMakePackageConfigHelpers)

# 生成版本文件
write_basic_package_version_file(
        "${CMAKE_CURRENT_BINARY_DIR}/TRAC_IKConfigVersion.cmake"
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion
)

# 从模板生成 Config 文件
configure_package_config_file(
        "${CMAKE_CURRENT_SOURCE_DIR}/cmake/TRAC_IKConfig.cmake.in"
        "${CMAKE_CURRENT_BINARY_DIR}/TRAC_IKConfig.cmake"
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/TRAC_IK
)

# 安装导出的 targets 文件
install(EXPORT TRAC_IKTargets
        FILE TRAC_IKTargets.cmake
        NAMESPACE TRAC_IK::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/TRAC_IK
)

# 安装 Config 和 ConfigVersion 文件
install(FILES
        "${CMAKE_CURRENT_BINARY_DIR}/TRAC_IKConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/TRAC_IKConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/TRAC_IK
)
