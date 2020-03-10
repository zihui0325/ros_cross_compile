# Ros2 Cross-compiling for Hisi3559
1. install development tools
   ```
   sudo apt update && sudo apt install -y \
       cmake \
       git \
       wget \
       python3-pip \
       qemu-user-static \
       g++-aarch64-linux-gnu \
       g++-arm-linux-gnueabihf \
       pkg-config-aarch64-linux-gnu

   python3 -m pip install -U \
       vcstool \
       colcon-common-extensions
   ```
2. Download ROS2 Source Code
   ```
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos
   vcs-import src < ros2.repos
   git clone https://github.com/ros-tooling/cross_compile.git -b 0.0.1 src/ros2/cross_compile
   cd ..
   ```
3. Prepare the sysroot
   ```
   tar -zxvf aarch64-himix100-linux.tgz
   cd aarch64-himix100-linux
   chmod +x aarch64-himix100-linux.install
   ./aarch64-himix100-linux.install
   ```
   This will install hisi3559 sysroot to /opt/hisi-linux

4. cmake_toolchain_file
   ```
   set(CMAKE_SYSTEM_NAME Linux)
   set(CMAKE_SYSTEM_PROCESSOR aarch64)

   set(HISI_ROOT "/opt/hisi-linux/x86-arm/aarch64-himix100-linux")

   set(CMAKE_SYSROOT "${HISI_ROOT}/target")
   set(CMAKE_C_COMPILER "${HISI_ROOT}/bin/aarch64-himix100-linux-gcc")
   set(CMAKE_CXX_COMPILER "${HISI_ROOT}/bin/aarch64-himix100-linux-g++")


   list(APPEND CMAKE_FIND_ROOT_PATH "/home/r18119/hisi_target")
   list(APPEND CMAKE_FIND_ROOT_PATH "/home/r18119/ros2_ws/install")
   #list(APPEND CMAKE_FIND_ROOT_PATH "/usr")
   #SET(PYTHON_LIBRARIES "/usr/lib")
   #SET(PYTHON_INCLUDE_DIRS "/usr/include/")

   set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
   set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
   set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
   ```

5. colcon build
   ```
   colcon build --continue-on-error \
      --merge-install \
      --cmake-clean-cache \
      --cmake-force-configure \
      --cmake-args \
      -DCMAKE_TOOLCHAIN_FILE="/home/r18119/hisi_target/hisi_toolchain.cmake" \
      -DBUILD_TESTING=OFF \
      -DTHIRDPARTY=ON \
      -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON \
      -DSECURITY=ON
  
   colcon build --merge-install --cmake-force-configure --cmake-args -DCMAKE_TOOLCHAIN_FILE="/home/r18119/hisi_target/hisi_toolchain.cmake" -DBUILD_TESTING:BOOL=OFF -DTHIRDPARTY=ON -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON -DSECURITY=ON
   ```
   `-DTHIRDPARTY=ON` will use internal Asio and FastCDR when build FastRTPS
   `-FORCE_BUILD_VENDOR_PKG=ON` will build `tinyxml` `poco` 



# Cross Compile log4cxx
Log4cxx is default log system in ROS1 and ROS2, which is depend on apr and apr-util. Download apr-1.5.2.tar.gz, apr-util-1.5.4.tar.gz and libexpat first.
1. cross-compile apr
   ```
   vim include/apr_want.h
   #IFndef APR_IOVEC_DEFINED
   #define APR_IOVEC_DEFINED

   找到上面这两个，注释掉，改为
   #if  0

   编译过程中会生成gen_test_char，需要在主机上运行来test，所以先正常编译生成gen_test_char，再在交叉编译的时候不编译gen_test_char

   ./configure
   make
   cp -a tools/gen_test_char ../
   make clean

   ./configure --prefix=/home/r18119/hisi_target/usr/local CC=aarch64-himix100-linux-gcc CXX=aarch64-himix100-linux-g++ CPPFLAGS=-I/opt/hisi-linux/x86-arm/aarch64-himix100-linux/target/usr/include LDFLAGS=-L/opt/hisi-linux/x86-arm/aarch64-himix100-linux/target/usr/lib --host=aarch64-unknown-linux-gnu ac_cv_file__dev_zero=yes ac_cv_func_setpgrp_void=yes apr_cv_process_shared_works=yes apr_cv_mutex_robust_shared=yes apr_cv_tcp_nodelay_with_cork=yes ap_void_ptr_lt_long=no

   cp -a ../gen_test_char tools/

   vim Makefile
   132行
   OBJECTS_gen_test_char = tools/gen_test_char.lo $(LOCAL_LIBS)
   注释
   #OBJECTS_gen_test_char = tools/gen_test_char.lo $(LOCAL_LIBS)

   make
   make install
   ```
2. cross-compile libexpat

3. cross-compile apr-util
   ```
   vim xlate/xlate.c
   将handle_special_names函数中的
   return apr_os_default_encoding(pool);
   注释掉，改为
   return page;

   ./configure --host=aarch64-unknown-linux-gnu --prefix=/home/r18119/hisi_target/usr/local CC=aarch64-himix100-linux-gcc CXX=aarch64-himix100-linux-g++ --with-apr=/home/r18119/hisi_target/usr/local --with-expat=/home/r18119/hisi_target/usr/local
   ```
4. cross-compile log4cxx
   ```
   ./configure --host=aarch64-unknown-linux-gnu --prefix=/home/r18119/hisi_target/usr/local CC=aarch64-himix100-linux-gcc CXX=aarch64-himix100-linux-g++ --with-apr=/home/r18119/hisi_target/usr/local --with-apr-util=/home/r18119/hisi_target/usr/local

   Error1: 'memmove' was not declared in this scope
     src/main/cpp/inputstreamreader.cpp :
   add 
     #include <cstdio>
     #include <cstring>

   Error2: 'memcpy' was not declared in this scope
     src/main/cpp/socketoutputstream.cpp : 
   add
     #include <string.h>

   Error3: ‘puts’在此作用域中尚未声明
     src/examples/cpp/console.cpp : 
   add
     #include <string.h>
     #include <stdio.h>
   ```
# Cross Compile Python3
1. Download python3.6.9
2. configure
   
   ```
   ./configure --build=x86_64-pc-linux-gnu --host=aarch64-himix100-linux --prefix=/home/r18119/hisi_target/usr/local CC=aarch64-himix100-linux-gcc CXX=aarch64-himix100-linux-g++ --disable-ipv6 --enable-optimizations ac_cv_file__dev_ptmx=no ac_cv_file__dev_ptc=no

   make
   make install
   ```
# Cross Compile OpenSSL
1. Download source code
   ```
   git clone https://github.com/openssl/openssl.git
   ```
2. configure
   ```
   ./config no-asm shared --prefix=/home/r18119/hisi_target/usr/local --cross-compile-prefix=aarch64-himix100-linux-
   vim Makefile
   delete -m64
   make & make install
   ```
# Some Errors

1. python3: ImportError: No module named lark
   ```
   pip3 install lark-parser
   ```
2. No such file or directory : numpy/*.h
   ```
   pip3 install numy
   ```
   然后将site-packages里的numpy/core/include链接到`{PYTHON_INCLUDE_DIRS}`文件夹下

3. resource_retriever
   
   This package retrieves data from url-format files such as http://, ftp://, package:// file://, etc., and loads the data into memory. The package:// url for ros packages is translated into a local file:// url. The resourse retriever was initially designed to load mesh files into memory, but it can be used for any type of data. The resource retriever is based on the the `libcurl` library. 
   
   ros2编译时需要curl下载第三方包，所以需要找到编译主机版本的libcurl。同时resource_retriever作为ros2的一个包依赖于libcurl，所以还需要交叉编译目标平台的libcurl。但是两个版本的libcurl不能被同时find_package找到。

4. rclcpp_component
   
   交叉编译rclcpp_component，会出现错误:
   ```
   --- stderr: rclcpp_components
   /opt/hisi-linux/x86-arm/aarch64-himix100-linux/host_bin/../lib/gcc/aarch64-linux-gnu/6.3.0/../../../../aarch64-linux-gnu/bin/ld: warning: librosidl_typesupport_fastrtps_cpp.so, needed by /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so, not found (try using -rpath or -rpath-link)
   /opt/hisi-linux/x86-arm/aarch64-himix100-linux/host_bin/../lib/gcc/aarch64-linux-gnu/6.3.0/../../../../aarch64-linux-gnu/bin/ld: warning: librosidl_typesupport_fastrtps_c.so, needed by /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so, not found (try using -rpath or -rpath-link)
   /opt/hisi-linux/x86-arm/aarch64-himix100-linux/host_bin/../lib/gcc/aarch64-linux-gnu/6.3.0/../../../../aarch64-linux-gnu/bin/ld: warning: libfastrtps.so.1, needed by /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so, not found (try using -rpath or -rpath-link)
   /opt/hisi-linux/x86-arm/aarch64-himix100-linux/host_bin/../lib/gcc/aarch64-linux-gnu/6.3.0/../../../../aarch64-linux-gnu/bin/ld: warning: libssl.so.3, needed by /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so, not found (try using -rpath or -rpath-link)
   /opt/hisi-linux/x86-arm/aarch64-himix100-linux/host_bin/../lib/gcc/aarch64-linux-gnu/6.3.0/../../../../aarch64-linux-gnu/bin/ld: warning: libcrypto.so.3, needed by /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so, not found (try using -rpath or -rpath-link)
   /opt/hisi-linux/x86-arm/aarch64-himix100-linux/host_bin/../lib/gcc/aarch64-linux-gnu/6.3.0/../../../../aarch64-linux-gnu/bin/ld: warning: libfastcdr.so.1, needed by /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so, not found (try using -rpath or -rpath-link)
   /opt/hisi-linux/x86-arm/aarch64-himix100-linux/host_bin/../lib/gcc/aarch64-linux-gnu/6.3.0/../../../../aarch64-linux-gnu/bin/ld: warning: libyaml.so, needed by /home/r18119/ros_hisi/dashing/install/lib/librcl_yaml_param_parser.so, not found (try using -rpath or -rpath-link)
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::deserialize(char&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so: undefined reference to `eprosima::fastcdr::Cdr::serializeBoolSequence(std::vector<bool, std::allocator<bool> > const&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `rosidl_typesupport_fastrtps_c__identifier'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serialize(double)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so: undefined reference to `eprosima::fastcdr::Cdr::state::state(eprosima::fastcdr::Cdr const&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::deserializeArray(char*, unsigned long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_yaml_param_parser.so: undefined reference to `yaml_parser_delete'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serializeArray(bool const*, unsigned long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::deserializeArray(double*, unsigned long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_yaml_param_parser.so: undefined reference to `yaml_parser_set_input_file'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serializeArray(long const*, unsigned long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_yaml_param_parser.so: undefined reference to `yaml_event_delete'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serialize(int)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::deserialize(int&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_yaml_param_parser.so: undefined reference to `yaml_parser_parse'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::deserialize(long&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so: undefined reference to `eprosima::fastcdr::Cdr::deserializeBoolSequence(std::vector<bool, std::allocator<bool> >&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so: undefined reference to `eprosima::fastcdr::Cdr::setState(eprosima::fastcdr::Cdr::state&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serialize(long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so: undefined reference to `rosidl_typesupport_fastrtps_cpp::typesupport_identifier'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::deserializeArray(long*, unsigned long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serializeArray(double const*, unsigned long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_cpp.so: undefined reference to `typeinfo for eprosima::fastcdr::exception::Exception'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_yaml_param_parser.so: undefined reference to `yaml_parser_initialize'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::readString(unsigned int&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serializeArray(char const*, unsigned long)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serialize(bool)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::deserialize(double&)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serialize(char const*)'
   /home/r18119/ros_hisi/dashing/install/lib/librcl_interfaces__rosidl_typesupport_fastrtps_c.so: undefined reference to `eprosima::fastcdr::Cdr::serialize(char)'
   collect2: error: ld returned 1 exit status
   make[2]: *** [component_container_mt] Error 1
   make[1]: *** [CMakeFiles/component_container_mt.dir/all] Error 2
   make: *** [all] Error 2
   ```

   在`ros2/rclcpp/rclcpp_components/CMakeLists.txt`中添加
   ```
    find_package(rmw_fastrtps_cpp REQUIRED)
    find_package(yaml REQUIRED)

    target_link_libraries(component_container component_manager)
    ament_target_dependencies(component_container
      "rclcpp"
      "rmw_fastrtps_cpp"
      "yaml"
    )

    target_link_libraries(component_container_mt component_manager)
    ament_target_dependencies(component_container_mt
      "rclcpp"
      "rmw_fastrtps_cpp"
      "yaml"
    )
   ```

5. tf2_ros
   
   如上类似问题, 在`ros2/geometry2/tf2_ros/CMakeLists.txt`中添加
   ```
   find_package(rmw_fastrtps_cpp REQUIRED)
   find_package(yaml REQUIRED)
   find_package(class_loader REQUIRED)

   target_link_libraries(static_transform_publisher
     ${PROJECT_NAME}
     ${geometry_msgs_LIBRARIES}
     ${rclcpp_LIBRARIES}
     ${tf2_LIBRARIES}
     ${tf2_msgs_LIBRARIES}
     ${rmw_fastrtps_cpp_LIBRARIES}
     ${yaml_LIBRARIES}
     ${class_loader_LIBRARIES}
   )

   target_link_libraries(tf2_echo
     ${PROJECT_NAME}
     ${geometry_msgs_LIBRARIES}
     ${rclcpp_LIBRARIES}
     ${tf2_LIBRARIES}
     ${tf2_msgs_LIBRARIES}
     ${rmw_fastrtps_cpp_LIBRARIES}
     ${yaml_LIBRARIES}
     ${class_loader_LIBRARIES}
   )

   target_link_libraries(tf2_monitor
     ${PROJECT_NAME}
     ${geometry_msgs_LIBRARIES}
     ${rclcpp_LIBRARIES}
     ${tf2_LIBRARIES}
     ${tf2_msgs_LIBRARIES}
     ${rmw_fastrtps_cpp_LIBRARIES}
     ${yaml_LIBRARIES}
     ${class_loader_LIBRARIES}
   )
   ```
6. tlsf_cpp
7. 4 5 6问题可归类与rclcpp动态库链接问题。4 5 6都使用了`ament_target_dependencies(target rclcpp)`，按照`ament_target_dependencies`设计原则，会递归的链接rclcpp所链接的库，但是在交叉编译的时候没有生效，正常本机编译则没有这个问题，可能是由于交叉编译cmake_toolchain_file 设置了sysroot的原因？



#### Dependencies
1. openssl when `-DSECURITY=ON`
2. log4cxx
3. python3.6
4. 

### Another Way
1. Download ros2-arm64.tar.bz
2. cross compile `Poco` `tinyxml2` `openssl 1.1.1`, install to ros2 directory
3. run app in hisi3559 directly
