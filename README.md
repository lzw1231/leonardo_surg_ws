# leonardo_surg_ws

```
leonardo_surg_ws
├─ .clang-format
├─ README.md
├─ colcon_build.sh
├─ compile_commands.json
└─ src
   ├─ leonardo_bringup
   │  ├─ CMakeLists.txt
   │  ├─ config
   │  │  ├─ manager
   │  │  │  └─ leonardo_controller_manager.yaml
   │  │  ├─ mapping
   │  │  │  └─ pedal_mapping_controller.yaml
   │  │  ├─ mtm
   │  │  │  └─ fd_controllers.yaml
   │  │  ├─ psm
   │  │  │  └─ ft3215_controllers.yaml
   │  │  └─ teleop
   │  │     └─ teleop_params.yaml
   │  ├─ launch
   │  │  ├─ launch_utils.py
   │  │  └─ leonardo_bringup.launch.py
   │  └─ package.xml
   ├─ leonardo_controllers
   │  ├─ mtm_controllers
   │  │  ├─ CMakeLists.txt
   │  │  ├─ include
   │  │  │  └─ mtm_controllers
   │  │  │     ├─ common
   │  │  │     │  ├─ state_interface_utils.hpp
   │  │  │     │  └─ visibility_control.hpp
   │  │  │     ├─ fd_left
   │  │  │     │  └─ fd_left_ee_controller.hpp
   │  │  │     └─ fd_right
   │  │  │        └─ fd_right_ee_controller.hpp
   │  │  ├─ mtm_controllers_plugin.xml
   │  │  ├─ package.xml
   │  │  └─ src
   │  │     ├─ fd_left
   │  │     │  └─ fd_left_ee_controller.cpp
   │  │     └─ fd_right
   │  │        └─ fd_right_ee_controller.cpp
   │  ├─ psm_controllers
   │  │  ├─ CMakeLists.txt
   │  │  ├─ include
   │  │  │  └─ psm_controllers
   │  │  │     ├─ common
   │  │  │     │  └─ visibility_control.hpp
   │  │  │     └─ ft3215
   │  │  │        └─ ft3215_controller.hpp
   │  │  ├─ package.xml
   │  │  ├─ psm_controllers_plugin.xml
   │  │  └─ src
   │  │     └─ ft3215
   │  │        └─ ft3215_controller.cpp
   │  └─ teleop_controllers
   │     ├─ CMakeLists.txt
   │     ├─ include
   │     │  └─ teleop_controllers
   │     │     ├─ common
   │     │     │  └─ visibility_control.hpp
   │     │     └─ pedal_mapping
   │     │        └─ pedal_mapping_controller.hpp
   │     ├─ package.xml
   │     ├─ src
   │     │  └─ pedal_mapping
   │     │     └─ pedal_mapping_controller.cpp
   │     └─ teleop_controllers_plugin.xml
   ├─ leonardo_description
   │  ├─ CMakeLists.txt
   │  ├─ ft3215_description
   │  │  └─ ft3215.urdf.xacro
   │  ├─ mtm_description
   │  │  ├─ bringup
   │  │  │  └─ launch
   │  │  │     ├─ fd_left_view.launch.py
   │  │  │     └─ fd_right_view.launch.py
   │  │  ├─ rviz
   │  │  │  └─ fd_bimanual_config.rviz
   │  │  └─ urdf
   │  │     ├─ common
   │  │     │  ├─ fd.ros2_control_hardware.xacro
   │  │     │  └─ fd.urdf.xacro
   │  │     └─ fd_bimanual.config.xacro
   │  ├─ package.xml
   │  └─ urdf
   │     └─ leonardo_urdf.xacro
   ├─ leonardo_hardware
   │  ├─ dof4_hardware
   │  │  ├─ CMakeLists.txt
   │  │  └─ package.xml
   │  ├─ ecm_hardware
   │  │  ├─ CMakeLists.txt
   │  │  └─ package.xml
   │  ├─ ft3215_hardware
   │  │  ├─ CMakeLists.txt
   │  │  ├─ ft3215_hardware_plugin.xml
   │  │  ├─ include
   │  │  │  └─ ft3215_hardware
   │  │  │     └─ ft3215_hardware_interface.hpp
   │  │  ├─ package.xml
   │  │  └─ src
   │  │     └─ ft3215_hardware_interface.cpp
   │  ├─ mtm_hardware
   │  │  ├─ CMakeLists.txt
   │  │  ├─ include
   │  │  │  └─ mtm_hardware
   │  │  │     ├─ common
   │  │  │     │  ├─ math_utils.hpp
   │  │  │     │  └─ visibility_control.hpp
   │  │  │     ├─ fd_base
   │  │  │     │  └─ fd_hardware_base.hpp
   │  │  │     ├─ fd_left
   │  │  │     │  └─ fd_left_hardware.hpp
   │  │  │     └─ fd_right
   │  │  │        └─ fd_right_hardware.hpp
   │  │  ├─ mtm_hardware_plugin.xml
   │  │  ├─ package.xml
   │  │  └─ src
   │  │     ├─ fd_base
   │  │     │  └─ fd_hardware_base.cpp
   │  │     ├─ fd_left
   │  │     │  └─ fd_left_hardware.cpp
   │  │     └─ fd_right
   │  │        └─ fd_right_hardware.cpp
   │  └─ psm_hardware
   │     ├─ CMakeLists.txt
   │     ├─ include
   │     │  └─ psm_hardware
   │     │     ├─ common
   │     │     │  └─ visibility_control.hpp
   │     │     └─ ft3215
   │     │        └─ ft3215_hardware_interface.hpp
   │     ├─ package.xml
   │     └─ src
   │        └─ ft3215
   │           └─ ft3215_hardware_interface.cpp
   ├─ leonardo_teleop
   │  ├─ CMakeLists.txt
   │  ├─ include
   │  │  └─ leonardo_teleop
   │  │     └─ pedal_publisher.hpp
   │  ├─ package.xml
   │  └─ src
   │     └─ pedal_publisher.cpp
   ├─ leonardo_test
   │  ├─ CMakeLists.txt
   │  ├─ package.xml
   │  └─ src
   │     └─ vendor_test.cpp
   └─ leonardo_vendors
      ├─ fd_vendor
      │  ├─ CMakeLists.txt
      │  ├─ include
      │  │  └─ fd_vendor
      │  │     ├─ dhdc.h
      │  │     ├─ drdc.h
      │  │     └─ fd_sdk.hpp
      │  ├─ lib
      │  │  ├─ libdhd.so.3.17.8
      │  │  └─ libdrd.so.3.17.8
      │  └─ package.xml
      └─ sts_vendor
         ├─ CMakeLists.txt
         ├─ include
         │  └─ sts_vendor
         │     ├─ HLSCL.h
         │     ├─ INST.h
         │     ├─ SCS.h
         │     ├─ SCSCL.h
         │     ├─ SCSerial.h
         │     ├─ SCServo.h
         │     ├─ SCServo.hpp
         │     └─ SMS_STS.h
         ├─ lib
         │  └─ libSCServo.so
         └─ package.xml

```