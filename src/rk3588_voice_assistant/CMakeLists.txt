cmake_minimum_required(VERSION 3.16)
project(rk3588_voice_assistant)

# ============================================================================
# ROS2
# ============================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rk3588_voice_assistant_interfaces REQUIRED)

# ============================================================================
# C++17
# ============================================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# 路径（自包含，所有代码在本目录内）
# ============================================================================
set(ENGINE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/engine)
set(TP_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/third_party)

set(RKLLM_DIR  ${TP_DIR}/rkllm)
set(RKNN_DIR   ${TP_DIR}/rknn)
set(SHERPA_DIR ${TP_DIR}/sherpa-onnx)
set(FVAD_DIR   ${TP_DIR}/libfvad)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux" AND CMAKE_SIZEOF_VOID_P EQUAL 8)
    set(TARGET_ARCH aarch64)
else()
    set(TARGET_ARCH armhf)
endif()

# ============================================================================
# miniaudio (header-only)
# ============================================================================
add_library(miniaudio INTERFACE)
target_include_directories(miniaudio INTERFACE ${TP_DIR}/miniaudio)

# ============================================================================
# libfvad
# ============================================================================
if(EXISTS "${FVAD_DIR}/CMakeLists.txt")
    add_subdirectory(${FVAD_DIR} ${CMAKE_BINARY_DIR}/fvad_build EXCLUDE_FROM_ALL)
endif()

# ============================================================================
# audio_capture
# ============================================================================
add_library(audio_capture STATIC ${ENGINE_DIR}/audio/audio_capture.cpp)
target_include_directories(audio_capture PUBLIC ${ENGINE_DIR} ${TP_DIR}/miniaudio)

# ============================================================================
# rkllm_engine
# ============================================================================
add_library(rkllm_engine STATIC ${ENGINE_DIR}/llm/rkllm_engine.cpp)
target_include_directories(rkllm_engine PUBLIC ${ENGINE_DIR} ${RKLLM_DIR}/include)

# ============================================================================
# vad_engine
# ============================================================================
if(TARGET fvad)
    add_library(vad_engine STATIC ${ENGINE_DIR}/vad/vad_engine.cpp)
    target_include_directories(vad_engine PUBLIC ${ENGINE_DIR} ${FVAD_DIR}/include)
    target_link_libraries(vad_engine PUBLIC fvad)
    target_compile_definitions(vad_engine PUBLIC HAS_VAD=1)
endif()

# ============================================================================
# sherpa_asr
# ============================================================================
if(EXISTS "${SHERPA_DIR}/lib/libsherpa-onnx-c-api.so")
    add_library(sherpa_asr STATIC ${ENGINE_DIR}/asr/sherpa_asr.cpp)
    target_include_directories(sherpa_asr PUBLIC ${ENGINE_DIR} ${SHERPA_DIR}/include)
    target_compile_definitions(sherpa_asr PUBLIC HAS_ASR=1)
endif()

# ============================================================================
# RGA
# ============================================================================
set(RGA_DIR ${TP_DIR}/rga)
if(EXISTS "${RGA_DIR}/include/rga.h")
    add_library(rga_headers INTERFACE)
    target_include_directories(rga_headers INTERFACE ${RGA_DIR}/include)
endif()

# ============================================================================
# yolo_engine (+ camera_capture)
# ============================================================================
if(EXISTS "${RKNN_DIR}/aarch64/librknnrt.so")
    add_library(yolo_engine STATIC
        ${ENGINE_DIR}/yolo/yolo_engine.cpp
        ${ENGINE_DIR}/yolo/camera_capture.cpp)
    target_include_directories(yolo_engine PUBLIC
        ${ENGINE_DIR} ${RKNN_DIR}/include)
    if(TARGET rga_headers)
        target_include_directories(yolo_engine PUBLIC ${RGA_DIR}/include)
        target_compile_definitions(yolo_engine PUBLIC HAS_RGA=1)
    endif()
    target_link_libraries(yolo_engine PUBLIC ${RKNN_DIR}/${TARGET_ARCH}/librknnrt.so)
endif()

# ============================================================================
# audio_vad_node
# ============================================================================
add_executable(audio_vad_node nodes/audio_vad_node.cpp)
target_include_directories(audio_vad_node PRIVATE ${ENGINE_DIR} include)
ament_target_dependencies(audio_vad_node rclcpp rclcpp_lifecycle std_msgs std_srvs)
target_link_libraries(audio_vad_node rk3588_voice_assistant_interfaces::rk3588_voice_assistant_interfaces)
target_link_libraries(audio_vad_node audio_capture pthread dl)
if(TARGET vad_engine)
    target_link_libraries(audio_vad_node vad_engine)
    target_compile_definitions(audio_vad_node PRIVATE HAS_VAD=1)
endif()

# ============================================================================
# asr_node
# ============================================================================
add_executable(asr_node nodes/asr_node.cpp)
target_include_directories(asr_node PRIVATE ${ENGINE_DIR} include)
ament_target_dependencies(asr_node rclcpp rclcpp_lifecycle std_msgs)
target_link_libraries(asr_node rk3588_voice_assistant_interfaces::rk3588_voice_assistant_interfaces)
if(TARGET sherpa_asr)
    target_link_libraries(asr_node sherpa_asr
        ${SHERPA_DIR}/lib/libsherpa-onnx-c-api.so
        ${SHERPA_DIR}/lib/libonnxruntime.so)
    target_compile_definitions(asr_node PRIVATE HAS_ASR=1)
endif()
target_link_libraries(asr_node pthread dl)

# ============================================================================
# llm_node — 含 YOLO Action client
# ============================================================================
add_executable(llm_node nodes/llm_node.cpp)
target_include_directories(llm_node PRIVATE ${ENGINE_DIR} include)
ament_target_dependencies(llm_node rclcpp rclcpp_lifecycle rclcpp_action std_msgs std_srvs)
target_link_libraries(llm_node rk3588_voice_assistant_interfaces::rk3588_voice_assistant_interfaces)
target_link_libraries(llm_node rkllm_engine pthread dl)
if(EXISTS "${RKLLM_DIR}/${TARGET_ARCH}/librkllmrt.so")
    target_link_libraries(llm_node ${RKLLM_DIR}/${TARGET_ARCH}/librkllmrt.so)
endif()

# ============================================================================
# yolo_node
# ============================================================================
if(TARGET yolo_engine)
    add_executable(yolo_node nodes/yolo_node.cpp)
    target_include_directories(yolo_node PRIVATE ${ENGINE_DIR} include)
    ament_target_dependencies(yolo_node rclcpp rclcpp_lifecycle rclcpp_action)
    target_link_libraries(yolo_node rk3588_voice_assistant_interfaces::rk3588_voice_assistant_interfaces)
    target_link_libraries(yolo_node yolo_engine pthread dl)
    # RGA 系统库（板端 /usr/lib/aarch64-linux-gnu/librga.so）
    find_library(RGA_LIB rga)
    if(RGA_LIB)
        target_link_libraries(yolo_node ${RGA_LIB})
        target_compile_definitions(yolo_node PRIVATE HAS_RGA=1)
    endif()
endif()

# ============================================================================
# Install
# ============================================================================
install(TARGETS audio_vad_node asr_node llm_node yolo_node
    DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config
    DESTINATION share/${PROJECT_NAME})
install(DIRECTORY third_party/rkllm/aarch64/ DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.so")
install(DIRECTORY third_party/rknn/aarch64/ DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.so")
install(DIRECTORY third_party/sherpa-onnx/lib/ DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.so*")

ament_package()
