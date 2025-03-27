# Copyright 2019-2023, Collabora, Ltd.
# Copyright 2023, Pluto VR, Inc.
#
# SPDX-License-Identifier: BSL-1.0

add_subdirectory(include)

add_subdirectory(proto ${CMAKE_CURRENT_BINARY_DIR}/proto)

add_library(ems_callbacks STATIC ems_callbacks.cpp)
target_link_libraries(
	ems_callbacks
	PUBLIC xrt-interfaces # monado stuff
	PRIVATE aux_util # monado stuff
	)

target_include_directories(ems_callbacks PUBLIC . ${GLIB_INCLUDE_DIRS})

add_subdirectory(gst)

add_library(comp_ems STATIC ems_compositor.cpp ems_compositor.h)
target_link_libraries(
	comp_ems
	PUBLIC xrt-interfaces # monado stuff
	PRIVATE
		aux_util # monado stuff
		aux_os # monado stuff
		aux_vk # monado stuff
		comp_util # monado stuff
		comp_multi # monado stuff
		ems_gst
	)
target_include_directories(comp_ems PUBLIC . ${GST_INCLUDE_DIRS})

add_library(drv_ems STATIC ems_hmd.cpp ems_motion_controller.cpp)

target_link_libraries(drv_ems PRIVATE xrt-interfaces aux_util em_proto ems_callbacks)

add_library(ems_streaming_server ems_instance.cpp ems_server_internal.h)

target_include_directories(ems_streaming_server PRIVATE ${GLIB_INCLUDE_DIRS} ${GST_INCLUDE_DIRS})

target_link_libraries(
	ems_streaming_server
	PRIVATE
		aux_util
		aux_util_debug_gui
		st_prober
		ipc_server
		drv_ems
		em_proto
		comp_ems
		ems_callbacks
		gstreamer_android
	)

target_link_libraries(ems_streaming_server PRIVATE st_gui aux_ogl)
