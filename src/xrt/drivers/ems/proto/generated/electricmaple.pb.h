/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.7 */

#ifndef PB_EM_PROTO_ELECTRICMAPLE_PB_H_INCLUDED
#define PB_EM_PROTO_ELECTRICMAPLE_PB_H_INCLUDED

#include "../../../../../external/nanopb/pb.h"
#include <stdint.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
/* todo: make this bitflags, make this support "inferred" */
typedef enum _em_proto_TrackedStatus {
    em_proto_TrackedStatus_UNTRACKED = 0,
    em_proto_TrackedStatus_TRACKED = 1
} em_proto_TrackedStatus;

/* Struct definitions */
typedef struct _em_proto_Quaternion {
    float w;
    float x;
    float y;
    float z;
} em_proto_Quaternion;

typedef struct _em_proto_Vec3 {
    float x;
    float y;
    float z;
} em_proto_Vec3;

typedef struct _em_proto_Vec2 {
    float x;
    float y;
} em_proto_Vec2;

typedef struct _em_proto_Pose {
    bool has_position;
    em_proto_Vec3 position;
    bool has_orientation;
    em_proto_Quaternion orientation;
} em_proto_Pose;

typedef struct _em_proto_TrackingMessage {
    bool has_P_localSpace_viewSpace;
    em_proto_Pose P_localSpace_viewSpace;
    bool has_P_viewSpace_view0;
    em_proto_Pose P_viewSpace_view0; /* Left view */
    bool has_P_viewSpace_view1;
    em_proto_Pose P_viewSpace_view1; /* Right view */
    bool has_P_local_controller_grip_left;
    em_proto_Pose P_local_controller_grip_left;
    bool has_controller_aim_left;
    em_proto_Pose controller_aim_left;
    bool has_controller_grip_right;
    em_proto_Pose controller_grip_right;
    bool has_controller_aim_right;
    em_proto_Pose controller_aim_right;
    int64_t timestamp;
    int64_t sequence_idx;
} em_proto_TrackingMessage;

typedef struct _em_proto_InputThumbstick {
    bool has_xy;
    em_proto_Vec2 xy;
    bool click;
    bool touch; /* bool proximity = 4; */
} em_proto_InputThumbstick;

typedef struct _em_proto_InputValueTouch {
    float value;
    bool touch;
} em_proto_InputValueTouch;

typedef struct _em_proto_InputClickTouch {
    bool click;
    bool touch;
} em_proto_InputClickTouch;

typedef struct _em_proto_TouchControllerCommon {
    bool has_thumbstick;
    em_proto_InputThumbstick thumbstick;
    bool has_trigger;
    em_proto_InputValueTouch trigger;
    bool has_squeeze;
    em_proto_InputValueTouch squeeze; /* no "touch" */
    bool thumbrest_touch;
} em_proto_TouchControllerCommon;

typedef struct _em_proto_TouchControllerLeft {
    bool has_x;
    em_proto_InputClickTouch x;
    bool has_y;
    em_proto_InputClickTouch y;
    bool has_menu;
    em_proto_InputClickTouch menu; /* no "touch" */
    bool has_common;
    em_proto_TouchControllerCommon common;
} em_proto_TouchControllerLeft;

typedef struct _em_proto_TouchControllerRight {
    bool has_a;
    em_proto_InputClickTouch a;
    bool has_b;
    em_proto_InputClickTouch b;
    bool has_system;
    em_proto_InputClickTouch system; /* no "touch", probably not accessible */
    bool has_common;
    em_proto_TouchControllerCommon common;
} em_proto_TouchControllerRight;

typedef struct _em_proto_UpFrameMessage {
    int64_t frame_sequence_id;
    int64_t decode_complete_time; /* nanoseconds, in client OpenXR time domain */
    int64_t begin_frame_time; /* nanoseconds, in client OpenXR time domain */
    int64_t display_time; /* nanoseconds, in client OpenXR time domain */
} em_proto_UpFrameMessage;

typedef struct _em_proto_UpMessage {
    int64_t up_message_id;
    bool has_tracking;
    em_proto_TrackingMessage tracking;
    bool has_frame;
    em_proto_UpFrameMessage frame;
} em_proto_UpMessage;

typedef struct _em_proto_DownFrameDataMessage {
    int64_t frame_sequence_id;
    bool has_P_localSpace_viewSpace;
    em_proto_Pose P_localSpace_viewSpace;
    int64_t display_time; /* TODO fovs here */
} em_proto_DownFrameDataMessage;

typedef struct _em_proto_DownMessage {
    bool has_frame_data;
    em_proto_DownFrameDataMessage frame_data;
} em_proto_DownMessage;


#ifdef __cplusplus
extern "C" {
#endif

/* Helper constants for enums */
#define _em_proto_TrackedStatus_MIN em_proto_TrackedStatus_UNTRACKED
#define _em_proto_TrackedStatus_MAX em_proto_TrackedStatus_TRACKED
#define _em_proto_TrackedStatus_ARRAYSIZE ((em_proto_TrackedStatus)(em_proto_TrackedStatus_TRACKED+1))

















/* Initializer values for message structs */
#define em_proto_Quaternion_init_default         {0, 0, 0, 0}
#define em_proto_Vec3_init_default               {0, 0, 0}
#define em_proto_Vec2_init_default               {0, 0}
#define em_proto_Pose_init_default               {false, em_proto_Vec3_init_default, false, em_proto_Quaternion_init_default}
#define em_proto_TrackingMessage_init_default    {false, em_proto_Pose_init_default, false, em_proto_Pose_init_default, false, em_proto_Pose_init_default, false, em_proto_Pose_init_default, false, em_proto_Pose_init_default, false, em_proto_Pose_init_default, false, em_proto_Pose_init_default, 0, 0}
#define em_proto_InputThumbstick_init_default    {false, em_proto_Vec2_init_default, 0, 0}
#define em_proto_InputValueTouch_init_default    {0, 0}
#define em_proto_InputClickTouch_init_default    {0, 0}
#define em_proto_TouchControllerCommon_init_default {false, em_proto_InputThumbstick_init_default, false, em_proto_InputValueTouch_init_default, false, em_proto_InputValueTouch_init_default, 0}
#define em_proto_TouchControllerLeft_init_default {false, em_proto_InputClickTouch_init_default, false, em_proto_InputClickTouch_init_default, false, em_proto_InputClickTouch_init_default, false, em_proto_TouchControllerCommon_init_default}
#define em_proto_TouchControllerRight_init_default {false, em_proto_InputClickTouch_init_default, false, em_proto_InputClickTouch_init_default, false, em_proto_InputClickTouch_init_default, false, em_proto_TouchControllerCommon_init_default}
#define em_proto_UpFrameMessage_init_default     {0, 0, 0, 0}
#define em_proto_UpMessage_init_default          {0, false, em_proto_TrackingMessage_init_default, false, em_proto_UpFrameMessage_init_default}
#define em_proto_DownFrameDataMessage_init_default {0, false, em_proto_Pose_init_default, 0}
#define em_proto_DownMessage_init_default        {false, em_proto_DownFrameDataMessage_init_default}
#define em_proto_Quaternion_init_zero            {0, 0, 0, 0}
#define em_proto_Vec3_init_zero                  {0, 0, 0}
#define em_proto_Vec2_init_zero                  {0, 0}
#define em_proto_Pose_init_zero                  {false, em_proto_Vec3_init_zero, false, em_proto_Quaternion_init_zero}
#define em_proto_TrackingMessage_init_zero       {false, em_proto_Pose_init_zero, false, em_proto_Pose_init_zero, false, em_proto_Pose_init_zero, false, em_proto_Pose_init_zero, false, em_proto_Pose_init_zero, false, em_proto_Pose_init_zero, false, em_proto_Pose_init_zero, 0, 0}
#define em_proto_InputThumbstick_init_zero       {false, em_proto_Vec2_init_zero, 0, 0}
#define em_proto_InputValueTouch_init_zero       {0, 0}
#define em_proto_InputClickTouch_init_zero       {0, 0}
#define em_proto_TouchControllerCommon_init_zero {false, em_proto_InputThumbstick_init_zero, false, em_proto_InputValueTouch_init_zero, false, em_proto_InputValueTouch_init_zero, 0}
#define em_proto_TouchControllerLeft_init_zero   {false, em_proto_InputClickTouch_init_zero, false, em_proto_InputClickTouch_init_zero, false, em_proto_InputClickTouch_init_zero, false, em_proto_TouchControllerCommon_init_zero}
#define em_proto_TouchControllerRight_init_zero  {false, em_proto_InputClickTouch_init_zero, false, em_proto_InputClickTouch_init_zero, false, em_proto_InputClickTouch_init_zero, false, em_proto_TouchControllerCommon_init_zero}
#define em_proto_UpFrameMessage_init_zero        {0, 0, 0, 0}
#define em_proto_UpMessage_init_zero             {0, false, em_proto_TrackingMessage_init_zero, false, em_proto_UpFrameMessage_init_zero}
#define em_proto_DownFrameDataMessage_init_zero  {0, false, em_proto_Pose_init_zero, 0}
#define em_proto_DownMessage_init_zero           {false, em_proto_DownFrameDataMessage_init_zero}

/* Field tags (for use in manual encoding/decoding) */
#define em_proto_Quaternion_w_tag                1
#define em_proto_Quaternion_x_tag                2
#define em_proto_Quaternion_y_tag                3
#define em_proto_Quaternion_z_tag                4
#define em_proto_Vec3_x_tag                      1
#define em_proto_Vec3_y_tag                      2
#define em_proto_Vec3_z_tag                      3
#define em_proto_Vec2_x_tag                      1
#define em_proto_Vec2_y_tag                      2
#define em_proto_Pose_position_tag               1
#define em_proto_Pose_orientation_tag            2
#define em_proto_TrackingMessage_P_localSpace_viewSpace_tag 1
#define em_proto_TrackingMessage_P_viewSpace_view0_tag 2
#define em_proto_TrackingMessage_P_viewSpace_view1_tag 3
#define em_proto_TrackingMessage_P_local_controller_grip_left_tag 4
#define em_proto_TrackingMessage_controller_aim_left_tag 5
#define em_proto_TrackingMessage_controller_grip_right_tag 6
#define em_proto_TrackingMessage_controller_aim_right_tag 7
#define em_proto_TrackingMessage_timestamp_tag   8
#define em_proto_TrackingMessage_sequence_idx_tag 9
#define em_proto_InputThumbstick_xy_tag          1
#define em_proto_InputThumbstick_click_tag       2
#define em_proto_InputThumbstick_touch_tag       3
#define em_proto_InputValueTouch_value_tag       1
#define em_proto_InputValueTouch_touch_tag       2
#define em_proto_InputClickTouch_click_tag       1
#define em_proto_InputClickTouch_touch_tag       2
#define em_proto_TouchControllerCommon_thumbstick_tag 1
#define em_proto_TouchControllerCommon_trigger_tag 2
#define em_proto_TouchControllerCommon_squeeze_tag 3
#define em_proto_TouchControllerCommon_thumbrest_touch_tag 4
#define em_proto_TouchControllerLeft_x_tag       1
#define em_proto_TouchControllerLeft_y_tag       2
#define em_proto_TouchControllerLeft_menu_tag    3
#define em_proto_TouchControllerLeft_common_tag  4
#define em_proto_TouchControllerRight_a_tag      1
#define em_proto_TouchControllerRight_b_tag      2
#define em_proto_TouchControllerRight_system_tag 3
#define em_proto_TouchControllerRight_common_tag 4
#define em_proto_UpFrameMessage_frame_sequence_id_tag 1
#define em_proto_UpFrameMessage_decode_complete_time_tag 2
#define em_proto_UpFrameMessage_begin_frame_time_tag 3
#define em_proto_UpFrameMessage_display_time_tag 4
#define em_proto_UpMessage_up_message_id_tag     1
#define em_proto_UpMessage_tracking_tag          2
#define em_proto_UpMessage_frame_tag             3
#define em_proto_DownFrameDataMessage_frame_sequence_id_tag 1
#define em_proto_DownFrameDataMessage_P_localSpace_viewSpace_tag 2
#define em_proto_DownFrameDataMessage_display_time_tag 3
#define em_proto_DownMessage_frame_data_tag      1

/* Struct field encoding specification for nanopb */
#define em_proto_Quaternion_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    w,                 1) \
X(a, STATIC,   SINGULAR, FLOAT,    x,                 2) \
X(a, STATIC,   SINGULAR, FLOAT,    y,                 3) \
X(a, STATIC,   SINGULAR, FLOAT,    z,                 4)
#define em_proto_Quaternion_CALLBACK NULL
#define em_proto_Quaternion_DEFAULT NULL

#define em_proto_Vec3_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    x,                 1) \
X(a, STATIC,   SINGULAR, FLOAT,    y,                 2) \
X(a, STATIC,   SINGULAR, FLOAT,    z,                 3)
#define em_proto_Vec3_CALLBACK NULL
#define em_proto_Vec3_DEFAULT NULL

#define em_proto_Vec2_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    x,                 1) \
X(a, STATIC,   SINGULAR, FLOAT,    y,                 2)
#define em_proto_Vec2_CALLBACK NULL
#define em_proto_Vec2_DEFAULT NULL

#define em_proto_Pose_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  position,          1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  orientation,       2)
#define em_proto_Pose_CALLBACK NULL
#define em_proto_Pose_DEFAULT NULL
#define em_proto_Pose_position_MSGTYPE em_proto_Vec3
#define em_proto_Pose_orientation_MSGTYPE em_proto_Quaternion

#define em_proto_TrackingMessage_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  P_localSpace_viewSpace,   1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  P_viewSpace_view0,   2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  P_viewSpace_view1,   3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  P_local_controller_grip_left,   4) \
X(a, STATIC,   OPTIONAL, MESSAGE,  controller_aim_left,   5) \
X(a, STATIC,   OPTIONAL, MESSAGE,  controller_grip_right,   6) \
X(a, STATIC,   OPTIONAL, MESSAGE,  controller_aim_right,   7) \
X(a, STATIC,   SINGULAR, INT64,    timestamp,         8) \
X(a, STATIC,   SINGULAR, INT64,    sequence_idx,      9)
#define em_proto_TrackingMessage_CALLBACK NULL
#define em_proto_TrackingMessage_DEFAULT NULL
#define em_proto_TrackingMessage_P_localSpace_viewSpace_MSGTYPE em_proto_Pose
#define em_proto_TrackingMessage_P_viewSpace_view0_MSGTYPE em_proto_Pose
#define em_proto_TrackingMessage_P_viewSpace_view1_MSGTYPE em_proto_Pose
#define em_proto_TrackingMessage_P_local_controller_grip_left_MSGTYPE em_proto_Pose
#define em_proto_TrackingMessage_controller_aim_left_MSGTYPE em_proto_Pose
#define em_proto_TrackingMessage_controller_grip_right_MSGTYPE em_proto_Pose
#define em_proto_TrackingMessage_controller_aim_right_MSGTYPE em_proto_Pose

#define em_proto_InputThumbstick_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  xy,                1) \
X(a, STATIC,   SINGULAR, BOOL,     click,             2) \
X(a, STATIC,   SINGULAR, BOOL,     touch,             3)
#define em_proto_InputThumbstick_CALLBACK NULL
#define em_proto_InputThumbstick_DEFAULT NULL
#define em_proto_InputThumbstick_xy_MSGTYPE em_proto_Vec2

#define em_proto_InputValueTouch_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, FLOAT,    value,             1) \
X(a, STATIC,   SINGULAR, BOOL,     touch,             2)
#define em_proto_InputValueTouch_CALLBACK NULL
#define em_proto_InputValueTouch_DEFAULT NULL

#define em_proto_InputClickTouch_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, BOOL,     click,             1) \
X(a, STATIC,   SINGULAR, BOOL,     touch,             2)
#define em_proto_InputClickTouch_CALLBACK NULL
#define em_proto_InputClickTouch_DEFAULT NULL

#define em_proto_TouchControllerCommon_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  thumbstick,        1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  trigger,           2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  squeeze,           3) \
X(a, STATIC,   SINGULAR, BOOL,     thumbrest_touch,   4)
#define em_proto_TouchControllerCommon_CALLBACK NULL
#define em_proto_TouchControllerCommon_DEFAULT NULL
#define em_proto_TouchControllerCommon_thumbstick_MSGTYPE em_proto_InputThumbstick
#define em_proto_TouchControllerCommon_trigger_MSGTYPE em_proto_InputValueTouch
#define em_proto_TouchControllerCommon_squeeze_MSGTYPE em_proto_InputValueTouch

#define em_proto_TouchControllerLeft_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  x,                 1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  y,                 2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  menu,              3) \
X(a, STATIC,   OPTIONAL, MESSAGE,  common,            4)
#define em_proto_TouchControllerLeft_CALLBACK NULL
#define em_proto_TouchControllerLeft_DEFAULT NULL
#define em_proto_TouchControllerLeft_x_MSGTYPE em_proto_InputClickTouch
#define em_proto_TouchControllerLeft_y_MSGTYPE em_proto_InputClickTouch
#define em_proto_TouchControllerLeft_menu_MSGTYPE em_proto_InputClickTouch
#define em_proto_TouchControllerLeft_common_MSGTYPE em_proto_TouchControllerCommon

#define em_proto_TouchControllerRight_FIELDLIST(X, a_) \
X(a_, STATIC,   OPTIONAL, MESSAGE,  a,                 1) \
X(a_, STATIC,   OPTIONAL, MESSAGE,  b,                 2) \
X(a_, STATIC,   OPTIONAL, MESSAGE,  system,            3) \
X(a_, STATIC,   OPTIONAL, MESSAGE,  common,            4)
#define em_proto_TouchControllerRight_CALLBACK NULL
#define em_proto_TouchControllerRight_DEFAULT NULL
#define em_proto_TouchControllerRight_a_MSGTYPE em_proto_InputClickTouch
#define em_proto_TouchControllerRight_b_MSGTYPE em_proto_InputClickTouch
#define em_proto_TouchControllerRight_system_MSGTYPE em_proto_InputClickTouch
#define em_proto_TouchControllerRight_common_MSGTYPE em_proto_TouchControllerCommon

#define em_proto_UpFrameMessage_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT64,    frame_sequence_id,   1) \
X(a, STATIC,   SINGULAR, INT64,    decode_complete_time,   2) \
X(a, STATIC,   SINGULAR, INT64,    begin_frame_time,   3) \
X(a, STATIC,   SINGULAR, INT64,    display_time,      4)
#define em_proto_UpFrameMessage_CALLBACK NULL
#define em_proto_UpFrameMessage_DEFAULT NULL

#define em_proto_UpMessage_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT64,    up_message_id,     1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  tracking,          2) \
X(a, STATIC,   OPTIONAL, MESSAGE,  frame,             3)
#define em_proto_UpMessage_CALLBACK NULL
#define em_proto_UpMessage_DEFAULT NULL
#define em_proto_UpMessage_tracking_MSGTYPE em_proto_TrackingMessage
#define em_proto_UpMessage_frame_MSGTYPE em_proto_UpFrameMessage

#define em_proto_DownFrameDataMessage_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, INT64,    frame_sequence_id,   1) \
X(a, STATIC,   OPTIONAL, MESSAGE,  P_localSpace_viewSpace,   2) \
X(a, STATIC,   SINGULAR, INT64,    display_time,      3)
#define em_proto_DownFrameDataMessage_CALLBACK NULL
#define em_proto_DownFrameDataMessage_DEFAULT NULL
#define em_proto_DownFrameDataMessage_P_localSpace_viewSpace_MSGTYPE em_proto_Pose

#define em_proto_DownMessage_FIELDLIST(X, a) \
X(a, STATIC,   OPTIONAL, MESSAGE,  frame_data,        1)
#define em_proto_DownMessage_CALLBACK NULL
#define em_proto_DownMessage_DEFAULT NULL
#define em_proto_DownMessage_frame_data_MSGTYPE em_proto_DownFrameDataMessage

extern const pb_msgdesc_t em_proto_Quaternion_msg;
extern const pb_msgdesc_t em_proto_Vec3_msg;
extern const pb_msgdesc_t em_proto_Vec2_msg;
extern const pb_msgdesc_t em_proto_Pose_msg;
extern const pb_msgdesc_t em_proto_TrackingMessage_msg;
extern const pb_msgdesc_t em_proto_InputThumbstick_msg;
extern const pb_msgdesc_t em_proto_InputValueTouch_msg;
extern const pb_msgdesc_t em_proto_InputClickTouch_msg;
extern const pb_msgdesc_t em_proto_TouchControllerCommon_msg;
extern const pb_msgdesc_t em_proto_TouchControllerLeft_msg;
extern const pb_msgdesc_t em_proto_TouchControllerRight_msg;
extern const pb_msgdesc_t em_proto_UpFrameMessage_msg;
extern const pb_msgdesc_t em_proto_UpMessage_msg;
extern const pb_msgdesc_t em_proto_DownFrameDataMessage_msg;
extern const pb_msgdesc_t em_proto_DownMessage_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define em_proto_Quaternion_fields &em_proto_Quaternion_msg
#define em_proto_Vec3_fields &em_proto_Vec3_msg
#define em_proto_Vec2_fields &em_proto_Vec2_msg
#define em_proto_Pose_fields &em_proto_Pose_msg
#define em_proto_TrackingMessage_fields &em_proto_TrackingMessage_msg
#define em_proto_InputThumbstick_fields &em_proto_InputThumbstick_msg
#define em_proto_InputValueTouch_fields &em_proto_InputValueTouch_msg
#define em_proto_InputClickTouch_fields &em_proto_InputClickTouch_msg
#define em_proto_TouchControllerCommon_fields &em_proto_TouchControllerCommon_msg
#define em_proto_TouchControllerLeft_fields &em_proto_TouchControllerLeft_msg
#define em_proto_TouchControllerRight_fields &em_proto_TouchControllerRight_msg
#define em_proto_UpFrameMessage_fields &em_proto_UpFrameMessage_msg
#define em_proto_UpMessage_fields &em_proto_UpMessage_msg
#define em_proto_DownFrameDataMessage_fields &em_proto_DownFrameDataMessage_msg
#define em_proto_DownMessage_fields &em_proto_DownMessage_msg

/* Maximum encoded size of messages (where known) */
#define em_proto_DownFrameDataMessage_size       63
#define em_proto_DownMessage_size                65
#define em_proto_InputClickTouch_size            4
#define em_proto_InputThumbstick_size            16
#define em_proto_InputValueTouch_size            7
#define em_proto_Pose_size                       39
#define em_proto_Quaternion_size                 20
#define em_proto_TouchControllerCommon_size      38
#define em_proto_TouchControllerLeft_size        58
#define em_proto_TouchControllerRight_size       58
#define em_proto_TrackingMessage_size            309
#define em_proto_UpFrameMessage_size             44
#define em_proto_UpMessage_size                  369
#define em_proto_Vec2_size                       10
#define em_proto_Vec3_size                       15

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
