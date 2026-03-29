DEFAULT_PX4_FRAME = "uav"

DEFAULT_CAMERA_INTRINSICS = {
    "fx": "762.7223004959626",
    "fy": "762.7223004959626",
    "cx": "640.0",
    "cy": "360.0",
}

DEFAULT_CAMERA_TO_BODY = {
    "x": "0.08",
    "y": "0.0",
    "z": "0.03",
}

DEFAULT_STEREO_CAMERA_TO_BODY = {
    "x": "0.0503",
    "y": "0.0",
    "z": "0.1043",
}

DEFAULT_ORBBEC_IR_STREAM = {
    "width": "848",
    "height": "480",
    "fps": "30",
    "format": "ANY",
}

DEFAULT_ORBBEC_IR_STREAM_720P = {
    "width": "1280",
    "height": "720",
    "fps": "30",
    "format": "Y8",
}

DEFAULT_ORBBEC_IR_EXPOSURE = {
    "enable_auto_exposure": "true",
    "exposure": "-1",
    "gain": "-1",
    "ae_max_exposure": "-1",
    "brightness": "-1",
}

DEFAULT_ORBBEC_STANDALONE_PROFILE = {
    "enable_depth": "true",
    "enable_color": "false",
    "enable_left_ir": "false",
    "enable_right_ir": "false",
    "enable_point_cloud": "true",
    "enable_colored_point_cloud": "false",
    "enable_sync_output_accel_gyro": "false",
    "enable_publish_extrinsic": "false",
    "enable_accel": "false",
    "enable_gyro": "false",
    "accel_rate": "200hz",
    "gyro_rate": "200hz",
    "enable_laser": "false",
    "enable_ldp": "false",
}

DEFAULT_ORBBEC_OPENVINS_PROFILE = {
    "enable_depth": "false",
    "enable_color": "false",
    "enable_left_ir": "true",
    "enable_right_ir": "true",
    "enable_point_cloud": "false",
    "enable_colored_point_cloud": "false",
    "enable_sync_output_accel_gyro": "true",
    "enable_publish_extrinsic": "false",
    "enable_accel": "true",
    "enable_gyro": "true",
    "accel_rate": "200hz",
    "gyro_rate": "200hz",
    "enable_laser": "false",
    "enable_ldp": "false",
}

DEFAULT_ORBBEC_BOOTSTRAP_PROFILE = {
    "enable_depth": "true",
    "enable_color": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_color"],
    "enable_left_ir": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_left_ir"],
    "enable_right_ir": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_right_ir"],
    "enable_point_cloud": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_point_cloud"],
    "enable_colored_point_cloud": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_colored_point_cloud"],
    "enable_sync_output_accel_gyro": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_sync_output_accel_gyro"],
    "enable_publish_extrinsic": "true",
    "enable_accel": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_accel"],
    "enable_gyro": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_gyro"],
    "accel_rate": DEFAULT_ORBBEC_OPENVINS_PROFILE["accel_rate"],
    "gyro_rate": DEFAULT_ORBBEC_OPENVINS_PROFILE["gyro_rate"],
    "enable_laser": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_laser"],
    "enable_ldp": DEFAULT_ORBBEC_OPENVINS_PROFILE["enable_ldp"],
}
