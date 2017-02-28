int32 message_id    # to confirm response
uint8 marker_family # 0) whycon, 1) aruco_single, 2) aruco_multi, 3) pitag
uint8 marker_id     # As defined by each marker family
float32 marker_size   # Required size of the marker in meters
string marker_name  # The name of the marker to be used as coordinate frame of the marker in TF
