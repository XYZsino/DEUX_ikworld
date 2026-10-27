int32 class_id
string class_name
float32 confidence
# Pixel coordinates in the original RGB image: xmin, ymin, xmax, ymax.
float64[4] xyxy
# Image tracker identity; empty for legacy detection-only producers.
string track_id
# Optional visible-instance polygon, alternating x/y pixels in original RGB.
# Empty means detector provides no instance mask.
float64[] polygon_xy
