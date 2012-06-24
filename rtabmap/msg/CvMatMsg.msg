
########################################
# OpenCV Matrix description
########################################

uint32 width     # Matrix width
uint32 height    # Matrix height
uint32 dataType  # Matrix data type (refer to OpenCV type, e.g. CV_8UC3 for a RGB image 8bits)
uint8[] data     # Matrix data
uint8 compressed  # if data is compressed (in case of an image for example)