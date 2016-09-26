
Header header

# OpenCV matrix containing the user data. A matrix of type CV_8UC1 
# with 1 row is considered to be compressed (with rtabmap::compressData() method).
# If you have one dimension unsigned 8 bits uncompressed data, make sure to transpose it
# (to have multiple rows instead of multiple columns) in order to be detected as
# not compressed.
uint32 rows
uint32 cols
uint32 type
uint8[] data