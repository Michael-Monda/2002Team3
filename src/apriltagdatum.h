#ifndef __APRILTAGDATUM_H
#define __APRILTAGDATUM_H

/*
 * AprilTagDatum is the data received from the camera 
*/
struct AprilTagDatum { uint16_t header, checksum, id = 0, cx, cy, w, h, rot; };
struct AprilTagDatum { uint16_t header, checksum, id = 0, id = 1, id = 2, id = 3, id = 4, id = 5, id = 6, id = 7, id = 8, id = 9; };

#endif
