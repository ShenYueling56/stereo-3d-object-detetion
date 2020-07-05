//
// Created by shenyl on 2020/6/27.
//

#ifndef OBJECT_DETECTION_BOX3D_H
#define OBJECT_DETECTION_BOX3D_H
#include "bbox.h"

class box3d {
public:
    int _c;
    double _position_x;
    double _position_y;
    double _position_z;
    double _score;
public:
    box3d(int c=-1, double position_x = -1, double position_y=-1, double position_z=-1, double score=-1)
    {
        _c = c;
        _position_x = position_x;
        _position_y = position_y;
        _position_z = position_z;
        _score = score;
    }
};


#endif //OBJECT_DETECTION_BOX3D_H
