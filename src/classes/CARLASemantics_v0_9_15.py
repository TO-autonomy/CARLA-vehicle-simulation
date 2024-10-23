from enum import Enum

class SemanticColors(Enum):
    UNLABELED =     (0, 0, 0)
    ROAD =         (1, 0, 0)
    SIDEWALK =     (2, 0, 0)
    BUILDING =      (3, 0, 0)
    WALL =          (4, 0, 0)
    FENCE =         (5, 0, 0)
    POLE =          (6, 0, 0)
    TRAFFICLIGHT =  (7, 0, 0)
    TRAFFICSIGN =   (8, 0, 0)
    VEGETATION =    (9, 0, 0)
    TERRAIN =      (10, 0, 0)
    SKY =          (11, 0, 0)
    PEDESTRIAN =   (12, 0, 0)
    RIDER =        (13, 0, 0)
    CAR =          (14, 0, 0)
    TRUCK =        (15, 0, 0)
    BUS =          (16, 0, 0)
    TRAIN =        (17, 0, 0)
    MOTORCYCLE =   (18, 0, 0)
    BICYCLE =      (19, 0, 0)
    STATIC =       (20, 0, 0)
    DYNAMIC =      (21, 0, 0)
    OTHER =        (22, 0, 0)
    WATER =        (23, 0, 0)
    ROADLINE =     (24, 0, 0)
    GROUND =       (25, 0, 0)
    BRIDGE =       (26, 0, 0)
    RAILTRACK =    (27, 0, 0)
    GUARDRAIL =    (28, 0, 0)

class SemanticTags(Enum):
    UNLABELED =     0
    ROAD =          1
    SIDEWALK =      2
    BUILDING =      3
    WALL =          4
    FENCE =         5
    POLE =          6
    TRAFFICLIGHT =  7
    TRAFFICSIGN =   8
    VEGETATION =    9
    TERRAIN =      10
    SKY =          11
    PEDESTRIAN =   12
    RIDER =        13
    CAR =          14
    TRUCK =        15
    BUS =          16
    TRAIN =        17
    MOTORCYCLE =   18
    BICYCLE =      19
    STATIC =       20
    DYNAMIC =      21
    OTHER =        22
    WATER =        23
    ROADLINE =     24
    GROUND =       25
    BRIDGE =       26
    RAILTRACK =    27
    GUARDRAIL =    28