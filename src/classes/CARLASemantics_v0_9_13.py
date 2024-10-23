from enum import Enum

class SemanticColors(Enum):
    UNLABELED =     (0, 0, 0)
    BUILDING =      (1, 0, 0)
    FENCE =         (2, 0, 0)
    OTHER =         (3, 0, 0)
    PEDESTRIAN =    (4, 0, 0)
    POLE =          (5, 0, 0)
    ROADLINE =      (6, 0, 0)
    ROAD =          (7, 0, 0)
    SIDEWALK =      (8, 0, 0)
    VEGETATION =    (9, 0, 0)
    VEHICLES =      (10, 0, 0)
    WALL =          (11, 0, 0)
    TRAFFICSIGN =   (12, 0, 0)
    SKY =           (13, 0, 0)
    GROUND =        (14, 0, 0)
    BRIDGE =        (15, 0, 0)
    RAILTRACK =     (16, 0, 0)
    GUARDRAIL =     (17, 0, 0)
    TRAFFICLIGHT =  (18, 0, 0)
    STATIC =        (19, 0, 0)
    DYNAMIC =       (20, 0, 0)
    WATER =         (21, 0, 0)
    TERRAIN =       (22, 0, 0)

class SemanticTags(Enum):
    UNLABELED =     0
    BUILDING =      1
    FENCE =         2
    OTHER =         3
    PEDESTRIAN =    4
    POLE =          5
    ROADLINE =      6
    ROAD =          7
    SIDEWALK =      8
    VEGETATION =    9
    VEHICLES =      10
    WALL =          11
    TRAFFICSIGN =   12
    SKY =           13
    GROUND =        14
    BRIDGE =        15
    RAILTRACK =     16
    GUARDRAIL =     17
    TRAFFICLIGHT =  18
    STATIC =        19
    DYNAMIC =       20
    WATER =         21
    TERRAIN =       22