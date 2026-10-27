"""Simulation startup waypoint and operational ready pose, radians; not hardware calibration.

Reference is URDF attention posture (all 28 joints zero) BEFORE the 20 cm base retreat.
Bend shoulders/elbows in the sagittal plane to retain that world EE XY, raising Z
for conveyor clearance. Other arm joints remain zero; hand targets retain the reader setup.
The folded posture is a clearance waypoint only. HOME_JOINTS restores the original
operational ready pose, including shoulder spacing and wrist rotation.
Order: left arm 7, left hand 7, right arm 7, right hand 7 (JOINT_NAMES).
"""
ATTENTION_JOINTS = [0.0] * 28

ATTENTION_CLEARANCE_JOINTS = [
    0.5501935066268202,
    0.0,
    0.0,
    1.6372260746040028,
    0.0,
    0.0,
    0.0,
    0.07000151710399607,
    0.3499994736443229,
    1.1800000885411905,
    -1.4000011148164482,
    -0.9000002439896532,
    1.400000914999964,
    0.8999998202606976,
    -0.5501991153362762,
    0.0,
    0.0,
    -1.6372337534136436,
    0.0,
    0.0,
    0.0,
    0.0,
    -1.0,
    -1.0,
    0.3,
    0.2,
    -0.3,
    -0.2
]

# Original operational ready pose, preserved from the pre-preparation scenario.
HOME_JOINTS = [0.8076106068176119, -0.5392788615535845, -0.279198973241034, 1.536378726045994, -0.8003885080907891, -0.05671930437290872, -0.6648923480224322, 0.07000151710399607, 0.3499994736443229, 1.1800000885411905, -1.4000011148164482, -0.9000002439896532, 1.400000914999964, 0.8999998202606976, -1.0725166786377145, 0.9255798752381671, 0.5115041738577636, -1.7763570768039048, -0.4811809320319554, 0.5395033473849511, 0.6464622559559697, -2.710092228401217e-19, -1.0, -1.0, 0.3, 0.2, -0.3, -0.2]
