import math

import numpy as np

from t800_stage3_navigation.core import yaw_to_quaternion


def test_yaw_quaternion():
    assert np.allclose(yaw_to_quaternion(math.pi), [0.0, 0.0, 1.0, 0.0], atol=1e-12)
