from typing import List, Tuple, Union
import numpy as np


class Plane:
    def __init__(self, location: np.ndarray, heading: np.ndarray = None):
        self.location: np.ndarray = location
        self.heading: Union[np.ndarray, None] = heading
