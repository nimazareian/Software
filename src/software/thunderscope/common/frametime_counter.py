import time
from PyQt6.QtWidgets import *
import collections
import numpy as np


class FrameTimeCounter:
    """
    FrameTimeCounver is basically just a list that stores the time difference between each consecutive function call.
    From that, it calculates the frametime of each function call.
    """

    def __init__(self) -> None:
        """
        Initialized framtime counter
        """
        self.datapoints = collections.deque(
            maxlen=100
        )  # stores the timeframe of every data cycle
        self.previous_timestamp = time.time()

        self.runtimes = np.zeros(20 * 30, dtype=np.float32)
        self.i = 0

    def add_one_datapoint(self):
        """
        Save the time difference between each consecutive function call.
        """
        current_time = time.time()
        time_difference = current_time - self.previous_timestamp
        self.datapoints.append(time_difference)

        self.previous_timestamp = current_time

        # Add time to the runtimes array
        self.runtimes[self.i] = time_difference
        self.i += 1
        if self.i == len(self.runtimes):
            self.i = 0
            # Print the percentiles
            print(f"Percentiles over {len(self.runtimes)} frames")
            print(f"50p: {np.percentile(self.runtimes, 50)}")
            print(f"80p: {np.percentile(self.runtimes, 80)}")
            print(f"90p: {np.percentile(self.runtimes, 90)}")
            print(f"95p: {np.percentile(self.runtimes, 95)}")

    def get_last_frametime(self):
        """
        Return the latest frametime from the list

        :return: the latest frametime, and -1 if the list is empty
        """

        if len(self.datapoints) == 0:
            return -1

        return self.datapoints[-1]

    def get_average_frametime(self):
        """
        Averaget the entire list 

        :return: the average of the entire list, and -1 if the list is empty
        """
        if len(self.datapoints) == 0:
            return -1

        return sum(self.datapoints) / len(self.datapoints)

    def get_average_last_30(self):
        """
        Average the last 30 frametime

        :return: the average of the last 30 frametime
        """
        if len(self.datapoints) == 0:
            return -1

        return sum(list(self.datapoints)[-30:]) / 30
