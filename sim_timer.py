import time
import sys 

class SimTimer:
    def __init__(self):
        self.min_time = sys.float_info.max
        self.max_time = 0.0
        self.average_loop_rate_hz = 0.0
        time_now = time.time()
        self.last_update_time = time_now
        self.averaging_const = 0.05
        self.print_time_period = 1.0
        self.last_print_time = time_now
        self.reset_period = 10.0
        self.last_reset_time = time_now

    def update(self):
        time_now = time.time()
        time_diff = (time_now - self.last_update_time)
        if ( time_diff > self.max_time):
            self.max_time = time_diff
        if ( time_diff < self.min_time):
            self.min_time = time_diff
        if (self.average_loop_rate_hz == 0.0):
            self.average_loop_rate_hz = 1.0 / time_diff
        else:
            self.average_loop_rate_hz = (1.0 - self.averaging_const) * self.average_loop_rate_hz + self.averaging_const * 1.0 / time_diff
        # Print out functions.
        time_diff2 = (time_now - self.last_print_time)
        if ( time_diff2 > self.print_time_period):
            print("min = {0} max = {1} average hz = {2}".format(self.min_time, self.max_time, self.average_loop_rate_hz))
            self.last_print_time = time_now
        # Reset timer.
        reset_diff = (time_now - self.last_reset_time)
        if ( reset_diff > self.reset_period):
            self.max_time = time_diff
            self.min_time = time_diff
            self.last_reset_time = time_now

        self.last_update_time = time_now
