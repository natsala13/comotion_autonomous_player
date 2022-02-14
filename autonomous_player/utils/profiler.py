import time


TAB = '\t'


class Profiler:
    def __init__(self, tabs=1):
        self.start_time = time.time()
        self.last_time = self.start_time
        self.tabs = tabs

    def log(self, message):
        current_time = time.time()
        print(f'{TAB * self.tabs}{message} - {current_time - self.last_time}')
        self.last_time = current_time

    def total(self):
        print(f'Total time - {time.time() - self.start_time}')
