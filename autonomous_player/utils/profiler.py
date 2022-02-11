import time


class Profiler:
    def __init__(self):
        self.start_time = time.time()
        self.last_time = self.start_time

    def log(self, message):
        current_time = time.time()
        print(f'\t{message} - {current_time - self.last_time}')
        self.last_time = current_time

    def total(self):
        print(f'Total time - {time.time() - self.start_time}')
