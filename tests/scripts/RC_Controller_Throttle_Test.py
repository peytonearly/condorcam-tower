import time
from statistics import median
from Tower_Class import Tower

if __name__ == "__main__":
    tower = Tower()
    
    while True:
        tower_input, _ = tower.get_input_averages()
        # print(f"Controller Throttle High Time (μs): {median(tower.throttle_high_time_dq)}")
        print(tower_input)