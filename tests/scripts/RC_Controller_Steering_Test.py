from statistics import median
from Tower_Class import Tower

if __name__ == "__main__":
    tower = Tower()
    
    while True:
        _, _ = tower.get_input_averages()
        print(f"Controller Steering High Time (μs): {median(tower.steering_high_time_dq)}")