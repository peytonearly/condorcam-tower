from statistics import median
from Tower_Class import Tower

if __name__ == "__main__":
    tower = Tower()
    
    while True:
        _, _ = tower.get_input_averages()
        print(tower._pedals_connected)