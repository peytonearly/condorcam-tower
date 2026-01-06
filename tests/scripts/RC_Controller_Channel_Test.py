from Tower_Class import Tower

if __name__ == "__main__":
    tower = Tower()
    
    while True:
        _, _ = tower.get_input_averages()
        print(f"{tower._max_allowed_downward_speed, tower._max_allowed_upward_speed}")