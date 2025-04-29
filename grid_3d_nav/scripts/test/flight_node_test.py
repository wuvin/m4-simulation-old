import numpy as np

raw_path = np.array([[1, 2, 3, 1], 
                     [5, 6, 7, 0],
                     [9, 10, 11, 1], 
                     [2, 1, 1, 0], 
                     [13, 14, 15, 1]]) 

prev_flag = None
inserted_rows = 0
counter=0

full_path = raw_path

for i in range(len(raw_path)):
    current_flag = int(raw_path[i, 3])  # Access the flight or walk mode 4th row

    if prev_flag is not None and prev_flag != current_flag:
        prev_element = raw_path[i - 1]  # Access the previous whole element
        current_element = raw_path[i] 
        counter += 1 # Access the current whole element

        # Perform takeoff point creation
        if prev_flag == 0 and current_flag == 1:
            extra_element = np.array([prev_element[0], prev_element[1], current_element[2], 1]) 
            full_path = np.insert(full_path, (i+counter)-1, extra_element, axis=0) # add the current x,y 
            print(counter, (i+counter))

        # Perform landing point creation
        elif prev_flag == 1 and current_flag == 0:
            extra_element = np.array([current_element[0], current_element[1], prev_element[2], 0])
            
            full_path = np.insert(full_path, (i+counter)-1, extra_element, axis=0)  # Create a 4x1 element filled with zeros
            print(counter, (i+counter))

        

        # Insert the extra element between the current and next element
        

    prev_flag = current_flag

print(full_path)