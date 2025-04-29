1 1 1 0
# 1 1 3 1 
1 2 3 1
3 4 3 1
3 4 5 0



1 => 0
First  2 {0,1} element from prior node
3  [2] element from next node

0 => 1
First  2 {0,1} element from next node
3  [2] element from prior node




Write a python code which take in np array and return a new np array 
If the last element in the array is 

Write a program in python which takes in np array

Input 
np.array([[1, 2, 3, 1],
                     [5, 6, 7, 0],
                     [9, 10, 11, 1],
                     [9, 1, 1, 0],
                     [13, 14, 15, 1]])

1 Loop over the array
2 Check the next array 
3. Check if the last element of the array is 0 or 1
4. If the previous array last element is 0 and the current arrays last element is 1
5. Then insert a new array with the value of the previous array 0th and 1st position and take the 2nd position from the next array and last index will be 1
6. Also check for the last element in the array after any new array is inserted

Output should be 
np.array([[1, 2, 3, 1],
                     [5, 6, 7, 0], 
                     [5, 6, 11, 1], 
                     [9, 10, 11, 1],
                     [9, 1, 1, 0],
                    [9, 1, 15, 1], 
                     [13, 14, 15, 1]])