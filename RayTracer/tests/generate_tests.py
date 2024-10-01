import random


for test in range(1, 11):
    with open(f"test_{str(test).zfill(3)}.in", "w+") as f:
        f.write("1\n")
        
    with open(f"test_{str(test).zfill(3)}.ans", "w+") as f:
        f.write("1\n")
        