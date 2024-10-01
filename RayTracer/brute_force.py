import random
from subprocess import Popen, PIPE, STDOUT


def randomTest():
    s = "1\n"
    return s


for i in range(10000):
    print(f"Test {i}", end="\r")
    test = randomTest()
    p = Popen(['./build1'], stdout=PIPE, stdin=PIPE, stderr=PIPE, text=True)
    res1 = p.communicate(input=test)[0]
    p = Popen(['./build2'], stdout=PIPE, stdin=PIPE, stderr=PIPE, text=True)
    res2 = p.communicate(input=test)[0]

    if (res1 != res2):
        print(f"Test {i} failed")
        print(test)
        print(res1)
        print(res2)
        break
