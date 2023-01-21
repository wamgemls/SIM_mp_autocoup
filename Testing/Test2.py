import matplotlib.pyplot as plt
import time

def ShowGraph():
    n = 2
    j = 1
    while j <= 10:
        x = [i for i in range(n)]
        y = [i for i in range(n)]
        plt.plot(x, y, 'r-')
        plt.ylim([0, 10])
        plt.xlim([0, 10])
        if j > 1:
            plt.draw()
        else:
            plt.show(block=False)
        time.sleep(1)
        n += 1
        j += 1

ShowGraph()