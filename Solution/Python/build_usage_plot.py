import matplotlib.pyplot as plt
import numpy as np


def read(filename):
    with open(filename) as f:
        lines = [line.rstrip('\n') for line in f]
        rows, cols = lines[0].split(' ')
        rows = int(rows)
        cols = int(cols)
        print(rows, cols)
        result = []
        for dir in range(0, 5):
            result.append([])
            for act in range(0, 5):
                map = [int(x) for x in lines[1 + dir * 6 + act].split(' ')]
                #print(map)

                data = []
                for x in range(0, rows):
                    data.append([])
                    for y in range(0, cols):
                        pos = x * cols + y
                        data[-1].append(map[pos])

                result[-1].append(data)

        return result


if __name__ == '__main__':
    data = read('../../output.txt')

    dirs = ["East", "South", "West", "North", "All"]

    acts = ["Forward", "Rotate", "C. rotate", "Wait", "All"]

    fig, axes = plt.subplots(5, 5, figsize=(10, 10))
    images = []
    for i in range(25):
        dir = i // 5
        act = i % 5
        map = data[dir][act]
        ax = axes[dir][act]
        print(dir, act, map)
        images.append(ax.imshow(map, cmap='viridis'))
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')

    plt.tight_layout()
    plt.show()
