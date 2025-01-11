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
        mn = 1000000000
        mx = 0
        for dir in range(0, 5):
            result.append([])
            for act in range(0, 5):
                map = [int(x) for x in lines[1 + dir * 6 + act].split(' ')]
                # print(map)

                data = []
                for x in range(0, rows):
                    data.append([])
                    for y in range(0, cols):
                        pos = x * cols + y
                        data[-1].append(map[pos])
                        if dir != 4 and act != 4:
                            mn = min(mn, map[pos])
                            mx = max(mx, map[pos])

                result[-1].append(data)

        return result, mn, mx


if __name__ == '__main__':
    data, mn, mx = read('../../Tmp/meta0')

    dirs = ["East", "South", "West", "North", "All"]

    acts = ["Forward", "Rotate", "C. rotate", "Wait", "All"]

    fig, axes = plt.subplots(4, 4, figsize=(10, 10))
    images = []
    for i in range(16):
        dir = i // 4
        act = i % 4
        map = data[dir][act]
        ax = axes[dir][act]
        # print(dir, act, map)
        images.append(ax.imshow(map, cmap='viridis'))#, vmin=mn, vmax=mx))
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')

    #fig.colorbar(images[-1], ax=axes.ravel().tolist())
    plt.show()
