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
        line_ptr = 1
        for act in range(0, 4):
            result.append([])
            for dir in range(0, 4):
                map = [int(x) for x in lines[line_ptr].split(' ')]
                line_ptr += 1

                data = []
                for x in range(0, rows):
                    data.append([])
                    for y in range(0, cols):
                        pos = x * cols + y
                        if pos >= len(map):
                            print(pos, len(map))
                        data[-1].append(map[pos])

                result[-1].append(data)

        return result


if __name__ == '__main__':
    data = read('../../graph_guidance')

    dirs = ["E", "S", "W", "N"]
    acts = ["FW", "R", "CR", "W"]

    # fig, axes = plt.subplots(4, 4, figsize=(10, 10))
    # images = []
    for i in range(4):
        fig, axes = plt.subplots(1, 1, figsize=(10, 10))
        dir = i  # i // 4
        act = 0  # i % 4
        map = data[dir][act]
        ax = axes
        print("processing:", dir, act)
        ax.imshow(map, cmap='viridis')
        ax.set_title(dirs[dir] + " & " + acts[act])
        ax.axis('off')
        plt.tight_layout()
        # plt.show()
        plt.savefig(dirs[dir] + "_" + acts[act] + ".svg", format='svg', dpi=1200)
