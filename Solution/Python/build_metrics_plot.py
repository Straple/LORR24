import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.colors
import pandas as pd
from PIL import Image

plan_algos = ['pibt', 'pibt_tf', 'epibt', 'pepibt_lns', 'wppl']
plan_algos_name = ['PIBT', 'PIBT+traffic flow', 'EPIBT', 'EPIBT+LNS', 'WPPL']
markers = ['o', 'v', 's', 'p', '*']


def add_map(map_name, map_text, column):
    df = pd.read_csv('../../Data_' + map_name + '/total_metrics.csv', sep=';')
    df['throughput'] = df['throughput'].str.replace(',', '.')
    df['throughput'] = df['throughput'].astype(float)
    grouped = df.groupby('planner algo')

    ax = axes[0][column]
    ax.imshow(np.asarray(Image.open(map_name + '.png')))
    ax.title.set_text(map_text)
    ax.set_xticks([])
    ax.set_yticks([])

    ax = axes[1][column]
    for i in range(len(plan_algos)):
        try:
            df = grouped.get_group(plan_algos[i])
            ax.plot(df['agents num'], df['throughput'], alpha=0.7, label=plan_algos_name[i], marker=markers[i])
            if map_name == "random":
                ax.set_ylabel('Throughput')
            ax.grid(True)
        except:
            print("no group:", plan_algos[i])

    ax = axes[2][column]
    for i in range(len(plan_algos)):
        try:
            df = grouped.get_group(plan_algos[i])
            ax.plot(df['agents num'], df['avg step time'], alpha=0.7, label=plan_algos_name[i], marker=markers[i])
            ax.set_yscale('log')
            if map_name == "random":
                ax.set_ylabel('Decision Time (ms)')
            ax.grid(True)
            ax.set_xlabel('Number of Agents')
        except:
            print("no group:", plan_algos[i])


if __name__ == '__main__':
    fig, axes = plt.subplots(3, 3, figsize=(10, 10))

    add_map('random', 'random-32-32-20\nSize: 32x32\n|V|=819', 0)
    add_map('warehouse', 'warehouse\nSize: 140x500\n|V|=38586', 1)
    add_map('game', 'brc202d\nSize: 530x481\n|V|=43151', 2)

    lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
    lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    # remove not unique lines
    while len(lines) > len(plan_algos):
        lines.pop(-1)
        labels.pop(-1)
    fig.legend(lines, labels, loc='lower center', ncol=6)

    plt.savefig("metrics_plot.pdf", format='pdf', dpi=800)
    plt.show()
