import matplotlib.pyplot as plt
import numpy as np
import sys
import matplotlib.colors
import pandas as pd

if __name__ == '__main__':
    df = pd.read_csv('../../Data_random/total_metrics.csv', sep=';')
    df['throughput'] = df['throughput'].str.replace(',', '.')
    df['throughput'] = df['throughput'].astype(float)
    # print(type(df['throughput'][0]))
    # print(df)

    grouped = df.groupby('planner algo')

    fig, axes = plt.subplots(2, 1, figsize=(10, 10))
    images = []

    ax = axes[0]

    plan_algos = ['pibt', 'pibt_tf', 'epibt', 'epibt_lns', 'pepibt_lns', 'wppl']
    plan_algos_name = ['PIBT', 'PIBT+traffic flow', 'EPIBT', 'EPIBT+LNS', 'Parallel EPIBT+LNS', 'WPPL']

    for i in range(len(plan_algos)):
        df = grouped.get_group(plan_algos[i])
        images.append(ax.plot(df['agents num'], df['throughput'], label=plan_algos_name[i]))
        ax.set_ylabel('throughput')

    ax = axes[1]
    for i in range(len(plan_algos)):
        df = grouped.get_group(plan_algos[i])
        images.append(ax.plot(df['agents num'], df['avg step time'], label=plan_algos_name[i]))
        ax.set_yscale('log')
        ax.set_ylabel('avg step time')

    fig.supxlabel('agents num')

    lines_labels = [ax.get_legend_handles_labels() for ax in fig.axes]
    lines, labels = [sum(lol, []) for lol in zip(*lines_labels)]
    # remove not unique lines
    while len(lines) > len(plan_algos):
        lines.pop(-1)
        labels.pop(-1)
    fig.legend(lines, labels, loc='upper center', ncol=4)

    plt.show()
