from math import pi
from itertools import product

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

data_path = "~/Documents/data/01.csv"

def grouped_mean_bar_plot(df, group_by, data_col):
    grouped = df.groupby(group_by)[data_col].mean()
    x = grouped.index
    plt.bar(x, grouped)
    plt.xlabel(group_by)
    plt.ylabel(data_col)
    plt.xticks(x)

df = pd.read_csv(data_path, header=None)

df.columns = ["experiment_index", "success", "time", "distance_error", "heading_error"]

df.experiment_index = df.experiment_index.map(lambda s: int(s.strip('TB').strip('*')))

sample_distances = [3.0, 6.0]
sample_directions = [-i*(pi/3) for i in range(4)]
sample_headings = [i*(pi/4) for i in range(8)]

samples = [*product(sample_distances, sample_directions, sample_headings)]

df["distances"] = df.experiment_index.map(lambda index: samples[index-1][0])
df["directions"] = df.experiment_index.map(lambda index: samples[index-1][1])
df["headings"] = df.experiment_index.map(lambda index: samples[index-1][2])

df1 = df[df.success]
