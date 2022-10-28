import re

from math import pi
from itertools import product

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

data_path = "~/Documents/data/02.csv"

input_factors = ["Khoảng cách mục tiêu (m)", "Hướng của mục tiêu (°)",
                 "Hướng đầu xe tại mục tiêu (°)"]
responds = ["Thời gian (giây)", "Sai số khoảng cách (m)", "Sai số hướng đầu xe (°)"]

# sample_distances = [3.0, 6.0]
sample_distances = [2.5, 5.0]
# sample_directions = [-i*(pi/3) for i in range(4)]
sample_directions = [-i*60 for i in range(4)]
# sample_headings = [i*(pi/4) for i in range(8)]
sample_headings = [i*45 for i in range(8)]
samples = [*product(sample_distances, sample_directions, sample_headings)]

def grouped_mean_bar_plot(df, group_by, data_col):
    grouped = df.groupby(group_by)[data_col].mean()
    x = grouped.index
    plt.bar(x, grouped)
    plt.xlabel(group_by)
    plt.ylabel(data_col)
    plt.xticks(x)

df = pd.read_csv(data_path, header=None)

df.columns = ["Số hiệu thí nghiệm", "Thành công"] + responds

df["Sai số hướng đầu xe (°)"] = np.degrees(df["Sai số hướng đầu xe (°)"])

filter = df["Sai số hướng đầu xe (°)"] > 180
df["Sai số hướng đầu xe (°)"][filter] = np.abs(df["Sai số hướng đầu xe (°)"][filter] - 360)

# df["Số hiệu thí nghiệm"] = \
#     df["Số hiệu thí nghiệm"].map(lambda s: int(s.strip('TB').strip('*')))

for i, col in enumerate(input_factors):
    df[col] = df["Số hiệu thí nghiệm"].map(lambda index: samples[index-1][i])

df1 = df[df["Thành công"]]

g = sns.PairGrid(df1, x_vars=input_factors, y_vars=responds)
g.map(sns.swarmplot)
# plt.savefig("figures/figure.png", bbox_inches='tight',dpi=100)
plt.savefig("figures/figure.png")
plt.clf()
