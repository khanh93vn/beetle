"""
1
00:00:00,000 --> 00:00:01,280
#Thí nghiệm 01
"""

import pandas as pd

data_path = "~/Documents/data/beetle_evaluation_data.csv"
# srt_path = "~/Documents/phu-de.srt"
srt_path = "phu-de.srt"

def format_time(t):
    s = t // 10
    m = s // 60
    h = m // 60

    s -= m * 60
    m -= h * 60

    t = (t * 100) % 1000

    return f"{h:02d}:{m:02d}:{s:02d},{t:03d}"

df = pd.read_csv(data_path, header=None)
t0 = df[2][0]

start_time = 7

i = 1
with open(srt_path, 'w') as file:
    for t in df[2][1:]:
        end_time = t - t0

        file.write(f"{i}\n")
        file.write(format_time(start_time))
        file.write(" --> ")
        file.write(format_time(end_time))
        file.write(f"\n#Thí nghiệm {i}\n\n")

        start_time = end_time
        i += 1
