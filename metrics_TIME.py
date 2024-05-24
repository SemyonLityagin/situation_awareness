from matplotlib import pyplot as plt
import numpy as np

def add_box_plot_fil(log_file, ax):
    with open(log_file, "r") as f:
        t_data = f.read();

    data = []

    for t_line in t_data.split("\n")[:-1]:
        try:
            x, t = t_line.split("|")
            if x == 0: continue
        except Exception as e:
            continue
        t = float(t)

        data.append(t)
    ax.boxplot(data)

def add_box_plot_agg(log_file, ax):
    with open(log_file, "r") as f:
        t_data = f.read();

    s_len = {0: [], 1:[], 2:[], 3:[], 4:[], 5:[]}

    for t_line in t_data.split("\n")[:-1]:
        try:
            x, t = t_line.split("|")
            if x == 0: continue
        except Exception as e:
            continue
        x = int(x)
        t = float(t)

        if x in s_len.keys():
            s_len[x].append(t)
        else:
            s_len[x] = [t]

    data = []
    ticks = []
    for k in s_len.keys():
        data.append(s_len[k])
        ticks.append(k)
    ax.boxplot(data)
    ax.set_xticklabels(ticks)
    
plots_list = [
    "close_middle_5_grid_85_3_class",
    "main"
    ]
for plots_title in plots_list:
    log_agg_file = f"TIME_log/logger_timer_agg_{plots_title}.txt"
    log_fil_file = f"TIME_log/logger_timer_fil_{plots_title}.txt"

    fig,(ax1,ax2) = plt.subplots(ncols=2)
    fig.set_figwidth(10)
    add_box_plot_agg(log_agg_file, ax1)
    add_box_plot_fil(log_fil_file, ax2)
    ax1.set_title(f"agg_{plots_title}")
    ax2.set_title(f"fil_{plots_title}")

    plt.savefig(f'TIME_log/{plots_title}_time.png')
