from matplotlib import pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.lines import Line2D

import numpy as np


def boxplot_2d(x,y, ax, color, whis=1.5):
    xlimits = [np.percentile(x, q) for q in (25, 50, 75)]
    ylimits = [np.percentile(y, q) for q in (25, 50, 75)]
    ##the box
    box = Rectangle(
        (xlimits[0],ylimits[0]),
        (xlimits[2]-xlimits[0]),
        (ylimits[2]-ylimits[0]),
        color = color,
        ec = 'k',
        zorder=0
    )
    ax.add_patch(box)

    ##the x median
    vline = Line2D(
        [xlimits[1],xlimits[1]],[ylimits[0],ylimits[2]],
        color='k',
        zorder=1
    )
    ax.add_line(vline)

    ##the y median
    hline = Line2D(
        [xlimits[0],xlimits[2]],[ylimits[1],ylimits[1]],
        color='k',
        zorder=1
    )
    ax.add_line(hline)

    ##the central point
    ax.plot([xlimits[1]],[ylimits[1]], color='k', marker='o')

    ##the x-whisker
    ##defined as in matplotlib boxplot:
    ##As a float, determines the reach of the whiskers to the beyond the
    ##first and third quartiles. In other words, where IQR is the
    ##interquartile range (Q3-Q1), the upper whisker will extend to
    ##last datum less than Q3 + whis*IQR). Similarly, the lower whisker
    ####will extend to the first datum greater than Q1 - whis*IQR. Beyond
    ##the whiskers, data are considered outliers and are plotted as
    ##individual points. Set this to an unreasonably high value to force
    ##the whiskers to show the min and max values. Alternatively, set this
    ##to an ascending sequence of percentile (e.g., [5, 95]) to set the
    ##whiskers at specific percentiles of the data. Finally, whis can
    ##be the string 'range' to force the whiskers to the min and max of
    ##the data.
    iqr = xlimits[2]-xlimits[0]

    ##left

    x_left = x[x > xlimits[0]-whis*iqr]
    left = np.min(x_left) if not len(x_left) == 0 else np.min(x)
    whisker_line = Line2D(
        [left, xlimits[0]], [ylimits[1],ylimits[1]],
        color = 'k',
        zorder = 1
    )
    ax.add_line(whisker_line)
    whisker_bar = Line2D(
        [left, left], [ylimits[0],ylimits[2]],
        color = 'k',
        zorder = 1
    )
    ax.add_line(whisker_bar)

    ##right

    x_right =x[x < xlimits[2]+whis*iqr] 
    right = np.max(x_right) if not len(x_left) == 0 else np.max(x)
    whisker_line = Line2D(
        [right, xlimits[2]], [ylimits[1],ylimits[1]],
        color = 'k',
        zorder = 1
    )
    ax.add_line(whisker_line)
    whisker_bar = Line2D(
        [right, right], [ylimits[0],ylimits[2]],
        color = 'k',
        zorder = 1
    )
    ax.add_line(whisker_bar)

    ##the y-whisker
    iqr = ylimits[2]-ylimits[0]

    ##bottom
    y_bottom = y[y > ylimits[0]-whis*iqr]
    bottom = np.min(y_bottom) if not len(y_bottom) == 0 else ylimits[0]
    whisker_line = Line2D(
        [xlimits[1],xlimits[1]], [bottom, ylimits[0]], 
        color = 'k',
        zorder = 1
    )
    
    ax.add_line(whisker_line)
    whisker_bar = Line2D(
        [xlimits[0],xlimits[2]], [bottom, bottom], 
        color = 'k',
        zorder = 1
    )
    ax.add_line(whisker_bar)

    ##top
    y_top_t = y[y < ylimits[2]+whis*iqr]
    y_top = y_top_t[y_top_t > ylimits[2]]
    top = np.max(y_top) if not len(y_top) == 0 else ylimits[2]
    whisker_line = Line2D(
        [xlimits[1],xlimits[1]], [top, ylimits[2]], 
        color = 'k',
        zorder = 1
    )

    ax.add_line(whisker_line)
    whisker_bar = Line2D(
        [xlimits[0],xlimits[2]], [top, top], 
        color = 'k',
        zorder = 1
    )
    ax.add_line(whisker_bar)

    ##outliers
    mask = (x<left)|(x>right)|(y<bottom)|(y>top)
    ax.scatter(
        x[mask],y[mask],
        facecolors='none', edgecolors=color
    )

if __name__=="__main__":

    log_func = "logger_agg"

    titles = [
        "close_middle_5_grid_85",
        "main"
        ]

    for plots_title in titles:

        logger_aggs = [
            f"TP_NP_log/{log_func}_{plots_title}_2_class.txt",
            f"TP_NP_log/{log_func}_{plots_title}_3_class.txt"
            ]

        if plots_title == "main":
            class_num = ["3_class"]
            logger_aggs = [
            f"TP_NP_log/{log_func}_{plots_title}.txt"
            ]
        else:
            class_num = ["2_class", "3_class"]

        colors = ["blue", "red"]



        #the figure and axes
        fig,(ax1,ax2) = plt.subplots(ncols=2)
        fig.set_figwidth(10)
        ax1.set_xlim([0.0, 1.0])
        ax1.set_xlim([-0.05, 1.05])
        ax2.set_xlim([0.0, 1.0])
        ax2.set_xlim([-0.05, 1.05])
        ax1.set_xlabel('False Positive Rate')
        ax1.set_ylabel('True Positive Rate')
        ax2.set_xlabel('False Positive Rate')
        ax2.set_ylabel('True Positive Rate')
        plt.title(plots_title)

        for idx, log_file in enumerate(logger_aggs):
            with open(log_file, "r") as f:
                agg_data = f.read();

            print(log_file)

            FPR = []
            TPR = []

            for data_line in agg_data.split("\n"):
                for data in data_line.split("|"):
                    if "FP" in data:
                        fp_data = data.split("_")

                        TP = int(fp_data[1])
                        TN = int(fp_data[3])
                        FP = int(fp_data[5])
                        FN = int(fp_data[7])
                        fpr = 0 if FP == 0 else FP/(TN+FP)
                        tpr = 0 if TP == 0 else TP/(TP+FN)
                        if tpr == 0 and fpr == 0:
                            continue
                        FPR.append(fpr)
                        TPR.append(tpr)


            #plotting the original data
            FPR = np.array(FPR)
            TPR = np.array(TPR)
            ax1.scatter(FPR,TPR,c=colors[idx], s=1)

            #doing the box plot
            boxplot_2d(FPR,TPR,ax=ax2, color=colors[idx], whis=1)

        ax2.legend(class_num)
        plt.savefig(f'TP_NP_log/{log_func}_{plots_title}.png')
