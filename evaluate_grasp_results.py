import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd

FOLDER = "out/BEAKER_LRG_CORRECTED_WITH_TABLE_PLOTS"
BASE_FOLDER = "out/TABLE ENVIRONMENT/BEAKER_LRG_BASE_TABLE"

def merge_evaluations(folder):
    normal_grasps = pd.read_csv(folder + "/grasp_evaluations.csv")
    flipped_grasps = pd.read_csv(folder + "/grasp_evaluations_flipped.csv")

    normal_grasps["order"] = normal_grasps.index
    flipped_grasps["order"] = flipped_grasps.index

    normal_grasps["flipped"] = False
    flipped_grasps["flipped"] = True

    all_grasps = pd.concat([normal_grasps, flipped_grasps])
    all_grasps.sort_values(["order", "flipped"], ascending=[True, True], inplace=True, ignore_index=True)

    return normal_grasps
    return all_grasps

def add_metrics(df: pd.DataFrame):
    df["cum_num_valid"] = np.isfinite(df["planing_score"]).cumsum()
    df["cum_percent_valid"] = df["cum_num_valid"]/np.array(range(1, len(df) + 1)) * 100
    df["cum_time"] = df["planning_time"].cumsum()
    df["cum_var_z"] = [np.var(df["z"][:i][df["flipped"]]) + np.var(df["z"][:i][~df["flipped"]]) for i in range(1, len(df) + 1)]
    df["cum_var_th"] = [np.var(df["th"][:i][df["flipped"]]) + np.var(df["th"][:i][~df["flipped"]]) for i in range(1, len(df) + 1)]
    df["best_planning_score"] = df["planing_score"].cummin()

def plot(fields: dict[str, pd.DataFrame]):
    fig, axs = plt.subplots(2, 4, figsize=(30, 12))

    for k, v in fields.items():
        axs[0, 0].plot(np.array(v["cum_num_valid"]), label=str(k))
        axs[1, 0].plot(np.array(v["cum_percent_valid"]), label=str(k))
        axs[0, 1].plot(np.array(v["cum_time"]), label=str(k))
        axs[1, 1].plot(np.array(v["cum_time"]), v["cum_num_valid"], label=str(k))
        axs[0, 2].plot(np.array(v["best_planning_score"]), label=str(k))
        axs[1, 2].plot(np.array(v["cum_time"]), np.array(v["best_planning_score"]), label=str(k))
        axs[0, 3].plot(np.array(v["cum_var_z"]), label=str(k))
        axs[1, 3].plot(np.array(v["cum_var_th"]), label=str(k))
    
    axs[0, 0].set_title("Number of Valid Grasps")
    axs[0, 0].set_xlabel("Number of Grasps Evaluated")
    axs[0, 0].set_ylabel("Cum. Number of Valid Grasps")
    axs[0, 0].legend()

    axs[1, 0].set_title("Number of Valid Grasps")
    axs[1, 0].set_xlabel("Number of Grasps Evaluated")
    axs[1, 0].set_ylabel("%% Valid Grasps")
    axs[1, 0].legend()

    axs[0, 1].set_title("Total Planning Time")
    axs[0, 1].set_xlabel("Number of Grasps Evaluated")
    axs[0, 1].set_ylabel("Planning Time (s)")
    axs[0, 1].legend()

    axs[1, 1].set_title("Number of Valid Grasps")
    axs[1, 1].set_xlabel("Planning Time (s)")
    axs[1, 1].set_ylabel("Number of Valid Grasps")
    axs[1, 1].legend()

    axs[0, 2].set_title("Best Planning Score")
    axs[0, 2].set_xlabel("Number of Grasps Evaluated")
    axs[0, 2].set_ylabel("Lowest Path Score")
    axs[0, 2].legend()

    axs[1, 2].set_title("Best Planning Score")
    axs[1, 2].set_xlabel("Planning Time (s)")
    axs[1, 2].set_ylabel("Lowest Path Score")
    axs[1, 2].legend()

    axs[0, 3].set_title("Radial Variance")
    axs[0, 3].set_xlabel("Number of Grasps Evaluated")
    axs[0, 3].set_ylabel("Variance in z")
    axs[0, 3].legend()

    axs[1, 3].set_title("Angular Variance")
    axs[1, 3].set_xlabel("Number of Grasps Evaluated")
    axs[1, 3].set_ylabel("Variance in theta")
    axs[1, 3].legend()

    fig.show()

# all_grasps = merge_evaluations(FOLDER)
# # all_grasps_no_eps = merge_evaluations("out/1777957320.9340703")
# base_grasps = merge_evaluations(BASE_FOLDER)
# base_grasps_random = base_grasps.sample(frac=1, ignore_index=True)

# add_metrics(all_grasps)
# # add_metrics(all_grasps_no_eps)
# add_metrics(base_grasps)
# add_metrics(base_grasps_random)

long = merge_evaluations("out/10_sec")
med = merge_evaluations("out/5_sec")
short = merge_evaluations("out/0.2_sec")

add_metrics(long)
add_metrics(med)
add_metrics(short)

data = {"Long": long, "Medium": med, "Short": short}

plot(data)

input()


