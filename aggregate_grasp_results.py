import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
import os

SAVE_FOLDER = "out/MERGED TABLE ENVIRONMENT/BEAKER_LRG"
FOLDERS = [
    "out/BEAKER_LRG_TABLE_RUN_1", 
    "out/BEAKER_LRG_TABLE_RUN_2", 
    "out/BEAKER_LRG_TABLE_RUN_3", 
    "out/BEAKER_LRG_TABLE_RUN_4", 
    "out/BEAKER_LRG_TABLE_RUN_5"
    ]
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
    input()
    return fig

def aggregate_grasps(dfs: list[pd.DataFrame]):
    return pd.concat(dfs).groupby(level=0).mean()

all_grasps = []
for folder in FOLDERS:
    df = merge_evaluations(folder)
    add_metrics(df)
    all_grasps.append(df)

average_grasps = aggregate_grasps(all_grasps)

base_grasps = merge_evaluations(BASE_FOLDER)
base_grasps_random = base_grasps.sample(frac=1, ignore_index=True)
add_metrics(base_grasps)
add_metrics(base_grasps_random)

data = {
    "Average Grasp Selection": average_grasps, 
    "Grasp Score Ordered": base_grasps, 
    "Random Ordering": base_grasps_random,
    }

fig = plot(data)

os.makedirs(SAVE_FOLDER, exist_ok=False)

data_runs = {
    "Grasp Score Ordered": base_grasps, 
    "Random Ordering": base_grasps_random,
    "Run 1": all_grasps[0],
    "Run 2": all_grasps[1],
    "Run 3": all_grasps[2],
    "Run 4": all_grasps[3],
    "Run 5": all_grasps[4]
    }

fig_runs = plot(data_runs)

fig.savefig(SAVE_FOLDER + "/images.png", format="png")
fig.savefig(SAVE_FOLDER + "/images.svg", format="svg")
fig_runs.savefig(SAVE_FOLDER + "/images_all_runs.png", format="png")
fig_runs.savefig(SAVE_FOLDER + "/images_all_runs.svg", format="svg")
base_grasps.to_csv(SAVE_FOLDER + "/base_merged.csv")
base_grasps_random.to_csv(SAVE_FOLDER + "/base_random_merged.csv")
average_grasps.to_csv(SAVE_FOLDER + "/average.csv")
for i in range(len(FOLDERS)):
    all_grasps[i].to_csv(SAVE_FOLDER + f"/run_{i}.csv")


