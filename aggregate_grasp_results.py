import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
import os

OBJECT_TYPE = "BEAKER_LRG"
ENVIRONMENT = "SIMPLE"
RUNS_BASE_FOLDER = f"out"

SAVE_FOLDER = f"out/MERGED {ENVIRONMENT} ENVIRONMENT/{OBJECT_TYPE}/"
BASE_FOLDER = f"out/{ENVIRONMENT} ENVIRONMENT BASE/{OBJECT_TYPE}"
BASELINE_FOLDERS = [f"{BASE_FOLDER}/{OBJECT_TYPE}_BASE_{i}" for i in range(1,6)]
SYMMETRY_FOLDERS = [f"{BASE_FOLDER}/{OBJECT_TYPE}_BASE_SYMMETRY_{i}" for i in range(1,6)]
CORRECTED_FOLDERS = [f"{BASE_FOLDER}/{OBJECT_TYPE}_BASE_CORRECTED_{i}" for i in range(1,6)]

RUNS_FOLDERS = [f"out/{OBJECT_TYPE}_RUN_{i}" for i in range(1,6)]

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

def aggregate_grasps(folders):
    all_runs = []
    for folder in folders:
        df = merge_evaluations(folder)
        add_metrics(df)
        all_runs.append(df)
    return all_runs

def aggregate_grasps_randomised(folders):
    all_runs = []
    for folder in folders:
        df = merge_evaluations(folder)
        df = df.sample(frac=1, ignore_index=True)
        add_metrics(df)
        all_runs.append(df)
    return all_runs

def average_grasps(dfs):
    return pd.concat(dfs).groupby(level=0).mean()

# average of selection runs
all_runs = aggregate_grasps(RUNS_FOLDERS)
average_run = average_grasps(all_runs)

all_baselines = aggregate_grasps(BASELINE_FOLDERS)
average_baseline = average_grasps(all_baselines)

all_symmetry_baselines = aggregate_grasps(SYMMETRY_FOLDERS)
average_symmetry_baseline = average_grasps(all_symmetry_baselines)

all_corrected_baselines = aggregate_grasps(CORRECTED_FOLDERS)
average_corrected_baseline = average_grasps(all_corrected_baselines)

# average of selection runs
all_runs_random = aggregate_grasps_randomised(RUNS_FOLDERS)
average_run_random = average_grasps(all_runs_random)

all_baselines_random = aggregate_grasps_randomised(BASELINE_FOLDERS)
average_baseline_random = average_grasps(all_baselines_random)

all_symmetry_baselines_random = aggregate_grasps_randomised(SYMMETRY_FOLDERS)
average_symmetry_baseline_random = average_grasps(all_symmetry_baselines_random)

all_corrected_baselines_random = aggregate_grasps_randomised(CORRECTED_FOLDERS)
average_corrected_baseline_random = average_grasps(all_corrected_baselines_random)

plot_baseline_symmetry = {
    "Baseline (Grasp Score Ordered)": average_baseline,
    "Baseline (Randomly Ordered)": average_baseline_random,
    "Symmetry-Enabled (Grasp Score Ordered)": average_symmetry_baseline,
    "Symmetry-Enabled (Randomly Ordered)": average_symmetry_baseline_random,
}

plot_symmetry_corrected = {
    "Symmetry-Enabled (Grasp Score Ordered)": average_symmetry_baseline,
    "Symmetry-Enabled (Randomly Ordered)": average_symmetry_baseline_random,
    "Stability-Corrected (Grasp Score Ordered)": average_corrected_baseline,
    "Stability-Corrected (Randomly Ordered)": average_corrected_baseline_random,
}

plot_corrected_selection = {
    "Stability-Corrected (Grasp Score Ordered)": average_corrected_baseline,
    "Stability-Corrected (Randomly Ordered)": average_corrected_baseline_random,
    "Grasp Selection": average_run,
}

plot_baseline_selection = {
    "Baseline (Grasp Score Ordered)": average_baseline,
    "Baseline (Randomly Ordered)": average_baseline_random,
    "Grasp Selection": average_run,
}

os.makedirs(SAVE_FOLDER, exist_ok=False)

fig_baseline_symmetry = plot(plot_baseline_symmetry)
fig_symmetry_corrected = plot(plot_symmetry_corrected)
fig_corrected_selection = plot(plot_corrected_selection)
fig_baseline_selection = plot(plot_baseline_selection)

fig_baseline_symmetry.savefig(SAVE_FOLDER + "/images_baseline_symmetry.png", format="png")
fig_baseline_symmetry.savefig(SAVE_FOLDER + "/images_baseline_symmetry.svg", format="svg")
fig_symmetry_corrected.savefig(SAVE_FOLDER + "/images_symmetry_corrected.png", format="png")
fig_symmetry_corrected.savefig(SAVE_FOLDER + "/images_symmetry_corrected.svg", format="svg")
fig_corrected_selection.savefig(SAVE_FOLDER + "/images_correction_selection.png", format="png")
fig_corrected_selection.savefig(SAVE_FOLDER + "/images_correction_selection.svg", format="svg")
fig_baseline_selection.savefig(SAVE_FOLDER + "/images_baseline_selection.png", format="png")
fig_baseline_selection.savefig(SAVE_FOLDER + "/images_baseline_selection.svg", format="svg")

data_runs = {
    "Corrected Baseline (Grasp Score Ordered)": average_corrected_baseline, 
    "Corrected Baseline (Randomly Ordered)": average_corrected_baseline_random,
    "Run 1": all_runs[0],
    "Run 2": all_runs[1],
    "Run 3": all_runs[2],
    "Run 4": all_runs[3],
    "Run 5": all_runs[4]
    }

fig_runs = plot(data_runs)

fig_runs.savefig(SAVE_FOLDER + "/images_all_runs.png", format="png")
fig_runs.savefig(SAVE_FOLDER + "/images_all_runs.svg", format="svg")

for i in range(5):
    all_runs[i].to_csv(f"{SAVE_FOLDER}/run_{i}.csv")
    all_baselines[i].to_csv(f"{SAVE_FOLDER}/baseline_{i}.csv")
    all_symmetry_baselines[i].to_csv(f"{SAVE_FOLDER}/symmetry_baseline_{i}.csv")
    all_corrected_baselines[i].to_csv(f"{SAVE_FOLDER}/corrected_baseline_{i}.csv")
    all_runs_random[i].to_csv(f"{SAVE_FOLDER}/run_random_{i}.csv")
    all_baselines_random[i].to_csv(f"{SAVE_FOLDER}/baseline_random_{i}.csv")
    all_symmetry_baselines_random[i].to_csv(f"{SAVE_FOLDER}/symmetry_baseline_random_{i}.csv")
    all_corrected_baselines_random[i].to_csv(f"{SAVE_FOLDER}/corrected_baseline_random_{i}.csv")

average_run.to_csv(f"{SAVE_FOLDER}/average_run.csv")
average_baseline.to_csv(f"{SAVE_FOLDER}/average_baseline.csv")
average_symmetry_baseline.to_csv(f"{SAVE_FOLDER}/average_symmetry_baseline.csv")
average_corrected_baseline.to_csv(f"{SAVE_FOLDER}/average_corrected_baseline.csv")
average_run_random.to_csv(f"{SAVE_FOLDER}/average_run_random.csv")
average_baseline_random.to_csv(f"{SAVE_FOLDER}/average_baseline_random.csv")
average_symmetry_baseline_random.to_csv(f"{SAVE_FOLDER}/average_symmetry_baseline_random.csv")
average_corrected_baseline_random.to_csv(f"{SAVE_FOLDER}/average_corrected_baseline_random.csv")