import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
import os

FOLDER = "out/ADAPTIVE PLANNING TIME/CLUTTERED"
SMALL_SMALL_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_0.1_1.0_0({i})" for i in range(1, 6)]
SMALL_MED_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_0.1_1.0_1({i})" for i in range(1, 6)]
SMALL_LARGE_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_0.1_1.0_2({i})" for i in range(1, 6)]

MED_SMALL_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_1.0_5.0_0({i})" for i in range(1, 6)]
MED_MED_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_1.0_5.0_1({i})" for i in range(1, 6)]
MED_LARGE_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_1.0_5.0_2({i})" for i in range(1, 6)]

LARGE_SMALL_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_3.0_10.0_0({i})" for i in range(1, 6)]
LARGE_MED_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_3.0_10.0_1({i})" for i in range(1, 6)]
LARGE_LARGE_FOLDERS = [f"{FOLDER}/CONIC_CLUTTERED_3.0_10.0_2({i})" for i in range(1, 6)]

SAVE_FOLDER = f"out/ADAPTIVE PLANNING TIME/CLUTTERED MERGED (MEDIAN)"
os.makedirs(SAVE_FOLDER, exist_ok=False)

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
    return pd.concat(dfs).groupby(level=0).median()[:min([len(x) for x in dfs])] # can modify this

all_small_small = aggregate_grasps(SMALL_SMALL_FOLDERS)
all_small_med = aggregate_grasps(SMALL_MED_FOLDERS)
all_small_large = aggregate_grasps(SMALL_LARGE_FOLDERS)

all_med_small = aggregate_grasps(MED_SMALL_FOLDERS)
all_med_med = aggregate_grasps(MED_MED_FOLDERS)
all_med_large = aggregate_grasps(MED_LARGE_FOLDERS)

all_large_small = aggregate_grasps(LARGE_SMALL_FOLDERS)
all_large_med = aggregate_grasps(LARGE_MED_FOLDERS)
all_large_large = aggregate_grasps(LARGE_LARGE_FOLDERS)

average_all_small_small = average_grasps(all_small_small)
average_all_small_med = average_grasps(all_small_med)
average_all_small_large = average_grasps(all_small_large)

average_all_med_small = average_grasps(all_med_small)
average_all_med_med = average_grasps(all_med_med)
average_all_med_large = average_grasps(all_med_large)

average_all_large_small = average_grasps(all_large_small)
average_all_large_med = average_grasps(all_large_med)
average_all_large_large = average_grasps(all_large_large)

average_all_small_small.to_csv(f"{SAVE_FOLDER}/av_small_small.csv")
average_all_small_med.to_csv(f"{SAVE_FOLDER}/av_small_med.csv")
average_all_small_large.to_csv(f"{SAVE_FOLDER}/av_small_large.csv")

average_all_med_small.to_csv(f"{SAVE_FOLDER}/av_med_small.csv")
average_all_med_med.to_csv(f"{SAVE_FOLDER}/av_med_med.csv")
average_all_med_large.to_csv(f"{SAVE_FOLDER}/av_med_large.csv")

average_all_large_small.to_csv(f"{SAVE_FOLDER}/av_large_small.csv")
average_all_large_med.to_csv(f"{SAVE_FOLDER}/av_large_med.csv")
average_all_large_large.to_csv(f"{SAVE_FOLDER}/av_large_large.csv")

# plot_averages = {
#     # "Small Initialisation, 50% CI": average_all_small_small,
#     # "Small Initialisation, 84% CI": average_all_small_med,
#     "Small Initialisation, 97.5% CI": average_all_small_large,
#     # "Medium Initialisation, 50% CI": average_all_med_small,
#     # "Medium Initialisation, 84% CI": average_all_med_med,
#     "Medium Initialisation, 97.5% CI": average_all_med_large,
#     # "Large Initialisation, 50% CI": average_all_large_small,
#     # "Large Initialisation, 84% CI": average_all_large_med,
#     "Large Initialisation, 97.5% CI": average_all_large_large,
# }

initialisation_comparison_1 = {
    "Small Initialisation, UCB 2": average_all_small_large,
    "Medium Initialisation, UCB 2": average_all_med_large,
    "Large Initialisation, UCB 2": average_all_large_large,
}
fig = plot(initialisation_comparison_1)
fig.savefig(f"{SAVE_FOLDER}/comparison_of_initialisations_UCB2.svg", format="svg")
fig.savefig(f"{SAVE_FOLDER}/comparison_of_initialisations_UCB2.png", format="png")

initialisation_comparison_2 = {
    "Small Initialisation, UCB 1": average_all_small_med,
    "Medium Initialisation, UCB 1": average_all_med_med,
    "Large Initialisation, UCB 1": average_all_large_med,
}
fig = plot(initialisation_comparison_2)
fig.savefig(f"{SAVE_FOLDER}/comparison_of_initialisations_UCB1.svg", format="svg")
fig.savefig(f"{SAVE_FOLDER}/comparison_of_initialisations_UCB1.png", format="png")

initialisation_comparison_3 = {
    "Small Initialisation, UCB 0": average_all_small_small,
    "Medium Initialisation, UCB 0": average_all_med_small,
    "Large Initialisation, UCB 0": average_all_large_small,
}
fig = plot(initialisation_comparison_3)
fig.savefig(f"{SAVE_FOLDER}/comparison_of_initialisations_UCB0.svg", format="svg")
fig.savefig(f"{SAVE_FOLDER}/comparison_of_initialisations_UCB0.png", format="png")

ucb_comparison_1 = {
    "Small Initialisation, UCB 0": average_all_small_small,
    "Small Initialisation, UCB 1": average_all_small_med,
    "Small Initialisation, UCB 2": average_all_small_large,
}
fig = plot(ucb_comparison_1)
fig.savefig(f"{SAVE_FOLDER}/comparison_of_ucbs_small.svg", format="svg")
fig.savefig(f"{SAVE_FOLDER}/comparison_of_ucbs_small.png", format="png")

ucb_comparison_2 = {
    "Medium Initialisation, UCB 0": average_all_med_small,
    "Medium Initialisation, UCB 1": average_all_med_med,
    "Medium Initialisation, UCB 2": average_all_med_large,
}
fig = plot(ucb_comparison_2)
fig.savefig(f"{SAVE_FOLDER}/comparison_of_ucbs_med.svg", format="svg")
fig.savefig(f"{SAVE_FOLDER}/comparison_of_ucbs_med.png", format="png")

ucb_comparison_3 = {
    "Large Initialisation, UCB 0": average_all_large_small,
    "Large Initialisation, UCB 1": average_all_large_med,
    "Large Initialisation, UCB 2": average_all_large_large,
}
fig = plot(ucb_comparison_3)
fig.savefig(f"{SAVE_FOLDER}/comparison_of_ucbs_large.svg", format="svg")
fig.savefig(f"{SAVE_FOLDER}/comparison_of_ucbs_large.png", format="png")


# get data
# we want average planning time per iteration
# best planning score after 80s
# % valid grasps after 80s
def get_metrics(dfs):
    av_planning_time_per_iteration = []
    best_planning_score = []
    percent_valid_grasps = []
    for df in dfs:
        av_planning_time_per_iteration.append(df[df["cum_time"] < 80]["cum_time"].iloc[-1]/len(df[df["cum_time"] < 80]))
        best_planning_score.append(df[df["cum_time"] < 80]["best_planning_score"].iloc[-1])
        percent_valid_grasps.append(df[df["cum_time"] < 80]["cum_percent_valid"].iloc[-1])
    df = pd.DataFrame(np.array([av_planning_time_per_iteration, best_planning_score, percent_valid_grasps]).T, columns=["Av Planning Time per Iteration", "Best Planning Score after 80s", "Percent Valid Grasps after 80s"])
    return df

metrics_all_small_small = get_metrics(all_small_small)
metrics_all_small_med = get_metrics(all_small_med)
metrics_all_small_large = get_metrics(all_small_large)

metrics_all_med_small = get_metrics(all_med_small)
metrics_all_med_med = get_metrics(all_med_med)
metrics_all_med_large = get_metrics(all_med_large)

metrics_all_large_small = get_metrics(all_large_small)
metrics_all_large_med = get_metrics(all_large_med)
metrics_all_large_large = get_metrics(all_large_large)

av_planning_time_summary = pd.DataFrame(columns=["mean", "median", "var", "iqr"])
fn = lambda x: [np.mean(x["Av Planning Time per Iteration"]), np.median(x["Av Planning Time per Iteration"]), np.var(x["Av Planning Time per Iteration"]), np.quantile(x["Av Planning Time per Iteration"], 0.75) - np.quantile(x["Av Planning Time per Iteration"], 0.25)]
av_planning_time_summary.loc[0] = fn(metrics_all_small_small)
av_planning_time_summary.loc[1] = fn(metrics_all_small_med)
av_planning_time_summary.loc[2] = fn(metrics_all_small_large)
av_planning_time_summary.loc[3] = fn(metrics_all_med_small)
av_planning_time_summary.loc[4] = fn(metrics_all_med_med)
av_planning_time_summary.loc[5] = fn(metrics_all_med_large)
av_planning_time_summary.loc[6] = fn(metrics_all_large_small)
av_planning_time_summary.loc[7] = fn(metrics_all_large_med)
av_planning_time_summary.loc[8] = fn(metrics_all_large_large)

best_planning_score_summary = pd.DataFrame(columns=["mean", "median", "var", "iqr"])
fn = lambda x: [np.mean(x["Best Planning Score after 80s"]), np.median(x["Best Planning Score after 80s"]), np.var(x["Best Planning Score after 80s"]), np.quantile(x["Best Planning Score after 80s"], 0.75) - np.quantile(x["Best Planning Score after 80s"], 0.25)]
best_planning_score_summary.loc[0] = fn(metrics_all_small_small)
best_planning_score_summary.loc[1] = fn(metrics_all_small_med)
best_planning_score_summary.loc[2] = fn(metrics_all_small_large)
best_planning_score_summary.loc[3] = fn(metrics_all_med_small)
best_planning_score_summary.loc[4] = fn(metrics_all_med_med)
best_planning_score_summary.loc[5] = fn(metrics_all_med_large)
best_planning_score_summary.loc[6] = fn(metrics_all_large_small)
best_planning_score_summary.loc[7] = fn(metrics_all_large_med)
best_planning_score_summary.loc[8] = fn(metrics_all_large_large)

percent_valid_grasps_summary = pd.DataFrame(columns=["mean", "median", "var", "iqr"])
fn = lambda x: [np.mean(x["Percent Valid Grasps after 80s"]), np.median(x["Percent Valid Grasps after 80s"]), np.var(x["Percent Valid Grasps after 80s"]), np.quantile(x["Percent Valid Grasps after 80s"], 0.75) - np.quantile(x["Percent Valid Grasps after 80s"], 0.25)]
percent_valid_grasps_summary.loc[0] = fn(metrics_all_small_small)
percent_valid_grasps_summary.loc[1] = fn(metrics_all_small_med)
percent_valid_grasps_summary.loc[2] = fn(metrics_all_small_large)
percent_valid_grasps_summary.loc[3] = fn(metrics_all_med_small)
percent_valid_grasps_summary.loc[4] = fn(metrics_all_med_med)
percent_valid_grasps_summary.loc[5] = fn(metrics_all_med_large)
percent_valid_grasps_summary.loc[6] = fn(metrics_all_large_small)
percent_valid_grasps_summary.loc[7] = fn(metrics_all_large_med)
percent_valid_grasps_summary.loc[8] = fn(metrics_all_large_large)

av_planning_time_summary.to_csv(f"{SAVE_FOLDER}/av_planning_time_summary.csv")
best_planning_score_summary.to_csv(f"{SAVE_FOLDER}/best_planning_score_summary.csv")
percent_valid_grasps_summary.to_csv(f"{SAVE_FOLDER}/percent_valid_grasps_summary.csv")

metrics_all_small_small.to_csv(f"{SAVE_FOLDER}/metrics_all_small_small.csv")
metrics_all_small_med.to_csv(f"{SAVE_FOLDER}/metrics_all_small_med.csv")
metrics_all_small_large.to_csv(f"{SAVE_FOLDER}/metrics_all_small_large.csv")

metrics_all_med_small.to_csv(f"{SAVE_FOLDER}/metrics_all_med_small.csv")
metrics_all_med_med.to_csv(f"{SAVE_FOLDER}/metrics_all_med_med.csv")
metrics_all_med_large.to_csv(f"{SAVE_FOLDER}/metrics_all_med_large.csv")

metrics_all_large_small.to_csv(f"{SAVE_FOLDER}/metrics_all_large_small.csv")
metrics_all_large_med.to_csv(f"{SAVE_FOLDER}/metrics_all_large_med.csv")
metrics_all_large_large.to_csv(f"{SAVE_FOLDER}/metrics_all_large_large.csv")