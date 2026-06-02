import numpy as np
import math
import matplotlib.pyplot as plt
import pandas as pd
import os

FOLDER = "out/MERGED SIMPLE ENVIRONMENT"
ALL_FOLDERS = {
    "conic": f"{FOLDER}/CONIC",
    "beaker_lrg": f"{FOLDER}/BEAKER_LRG",
    "beaker_med": f"{FOLDER}/BEAKER_MED",
    "measuring_cylinder": f"{FOLDER}/MEASURING_CYLINDER",
    "round_bottom": f"{FOLDER}/ROUND_BOTTOM",
    "volumetric": f"{FOLDER}/VOLUMETRIC_FLASK",
}

SAVE_FOLDER = f"out/MERGED SIMPLE ENVIRONMENT COLLATED"
os.makedirs(SAVE_FOLDER, exist_ok=False)

dfs_baseline = {k: pd.read_csv(f"{v}/average_baseline.csv") for k,v in ALL_FOLDERS.items()}
dfs_baseline_random = {k: pd.read_csv(f"{v}/average_baseline_random.csv") for k,v in ALL_FOLDERS.items()}
dfs_corrected_baseline = {k: pd.read_csv(f"{v}/average_corrected_baseline.csv") for k,v in ALL_FOLDERS.items()}
dfs_corrected_baseline_random = {k: pd.read_csv(f"{v}/average_corrected_baseline_random.csv") for k,v in ALL_FOLDERS.items()}
dfs = {k: pd.read_csv(f"{v}/average_run.csv") for k,v in ALL_FOLDERS.items()}
dfs_random = {k: pd.read_csv(f"{v}/average_run_random.csv") for k,v in ALL_FOLDERS.items()}

N = 1000

# create data for planning time after 50s
df_best_planning_score = pd.DataFrame(columns=["Baseline", "Baseline Random", "Corrected Baseline", "Corrected Baseline Random", "Ours", "Ours Random"])
for k,_ in ALL_FOLDERS.items():
    N = len(dfs_baseline[k]) - 1
    df_best_planning_score.loc[len(df_best_planning_score)] = [
        dfs_baseline[k].iloc[min(N, len(dfs_baseline[k])-1)]["best_planning_score"],
        dfs_baseline_random[k].iloc[min(N, len(dfs_baseline_random[k])-1)]["best_planning_score"],
        dfs_corrected_baseline[k].iloc[min(N, len(dfs_corrected_baseline[k])-1)]["best_planning_score"],
        dfs_corrected_baseline_random[k].iloc[min(N, len(dfs_corrected_baseline_random[k])-1)]["best_planning_score"],
        dfs[k].iloc[min(N, len(dfs[k])-1)]["best_planning_score"],
        dfs_random[k].iloc[min(N, len(dfs_random[k])-1)]["best_planning_score"],
    ]
print(df_best_planning_score)
df_best_planning_score.to_csv(f"{SAVE_FOLDER}/best_planning_score.csv")

# create data for planning time after 50s
N = 1000
df_num_valid = pd.DataFrame(columns=["Baseline", "Baseline Random", "Corrected Baseline", "Corrected Baseline Random", "Ours", "Ours Random"])
for k,_ in ALL_FOLDERS.items():
    N = len(dfs_baseline[k]) - 1
    df_num_valid.loc[len(df_num_valid)] = [
        dfs_baseline[k].iloc[min(N, len(dfs_baseline[k])-1)]["cum_percent_valid"],
        dfs_baseline_random[k].iloc[min(N, len(dfs_baseline_random[k])-1)]["cum_percent_valid"],
        dfs_corrected_baseline[k].iloc[min(N, len(dfs_corrected_baseline[k])-1)]["cum_percent_valid"],
        dfs_corrected_baseline_random[k].iloc[min(N, len(dfs_corrected_baseline_random[k])-1)]["cum_percent_valid"],
        dfs[k].iloc[min(N, len(dfs[k])-1)]["cum_percent_valid"],
        dfs_random[k].iloc[min(N, len(dfs_random[k])-1)]["cum_percent_valid"],
    ]
print(df_num_valid)
df_num_valid.to_csv(f"{SAVE_FOLDER}/cum_percent_valid.csv")


pass