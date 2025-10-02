import pandas as pd
import matplotlib.pyplot as plt
import os


script_dir = os.path.dirname(os.path.abspath(__file__))
csv_path = os.path.join(script_dir, "../output/results.csv")
csv_path = os.path.normpath(csv_path)
# Ensure the output directory exists
output_dir = os.path.join(script_dir, "../output/graphs")
output_dir = os.path.normpath(output_dir)
os.makedirs(output_dir, exist_ok=True)
# Load the CSV file
data = pd.read_csv(csv_path)
data = data.drop_duplicates(subset=["Topology", "Algorithm"], keep="last")
# Overwrite the CSV with the cleaned data (no index, same columns)
data.to_csv(csv_path, index=False)
# Get the unique topologies and algorithms
topologies = data["Topology"].unique()
algorithms = data["Algorithm"].unique()

# Function to add value labels on bars
def add_value_labels(ax, decimal_places=2):
    for bar in ax.patches:
        value = bar.get_height()
        formatted_value = f'{value:.{decimal_places}f}'
        ax.annotate(formatted_value,
                    (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                    ha='center', va='bottom', fontsize=8)

# Generate graphs for each topology
for topology in topologies:
    topology_data = data[data["Topology"] == topology]
    # Plot Total Hopcount vs Algorithm
    plt.figure(figsize=(10, 6))
    ax = plt.bar(topology_data["Algorithm"], topology_data["Total Hopcount"], color="purple")
    plt.title(f"Total Hopcount vs Algorithm ({topology})", fontsize=14)
    plt.xlabel("Algorithm", fontsize=12)
    plt.ylabel("Total Hopcount", fontsize=12)
    plt.grid(axis="y", linestyle="--", alpha=0.7)
    add_value_labels(plt.gca(), decimal_places=0)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir,f"{topology}_total_hopcount_vs_algorithm.png"))
    plt.show()
    # Plot Average Hopcount vs Algorithm
    plt.figure(figsize=(10, 6))
    ax = plt.bar(topology_data["Algorithm"], topology_data["Average Hopcount"], color="skyblue")
    plt.title(f"Average Hopcount vs Algorithm ({topology})", fontsize=14)
    plt.xlabel("Algorithm", fontsize=12)
    plt.ylabel("Average Hopcount", fontsize=12)
    plt.grid(axis="y", linestyle="--", alpha=0.7)
    add_value_labels(plt.gca(), decimal_places=2)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir,f"{topology}_avg_hopcount_vs_algorithm.png"))
    plt.show()

    # Plot Average Power Per Route vs Algorithm
    plt.figure(figsize=(10, 6))
    ax = plt.bar(topology_data["Algorithm"], topology_data["Average Power Per Route"], color="orange")
    plt.title(f"Average Power Per Route vs Algorithm ({topology})", fontsize=14)
    plt.xlabel("Algorithm", fontsize=12)
    plt.ylabel("Average Power Per Route (W)", fontsize=12)
    plt.grid(axis="y", linestyle="--", alpha=0.7)
    add_value_labels(plt.gca(), decimal_places=5)  # Keep 5 decimal places for power
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir,f"{topology}_avg_power_vs_algorithm.png"))
    plt.show()

    # Plot MCL vs Algorithm
    plt.figure(figsize=(10, 6))
    ax = plt.bar(topology_data["Algorithm"], topology_data["MCL"], color="green")
    plt.title(f"MCL vs Algorithm ({topology})", fontsize=14)
    plt.xlabel("Algorithm", fontsize=12)
    plt.ylabel("MCL (Gb)", fontsize=12)
    plt.grid(axis="y", linestyle="--", alpha=0.7)
    add_value_labels(plt.gca(), decimal_places=2)
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir,f"{topology}_global_max_load_vs_algorithm.png"))
    plt.show()

    # Plot Total Power vs Algorithm
    plt.figure(figsize=(10, 6))
    ax = plt.bar(topology_data["Algorithm"], topology_data["Total Power"], color="red")
    plt.title(f"Total Power vs Algorithm ({topology})", fontsize=14)
    plt.xlabel("Algorithm", fontsize=12)
    plt.ylabel("Total Power (W)", fontsize=12)
    plt.grid(axis="y", linestyle="--", alpha=0.7)
    add_value_labels(plt.gca(), decimal_places=5)  # Keep 5 decimal places for power
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir,f"{topology}_total_power_vs_algorithm.png"))
    plt.show()

# Generate comparison graphs between topologies for the same algorithm
for algorithm in algorithms:
    algorithm_data = data[data["Algorithm"] == algorithm]
    unique_topologies = algorithm_data["Topology"].unique()

    metrics = {
        "Total Hopcount": ("Total Hopcount", "purple", "Total Hopcount"),
        "Average Hopcount": ("Average Hopcount", "skyblue", "Average Hopcount"),
        "Average Power Per Route": ("Average Power Per Route", "orange", "Average Power Per Route (W)"),
        "MCL": ("MCL", "green", "MCL (Gb)"),
        "Total Power": ("Total Power", "red", "Total Power (W)")
    }

    # Case 1: Multiple topologies -> plot each metric vs topology
    if len(unique_topologies) > 1:
        for metric_key, (metric_label, color, ylabel) in metrics.items():
            plt.figure(figsize=(10, 6))
            ax = plt.bar(algorithm_data["Topology"], algorithm_data[metric_key], color=color)
            plt.title(f"{metric_label} vs Topology ({algorithm})", fontsize=14)
            plt.xlabel("Topology", fontsize=12)
            plt.ylabel(ylabel, fontsize=12)
            plt.grid(axis="y", linestyle="--", alpha=0.7)
            decimal_places = 5 if "Power" in metric_label else 2  # Use 5 decimal places for power metrics
            add_value_labels(plt.gca(), decimal_places=decimal_places)
            plt.tight_layout()
            plt.savefig(os.path.join(output_dir,f"comparison_{metric_key.lower().replace(' ', '_')}_vs_topology_{algorithm}.png"))
            plt.show()