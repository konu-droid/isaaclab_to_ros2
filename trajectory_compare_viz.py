import pandas as pd
import matplotlib.pyplot as plt
import argparse
import sys

def plot_trajectories(ideal_filepath, real_filepath):
    """
    Reads two CSV files containing robot trajectory data and plots a comparison.

    Args:
        ideal_filepath (str): The path to the CSV file for the ideal (simulated) trajectory.
        real_filepath (str): The path to the CSV file for the realistic (real robot) trajectory.
    """
    try:
        # --- Data Loading ---
        # Read the data into pandas DataFrames from the specified file paths.
        ideal_df = pd.read_csv(ideal_filepath)
        real_df = pd.read_csv(real_filepath)
    except FileNotFoundError as e:
        print(f"Error: File not found - {e}. Please check the file paths.", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"An error occurred while reading the CSV files: {e}", file=sys.stderr)
        sys.exit(1)


    # --- Plotting ---

    # Get the list of motor names from the columns (excluding the time column)
    # Assumes both files have the same column structure.
    motor_names = ideal_df.columns[1:]
    num_motors = len(motor_names)

    # Create a figure and a grid of subplots.
    # For 6 motors, a 2x3 grid is a good layout.
    fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(15, 8))
    fig.suptitle('Simulated vs. Real Robot Trajectory Comparison', fontsize=16)

    # Flatten the axes array to make it easy to iterate over
    axes = axes.flatten()

    # Loop through each motor and create a plot
    for i, motor_name in enumerate(motor_names):
        ax = axes[i]

        # Plot the ideal trajectory (blue, solid line)
        ax.plot(ideal_df.iloc[:, 0], ideal_df[motor_name],
                label='Ideal (Sim)', color='dodgerblue', linestyle='-')

        # Plot the real trajectory (red, dashed line)
        ax.plot(real_df.iloc[:, 0], real_df[motor_name],
                label='Realistic (Real)', color='orangered', linestyle='--')

        # Set the title and labels for each subplot
        ax.set_title(motor_name.replace('_', ' ').replace('(rad)', '').strip())
        ax.set_xlabel(ideal_df.columns[0]) # Use the name of the first column as x-label
        ax.set_ylabel('Position (rad)')
        ax.legend()
        ax.grid(True, linestyle=':', alpha=0.6)

    # If there are more subplots than motors, hide the unused ones.
    for i in range(num_motors, len(axes)):
        fig.delaxes(axes[i])

    # Adjust the layout to prevent titles and labels from overlapping
    plt.tight_layout(rect=[0, 0.03, 1, 0.95]) # Adjust rect to make space for suptitle

    # Display the plot window
    plt.show()

if __name__ == '__main__':
    # --- Command-Line Argument Parsing ---
    parser = argparse.ArgumentParser(
        description="Compares and plots ideal vs. real robot trajectories from CSV files."
    )
    parser.add_argument(
        "ideal_file",
        type=str,
        help="Path to the CSV file containing the ideal (simulated) trajectory data."
    )
    parser.add_argument(
        "real_file",
        type=str,
        help="Path to the CSV file containing the realistic (real robot) trajectory data."
    )

    args = parser.parse_args()

    # Call the main plotting function with the provided file paths
    plot_trajectories(args.ideal_file, args.real_file)
