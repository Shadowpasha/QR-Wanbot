import matplotlib.pyplot as plt
import numpy as np

# --- User Configuration ---
# Define your list of deviations here.
# These values represent the distance from the ideal straight path.
# Positive values can represent deviation to the left, negative to the right (or vice-versa).
# Example units: cm, mm, or meters.
deviations = [
    0.0,0.0,0.0,-0.066, -0.066,-0.066,-0.066, 0.0
]

# Optional: Define the distance or time step between measurements
# If unknown, just leave as 1 (effectively plotting against sample index)
step_size = 0.066 
x_label = "Measurement Step" # Label for the x-axis

# --- Plotting Code ---

def plot_deviations(data, step=1):
    """
    Plots the deviation of the robot from a straight path.
    """
    if not data:
        print("Error: No data to plot.")
        return

    # Create x-axis values (distance or time)
    x_values = [i * step for i in range(len(data))]
    
    plt.figure(figsize=(10, 6))
    
    # Plot the deviation data
    plt.plot(x_values, data, marker='o', linestyle='-', color='b', label='Robot Path')
    
    # Add a reference line for the straight path (y=0)
    plt.axhline(0, color='r', linestyle='--', linewidth=2, label='Ideal Straight Path')
    
    # Add labels and title
    plt.title("Robot Deviation from Straight Path", fontsize=16)
    plt.xlabel(x_label, fontsize=12)
    plt.ylabel("Deviation Amount", fontsize=12)
    
    # Invert x-axis (right to left)
    plt.gca().invert_xaxis()
    
    # Set y-axis limits
    plt.ylim(-0.4, 0.4)
    
    # Add grid and legend
    plt.grid(True, linestyle=':', alpha=0.7)
    plt.legend()
    
    # Show the plot
    plt.tight_layout()
    print("Displaying plot...")
    plt.show()

if __name__ == "__main__":
    print(f"Plotting {len(deviations)} data points...")
    plot_deviations(deviations, step_size)
