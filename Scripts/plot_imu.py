import pandas as pd
import matplotlib.pyplot as plt
import glob
import os
import argparse
import io

# User can manually specify a filename here to always plot that file by default
# Example: MANUAL_FILE_NAME = "serial_20260128_231628.txt"
MANUAL_FILE_NAME = "serial_20260129_001638.txt" 

def get_latest_file(pattern="serial_*.txt", search_dir="."):
    """Finds the latest file matching the pattern in the directory."""
    try:
        # Search in the specified directory
        files = glob.glob(os.path.join(search_dir, pattern))
        
        # If not found, check the parent directory (often useful if running from Scripts/)
        if not files:
            parent_dir = os.path.dirname(os.path.abspath(search_dir))
            files = glob.glob(os.path.join(parent_dir, pattern))
            
        if not files:
            return None
            
        latest_file = max(files, key=os.path.getctime)
        return latest_file
    except Exception as e:
        print(f"Error searching for files: {e}")
        return None

def plot_imu_data(txt_file):
    print(f"Loading data from: {txt_file}")
    
    try:
        # The format is described as:
        # Time       Roll      Pitch
        # 23:16:24.140 0.346823, 10.380535
        # So it's space separated for time vs data, and comma separated for Roll vs Pitch.
        # But actually it looks like: TIMESTAMP SPACE ROLL COMMA SPACE PITCH
        # We can treat it as a custom separator or just read it line by line and parse.
        
        data = []
        with open(txt_file, 'r') as f:
            for line in f:
                if not line.strip():
                    continue
                # Line: "23:16:24.140 0.346823, 10.380535"
                # Split by space first? -> ["23:16:24.140", "0.346823,", "10.380535"]
                parts = line.strip().split()
                if len(parts) >= 3:
                   timestamp = parts[0]
                   
                   try:
                       # Remove potential comma from roll
                       pitch = float(parts[1].replace(',', ''))
                       roll = float(parts[2]) - 10.25
                       data.append({'Timestamp': timestamp, 'Roll': roll, 'Pitch': pitch})
                   except ValueError:
                       # Skip lines that can't be parsed as numbers
                       continue
        
        if not data:
            print("No valid data found in file.")
            return

        df = pd.DataFrame(data)
        
        # Plot
        plt.figure(figsize=(12, 6))
        
        # Just plot against index for simplicity, or we could parse proper timestamps 
        # (but they restart or might have gaps, index is usually fine for inspection)
        plt.plot(df.index, df['Roll'], label='Roll', linewidth=1.5)
        plt.plot(df.index, df['Pitch'], label='Pitch', linewidth=1.5)
        
        plt.title(f'IMU Roll and Pitch Data\n{os.path.basename(txt_file)}')
        plt.xlabel('Sample Index')
        plt.ylabel('Angle (degrees)')
        plt.legend()
        plt.grid(True, which='both', linestyle='--', alpha=0.7)
        plt.tight_layout()
        
        print("Displaying plot...")
        plt.show()
        
    except Exception as e:
        print(f"Error processing or plotting data: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot latest IMU data from text file.")
    parser.add_argument("file", nargs="?", help="Specific txt file to plot. If omitted, uses MANUAL_FILE_NAME or finds latest 'serial_*.txt'.")
    args = parser.parse_args()
    
    target_file = args.file
    
    if not target_file:
        if MANUAL_FILE_NAME:
             target_file = MANUAL_FILE_NAME
             # Check if it exists in current or parent dir
             if not os.path.exists(target_file):
                 # Try parent dir
                 parent_file = os.path.join(os.path.dirname(os.path.abspath(".")), target_file)
                 if os.path.exists(parent_file):
                     target_file = parent_file
        
        if not target_file or not os.path.exists(target_file):
            # Try to find the latest file automatically
            target_file = get_latest_file()
            if target_file:
                print(f"Found latest log file: {target_file}")
            else:
                print("No log file found automatically.")
            
    if target_file and os.path.exists(target_file):
        plot_imu_data(target_file)
    elif target_file:
        print(f"File not found: {target_file}")
    else:
        print("Usage: python plot_imu.py [filename]")
        print("       or edit MANUAL_FILE_NAME in the script")
        print("       or just run to plot the latest serial_*.txt")
