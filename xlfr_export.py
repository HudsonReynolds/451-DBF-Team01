import os
import re
import numpy as np
from collections import defaultdict

# REPLACE this with the path to your folder containing the XFLR5 .txt files
folder_path = "C:\\Users\\jpreisle\\Downloads\\all wings export"

# Dictionary to store results grouped by Reynolds number
# Format will be: { reynolds_number: [ (airfoil_name, slope_deg, slope_rad), ... ] }
results_by_re = defaultdict(list)

for filename in os.listdir(folder_path):
    if filename.endswith(".txt"):
        filepath = os.path.join(folder_path, filename)
        
        airfoil_name = filename.replace(".txt", "") # Fallback name
        re_value = "Unknown Re"
        
        # 1. Parse the text file header to find Airfoil Name and Reynolds Number
        try:
            with open(filepath, 'r') as f:
                header_text = f.read(500) # The header is in the first few hundred characters
                
                # Extract Airfoil Name
                name_match = re.search(r'Calculated polar for:\s*(.+)', header_text)
                if name_match:
                    airfoil_name = name_match.group(1).strip()
                    
                # Extract Reynolds Number (XFLR5 usually formats it like "Re =     1.000 e 6")
                re_match = re.search(r'Re\s*=\s*([\d\.]+\s*e\s*\d+)', header_text)
                if re_match:
                    # Strip spaces so Python can convert "1.000 e 6" to a float
                    re_clean = re_match.group(1).replace(" ", "")
                    re_value = float(re_clean)
                else:
                    # Fallback for standard number formatting just in case
                    re_match2 = re.search(r'Re\s*=\s*([\d\.]+)', header_text)
                    if re_match2:
                        re_value = float(re_match2.group(1))
        except Exception:
            pass # If header parsing fails, it will use the fallback names

        # 2. Extract the Data and Calculate Slope
        try:
            data = np.genfromtxt(filepath, skip_header=11)
            
            if data.size == 0 or len(data.shape) != 2:
                continue
                
            alpha = data[:, 0]
            cl = data[:, 1]
            
            # Isolate the linear region (e.g., -2 to 4 degrees)
            mask = (alpha >= -2.0) & (alpha <= 4.0)
            alpha_lin = alpha[mask]
            cl_lin = cl[mask]
            
            if len(alpha_lin) > 1:
                slope_deg, intercept = np.polyfit(alpha_lin, cl_lin, 1)
                slope_rad = slope_deg * 57.2958 
                
                # Save to our grouped dictionary
                results_by_re[re_value].append((airfoil_name, slope_deg, slope_rad))
                
        except Exception:
            pass # Silently skip invalid files

# 3. Print the Grouped Results
# Sort the Reynolds numbers (handling any strings/unknowns safely)
sorted_res = sorted(results_by_re.keys(), key=lambda x: (isinstance(x, str), x))

for re_val in sorted_res:
    print(f"\n{'='*75}")
    if isinstance(re_val, float):
        print(f"REYNOLDS NUMBER: {re_val:,.0f}")
    else:
        print(f"REYNOLDS NUMBER: {re_val}")
    print(f"{'='*75}")
    print(f"{'Airfoil Name':<35} | {'Cl_alpha (/deg)':<15} | {'Cl_alpha (/rad)':<15}")
    print("-" * 75)
    
    # Sort airfoils alphabetically within each Re block
    for airfoil, s_deg, s_rad in sorted(results_by_re[re_val]):
        print(f"{airfoil[:33]:<35} | {s_deg:<15.4f} | {s_rad:<15.4f}")