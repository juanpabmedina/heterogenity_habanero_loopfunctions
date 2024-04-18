import ast
import pandas as pd 
import matplotlib.pyplot as plt
import numpy as np 
import os 
import subprocess
import stat
import ast
import pandas as pd 

def run_Argos(argos):
    # run argos with passed fsm to write hitory file
    with open("./argos.sh",'w+') as f:
        f.write("#!/usr/bin/env python\n")
        f.write(f"argos3 -c {argos}")

    st = os.stat('./argos.sh')
    os.chmod('./argos.sh', st.st_mode | stat.S_IEXEC)

    subprocess.run(["bash", 'argos.sh'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

argos_file_path = '/home/robotmaster/argos3-installation/habanero/habanero-loopfunctions/scenarios/communication/aggregation.argos'
n_exp = 10
file_path = '/home/robotmaster/argos3-installation/habanero/habanero-loopfunctions/data/score_aggregation.txt'

fsm_config_dict = "--nstates 2 --s0 7 --rwm0 5 --cle0 0 --phe0 0 --n0 1 --n0x0 0 --c0x0 5 --p0x0 0.9 --l0x0 6 --f0x0 1 --s1 9 --fov1 1 --vel1 1.0 --cle1 6 --clr1 6 --phe1 0 --n1 1 --n1x0 0 --c1x0 3 --p1x0 0.5"
new_quantity = 16

os.remove(file_path)
data_score = []
for _ in range(n_exp):
    run_Argos(argos_file_path)

score = open(file_path)


with open(file_path, 'r') as f:
    for score in f.readlines(): 
        data_score.append(ast.literal_eval(score[:-1]))

print(data_score)

exp_name = 'aggregation6_habanero_100%'

# Example data to append
new_data = {'name': exp_name, 'fsm': fsm_config_dict, 'swarm': new_quantity,'iteraciones':n_exp, 'score': data_score}

# Path to the CSV file
csv_file_path = '/home/robotmaster/argos3-installation/habanero/habanero-loopfunctions/data/score.csv'

# Try to read the existing CSV file, or create a new DataFrame if the file doesn't exist
try:
    df = pd.read_csv(csv_file_path)
except FileNotFoundError:
    # If the file doesn't exist, create a new DataFrame with the specified columns
    df = pd.DataFrame(columns=['name', 'fsm', 'swarm', 'iteraciones', 'score'])

# Create a new DataFrame from the new_data
new_df = pd.DataFrame([new_data])

# Concatenate the existing DataFrame and the new DataFrame along the rows (axis=0)
df = pd.concat([df, new_df], ignore_index=True)

# Write the updated DataFrame back to the CSV file
df.to_csv(csv_file_path, index=False)