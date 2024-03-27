import xml.etree.ElementTree as ET
import xml.dom.minidom
import os 
import subprocess
import stat
import ast
import pandas as pd 
import numpy as np 
def modify_fsm_config(xml_file_path, controller_id, new_fsm_config):
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()

    # Find the specified automode_controller by id
    automode_controllers = root.findall('.//automode_controller[@id="{}"]'.format(controller_id))
    for automode_controller in automode_controllers:
        # Find the params element inside the automode_controller
        params_element = automode_controller.find('./params')
        if params_element is not None:
            # Update the fsm-config attribute
            params_element.set('fsm-config', new_fsm_config)

    # Save the modified XML back to the file
    tree.write(xml_file_path)

def modify_quantity(xml_file_path, distribute_id, new_quantity, total_robots):
    # Parse the XML file
    tree = ET.parse(xml_file_path)
    root = tree.getroot()
    # Find the specified e-puck by id
    distribute_elements = root.findall('.//distribute[@id="{}"]'.format(distribute_id))
    for distribute_element in distribute_elements:
        # Find the parent entity element
        entity_element = distribute_element.find('./entity')
        if entity_element is not None:
            # Update the quantity attribute
            entity_element.set('quantity', str(new_quantity))

    entity_element = root.find('.//loop_functions/params')
    if entity_element is not None:
            # Update the quantity attribute
            entity_element.set('number_robots', str(total_robots))

    # Save the modified XML back to the file
    tree.write(xml_file_path)


def run_Argos(argos):
    # run argos with passed fsm to write hitory file
    with open("./argos.sh",'w+') as f:
        f.write("#!/usr/bin/env python\n")
        f.write(f"argos3 -c {argos}")

    st = os.stat('./argos.sh')
    os.chmod('./argos.sh', st.st_mode | stat.S_IEXEC)

    subprocess.run(["bash", 'argos.sh'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


p1 = 0.9
p2 = 0.5
p3 = 0.1

p = 0.9 

#f'--nstates 2 --s0 8 --vel0 1 --cle0 0 --clr0 3 --n0 2 --n0x0 0 --c0x0 0 --p0x0 {p1} --n0x1 0 --c0x1 2 --p0x1 {p1} --s1 9 --vel1 1 --cle1 0 --clr1 2 --n1 2 --n1x0 0 --c1x0 2 --p1x0 {p1} --n1x1 0 --c1x1 0 --p1x1 {p1}',

# ####### HOMING #######

# argos_file_path = 'homing.argos'
# fsm_config_dict = {
#     # 1: '--nstates 3 --s0 0 --rwm0 5 --cle0 0 --phe0 0 --n0 2 --n0x0 0 --c0x0 5 --p0x0 0.9 --l0x0 1 --f0x0 1 --n0x1 1 --c0x1 5 --p0x1 0.8 --l0x1 5 --f0x1 1 --s1 2 --fov1 1 --vel1 1.0 --cle1 0 --clr1 1 --phe1 1 --n1 1 --n1x0 0 --c1x0 3 --p1x0 0.5 --s2 2 --fov2 1 --vel2 1.0 --cle2 0 --clr2 5 --phe2 1 --n2 1 --n2x0 0 --c2x0 3 --p2x0 0.5',
#     1: '--nstates 3 --s0 7 --rwm0 5 --cle0 0 --phe0 0 --n0 2 --n0x0 0 --c0x0 5 --p0x0 0.9 --l0x0 1 --f0x0 1 --n0x1 1 --c0x1 5 --p0x1 0.8 --l0x1 6 --f0x1 1 --s1 9 --fov1 1 --vel1 1.0 --cle1 6 --clr1 1 --phe1 0 --n1 1 --n1x0 0 --c1x0 3 --p1x0 0.5 --s2 9 --fov2 1 --vel2 1.0 --cle2 6 --clr2 6 --phe2 0 --n2 1 --n2x0 0 --c2x0 3 --p2x0 0.5',
# }
# new_quantity = [10]

####### aggregation #######

##### habanero ######
# 1: '--nstates 3 --s0 0 --rwm0 5 --cle0 0 --phe0 0 --n0 2 --n0x0 0 --c0x0 5 --p0x0 0.9 --l0x0 6 --f0x0 1 --n0x1 1 --c0x1 5 --p0x1 0.8 --l0x1 5 --f0x1 1 --s1 2 --fov1 1 --vel1 1.0 --cle1 0 --clr1 6 --phe1 1 --n1 1 --n1x0 0 --c1x0 3 --p1x0 0.5 --s2 2 --fov2 1 --vel2 1.0 --cle2 0 --clr2 5 --phe2 1 --n2 1 --n2x0 0 --c2x0 3 --p2x0 0.5',

#### tutti ######
# argos_file_path = 'aggregation.argos'
# fsm_config_dict = {
#     # 1: '--nstates 2 --s0 7 --rwm0 5 --cle0 0 --phe0 0 --n0 1 --n0x0 0 --c0x0 5 --p0x0 0.9 --l0x0 6 --f0x0 1 --s1 9 --fov1 1 --vel1 1.0 --cle1 6 --clr1 6 --phe1 0 --n1 1 --n1x0 0 --c1x0 3 --p1x0 0.5',
#     1: '--nstates 3 --s0 0 --rwm0 5 --cle0 0 --phe0 0 --n0 2 --n0x0 0 --c0x0 5 --p0x0 0.9 --l0x0 5 --f0x0 1 --n0x1 1 --c0x1 5 --p0x1 0.3 --l0x1 6 --f0x1 1 --s1 2 --fov1 1 --vel1 1.0 --cle1 0 --clr1 5 --phe1 1 --n1 1 --n1x0 0 --c1x0 3 --p1x0 0.5 --s2 2 --fov2 1 --vel2 1.0 --cle2 0 --clr2 6 --phe2 1 --n2 1 --n2x0 0 --c2x0 3 --p2x0 0.5',
#     # 1: '--nstates 2 --s0 0 --rwm0 5 --cle0 0 --phe0 0 --n0 1 --n0x0 0 --c0x0 4 --p0x0 0.8 --s1 2 --fov1 1 --vel1 1.0 --cle1 0 --clr1 5 --phe1 2 --n1 1 --n1x0 0 --c1x0 4 --p1x0 0.9',
# }
# new_quantity = [10]


####### Tasking #######

argos_file_path = 'tasking.argos'
fsm_config_dict = {
    1: '--nstates 3 --s0 10 --fov0 1.6 --vel0 1.0 --clr0 2 --cle0 0 --phe0 0 --n0 1 --n0x0 0 --c0x0 5 --p0x0 0.5 --l0x0 2 --f0x0 1.6 --s1 9 --fov1 1.6 --vel1 1.0 --clr1 2 --cle1 0 --phe1 0 --n1 2 --n1x0 1 --c1x0 1 --p1x0 0.8 --n1x1 0 --c1x1 5 --p1x1 1 --l1x1 6 --f1x1 1.6 --s2 11 --phe2 0 --cle2 6 --n2 1 --n2x0 0 --c2x0 4 --p2x0 0.01 ',
    # 1: '--nstates 3 --s0 3 --fov0 1.6 --vel0 1.0 --clr0 2 --phe0 0 --n0 1 --n0x0 0 --c0x0 5 --p0x0 0.5 --l0x0 2 --f0x0 1.6 --s1 2 --fov1 1.6 --vel1 1.0 --clr1 2 --phe1 0 --n1 2 --n1x0 1 --c1x0 1 --p1x0 0.8 --n1x1 0 --c1x1 5 --p1x1 1 --l1x1 5 --f1x1 1.6 --s2 4 --phe2 2 --n2 1 --n2x0 0 --c2x0 4 --p2x0 0.05 ',
    # 1: '--nstates 3 --s0 0 --rwm0 5 --cle0 0 --phe0 0 --n0 1 --n0x0 0 --c0x0 5 --p0x0 0.5 --l0x0 3 --f0x0 1.6 --s1 2 --fov1 1.6 --vel1 1.0 --cle1 0 --clr1 3 --phe1 0 --n1 1 --n1x0 1 --c1x0 1 --p1x0 0.25 --s2 4 --phe2 1 --n2 1 --n2x0 0 --c2x0 4 --p2x0 0.05 ',
}
new_quantity = [4]

# argos_file_path = 'prueba1.argos'
# fsm_config_dict = {
#     1: '--nstates 1 --s0 2 --fov0 1 --vel0 1.0 --cle0 2 --clr0 3 --phe0 1',
#     2: '--nstates 1 --s0 8 --cle0 1',
#     3: '--nstates 3 --s0 0 --rwm0 5 --phe0 0 --n0 1 --n0x0 0 --c0x0 5 --p0x0 0.9 --l0x0 3 --f0x0 1 --s1 2 --fov1 1 --vel1 1.0 --clr1 3 --phe1 1 --n1 1 --n1x0 1 --c1x0 1 --p1x0 0.9 --s2 4 --phe2 1',
#     4: '--nstates 1 --s0 2 --fov0 1 --vel0 1.0 --clr0 3 --phe0 1',
# }
# new_quantity = [0,2,0,0]

total_robots = new_quantity

for n in range(1,len(new_quantity)+1):
    controller_id = 'heterogeneity'+str(n)
    new_fsm_config = fsm_config_dict[n]
    distribute_id = 'swarm'+str(n)
    modify_fsm_config(argos_file_path, controller_id, new_fsm_config)
    modify_quantity(argos_file_path, distribute_id, new_quantity[n-1], total_robots)

n_exp = 10
file_path = 'data/score_tasking.txt'
os.remove(file_path)
data_score = []
for _ in range(n_exp):
    run_Argos(argos_file_path)

score = open(file_path)


with open(file_path, 'r') as f:
    for score in f.readlines(): 
        data_score.append(ast.literal_eval(score[:-1]))

print(data_score)

exp_name = 'tasking_tutti_20%'

# Example data to append
new_data = {'name': exp_name, 'fsm': fsm_config_dict, 'swarm': new_quantity,'iteraciones':n_exp, 'score': data_score}

# Path to the CSV file
csv_file_path = 'data/score.csv'

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