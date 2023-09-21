import sys 
import os 
import numpy as np

""" 
TODO:
 [ ] Create an iterator function which can deterministically get an exact combination given an index (for post processing analysis and reducing the amount of files needed in post processing)
"""

#Why this? In python objects are pass by reference, except basic types like integers
class incrementer:
    def __init__(self):
        self.val = 0
    def inc(self):
        self.val+=1
    def get_val(self):
        return self.val

# A helper recursive function which writes all combinations of a set of collections/options you would like to explore
def get_all_combinations(mission_vars, remaining_options, fname, file_idx, rules):
    if len(remaining_options) == 0:
        #Reached base case - now write the arguments to a file
        for r in rules:
            if r(mission_vars) == False:
                return 
        if file_idx.get_val()%10 == 0:
            print(f"On File: {file_idx.val}")
        file_name = fname+"_"+str(file_idx.val)+".moos"
        with open(file_name, 'w') as f:
            f.write("// <VARIABLE BLOCK>\n")
            for k, v in mission_vars.items():
                f.write(f"\t{k} = {v}\n")
            f.write("// </VARIABLE BLOCK>")
            file_idx.inc()
    else:
        #for key, vals in remaining_options:
        c = remaining_options.copy()
        key, vals = c.pop()
        for v in vals:
            s = {**mission_vars, key:v}
            get_combination(s, c, fname, file_idx, rules)


#The function which is called by the user to write a collection of plug files to a specific directory, for which only conditions
# which meet certain rules and conditions will be written. See example. 
def gen_all_var_plugs(dirname, fname, collections, rules=None):
    os.makedirs(dirname, exist_ok=True)
    fname = dirname+"/"+fname
    ub = 1
    for _, val in collections:
        ub*=len(val)

    if(ub > 1000):
        print(f"The current configuration would generate up to {ub} mission parameter combinations - are you sure? It's not to late.")
        decision = input("Y/n: ")
        if(decision.lower().count("n") >= 1):
            print("Aborting")
            exit(0)

    file_idx = incrementer()
    mission_vars = {}
    get_combination(mission_vars, collections, fname, file_idx, rules)

# A helper recursive function which writes all combinations of a set of collections/options you would like to explore
def get_to_combination(mission_vars, remaining_options, fname, target_idx, current_idx, rules):
    if len(remaining_options) == 0:
        #Reached base case - now write the arguments to a file
        for r in rules:
            if r(mission_vars) == False:
                return 
        if current_idx.get_val()%10 == 0:
            print(f"On File: {current_idx.val}")
        file_name = fname+"_"+str(current_idx.val)+".moos"
        with open(file_name, 'w') as f:
            f.write("// <VARIABLE BLOCK>\n")
            for k, v in mission_vars.items():
                f.write(f"\t{k} = {v}\n")
            f.write("// </VARIABLE BLOCK>")
            current_idx.inc()
    else:
        #for key, vals in remaining_options:
        c = remaining_options.copy()
        key, vals = c.pop()
        for v in vals:
            s = {**mission_vars, key:v}
            get_to_combination(s, c, fname, current_idx, rules)

def get_ub(collections, idx, rules=None):
    #Reach the end of all combinations and just report the count
    pass 

#The function which is called by the user to write a collection of plug files to a specific directory, for which only conditions
# which meet certain rules and conditions will be written. See example. 
def gen_var_plug(fname, collections, target_idx, rules=None):

    ub = get_ub(collections, 0, rules)

    ub = 1

    for _, val in collections:
        ub*=len(val)

    if(ub > 1000):
        print(f"The current configuration would generate up to {ub} mission parameter combinations - are you sure? It's not to late.")
        decision = input("Y/n: ")
        if(decision.lower().count("n") >= 1):
            print("Aborting")
            exit(0)

    current_idx = incrementer()
    mission_vars = {}
    get_to_combination(mission_vars, collections, fname, target_idx, current_idx, rules)