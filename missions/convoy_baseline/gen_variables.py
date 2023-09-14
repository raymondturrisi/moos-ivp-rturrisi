#!/bin/bash

#List variables and ranges up top, then generate files which contain all the permutations for parameters

# List out parameters and bounds with numpy, reduce everything to a list

# Estimate the number of permutations with the lists provided, provide a warning if it is over 100, and an option to bail

import sys 
import os 
import numpy as np

class incrementer:
    def __init__(self):
        self.val = 0
    def inc(self):
        self.val+=1
    def get_val(self):
        return self.val


def get_combination(mission_vars, remaining_options, fname, file_idx):
    if len(remaining_options) == 0:
        #Reached base case - now write the arguments to a file
        print(f"{file_idx.val}")
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
            get_combination(s, c, fname, file_idx)


def gen_var_plugs(dirname, fname, collection, rules=None):
    os.makedirs(dirname, exist_ok=True)
    fname = dirname+"/"+fname
    ub = 1
    for _, val in collection:
        ub*=len(val)

    if(ub > 1000):
        print(f"The current configuration would generate at most {ub} missions - are you sure? It's not to late.")
        decision = input("Y/n: ")
        if(decision.lower().count("n") >= 1):
            print("Aborting")
            exit(0)

    file_idx = incrementer()
    mission_vars = {}
    get_combination(mission_vars, collections, fname, file_idx)



compression = np.linspace(0,1,2)
aft_patience = ["true", "false"]
holding_policy = ["zero", "off", "curr_hdg", "setpt_hdg"]
active_convoying = ["true", "false"]
slower_convoy_range = list(np.linspace(3,10,3))
ideal_convoy_range = list(np.linspace(3,20,3))
faster_convoy_range = list(np.linspace(7,20,3))
full_lag_convoy_range = list(np.linspace(12,45,3))
lag_speed_delta = list(np.linspace(0.5,1,2))

collections = [("compression",compression),
               ("aft_patience",aft_patience),
               ("holding_policy",holding_policy),
               ("active_convoying",active_convoying),
               ("slower_convoy_range",slower_convoy_range),
               ("ideal_convoy_range",ideal_convoy_range),
               ("faster_convoy_range",faster_convoy_range),
               ("full_lag_convoy_range",full_lag_convoy_range),
               ("lag_speed_delta",lag_speed_delta)]

gen_var_plugs("convoy_variations_1", "convoy_params", collections)
