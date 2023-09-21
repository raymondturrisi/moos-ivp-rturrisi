#!/bin/bash

import numpy as np
import gen_mission_utils as gmu

#

compression = list(np.linspace(0,1,3))
aft_patience = ["true", "false"]
holding_policy = ["curr_hdg", "setpt_hdg"]
active_convoying = ["true", "false"]
slower_convoy_range = list(np.linspace(3,10,3))
ideal_convoy_range = list(np.linspace(7,20,3))
faster_convoy_range = list(np.linspace(10,20,3))
full_lag_convoy_range = list(np.linspace(12,45,3))
lag_speed_delta = list(np.linspace(0.5,1,3))

collections = [("compression",compression),
               ("aft_patience",aft_patience),
               ("holding_policy",holding_policy),
               ("active_convoying",active_convoying),
               ("slower_convoy_range",slower_convoy_range),
               ("ideal_convoy_range",ideal_convoy_range),
               ("faster_convoy_range",faster_convoy_range),
               ("full_lag_convoy_range",full_lag_convoy_range),
               ("lag_speed_delta",lag_speed_delta)]

rules = [lambda c: c["slower_convoy_range"] < c["ideal_convoy_range"] and c["ideal_convoy_range"]< c["faster_convoy_range"]]

gmu.gen_var_plugs("convoy_variations_1", "plug_convoy_params", collections, rules)
