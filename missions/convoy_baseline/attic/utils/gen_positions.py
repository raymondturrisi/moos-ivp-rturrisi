#!/bin/python
#Raymond Turrisi - for seeding monte-carlo simulations with a collection of agents, starting positions, and headings

#For 1-N agents, generate K random configurations within a region

import sys 
import os 
import numpy as np 

def show_example():
    print("1) Create 20 random initial conditions for four agents within a bounding polygon")
    print("\t python3 gen_positions.py --amt=4 --points=4,4:5,5:-12,16 --nfiles=20")
    print("2) Create 10 random initial conditions for eight agents within a circle on the map")
    print("\t python3 gen_positions.py --amt=8 --center=4,4 --radius=20 --nfiles=10")

def show_help():
    pass 

agent_names = ["abe", 
               "ben", 
               "cal", 
               "deb", 
               "eve", 
               "fin", 
               "gus", 
               "hal", 
               "ira", 
               "jen", 
               "ken", 
               "leo", 
               "max", 
               "ned", 
               "oak", 
               "pat", 
               "quin", 
               "rob", 
               "sam", 
               "tim", 
               "uma", 
               "val", 
               "wes", 
               "xan", 
               "yen", 
               "zac"]

agent_colors = [
    "yellow",
    "red",
    "dodger_blue",
    "green",
    "purple",
    "orange",
    "white",
    "dark_green",
    "dark_red",
    "cyan",
    "coral",
    "brown",
    "bisque",
    "white",
    "pink",
    "darkslateblue",
    "brown",
    "burlywood",
    "goldenrod",
    "ivory",
    "khaki",
    "lime",
    "peru",
    "powderblue",
    "plum",
    "sienna",
    "sandybrown",
    "navy",
    "olive",
    "magenta"
]

def get_more_names(agent_names):
    remaining = amt - len(agent_names)
    new_names = []
    while(remaining != 0):
        name = names.get_first_name().lower().capitalize()
        while(agent_names.count(name) > 0 or 
                new_names.count(name) > 0 or 
                len(name) <= 3 or 
                len(name) >= 6):
            name = names.get_first_name().lower().capitalize()
        new_names.append(name)
        remaining-=1
    return new_names

def get_rnd_point_in_poly(polygon):
    minx, miny, maxx, maxy = polygon.bounds
    point = sh.Point(np.random.uniform(minx,maxx), np.random.uniform(miny, maxy))
    while(not polygon.contains(point)):
        point = sh.Point(np.random.uniform(minx,maxx), np.random.uniform(miny, maxy))
    x,y = point.xy
    return x[0], y[0]
        

if __name__ == "__main__":
    #Initialize variables
    amt = 0
    cx,cy = None, None #meters from origin warp!
    is_circular = False
    is_polygon = False
    radius = 0 #meters!
    points = []
    files = 1
    #Parse arguments
    for i, arg in enumerate(sys.argv[1:]):
        if(arg[0:1] == "-e" or arg[0:8] == "--example"):
            #Show example use cases
            show_example()
            exit(0)
        elif(arg[0:1] == "-h" or arg[0:3] == "--help"):
            #Show help and arguments
            show_help()
            exit(0)
        elif(arg[0:5] == "--amt"):
            #Get the number of desired agents. If it is less than 26, it will be deterministic and similar
            # to how we name our vehicles. If it is more than 26, the first 26 will be deterministic, but 
            # the rest of the agents will only be alphabetized. Only names between 3 and 5 characters are allowed
            # This has been tested with up to 100 agents with no issues
            amt = int(arg[6:])
            if(amt <= len(agent_names)):
                agent_names = agent_names[0:amt] 
            else:
                import names 
                new_names = get_more_names(agent_names)
                agent_names.extend(list(sorted(new_names)))
        elif(arg[0:8] == "--nfiles"):
            #Will generate nfiles with these parameters
            files = int(arg[9:])
        elif(arg[0:8] == "--center"):
            #Center of randomly generated agents within a circle
            #The use of this argument excludes a set of points, which would then imply a polygon
            is_circular = True
            center_args=arg[9:]
            cx,cy = [int(c) for c in center_args.split(",")]
        elif(arg[0:8] == "--radius"):
            #Radii about a center of randomly generated agents within a circle
            #The use of this argument excludes a set of points, which would then imply a polygon
            is_circular = True 
            radius = float(arg[9:])
        elif(arg[0:8] == "--points"):
            import shapely.geometry as sh
            #Extract points which arrive as --points=4,4:5,5:-12,16 , which is moosonic
            is_polygon = True 
            points_str = arg[9:].replace(" ","")
            points = [tuple([float(num) for num in elem.split(",")]) for elem in points_str.split(":")]
        else:
            print(f"Unknown argument {arg}")
            exit(1)
    if(is_polygon and is_circular):
        print(f"Misused arguments. Cannot provide arguments which bound agent locations within a circle and a polygon")
        exit(2)

    #If we are here, we have what we need to seed files

    #Make the directory for which all the files will be dumped, and write an info file with the arguments describing how 
    # the files which share this directory were spawned
    dirname = f"agent_poses_{amt}-agents_{files}-files"
    print(dirname)
    print(os.path.dirname(dirname))
    os.makedirs(dirname, exist_ok=True)
    
    with open(f"{dirname}/info.txt","w") as f:
        f.write(f"Automatically generated with {sys.argv[0]}\n")
        f.write(f"\tArgs: {sys.argv[1:]}\n")

    for f in range(1,files+1):
        fname = f"{dirname}/f{f}_{amt}-agents_{'poly' if is_polygon else 'circ'}.txt"
        if(is_polygon):
            polygon = sh.Polygon(points)

        with open(f"{fname}","w") as f:
            for idx, a in enumerate(agent_names):
                #Get the starting positions      
                if(is_polygon):
                    x,y = get_rnd_point_in_poly(polygon)
                else:
                    x = cx+np.random.random()*radius
                    y = cy+np.random.random()*radius

                #Get the starting heading
                hdg = int(np.random.random()*360)

                #Write the agent and this information to a file
                f.write(f"{a}, {float(x):0.2f}, {float(y):0.2f}, {int(hdg)}, {agent_colors[idx%len(agent_colors)]}")
                if(idx <= len(agent_names)-2):
                    f.write("\n")