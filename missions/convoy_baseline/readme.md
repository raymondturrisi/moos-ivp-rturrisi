# Readme

1) This mission is currently designed for monte carlo and batch simulations
    ./blaunch.sh 10 --cc="generator_set1_configs.py" --crange="[1..100]" --cp="generator_set1_params.py" --prange="[1..100]" --k=2 --pp=post_sequence.sh
    Timewarp of 10
    Specific configuration generator function and the desired range for this script to run through
    Specific parameter generator function and the desired range for this script to run through
    The number of trials for each condition
    The post process script

./blaunch.sh

Uses default arguments for the underlying launch scripts
For a job/launch script, will run with uQueryDB to poke the shoreside until conditions are met to bring the system down, or until timeout conditions are met
Once termination conditions are met, will run the post process script, before moving onto the next index

for configuration ...
    for parameter set ...
        for i in k trials ...
            ./launch.sh *args*
            ...
            while termination conditions are not met
                watch for termination conditions
                    break if met
            given the mission data file, and if given a post process script, pass the mission data file to the post processing script