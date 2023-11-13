#!/bin/bash
./blaunch.sh --cg generators/generator_set1_configs.py \
--crange="[1..2]" \
--pg generators/generator_set1_params.py \
--nogui \
--prange="[1978..2810]" --k 2
