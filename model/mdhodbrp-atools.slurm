#!/bin/bash

#SBATCH --job-name=CreateNetwork
#SBATCH --ntasks=1 --cpus-per-task=64
#SBATCH --time=24:00:00
#SBATCH --partition=zen2

module load atools
module load Gurobi

source <(aenv --data $1)
srun ./my_gurobi_program --filename_requests $input_file --seed $seed1 --filename_travel_time "travel_time_updated3.csv" --output_file "EXP_20_REQUESTS_MODEL.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 15 --number_vehicles2 5 --number_vehicles3 0 --capacity_vehicles 4 8 12 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
