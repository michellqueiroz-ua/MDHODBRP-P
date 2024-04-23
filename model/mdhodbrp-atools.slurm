#!/bin/bash

#SBATCH --job-name=CreateNetwork
#SBATCH --ntasks=1 --cpus-per-task=64
#SBATCH --time=4:00:00
#SBATCH --partition=zen2

module load atools
module load Gurobi

source <(aenv --data $1)
srun ./my_gurobi_program --filename_requests $input_file --seed $seed1 --filename_travel_time "travel_time_updated3.csv" --output_file "testing_model.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 7 --number_vehicles2 7 --number_vehicles3 7 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
