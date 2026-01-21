#!/bin/bash

#SBATCH --job-name=CreateNetwork
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
#SBATCH --partition=skylake

module load atools
module load calcua/2024a GCCcore/13.3.0

source <(aenv --data $1)
srun -n 1 -c ${SLURM_CPUS_PER_TASK} ./a.out --filename_requests "3" $input_file --seed $seed1 --filename_travel_time "travel_time_updated3.csv" --output_file "R1_nightlife_S2_C.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 15 --number_vehicles2 8 --number_vehicles3 0 --capacity_vehicles 4 8 12 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
