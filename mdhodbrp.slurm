#!/bin/bash
#

#SBATCH --job-name=CreateNetwork
#SBATCH --nodes=1
#SBATCH --ntasks=7
#SBATCH --cpus-per-task=2
#SBATCH --time=24:00:00
#SBATCH -o stdout.file
#SBATCH -e stderr.file
#SBATCH --partition=skylake

for input_file in `find ../../large-instances/festivalT -type f -name "*.csv"` ; do
    counterx=0
    for iter in 1
    do
        counterx=$((counterx+1))
        seed1=0
        seed1=$(($seed1 + $counterx))
	file_name=$(basename "$input_file")
	srun -n 1 -c ${SLURM_CPUS_PER_TASK} ./a.out --filename_requests $input_file --seed $seed1 --filename_travel_time "travel_time_updated3.csv" --output_file "R1_festival_S1.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 7 --number_vehicles2 7 --number_vehicles3 7 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900 &
    done
done

wait

