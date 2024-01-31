#!/bin/bash

module load atools

run_file=parallelheur18.csv

echo "input_file; seed1" > $run_file
for input_file in `find ../../large-instances/missingconc2 -type f -name "*.csv"` ; do
    counterx=6
    for iter in 1
    do
        counterx=$((counterx+1))
        seed1=0
        seed1=$(($seed1 + $counterx))
        file_name=$(basename "$input_file")
        echo "$input_file; $seed1" >> $run_file
    done
done

sbatch --array $(arange --data $run_file) mdhodbrp-atools.slurm $run_file


