#!/bin/bash

module load atools
module load calcua/2024a GCCcore/13.3.0

run_file=parallelheur9.csv

echo "input_file; seed1" > $run_file
for input_file in `find ../../instances/EXP_NN/2700_NN -type f -name "*.csv"` ; do
    counterx=6
    for iter in {1..10}
    do
        counterx=$((counterx+1))
        seed1=111
        seed1=$(($seed1 + $counterx))
        file_name=$(basename "$input_file")
        echo "$input_file; $seed1" >> $run_file
    done
done

sbatch --account ap_enm --array $(arange --data $run_file) mdhodbrp-atools9.slurm $run_file


