#!/bin/bash
#

#SBATCH --job-name=CreateNetwork
#SBATCH --ntasks=4
#SBATCH --cpus-per-task=6
#SBATCH --mem-per-cpu=4g
#SBATCH --time=24:00:00
#SBATCH --mail-type=ALL
#SBATCH --mail-user=michell.queiroz@uantwerpen.be
#SBATCH -o stdout.file
#SBATCH -e stderr.file
#SBATCH --partition=skylake

module load calcua/supported
module load parallel/20200422

./bash.sh
