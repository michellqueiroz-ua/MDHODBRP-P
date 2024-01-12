#!/bin/bash
#

#SBATCH --job-name=CreateNetwork
#SBATCH --ntasks=8
#SBATCH --cpus-per-task=8
#SBATCH --mem-per-cpu=4g
#SBATCH --time=24:00:00
#SBATCH --mail-type=ALL
#SBATCH --mail-user=michell.queiroz@uantwerpen.be
#SBATCH -o stdout.file
#SBATCH -e stderr.file
#SBATCH --partition=zen2

module load calcua/supported
module load parallel/20200422

./bash.sh
