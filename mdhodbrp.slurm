#!/bin/bash
#

#SBATCH --job-name=CreateNetwork
#SBATCH --ntasks=12
#SBATCH --cpus-per-task=5
#SBATCH --mem-per-cpu=3g
#SBATCH --time=24:00:00
#SBATCH --mail-type=ALL
#SBATCH --mail-user=michell.queiroz@uantwerpen.be
#SBATCH -o stdout.file
#SBATCH -e stderr.file
#SBATCH --partition=broadwell

module load calcua/supported
module load parallel/20200422

./bash.sh
