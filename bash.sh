# Set the number of parallel processes
num_parallel=4

# Define the function to execute your binary
run_binary() {
	input_file="$1"
	seed1=502
    file_name=$(basename "$input_file")
    ./a.out --filename_requests $input_file --seed $seed1 --filename_travel_time "travel_time_updated3.csv" --output_file "concert3.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 14 --number_vehicles2 14 --number_vehicles3 14 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900  
}

export -f run_binary

# Use find to generate a list of input files and pass them to parallel
find ../../large-instances/concert -type f -name "*.csv" | parallel -j $num_parallel run_binary {}

# Wait for all parallel processes to finish
wait










