# Set the number of parallel processes
num_parallel=4

# Define the function to execute your binary
run_binary() {
	counterx=0
	for iter in 1 2 3 4 5
	do
		counterx=$((counterx+1))
		input_file="$1"
		seed1=0
		seed1=$(($seed1 + $counterx))
	    file_name=$(basename "$input_file")
	    ./a.out --filename_requests $input_file --seed $seed1 --filename_travel_time "travel_time_updated3.csv" --output_file "commuting2KR16.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 7 --number_vehicles2 7 --number_vehicles3 7 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900  
	done
}

export -f run_binary

# Use find to generate a list of input files and pass them to parallel
find ../../large-instances/commuting2 -type f -name "*.csv" | parallel -j $num_parallel run_binary {}

# Wait for all parallel processes to finish (old 502)
wait










