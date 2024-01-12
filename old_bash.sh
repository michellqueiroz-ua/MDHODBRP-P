j=500
for i in ../../large-instances/festival1/*.csv
do
	#echo $i
	#let j=j+47
	#./a.out --filename_requests $input_file --seed $j --filename_travel_time "travel_time_updated3.csv" --output_file "concert.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 5 --number_vehicles2 5 --number_vehicles3 5 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
	#
	gdb -- a.out --filename_requests $input_file --seed $j --filename_travel_time "travel_time_updated3.csv" --output_file "festival8.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 5 --number_vehicles2 5 --number_vehicles3 5 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
done











