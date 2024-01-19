j=502
for i in ../../large-instances/nightlifeT/*.csv
do
	#echo $i
	#let j=j+47
	./a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --output_file "nightlife1.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 12 --number_vehicles2 12 --number_vehicles3 12 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
	#
	#gdb -- a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --output_file "nightlife1.txt" --number_depots 3 --depot 5825 5826 5827 --type_vehicles 3 --number_vehicles1 12 --number_vehicles2 12 --number_vehicles3 5 --capacity_vehicles 8 16 32 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
done











