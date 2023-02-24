j=500
for i in instances/900_ni/*.csv
do
	#echo $i
	#let j=j+47
	./a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --number_depots 4 --depot 5585 5586 5587 5588 --type_vehicles 3 --number_vehicles 90 90 90 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
	#
	#gdb -- a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --number_depots 4 --depot 5585 5586 5587 5588 --type_vehicles 3 --number_vehicles 30 60 30 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
done











