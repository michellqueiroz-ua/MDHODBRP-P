j=500
for i in instances/900_180_1000/*.csv
do
	#echo $i
	let j=j+47
	./a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --number_depots 4 --depot 5585 5586 5587 5588 --type_vehicles 3 --number_vehicles 20 20 40 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
	#
	#gdb -- a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --number_depots 4 --depot 5585 5586 5587 5588 --type_vehicles 3 --number_vehicles 30 60 30 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
done

for i in instances/900_1000_3000/*.csv
do
	#echo $i
	let j=j+48
	./a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --number_depots 4 --depot 5584 5585 5586 5587 --type_vehicles 3 --number_vehicles 20 20 40 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
	#let j=j+47
	#lldb -- a.out --filename_requests "Chicago,Illinois_ODBRP_7.0_8.0_300_0.90_1800.0_0.0_3000_6000_4.csv" --filename_travel_time "Chicago, Illinois.tt.matrix.stations.csv" --number_depots 1 --depot 5584 --type_vehicles 3 --number_vehicles 20 40 20 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800
done

for i in instances/900_3000_6000/*.csv
do
	#echo $i
	let j=j+49
	./a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --number_depots 4 --depot 5584 5585 5586 5587 --type_vehicles 3 --number_vehicles 20 20 40 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
	#let j=j+47
	#lldb -- a.out --filename_requests "Chicago,Illinois_ODBRP_7.0_8.0_300_0.90_1800.0_0.0_3000_6000_4.csv" --filename_travel_time "Chicago, Illinois.tt.matrix.stations.csv" --number_depots 1 --depot 5584 --type_vehicles 3 --number_vehicles 20 40 20 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800
done

for i in instances/900_ni/*.csv
do
	#echo $i
	let j=j+50
	./a.out --filename_requests $i --seed $j --filename_travel_time "travel_time_updated3.csv" --number_depots 4 --depot 5584 5585 5586 5587 --type_vehicles 3 --number_vehicles 20 20 40 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800 --total_requests 900
	#let j=j+47
	#lldb -- a.out --filename_requests "Chicago,Illinois_ODBRP_7.0_8.0_300_0.90_1800.0_0.0_3000_6000_4.csv" --filename_travel_time "Chicago, Illinois.tt.matrix.stations.csv" --number_depots 1 --depot 5584 --type_vehicles 3 --number_vehicles 20 40 20 --capacity_vehicles 4 8 20 --init_temperature 1.3 --lamba 0.9 --maxnrep 350 --increase_rep 800
done









