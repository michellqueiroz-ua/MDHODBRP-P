#include "Input.h"

listD depot;
map<int, int> station_id_map;
matrixSS travel_time;
listS stations_ids;
int number_nodes;
int n;

double comp_time;

listV vehicle_type;

listT number_vehicles;
listD number_vehicles_at_depot;
matrixDV vehicles_at_depot;
int total_number_vehicles;
listT maxcapacity;
listV vehicle_located_at_depot;
int total_served_passengers;
int number_type_vehicles;
int number_depots;
int number_requests;

matrixVSS M, W;
listP time_stamp, earliest_departure, latest_departure, earliest_arrival, latest_arrival, direct_travel_time;
listS q;

map<int, int> nodes;
map<int, int> type_node;

listP delay;
listP passengers_departure_time_from_home;
matrixPS stops_origin, stops_destination;
matrixPS walking_time_stops_origin, walking_time_stops_destination;
listP number_stops_origin, number_stops_destination;
int number_stations;

int ts_min, ts_max;

void input_travel_time(char *filename) {

	fstream file (filename, ios::in);
	string line, data;
	int s, stop1, stop2;
	//<<"here ";
	if(file.is_open())
	{
		getline(file, line);
		stringstream str(line);
		s = 0;
		getline(str, data, ',');

		int count = 0;
		while(getline(str, data, ',')) { //reads the header

			if (count > 0) {
				stations_ids[s] = stoi(data);
				//cout<<stations_ids[s]<<" ";
				//station_map.insert(pair<int, int>(stations_ids[s], s));
				station_id_map[stations_ids[s]] = s;
				s = s + 1;
			}
			count++;
		}
		//<<endl;


		stop1 = 0;
		
		while(getline(file, line))
		{

			stringstream str(line);
			stop2 = 0;
			getline(str, data, ',');
			//<<data<<endl;
			
			count = 0;
			while(getline(str, data, ',')) {
				//cout<<data<<endl;
				if (count > 0) {
					travel_time[stop1][stop2] = stoi(data);
					travel_time[stop2][stop1] = stoi(data);
					stop2 = stop2 + 1;
				} 
				count++;
			}
			//<<stop1<<" "<<stop2<<endl;
			stop1 = stop1 + 1;

		}

	}
}

void input_requests(char *filename) {

	fstream file (filename, ios::in);
	string line, data, stop;
	int p, s;

	number_nodes = 0;
	if(file.is_open())
	{
		getline(file, line);
		stringstream str(line);
		while(getline(str, data, ',')); //reads the header
		//<<data<<endl;
		while(getline(file, line))
		{

			stringstream str(line);
			getline(str, data, ',');
			p = stoi(data);
			//<<p<<endl;
			
			getline(str, data, ',');
			time_stamp[p] = stoi(data);
			//<<time_stamp[p]<<endl;

			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');
			getline(str, data, ',');

			earliest_departure[p] = stoi(data);

			//printf("%d\n", earliest_departure[p]);
			
			getline(str, data, ',');
			latest_departure[p] = stoi(data);
			//printf("%d\n", latest_departure[p]);

			getline(str, data, ','); //direct travel time
			direct_travel_time[p] = stoi(data);
			getline(str, data, ',');
			//getline(str, data, ',');


			getline(str, data, ',');
			bool leave_loop = false;
			if (data.find(']') != std::string::npos)
					leave_loop = true; 
			data.erase(remove(data.begin(), data.end(), '['), data.end());
			data.erase(remove(data.begin(), data.end(), '"'), data.end());
			data.erase(remove(data.begin(), data.end(), ']'), data.end());
			
			//<<data<<endl;
			//<<data.end()<<endl;
			s = 0;
			stops_origin[p][s] = number_nodes;
			nodes[number_nodes] = stoi(data);
			type_node[number_nodes] = 1;
			number_nodes++;
			//printf("%d ", stops_origin[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					stops_origin[p][s] = number_nodes;
					nodes[number_nodes] = stoi(stop);
					type_node[number_nodes] = 1;
					number_nodes++;
					//printf("%d ", stops_origin[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			//printf("\n");
			number_stops_origin[p] = s;

			getline(str, data, ',');
			leave_loop = false;
			if (data.find(']') != std::string::npos)
					leave_loop = true; 
			data.erase(remove(data.begin(), data.end(), '['), data.end());
			data.erase(remove(data.begin(), data.end(), '"'), data.end());
			data.erase(remove(data.begin(), data.end(), ']'), data.end());
			
			//<<data<<endl;
			//<<data.end()<<endl;
			s = 0;
			walking_time_stops_origin[p][s] = stoi(data);
			//printf("%d ", stops_origin[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					walking_time_stops_origin[p][s] = stoi(stop);
					//printf("%d ", stops_origin[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			
			
			getline(str, data, ',');
			latest_arrival[p] = stoi(data);

			latest_departure[p] = latest_arrival[p] - direct_travel_time[p];
			earliest_arrival[p] = earliest_departure[p] + direct_travel_time[p];

			//<<latest_arrival[p]<<endl;

			/*
			getline(str, data, ',');
			data.erase(remove(data.begin(), data.end(), '['), data.end());
			data.erase(remove(data.begin(), data.end(), ']'), data.end());
			s = 0;
			stringstream str3(data);
			while(getline(str3, stop, ',')) {
				stops_destination[p][s] = stoi(stop);
				s = s + 1;
			}
			number_stops_destination = s;*/

			getline(str, data, ',');
			leave_loop = false;
			if (data.find(']') != std::string::npos)
					leave_loop = true; 
			data.erase(remove(data.begin(), data.end(), '['), data.end());
			data.erase(remove(data.begin(), data.end(), '"'), data.end());
			data.erase(remove(data.begin(), data.end(), ']'), data.end());
			
			//<<data<<endl;
			//<<data.end()<<endl;
			s = 0;
			stops_destination[p][s] = number_nodes;
			nodes[number_nodes] = stoi(data);
			type_node[number_nodes] = 2;
			number_nodes++;
			//printf("%d ", stops_destination[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					stops_destination[p][s] = number_nodes;
					nodes[number_nodes] = stoi(stop);
					type_node[number_nodes] = 2;
					number_nodes++;
					//printf("%d ", stops_destination[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			//printf("\n");
			number_stops_destination[p] = s;

			getline(str, data, ',');
			leave_loop = false;
			if (data.find(']') != std::string::npos)
					leave_loop = true; 
			data.erase(remove(data.begin(), data.end(), '['), data.end());
			data.erase(remove(data.begin(), data.end(), '"'), data.end());
			data.erase(remove(data.begin(), data.end(), ']'), data.end());
			
			//<<data<<endl;
			//<<data.end()<<endl;
			s = 0;
			walking_time_stops_destination[p][s] = stoi(data);
			//printf("%d ", stops_destination[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					walking_time_stops_destination[p][s] = stoi(stop);
					//printf("%d ", stops_destination[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			//printf("\n");
			
			getline(str, data, ',');
			getline(str, data, ',');

		}

	} else cout<<"Could not open the file\n";
}