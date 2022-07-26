#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <curses.h>
#include <time.h>
#include <math.h>
#include <climits>
#include <ctime>
#include <map>
#include <sys/time.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
//#include<Eigen/Dense>
//using namespace Eigen;
using namespace std;

#define maxvehicles 100
#define maxpassengers 3000
#define maxstations 6000
#define maxtotalcapacity 40
#define maxtypevehicles 40
#define maxnumberdepots 10

typedef int listP[maxpassengers + 1];
typedef int matrixVP[maxvehicles + 1][maxpassengers + 1];
typedef int matrixVC[maxvehicles + 1][maxtotalcapacity*2];
typedef int matrixPS[maxpassengers + 1][maxstations + 1];
typedef int matrixSS[maxstations + 1][maxstations + 1];
typedef int matrixDV[maxnumberdepots + 1][maxvehicles + 1];
typedef int listS[maxstations + 1];
typedef int listV[maxvehicles + 1];
typedef int listD[maxnumberdepots + 1];
typedef int listT[maxtypevehicles + 1];
typedef double listTd[maxtypevehicles + 1];
typedef int matrixVSS[maxvehicles + 1][maxstations + 1][maxstations + 1]; //to know which stations are visited after another (in case its necessary)
typedef int matrixVPP[maxvehicles + 1][maxpassengers + 1][maxtotalcapacity + 1];


int n;
double comp_time;

//matrixVP stopi, stopj; //route of the bus: stopj is visited after stopi)
vector<vector<int> > stops(maxvehicles + 1);
listV number_stops; //number of stops vehicle v is performing for each route
vector<vector<int> > number_passengers_action(maxvehicles + 1); //store number of passengers performing an action at given vehicle and given stop
vector<vector<int> > arrival_time_stop(maxvehicles + 1); //stores which time the stop was/will be visited by the bus
vector<vector<int> > departure_time_stop(maxvehicles + 1);
vector<vector<int> > slack_time(maxvehicles + 1);
vector<vector<int> > free_capacity(maxvehicles + 1);
vector<vector<vector<int> > > action_passengers(maxvehicles + 1); //store which passengers are performing an action at given vehicle and given stop
listS saved_arrival_time, saved_departure_time, saved_slack_time;
vector<int> filtered_vehicles, vehicles_still_depot;
vector<vector<int> > passengers_at_vehicle(maxvehicles + 1);

//best solution saving
vector<vector<int> > best_stops(maxvehicles + 1);
vector<vector<vector<int> > > best_action_passengers(maxvehicles + 1);
vector<vector<int> > best_number_passengers_action(maxvehicles + 1);
vector<vector<int> > best_arrival_time_stop(maxvehicles + 1); 
vector<vector<int> > best_departure_time_stop(maxvehicles + 1);
vector<vector<int> > best_slack_time(maxvehicles + 1); 
vector<vector<int> > best_free_capacity(maxvehicles + 1);
listP best_user_ride_time;
listV best_number_stops;
listP best_vehicle_assigned, best_assigned_to_3rd_party;
listP best_passengers_departure_time_from_home;
int best_served_passengers;
int best_served_passengers_3party;
int best_total_served_passengers;
int total_requests, seed;

//intermidate solution saving
vector<vector<int> > intm_stops(maxvehicles + 1);
vector<vector<vector<int> > > intm_action_passengers(maxvehicles + 1);
vector<vector<int> > intm_number_passengers_action(maxvehicles + 1);
vector<vector<int> > intm_arrival_time_stop(maxvehicles + 1); 
vector<vector<int> > intm_departure_time_stop(maxvehicles + 1);
vector<vector<int> > intm_slack_time(maxvehicles + 1); 
vector<vector<int> > intm_free_capacity(maxvehicles + 1);
listP intm_user_ride_time;
listV intm_number_stops;
listP intm_vehicle_assigned, intm_assigned_to_3rd_party;
listP intm_passengers_departure_time_from_home;
int intm_served_passengers;
int intm_served_passengers_3party;
int intm_total_served_passengers;

int oldy_urt, computedDELTA, totalcomputedDELTA;
int total_difference;
int addedAtV;
matrixVC arrival_time_stop_temp, departure_time_stop_temp;
listP user_ride_time_temp, user_ride_time_start, affected_passengers;
//listV total_capacity, max_capacity;
listV blocked_vehicles;
listV cost_trip;
listV vehicle_type;
matrixVP blocked_positions;
listV current_position;
listD depot;
listT number_vehicles;
listD number_vehicles_at_depot;
matrixDV vehicles_at_depot;
int total_number_vehicles;
listT maxcapacity;
listV vehicle_located_at_depot;
int served_passengers;
int served_passengers_3party;
int total_served_passengers;
int number_type_vehicles;
int number_depots;

matrixSS travel_time;
listS stations_ids;
map<int, int> station_id_map;

//information about the requests
listP time_stamp, earliest_departure, latest_departure, latest_arrival, direct_travel_time;
listP delay;
listP passengers_departure_time_from_home;
matrixPS stops_origin, stops_destination;
matrixPS walking_time_stops_origin, walking_time_stops_destination;
listP number_stops_origin, number_stops_destination;
int number_stations;

listP route_assigned, vehicle_assigned;
listP user_ride_time, assigned_to_3rd_party;
int total_user_ride_time;
int best_total_user_ride_time;
int intm_total_user_ride_time;

int current_time;
clock_t start_time;
double elapsed;
int max_flex_delay;

listTd ODB_booking_fee, ODB_base_fare, ODB_per_minute_charge, ODB_km_charge, ODB_surge_multiplier, ODB_minimum_fare;
double tp_booking_fee, tp_base_fare, tp_per_minute_charge, tp_km_charge, tp_surge_multiplier, tp_minimum_fare;

//SA parameters
double init_temperature, lambda;
int maxnrep, ntrials, increase_rep;

struct Insertions {

	int increase_length;
	int pos_station;
	int sel_station;
	int v;
	int delay;
	bool repeated_station;
	int passengers_departure_time_from_home;
};

Insertions insertions[5000];
int curr_number_insertions; 

bool comparator( Insertions a, Insertions b){
	if(a.increase_length < b.increase_length)
		return 1;
	else 
		return 0;
}


double get_wall_time(){
    struct timeval time;
    if (gettimeofday(&time,NULL)){
        //  Handle error
        return 0;
    }
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

void update_URT(int v){

	int current_user_ride_time, difference;
	bool leave_loop = false;
	for (int i=0; i<=number_stops[v];i++) {
		for (int j=0; j<number_passengers_action[v][i];j++) {
			int save_p = action_passengers[v][i][j];
			for (int k=i+1; k<=number_stops[v];k++) {
				leave_loop = false;
				for (int l=0; l<number_passengers_action[v][k];l++) {
					if (action_passengers[v][k][l] == save_p){
						current_user_ride_time = arrival_time_stop[v][k] - departure_time_stop[v][i];
						if (current_user_ride_time != user_ride_time[save_p]) {
							difference = current_user_ride_time -  user_ride_time[save_p];
							//<<"difference: "<<difference<<endl;
							total_user_ride_time += difference;
							//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
							user_ride_time[save_p] = current_user_ride_time;
							l = number_passengers_action[v][k]+1;
							leave_loop = true;
						}
					}
				}
				if (leave_loop)
					k = number_stops[v]+2;
			}	
		} 
	}
}

void save_best_solution(){

	
	std::copy(std::begin(passengers_departure_time_from_home), std::end(passengers_departure_time_from_home), std::begin(best_passengers_departure_time_from_home));
	best_stops = stops;
	std::copy(std::begin(action_passengers), std::end(action_passengers), std::begin(best_action_passengers));
	
	best_number_passengers_action = number_passengers_action;
	best_arrival_time_stop = arrival_time_stop;
	
	best_departure_time_stop = departure_time_stop;
	best_slack_time = slack_time;
	
	best_free_capacity = free_capacity;
	std::copy(std::begin(user_ride_time), std::end(user_ride_time), std::begin(best_user_ride_time));
	std::copy(std::begin(number_stops), std::end(number_stops), std::begin(best_number_stops));
	std::copy(std::begin(vehicle_assigned), std::end(vehicle_assigned), std::begin(best_vehicle_assigned));
	best_served_passengers = served_passengers;
	best_served_passengers_3party = served_passengers_3party;
	best_total_served_passengers = total_served_passengers;
	std::copy(std::begin(assigned_to_3rd_party), std::end(assigned_to_3rd_party), std::begin(best_assigned_to_3rd_party));
	best_total_user_ride_time = total_user_ride_time;
}

void return_best_solution(){

	//<<"hier1"<<endl;
	//<<"hier2 "<<best_action_passengers.size()<<" "<<action_passengers.size()<<endl;
	/*for(int v=0;v<=total_number_vehicles;v++) {
		cout<<"v "<<v<<": "<<endl;
		cout<<"stps "<<best_number_stops[v]<<": "<<endl;
		for (int l=0; l<=best_number_stops[v];l++) {
			cout<<"nps "<<best_number_passengers_action[v][l]<<": "<<endl;
			for (int m=0; m<best_number_passengers_action[v][l];m++) 
				cout<<best_action_passengers[v][l][m]<<" ";

			}
		cout<<endl;
	}*/
	std::copy(std::begin(best_passengers_departure_time_from_home), std::end(best_passengers_departure_time_from_home), std::begin(passengers_departure_time_from_home));
	stops = best_stops;
	//<<"hier4"<<endl;
	//<<"hier5"<<endl;
	std::copy(std::begin(best_action_passengers), std::end(best_action_passengers), std::begin(action_passengers));
	//<<"hier4"<<endl;
	//<<"hier6"<<endl;
	number_passengers_action = best_number_passengers_action;
	//<<"hier5"<<endl;
	arrival_time_stop = best_arrival_time_stop;
	departure_time_stop = best_departure_time_stop;
	slack_time = best_slack_time;
	//<<"hier6"<<endl;
	free_capacity = best_free_capacity;
	std::copy(std::begin(best_user_ride_time), std::end(best_user_ride_time), std::begin(user_ride_time));
	std::copy(std::begin(best_number_stops), std::end(best_number_stops), std::begin(number_stops));
	std::copy(std::begin(best_vehicle_assigned), std::end(best_vehicle_assigned), std::begin(vehicle_assigned));
	served_passengers = best_served_passengers;
	served_passengers_3party = best_served_passengers_3party;
	total_served_passengers = best_total_served_passengers;
	std::copy(std::begin(best_assigned_to_3rd_party), std::end(best_assigned_to_3rd_party), std::begin(assigned_to_3rd_party));
	total_user_ride_time = best_total_user_ride_time;
}

void save_intm_solution(){

	//<<"hiers1"<<endl;
	std::copy(std::begin(passengers_departure_time_from_home), std::end(passengers_departure_time_from_home), std::begin(intm_passengers_departure_time_from_home));
	intm_stops = stops;
	std::copy(std::begin(action_passengers), std::end(action_passengers), std::begin(intm_action_passengers));
	//<<"hiers2"<<endl;
	intm_number_passengers_action = number_passengers_action;
	intm_arrival_time_stop = arrival_time_stop;
	//<<"hiers3"<<endl;
	intm_departure_time_stop = departure_time_stop;
	intm_slack_time = slack_time;
	//<<"hiers4"<<endl;
	intm_free_capacity = free_capacity;
	std::copy(std::begin(user_ride_time), std::end(user_ride_time), std::begin(intm_user_ride_time));
	std::copy(std::begin(number_stops), std::end(number_stops), std::begin(intm_number_stops));
	std::copy(std::begin(vehicle_assigned), std::end(vehicle_assigned), std::begin(intm_vehicle_assigned));
	intm_served_passengers = served_passengers;
	intm_served_passengers_3party = served_passengers_3party;
	intm_total_served_passengers = total_served_passengers;
	std::copy(std::begin(assigned_to_3rd_party), std::end(assigned_to_3rd_party), std::begin(intm_assigned_to_3rd_party));
	intm_total_user_ride_time = total_user_ride_time;
}

void return_intm_solution(){

	//<<"hier1"<<endl;
	//<<"hier2 "<<intm_action_passengers.size()<<" "<<action_passengers.size()<<endl;
	/*for(int v=0;v<=total_number_vehicles;v++) {
		cout<<"v "<<v<<": "<<endl;
		cout<<"stps "<<intm_number_stops[v]<<": "<<endl;
		for (int l=0; l<=intm_number_stops[v];l++) {
			cout<<"nps "<<intm_number_passengers_action[v][l]<<": "<<endl;
			for (int m=0; m<intm_number_passengers_action[v][l];m++) 
				cout<<intm_action_passengers[v][l][m]<<" ";

			}
		cout<<endl;
	}*/
	std::copy(std::begin(intm_passengers_departure_time_from_home), std::end(intm_passengers_departure_time_from_home), std::begin(passengers_departure_time_from_home));
	stops = intm_stops;
	//<<"hier4"<<endl;
	//<<"hier5"<<endl;
	std::copy(std::begin(intm_action_passengers), std::end(intm_action_passengers), std::begin(action_passengers));
	//<<"hier4"<<endl;
	//<<"hier6"<<endl;
	number_passengers_action = intm_number_passengers_action;
	//<<"hier5"<<endl;
	arrival_time_stop = intm_arrival_time_stop;
	departure_time_stop = intm_departure_time_stop;
	slack_time = intm_slack_time;
	//<<"hier6"<<endl;
	free_capacity = intm_free_capacity;
	std::copy(std::begin(intm_user_ride_time), std::end(intm_user_ride_time), std::begin(user_ride_time));
	std::copy(std::begin(intm_number_stops), std::end(intm_number_stops), std::begin(number_stops));
	std::copy(std::begin(intm_vehicle_assigned), std::end(intm_vehicle_assigned), std::begin(vehicle_assigned));
	served_passengers = intm_served_passengers;
	served_passengers_3party = intm_served_passengers_3party;
	total_served_passengers = intm_total_served_passengers;
	std::copy(std::begin(intm_assigned_to_3rd_party), std::end(intm_assigned_to_3rd_party), std::begin(assigned_to_3rd_party));
	total_user_ride_time = intm_total_user_ride_time;
}

void update_departure_time_empty_vehicles(){

	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v].size() == 2) {
			departure_time_stop[v][0] = current_time;
		}
	}
}

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


		for (int i=0;i<stop1;i++){
			//cout<<i<<endl;
			for (int j=i+1;j<stop1;j++){
				for(int k=0;k<stop1;k++){
					if (travel_time[i][j] > travel_time[i][k] + travel_time[k][j]){
						travel_time[i][j] = travel_time[i][k] + travel_time[k][j];
						travel_time[j][i] = travel_time[i][j];
						cout<<"err "<<i<<" "<<j<<" "<<k<<endl;
					}
				}
			}
		}
		//saveData("travel_time_matrix.csv", travel_time);
		//cout<<"out"<<endl;

	}
}

void input_requests(char *filename) {

	fstream file (filename, ios::in);
	string line, data, stop;
	int p, s;

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
			stops_origin[p][s] = stoi(data);
			//printf("%d ", stops_origin[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					stops_origin[p][s] = stoi(stop);
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
			stops_destination[p][s] = stoi(data);
			//printf("%d ", stops_destination[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					stops_destination[p][s] = stoi(stop);
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

void serve_passenger_third_party_vehicle(int p){

	//int tp_fare, shortest_distance, shortest_travel_time;
	//none of the available vehicles were able to serve the request on time according to the constraints
	//assigning it to a third party so it can serve the passenger directly and probably faster
	/*shortest_travel_time = INT_MAX;
	for (int i=0;i<number_stops_origin[p];i++) {
		for (int j=0;j<number_stops_origin[p];j++) {

			if (travel_time[stops_origin[p][i]][stops_destination[p][j]] < shortest_travel_time)
				shortest_travel_time = travel_time[stops_origin[p][i]][stops_destination[p][j]];
			
		}
	}*/

	//updates solution cost
	user_ride_time[p] = direct_travel_time[p]*4;
	total_user_ride_time += user_ride_time[p];
	assigned_to_3rd_party[p] = 1;

	/*shortest_distance = shortest_travel_time*5.56;
	
	shortest_distance = shortest_travel_time/1000; //meters to km
	shortest_travel_time = shortest_travel_time/60; //from second to minutes

	tp_fare = ((tp_base_fare + tp_per_minute_charge*shortest_travel_time + tp_km_charge*shortest_distance) * tp_surge_multiplier) + tp_booking_fee; //updates solution cost

	if (tp_fare < tp_minimum_fare)
		tp_fare = tp_minimum_fare;
	solution_cost += tp_fare;
	cost_trip[p] = tp_fare;*/

	served_passengers_3party++;
	//considering that the average to arrive at selected location is around 5 minutes, compute possible arrival time
}

void cheapest_origin2(int p, int v, int &min_increase_length, int &sel_origin, int &pos_origin, bool &repeated_station, bool &flexibilize_lat_departure_time){

	int remove_edge;
	int s_origin, increase;
	int pick_up_time;
	int feasible_insertion = false;
	int new_arrival_time, new_departure_time, new_slack_time, new_arrival_nxt_stop, old_arrival_nxt_stop, old_slack_time;
	int curr_dpt_time, latest_departure_passenger;
	int new_capacity;
	int departure_time_from_home;

	//<<"vx "<<v<<endl;
	//look for cheapest insertion of origin
	for (int j=0;j<number_stops_origin[p];j++) {
		s_origin = stops_origin[p][j];
		for (int i=0; i<number_stops[v];i++) {

			//means that a detour can only be made if the bus has not departure yet from the stop
			//only future actions can be modified
			if (departure_time_stop[v][i] > 0)
				curr_dpt_time = departure_time_stop[v][i];
			else {
				curr_dpt_time = earliest_departure[p]-travel_time[stops[v][i]][s_origin];
				if (curr_dpt_time < current_time){
					curr_dpt_time = current_time;
				}
			}
			//<<curr_dpt_time<<" "<<current_time<<endl;
			if (curr_dpt_time >= current_time) {

				//<<"i:"<<i<<endl;
				increase = travel_time[stops[v][i]][s_origin] + travel_time[s_origin][stops[v][i+1]] - travel_time[stops[v][i]][stops[v][i+1]];
				
				//<<increase<<" "<<slack_time[v][i+1]<<endl;
				//it will arrive at the stop to pick up the passenger at this time
				pick_up_time = curr_dpt_time+travel_time[stops[v][i]][s_origin];

				
				if (s_origin == stops[v][i]){
					//<<"heeere1"<<endl;
					//arrival time does not change from the stop
					
					new_capacity = free_capacity[v][i]-1;
					if (earliest_departure[p] > departure_time_stop[v][i]) 
						new_departure_time = earliest_departure[p];
					else
						new_departure_time = departure_time_stop[v][i];

					old_arrival_nxt_stop = arrival_time_stop[v][i+1];
					new_arrival_nxt_stop = new_departure_time+travel_time[s_origin][stops[v][i+1]];
					old_slack_time = slack_time[v][i+1];

				} else {
					if (s_origin == stops[v][i+1]) {
						//<<"heeere2"<<endl;
						//arrival time does not change from the stop
						new_capacity = free_capacity[v][i+1]-1;
						if (earliest_departure[p] > departure_time_stop[v][i+1]) 
							new_departure_time = earliest_departure[p];
						else
							new_departure_time = departure_time_stop[v][i+1];

						old_arrival_nxt_stop = arrival_time_stop[v][i+2];
						new_arrival_nxt_stop = new_departure_time+travel_time[s_origin][stops[v][i+2]];
						old_slack_time = slack_time[v][i+2];

					} else {

						new_arrival_time = pick_up_time;
						new_capacity = free_capacity[v][i]-1;
						if (earliest_departure[p] > new_arrival_time) 
							new_departure_time = earliest_departure[p];
						else
							new_departure_time = new_arrival_time;
	 
	 					//<<"size: "<<arrival_time_stop[v].size()<<endl<<"i+1:"<<i+1<<endl;
	 					//<<v<<" "<<i+1<<endl;
	 					
						old_arrival_nxt_stop = arrival_time_stop[v][i+1];
						new_arrival_nxt_stop = new_departure_time+travel_time[s_origin][stops[v][i+1]];
						old_slack_time = slack_time[v][i+1];
					}
				}

				//at what time the passenger will have to leave their home to pick up the bus on time?
				departure_time_from_home = new_departure_time-walking_time_stops_origin[p][j];
				//if the departure time from home is greater than or equal to current time, it means it is still a future event, which means it is feasible to happen

				if (old_arrival_nxt_stop == 86400)
					old_arrival_nxt_stop = 0;
				new_slack_time = old_slack_time - (new_arrival_nxt_stop - old_arrival_nxt_stop);
				
			 	//<<old_slack_time<<endl<<new_arrival_nxt_stop<<endl<<old_arrival_nxt_stop<<endl;
			 	//<<"herexxx"<<endl<<endl;


			 	if (flexibilize_lat_departure_time) {
			 		latest_departure_passenger = pick_up_time;
			 		//delay[p] += pick_up_time - latest_departure[p];
			 	} else {
			 		latest_departure_passenger = latest_departure[p];
			 	}
			 	//<<"ptl: "<<pick_up_time<<" "<<latest_departure_passenger<<endl;
				//if ((increase < min_increase_length) && (new_slack_time >= 0) && (pick_up_time <= latest_departure_passenger)) {
			 	//<<"capacity: "<<new_capacity<<endl;
			 	
			 	//<<"vx2 "<<v<<endl;
			 	if ((new_slack_time >= 0) && (pick_up_time <= latest_departure_passenger) && (new_capacity >= 0) && (departure_time_from_home >= current_time)) {


			 		insertions[curr_number_insertions].passengers_departure_time_from_home = departure_time_from_home;
					insertions[curr_number_insertions].increase_length = increase;
					//min_increase_length = increase;
					//remove_edge = i;
					insertions[curr_number_insertions].pos_station = i+1;
					//sel_origin = s_origin;
					insertions[curr_number_insertions].sel_station = s_origin;

					insertions[curr_number_insertions].v = v;

					//repeated_station = false;
					insertions[curr_number_insertions].repeated_station = false;

					//check if included station is equal to one of the two stations
					if (s_origin == stops[v][i]){
						insertions[curr_number_insertions].repeated_station = true;
						insertions[curr_number_insertions].pos_station = i;
					}

					//check if included station is equal to one of the two stations
					if (s_origin == stops[v][i+1]) {
						insertions[curr_number_insertions].repeated_station = true;
						insertions[curr_number_insertions].pos_station = i+1;
					}

					curr_number_insertions++;
					//<<curr_number_insertions<<" ";

					//<<s_origin<<" "<<stops[v][i]<<" "<<stops[v][i+1]<<" "<<new_arrival_nxt_stop<<" "<<old_arrival_nxt_stop<<" "<<new_slack_time<<endl;

					//<<"app: true"<<endl;
					//<<"repeated: "<<repeated_station<<endl;
				}
			}
		}
	}


	//if (not repeated_station)
	///	pos_origin = remove_edge+1;
	//else 
	//	pos_origin = remove_edge;

	//<<"pos origin:"<<pos_origin<<" "<<stops[v][pos_origin]<<endl;
}

void cheapest_origin(int p, int v, int &min_increase_length, int &sel_origin, int &pos_origin, bool &repeated_station, bool &flexibilize_lat_departure_time, int &best_departure_time_from_home, bool &empty_running_vehicle){

	int remove_edge;
	int s_origin, increase;
	int pick_up_time;
	int feasible_insertion = false;
	int new_arrival_time, new_departure_time, new_slack_time, new_arrival_nxt_stop, old_arrival_nxt_stop, old_slack_time;
	int curr_dpt_time, latest_departure_passenger;
	int new_capacity;
	int departure_time_from_home;

	//look for cheapest insertion of origin
	int bi = 0;
	if (empty_running_vehicle){
		bi = number_stops[v] - 1;
	}
	for (int j=0;j<number_stops_origin[p];j++) {
		s_origin = stops_origin[p][j];
		
		for (int i=bi; i<number_stops[v];i++) {
			/*if (empty_running_vehicle){
				cout<<i<<" "<<i<<endl;
			}*/
			//means that a detour can only be made if the bus has not departure yet from the stop
			//only future actions can be modified
			if (departure_time_stop[v][i] > 0)
				curr_dpt_time = departure_time_stop[v][i];
			else {
				curr_dpt_time = earliest_departure[p]-travel_time[stops[v][i]][s_origin];
				if (curr_dpt_time < current_time){
					curr_dpt_time = current_time;
				}
			}
			//<<curr_dpt_time<<" "<<current_time<<endl;
			if (curr_dpt_time >= current_time) {

				//<<"i:"<<i<<endl;
				increase = travel_time[stops[v][i]][s_origin] + travel_time[s_origin][stops[v][i+1]] - travel_time[stops[v][i]][stops[v][i+1]];
				
				//<<increase<<" "<<slack_time[v][i+1]<<endl;
				pick_up_time = curr_dpt_time+travel_time[stops[v][i]][s_origin];

				if (s_origin == stops[v][i]){
					//<<"heeere1"<<endl;
					//arrival time does not change from the stop

					new_capacity = free_capacity[v][i]-1;
					if (earliest_departure[p] > departure_time_stop[v][i]) 
						new_departure_time = earliest_departure[p];
					else
						new_departure_time = departure_time_stop[v][i];

					old_arrival_nxt_stop = arrival_time_stop[v][i+1];
					new_arrival_nxt_stop = new_departure_time+travel_time[s_origin][stops[v][i+1]];
					old_slack_time = slack_time[v][i+1];

				} else {
					if (s_origin == stops[v][i+1]) {
						//<<"heeere2"<<endl;
						//arrival time does not change from the stop
						new_capacity = free_capacity[v][i+1]-1;
						if (earliest_departure[p] > departure_time_stop[v][i+1]) 
							new_departure_time = earliest_departure[p];
						else
							new_departure_time = departure_time_stop[v][i+1];

						old_arrival_nxt_stop = arrival_time_stop[v][i+2];
						new_arrival_nxt_stop = new_departure_time+travel_time[s_origin][stops[v][i+2]];
						old_slack_time = slack_time[v][i+2];

					} else {

						new_arrival_time = pick_up_time;
						new_capacity = free_capacity[v][i]-1;
						if (earliest_departure[p] > new_arrival_time) 
							new_departure_time = earliest_departure[p];
						else
							new_departure_time = new_arrival_time;
	 
	 					//<<"size: "<<arrival_time_stop[v].size()<<endl<<"i+1:"<<i+1<<endl;
	 					//<<v<<" "<<i+1<<endl;
	 					
						old_arrival_nxt_stop = arrival_time_stop[v][i+1];
						new_arrival_nxt_stop = new_departure_time+travel_time[s_origin][stops[v][i+1]];
						old_slack_time = slack_time[v][i+1];
					}
				}

				if (old_arrival_nxt_stop == 86400)
					old_arrival_nxt_stop = 0;
				new_slack_time = old_slack_time - (new_arrival_nxt_stop - old_arrival_nxt_stop);
				
			 	//<<old_slack_time<<endl<<new_arrival_nxt_stop<<endl<<old_arrival_nxt_stop<<endl;
			 	//<<"herexxx"<<endl<<endl;

			 	//at what time the passenger will have to leave their home to pick up the bus on time?
				departure_time_from_home = new_departure_time-walking_time_stops_origin[p][j];
				//if the departure time from home is greater than or equal to current time, it means it is still a future event, which means it is feasible to happen


			 	if (flexibilize_lat_departure_time) {
			 		latest_departure_passenger = pick_up_time;
			 		//delay[p] += pick_up_time - latest_departure[p];
			 	} else {
			 		latest_departure_passenger = latest_departure[p];
			 	}
			 	//<<"ptl: "<<pick_up_time<<" "<<latest_departure_passenger<<endl;
			 	//<<"capacity: "<<new_capacity<<endl;
				if ((increase < min_increase_length) && (new_slack_time >= 0) && (pick_up_time <= latest_departure_passenger) && (new_capacity >= 0) && (departure_time_from_home >= current_time)) {

					//passengers_departure_time_from_home[p] = departure_time_from_home;
					best_departure_time_from_home = departure_time_from_home;
					min_increase_length = increase;
					remove_edge = i;
					sel_origin = s_origin;

					repeated_station = false;

					//check if included station is equal to one of the two stations
					if (s_origin == stops[v][i]){
						repeated_station = true;
						remove_edge = i;
					}

					//check if included station is equal to one of the two stations
					if (s_origin == stops[v][i+1]) {
						repeated_station = true;
						remove_edge = i+1;
					}

					//<<s_origin<<" "<<stops[v][i]<<" "<<stops[v][i+1]<<" "<<new_arrival_nxt_stop<<" "<<old_arrival_nxt_stop<<" "<<new_slack_time<<endl;

					//<<"app: true"<<endl;
					//<<"repeated: "<<repeated_station<<endl;
				}
			}
		}
	}

	if (not repeated_station)
		pos_origin = remove_edge+1;
	else 
		pos_origin = remove_edge;

	//<<"pos origin:"<<pos_origin<<" "<<stops[v][pos_origin]<<endl;
}

void cheapest_destination(int p, int v, int pos_origin, int &min_increase_length, int &sel_destination, int &pos_destination, bool &repeated_station, bool &flexibilize_arrival_time) {

	//put here a check for the origin first then the loop (because of feasibility checks it may not be wise to insert the origin before checking for destination feasibility)
	int s_destination, increase;	
	int remove_edge;
	int drop_off_time;
	int new_arrival_time, new_departure_time, new_slack_time, new_arrival_nxt_stop, old_arrival_nxt_stop, old_slack_time;
	int latest_arrival_passenger, delay_trip;
	int new_capacity;

	//look for cheapest insertion of destination
	for (int j=0;j<number_stops_destination[p];j++) {
		s_destination = stops_destination[p][j];
		for (int i=pos_origin; i<number_stops[v];i++) {
			increase = travel_time[stops[v][i]][s_destination] + travel_time[s_destination][stops[v][i+1]] - travel_time[stops[v][i]][stops[v][i+1]];
			//<<s_destination<<" "<<stops[v][i]<<" "<<departure_time_stop[v][i]<<" "<<travel_time[stops[v][i]][s_destination]<<endl;
			drop_off_time = departure_time_stop[v][i]+travel_time[stops[v][i]][s_destination];
			
			if (s_destination == stops[v][i]){
				//arrival time does not change from the stop
				
				//if (earliest_departure[p] > departure_time_stop[v][i]) 
				//	new_departure_time = earliest_departure[p];
				//else
				//new_departure_time = departure_time_stop[v][i];
				new_capacity = free_capacity[v][i]-1;
				old_arrival_nxt_stop = arrival_time_stop[v][i+1];
				new_arrival_nxt_stop = departure_time_stop[v][i]+travel_time[s_destination][stops[v][i+1]];
				old_slack_time = slack_time[v][i+1];

			} else {
				if (s_destination == stops[v][i+1]) {
					//arrival time does not change from the stop
					
					//if (earliest_departure[p] > departure_time_stop[v][i+1]) 
					//	new_departure_time = earliest_departure[p];
					//else
					//	new_departure_time = departure_time_stop[v][i+1];
					new_capacity = free_capacity[v][i+1]-1;
					old_arrival_nxt_stop = arrival_time_stop[v][i+2];
					new_arrival_nxt_stop = departure_time_stop[v][i+1]+travel_time[s_destination][stops[v][i+2]];
					old_slack_time = slack_time[v][i+2];

				} else {

					new_capacity = free_capacity[v][i]-1;
					new_arrival_time = drop_off_time;
					new_departure_time = new_arrival_time;

					old_arrival_nxt_stop = arrival_time_stop[v][i+1];
					
					new_arrival_nxt_stop = new_departure_time+travel_time[s_destination][stops[v][i+1]];
					old_slack_time = slack_time[v][i+1];
				}
			}

			if (old_arrival_nxt_stop == 86400)
				old_arrival_nxt_stop = 0;
			new_slack_time = old_slack_time - (new_arrival_nxt_stop - old_arrival_nxt_stop);
			//<<old_slack_time<<endl<<new_arrival_nxt_stop<<endl<<old_arrival_nxt_stop<<endl<<endl;

			if (flexibilize_arrival_time) {
				latest_arrival_passenger = drop_off_time;
				delay_trip = drop_off_time - latest_arrival[p];
				//<<"delay: "<<delay[p]<<endl;
			} else {
				latest_arrival_passenger = latest_arrival[p];
				delay_trip = 0;
			}
			//cout<<"dpt: "<<drop_off_time<<" "<<latest_arrival_passenger<<endl;
			//cout<<new_slack_time<<endl;
			if ((increase < min_increase_length) && (new_slack_time >= 0) && (drop_off_time <= latest_arrival_passenger) && (new_capacity >= 0)) {
				min_increase_length = increase;
				remove_edge = i;
				sel_destination = s_destination;

				repeated_station = false;
				//check if included station is equal to one of the two stations
				if (s_destination == stops[v][i]){
					repeated_station = true;
					remove_edge = i;
				}

				//check if included station is equal to one of the two stations
				if (s_destination == stops[v][i+1]) {
					repeated_station = true;
					remove_edge = i+1;
				}

				delay[p] = delay_trip;
			}
		}
	}

	if (not repeated_station)
		pos_destination = remove_edge+1;
	else
		pos_destination = remove_edge;
}

void filter_vehicles(int p){

	int v;
	bool leave_loop = false;
	int arrival_time_at_stop;
	for (int i = 0; i < total_number_vehicles; i++) {
		v = i;
		if (free_capacity[v].size()>2){//means that vehicle is not empty
			
			leave_loop = false;
			for (int j=0; j<=number_stops[v];j++) {
				for (int k=0;k<number_stops_origin[p];k++) {
					arrival_time_at_stop = departure_time_stop[v][j]+travel_time[stops[v][j]][stops_origin[p][k]];
					if (arrival_time_at_stop <= latest_arrival[p]) {
						//vehicle is feasible
						filtered_vehicles.push_back(v);
						leave_loop = true;
						break;
					}

				}
				if (leave_loop)
					break; 

			}
		}
	}
}

void select_vehicles_havent_left_depot(){

	if (vehicles_still_depot.size() > 0)
		vehicles_still_depot.clear();

	int v;
	for ( v = 0; v < total_number_vehicles; v++) {
		if (departure_time_stop[v][0] > current_time){ //this means the vehicle hasnt left the depot yet
			vehicles_still_depot.push_back(v);
		}
	}

	//getting which passengers are at the vehicle
	for (int i=0; i<vehicles_still_depot.size();i++){
		v= vehicles_still_depot[i];
		
		if (passengers_at_vehicle[v].size() > 0)
			passengers_at_vehicle[v].clear();

		for (int j=0; j<number_stops[v]; j++){
			for (int k=0; k<number_passengers_action[v][j];k++) {
				passengers_at_vehicle[v].push_back(action_passengers[v][j][k]);
			}
		}

		//erasing the duplicates
		passengers_at_vehicle[v].erase(unique( passengers_at_vehicle[v].begin(), passengers_at_vehicle[v].end() ), passengers_at_vehicle[v].end());

	}
}

void select_vehicles_havent_that_can_be_turned_empty(){

	//<<"SELECT EMPTYING VEHICLES"<<endl;
	if (vehicles_still_depot.size() > 0)
		vehicles_still_depot.clear();
	//<<"hier1"<<endl;
	int v;
	for ( v = 0; v < total_number_vehicles; v++) {

		if (passengers_at_vehicle[v].size() > 0)
			passengers_at_vehicle[v].clear();

		if (free_capacity[v].size() > 2) {
			for (int i=0; i<number_stops[v];i++) {
				if (departure_time_stop[v][i] > current_time){ //this means the vehicle hasnt left the stop yet
					
					//<<"hier2"<<endl;
					int counter = 0;
					for (int j=i;j<=number_stops[v];j++) {
						for (int k=0; k<number_passengers_action[v][j];k++) {
							int p = action_passengers[v][j][k];
							for (int l=j+1;l<=number_stops[v];l++) {
								for (int m=0; m<number_passengers_action[v][l];m++) {
									if (action_passengers[v][l][m] == p) {
										counter++;
										passengers_at_vehicle[v].push_back(p);
									}
								}
							}
						}
					}
					//<<"hier3"<<endl;

					double number_to_be_empty2 = (double)(number_stops[v]-i)/(double)2;
					int number_to_be_empty = ceil(number_to_be_empty2);
					//<<v<<" "<<number_to_be_empty<<" "<<counter<<endl;

					if (number_to_be_empty == counter)
						vehicles_still_depot.push_back(v);

					//erasing the duplicates
					//passengers_at_vehicle[v].erase(unique( passengers_at_vehicle[v].begin(), passengers_at_vehicle[v].end() ), passengers_at_vehicle[v].end());

					break;
				}
			}
		}
	}

	//getting which passengers are at the vehicle
	/*for (int i=0; i<vehicles_still_depot.size();i++){
		v= vehicles_still_depot[i];
		
		if (passengers_at_vehicle[v].size() > 0)
			passengers_at_vehicle[v].clear();

		for (int j=0; j<number_stops[v]; j++){
			for (int k=0; k<number_passengers_action[v][j];k++) {
				passengers_at_vehicle[v].push_back(action_passengers[v][j][k]);
			}
		}

		//erasing the duplicates
		passengers_at_vehicle[v].erase(unique( passengers_at_vehicle[v].begin(), passengers_at_vehicle[v].end() ), passengers_at_vehicle[v].end());

	}*/
}

void remove_passenger_from_vehicle(int v, int p) {

	/*cout<<"passenger removed "<<p<<endl;
	for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}
	cout<<endl<<endl;*/

	int greatest_ed = INT_MIN;
	int count = 0;
	int begin, end, old_arrival_time;

	
	//printf("remove heeeere func\n");
	//<<"number stops "<<number_stops[v]<<endl;
	//it means the passenger was relocated to another trip 
	for (int i=0; i<number_stops[v]; i++){
		for (int j=0; j<number_passengers_action[v][i];j++) {
			//<<"actp "<<action_passengers[v][i][j]<<endl;
			if (action_passengers[v][i][j] == p) {
				count++;
			
				//<<"hiierx"<<endl;
				//removes previous origin stuff
				if (number_passengers_action[v][i] == 1) {


					//<<"hiierx1 "<<i<<endl;
					number_stops[v]--;
					stops[v].erase(stops[v].begin() + i);
					action_passengers[v].erase(action_passengers[v].begin() + i);
					number_passengers_action[v].erase(number_passengers_action[v].begin() + i);
					//<<"hiierx2"<<endl;
					arrival_time_stop[v].erase(arrival_time_stop[v].begin() + i);
					departure_time_stop[v].erase(departure_time_stop[v].begin() + i);
					slack_time[v].erase(slack_time[v].begin() + i);
					free_capacity[v].erase(free_capacity[v].begin() + i);

					//<<"hiierx3"<<endl;
					//re-update further arrival and departure times
					for (int k=i; k<=number_stops[v];k++) {
						old_arrival_time = arrival_time_stop[v][k];
						arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
						departure_time_stop[v][k] = arrival_time_stop[v][k];

						greatest_ed = INT_MIN;
						//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
						for (int l=0; l<number_passengers_action[v][k];l++) {
							if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[v][k][l]];
						}

						if (greatest_ed > departure_time_stop[v][k])
							departure_time_stop[v][k] = greatest_ed;

						//gives "extra" slack
						//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];
					}
					

				} else {

					//<<"hiierx3"<<endl;
					action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
					number_passengers_action[v][i]--;

					//arrival_time_stop[v][i] = //shouldnt change

					greatest_ed = INT_MIN;
					for (int l=0; l<number_passengers_action[v][i];l++) {
						if (earliest_departure[action_passengers[v][i][l]] > greatest_ed)
							greatest_ed = earliest_departure[action_passengers[v][i][l]];
					}

					//<<"hiierx4"<<endl;
					//departure time only changes if the greatest earliest departure < departure time
					//it means that by removing the passenger the bus can leave earlier
					if (greatest_ed < departure_time_stop[v][i]) {
						if (arrival_time_stop[v][i] > greatest_ed){
							departure_time_stop[v][i] = arrival_time_stop[v][i];
						} else {
							departure_time_stop[v][i] = greatest_ed;
						}
					}
					

					//free_capacity[v][i]++;

					//re-update further arrival and departure times
					for (int k=i+1; k<=number_stops[v];k++) {
						
						old_arrival_time = arrival_time_stop[v][k];
						arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
						departure_time_stop[v][k] = arrival_time_stop[v][k];

						greatest_ed = INT_MIN;
						//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
						for (int l=0; l<number_passengers_action[v][k];l++) {
							if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[v][k][l]];
						}

						if (greatest_ed > departure_time_stop[v][k])
							departure_time_stop[v][k] = greatest_ed;

						//gives "extra" slack
						//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];

					}

				}

				/*int current_user_ride_time, difference;
				bool leave_loop = false;
				for (int i=0; i<=number_stops[v];i++) {
					for (int j=0; j<number_passengers_action[v][i];j++) {
						int save_p = action_passengers[v][i][j];
						for (int k=i+1; k<=number_stops[v];k++) {
							leave_loop = false;
							for (int l=0; l<number_passengers_action[v][k];l++) {
								if (action_passengers[v][k][l] == save_p){
									current_user_ride_time = arrival_time_stop[v][k] - departure_time_stop[v][i];
									if (current_user_ride_time != user_ride_time[save_p]) {
										difference = current_user_ride_time -  user_ride_time[save_p];
										//<<"difference: "<<difference<<endl;
										total_user_ride_time += difference;
										//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
										user_ride_time[save_p] = current_user_ride_time;
										l = number_passengers_action[v][k]+1;
										//k = number_stops[best_v]+2; //leave loop
										leave_loop = true;
									}
								}
							}
							if (leave_loop)
								k = number_stops[v]+2;
						}	
					} 
				}*/

				
				if (count == 1) {
					begin = i;
					//free_capacity[v][i]++;
					i--;
				} else {
					end = i;
				}
				
				break;
			} 		
		}

		if (count == 2)
			break;
	}

	for (int l=begin;l<end;l++) {
		free_capacity[v][l]++;
	}

	if (count == 2) {

		if (free_capacity[v].size()==2){
			//means the vehicle is empty at depot
			departure_time_stop[v][0] = current_time;
		}

		int min_slack_time_so_far, tightest_arrival_time, possible_new_slack_time;
		min_slack_time_so_far = INT_MAX;
		tightest_arrival_time = INT_MAX;
		//updates the slack times
		for (int k = number_stops[v]; k>=1; k--){

			tightest_arrival_time = INT_MAX;
			for (int l = 0; l < number_passengers_action[v][k]; l++) {
				if (latest_arrival[action_passengers[v][k][l]] < tightest_arrival_time)
					tightest_arrival_time = latest_arrival[action_passengers[v][k][l]];
			}

			if (tightest_arrival_time != INT_MAX)
				slack_time[v][k] = tightest_arrival_time - arrival_time_stop[v][k];
			else
				slack_time[v][k] = 86400;
			if (slack_time[v][k] < min_slack_time_so_far) {
				min_slack_time_so_far = slack_time[v][k];
			} else {
				slack_time[v][k] = min_slack_time_so_far;
			}

		}

		//break;
	}
	

	/*for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}*/
}

void try_to_insert_in_already_running_vehicle(int p){
	//function to compare the insertion of insertion of a passenger in an already running vehicle when compared to inserting in an empty vehicle

	int v;
	int pos_origin, pos_destination;
	int sel_origin, sel_destination;
	int min_increase_length;
	bool repeated_station, no_violation_capacity;
	int old_dpt_time, old_arr_time;
	int prv_capacity;
	bool flexibilize_arrival_time, flexibilize_lat_departure_time, accept_delay_trip;
	int best_empty_vehicle, capacity_best_empty_vehicle;
	int best_distance_depot;
	best_empty_vehicle = -1;
	capacity_best_empty_vehicle = INT_MIN;
	best_distance_depot = INT_MAX;
	int best_depot;
	int best_departure_time_from_home;
	int veh;
	int ttcsd = INT_MAX;
	int selected_insertion, next_replace, remaining_insertions;
	int increase_total_user_ride_time = 0;

	if (filtered_vehicles.size()>0)
		filtered_vehicles.clear();
	filter_vehicles(p);

	int fsize = filtered_vehicles.size();
	for (int i=0;i<fsize;i++){
		if (vehicle_assigned[p] == filtered_vehicles[i]) {
			filtered_vehicles.erase(filtered_vehicles.begin() + i);
			break;
		}
	}
	bool not_so_big_increase_found = false;
	//<<"TRY TO INSERT RUNNING VEHICLE"<<endl;
	//<<"passengeryy "<<p<<endl;
	//<<"filtered_vehicles SIZE"<<filtered_vehicles.size()<<endl;
	bool not_feasible_insertion = true;
	int iterations = 0;
	bool tested_all_vehicles_once = false;

	int initially_assigned_vehicle = vehicle_assigned[p];
	/*if (filtered_vehicles.size() == 0){
		serve_passenger_third_party_vehicle(p);
	}*/
	
	while ((not_feasible_insertion) && (iterations < filtered_vehicles.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		curr_number_insertions = 0;
		//<<"heeerexx";
		for (int vf=0; vf<filtered_vehicles.size();vf++) {
			int v = filtered_vehicles[vf];
			
			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			if (blocked_vehicles[v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				
				/*cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);

				if (min_increase_length < best_min_increase_length) {
					best_min_increase_length = min_increase_length;
					best_sel_origin = sel_origin;
					best_pos_origin = pos_origin;
					best_v = v;
					best_repeated_station = repeated_station;
				}*/
				//<<"vehiclex: "<<v<<endl;
				cheapest_origin2(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);
				
			}
		}
		//<<endl;

		/*cout<<"curr insert4: " << curr_number_insertions<<endl;
		for (int i=0;i<curr_number_insertions;i++){
			cout<<"vy: "<<insertions[i].v<<endl;
		}*/
		sort(insertions, insertions+curr_number_insertions, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions > 0) {
		
			//int rclsize = 5;
			remaining_insertions = curr_number_insertions;
			//if (rclsize < remaining_insertions)
			//	next_replace = rclsize;
			//else
			next_replace = remaining_insertions-1;

			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions)) {

				//<<"hiiier"<<endl;
				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				//if (rclsize <= remaining_insertions)
				//	selected_insertion = rand() % rclsize;
				//else
				selected_insertion = rand() % remaining_insertions;

				//<<"selec inser"<<selected_insertion<<endl;
				
				best_min_increase_length = insertions[selected_insertion].increase_length;
				best_sel_origin = insertions[selected_insertion].sel_station;
				best_pos_origin = insertions[selected_insertion].pos_station;
				best_v = insertions[selected_insertion].v;
				//<<"vehicley: "<<best_v<<endl;
				best_repeated_station = insertions[selected_insertion].repeated_station;
				remaining_insertions--; //updates the number of remaining feasible insertions

				//bring the next insertion to the top 3
				insertions[selected_insertion].increase_length = insertions[next_replace].increase_length;
				insertions[selected_insertion].sel_station = insertions[next_replace].sel_station;
				insertions[selected_insertion].pos_station = insertions[next_replace].pos_station;
				insertions[selected_insertion].v = insertions[next_replace].v;
				insertions[selected_insertion].repeated_station = insertions[next_replace].repeated_station;
				next_replace--;

				
				if (not best_repeated_station) {
					//<<"heere3"<<endl;

					//<<best_sel_origin<<" "<<best_min_increase_length<<endl;

					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

					//updating arrival and departure from stops
					
					//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
					
					arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
					
					if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
					else
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
					slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
					
					prv_capacity = free_capacity[best_v][best_pos_origin-1];
					free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);
					
				} else {
					//<<"heere4"<<endl;
					//update passenger performing actions on the stops
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v][best_pos_origin]++;

					prv_arr_time_at_origin = arrival_time_stop[best_v][best_pos_origin];
					prv_dpt_time_at_origin = departure_time_stop[best_v][best_pos_origin];
					//no need to update the arrival time as it does not change
					//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
					if (earliest_departure[p] > departure_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v][best_pos_origin] = earliest_departure[p];
					//slack_time[best_v][best_pos_origin] //no need to update
					prv_capacity = free_capacity[best_v][best_pos_origin];
					free_capacity[best_v][best_pos_origin] = prv_capacity-1;
				}

				//update further arrival, departure times and slack times
				for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
					saved_arrival_time[i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					}
					saved_slack_time[i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				//vehicle_assigned[p] = best_v;

				//if (best_v == 24) {
				/*cout<<"flex: "<<flexibilize_lat_departure_time<<endl;
				for (int i=0; i<=number_stops[best_v];i++) {
					cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
					for (int j=0; j<number_passengers_action[best_v][i];j++) 
						cout<<action_passengers[best_v][i][j]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
					cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
					cout<<"|"<<slack_time[best_v][i]<<"|  ";
					cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
				}*/
				//}
				//<<endl;

				best_min_increase_length = INT_MAX;
				
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				if (tested_all_vehicles_once)
					flexibilize_arrival_time = true;
				else
					flexibilize_arrival_time = false;

				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
			
				//<<"pos origin: "<<best_pos_origin<<" "<<endl;
				//<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
				
				if (sel_destination != -1) { 

					no_violation_capacity = true;
					for (int i=best_pos_origin+1;i<pos_destination;i++){

						free_capacity[best_v][i]--;

						if (free_capacity[best_v][i] < 0) {
							no_violation_capacity = false;
						}
						
					}
				}

				if (delay[p] <= max_flex_delay)
					accept_delay_trip = true;
				else 
					accept_delay_trip = false;


				if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip)) {
					
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;

					passengers_departure_time_from_home[p] = insertions[next_replace].passengers_departure_time_from_home;
					/*if (iterations2 >  0) {
						printf("HEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEERE\n");
						printf("HEEEERE\n");
						printf("HEEEERE\n");
						printf("HEEEERE\n");
						printf("HEEEERE\n");
						printf("HEEEERE\n");
						printf("HEEEERE\n");
						printf("%d it\n", iterations2);
					}*/
					if (not repeated_station) {
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);

						//updating arrival and departure from stops
						int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
						arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
						slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

						prv_capacity = free_capacity[best_v][pos_destination-1];
						free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
					} else {

						//update passenger performing actions on the stops
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v][pos_destination]++;

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						if (slc_time < slack_time[best_v][pos_destination]) 
							slack_time[best_v][pos_destination] = slc_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}


					//update further arrival, departure times, and slack times
					for (int i=pos_destination+1; i<=number_stops[best_v];i++) {
						old_arr_time = arrival_time_stop[best_v][i];
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						old_dpt_time = departure_time_stop[best_v][i];
						if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						}
						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
					}

					//for (int i=1; i<pos_destination;i++){
					//	int min_slack_time = INT_MAX;
					//	for (int j=i+1; i<number_stops[best_v];i++){
					//		if (slack_time[best_v][j] < min_slack_time)
					//			min_slack_time = slack_time[best_v][j];
					//	}
					//	slack_time[best_v][i] = min_slack_time;
					//}

					int min_slack_time_so_far = INT_MAX;	
					for (int i=number_stops[best_v]; i>=1; i--){
						if (slack_time[best_v][i] < min_slack_time_so_far) {
							min_slack_time_so_far = slack_time[best_v][i];
						} else {
							slack_time[best_v][i] = min_slack_time_so_far;
						}
					}
					//updates solution cost
					
					//<<"heere5"<<endl;
					//<<"stations "<<best_pos_origin<<" "<<sel_destination<<endl;
					user_ride_time[p] = 0;
					//total_user_ride_time += user_ride_time[p];
					int prv_user_ride_time = user_ride_time[p];

					//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
					int current_user_ride_time, difference;
					bool leave_loop = false;
					for (int i=0; i<=number_stops[best_v];i++) {
						for (int j=0; j<number_passengers_action[best_v][i];j++) {
							int save_p = action_passengers[best_v][i][j];
							//if (save_p != p){
								for (int k=i+1; k<=number_stops[best_v];k++) {
									leave_loop = false;
									for (int l=0; l<number_passengers_action[best_v][k];l++) {
										if (action_passengers[best_v][k][l] == save_p){
											current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time -  user_ride_time[save_p];
												increase_total_user_ride_time += difference;
												//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
												//user_ride_time[save_p] = current_user_ride_time;
												l = number_passengers_action[best_v][k]+1;
												//k = number_stops[best_v]+2; //leave loop
												leave_loop = true;
											}
										}
									}
									if (leave_loop)
										k = number_stops[best_v]+2;
								}
							//}
						} 
					}

					user_ride_time[p] = prv_user_ride_time;


					if (increase_total_user_ride_time <= user_ride_time[p] + 120){
						//<<"NOT SO BIG INCREASEEEE"<<endl;
						not_so_big_increase_found = true;
						//remove from the empty vehicle
						remove_passenger_from_vehicle(vehicle_assigned[p], p);
						vehicle_assigned[p] = best_v;

					} else {
						remove_passenger_from_vehicle(best_v, p);
						no_feasible_insertion2 = true;
						not_feasible_insertion = true;
						iterations2++;
						vehicle_assigned[p] = initially_assigned_vehicle;
					}



					/*odb_travel_time = travel_time[best_sel_origin][sel_destination];
					odb_distance = odb_travel_time*5.56;
					odb_distance = odb_distance/1000;
					odb_travel_time = odb_travel_time/60;
					odb_fare = ((ODB_base_fare[vehicle_type[best_v]] + ODB_km_charge[vehicle_type[best_v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[best_v]]) + ODB_booking_fee[vehicle_type[best_v]]; 
					if (odb_fare < ODB_minimum_fare[vehicle_type[best_v]])
						odb_fare = ODB_minimum_fare[vehicle_type[best_v]];
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;*/

				} else {

					//no feasible insertion for destination
					//<<"no feasible insertion for destination"<<endl;
					//blocked_vehicles[best_v] = 1; //I suppose this is not necessary anymore. I'm already testing all vehicles with array insertions

					iterations2++;
					//solution_cost -= saved_increase;
					vehicle_assigned[p] = -1;
					if (sel_destination != -1) {
						for (int i=best_pos_origin+1;i<pos_destination;i++){

							free_capacity[best_v][i]++;
		
						}
					}

					if (number_passengers_action[best_v][best_pos_origin] == 1) {
						number_stops[best_v]--;
						stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
						action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
						number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
						

						//re-update further arrival and departure times
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[i];
							departure_time_stop[best_v][i] = saved_departure_time[i];
							slack_time[best_v][i] = saved_slack_time[i];
						}
						arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
						departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
						slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


						free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


					} else {

						//<<"heere"<<endl;
						//<<action_passengers[best_v][best_pos_origin][1]<<endl;
						action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
						
						number_passengers_action[best_v][best_pos_origin]--;

						arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
						departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
						free_capacity[best_v][best_pos_origin]++;

						//re-update further arrival and departure times
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[i];
							departure_time_stop[best_v][i] = saved_departure_time[i];
							slack_time[best_v][i] = saved_slack_time[i];
						}

					}
				}
				
				//<<"heeere"<<endl;

				//if (best_v == 24) {
					//<<flexibilize_lat_departure_time<<endl;
				//<<"xyz1"<<endl;
 				/*for (int i=0; i<=number_stops[best_v];i++) {
					cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
					for (int j=0; j<number_passengers_action[best_v][i];j++) 
						cout<<action_passengers[best_v][i][j]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
					cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
					cout<<"|"<<slack_time[best_v][i]<<"|  ";
					cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
				}*/
				//}	
				//<<endl;
			} //end while
		} else {
			iterations = total_number_vehicles;
		}

		
		if (no_feasible_insertion2) {
			//iterations++;
			iterations = total_number_vehicles;
		}

		/*if (iterations >= total_number_vehicles) {
			if (not tested_all_vehicles_once) {
				tested_all_vehicles_once = true;
				//usually this is restarted flexibilizing the latest departure time, but we dont want that anymore
				//now lets just keep this way
				//iterations = 0;
				cout<<"serving 3rd party";
				serve_passenger_third_party_vehicle(p);
				//for (int i=0;i<total_number_vehicles;i++)
				//	blocked_vehicles[i] = 0;
			} else {
				//ultimately serve passenger with a 3rd party vehicle
				cout<<"serving 3rd party";
				serve_passenger_third_party_vehicle(p);
			}
		}*/
	}

	if (!not_so_big_increase_found) {
		vehicle_assigned[p] = initially_assigned_vehicle;
	}
}

void see_if_arrival_departure_dont_match(int best_v) {

	for (int i = 1; i < number_stops[best_v]; i++) {
		if (arrival_time_stop[best_v][i] != departure_time_stop[best_v][i]) {

			int greatest_ed = INT_MIN;
			//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
			for (int m=0; m<number_passengers_action[best_v][i];m++) {
				if (earliest_departure[action_passengers[best_v][i][m]] > greatest_ed)
					greatest_ed = earliest_departure[action_passengers[best_v][i][m]];
			}

			if (greatest_ed != departure_time_stop[best_v][i]) {
				departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];

				int old_arr_time, old_dpt_time;
				//update further arrival, departure times, and slack times
				for (int k=i+1; k<=number_stops[best_v];k++) {
					old_arr_time = arrival_time_stop[best_v][k];
					arrival_time_stop[best_v][k] = departure_time_stop[best_v][k-1]+travel_time[stops[best_v][k-1]][stops[best_v][k]];
					old_dpt_time = departure_time_stop[best_v][k];
					departure_time_stop[best_v][k] = arrival_time_stop[best_v][k];
					
					int greatest_ed = INT_MIN;
					//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
					for (int l=0; l<number_passengers_action[best_v][k];l++) {
						if (earliest_departure[action_passengers[best_v][k][l]] > greatest_ed)
							greatest_ed = earliest_departure[action_passengers[best_v][k][l]];
					}

					if (greatest_ed > departure_time_stop[best_v][k])
						departure_time_stop[best_v][k] = greatest_ed;

					slack_time[best_v][k] -= arrival_time_stop[best_v][k] - old_arr_time;
				}
				
				/*arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
				departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];

				int greatest_ed = INT_MIN;
				//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
				for (int l=0; l<number_passengers_action[best_v][i];l++) {
					if (earliest_departure[action_passengers[best_v][i][l]] > greatest_ed)
						greatest_ed = earliest_departure[action_passengers[best_v][i][l]];
				}

				if (greatest_ed > departure_time_stop[best_v][i])
					departure_time_stop[best_v][i] = greatest_ed;*/ 
			}

		}
	}
}

//this is a randomized version of cheapest insertion
//inserts the passenger at a randomized position between (0, k)
void cheapest_insertion_randomized(int p){

	int odb_fare, odb_travel_time, odb_distance;
	int v;
	int pos_origin, pos_destination;
	int sel_origin, sel_destination;
	int min_increase_length;
	bool repeated_station, no_violation_capacity;
	int old_dpt_time, old_arr_time;
	int prv_capacity;
	bool flexibilize_arrival_time, flexibilize_lat_departure_time, accept_delay_trip;
	int best_empty_vehicle, capacity_best_empty_vehicle;
	int best_distance_depot;
	best_empty_vehicle = -1;
	capacity_best_empty_vehicle = INT_MIN;
	best_distance_depot = INT_MAX;
	int best_depot;
	int best_departure_time_from_home;
	int veh;
	int ttcsd = INT_MAX;
	
	//check for closest depot that has an empty vehicle
	//second check for highest capacitated vehicle from that depot
	for (int i = 0; i < number_depots; i++) {

		ttcsd = INT_MAX;
		for (int k=0;k<number_stops_origin[p];k++) {
			if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
				ttcsd = travel_time[stops_origin[p][k]][depot[i]];
			}
		}

		//if ((best_distance_depot == 1) || (ttcsd <= best_distance_depot)) {
		if (ttcsd <= best_distance_depot) {
			capacity_best_empty_vehicle = INT_MIN;
			for (int j = 0; j < number_vehicles_at_depot[i]; j++) {

				veh = vehicles_at_depot[i][j];
				if (free_capacity[veh].size() == 2){ //means that vehicle is empty
					if (free_capacity[veh][0] >= capacity_best_empty_vehicle) {
						best_empty_vehicle = veh;
						best_depot = depot[i];
						capacity_best_empty_vehicle = free_capacity[veh][0];
						best_distance_depot = ttcsd;

					}
				}
			}
		}
	}

	//<<"hieerxxempt"<<" "<<best_empty_vehicle<<endl;

	int best_distance_location = best_distance_depot;

	bool empty_running_vehicle = false;
	for (int v=0;v<total_number_vehicles;v++){
		if (free_capacity[v].size() > 2) { //the vehicle has passengers assigned but might be empty at some point in time 
			for (int i=number_stops[v]-1; i<number_stops[v];i++){
				if (free_capacity[v][i] == free_capacity[v][0]) { //means that vehicle is empty

					ttcsd = INT_MAX;
					for (int k=0;k<number_stops_origin[p];k++) {
						if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
							ttcsd = travel_time[stops[v][i]][stops_origin[p][k]];
						}
					}

					if (ttcsd <= best_distance_location) {

						if (departure_time_stop[v][i] + ttcsd <= latest_departure[p]) { //important to verify if is feasible according to time window constraints
							empty_running_vehicle = true;
							//<<"HIERRR"<<endl;
							best_empty_vehicle = v;
							best_distance_location = ttcsd;
						}

					}
				}
			}
		}
	}

	//<<"hieerx2xempt"<<" "<<best_empty_vehicle<<" "<<number_stops[best_empty_vehicle]<<endl;
	bool same_station = false;
	v = best_empty_vehicle;
	int selected_insertion, next_replace, remaining_insertions;
	//inserting in an empty vehicle stays the same as before
	if (v != -1) {

		int prv_arr_time_at_origin, prv_dpt_time_at_origin;
		int old_user_ride_time = total_user_ride_time;

		//<<"herexx";
		if (free_capacity[v].size() == 2) 
			departure_time_stop[v][0] = 0;
		/*for (int i=0; i<=number_stops[v];i++)
			cout<<stops[v][i]<<" ";
		cout<<endl;*/

		//there's at least one empty vehicle, so the passenger will be inserted on it
		min_increase_length = INT_MAX;
		repeated_station = false;
		flexibilize_lat_departure_time = false;
		sel_origin = -1;
		cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home, empty_running_vehicle);
		
		if (sel_origin != -1) {


			if (not repeated_station) {
				same_station = false;
				//<<"sel o"<<sel_origin;
				stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
				
				//update passenger performing actions on the stops
				action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
				action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
				number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_origin, 1);

				//updating arrival and departure from stops
				//arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[depot][sel_origin]);
				//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
				
				int curr_dpt_time = earliest_departure[p]-travel_time[stops[v][pos_origin-1]][sel_origin];
				if (curr_dpt_time < current_time){
					curr_dpt_time = current_time;
				}

				if (pos_origin == 1)
					departure_time_stop[v][0] = curr_dpt_time;
				
				arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[stops[v][pos_origin-1]][sel_origin]);
				departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[stops[v][pos_origin-1]][sel_origin]);
				slack_time[v].insert(slack_time[v].begin() + pos_origin, 86400);

				prv_capacity = free_capacity[v][pos_origin-1];
				free_capacity[v].insert(free_capacity[v].begin() + pos_origin, prv_capacity-1);
				//free_capacity[v]--;

				//solution_cost += min_increase_length; //updates solution cost
				number_stops[v]++;

			} else {

				same_station = true;
				//<<"heere4"<<endl;
				//update passenger performing actions on the stops
				action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
				number_passengers_action[v][pos_origin]++;

				prv_arr_time_at_origin = arrival_time_stop[v][pos_origin];
				prv_dpt_time_at_origin = departure_time_stop[v][pos_origin];
				//no need to update the arrival time as it does not change
				//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
				if (earliest_departure[p] > departure_time_stop[v][pos_origin])
					departure_time_stop[v][pos_origin] = earliest_departure[p];
				//slack_time[best_v][best_pos_origin] //no need to update
				prv_capacity = free_capacity[v][pos_origin];
				free_capacity[v][pos_origin] = prv_capacity-1;



			}



			vehicle_assigned[p] = v;

			/*for (int i=0; i<=number_stops[v];i++) {
				cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" ";
				for (int j=0; j<number_passengers_action[v][i];j++) 
					cout<<action_passengers[v][i][j]<<" ";
				cout<<"] ";
				cout<<"{"<<arrival_time_stop[v][i]<<"} ";
				cout<<"{"<<departure_time_stop[v][i]<<"} ";
				cout<<"|"<<slack_time[v][i]<<"|  ";
				cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
			
			}
			cout<<endl;*/

			min_increase_length = INT_MAX;
			repeated_station = false;
			sel_destination = -1;
			flexibilize_arrival_time = false;

			see_if_arrival_departure_dont_match(v);
			cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
			//<<min_increase_length<<" "<<pos_destination<<endl;
			
			if (sel_destination != -1) {
				
				//cout<<"1hierxx"<<endl;
				passengers_departure_time_from_home[p] = best_departure_time_from_home;
				//<<"pos dest: "<<sel_destination<<endl;
				stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
				
				//update passenger performing actions on the stops
				action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
				action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
				number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


				//updating arrival and departure from stops
				int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
				//<<"xx "<<stops[v][pos_destination-1]<<endl;
				arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
				departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
				slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
				slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

				prv_capacity = free_capacity[v][pos_destination-1];
				free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

				//solution_cost += min_increase_length; //updates solution cost
				//updates user ride time
				//<<"heere1"<<endl;
				user_ride_time[p] = travel_time[sel_origin][sel_destination];
				total_user_ride_time += user_ride_time[p];
				//<<"total_user_ride_time1: "<<total_user_ride_time<<endl;

				/*odb_travel_time = travel_time[sel_origin][sel_destination];
				odb_distance = odb_travel_time*5.56;

				odb_distance = odb_distance/1000;
				odb_travel_time = odb_travel_time/60;
				odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
				if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
					odb_fare = ODB_minimum_fare[vehicle_type[v]];
				solution_cost += odb_fare; //updates solution cost
				cost_trip[p] = odb_fare;*/

				//saved_increase = min_increase_length;
				number_stops[v]++;


				arrival_time_stop[v][pos_destination+1] = departure_time_stop[v][pos_destination]+travel_time[stops[v][pos_destination]][stops[v][pos_destination+1]];	
				departure_time_stop[v][pos_destination+1] = arrival_time_stop[v][pos_destination+1];
			

				int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[v]; i>=1; i--){
							if (slack_time[v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[v][i];
							} else {
								slack_time[v][i] = min_slack_time_so_far;
							}
						}
				//checks for next empty vehicle
				/*while (free_capacity[empty_vehicle].size() > 2) { 
					empty_vehicle++;
					if (empty_vehicle == total_number_vehicles) {
						//there are no empty vehicles
						empty_vehicle = -1;
						break;
					}
				}*/

				
				
			} else {

				//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
				//try to insert giving flexibility according to latest_arrivel_time
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				flexibilize_arrival_time = true;
				see_if_arrival_departure_dont_match(v);	
				cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);

				if (delay[p] <= max_flex_delay)
					accept_delay_trip = true;
				else 
					accept_delay_trip = false;
				if ((sel_destination != -1) && (accept_delay_trip)) {
					
					//cout<<"2hierxx"<<endl;
					passengers_departure_time_from_home[p] = best_departure_time_from_home;
					//<<"pos dest: "<<sel_destination<<endl;
					stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
					
					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
					action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


					//updating arrival and departure from stops
					int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
					//<<"xx "<<stops[v][pos_destination-1]<<endl;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
					slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
					slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

					prv_capacity = free_capacity[v][pos_destination-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

					//updates solution cost
					//<<"heere2"<<endl;
					user_ride_time[p] = travel_time[sel_origin][sel_destination];
					total_user_ride_time += user_ride_time[p];
					//<<"total_user_ride_time: "<<total_user_ride_time<<endl;

					/*odb_travel_time = travel_time[sel_origin][sel_destination];
					odb_distance = odb_travel_time*5.56;
					odb_distance = odb_distance/1000;
					odb_travel_time = odb_travel_time/60;
					odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
					if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
						odb_fare = ODB_minimum_fare[vehicle_type[v]];
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;*/
					number_stops[v]++;

					arrival_time_stop[v][pos_destination+1] = departure_time_stop[v][pos_destination]+travel_time[stops[v][pos_destination]][stops[v][pos_destination+1]];	
					departure_time_stop[v][pos_destination+1] = arrival_time_stop[v][pos_destination+1];

					int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[v]; i>=1; i--){
							if (slack_time[v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[v][i];
							} else {
								slack_time[v][i] = min_slack_time_so_far;
							}
						}

					
					update_URT(v);
					//checks for next empty vehicle
					/*while (free_capacity[empty_vehicle].size() > 2) { 
						empty_vehicle++;
						if (empty_vehicle == total_number_vehicles) {
							//there are no empty vehicles
							empty_vehicle = -1;
							break;
						}
					}*/
				} else {
					//cout<<"3hierxx"<<endl;
					//solution_cost -= saved_increase;
					vehicle_assigned[p] = -1;
					number_stops[v]--;
					stops[v].erase(stops[v].begin() + pos_origin);
					action_passengers[v].erase(action_passengers[v].begin() + pos_origin);
					number_passengers_action[v].erase(number_passengers_action[v].begin() + pos_origin);
					//re-update further arrival and departure times
					/*for (int i=pos_origin+1; i<=number_stops[v];i++) {
						arrival_time_stop[v][i] = saved_arrival_time[i];
						departure_time_stop[v][i] = saved_departure_time[i];
						slack_time[best_v][i] = saved_slack_time[i];
					}*/
					arrival_time_stop[v].erase(arrival_time_stop[v].begin() + pos_origin);
					departure_time_stop[v].erase(departure_time_stop[v].begin() + pos_origin);
					slack_time[v].erase(slack_time[v].begin() + pos_origin);
					free_capacity[v].erase(free_capacity[v].begin() + pos_origin);
					
					if (free_capacity[v].size() == 2)
						departure_time_stop[v][0] = 0;

					//ultimately serve passenger with a 3rd party vehicle
					//serve_passenger_third_party_vehicle(p);
				}
			}

			update_URT(v);

		} else {

			min_increase_length = INT_MAX;
			repeated_station = false;
			flexibilize_lat_departure_time = true;
			sel_origin = -1;
			cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home, empty_running_vehicle);

			if (sel_origin != -1) {

				if (not repeated_station) {
					same_station = false;
					//<<"sel o"<<sel_origin;
					stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
					
					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
					action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_origin, 1);

					//updating arrival and departure from stops
					//arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[depot][sel_origin]);
					//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
					
					int curr_dpt_time = earliest_departure[p]-travel_time[stops[v][0]][sel_origin];
					if (curr_dpt_time < current_time){
						curr_dpt_time = current_time;
					}

					if (pos_origin == 1)
						departure_time_stop[v][0] = curr_dpt_time;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[stops[v][pos_origin-1]][sel_origin]);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[stops[v][pos_origin-1]][sel_origin]);
					slack_time[v].insert(slack_time[v].begin() + pos_origin, 86400);

					prv_capacity = free_capacity[v][pos_origin-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_origin, prv_capacity-1);
					//free_capacity[v]--;

					//solution_cost += min_increase_length; //updates solution cost
					number_stops[v]++;
				} else {

					same_station = true;
					//<<"heere4"<<endl;
					//update passenger performing actions on the stops
					action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
					number_passengers_action[v][pos_origin]++;

					prv_arr_time_at_origin = arrival_time_stop[v][pos_origin];
					prv_dpt_time_at_origin = departure_time_stop[v][pos_origin];
					//no need to update the arrival time as it does not change
					//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
					if (earliest_departure[p] > departure_time_stop[v][pos_origin])
						departure_time_stop[v][pos_origin] = earliest_departure[p];
					//slack_time[best_v][best_pos_origin] //no need to update
					prv_capacity = free_capacity[v][pos_origin];
					free_capacity[v][pos_origin] = prv_capacity-1;

				}

				vehicle_assigned[p] = v;

				/*for (int i=0; i<=number_stops[v];i++) {
					cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" ";
					for (int j=0; j<number_passengers_action[v][i];j++) 
						cout<<action_passengers[v][i][j]<<" ";
					cout<<"] ";
					cout<<"{"<<arrival_time_stop[v][i]<<"} ";
					cout<<"{"<<departure_time_stop[v][i]<<"} ";
					cout<<"|"<<slack_time[v][i]<<"|  ";
					cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
				
				}
				cout<<endl;*/

				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				flexibilize_arrival_time = false;
				see_if_arrival_departure_dont_match(v);
				cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
				//<<min_increase_length<<" "<<pos_destination<<endl;
				
				if (sel_destination != -1) {
					//cout<<"4hierxx"<<endl;
					
					//<<"pos dest: "<<sel_destination<<endl;
					stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
					
					passengers_departure_time_from_home[p] = best_departure_time_from_home;
					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
					action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


					//updating arrival and departure from stops
					int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
					//<<"xx "<<stops[v][pos_destination-1]<<endl;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
					slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
					slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

					prv_capacity = free_capacity[v][pos_destination-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

					//updates solution cost
					//<<"heere3"<<endl;
					user_ride_time[p] = travel_time[sel_origin][sel_destination];
					total_user_ride_time += user_ride_time[p];
					//<<"total_user_ride_time: "<<total_user_ride_time<<endl;

					/*odb_travel_time = travel_time[sel_origin][sel_destination];
					odb_distance = odb_travel_time*5.56;
					odb_distance = odb_distance/1000;
					odb_travel_time = odb_travel_time/60;
					odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
					if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
						odb_fare = ODB_minimum_fare[vehicle_type[v]];
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;*/

					number_stops[v]++;

					arrival_time_stop[v][pos_destination+1] = departure_time_stop[v][pos_destination]+travel_time[stops[v][pos_destination]][stops[v][pos_destination+1]];	
					departure_time_stop[v][pos_destination+1] = arrival_time_stop[v][pos_destination+1];

					int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[v]; i>=1; i--){
							if (slack_time[v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[v][i];
							} else {
								slack_time[v][i] = min_slack_time_so_far;
							}
						}

					//checks for next empty vehicle
					/*while (free_capacity[empty_vehicle].size() > 2) { 
						empty_vehicle++;
						if (empty_vehicle == total_number_vehicles) {
							//there are no empty vehicles
							empty_vehicle = -1;
							break;
						}
					}*/
					
					
				} else {

					//cout<<"5hierxx"<<endl;
					//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
					//try to insert giving flexibility according to latest_arrivel_time
					min_increase_length = INT_MAX;
					repeated_station = false;
					sel_destination = -1;
					flexibilize_arrival_time = true;
					see_if_arrival_departure_dont_match(v);
					cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);

					if (delay[p] <= max_flex_delay)
						accept_delay_trip = true;
					else 
						accept_delay_trip = false;
					if ((sel_destination != -1) && (accept_delay_trip)) {
						//<<"pos dest: "<<sel_destination<<endl;
						
						
						stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
						
						passengers_departure_time_from_home[p] = best_departure_time_from_home;
						//update passenger performing actions on the stops
						action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
						action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
						number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


						//updating arrival and departure from stops
						int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
						//<<"xx "<<stops[v][pos_destination-1]<<endl;
						arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
						departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
						slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
						slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

						prv_capacity = free_capacity[v][pos_destination-1];
						free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

						//updates solution cost
						//<<"heere4"<<endl;
						user_ride_time[p] = travel_time[sel_origin][sel_destination];
						total_user_ride_time += user_ride_time[p];
						//<<"total_user_ride_time: "<<total_user_ride_time<<endl;

						/*odb_travel_time = travel_time[sel_origin][sel_destination];
						odb_distance = odb_travel_time*5.56;
						odb_distance = odb_distance/1000;
						odb_travel_time = odb_travel_time/60;
						odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
						if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
							odb_fare = ODB_minimum_fare[vehicle_type[v]];
						solution_cost += odb_fare; //updates solution cost
						cost_trip[p] = odb_fare;*/



						number_stops[v]++;

						arrival_time_stop[v][pos_destination+1] = departure_time_stop[v][pos_destination]+travel_time[stops[v][pos_destination]][stops[v][pos_destination+1]];	
						departure_time_stop[v][pos_destination+1] = arrival_time_stop[v][pos_destination+1];

						int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[v]; i>=1; i--){
							if (slack_time[v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[v][i];
							} else {
								slack_time[v][i] = min_slack_time_so_far;
							}
						}

						//checks for next empty vehicle
						/*while (free_capacity[empty_vehicle].size() > 2) { 
							empty_vehicle++;
							if (empty_vehicle == total_number_vehicles) {
								//there are no empty vehicles
								empty_vehicle = -1;
								break;
							}
						}*/
						
						//update_URT(v);
					} else {

						//cout<<"5hierxx"<<endl;
						vehicle_assigned[p] = -1;
						if (not same_station) {
							//solution_cost -= saved_increase;
							
							number_stops[v]--;
							stops[v].erase(stops[v].begin() + pos_origin);
							action_passengers[v].erase(action_passengers[v].begin() + pos_origin);
							number_passengers_action[v].erase(number_passengers_action[v].begin() + pos_origin);
							//re-update further arrival and departure times
							/*for (int i=pos_origin+1; i<=number_stops[v];i++) {
								arrival_time_stop[v][i] = saved_arrival_time[i];
								departure_time_stop[v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}*/
							arrival_time_stop[v].erase(arrival_time_stop[v].begin() + pos_origin);
							departure_time_stop[v].erase(departure_time_stop[v].begin() + pos_origin);
							slack_time[v].erase(slack_time[v].begin() + pos_origin);
							free_capacity[v].erase(free_capacity[v].begin() + pos_origin);

							if (free_capacity[v].size() == 2)
								departure_time_stop[v][0] = 0;
						} else {

							action_passengers[v][pos_origin].erase(action_passengers[v][pos_origin].begin());

							number_passengers_action[v][pos_origin]--;

							arrival_time_stop[v][pos_origin] = prv_arr_time_at_origin;
							departure_time_stop[v][pos_origin] = prv_dpt_time_at_origin;
							free_capacity[v][pos_origin]++;

						}

						//ultimately serve passenger with a 3rd party vehicle
						//serve_passenger_third_party_vehicle(p);
					}
				}

				update_URT(v);

			}


		}




		
		
		/*for (int i=0; i<=number_stops[v];i++) {
			cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" [ ";
			for (int j=0; j<number_passengers_action[v][i];j++) 
				cout<<action_passengers[v][i][j]<<" ";
			cout<<"]  ";

			cout<<"{"<<arrival_time_stop[v][i]<<"} ";
			cout<<"{"<<departure_time_stop[v][i]<<"} ";
			cout<<"|"<<slack_time[v][i]<<"|  ";
			cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
		}

			
		cout<<endl;*/

		//try_to_insert_in_already_running_vehicle(p);

		

	} else {

		//no empty vehicles. passenger will be inserted at the cheapest cost amongst all vehicles that is feasible

		if (filtered_vehicles.size()>0)
			filtered_vehicles.clear();
		filter_vehicles(p);

		//<<"filtered_vehicles SIZE"<<filtered_vehicles.size()<<endl;
		bool not_feasible_insertion = true;
		int iterations = 0;
		bool tested_all_vehicles_once = false;

		/*if (filtered_vehicles.size() == 0){
			serve_passenger_third_party_vehicle(p);
		}*/
		
		while ((not_feasible_insertion) && (iterations < filtered_vehicles.size())) {
			
			int best_pos_origin, best_pos_destination;
			int best_min_increase_length;
			int best_sel_origin, best_sel_destination, best_v;
			bool best_repeated_station;
			
			best_min_increase_length = INT_MAX;
			best_v = -1;
			curr_number_insertions = 0;
			//<<"heeerexx";
			for (int vf=0; vf<filtered_vehicles.size();vf++) {
				int v = filtered_vehicles[vf];
				//<<v<<" "<<free_capacity[v]<<endl;
				min_increase_length = INT_MAX;
				repeated_station = false;
				//(free_capacity[v] > 0) && 
				if (blocked_vehicles[v] == 0) {

					if (tested_all_vehicles_once)
						flexibilize_lat_departure_time = true;
					else
						flexibilize_lat_departure_time = false;
					
					/*cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);

					if (min_increase_length < best_min_increase_length) {
						best_min_increase_length = min_increase_length;
						best_sel_origin = sel_origin;
						best_pos_origin = pos_origin;
						best_v = v;
						best_repeated_station = repeated_station;
					}*/

					cheapest_origin2(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);
					
				}
			}
			//<<endl;

			//<<"curr insert5: " << curr_number_insertions<<endl;
			sort(insertions, insertions+curr_number_insertions, comparator);

			//<<best_v<<endl;
			//if (best_v != -1) {
			bool no_feasible_insertion2 = true;
			int iterations2 = 0;
			if (curr_number_insertions > 0) {
			
				//int rclsize = 5;
				remaining_insertions = curr_number_insertions;
				//if (rclsize < remaining_insertions)
				//	next_replace = rclsize;
				//else
				next_replace = remaining_insertions-1;

				while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions)) {

					//<<best_v<<"\n";
					int prv_arr_time_at_origin, prv_dpt_time_at_origin;

					//if (rclsize <= remaining_insertions)
					//	selected_insertion = rand() % rclsize;
					//else
					selected_insertion = rand() % remaining_insertions;

					//<<"selec inser"<<selected_insertion<<endl;
					
					best_min_increase_length = insertions[selected_insertion].increase_length;
					best_sel_origin = insertions[selected_insertion].sel_station;
					best_pos_origin = insertions[selected_insertion].pos_station;
					best_v = insertions[selected_insertion].v;
					best_repeated_station = insertions[selected_insertion].repeated_station;
					remaining_insertions--; //updates the number of remaining feasible insertions

					//bring the next insertion to the top 3
					insertions[selected_insertion].increase_length = insertions[next_replace].increase_length;
					insertions[selected_insertion].sel_station = insertions[next_replace].sel_station;
					insertions[selected_insertion].pos_station = insertions[next_replace].pos_station;
					insertions[selected_insertion].v = insertions[next_replace].v;
					insertions[selected_insertion].repeated_station = insertions[next_replace].repeated_station;
					next_replace--;

					
					if (not best_repeated_station) {
						//<<"heere3"<<endl;

						//<<best_sel_origin<<" "<<best_min_increase_length<<endl;

						stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
						number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

						//updating arrival and departure from stops
						
						//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
						
						arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
						
						if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
						else
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
						slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
						
						prv_capacity = free_capacity[best_v][best_pos_origin-1];
						free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);
						
					} else {
						//<<"heere4"<<endl;
						//update passenger performing actions on the stops
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
						number_passengers_action[best_v][best_pos_origin]++;

						prv_arr_time_at_origin = arrival_time_stop[best_v][best_pos_origin];
						prv_dpt_time_at_origin = departure_time_stop[best_v][best_pos_origin];
						//no need to update the arrival time as it does not change
						//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
						if (earliest_departure[p] > departure_time_stop[best_v][best_pos_origin])
							departure_time_stop[best_v][best_pos_origin] = earliest_departure[p];
						//slack_time[best_v][best_pos_origin] //no need to update
						prv_capacity = free_capacity[best_v][best_pos_origin];
						free_capacity[best_v][best_pos_origin] = prv_capacity-1;
					}

					//update further arrival, departure times and slack times
					for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
						saved_arrival_time[i] = arrival_time_stop[best_v][i];
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						saved_departure_time[i] = departure_time_stop[best_v][i];
						if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						}
						saved_slack_time[i] = slack_time[best_v][i];
						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[i];
						//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
					}
					
					
					//solution_cost += best_min_increase_length;
					//saved_increase = best_min_increase_length;
					
					vehicle_assigned[p] = best_v;

					//if (best_v == 24) {
					/*cout<<"flex: "<<flexibilize_lat_departure_time<<endl;
					for (int i=0; i<=number_stops[best_v];i++) {
						cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
						for (int j=0; j<number_passengers_action[best_v][i];j++) 
							cout<<action_passengers[best_v][i][j]<<" ";
						cout<<"]  ";

						cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
						cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
						cout<<"|"<<slack_time[best_v][i]<<"|  ";
						cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
					}
					//}
					cout<<endl;*/

					best_min_increase_length = INT_MAX;
					
					min_increase_length = INT_MAX;
					repeated_station = false;
					sel_destination = -1;
					if (tested_all_vehicles_once)
						flexibilize_arrival_time = true;
					else
						flexibilize_arrival_time = false;

					see_if_arrival_departure_dont_match(best_v);
					cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
				
					//<<"pos origin: "<<best_pos_origin<<" "<<endl;
					//<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
					
					if (sel_destination != -1) { 

						no_violation_capacity = true;
						for (int i=best_pos_origin+1;i<pos_destination;i++){

							free_capacity[best_v][i]--;

							if (free_capacity[best_v][i] < 0) {
								no_violation_capacity = false;
							}
							
						}
					}

					if (delay[p] <= max_flex_delay)
						accept_delay_trip = true;
					else 
						accept_delay_trip = false;


					if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip)) {
						
						
						//cout<<"7hierxx"<<endl;
						no_feasible_insertion2 = false;
						not_feasible_insertion = false;

						passengers_departure_time_from_home[p] = insertions[next_replace].passengers_departure_time_from_home;
						/*if (iterations2 >  0) {
							printf("HEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEEREHEEEERE\n");
							printf("HEEEERE\n");
							printf("HEEEERE\n");
							printf("HEEEERE\n");
							printf("HEEEERE\n");
							printf("HEEEERE\n");
							printf("HEEEERE\n");
							printf("%d it\n", iterations2);
						}*/
						if (not repeated_station) {
							stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
							number_stops[best_v]++;

							//update passenger performing actions on the stops
							action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);

							//updating arrival and departure from stops
							int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
							arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
							slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

							prv_capacity = free_capacity[best_v][pos_destination-1];
							free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
						} else {

							//update passenger performing actions on the stops
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v][pos_destination]++;

							//according to the update when the origin is added, if the stop already exists then no need to change
							//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//arrival_time_stop[best_v][pos_destination] = arr_time;
							//departure_time_stop[best_v][pos_destination] = arr_time;
							int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < slack_time[best_v][pos_destination]) 
								slack_time[best_v][pos_destination] = slc_time;

							//prv_capacity = free_capacity[best_v][pos_destination];
							//free_capacity[best_v][pos_destination] = prv_capacity+1;

						}


						//update further arrival, departure times, and slack times
						for (int i=pos_destination+1; i<=number_stops[best_v];i++) {
							old_arr_time = arrival_time_stop[best_v][i];
							arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
							old_dpt_time = departure_time_stop[best_v][i];
							if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
								departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
							}
							slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						}

						//for (int i=1; i<pos_destination;i++){
						//	int min_slack_time = INT_MAX;
						//	for (int j=i+1; i<number_stops[best_v];i++){
						//		if (slack_time[best_v][j] < min_slack_time)
						//			min_slack_time = slack_time[best_v][j];
						//	}
						//	slack_time[best_v][i] = min_slack_time;
						//}

						int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[best_v]; i>=1; i--){
							if (slack_time[best_v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[best_v][i];
							} else {
								slack_time[best_v][i] = min_slack_time_so_far;
							}
						}
						//updates solution cost
						
						//<<"heere5"<<endl;
						//<<"stations "<<best_pos_origin<<" "<<sel_destination<<endl;
						user_ride_time[p] = 0;
						//total_user_ride_time += user_ride_time[p];
						//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
						int current_user_ride_time, difference;
						bool leave_loop = false;
						for (int i=0; i<=number_stops[best_v];i++) {
							for (int j=0; j<number_passengers_action[best_v][i];j++) {
								int save_p = action_passengers[best_v][i][j];
								//if (save_p != p){
									for (int k=i+1; k<=number_stops[best_v];k++) {
										leave_loop = false;
										for (int l=0; l<number_passengers_action[best_v][k];l++) {
											if (action_passengers[best_v][k][l] == save_p){
												current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
												if (current_user_ride_time != user_ride_time[save_p]) {
													difference = current_user_ride_time -  user_ride_time[save_p];
													total_user_ride_time += difference;
													//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
													user_ride_time[save_p] = current_user_ride_time;
													l = number_passengers_action[best_v][k]+1;
													//k = number_stops[best_v]+2; //leave loop
													leave_loop = true;
												}
											}
										}
										if (leave_loop)
											k = number_stops[best_v]+2;
									}
								//}
							} 
						}


						
						
						/*odb_travel_time = travel_time[best_sel_origin][sel_destination];
						odb_distance = odb_travel_time*5.56;
						odb_distance = odb_distance/1000;
						odb_travel_time = odb_travel_time/60;
						odb_fare = ((ODB_base_fare[vehicle_type[best_v]] + ODB_km_charge[vehicle_type[best_v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[best_v]]) + ODB_booking_fee[vehicle_type[best_v]]; 
						if (odb_fare < ODB_minimum_fare[vehicle_type[best_v]])
							odb_fare = ODB_minimum_fare[vehicle_type[best_v]];
						solution_cost += odb_fare; //updates solution cost
						cost_trip[p] = odb_fare;*/

					} else {

						//no feasible insertion for destination
						//<<"no feasible insertion for destination"<<endl;
						//blocked_vehicles[best_v] = 1; //I suppose this is not necessary anymore. I'm already testing all vehicles with array insertions

						iterations2++;
						//solution_cost -= saved_increase;
						vehicle_assigned[p] = -1;
						if (sel_destination != -1) {
							for (int i=best_pos_origin+1;i<pos_destination;i++){

								free_capacity[best_v][i]++;
			
							}
						}

						if (number_passengers_action[best_v][best_pos_origin] == 1) {
							number_stops[best_v]--;
							stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
							action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
							number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
							

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}
							arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
							departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
							slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


							free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


						} else {

							//<<"heere"<<endl;
							//<<action_passengers[best_v][best_pos_origin][1]<<endl;
							action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
							
							number_passengers_action[best_v][best_pos_origin]--;

							arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
							departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
							free_capacity[best_v][best_pos_origin]++;

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}

						}
					}

					update_URT(best_v);
					

					//if (best_v == 24) {
						//<<flexibilize_lat_departure_time<<endl;
					/*cout<<"xyz1"<<endl;
	 				for (int i=0; i<=number_stops[best_v];i++) {
						cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
						for (int j=0; j<number_passengers_action[best_v][i];j++) 
							cout<<action_passengers[best_v][i][j]<<" ";
						cout<<"]  ";

						cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
						cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
						cout<<"|"<<slack_time[best_v][i]<<"|  ";
						cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
					}*/
					//}	
					//<<endl;
				} //end while
			} else {
				iterations = total_number_vehicles;
			}

			
			if (no_feasible_insertion2) {

				//iterations++;
				iterations = total_number_vehicles;
			}
			if (iterations >= total_number_vehicles) {
				if (not tested_all_vehicles_once) {
					tested_all_vehicles_once = true;
					//usually this is restarted flexibilizing the latest departure time, but we dont want that anymore
					//now lets just keep this way
					iterations = 0;
					//<<"serving 3rd party";
					//serve_passenger_third_party_vehicle(p);
					//for (int i=0;i<total_number_vehicles;i++)
					//	blocked_vehicles[i] = 0;
				} else {
					//ultimately serve passenger with a 3rd party vehicle
					//<<"serving 3rd party";
					//serve_passenger_third_party_vehicle(p);
				}
			}
		}
	}

	
	if (vehicle_assigned[p] == -1) {
		serve_passenger_third_party_vehicle(p);
	}

	route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[i] = 0;
}

//this cheapest insertion considers to insert passengers at positions that are not the min increase in length traveled
//although this is slower than previous version. compare if it's worth it :) [regarding number of passengers served]
/*void cheapest_insertion2(int p){

	int odb_fare, odb_travel_time, odb_distance;
	int v;
	int pos_origin, pos_destination;
	int sel_origin, sel_destination;
	int min_increase_length;
	bool repeated_station, no_violation_capacity;
	int saved_increase;
	int old_dpt_time, old_arr_time;
	int prv_capacity;
	bool flexibilize_arrival_time, flexibilize_lat_departure_time, accept_delay_trip;

	int best_empty_vehicle, capacity_best_empty_vehicle;
	int best_distance_depot;
	best_empty_vehicle = -1;
	capacity_best_empty_vehicle = INT_MIN;
	best_distance_depot = INT_MAX;
	int best_depot;
	int best_departure_time_from_home;

	int veh;
	int ttcsd = INT_MAX;
	//check for closest depot that has an empty vehicle
	//second check for highest capacitated vehicle from that depot
	for (int i = 0; i < number_depots; i++) {

		for (int k=0;k<number_stops_origin[p];k++) {
			if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
				ttcsd = travel_time[stops_origin[p][k]][depot[i]];
			}
		}

		if ((best_distance_depot == 1) || (ttcsd <= best_distance_depot)) {
			capacity_best_empty_vehicle = INT_MIN;
			for (int j = 0; j < number_vehicles_at_depot[i]; j++) {

				veh = vehicles_at_depot[i][j];
				if (free_capacity[veh].size() == 2){ //means that vehicle is empty
					if (free_capacity[veh][0] >= capacity_best_empty_vehicle) {
						best_empty_vehicle = veh;
						best_depot = depot[i];
						capacity_best_empty_vehicle = free_capacity[veh][0];
						best_distance_depot = ttcsd;
					}
				}
			}
		}
	}

	v = best_empty_vehicle;

	if (v != -1) {
		//<<"herexx";
		
		for (int i=0; i<=number_stops[v];i++)
			cout<<stops[v][i]<<" ";
		cout<<endl;

		//there's at least one empty vehicle, so the passenger will be inserted on it
		min_increase_length = INT_MAX;
		repeated_station = false;
		flexibilize_lat_departure_time = false;
		sel_origin = -1;
		cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);
		
		if (sel_origin != -1) {
			//<<"sel o"<<sel_origin;
			stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
			
			//update passenger performing actions on the stops
			action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
			action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
			number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_origin, 1);

			//updating arrival and departure from stops
			//arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[depot][sel_origin]);
			cout<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
			
			int curr_dpt_time = earliest_departure[p]-travel_time[stops[v][0]][sel_origin];
			if (curr_dpt_time < current_time){
				curr_dpt_time = current_time;
			}
			departure_time_stop[v][0] = curr_dpt_time;
			arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
			departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
			slack_time[v].insert(slack_time[v].begin() + pos_origin, 86400);

			prv_capacity = free_capacity[v][pos_origin-1];
			free_capacity[v].insert(free_capacity[v].begin() + pos_origin, prv_capacity-1);
			//free_capacity[v]--;

			//solution_cost += min_increase_length; //updates solution cost
			number_stops[v]++;

			vehicle_assigned[p] = v;

			for (int i=0; i<=number_stops[v];i++) {
				cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" ";
				for (int j=0; j<number_passengers_action[v][i];j++) 
					cout<<action_passengers[v][i][j]<<" ";
				cout<<"] ";
				cout<<"{"<<arrival_time_stop[v][i]<<"} ";
				cout<<"{"<<departure_time_stop[v][i]<<"} ";
				cout<<"|"<<slack_time[v][i]<<"|  ";
				cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
			
			}
			cout<<endl;

			min_increase_length = INT_MAX;
			repeated_station = false;
			sel_destination = -1;
			flexibilize_arrival_time = false;
			cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
			//<<min_increase_length<<" "<<pos_destination<<endl;
			
			if (sel_destination != -1) {
				cout<<"pos dest: "<<sel_destination<<endl;
				stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
				
				passengers_departure_time_from_home[p] = best_departure_time_from_home;
				//update passenger performing actions on the stops
				action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
				action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
				number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


				//updating arrival and departure from stops
				int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
				//<<"xx "<<stops[v][pos_destination-1]<<endl;
				arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
				departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
				slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
				slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

				prv_capacity = free_capacity[v][pos_destination-1];
				free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

				//solution_cost += min_increase_length; //updates solution cost
				odb_travel_time = travel_time[sel_origin][sel_destination];
				odb_distance = odb_travel_time*5.56;

				odb_distance = odb_distance/1000;
				odb_travel_time = odb_travel_time/60;
				odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
				if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
					odb_fare = ODB_minimum_fare[vehicle_type[v]];
				solution_cost += odb_fare; //updates solution cost
				cost_trip[p] = odb_fare;

				//saved_increase = min_increase_length;
				number_stops[v]++;

				
			} else {
				//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
				//try to insert giving flexibility according to latest_arrivel_time
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				flexibilize_arrival_time = true;
				cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);

				if (delay[p] <= max_flex_delay)
					accept_delay_trip = true;
				else 
					accept_delay_trip = false;
				if ((sel_destination != -1) && (accept_delay_trip)) {

					passengers_departure_time_from_home[p] = best_departure_time_from_home;
					cout<<"pos dest: "<<sel_destination<<endl;
					stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
					
					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
					action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


					//updating arrival and departure from stops
					int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
					//<<"xx "<<stops[v][pos_destination-1]<<endl;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
					slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
					slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

					prv_capacity = free_capacity[v][pos_destination-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

					odb_travel_time = travel_time[sel_origin][sel_destination];
					odb_distance = odb_travel_time*5.56;
					odb_distance = odb_distance/1000;
					odb_travel_time = odb_travel_time/60;
					odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
					if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
						odb_fare = ODB_minimum_fare[vehicle_type[v]];
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;
					number_stops[v]++;

					
				} else {
					//solution_cost -= saved_increase;
					vehicle_assigned[p] = -1;
					number_stops[v]--;
					stops[v].erase(stops[v].begin() + pos_origin);
					action_passengers[v].erase(action_passengers[v].begin() + pos_origin);
					number_passengers_action[v].erase(number_passengers_action[v].begin() + pos_origin);
					
					arrival_time_stop[v].erase(arrival_time_stop[v].begin() + pos_origin);
					departure_time_stop[v].erase(departure_time_stop[v].begin() + pos_origin);
					slack_time[v].erase(slack_time[v].begin() + pos_origin);
					free_capacity[v].erase(free_capacity[v].begin() + pos_origin);
					departure_time_stop[v][0] = 0;

					//ultimately serve passenger with a 3rd party vehicle
					serve_passenger_third_party_vehicle(p);
				}
			}

		} else {

			min_increase_length = INT_MAX;
			repeated_station = false;
			flexibilize_lat_departure_time = true;
			sel_origin = -1;
			cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);

			if (sel_origin != -1) {
				//<<"sel o"<<sel_origin;
				stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
				
				//update passenger performing actions on the stops
				action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
				action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
				number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_origin, 1);

				//updating arrival and departure from stops
				//arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[depot][sel_origin]);
				cout<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
				
				int curr_dpt_time = earliest_departure[p]-travel_time[stops[v][0]][sel_origin];
				if (curr_dpt_time < current_time){
					curr_dpt_time = current_time;
				}
				departure_time_stop[v][0] = curr_dpt_time;
				arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
				departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
				slack_time[v].insert(slack_time[v].begin() + pos_origin, 86400);

				prv_capacity = free_capacity[v][pos_origin-1];
				free_capacity[v].insert(free_capacity[v].begin() + pos_origin, prv_capacity-1);
				//free_capacity[v]--;

				//solution_cost += min_increase_length; //updates solution cost
				number_stops[v]++;

				vehicle_assigned[p] = v;

				for (int i=0; i<=number_stops[v];i++) {
					cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" ";
					for (int j=0; j<number_passengers_action[v][i];j++) 
						cout<<action_passengers[v][i][j]<<" ";
					cout<<"] ";
					cout<<"{"<<arrival_time_stop[v][i]<<"} ";
					cout<<"{"<<departure_time_stop[v][i]<<"} ";
					cout<<"|"<<slack_time[v][i]<<"|  ";
					cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
				
				}
				cout<<endl;

				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				flexibilize_arrival_time = false;
				cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
				//<<min_increase_length<<" "<<pos_destination<<endl;
				
				if (sel_destination != -1) {
					cout<<"pos dest: "<<sel_destination<<endl;
					stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
					
					passengers_departure_time_from_home[p] = best_departure_time_from_home;

					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
					action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


					//updating arrival and departure from stops
					int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
					//<<"xx "<<stops[v][pos_destination-1]<<endl;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
					slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
					slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

					prv_capacity = free_capacity[v][pos_destination-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

					odb_travel_time = travel_time[sel_origin][sel_destination];
					odb_distance = odb_travel_time*5.56;
					odb_distance = odb_distance/1000;
					odb_travel_time = odb_travel_time/60;
					odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
					if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
						odb_fare = ODB_minimum_fare[vehicle_type[v]];
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;
					number_stops[v]++;

					
				} else {
					//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
					//try to insert giving flexibility according to latest_arrivel_time
					min_increase_length = INT_MAX;
					repeated_station = false;
					sel_destination = -1;
					flexibilize_arrival_time = true;
					cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);

					if (delay[p] <= max_flex_delay)
						accept_delay_trip = true;
					else 
						accept_delay_trip = false;
					if ((sel_destination != -1) && (accept_delay_trip)) {

						passengers_departure_time_from_home[p] = best_departure_time_from_home;

						cout<<"pos dest: "<<sel_destination<<endl;
						stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
						
						//update passenger performing actions on the stops
						action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
						action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
						number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


						//updating arrival and departure from stops
						int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
						//<<"xx "<<stops[v][pos_destination-1]<<endl;
						arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
						departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
						slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
						slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

						prv_capacity = free_capacity[v][pos_destination-1];
						free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

						odb_travel_time = travel_time[sel_origin][sel_destination];
						odb_distance = odb_travel_time*5.56;
						odb_distance = odb_distance/1000;
						odb_travel_time = odb_travel_time/60;
						odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
						if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
							odb_fare = ODB_minimum_fare[vehicle_type[v]];
						solution_cost += odb_fare; //updates solution cost
						cost_trip[p] = odb_fare;
						number_stops[v]++;

						
					} else {

						//solution_cost -= saved_increase;
						vehicle_assigned[p] = -1;
						number_stops[v]--;
						stops[v].erase(stops[v].begin() + pos_origin);
						action_passengers[v].erase(action_passengers[v].begin() + pos_origin);
						number_passengers_action[v].erase(number_passengers_action[v].begin() + pos_origin);
						//re-update further arrival and departure times
						
						arrival_time_stop[v].erase(arrival_time_stop[v].begin() + pos_origin);
						departure_time_stop[v].erase(departure_time_stop[v].begin() + pos_origin);
						slack_time[v].erase(slack_time[v].begin() + pos_origin);
						free_capacity[v].erase(free_capacity[v].begin() + pos_origin);
						departure_time_stop[v][0] = 0;

						//ultimately serve passenger with a 3rd party vehicle
						serve_passenger_third_party_vehicle(p);
					}
				}

			}


		}

		
		for (int i=0; i<=number_stops[v];i++) {
			cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" [ ";
			for (int j=0; j<number_passengers_action[v][i];j++) 
				cout<<action_passengers[v][i][j]<<" ";
			cout<<"]  ";

			cout<<"{"<<arrival_time_stop[v][i]<<"} ";
			cout<<"{"<<departure_time_stop[v][i]<<"} ";
			cout<<"|"<<slack_time[v][i]<<"|  ";
			cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
		}

			//<<stops[v][i]<<" ";
		cout<<endl;

		

	} else {

		//no empty vehicles. passenger will be inserted at the cheapest cost amongst all vehicles that is feasible
		bool not_feasible_insertion = true;
		int iterations = 0;
		bool tested_all_vehicles_once = false;
		
		while ((not_feasible_insertion) && (iterations < total_number_vehicles)) {
			
			int best_pos_origin, best_pos_destination;
			int best_min_increase_length;
			int best_sel_origin, best_sel_destination, best_v;
			bool best_repeated_station;
			
			best_min_increase_length = INT_MAX;
			best_v = -1;
			curr_number_insertions = 0;
			cout<<"heeerexx";
			for (v=0; v<total_number_vehicles;v++) {
				//<<v<<" "<<free_capacity[v]<<endl;
				min_increase_length = INT_MAX;
				repeated_station = false;
				//(free_capacity[v] > 0) && 
				if (blocked_vehicles[v] == 0) {

					if (tested_all_vehicles_once)
						flexibilize_lat_departure_time = true;
					else
						flexibilize_lat_departure_time = false;
					
					

					cheapest_origin2(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);
					
				}
			}
			cout<<endl;

			cout<<"curr insert: " << curr_number_insertions<<endl;
			sort(insertions, insertions+curr_number_insertions, comparator);

			cout<<best_v<<endl;
			//if (best_v != -1) {
			bool no_feasible_insertion2 = true;
			int iterations2 = 0;
			if (curr_number_insertions > 0) {
			
				while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions)) {

					//<<best_v<<"\n";
					int prv_arr_time_at_origin, prv_dpt_time_at_origin;

					best_min_increase_length = insertions[iterations2].increase_length;
					best_sel_origin = insertions[iterations2].sel_station;
					best_pos_origin = insertions[iterations2].pos_station;
					best_v = insertions[iterations2].v;
					best_repeated_station = insertions[iterations2].repeated_station;

					if (not best_repeated_station) {
						stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
						number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

						//updating arrival and departure from stops
						
						//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
						
						arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
						
						if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
						else
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
						slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
						prv_capacity = free_capacity[best_v][best_pos_origin-1];
						free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);
					} else {
						//update passenger performing actions on the stops
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
						number_passengers_action[best_v][best_pos_origin]++;

						prv_arr_time_at_origin = arrival_time_stop[best_v][best_pos_origin];
						prv_dpt_time_at_origin = departure_time_stop[best_v][best_pos_origin];
						//no need to update the arrival time as it does not change
						//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
						if (earliest_departure[p] > departure_time_stop[best_v][best_pos_origin])
							departure_time_stop[best_v][best_pos_origin] = earliest_departure[p];
						//slack_time[best_v][best_pos_origin] //no need to update
						prv_capacity = free_capacity[best_v][best_pos_origin];
						free_capacity[best_v][best_pos_origin] = prv_capacity-1;
					}

					//update further arrival, departure times and slack times
					for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
						saved_arrival_time[i] = arrival_time_stop[best_v][i];
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						saved_departure_time[i] = departure_time_stop[best_v][i];
						if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						}
						saved_slack_time[i] = slack_time[best_v][i];
						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[i];
						//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
					}
					
					
					//solution_cost += best_min_increase_length;
					//saved_increase = best_min_increase_length;
					
					vehicle_assigned[p] = best_v;

					//if (best_v == 24) {
					cout<<"flex: "<<flexibilize_lat_departure_time<<endl;
					for (int i=0; i<=number_stops[best_v];i++) {
						cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
						for (int j=0; j<number_passengers_action[best_v][i];j++) 
							cout<<action_passengers[best_v][i][j]<<" ";
						cout<<"]  ";

						cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
						cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
						cout<<"|"<<slack_time[best_v][i]<<"|  ";
						cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
					}
					//}
					cout<<endl;

					best_min_increase_length = INT_MAX;
					
					min_increase_length = INT_MAX;
					repeated_station = false;
					sel_destination = -1;
					if (tested_all_vehicles_once)
						flexibilize_arrival_time = true;
					else
						flexibilize_arrival_time = false;
					cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
				
					//<<"pos origin: "<<best_pos_origin<<" "<<endl;
					//<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
					
					if (sel_destination != -1) { 

						no_violation_capacity = true;
						for (int i=best_pos_origin+1;i<pos_destination;i++){

							free_capacity[best_v][i]--;

							if (free_capacity[best_v][i] < 0) {
								no_violation_capacity = false;
							}
							
						}
					}

					if (delay[p] <= max_flex_delay)
						accept_delay_trip = true;
					else 
						accept_delay_trip = false;

					if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip)) {
						
						passengers_departure_time_from_home[p] = insertions[iterations2].passengers_departure_time_from_home;
						no_feasible_insertion2 = false;
						not_feasible_insertion = false;

						
						if (not repeated_station) {
							stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
							number_stops[best_v]++;

							//update passenger performing actions on the stops
							action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);

							//updating arrival and departure from stops
							int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
							arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
							slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

							prv_capacity = free_capacity[best_v][pos_destination-1];
							free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
						} else {

							//update passenger performing actions on the stops
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v][pos_destination]++;

							//according to the update when the origin is added, if the stop already exists then no need to change
							//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//arrival_time_stop[best_v][pos_destination] = arr_time;
							//departure_time_stop[best_v][pos_destination] = arr_time;
							int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < slack_time[best_v][pos_destination]) 
								slack_time[best_v][pos_destination] = slc_time;

							//prv_capacity = free_capacity[best_v][pos_destination];
							//free_capacity[best_v][pos_destination] = prv_capacity+1;

						}


						//update further arrival, departure times, and slack times
						for (int i=pos_destination+1; i<=number_stops[best_v];i++) {
							old_arr_time = arrival_time_stop[best_v][i];
							arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
							old_dpt_time = departure_time_stop[best_v][i];
							if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
								departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
							}
							slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						}

						//for (int i=1; i<pos_destination;i++){
						//	int min_slack_time = INT_MAX;
						//	for (int j=i+1; i<number_stops[best_v];i++){
						//		if (slack_time[best_v][j] < min_slack_time)
						//			min_slack_time = slack_time[best_v][j];
						//	}
						//	slack_time[best_v][i] = min_slack_time;
						//}

						int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[best_v]; i>=1; i--){
							if (slack_time[best_v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[best_v][i];
							} else {
								slack_time[best_v][i] = min_slack_time_so_far;
							}
						}
						
						odb_travel_time = travel_time[best_sel_origin][sel_destination];
						odb_distance = odb_travel_time*5.56;
						odb_distance = odb_distance/1000;
						odb_travel_time = odb_travel_time/60;
						odb_fare = ((ODB_base_fare[vehicle_type[best_v]] + ODB_km_charge[vehicle_type[best_v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[best_v]]) + ODB_booking_fee[vehicle_type[best_v]]; 
						if (odb_fare < ODB_minimum_fare[vehicle_type[best_v]])
							odb_fare = ODB_minimum_fare[vehicle_type[best_v]];
						solution_cost += odb_fare; //updates solution cost
						cost_trip[p] = odb_fare;
					} else {

						//no feasible insertion for destination
						cout<<"no feasible insertion for destination"<<endl;
						//blocked_vehicles[best_v] = 1; //I suppose this is not necessary anymore. I'm already testing all vehicles with array insertions

						iterations2++;
						//solution_cost -= saved_increase;
						vehicle_assigned[p] = -1;
						if (sel_destination != -1) {
							for (int i=best_pos_origin+1;i<pos_destination;i++){

								free_capacity[best_v][i]++;
			
							}
						}

						if (number_passengers_action[best_v][best_pos_origin] == 1) {
							number_stops[best_v]--;
							stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
							action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
							number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
							

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}
							arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
							departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
							slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


							free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


						} else {

							//<<"heere"<<endl;
							//<<action_passengers[best_v][best_pos_origin][1]<<endl;
							action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
							
							number_passengers_action[best_v][best_pos_origin]--;

							arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
							departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
							free_capacity[best_v][best_pos_origin]++;

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}

						}
					}
					

					//if (best_v == 24) {
						//<<flexibilize_lat_departure_time<<endl;
					cout<<"xyz2"<<endl;
	 				for (int i=0; i<=number_stops[best_v];i++) {
						cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
						for (int j=0; j<number_passengers_action[best_v][i];j++) 
							cout<<action_passengers[best_v][i][j]<<" ";
						cout<<"]  ";

						cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
						cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
						cout<<"|"<<slack_time[best_v][i]<<"|  ";
						cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
					}
					//}	
					cout<<endl;
				} //end while
			} else {
				iterations = total_number_vehicles;
			}

			if (no_feasible_insertion2) {

				//iterations++;
				iterations = total_number_vehicles;
			}
			if (iterations >= total_number_vehicles) {
				if (not tested_all_vehicles_once) {
					tested_all_vehicles_once = true;
					//iterations = 0;
					cout<<"serving 3rd party";
					serve_passenger_third_party_vehicle(p);
					//for (int i=0;i<total_number_vehicles;i++)
					//	blocked_vehicles[i] = 0;
				} else {
					//ultimately serve passenger with a 3rd party vehicle
					cout<<"serving 3rd party";
					serve_passenger_third_party_vehicle(p);
				}
			}
		}
	}

	route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[i] = 0;
}*/

/*void cheapest_insertion(int p){

	int odb_fare, odb_travel_time, odb_distance;
	int v;
	int pos_origin, pos_destination;
	int sel_origin, sel_destination;
	int min_increase_length;
	bool repeated_station, no_violation_capacity;
	int saved_increase;
	int old_dpt_time, old_arr_time;
	int prv_capacity;
	bool flexibilize_arrival_time, flexibilize_lat_departure_time, accept_delay_trip;

	int best_empty_vehicle, capacity_best_empty_vehicle;
	int best_distance_depot;
	best_empty_vehicle = -1;
	capacity_best_empty_vehicle = INT_MIN;
	best_distance_depot = INT_MAX;
	int best_depot;
	int best_departure_time_from_home;

	int veh;
	int ttcsd = INT_MAX;
	//check for closest depot that has an empty vehicle
	//second check for highest capacitated vehicle from that depot
	for (int i = 0; i < number_depots; i++) {

		for (int k=0;k<number_stops_origin[p];k++) {
			if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
				ttcsd = travel_time[stops_origin[p][k]][depot[i]];
			}
		}

		if ((best_distance_depot == 1) || (ttcsd <= best_distance_depot)) {
			capacity_best_empty_vehicle = INT_MIN;
			for (int j = 0; j < number_vehicles_at_depot[i]; j++) {

				veh = vehicles_at_depot[i][j];
				if (free_capacity[veh].size() == 2){ //means that vehicle is empty
					if (free_capacity[veh][0] >= capacity_best_empty_vehicle) {
						best_empty_vehicle = veh;
						best_depot = depot[i];
						capacity_best_empty_vehicle = free_capacity[veh][0];
						best_distance_depot = ttcsd;
					}
				}
			}
		}
	}

	v = best_empty_vehicle;

	if (v != -1) {
		//<<"herexx";
		
		for (int i=0; i<=number_stops[v];i++)
			cout<<stops[v][i]<<" ";
		cout<<endl;

		//there's at least one empty vehicle, so the passenger will be inserted on it
		min_increase_length = INT_MAX;
		repeated_station = false;
		flexibilize_lat_departure_time = false;
		sel_origin = -1;
		cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);
		
		if (sel_origin != -1) {
			//<<"sel o"<<sel_origin;
			stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
			
			//update passenger performing actions on the stops
			action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
			action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
			number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_origin, 1);

			//updating arrival and departure from stops
			//arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[depot][sel_origin]);
			cout<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
			
			int curr_dpt_time = earliest_departure[p]-travel_time[stops[v][0]][sel_origin];
			if (curr_dpt_time < current_time){
				curr_dpt_time = current_time;
			}
			departure_time_stop[v][0] = curr_dpt_time;
			arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
			departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
			slack_time[v].insert(slack_time[v].begin() + pos_origin, 86400);

			prv_capacity = free_capacity[v][pos_origin-1];
			free_capacity[v].insert(free_capacity[v].begin() + pos_origin, prv_capacity-1);
			//free_capacity[v]--;

			//solution_cost += min_increase_length; //updates solution cost
			number_stops[v]++;

			vehicle_assigned[p] = v;

			for (int i=0; i<=number_stops[v];i++) {
				cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" ";
				for (int j=0; j<number_passengers_action[v][i];j++) 
					cout<<action_passengers[v][i][j]<<" ";
				cout<<"] ";
				cout<<"{"<<arrival_time_stop[v][i]<<"} ";
				cout<<"{"<<departure_time_stop[v][i]<<"} ";
				cout<<"|"<<slack_time[v][i]<<"|  ";
				cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
			
			}
			cout<<endl;

			min_increase_length = INT_MAX;
			repeated_station = false;
			sel_destination = -1;
			flexibilize_arrival_time = false;
			cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
			//<<min_increase_length<<" "<<pos_destination<<endl;
			
			if (sel_destination != -1) {
				cout<<"pos dest: "<<sel_destination<<endl;
				stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
				
				passengers_departure_time_from_home[p] = best_departure_time_from_home;

				//update passenger performing actions on the stops
				action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
				action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
				number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


				//updating arrival and departure from stops
				int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
				//<<"xx "<<stops[v][pos_destination-1]<<endl;
				arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
				departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
				slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
				slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

				prv_capacity = free_capacity[v][pos_destination-1];
				free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

				odb_travel_time = travel_time[sel_origin][sel_destination];
				odb_distance = odb_travel_time*5.56;
				odb_distance = odb_distance/1000;
				odb_travel_time = odb_travel_time/60;
				odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
				if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
					odb_fare = ODB_minimum_fare[vehicle_type[v]];
				solution_cost += odb_fare; //updates solution cost
				cost_trip[p] = odb_fare;
				number_stops[v]++;

				
			} else {
				//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
				//try to insert giving flexibility according to latest_arrivel_time
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				flexibilize_arrival_time = true;
				cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);

				if (delay[p] <= max_flex_delay)
					accept_delay_trip = true;
				else 
					accept_delay_trip = false;
				if ((sel_destination != -1) && (accept_delay_trip)) {
					cout<<"pos dest: "<<sel_destination<<endl;
					stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
					
					passengers_departure_time_from_home[p] = best_departure_time_from_home;
					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
					action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


					//updating arrival and departure from stops
					int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
					//<<"xx "<<stops[v][pos_destination-1]<<endl;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
					slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
					slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

					prv_capacity = free_capacity[v][pos_destination-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

					//solution_cost += min_increase_length; //updates solution cost
					//saved_increase = min_increase_length;
					number_stops[v]++;

					
				} else {
					//solution_cost -= saved_increase;
					vehicle_assigned[p] = -1;
					number_stops[v]--;
					stops[v].erase(stops[v].begin() + pos_origin);
					action_passengers[v].erase(action_passengers[v].begin() + pos_origin);
					number_passengers_action[v].erase(number_passengers_action[v].begin() + pos_origin);
					//re-update further arrival and departure times
					
					arrival_time_stop[v].erase(arrival_time_stop[v].begin() + pos_origin);
					departure_time_stop[v].erase(departure_time_stop[v].begin() + pos_origin);
					slack_time[v].erase(slack_time[v].begin() + pos_origin);
					free_capacity[v].erase(free_capacity[v].begin() + pos_origin);
					departure_time_stop[v][0] = 0;

					//ultimately serve passenger with a 3rd party vehicle
					serve_passenger_third_party_vehicle(p);
				}
			}

		} else {

			min_increase_length = INT_MAX;
			repeated_station = false;
			flexibilize_lat_departure_time = true;
			sel_origin = -1;
			cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);

			if (sel_origin != -1) {
				//<<"sel o"<<sel_origin;
				stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
				
				//update passenger performing actions on the stops
				action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
				action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
				number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_origin, 1);

				//updating arrival and departure from stops
				//arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][pos_origin-1]+travel_time[depot][sel_origin]);
				cout<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
				
				int curr_dpt_time = earliest_departure[p]-travel_time[stops[v][0]][sel_origin];
				if (curr_dpt_time < current_time){
					curr_dpt_time = current_time;
				}
				departure_time_stop[v][0] = curr_dpt_time;
				arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
				departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_origin, departure_time_stop[v][0]+travel_time[stops[v][0]][sel_origin]);
				slack_time[v].insert(slack_time[v].begin() + pos_origin, 86400);

				prv_capacity = free_capacity[v][pos_origin-1];
				free_capacity[v].insert(free_capacity[v].begin() + pos_origin, prv_capacity-1);
				//free_capacity[v]--;

				//solution_cost += min_increase_length; //updates solution cost
				number_stops[v]++;

				vehicle_assigned[p] = v;

				for (int i=0; i<=number_stops[v];i++) {
					cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" ";
					for (int j=0; j<number_passengers_action[v][i];j++) 
						cout<<action_passengers[v][i][j]<<" ";
					cout<<"] ";
					cout<<"{"<<arrival_time_stop[v][i]<<"} ";
					cout<<"{"<<departure_time_stop[v][i]<<"} ";
					cout<<"|"<<slack_time[v][i]<<"|  ";
					cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
				
				}
				cout<<endl;

				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				flexibilize_arrival_time = false;
				cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
				//<<min_increase_length<<" "<<pos_destination<<endl;
				
				if (sel_destination != -1) {

					passengers_departure_time_from_home[p] = best_departure_time_from_home;
					cout<<"pos dest: "<<sel_destination<<endl;
					stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
					
					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
					action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


					//updating arrival and departure from stops
					int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
					//<<"xx "<<stops[v][pos_destination-1]<<endl;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
					slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
					slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

					prv_capacity = free_capacity[v][pos_destination-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

					odb_travel_time = travel_time[sel_origin][sel_destination];
					odb_distance = odb_travel_time*5.56;
					odb_distance = odb_distance/1000;
					odb_travel_time = odb_travel_time/60;
					odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
					if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
						odb_fare = ODB_minimum_fare[vehicle_type[v]];
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;
					number_stops[v]++;

					
				} else {
					//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
					//try to insert giving flexibility according to latest_arrivel_time
					min_increase_length = INT_MAX;
					repeated_station = false;
					sel_destination = -1;
					flexibilize_arrival_time = true;
					cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);

					if (delay[p] <= max_flex_delay)
						accept_delay_trip = true;
					else 
						accept_delay_trip = false;
					if ((sel_destination != -1) && (accept_delay_trip)) {
						cout<<"pos dest: "<<sel_destination<<endl;
						stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
						
						passengers_departure_time_from_home[p] = best_departure_time_from_home;
						//update passenger performing actions on the stops
						action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
						action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
						number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


						//updating arrival and departure from stops
						int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
						//<<"xx "<<stops[v][pos_destination-1]<<endl;
						arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
						departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
						slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
						slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

						prv_capacity = free_capacity[v][pos_destination-1];
						free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

						odb_travel_time = travel_time[sel_origin][sel_destination];
						odb_distance = odb_travel_time*5.56;
						odb_distance = odb_distance/1000;
						odb_travel_time = odb_travel_time/60;
						odb_fare = ((ODB_base_fare[vehicle_type[v]] + ODB_km_charge[vehicle_type[v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[v]]) + ODB_booking_fee[vehicle_type[v]]; 
						if (odb_fare < ODB_minimum_fare[vehicle_type[v]])
							odb_fare = ODB_minimum_fare[vehicle_type[v]];
						solution_cost += odb_fare; //updates solution cost
						cost_trip[p] = odb_fare;
						number_stops[v]++;

						
					} else {

						//solution_cost -= saved_increase;
						vehicle_assigned[p] = -1;
						number_stops[v]--;
						stops[v].erase(stops[v].begin() + pos_origin);
						action_passengers[v].erase(action_passengers[v].begin() + pos_origin);
						number_passengers_action[v].erase(number_passengers_action[v].begin() + pos_origin);
						//re-update further arrival and departure times
						
						arrival_time_stop[v].erase(arrival_time_stop[v].begin() + pos_origin);
						departure_time_stop[v].erase(departure_time_stop[v].begin() + pos_origin);
						slack_time[v].erase(slack_time[v].begin() + pos_origin);
						free_capacity[v].erase(free_capacity[v].begin() + pos_origin);
						departure_time_stop[v][0] = 0;

						//ultimately serve passenger with a 3rd party vehicle
						serve_passenger_third_party_vehicle(p);
					}
				}

			}


		}

		
		for (int i=0; i<=number_stops[v];i++) {
			cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" [ ";
			for (int j=0; j<number_passengers_action[v][i];j++) 
				cout<<action_passengers[v][i][j]<<" ";
			cout<<"]  ";

			cout<<"{"<<arrival_time_stop[v][i]<<"} ";
			cout<<"{"<<departure_time_stop[v][i]<<"} ";
			cout<<"|"<<slack_time[v][i]<<"|  ";
			cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
		}

			//<<stops[v][i]<<" ";
		cout<<endl;

		

	} else {

		//no empty vehicles. passenger will be inserted at the cheapest cost amongst all vehicles that is feasible
		bool not_feasible_insertion = true;
		int iterations = 0;
		bool tested_all_vehicles_once = false;
		
		while ((not_feasible_insertion) && (iterations < total_number_vehicles)) {
			
			int best_pos_origin, best_pos_destination;
			int best_min_increase_length;
			int best_sel_origin, best_sel_destination, best_v;
			bool best_repeated_station;
			
			best_min_increase_length = INT_MAX;
			best_v = -1;
			curr_number_insertions = 0;
			for (v=0; v<total_number_vehicles;v++) {
				//<<v<<" "<<free_capacity[v]<<endl;
				min_increase_length = INT_MAX;
				repeated_station = false;
				//(free_capacity[v] > 0) && 
				if (blocked_vehicles[v] == 0) {

					if (tested_all_vehicles_once)
						flexibilize_lat_departure_time = true;
					else
						flexibilize_lat_departure_time = false;
					
					cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);

					if (min_increase_length < best_min_increase_length) {
						best_min_increase_length = min_increase_length;
						best_sel_origin = sel_origin;
						best_pos_origin = pos_origin;
						best_v = v;
						best_repeated_station = repeated_station;
					}

					//cheapest_origin2(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);
					
				}
			}
			cout<<endl;

			//sort(insertions, insertions+curr_number_insertions, comparator);

			cout<<best_v<<endl;
			if (best_v != -1) {
			bool no_feasible_insertion2 = true;
			int iterations2 = 0;
			//if (curr_number_insertions > 0) {
			
				while ((no_feasible_insertion2) && (iterations2 < 1)) {

					//<<best_v<<"\n";
					int prv_arr_time_at_origin, prv_dpt_time_at_origin;

					

					if (not best_repeated_station) {
						stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
						number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

						//updating arrival and departure from stops
						
						//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
						
						arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
						
						if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
						else
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
						slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
						prv_capacity = free_capacity[best_v][best_pos_origin-1];
						free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);
					} else {
						//update passenger performing actions on the stops
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
						number_passengers_action[best_v][best_pos_origin]++;

						prv_arr_time_at_origin = arrival_time_stop[best_v][best_pos_origin];
						prv_dpt_time_at_origin = departure_time_stop[best_v][best_pos_origin];
						//no need to update the arrival time as it does not change
						//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
						if (earliest_departure[p] > departure_time_stop[best_v][best_pos_origin])
							departure_time_stop[best_v][best_pos_origin] = earliest_departure[p];
						//slack_time[best_v][best_pos_origin] //no need to update
						prv_capacity = free_capacity[best_v][best_pos_origin];
						free_capacity[best_v][best_pos_origin] = prv_capacity-1;
					}

					//update further arrival, departure times and slack times
					for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
						saved_arrival_time[i] = arrival_time_stop[best_v][i];
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						saved_departure_time[i] = departure_time_stop[best_v][i];
						if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						}
						saved_slack_time[i] = slack_time[best_v][i];
						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[i];
						//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
					}
					
					
					//solution_cost += best_min_increase_length;
					//saved_increase = best_min_increase_length;
					
					//free_capacity[best_v]--;

					vehicle_assigned[p] = best_v;

					//if (best_v == 24) {
					cout<<"flex: "<<flexibilize_lat_departure_time<<endl;
					for (int i=0; i<=number_stops[best_v];i++) {
						cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
						for (int j=0; j<number_passengers_action[best_v][i];j++) 
							cout<<action_passengers[best_v][i][j]<<" ";
						cout<<"]  ";

						cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
						cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
						cout<<"|"<<slack_time[best_v][i]<<"|  ";
						cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
					}
					//}
					cout<<endl;

					best_min_increase_length = INT_MAX;
					
					min_increase_length = INT_MAX;
					repeated_station = false;
					sel_destination = -1;
					if (tested_all_vehicles_once)
						flexibilize_arrival_time = true;
					else
						flexibilize_arrival_time = false;
					cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
				
					//<<"pos origin: "<<best_pos_origin<<" "<<endl;
					//<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
					
					if (sel_destination != -1) { 
					//if (curr_number_insertions2 > 0) {

						no_violation_capacity = true;
						for (int i=best_pos_origin+1;i<pos_destination;i++){

							free_capacity[best_v][i]--;

							if (free_capacity[best_v][i] < 0) {
								no_violation_capacity = false;
							}
							
						}
					}

					if (delay[p] <= max_flex_delay)
						accept_delay_trip = true;
					else 
						accept_delay_trip = false;

					if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip)) {
						
						passengers_departure_time_from_home[p] = best_departure_time_from_home;
						no_feasible_insertion2 = false;
						not_feasible_insertion = false;

						

						if (not repeated_station) {
							stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
							number_stops[best_v]++;

							//update passenger performing actions on the stops
							action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);

							//updating arrival and departure from stops
							int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
							arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
							slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

							prv_capacity = free_capacity[best_v][pos_destination-1];
							free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
						} else {

							//update passenger performing actions on the stops
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v][pos_destination]++;

							//according to the update when the origin is added, if the stop already exists then no need to change
							//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//arrival_time_stop[best_v][pos_destination] = arr_time;
							//departure_time_stop[best_v][pos_destination] = arr_time;
							int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < slack_time[best_v][pos_destination]) 
								slack_time[best_v][pos_destination] = slc_time;

							//prv_capacity = free_capacity[best_v][pos_destination];
							//free_capacity[best_v][pos_destination] = prv_capacity+1;

						}


						//update further arrival, departure times, and slack times
						for (int i=pos_destination+1; i<=number_stops[best_v];i++) {
							old_arr_time = arrival_time_stop[best_v][i];
							arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
							old_dpt_time = departure_time_stop[best_v][i];
							if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
								departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
							}
							slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						}

						//for (int i=1; i<pos_destination;i++){
						//	int min_slack_time = INT_MAX;
						//	for (int j=i+1; i<number_stops[best_v];i++){
						//		if (slack_time[best_v][j] < min_slack_time)
						//			min_slack_time = slack_time[best_v][j];
						//	}
						//	slack_time[best_v][i] = min_slack_time;
						//}

						int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[best_v]; i>=1; i--){
							if (slack_time[best_v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[best_v][i];
							} else {
								slack_time[best_v][i] = min_slack_time_so_far;
							}
						}
						
						//solution_cost += min_increase_length;
						odb_travel_time = travel_time[best_sel_origin][sel_destination];
						odb_distance = odb_travel_time*5.56;
						odb_distance = odb_distance/1000;
						odb_travel_time = odb_travel_time/60;
						odb_fare = ((ODB_base_fare[vehicle_type[best_v]] + ODB_km_charge[vehicle_type[best_v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[best_v]]) + ODB_booking_fee[vehicle_type[best_v]]; 
						if (odb_fare < ODB_minimum_fare[vehicle_type[best_v]])
							odb_fare = ODB_minimum_fare[vehicle_type[best_v]];
						solution_cost += odb_fare; //updates solution cost
						cost_trip[p] = odb_fare;
					} else {

						//no feasible insertion for destination
						cout<<"no feasible insertion for destination"<<endl;
						blocked_vehicles[best_v] = 1; //I suppose this is not necessary anymore. I'm already testing all vehicles with array insertions

						iterations2++;
						//solution_cost -= saved_increase;
						vehicle_assigned[p] = -1;
						if (sel_destination != -1) {
							for (int i=best_pos_origin+1;i<pos_destination;i++){

								free_capacity[best_v][i]++;
			
							}
						}

						if (number_passengers_action[best_v][best_pos_origin] == 1) {
							number_stops[best_v]--;
							stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
							action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
							number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
							

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}
							arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
							departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
							slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


							free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


						} else {

							//<<"heere"<<endl;
							//<<action_passengers[best_v][best_pos_origin][1]<<endl;
							action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
							
							number_passengers_action[best_v][best_pos_origin]--;

							arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
							departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
							free_capacity[best_v][best_pos_origin]++;

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}

						}
					}
					

					//if (best_v == 24) {
						//<<flexibilize_lat_departure_time<<endl;
	 				for (int i=0; i<=number_stops[best_v];i++) {
						cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
						for (int j=0; j<number_passengers_action[best_v][i];j++) 
							cout<<action_passengers[best_v][i][j]<<" ";
						cout<<"]  ";

						cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
						cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
						cout<<"|"<<slack_time[best_v][i]<<"|  ";
						cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
					}
					//}	
					cout<<endl;
				} //end while
			} else {
				iterations = total_number_vehicles;
			}

			//if (no_feasible_insertion2) {

				iterations++;
				//iterations = total_number_vehicles;
			//}
			if (iterations >= total_number_vehicles) {
				if (not tested_all_vehicles_once) {
					tested_all_vehicles_once = true;
					//iterations = 0;
					cout<<"serving 3rd party";
					serve_passenger_third_party_vehicle(p);
					//for (int i=0;i<total_number_vehicles;i++)
					//	blocked_vehicles[i] = 0;
				} else {
					//ultimately serve passenger with a 3rd party vehicle
					serve_passenger_third_party_vehicle(p);
				}
			}
		}
	}

	route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[i] = 0;
}*/

bool solution_validation(int p, int v){

	bool pick_up = true;
	bool valid_solution = true;
	int count = 0;
	for (int i=0; i<=number_stops[v];i++) {

		if (i == 0) {
			if (departure_time_stop[v][i] == 0) {
				cout<<"megra error 2"<<endl;
				for (int l=0; l<=number_stops[v];l++) {
					cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
					for (int m=0; m<number_passengers_action[v][l];m++) 
						cout<<action_passengers[v][l][m]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[v][l]<<"} ";
					cout<<"{"<<departure_time_stop[v][l]<<"} ";
					cout<<"|"<<slack_time[v][l]<<"|  ";
					cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
				}
				cout<<endl<<endl;
			}
		}

		if (i >= 1) {
			if (arrival_time_stop[v][i] < departure_time_stop[v][i-1]) {
				cout<<"megra erro"<<endl;
				
				for (int l=0; l<=number_stops[v];l++) {
					cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
					for (int m=0; m<number_passengers_action[v][l];m++) 
						cout<<action_passengers[v][l][m]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[v][l]<<"} ";
					cout<<"{"<<departure_time_stop[v][l]<<"} ";
					cout<<"|"<<slack_time[v][l]<<"|  ";
					cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
				}
				cout<<endl<<endl;
			}
		}

		for (int j=0; j<number_passengers_action[v][i];j++) {
			if (action_passengers[v][i][j] < 0){
				cout<<"MEGAERRORRRR3"<<endl;
				//<<action_passengers[v][i][j]<<endl;
				return false;
			}
			if (action_passengers[v][i][j] == p) {
				count++;
				if (pick_up) {
					if (departure_time_stop[v][i] < earliest_departure[p]) {
						cout<<departure_time_stop[v][i]<<endl;
						valid_solution = false;
					}
					pick_up = false;
				} else {

					if (delay[p] > 0) {
						if (arrival_time_stop[v][i] - delay[p] > latest_arrival[p]) {
							cout<<arrival_time_stop[v][i]<<" "<<delay[p]<<endl;
							valid_solution = false;
						}
					} else {
						if (arrival_time_stop[v][i] > latest_arrival[p]) {
							cout<<arrival_time_stop[v][i]<<" "<<delay[p]<<endl;
							valid_solution = false;
						}

					}
				}

			}
		}

	} 

	if (not valid_solution) {
		
		//cout<<travel_time[1174][3081]<<" "<<travel_time[3081][2142]<<" "<<travel_time[1174][2142]<<" "<<endl;
		cout <<"not valid for passenger: "<<p<<" "<<earliest_departure[p]<<" "<<latest_arrival[p]<<endl;
		
		for (int l=0; l<=number_stops[v];l++) {
			cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
			for (int m=0; m<number_passengers_action[v][l];m++) 
				cout<<action_passengers[v][l][m]<<" ";
			cout<<"]  ";

			cout<<"{"<<arrival_time_stop[v][l]<<"} ";
			cout<<"{"<<departure_time_stop[v][l]<<"} ";
			cout<<"|"<<slack_time[v][l]<<"|  ";
			cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
		}
		cout<<endl<<endl;
		return false;
	}
	if ((count < 2) || (count > 2)) {
		
		cout<<count<<" not valid for passenger (not picking up or dropping off): "<<p<<endl;
		return false;
	}

	return true;
}

int compute_difference_URT_by_fake_removing_passenger(int p){
	//<<"faking removing: "<<p<<endl;
	int v = vehicle_assigned[p];
	int greatest_ed;
	
	for (int i=0; i<number_stops[v]; i++){
			for (int j=0; j<number_passengers_action[v][i];j++) {
				departure_time_stop_temp[v][i] = -1;
				arrival_time_stop_temp[v][i] = -1;
		}
	}

	int prev_non_removed_position = 0;
	departure_time_stop_temp[v][0] = departure_time_stop[v][0];
	arrival_time_stop_temp[v][0] = arrival_time_stop[v][0];
	for (int i=0; i<number_stops[v]; i++){
			for (int j=0; j<number_passengers_action[v][i];j++) {
				//<<"actp "<<action_passengers[v][i][j]<<endl;

				if (action_passengers[v][i][j] == p) {

					if (number_passengers_action[v][i] == 1) {
						//MEANS THAT THE STOP WILL BE REMOVED

						if (prev_non_removed_position == 0) {
							//departure_time_stop_temp[v][0] = current_time;
							//arrival_time_stop_temp[v][i+1] = current_time+travel_time[stops[v][prev_non_removed_position]][stops[v][i+1]];
							//departure_time_stop_temp[v][i+1] = arrival_time_stop_temp[v][i+1];
							departure_time_stop_temp[v][0] = departure_time_stop[v][0];
							arrival_time_stop_temp[v][i+1] = arrival_time_stop[v][i+1];
							departure_time_stop_temp[v][i+1] = arrival_time_stop_temp[v][i+1];
						} else {
							arrival_time_stop_temp[v][i+1] = departure_time_stop_temp[v][prev_non_removed_position]+travel_time[stops[v][prev_non_removed_position]][stops[v][i+1]];
							departure_time_stop_temp[v][i+1] = arrival_time_stop_temp[v][i+1];
						}
						//<<"tst: "<<stops[v][prev_non_removed_position]<<" "<<stops[v][i+1]<<" "<<arrival_time_stop_temp[v][i+1]<<" "<<departure_time_stop_temp[v][prev_non_removed_position]<<" "<<travel_time[stops[v][prev_non_removed_position]][stops[v][i+1]]<<endl;
						greatest_ed = INT_MIN;
						//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
						for (int l=0; l<number_passengers_action[v][i+1];l++) {
							//<<"ed fake: "<<action_passengers[v][i+1][l]<<" "<<earliest_departure[action_passengers[v][i+1][l]]<<endl;
							if (earliest_departure[action_passengers[v][i+1][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[v][i+1][l]];
						}

						if (greatest_ed > departure_time_stop_temp[v][i+1])
							departure_time_stop_temp[v][i+1] = greatest_ed;

						//re-update further arrival and departure times
						for (int k=i+2; k<=number_stops[v];k++) {

							bool p_is_not_at_stop = true;
							for (int l=0; l<number_passengers_action[v][k];l++) {
								if (action_passengers[v][k][l] == p)
									p_is_not_at_stop = false;

							}

							//if (p_is_not_at_stop) {
								//old_arrival_time = arrival_time_stop[v][k];
								arrival_time_stop_temp[v][k] = departure_time_stop_temp[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
								departure_time_stop_temp[v][k] = arrival_time_stop_temp[v][k];

								greatest_ed = INT_MIN;
								//<<"tst: "<<stops[v][k-1]<<" "<<stops[v][k]<<" "<<arrival_time_stop_temp[v][k]<<" "<<departure_time_stop_temp[v][k]<<" "<<travel_time[stops[v][k-1]][stops[v][k]]<<endl;
								//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
								for (int l=0; l<number_passengers_action[v][k];l++) {
									//<<"ed fake: "<<action_passengers[v][k][l]<<" "<<earliest_departure[action_passengers[v][k][l]]<<endl;
									if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
										greatest_ed = earliest_departure[action_passengers[v][k][l]];
								}

								if (greatest_ed > departure_time_stop_temp[v][k])
									departure_time_stop_temp[v][k] = greatest_ed;
							//}

							
						}

						if (departure_time_stop_temp[v][i] - arrival_time_stop_temp[v][i] <= 3) {
							departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i];
						}

						prev_non_removed_position = i-1;
					} else {

						if (departure_time_stop_temp[v][i] == -1)
							departure_time_stop_temp[v][i] = departure_time_stop[v][i];
						if (arrival_time_stop_temp[v][i] == -1)
							arrival_time_stop_temp[v][i] = arrival_time_stop[v][i];

						greatest_ed = INT_MIN;
						for (int l=0; l<number_passengers_action[v][i];l++) {
							if (action_passengers[v][i][l] != p) {
								if (earliest_departure[action_passengers[v][i][l]] > greatest_ed)
									greatest_ed = earliest_departure[action_passengers[v][i][l]];
							}
						}

						//departure time only changes if the greatest earliest departure < departure time
						//it means that by removing the passenger the bus can leave earlier
						if (greatest_ed < departure_time_stop[v][i]) {
							if (arrival_time_stop_temp[v][i] > greatest_ed){
								departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i];
							} else {
								departure_time_stop_temp[v][i] = greatest_ed;
							}
						}

						//re-update further arrival and departure times
						for (int k=i+1; k<=number_stops[v];k++) {
							//old_arrival_time = arrival_time_stop[v][k];
							arrival_time_stop_temp[v][k] = departure_time_stop_temp[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
							departure_time_stop_temp[v][k] = arrival_time_stop_temp[v][k];

							greatest_ed = INT_MIN;
							//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
							for (int l=0; l<number_passengers_action[v][k];l++) {
								if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
									greatest_ed = earliest_departure[action_passengers[v][k][l]];
							}

							if (greatest_ed > departure_time_stop_temp[v][k])
								departure_time_stop_temp[v][k] = greatest_ed;

							
						}

						prev_non_removed_position = i;

						if (departure_time_stop_temp[v][i] - arrival_time_stop_temp[v][i] <= 3) {
							departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i];
						}

					}

				} else {
					prev_non_removed_position = i;
					if (departure_time_stop_temp[v][i] == -1)
						departure_time_stop_temp[v][i] = departure_time_stop[v][i];
					if (arrival_time_stop_temp[v][i] == -1)
						arrival_time_stop_temp[v][i] = arrival_time_stop[v][i];
				}
		}
	}	

	//<<"faking times: "<<endl;
	/*for (int i=0; i<number_stops[v]; i++){

		cout<<stops[v][i]<<" ";
		cout<<arrival_time_stop_temp[v][i]<<" "<<departure_time_stop_temp[v][i]<<endl;
	}
	cout<<endl;*/


	int current_user_ride_time, difference;
	difference = 0;
	bool leave_loop = false;
	for (int i=0; i<=number_stops[v];i++) {
		for (int j=0; j<number_passengers_action[v][i];j++) {
			int save_p = action_passengers[v][i][j];
			if (save_p != p) {
				for (int k=i+1; k<=number_stops[v];k++) {
					leave_loop = false;
					for (int l=0; l<number_passengers_action[v][k];l++) {
						if (action_passengers[v][k][l] == save_p){
							current_user_ride_time = arrival_time_stop_temp[v][k] - departure_time_stop_temp[v][i];
							user_ride_time_temp[save_p] = current_user_ride_time;
							if (current_user_ride_time != user_ride_time[save_p]) {
								difference += (user_ride_time[save_p] - current_user_ride_time);
								//<<"COMP DIFFERENCE rem "<<save_p<<" "<<user_ride_time[save_p] - current_user_ride_time<<endl;
								//new_total_user_ride_time += difference;
								//user_ride_time[save_p] = current_user_ride_time;
								l = number_passengers_action[v][k]+1;
								//k = number_stops[best_v]+2; //leave loop
								leave_loop = true;
							}
						}
					}
					if (leave_loop)
						k = number_stops[v]+2;
				}
			}	
		} 
	}

	//<<"TOTAL DIFFERENCE "<<difference<<" "<<user_ride_time[p]<<endl;
	//difference += user_ride_time[p];
	return difference;
}

int compute_difference_URT_by_fake_adding_passenger(int p, int v, int sel_destination, int pos_destination, int repeated_station){
	
	int greatest_ed;
	//<<"faking adding: "<<p<<endl;


	for (int i=0; i<pos_destination;i++){

		arrival_time_stop_temp[v][i] = arrival_time_stop[v][i];
		departure_time_stop_temp[v][i] = departure_time_stop[v][i];  
		//cout<<stops[v][i]<<" "<<arrival_time_stop_temp[v][i]<<" "<<departure_time_stop_temp[v][i]<<endl;
	}


	for (int i = 1; i < pos_destination; i++) {
		if (arrival_time_stop_temp[v][i] != departure_time_stop_temp[v][i]) {

			int greatest_ed = INT_MIN;
			//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
			for (int m=0; m<number_passengers_action[v][i];m++) {
				if (earliest_departure[action_passengers[v][i][m]] > greatest_ed)
					greatest_ed = earliest_departure[action_passengers[v][i][m]];
			}

			if (greatest_ed != departure_time_stop_temp[v][i]) {
				departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i]; 

				arrival_time_stop_temp[v][i] = departure_time_stop_temp[v][i-1]+travel_time[stops[v][i-1]][stops[v][i]];
				departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i];

				int greatest_ed = INT_MIN;
				//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
				for (int l=0; l<number_passengers_action[v][i];l++) {
					if (earliest_departure[action_passengers[v][i][l]] > greatest_ed)
						greatest_ed = earliest_departure[action_passengers[v][i][l]];
				}

				if (greatest_ed > departure_time_stop_temp[v][i])
					departure_time_stop_temp[v][i] = greatest_ed;
			}
			//cout<<"diff "<<" "<<arrival_time_stop_temp[v][i]<<" "<<departure_time_stop_temp[v][i]<<endl;
		}
	}

	//<<"hierx"<<endl;

	arrival_time_stop_temp[v][pos_destination] = departure_time_stop_temp[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
	//departure_time_stop_temp[v][pos_destination] = arrival_time_stop_temp[v][pos_destination];
	
	//if (arrival_time_stop_temp[v][pos_destination] > departure_time_stop[v][pos_destination-1]) {
	departure_time_stop_temp[v][pos_destination] = arrival_time_stop_temp[v][pos_destination]; 
	//} else {
		//departure_time_stop_temp[v][pos_destination] = departure_time_stop[v][pos_destination-1];
	//}

	//if (departure_time_stop_temp[v][pos_destination] - arrival_time_stop_temp[v][pos_destination] <= 3) {
	//	departure_time_stop_temp[v][pos_destination] = arrival_time_stop_temp[v][pos_destination];
	//}

	//<<"hierx2"<<endl;
	greatest_ed = INT_MIN;
	if (repeated_station) {

		//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
		for (int l=0; l<number_passengers_action[v][pos_destination-1];l++) {
			if (earliest_departure[action_passengers[v][pos_destination-1][l]] > greatest_ed)
				greatest_ed = earliest_departure[action_passengers[v][pos_destination-1][l]];
		}

	}
	//<<"hier3"<<endl;
	if (earliest_departure[p] > greatest_ed)
		greatest_ed = earliest_departure[p];

	if (greatest_ed > departure_time_stop_temp[v][pos_destination])
		departure_time_stop_temp[v][pos_destination] = greatest_ed;

	//cout<<stops[v][pos_destination]<<"dp "<<arrival_time_stop_temp[v][pos_destination]<<" "<<departure_time_stop_temp[v][pos_destination]<<endl;

	if (repeated_station) {

		//cout<<" "<<departure_time_stop_temp[v][pos_destination]<<" "<<travel_time[sel_destination][stops[v][pos_destination+1]]<<" "<<sel_destination<<" "<<stops[v][pos_destination+1]<<endl;
		arrival_time_stop_temp[v][pos_destination+1] = departure_time_stop_temp[v][pos_destination]+travel_time[sel_destination][stops[v][pos_destination+1]];
		departure_time_stop_temp[v][pos_destination+1] = arrival_time_stop_temp[v][pos_destination+1];
	} else {
		//<<number_passengers_action[v][pos_destination+1]<<" "<<pos_destination+1<<" "<<number_stops[v]<<endl;
		arrival_time_stop_temp[v][pos_destination+1] = departure_time_stop_temp[v][pos_destination]+travel_time[sel_destination][stops[v][pos_destination]];
		//departure_time_stop_temp[v][pos_destination+1] = arrival_time_stop_temp[v][pos_destination+1];
		//if (arrival_time_stop_temp[v][pos_destination+1] >= departure_time_stop[v][pos_destination]) {
		departure_time_stop_temp[v][pos_destination+1] = arrival_time_stop_temp[v][pos_destination+1]; 
	}
	//} else {
	//	departure_time_stop_temp[v][pos_destination+1] = departure_time_stop[v][pos_destination];
	//	cout<<"hiierx2 "<<departure_time_stop[v][pos_destination]<<endl;
	//}

	//if (departure_time_stop_temp[v][pos_destination+1] - arrival_time_stop_temp[v][pos_destination+1] <= 3) {
	//	departure_time_stop_temp[v][pos_destination+1] = arrival_time_stop_temp[v][pos_destination+1];
	//}

	//<<"fad: "<<stops[v][pos_destination]<<" "<<arrival_time_stop_temp[v][pos_destination+1]<<" "<<departure_time_stop_temp[v][pos_destination+1]<<endl;
	//<<"hier4"<<endl;
	
	//cout<<stops[v][pos_destination+1]<<"ps1 "<<arrival_time_stop_temp[v][pos_destination+1]<<" "<<departure_time_stop_temp[v][pos_destination+1]<<endl;

	greatest_ed = INT_MIN;
	//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
	//<<number_passengers_action[v][pos_destination+1]<<" "<<pos_destination+1<<" "<<number_stops[v]<<endl;
	if (not repeated_station){
		for (int l=0; l<number_passengers_action[v][pos_destination];l++) {
			if (earliest_departure[action_passengers[v][pos_destination][l]] > greatest_ed)
				greatest_ed = earliest_departure[action_passengers[v][pos_destination][l]];
		}
		//<<"hier4.2"<<endl;
		if (greatest_ed > departure_time_stop_temp[v][pos_destination+1]) {
			//cout<<"hiierx3 "<<greatest_ed<<endl;
			departure_time_stop_temp[v][pos_destination+1] = greatest_ed;
		}
	}
	//<<"hier5"<<endl;

	//cout<<stops[v][pos_destination+1]<<"ps2 "<<arrival_time_stop_temp[v][pos_destination+1]<<" "<<departure_time_stop_temp[v][pos_destination+1]<<endl;
	
	for (int i=pos_destination+2; i<=number_stops[v];i++){

		if (repeated_station) {
			arrival_time_stop_temp[v][i] = departure_time_stop_temp[v][i-1]+travel_time[stops[v][i-1]][stops[v][i]];
			departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i]; 

			greatest_ed = INT_MIN;
			//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
			for (int l=0; l<number_passengers_action[v][i];l++) {
				if (earliest_departure[action_passengers[v][i][l]] > greatest_ed)
					greatest_ed = earliest_departure[action_passengers[v][i][l]];
			}

			if (greatest_ed > departure_time_stop_temp[v][i])
				departure_time_stop_temp[v][i] = greatest_ed; 

		} else {
			arrival_time_stop_temp[v][i] = departure_time_stop_temp[v][i-1]+travel_time[stops[v][i-2]][stops[v][i-1]];
			//if (arrival_time_stop_temp[v][i] > departure_time_stop[v][i-1]) {
			departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i]; 

			greatest_ed = INT_MIN;
			//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
			for (int l=0; l<number_passengers_action[v][i-1];l++) {
				if (earliest_departure[action_passengers[v][i-1][l]] > greatest_ed)
					greatest_ed = earliest_departure[action_passengers[v][i-1][l]];
			}

			if (greatest_ed > departure_time_stop_temp[v][i])
				departure_time_stop_temp[v][i] = greatest_ed; 
		}
		//} else {
			//departure_time_stop_temp[v][i] = departure_time_stop[v][i-1];
		//}

		/*if ((departure_time_stop_temp[v][i] - arrival_time_stop_temp[v][i] == 1) || (departure_time_stop_temp[v][i] - arrival_time_stop_temp[v][i] == 2)) {
			departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i];
		}*/
		//<<"cfad: "<<departure_time_stop_temp[v][i-1]<<" "<<travel_time[stops[v][i-1]][stops[v][i]]<<endl;

		

		//if (departure_time_stop_temp[v][i] - arrival_time_stop_temp[v][i] <= 3) {
		//	departure_time_stop_temp[v][i] = arrival_time_stop_temp[v][i];
		//}

		//cout<<stops[v][i]<<" "<<arrival_time_stop_temp[v][i]<<" "<<departure_time_stop_temp[v][i]<<endl;
	}
	//<<"hier6"<<endl;

	//compute here difference in URT
	int current_user_ride_time, difference;
	bool leave_loop = false;
	difference = 0;
	for (int i=0; i<=number_stops[v];i++) {
		for (int j=0; j<number_passengers_action[v][i];j++) {
			int save_p = action_passengers[v][i][j];
			//cout<<save_p<<endl;
			if (save_p == p){
				user_ride_time_temp[save_p] = arrival_time_stop_temp[v][pos_destination] - departure_time_stop_temp[v][i];
				difference += user_ride_time[save_p] - user_ride_time_temp[save_p];
				//cout<<"COMP DIFFERENCE add "<<save_p<<" "<<user_ride_time[save_p] - user_ride_time_temp[save_p]<<endl;
			} else {
				for (int k=i+1; k<=number_stops[v];k++) {
					leave_loop = false;
					for (int l=0; l<number_passengers_action[v][k];l++) {
						//cout<<action_passengers[v][k][l]<<endl;
						if (action_passengers[v][k][l] == save_p){

							if ((k >= pos_destination) && (not repeated_station))  {
								k = k + 1;
							} 

							if ((i >= pos_destination) && (not repeated_station)) {
								i = i + 1;
							}

						
							current_user_ride_time = arrival_time_stop_temp[v][k] - departure_time_stop_temp[v][i];
							
							user_ride_time_temp[save_p] = current_user_ride_time;
							
							//cout<<"fad2: "<<save_p<<"; "<<current_user_ride_time<<" "<<arrival_time_stop_temp[v][k]<<" "<<departure_time_stop_temp[v][i]<<endl;
							if (current_user_ride_time != user_ride_time[save_p]) {
								difference += user_ride_time[save_p] - current_user_ride_time;
								//<<"COMP DIFFERENCE add "<<save_p<<" "<<user_ride_time[save_p] - current_user_ride_time<<endl;
								//new_total_user_ride_time -= difference;
								//user_ride_time[save_p] = current_user_ride_time;
								l = number_passengers_action[v][k]+1;
								//k = number_stops[best_v]+2; //leave loop
								leave_loop = true;
							}

							if ((i >= pos_destination) && (not repeated_station)) {
								i = i - 1;
							}
						}
					}
					if (leave_loop)
						k = number_stops[v]+2;
				}
			}	
		} 
	}

	return difference;
}

/*void re_insertion2(int p, bool &accept_relocate_trip, double &temperature, int &type_move){

	int odb_travel_time, odb_distance, v;
	int pos_origin, pos_destination, sel_origin, sel_destination, min_increase_length, veh, ttcsd;
	bool repeated_station, no_violation_capacity, flexibilize_arrival_time, flexibilize_lat_departure_time, accept_delay_trip;
	int old_dpt_time, old_arr_time, prv_capacity, saved_increase;

	int delta; 
	double x;

	ttcsd = INT_MAX;

	bool not_feasible_insertion = true;
	int iterations = 1; //starts with one because you dont consider the vehicle the passenger was inserted previously
	bool tested_all_vehicles_once = false;
	int best_distance_depot = INT_MAX;
	int capacity_best_empty_vehicle = INT_MIN;
	int best_empty_vehicle, best_depot;
	best_empty_vehicle = -1;
	if (filtered_vehicles.size()>0)
		filtered_vehicles.clear();

	//check for closest depot that has an empty vehicle
	//second check for highest capacitated vehicle from that depot
	for (int i = 0; i < number_depots; i++) {

		ttcsd = INT_MAX;
		for (int k=0;k<number_stops_origin[p];k++) {
			if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
				ttcsd = travel_time[stops_origin[p][k]][depot[i]];
			}
		}

		//if ((best_distance_depot == 1) || (ttcsd <= best_distance_depot)) {
		if (ttcsd <= best_distance_depot) {
			capacity_best_empty_vehicle = INT_MIN;
			for (int j = 0; j < number_vehicles_at_depot[i]; j++) {

				veh = vehicles_at_depot[i][j];
				if (free_capacity[veh].size() == 2){ //means that vehicle is empty
					if (free_capacity[veh][0] >= capacity_best_empty_vehicle) {
						best_empty_vehicle = veh;
						best_depot = depot[i];
						capacity_best_empty_vehicle = free_capacity[veh][0];
						best_distance_depot = ttcsd;

					}
				}
			}
		}
	}

	int best_distance_location = best_distance_depot;

	for (int v=0;v<total_number_vehicles;v++){
		if (free_capacity[v].size() > 2) { //the vehicle has passengers assigned but might be empty at some point in time 
			for (int i=number_stops[v]-1; i<number_stops[v];i++){
				if (free_capacity[v][i] == free_capacity[v][0]) { //means that vehicle is empty

					ttcsd = INT_MAX;
					for (int k=0;k<number_stops_origin[p];k++) {
						if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
							ttcsd = travel_time[stops[v][i]][stops_origin[p][k]];
						}
					}

					if (ttcsd <= best_distance_location) {

						if (departure_time_stop[v][i] + ttcsd <= latest_departure[p]) { //important to verify if is feasible according to time window constraints

							//<<"HIERRR SA"<<endl;
							best_empty_vehicle = v;
							best_distance_location = ttcsd;
						}

					}
				}
			}
		}
	}

	if (best_empty_vehicle != -1)
		filtered_vehicles.push_back(best_empty_vehicle);

	//if (filtered_vehicles.size() == 0)
	filter_vehicles(p);

	cout<<"filtered_vehicles SIZE "<<filtered_vehicles.size()<<endl;
	while ((not_feasible_insertion) && (iterations < filtered_vehicles.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		curr_number_insertions = 0;
		
		for (int vf=0; vf<filtered_vehicles.size();vf++) {
			int v = filtered_vehicles[vf];
			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			if (blocked_vehicles[v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				
				cheapest_origin2(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);
				
			}
		}
		cout<<endl;

		cout<<"curr insert1: " << curr_number_insertions<<endl;
		sort(insertions, insertions+curr_number_insertions, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions > 0) {
		
			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions)) {

				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				best_min_increase_length = insertions[iterations2].increase_length;
				best_sel_origin = insertions[iterations2].sel_station;
				best_pos_origin = insertions[iterations2].pos_station;
				best_v = insertions[iterations2].v;
				best_repeated_station = insertions[iterations2].repeated_station;

				if (not best_repeated_station) {
					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

					//updating arrival and departure from stops
					
					//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
					
					arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
					
					if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
					else
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
					slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
					prv_capacity = free_capacity[best_v][best_pos_origin-1];
					free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);
				} else {
					//update passenger performing actions on the stops
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v][best_pos_origin]++;

					prv_arr_time_at_origin = arrival_time_stop[best_v][best_pos_origin];
					prv_dpt_time_at_origin = departure_time_stop[best_v][best_pos_origin];
					//no need to update the arrival time as it does not change
					//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
					if (earliest_departure[p] > departure_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v][best_pos_origin] = earliest_departure[p];
					//slack_time[best_v][best_pos_origin] //no need to update
					prv_capacity = free_capacity[best_v][best_pos_origin];
					free_capacity[best_v][best_pos_origin] = prv_capacity-1;
				}

				//update further arrival, departure times and slack times
				for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
					saved_arrival_time[i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					}
					saved_slack_time[i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				

				//if (best_v == 24) {
				cout<<"flex: "<<flexibilize_lat_departure_time<<endl;
				for (int i=0; i<=number_stops[best_v];i++) {
					cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
					for (int j=0; j<number_passengers_action[best_v][i];j++) 
						cout<<action_passengers[best_v][i][j]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
					cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
					cout<<"|"<<slack_time[best_v][i]<<"|  ";
					cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
				}
				//}
				cout<<endl;

				best_min_increase_length = INT_MAX;
				
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				if (tested_all_vehicles_once)
					flexibilize_arrival_time = true;
				else
					flexibilize_arrival_time = false;
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
			
				//<<"pos origin: "<<best_pos_origin<<" "<<endl;
				//<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
				
				if (sel_destination != -1) { 

					no_violation_capacity = true;
					for (int i=best_pos_origin+1;i<pos_destination;i++){

						free_capacity[best_v][i]--;

						if (free_capacity[best_v][i] < 0) {
							no_violation_capacity = false;
						}
						
					}
				}

				if (delay[p] <= max_flex_delay)
					accept_delay_trip = true;
				else 
					accept_delay_trip = false;

				

				//<<"heere6"<<endl;
				int new_total_user_ride_time = total_user_ride_time;
				//updates solution cost
				//user_ride_time[p] = travel_time[sel_origin][sel_destination];
				//total_user_ride_time += user_ride_time;
				//int DELTA = total_user_ride_time - new_total_user_ride_time;

				int DELTA;
				int diff_remov_passenger = compute_difference_URT_by_fake_removing_passenger(p);
				int diff_add_passenger = 0;
				if ((sel_destination != -1) && (no_violation_capacity))
					diff_add_passenger = compute_difference_URT_by_fake_adding_passenger(p, best_v, sel_destination, pos_destination, repeated_station);
				DELTA = diff_remov_passenger + diff_add_passenger;

				if (diff_remov_passenger < 0)
					cout<<"NEGATIVE ERROR"<<endl;
				DELTA += diff_remov_passenger;
				accept_relocate_trip = false;
				if (DELTA > 0)
					accept_relocate_trip = true;

				printf("DELTA:%d \n", DELTA);

				
				if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip) && (accept_relocate_trip)) {

					passengers_departure_time_from_home[p] = insertions[iterations2].passengers_departure_time_from_home;
					type_move = 1;
					vehicle_assigned[p] = best_v;

					cout<<"feasible and reduce fare"<<endl;
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;

					cout<<"hieerxx"<<endl;
					if (not repeated_station) {

						cout<<"hieerxx1"<<endl;
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);

						//updating arrival and departure from stops
						int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
						arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
						slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

						prv_capacity = free_capacity[best_v][pos_destination-1];
						free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
					} else {

						cout<<"hieerxx2"<<endl;
						//update passenger performing actions on the stops
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v][pos_destination]++;

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						if (slc_time < slack_time[best_v][pos_destination]) 
							slack_time[best_v][pos_destination] = slc_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}


					//update further arrival, departure times, and slack times
					for (int i=pos_destination+1; i<=number_stops[best_v];i++) {
						old_arr_time = arrival_time_stop[best_v][i];
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						old_dpt_time = departure_time_stop[best_v][i];
						if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						}
						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
					}

					//for (int i=1; i<pos_destination;i++){
					//	int min_slack_time = INT_MAX;
					//	for (int j=i+1; i<number_stops[best_v];i++){
					//		if (slack_time[best_v][j] < min_slack_time)
					//			min_slack_time = slack_time[best_v][j];
					//	}
					//	slack_time[best_v][i] = min_slack_time;
					//}

					int min_slack_time_so_far = INT_MAX;	
					for (int i=number_stops[best_v]; i>=1; i--){
						if (slack_time[best_v][i] < min_slack_time_so_far) {
							min_slack_time_so_far = slack_time[best_v][i];
						} else {
							slack_time[best_v][i] = min_slack_time_so_far;
						}
					}
					
					//updates solution cost
					//user_ride_time[p] = travel_time[sel_origin][sel_destination];
					//total_user_ride_time += user_ride_time;
					int current_user_ride_time, difference;
					
					bool leave_loop = false;
					for (int i=0; i<=number_stops[best_v];i++) {
						for (int j=0; j<number_passengers_action[best_v][i];j++) {
							int save_p = action_passengers[best_v][i][j];
							for (int k=i+1; k<=number_stops[best_v];k++) {
								leave_loop = false;
								for (int l=0; l<number_passengers_action[best_v][k];l++) {
									if (action_passengers[best_v][k][l] == save_p){
										current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
										if (current_user_ride_time != user_ride_time[save_p]) {
											difference = current_user_ride_time -  user_ride_time[save_p];
											cout<<"difference: "<<difference<<endl;
											total_user_ride_time += difference;
											cout<<"total_user_ride_time: "<<total_user_ride_time<<endl;
											user_ride_time[save_p] = current_user_ride_time;
											l = number_passengers_action[best_v][k]+1;
											//k = number_stops[best_v]+2; //leave loop
											leave_loop = true;
										}
									}
								}
								if (leave_loop)
									k = number_stops[best_v]+2;
							}	
						} 
					}
					
				} else {

					accept_delay_trip = false; //making sure non improving moves are not accepted. change this later
					if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip)) {

						x = (double)rand() / (double)RAND_MAX;

						delta = DELTA - cost_trip[p];
						if (x < exp(-delta / (temperature))) {
							//non improving move is accepted according to SA
							accept_relocate_trip = true;
						} else {
							accept_relocate_trip = false;
						}
					} else {
						accept_relocate_trip = false;
					} 

					if (accept_relocate_trip) {

						passengers_departure_time_from_home[p] = insertions[iterations2].passengers_departure_time_from_home;
						type_move = 2;
						vehicle_assigned[p] = best_v;

						cout<<"feasible and accepted according to SA (its NON IMPROVING)"<<endl;
						no_feasible_insertion2 = false;
						not_feasible_insertion = false;


						if (not repeated_station) {

							//<<"vehicle "<<best_v<<endl;
							stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
							
							number_stops[best_v]++;


							//update passenger performing actions on the stops
							action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);


							//updating arrival and departure from stops
							int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
							arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
							slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);


							prv_capacity = free_capacity[best_v][pos_destination-1];
							free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
							
						} else {
							cout<<"hereeeeLSE"<<endl;
							//update passenger performing actions on the stops
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v][pos_destination]++;

							//according to the update when the origin is added, if the stop already exists then no need to change
							//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//arrival_time_stop[best_v][pos_destination] = arr_time;
							//departure_time_stop[best_v][pos_destination] = arr_time;
							int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < slack_time[best_v][pos_destination]) 
								slack_time[best_v][pos_destination] = slc_time;

							//prv_capacity = free_capacity[best_v][pos_destination];
							//free_capacity[best_v][pos_destination] = prv_capacity+1;

						}


						//update further arrival, departure times, and slack times
						for (int i=pos_destination+1; i<=number_stops[best_v];i++) {
							old_arr_time = arrival_time_stop[best_v][i];
							arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
							old_dpt_time = departure_time_stop[best_v][i];
							if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
								departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
							}
							slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						}

						//for (int i=1; i<pos_destination;i++){
						//	int min_slack_time = INT_MAX;
						//	for (int j=i+1; i<number_stops[best_v];i++){
						//		if (slack_time[best_v][j] < min_slack_time)
						//			min_slack_time = slack_time[best_v][j];
						//	}
						//	slack_time[best_v][i] = min_slack_time;
						//}

						int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[best_v]; i>=1; i--){
							if (slack_time[best_v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[best_v][i];
							} else {
								slack_time[best_v][i] = min_slack_time_so_far;
							}
						}
						
						
						//updates solution cost
						//user_ride_time[p] = travel_time[sel_origin][sel_destination];
						//total_user_ride_time += user_ride_time;
						int current_user_ride_time, difference;
						bool leave_loop = false;
						for (int i=0; i<=number_stops[best_v];i++) {
							for (int j=0; j<number_passengers_action[best_v][i];j++) {
								int save_p = action_passengers[best_v][i][j];
								for (int k=i+1; k<=number_stops[best_v];k++) {
									leave_loop = false;
									for (int l=0; l<number_passengers_action[best_v][k];l++) {
										cout<<number_passengers_action[best_v][k]<<endl;
										if (action_passengers[best_v][k][l] == save_p){
											current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time - user_ride_time[save_p];
												//<<"difference: "<<difference<<endl;
												total_user_ride_time += difference;
												cout<<"total_user_ride_time: "<<total_user_ride_time<<endl;
												user_ride_time[save_p] = current_user_ride_time;
												l = number_passengers_action[best_v][k]+1;
												//k = number_stops[best_v]+2; //leave loop
												leave_loop = true;
											}
										}
									}
									if (leave_loop)
										k = number_stops[best_v]+2;

								}	
							} 
						}
						//<<"heeere re"<<endl;

					} else {

						type_move = 3;
						//no feasible insertion for destination
						accept_relocate_trip = false;
						
						cout<<"no feasible insertion for destination"<<endl;
						//blocked_vehicles[best_v] = 1; //I suppose this is not necessary anymore. I'm already testing all vehicles with array insertions

						iterations2++;
						//solution_cost -= saved_increase;
						//vehicle_assigned[p] = -1;
						if (sel_destination != -1) {
							for (int i=best_pos_origin+1;i<pos_destination;i++){

								free_capacity[best_v][i]++;
			
							}
						}

						if (number_passengers_action[best_v][best_pos_origin] == 1) {
							number_stops[best_v]--;
							stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
							action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
							number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
							

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}
							arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
							departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
							slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


							free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


						} else {

							//<<"heere"<<endl;
							//<<action_passengers[best_v][best_pos_origin][1]<<endl;
							action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
							
							number_passengers_action[best_v][best_pos_origin]--;

							arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
							departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
							free_capacity[best_v][best_pos_origin]++;

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}

						}
					}

				}
				

				//if (best_v == 24) {
					//<<flexibilize_lat_departure_time<<endl;
 				for (int i=0; i<=number_stops[best_v];i++) {
					cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
					for (int j=0; j<number_passengers_action[best_v][i];j++) 
						cout<<action_passengers[best_v][i][j]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
					cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
					cout<<"|"<<slack_time[best_v][i]<<"|  ";
					cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
				}
				//}	
				cout<<endl;
			} //end while
		} else {
			iterations = filtered_vehicles.size()+1;
		}

		if (no_feasible_insertion2) {

			type_move = 3;
			//iterations++;
			iterations = filtered_vehicles.size()+1;
		}
		if (iterations >= filtered_vehicles.size()) {
			if (not tested_all_vehicles_once) {
				tested_all_vehicles_once = true;
				//iterations = 0;
				//<<"serving 3rd party";
				//serve_passenger_third_party_vehicle(p);
				//for (int i=0;i<total_number_vehicles;i++)
				//	blocked_vehicles[i] = 0;
			} else {
				//ultimately serve passenger with a 3rd party vehicle
				//<<"serving 3rd party";
				//serve_passenger_third_party_vehicle(p);
			}
		}
	}

	route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[i] = 0;
}*/

/*void relocate_all_passengers_vehicle(int v, double &temperature, int &type_move){


	bool accept_relocate_trip = false;
	//compute the removal of passenger p from the solution
	int decrease_solution = 0;
	int count=0;
	int greatest_ed = INT_MIN;
	int old_arrival_time;
	int p;
	int counter = 0;

	cout<<"NEW RELOCATE TO EMPTY A VEHICLE"<<endl;
	cout<<"vnr: "<<v<<endl;
	for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}

	
	for (int kk=0; kk<passengers_at_vehicle[v].size();kk++){
		p = passengers_at_vehicle[v][kk]; //passenger that will try to relocate

		cout<<"passengerx "<<p<<endl;
		cout<<"before"<<endl;
		cout<<"number stops "<<number_stops[v]<<endl;
		

		if (vehicle_assigned[p] != v)
			cout<<"ERROR HYER"<<endl;

		blocked_vehicles[vehicle_assigned[p]] = 1;

		accept_relocate_trip = false;
		re_insertion2(p, accept_relocate_trip, temperature, type_move);

		int begin, end;
		begin = 0;
		end = 0;


		if (accept_relocate_trip) {
			count = 0;
			counter++; //one passenger has been succesfully been removed
			printf("remove heeeere\n");
			cout<<"number stops "<<number_stops[v]<<endl;
			//it means the passenger was relocated to a trip with a lower cost
			for (int i=0; i<number_stops[v]; i++){
				cout<<"waittty"<<number_passengers_action[v][i]<<endl;
				for (int j=0; j<number_passengers_action[v][i];j++) {
					cout<<"actp "<<action_passengers[v][i][j]<<endl;
					if (action_passengers[v][i][j] == p) {
						count++;
					
						//removes previous origin stuff
						if (number_passengers_action[v][i] == 1) {
							number_stops[v]--;
							stops[v].erase(stops[v].begin() + i);
							action_passengers[v].erase(action_passengers[v].begin() + i);
							number_passengers_action[v].erase(number_passengers_action[v].begin() + i);
							
							arrival_time_stop[v].erase(arrival_time_stop[v].begin() + i);
							departure_time_stop[v].erase(departure_time_stop[v].begin() + i);
							slack_time[v].erase(slack_time[v].begin() + i);
							free_capacity[v].erase(free_capacity[v].begin() + i);

							//re-update further arrival and departure times
							for (int k=i; k<=number_stops[v];k++) {
								old_arrival_time = arrival_time_stop[v][k];
								arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
								departure_time_stop[v][k] = arrival_time_stop[v][k];

								greatest_ed = INT_MIN;
								//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
								for (int l=0; l<number_passengers_action[v][k];l++) {
									if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
										greatest_ed = earliest_departure[action_passengers[v][k][l]];
								}

								if (greatest_ed > departure_time_stop[v][k])
									departure_time_stop[v][k] = greatest_ed;

								//gives "extra" slack
								//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];
							}
							

						} else {

							action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
							number_passengers_action[v][i]--;

							//arrival_time_stop[v][i] = //shouldnt change

							greatest_ed = INT_MIN;
							for (int l=0; l<number_passengers_action[v][i];l++) {
								if (earliest_departure[action_passengers[v][i][l]] > greatest_ed)
									greatest_ed = earliest_departure[action_passengers[v][i][l]];
							}

							//departure time only changes if the greatest earliest departure < departure time
							//it means that by removing the passenger the bus can leave earlier
							if (greatest_ed < departure_time_stop[v][i]) {
								if (arrival_time_stop[v][i] > greatest_ed){
									departure_time_stop[v][i] = arrival_time_stop[v][i];
								} else {
									departure_time_stop[v][i] = greatest_ed;
								}
							}
							

							//free_capacity[v][i]++;

							//re-update further arrival and departure times
							for (int k=i+1; k<=number_stops[v];k++) {
								
								old_arrival_time = arrival_time_stop[v][k];
								arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
								departure_time_stop[v][k] = arrival_time_stop[v][k];

								greatest_ed = INT_MIN;
								//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
								for (int l=0; l<number_passengers_action[v][k];l++) {
									if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
										greatest_ed = earliest_departure[action_passengers[v][k][l]];
								}

								if (greatest_ed > departure_time_stop[v][k])
									departure_time_stop[v][k] = greatest_ed;

								//gives "extra" slack
								//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];

							}

						}

						int current_user_ride_time, difference;
						bool leave_loop = false;
						for (int ii=0; ii<=number_stops[v];ii++) {
							for (int jj=0; jj<number_passengers_action[v][ii];jj++) {
								int save_p = action_passengers[v][ii][jj];
								for (int k=ii+1; k<=number_stops[v];k++) {
									leave_loop = false;
									for (int l=0; l<number_passengers_action[v][k];l++) {
										if (action_passengers[v][k][l] == save_p){
											current_user_ride_time = arrival_time_stop[v][k] - departure_time_stop[v][ii];
											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time -  user_ride_time[save_p];
												//<<"difference: "<<difference<<endl;
												total_user_ride_time += difference;
												//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
												user_ride_time[save_p] = current_user_ride_time;
												l = number_passengers_action[v][k]+1;
												//k = number_stops[best_v]+2; //leave loop
												leave_loop = true;
											}
										}
									}
									if (leave_loop)
										k = number_stops[v]+2;
								}	
							} 
						}

						
						if (count == 1) {
							begin = i;
							//free_capacity[v][i]++;
							i--;
						} else {
							end = i;
						}
						
						break;
					} 
		
				}

				
				for (int l=begin;l<end;l++) {
					free_capacity[v][l]++;
				}
			
				if (count == 2) {

					if (free_capacity[v].size()==2){
						//means the vehicle is empty at depot
						departure_time_stop[v][0] = current_time;
					}

					int min_slack_time_so_far, tightest_arrival_time, possible_new_slack_time;
					min_slack_time_so_far = INT_MAX;
					tightest_arrival_time = INT_MAX;
					//updates the slack times
					for (int k = number_stops[v]; k>=1; k--){

						tightest_arrival_time = INT_MAX;
						for (int l = 0; l < number_passengers_action[v][k]; l++) {
							if (latest_arrival[action_passengers[v][k][l]] < tightest_arrival_time)
								tightest_arrival_time = latest_arrival[action_passengers[v][k][l]];
						}

						if (tightest_arrival_time != INT_MAX)
							slack_time[v][k] = tightest_arrival_time - arrival_time_stop[v][k];
						else
							slack_time[v][k] = 86400;
						if (slack_time[v][k] < min_slack_time_so_far) {
							min_slack_time_so_far = slack_time[v][k];
						} else {
							slack_time[v][k] = min_slack_time_so_far;
						}

					}

					break;
				}
				
			}
		}

	}

	//v after the removal of all passengers? or not
	for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}
}*/

void re_insertion(int p, bool &accept_relocate_trip, double &temperature, int &type_move){

	int odb_travel_time, odb_distance, v;
	int pos_origin, pos_destination, sel_origin, sel_destination, min_increase_length, veh, ttcsd;
	bool repeated_station, no_violation_capacity, flexibilize_arrival_time, flexibilize_lat_departure_time, accept_delay_trip;
	int old_dpt_time, old_arr_time, prv_capacity, saved_increase;

	int delta; 
	double x;

	ttcsd = INT_MAX;

	bool not_feasible_insertion = true;
	int iterations = 1; //starts with one because you dont consider the vehicle the passenger was inserted previously
	bool tested_all_vehicles_once = false;
	int best_distance_depot = INT_MAX;
	int capacity_best_empty_vehicle = INT_MIN;
	int best_empty_vehicle, best_depot;
	best_empty_vehicle = -1;
	if (filtered_vehicles.size()>0)
		filtered_vehicles.clear();

	total_difference = 0;
	oldy_urt=total_user_ride_time;
	//check for closest depot that has an empty vehicle
	//second check for highest capacitated vehicle from that depot
	for (int i = 0; i < number_depots; i++) {

		ttcsd = INT_MAX;
		for (int k=0;k<number_stops_origin[p];k++) {
			if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
				ttcsd = travel_time[stops_origin[p][k]][depot[i]];
			}
		}

		//if ((best_distance_depot == 1) || (ttcsd <= best_distance_depot)) {
		if (ttcsd <= best_distance_depot) {
			capacity_best_empty_vehicle = INT_MIN;
			for (int j = 0; j < number_vehicles_at_depot[i]; j++) {

				veh = vehicles_at_depot[i][j];
				if (free_capacity[veh].size() == 2){ //means that vehicle is empty
					if (free_capacity[veh][0] >= capacity_best_empty_vehicle) {
						best_empty_vehicle = veh;
						best_depot = depot[i];
						capacity_best_empty_vehicle = free_capacity[veh][0];
						best_distance_depot = ttcsd;

					}
				}
			}
		}
	}

	int best_distance_location = best_distance_depot;

	for (int v=0;v<total_number_vehicles;v++){
		if (free_capacity[v].size() > 2) { //the vehicle has passengers assigned but might be empty at some point in time 
			for (int i=number_stops[v]-1; i<number_stops[v];i++){
				if (free_capacity[v][i] == free_capacity[v][0]) { //means that vehicle is empty

					ttcsd = INT_MAX;
					for (int k=0;k<number_stops_origin[p];k++) {
						if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
							ttcsd = travel_time[stops[v][i]][stops_origin[p][k]];
						}
					}

					if (ttcsd <= best_distance_location) {

						if (departure_time_stop[v][i] + ttcsd <= latest_departure[p]) { //important to verify if is feasible according to time window constraints

							//<<"HIERRR SA"<<endl;
							best_empty_vehicle = v;
							best_distance_location = ttcsd;
						}

					}
				}
			}
		}
	}

	if (best_empty_vehicle != -1)
		filtered_vehicles.push_back(best_empty_vehicle);

	//if (filtered_vehicles.size() == 0)
	filter_vehicles(p);

	//<<"filtered_vehicles SIZE "<<filtered_vehicles.size()<<endl;
	while ((not_feasible_insertion) && (iterations < filtered_vehicles.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		curr_number_insertions = 0;
		
		for (int vf=0; vf<filtered_vehicles.size();vf++) {
			int v = filtered_vehicles[vf];
			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			if (blocked_vehicles[v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				
				cheapest_origin2(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);
				
			}
		}
		//<<endl;

		//<<"curr insert2: " << curr_number_insertions<<endl;
		sort(insertions, insertions+curr_number_insertions, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions > 0) {
		
			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions)) {

				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				best_min_increase_length = insertions[iterations2].increase_length;
				best_sel_origin = insertions[iterations2].sel_station;
				best_pos_origin = insertions[iterations2].pos_station;
				best_v = insertions[iterations2].v;
				best_repeated_station = insertions[iterations2].repeated_station;

				if (not best_repeated_station) {
					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

					//updating arrival and departure from stops
					
					//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;
					

					if (best_pos_origin == 1) {
						int curr_dpt_time = earliest_departure[p]-travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
						if (curr_dpt_time < current_time){
							curr_dpt_time = current_time;
						}

						
						departure_time_stop[best_v][0] = curr_dpt_time;
					}

					arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
					
					if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
					else
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
					slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
					prv_capacity = free_capacity[best_v][best_pos_origin-1];
					free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);
				} else {
					//update passenger performing actions on the stops
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v][best_pos_origin]++;

					prv_arr_time_at_origin = arrival_time_stop[best_v][best_pos_origin];
					prv_dpt_time_at_origin = departure_time_stop[best_v][best_pos_origin];
					//no need to update the arrival time as it does not change
					//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
					if (earliest_departure[p] > departure_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v][best_pos_origin] = earliest_departure[p];
					//slack_time[best_v][best_pos_origin] //no need to update
					prv_capacity = free_capacity[best_v][best_pos_origin];
					free_capacity[best_v][best_pos_origin] = prv_capacity-1;
				}

				//update further arrival, departure times and slack times
				for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
					saved_arrival_time[i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					}
					saved_slack_time[i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				

				//if (best_v == 24) {
				/*cout<<"flex: "<<flexibilize_lat_departure_time<<endl;
				for (int i=0; i<=number_stops[best_v];i++) {
					cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
					for (int j=0; j<number_passengers_action[best_v][i];j++) 
						cout<<action_passengers[best_v][i][j]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
					cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
					cout<<"|"<<slack_time[best_v][i]<<"|  ";
					cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
				}
				//}
				cout<<endl;*/

				best_min_increase_length = INT_MAX;
				
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				if (tested_all_vehicles_once)
					flexibilize_arrival_time = true;
				else
					flexibilize_arrival_time = false;

				see_if_arrival_departure_dont_match(best_v);
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
			
				//<<"pos origin: "<<best_pos_origin<<" "<<endl;
				//<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
				
				if (sel_destination != -1) { 

					no_violation_capacity = true;
					for (int i=best_pos_origin+1;i<pos_destination;i++){

						free_capacity[best_v][i]--;

						if (free_capacity[best_v][i] < 0) {
							no_violation_capacity = false;
						}
						
					}
				}

				if (delay[p] <= max_flex_delay)
					accept_delay_trip = true;
				else 
					accept_delay_trip = false;

				/*odb_travel_time = travel_time[best_sel_origin][sel_destination];
				odb_distance = odb_travel_time*5.56;
				odb_distance = odb_distance/1000;
				
				odb_fare = ((ODB_base_fare[vehicle_type[best_v]]+ ODB_km_charge[vehicle_type[best_v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[best_v]]) + ODB_booking_fee[vehicle_type[best_v]]; 
				if (odb_fare < ODB_minimum_fare[vehicle_type[best_v]])
					odb_fare = ODB_minimum_fare[vehicle_type[best_v]];

				accept_relocate_trip = false;
				if (odb_fare < cost_trip[p])
					accept_relocate_trip = true;*/

				//<<"heere6"<<endl;
				//int new_total_user_ride_time = total_user_ride_time;
				//updates solution cost
				//user_ride_time[p] = travel_time[sel_origin][sel_destination];
				//total_user_ride_time += user_ride_time;
				/*int current_user_ride_time, difference;
				bool leave_loop = false;
				for (int i=0; i<=number_stops[best_v];i++) {
					for (int j=0; j<number_passengers_action[best_v][i];j++) {
						int save_p = action_passengers[best_v][i][j];
						for (int k=i+1; k<=number_stops[best_v];k++) {
							leave_loop = false;
							for (int l=0; l<number_passengers_action[best_v][k];l++) {
								if (action_passengers[best_v][k][l] == save_p){
									current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
									user_ride_time_temp[save_p] = current_user_ride_time;
									if (current_user_ride_time != user_ride_time[save_p]) {
										difference = user_ride_time[save_p] - current_user_ride_time;
										new_total_user_ride_time -= difference;
										//user_ride_time[save_p] = current_user_ride_time;
										l = number_passengers_action[best_v][k]+1;
										//k = number_stops[best_v]+2; //leave loop
										leave_loop = true;
									}
								}
							}
							if (leave_loop)
								k = number_stops[best_v]+2;
						}	
					} 
				}*/

				//int DELTA = total_user_ride_time - new_total_user_ride_time;
				int DELTA;
				int diff_remov_passenger = compute_difference_URT_by_fake_removing_passenger(p);
				int diff_add_passenger = 0;
				if ((sel_destination != -1) && (no_violation_capacity))
					diff_add_passenger = compute_difference_URT_by_fake_adding_passenger(p, best_v, sel_destination, pos_destination, repeated_station);
				DELTA = diff_remov_passenger + diff_add_passenger;

				//if (diff_remov_passenger < 0)
				//	cout<<"NEGATIVE ERROR"<<endl;
				//DELTA += diff_remov_passenger;
				accept_relocate_trip = false;
				if (DELTA > 0)
					accept_relocate_trip = true;

				//printf("DELTA:%d \n", DELTA);

				
				computedDELTA=DELTA;
				addedAtV = best_v;
				//<<"oldy and delta "<<oldy_urt<<DELTA<<endl;
				if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip) && (accept_relocate_trip)) {

					passengers_departure_time_from_home[p] = insertions[iterations2].passengers_departure_time_from_home;
					type_move = 1;
					vehicle_assigned[p] = best_v;

					//<<"feasible and reduce fare1"<<endl;
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;

					
					if (not repeated_station) {
						//cout<<"hierrs"<<endl;
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);
						//<<"hierrs2"<<endl;
						//updating arrival and departure from stops
						int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
						arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
						slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

						prv_capacity = free_capacity[best_v][pos_destination-1];
						free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
					} else {
						//cout<<"hierNrs1"<<endl;
						//update passenger performing actions on the stops
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v][pos_destination]++;

						

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						if (slc_time < slack_time[best_v][pos_destination]) 
							slack_time[best_v][pos_destination] = slc_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}


					//update further arrival, departure times, and slack times
					for (int i=pos_destination; i<=number_stops[best_v];i++) {
						old_arr_time = arrival_time_stop[best_v][i];
						//cout<<"aki "<<" "<<departure_time_stop[best_v][i-1]<<" "<<travel_time[stops[best_v][i-1]][stops[best_v][i]]<<" "<<stops[best_v][i-1]<<" "<<stops[best_v][i]<<endl;
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						old_dpt_time = departure_time_stop[best_v][i];
						//if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						//}

						//cout<<arrival_time_stop[best_v][i]<<" "<<departure_time_stop[best_v][i]<<endl;

						int greatest_ed = INT_MIN;
						//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
						for (int l=0; l<number_passengers_action[best_v][i];l++) {
							if (earliest_departure[action_passengers[best_v][i][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[best_v][i][l]];
						}

						if (greatest_ed > departure_time_stop[best_v][i])
							departure_time_stop[best_v][i] = greatest_ed;

						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						//cout<<arrival_time_stop[best_v][i]<<" "<<departure_time_stop[best_v][i]<<endl;
					}

					//for (int i=1; i<pos_destination;i++){
					//	int min_slack_time = INT_MAX;
					//	for (int j=i+1; i<number_stops[best_v];i++){
					//		if (slack_time[best_v][j] < min_slack_time)
					//			min_slack_time = slack_time[best_v][j];
					//	}
					//	slack_time[best_v][i] = min_slack_time;
					//}

					int min_slack_time_so_far = INT_MAX;	
					for (int i=number_stops[best_v]; i>=1; i--){
						if (slack_time[best_v][i] < min_slack_time_so_far) {
							min_slack_time_so_far = slack_time[best_v][i];
						} else {
							slack_time[best_v][i] = min_slack_time_so_far;
						}
					}

					
					//update_URT(best_v);
					
					/*solution_cost -= cost_trip[p]; //removes previous trip cost
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;*/

					//updates solution cost
					//user_ride_time[p] = travel_time[sel_origin][sel_destination];
					//total_user_ride_time += user_ride_time;
					int current_user_ride_time, difference;
					
					bool leave_loop = false;
					for (int i=0; i<=number_stops[best_v];i++) {
						for (int j=0; j<number_passengers_action[best_v][i];j++) {
							int save_p = action_passengers[best_v][i][j];
							for (int k=i+1; k<=number_stops[best_v];k++) {
								leave_loop = false;
								for (int l=0; l<number_passengers_action[best_v][k];l++) {
									if (action_passengers[best_v][k][l] == save_p){
										

										/*if (arrival_time_stop[best_v][k] != departure_time_stop[best_v][k]) {

											int greatest_ed = INT_MIN;
											//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
											for (int m=0; m<number_passengers_action[best_v][k];m++) {
												if (earliest_departure[action_passengers[best_v][k][m]] > greatest_ed)
													greatest_ed = earliest_departure[action_passengers[best_v][k][m]];
											}

											if (greatest_ed != departure_time_stop[best_v][k])
												departure_time_stop[best_v][k] = arrival_time_stop[best_v][k]; 

										}*/

										/*if (arrival_time_stop[best_v][i] != departure_time_stop[best_v][i]) {

											int greatest_ed = INT_MIN;
											//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
											for (int m=0; m<number_passengers_action[best_v][i];m++) {
												if (earliest_departure[action_passengers[best_v][i][m]] > greatest_ed)
													greatest_ed = earliest_departure[action_passengers[best_v][i][m]];
											}

											if (greatest_ed != departure_time_stop[best_v][i])
												departure_time_stop[best_v][i] = arrival_time_stop[best_v][i]; 

										}*/

										current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];

										if (current_user_ride_time != user_ride_time_temp[save_p]) {
											cout<<"crime fake add "<<save_p<<endl;
										}
										if (current_user_ride_time != user_ride_time[save_p]) {
											difference = current_user_ride_time -  user_ride_time[save_p];
											//<<"difference: "<<difference<<endl;
											total_user_ride_time += difference;
											total_difference += difference;
											//<<"COMP DIFFERENCE add (real) "<<save_p<<" "<<difference<<endl;
											//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
											user_ride_time[save_p] = current_user_ride_time;
											l = number_passengers_action[best_v][k]+1;
											//k = number_stops[best_v]+2; //leave loop
											leave_loop = true;
										}
									}
								}
								if (leave_loop)
									k = number_stops[best_v]+2;
							}	
						} 
					}

					
					
				} else {

					if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip)) {

						x = (double)rand() / (double)RAND_MAX;

						delta = DELTA;
						if (x < exp(-delta / (temperature))) {
							//non improving move is accepted according to SA
							accept_relocate_trip = true;
						} else {
							accept_relocate_trip = false;
						}
					} else {
						accept_relocate_trip = false;
					} 

					if (accept_relocate_trip) {

						passengers_departure_time_from_home[p] = insertions[iterations2].passengers_departure_time_from_home;
						type_move = 2;
						vehicle_assigned[p] = best_v;

						//<<"feasible and accepted according to SA (its NON IMPROVING)"<<endl;
						no_feasible_insertion2 = false;
						not_feasible_insertion = false;

						
						if (not repeated_station) {

							//cout<<"huiishx"<<endl;
							//<<"vehicle "<<best_v<<endl;
							stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
							
							number_stops[best_v]++;


							//update passenger performing actions on the stops
							action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);


							//updating arrival and departure from stops
							int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
							arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
							departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
							slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);


							prv_capacity = free_capacity[best_v][pos_destination-1];
							free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
							
						} else {
							//cout<<"huiishxESe"<<endl;
							//<<"hereeeeLSE"<<endl;
							//update passenger performing actions on the stops
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							number_passengers_action[best_v][pos_destination]++;

							//according to the update when the origin is added, if the stop already exists then no need to change
							//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//arrival_time_stop[best_v][pos_destination] = arr_time;
							//departure_time_stop[best_v][pos_destination] = arr_time;
							int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < slack_time[best_v][pos_destination]) 
								slack_time[best_v][pos_destination] = slc_time;

							//prv_capacity = free_capacity[best_v][pos_destination];
							//free_capacity[best_v][pos_destination] = prv_capacity+1;

						}


						//update further arrival, departure times, and slack times
						for (int i=pos_destination; i<=number_stops[best_v];i++) {
							old_arr_time = arrival_time_stop[best_v][i];
							arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
							old_dpt_time = departure_time_stop[best_v][i];
							//if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
							//}

							int greatest_ed = INT_MIN;
							//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
							for (int l=0; l<number_passengers_action[best_v][i];l++) {
								if (earliest_departure[action_passengers[best_v][i][l]] > greatest_ed)
									greatest_ed = earliest_departure[action_passengers[best_v][i][l]];
							}

							if (greatest_ed > departure_time_stop[best_v][i])
								departure_time_stop[best_v][i] = greatest_ed;
							slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						}

						//for (int i=1; i<pos_destination;i++){
						//	int min_slack_time = INT_MAX;
						//	for (int j=i+1; i<number_stops[best_v];i++){
						//		if (slack_time[best_v][j] < min_slack_time)
						//			min_slack_time = slack_time[best_v][j];
						//	}
						//	slack_time[best_v][i] = min_slack_time;
						//}

						int min_slack_time_so_far = INT_MAX;	
						for (int i=number_stops[best_v]; i>=1; i--){
							if (slack_time[best_v][i] < min_slack_time_so_far) {
								min_slack_time_so_far = slack_time[best_v][i];
							} else {
								slack_time[best_v][i] = min_slack_time_so_far;
							}
						}

						
						
						/*solution_cost -= cost_trip[p]; //removes previous trip cost
						solution_cost += odb_fare; //updates solution cost
						cost_trip[p] = odb_fare;*/
						//updates solution cost
						//user_ride_time[p] = travel_time[sel_origin][sel_destination];
						//total_user_ride_time += user_ride_time;
						int current_user_ride_time, difference;
						bool leave_loop = false;
						for (int i=0; i<=number_stops[best_v];i++) {
							for (int j=0; j<number_passengers_action[best_v][i];j++) {
								int save_p = action_passengers[best_v][i][j];
								for (int k=i+1; k<=number_stops[best_v];k++) {
									leave_loop = false;
									for (int l=0; l<number_passengers_action[best_v][k];l++) {
										//<<number_passengers_action[best_v][k]<<endl;
										if (action_passengers[best_v][k][l] == save_p){
											current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
											/*if (current_user_ride_time != user_ride_time_temp[save_p]) {
												cout<<"crime fake add2 "<<save_p<<endl;
											}*/
											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time - user_ride_time[save_p];
												//<<"difference: "<<difference<<endl;
												total_user_ride_time += difference;
												total_difference += difference;
												//<<"COMP DIFFERENCE add (real) "<<save_p<<" "<<difference<<endl;
												//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
												user_ride_time[save_p] = current_user_ride_time;
												l = number_passengers_action[best_v][k]+1;
												//k = number_stops[best_v]+2; //leave loop
												leave_loop = true;
											}
										}
									}
									if (leave_loop)
										k = number_stops[best_v]+2;

								}	
							} 
						}
						//<<"heeere re"<<endl;
						
					} else {

						type_move = 3;
						//no feasible insertion for destination
						accept_relocate_trip = false;
						
						//<<"no feasible insertion for destination"<<endl;
						//blocked_vehicles[best_v] = 1; //I suppose this is not necessary anymore. I'm already testing all vehicles with array insertions

						iterations2++;
						//solution_cost -= saved_increase;
						//vehicle_assigned[p] = -1;
						if (sel_destination != -1) {
							for (int i=best_pos_origin+1;i<pos_destination;i++){

								free_capacity[best_v][i]++;
			
							}
						}

						if (number_passengers_action[best_v][best_pos_origin] == 1) {
							number_stops[best_v]--;
							stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
							action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
							number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
							

							//re-update further arrival and departure times
							//cout<<"ns: "<<best_pos_origin<<" "<<number_stops[best_v]<<endl;
							for (int i=best_pos_origin+1; i<=number_stops[best_v]+1;i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
								//cout<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
							}
							arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
							departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
							slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


							free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


						} else {

							//<<"heere"<<endl;
							//<<action_passengers[best_v][best_pos_origin][1]<<endl;
							action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
							
							number_passengers_action[best_v][best_pos_origin]--;

							arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
							departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
							free_capacity[best_v][best_pos_origin]++;

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[i];
								departure_time_stop[best_v][i] = saved_departure_time[i];
								slack_time[best_v][i] = saved_slack_time[i];
							}

						}
					}

				}

				update_URT(best_v);
				

				//if (best_v == 24) {
					//<<flexibilize_lat_departure_time<<endl;
 				/*for (int i=0; i<=number_stops[best_v];i++) {
					cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
					for (int j=0; j<number_passengers_action[best_v][i];j++) 
						cout<<action_passengers[best_v][i][j]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
					cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
					cout<<"|"<<slack_time[best_v][i]<<"|  ";
					cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
				}*/
				//}	
				//cout<<endl;
			} //end while
		} else {
			iterations = filtered_vehicles.size()+1;
		}

		//<<"hieerxx";
		if (no_feasible_insertion2) {

			type_move = 3;
			//iterations++;
			iterations = filtered_vehicles.size()+1;
		}
		if (iterations >= filtered_vehicles.size()) {
			if (not tested_all_vehicles_once) {
				tested_all_vehicles_once = true;
				//iterations = 0;
				//<<"serving 3rd party";
				//serve_passenger_third_party_vehicle(p);
				//for (int i=0;i<total_number_vehicles;i++)
				//	blocked_vehicles[i] = 0;
			} else {
				//ultimately serve passenger with a 3rd party vehicle
				//<<"serving 3rd party";
				//serve_passenger_third_party_vehicle(p);
			}
		}
	}

	//<<"hieerxx2";
	route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[i] = 0;
}

std::vector<int> new_insertions_v;
std::vector<int> new_insertions_p;

void re_insertion_nn(int p, bool &accept_relocate_trip, double &temperature, int &type_move, int &diffURT){

	int odb_travel_time, odb_distance, v;
	int pos_origin, pos_destination, sel_origin, sel_destination, min_increase_length, veh, ttcsd;
	bool repeated_station, no_violation_capacity, flexibilize_arrival_time, flexibilize_lat_departure_time, accept_delay_trip;
	int old_dpt_time, old_arr_time, prv_capacity, saved_increase;

	int delta; 
	double x;

	ttcsd = INT_MAX;

	bool not_feasible_insertion = true;
	int iterations = 1; //starts with one because you dont consider the vehicle the passenger was inserted previously
	bool tested_all_vehicles_once = false;
	int best_distance_depot = INT_MAX;
	int capacity_best_empty_vehicle = INT_MIN;
	int best_empty_vehicle, best_depot;
	best_empty_vehicle = -1;
	if (filtered_vehicles.size()>0)
		filtered_vehicles.clear();

	//check for closest depot that has an empty vehicle
	//second check for highest capacitated vehicle from that depot
	for (int i = 0; i < number_depots; i++) {

		ttcsd = INT_MAX;
		for (int k=0;k<number_stops_origin[p];k++) {
			if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
				ttcsd = travel_time[stops_origin[p][k]][depot[i]];
			}
		}

		//if ((best_distance_depot == 1) || (ttcsd <= best_distance_depot)) {
		if (ttcsd <= best_distance_depot) {
			capacity_best_empty_vehicle = INT_MIN;
			for (int j = 0; j < number_vehicles_at_depot[i]; j++) {

				veh = vehicles_at_depot[i][j];
				if (free_capacity[veh].size() == 2){ //means that vehicle is empty
					if (free_capacity[veh][0] >= capacity_best_empty_vehicle) {
						best_empty_vehicle = veh;
						best_depot = depot[i];
						capacity_best_empty_vehicle = free_capacity[veh][0];
						best_distance_depot = ttcsd;

					}
				}
			}
		}
	}

	int best_distance_location = best_distance_depot;

	for (int v=0;v<total_number_vehicles;v++){
		if (free_capacity[v].size() > 2) { //the vehicle has passengers assigned but might be empty at some point in time 
			for (int i=number_stops[v]-1; i<number_stops[v];i++){
				if (free_capacity[v][i] == free_capacity[v][0]) { //means that vehicle is empty

					ttcsd = INT_MAX;
					for (int k=0;k<number_stops_origin[p];k++) {
						if (travel_time[stops_origin[p][k]][depot[i]] < ttcsd) {
							ttcsd = travel_time[stops[v][i]][stops_origin[p][k]];
						}
					}

					if (ttcsd <= best_distance_location) {

						if (departure_time_stop[v][i] + ttcsd <= latest_departure[p]) { //important to verify if is feasible according to time window constraints

							//<<"HIERRR SA"<<endl;
							best_empty_vehicle = v;
							best_distance_location = ttcsd;
						}

					}
				}
			}
		}
	}

	if (best_empty_vehicle != -1)
		filtered_vehicles.push_back(best_empty_vehicle);

	//if (filtered_vehicles.size() == 0)
	filter_vehicles(p);

	//<<"filtered_vehicles SIZE "<<filtered_vehicles.size()<<endl;
	while ((not_feasible_insertion) && (iterations < filtered_vehicles.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		curr_number_insertions = 0;
		
		for (int vf=0; vf<filtered_vehicles.size();vf++) {
			int v = filtered_vehicles[vf];
			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			if (blocked_vehicles[v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				
				cheapest_origin2(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);
				
			}
		}
		//<<endl;

		//<<"curr insert3: " << curr_number_insertions<<endl;
		sort(insertions, insertions+curr_number_insertions, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions > 0) {
		
			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions)) {

				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				best_min_increase_length = insertions[iterations2].increase_length;
				best_sel_origin = insertions[iterations2].sel_station;
				best_pos_origin = insertions[iterations2].pos_station;
				best_v = insertions[iterations2].v;
				best_repeated_station = insertions[iterations2].repeated_station;

				if (not best_repeated_station) {
					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

					//updating arrival and departure from stops
					
					//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;

					if (best_pos_origin == 1) {
						int curr_dpt_time = earliest_departure[p]-travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
						if (curr_dpt_time < current_time){
							curr_dpt_time = current_time;
						}

						
						departure_time_stop[best_v][0] = curr_dpt_time;
					}
					
					arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
					
					if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
					else
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
					slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
					prv_capacity = free_capacity[best_v][best_pos_origin-1];
					free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);
				} else {
					//update passenger performing actions on the stops
					action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					number_passengers_action[best_v][best_pos_origin]++;

					prv_arr_time_at_origin = arrival_time_stop[best_v][best_pos_origin];
					prv_dpt_time_at_origin = departure_time_stop[best_v][best_pos_origin];
					//no need to update the arrival time as it does not change
					//arrival_time_stop[best_v][best_pos_origin] = departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
					if (earliest_departure[p] > departure_time_stop[best_v][best_pos_origin])
						departure_time_stop[best_v][best_pos_origin] = earliest_departure[p];
					//slack_time[best_v][best_pos_origin] //no need to update
					prv_capacity = free_capacity[best_v][best_pos_origin];
					free_capacity[best_v][best_pos_origin] = prv_capacity-1;
				}

				//update further arrival, departure times and slack times
				for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
					saved_arrival_time[i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					}
					saved_slack_time[i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				

				//if (best_v == 24) {
				/*cout<<"flex: "<<flexibilize_lat_departure_time<<endl;
				for (int i=0; i<=number_stops[best_v];i++) {
					cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
					for (int j=0; j<number_passengers_action[best_v][i];j++) 
						cout<<action_passengers[best_v][i][j]<<" ";
					cout<<"]  ";

					cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
					cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
					cout<<"|"<<slack_time[best_v][i]<<"|  ";
					cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
				}*/
				//}
				//<<endl;

				best_min_increase_length = INT_MAX;
				
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				if (tested_all_vehicles_once)
					flexibilize_arrival_time = true;
				else
					flexibilize_arrival_time = false;
				see_if_arrival_departure_dont_match(best_v);
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time);
			
				//<<"pos origin: "<<best_pos_origin<<" "<<endl;
				//<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
				
				if (sel_destination != -1) { 

					no_violation_capacity = true;
					for (int i=best_pos_origin+1;i<pos_destination;i++){

						free_capacity[best_v][i]--;

						if (free_capacity[best_v][i] < 0) {
							no_violation_capacity = false;
						}
						
					}
				}

				if (delay[p] <= max_flex_delay)
					accept_delay_trip = true;
				else 
					accept_delay_trip = false;

				/*odb_travel_time = travel_time[best_sel_origin][sel_destination];
				odb_distance = odb_travel_time*5.56;
				odb_distance = odb_distance/1000;
				
				odb_fare = ((ODB_base_fare[vehicle_type[best_v]]+ ODB_km_charge[vehicle_type[best_v]]*odb_distance) * ODB_surge_multiplier[vehicle_type[best_v]]) + ODB_booking_fee[vehicle_type[best_v]]; 
				if (odb_fare < ODB_minimum_fare[vehicle_type[best_v]])
					odb_fare = ODB_minimum_fare[vehicle_type[best_v]];

				accept_relocate_trip = false;
				if (odb_fare < cost_trip[p])
					accept_relocate_trip = true;*/

				//<<"heere6"<<endl;
				//int new_total_user_ride_time = total_user_ride_time;
				//updates solution cost
				//user_ride_time[p] = travel_time[sel_origin][sel_destination];
				//total_user_ride_time += user_ride_time;
				/*int current_user_ride_time, difference;
				bool leave_loop = false;
				for (int i=0; i<=number_stops[best_v];i++) {
					for (int j=0; j<number_passengers_action[best_v][i];j++) {
						int save_p = action_passengers[best_v][i][j];
						for (int k=i+1; k<=number_stops[best_v];k++) {
							leave_loop = false;
							for (int l=0; l<number_passengers_action[best_v][k];l++) {
								if (action_passengers[best_v][k][l] == save_p){
									current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
									if (current_user_ride_time != user_ride_time[save_p]) {
										difference = user_ride_time[save_p] - current_user_ride_time;
										new_total_user_ride_time -= difference;
										//user_ride_time[save_p] = current_user_ride_time;
										l = number_passengers_action[best_v][k]+1;
										//k = number_stops[best_v]+2; //leave loop
										leave_loop = true;
									}
								}
							}
							if (leave_loop)
								k = number_stops[best_v]+2;
						}	
					} 
				}*/

				int DELTA;
				int diff_remov_passenger = compute_difference_URT_by_fake_removing_passenger(p);
				int diff_add_passenger = 0;
				if ((sel_destination != -1) && (no_violation_capacity))
					diff_add_passenger = compute_difference_URT_by_fake_adding_passenger(p, best_v, sel_destination, pos_destination, repeated_station);
				DELTA = diff_remov_passenger + diff_add_passenger;

				//if (diff_remov_passenger < 0)
				//	cout<<"NEGATIVE ERROR"<<endl;
				//DELTA += diff_remov_passenger;
				//accept_relocate_trip = false;
				//if (DELTA > 0)
					accept_relocate_trip = true; //first feasible is accepted?

				//printf("DELTA:%d \n", DELTA);

				totalcomputedDELTA += DELTA;
				if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip) && (accept_relocate_trip)) {

					diffURT = DELTA;
					new_insertions_v.push_back(best_v);
					new_insertions_p.push_back(p);
					passengers_departure_time_from_home[p] = insertions[iterations2].passengers_departure_time_from_home;
					type_move = 1;
					vehicle_assigned[p] = best_v;

					//<<"feasible and reduce fare2"<<endl;
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;
					
					if (not repeated_station) {
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);

						//updating arrival and departure from stops
						int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
						arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
						departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
						slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

						prv_capacity = free_capacity[best_v][pos_destination-1];
						free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
					} else {

						//update passenger performing actions on the stops
						action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						number_passengers_action[best_v][pos_destination]++;

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						if (slc_time < slack_time[best_v][pos_destination]) 
							slack_time[best_v][pos_destination] = slc_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}


					//update further arrival, departure times, and slack times
					for (int i=pos_destination; i<=number_stops[best_v];i++) {
						old_arr_time = arrival_time_stop[best_v][i];
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						old_dpt_time = departure_time_stop[best_v][i];
						//if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						//}

						int greatest_ed = INT_MIN;
						//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
						for (int l=0; l<number_passengers_action[best_v][i];l++) {
							if (earliest_departure[action_passengers[best_v][i][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[best_v][i][l]];
						}

						if (greatest_ed > departure_time_stop[best_v][i])
							departure_time_stop[best_v][i] = greatest_ed;

						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
					}

					//for (int i=1; i<pos_destination;i++){
					//	int min_slack_time = INT_MAX;
					//	for (int j=i+1; i<number_stops[best_v];i++){
					//		if (slack_time[best_v][j] < min_slack_time)
					//			min_slack_time = slack_time[best_v][j];
					//	}
					//	slack_time[best_v][i] = min_slack_time;
					//}

					int min_slack_time_so_far = INT_MAX;	
					for (int i=number_stops[best_v]; i>=1; i--){
						if (slack_time[best_v][i] < min_slack_time_so_far) {
							min_slack_time_so_far = slack_time[best_v][i];
						} else {
							slack_time[best_v][i] = min_slack_time_so_far;
						}
					}

					
					
					/*solution_cost -= cost_trip[p]; //removes previous trip cost
					solution_cost += odb_fare; //updates solution cost
					cost_trip[p] = odb_fare;*/

					//updates solution cost
					//user_ride_time[p] = travel_time[sel_origin][sel_destination];
					//total_user_ride_time += user_ride_time;
					int current_user_ride_time, difference;
					bool leave_loop = false;
					for (int i=0; i<=number_stops[best_v];i++) {
						for (int j=0; j<number_passengers_action[best_v][i];j++) {
							int save_p = action_passengers[best_v][i][j];
							affected_passengers[save_p] = 1;
							for (int k=i+1; k<=number_stops[best_v];k++) {
								leave_loop = false;
								for (int l=0; l<number_passengers_action[best_v][k];l++) {
									if (action_passengers[best_v][k][l] == save_p){
										current_user_ride_time = arrival_time_stop[best_v][k] - departure_time_stop[best_v][i];
										
										/*if (current_user_ride_time != user_ride_time_temp[save_p]) {
											cout<<"crime fake add2 "<<save_p<<endl;
										}*/
										if (current_user_ride_time != user_ride_time[save_p]) {
											difference = current_user_ride_time -  user_ride_time[save_p];
											//<<"difference: "<<difference<<endl;
											total_user_ride_time += difference;
											//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
											
											user_ride_time[save_p] = current_user_ride_time;
											l = number_passengers_action[best_v][k]+1;
											//k = number_stops[best_v]+2; //leave loop
											leave_loop = true;
										}
									}
								}
								if (leave_loop)
									k = number_stops[best_v]+2;
							}	
						} 
					}

					
				} else {

					type_move = 3;
					//no feasible insertion for destination
					accept_relocate_trip = false;
					
					//<<"no feasible insertion for destination"<<endl;
					//blocked_vehicles[best_v] = 1; //I suppose this is not necessary anymore. I'm already testing all vehicles with array insertions

					iterations2++;
					//solution_cost -= saved_increase;
					//vehicle_assigned[p] = -1;
					if (sel_destination != -1) {
						for (int i=best_pos_origin+1;i<pos_destination;i++){

							free_capacity[best_v][i]++;
		
						}
					}

					if (number_passengers_action[best_v][best_pos_origin] == 1) {
						number_stops[best_v]--;
						stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
						action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
						number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
						

						//re-update further arrival and departure times
						//cout<<"ns: "<<best_pos_origin<<" "<<number_stops[best_v]<<endl;
						for (int i=best_pos_origin+1; i<=number_stops[best_v]+1;i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[i];
							departure_time_stop[best_v][i] = saved_departure_time[i];
							slack_time[best_v][i] = saved_slack_time[i];
							//cout<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
						}
						arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
						departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
						slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


						free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);
					} else {

						//<<"heere"<<endl;
						//<<action_passengers[best_v][best_pos_origin][1]<<endl;
						action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
						
						number_passengers_action[best_v][best_pos_origin]--;

						arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
						departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
						free_capacity[best_v][best_pos_origin]++;

						//re-update further arrival and departure times
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[i];
							departure_time_stop[best_v][i] = saved_departure_time[i];
							slack_time[best_v][i] = saved_slack_time[i];
							//cout<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
						}
					}
				}

				update_URT(best_v);
			
				

			//if (best_v == 24) {
				//<<flexibilize_lat_departure_time<<endl;
			/*for (int i=0; i<=number_stops[best_v];i++) {
				cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
				for (int j=0; j<number_passengers_action[best_v][i];j++) 
					cout<<action_passengers[best_v][i][j]<<" ";
				cout<<"]  ";

				cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
				cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
				cout<<"|"<<slack_time[best_v][i]<<"|  ";
				cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
			}*/
				//}	
				//<<endl;
			} //end while
		} else {
			iterations = filtered_vehicles.size()+1;
		}

		if (no_feasible_insertion2) {

			type_move = 3;
			//iterations++;
			iterations = filtered_vehicles.size()+1;
		}
		if (iterations >= filtered_vehicles.size()) {
			if (not tested_all_vehicles_once) {
				tested_all_vehicles_once = true;
				//iterations = 0;
				//<<"serving 3rd party";
				//serve_passenger_third_party_vehicle(p);
				//for (int i=0;i<total_number_vehicles;i++)
				//	blocked_vehicles[i] = 0;
			} else {
				//ultimately serve passenger with a 3rd party vehicle
				//<<"serving 3rd party";
				//serve_passenger_third_party_vehicle(p);
			}
		}
	}

	route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[i] = 0;
}

void relocate_all_passengers_vehicle_nn(int v, double &temperature, int &type_move, int& counter, int& deltaURT, bool&megaerror){


	bool accept_relocate_trip = false;
	//compute the removal of passenger p from the solution
	int decrease_solution = 0;
	int count=0;
	int greatest_ed = INT_MIN;
	int old_arrival_time;
	int p;
	int diffURT;
	

	/*cout<<"NEW RELOCATE TO EMPTY A VEHICLE"<<endl;
	for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}*/

	
	for (int kk=0; kk<passengers_at_vehicle[v].size();kk++){
		p = passengers_at_vehicle[v][kk]; //passenger that will try to relocate

		/*if ((p < 0) || (p > 300)){
			cout<<"SUPERR ERROR"<<endl;
			megaerror = true;
			return;
		}*/
		//<<"passengerx "<<p<<endl;
		//<<"before"<<endl;
		//<<"number stops "<<number_stops[v]<<endl;
		/*for (int l=0; l<=number_stops[v];l++) {
			cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
			for (int m=0; m<number_passengers_action[v][l];m++) 
				cout<<action_passengers[v][l][m]<<" ";
			cout<<"]  ";

			cout<<"{"<<arrival_time_stop[v][l]<<"} ";
			cout<<"{"<<departure_time_stop[v][l]<<"} ";
			cout<<"|"<<slack_time[v][l]<<"|  ";
			cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
		}*/

		if (vehicle_assigned[p] != v)
			cout<<"ERROR HYER"<<endl;

		blocked_vehicles[vehicle_assigned[p]] = 1;

		accept_relocate_trip = false;
		re_insertion_nn(p, accept_relocate_trip, temperature, type_move, diffURT);

		int begin, end;
		begin = 0;
		end = 0;


		if (accept_relocate_trip) {
			
			counter++; //one passenger has been succesfully been removed
			
			count = 0;
			//printf("remove heeeere\n");
			//<<"number stops "<<number_stops[v]<<endl;
			deltaURT += diffURT;
			//it means the passenger was relocated to a trip with a lower cost
			/*for (int i=0; i<number_stops[v]; i++){
				cout<<"waittty"<<number_passengers_action[v][i]<<endl;
				for (int j=0; j<number_passengers_action[v][i];j++) {
					cout<<"actp "<<action_passengers[v][i][j]<<endl;
					if (action_passengers[v][i][j] == p) {
						count++;
					
						//removes previous origin stuff
						if (number_passengers_action[v][i] == 1) {
							number_stops[v]--;
							stops[v].erase(stops[v].begin() + i);
							action_passengers[v].erase(action_passengers[v].begin() + i);
							number_passengers_action[v].erase(number_passengers_action[v].begin() + i);
							
							arrival_time_stop[v].erase(arrival_time_stop[v].begin() + i);
							departure_time_stop[v].erase(departure_time_stop[v].begin() + i);
							slack_time[v].erase(slack_time[v].begin() + i);
							free_capacity[v].erase(free_capacity[v].begin() + i);

							//re-update further arrival and departure times
							for (int k=i; k<=number_stops[v];k++) {
								old_arrival_time = arrival_time_stop[v][k];
								arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
								departure_time_stop[v][k] = arrival_time_stop[v][k];

								greatest_ed = INT_MIN;
								//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
								for (int l=0; l<number_passengers_action[v][k];l++) {
									if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
										greatest_ed = earliest_departure[action_passengers[v][k][l]];
								}

								if (greatest_ed > departure_time_stop[v][k])
									departure_time_stop[v][k] = greatest_ed;

								//gives "extra" slack
								//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];
							}
							

						} else {

							action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
							number_passengers_action[v][i]--;

							//arrival_time_stop[v][i] = //shouldnt change

							greatest_ed = INT_MIN;
							for (int l=0; l<number_passengers_action[v][i];l++) {
								if (earliest_departure[action_passengers[v][i][l]] > greatest_ed)
									greatest_ed = earliest_departure[action_passengers[v][i][l]];
							}

							//departure time only changes if the greatest earliest departure < departure time
							//it means that by removing the passenger the bus can leave earlier
							if (greatest_ed < departure_time_stop[v][i]) {
								if (arrival_time_stop[v][i] > greatest_ed){
									departure_time_stop[v][i] = arrival_time_stop[v][i];
								} else {
									departure_time_stop[v][i] = greatest_ed;
								}
							}
							

							//free_capacity[v][i]++;

							//re-update further arrival and departure times
							for (int k=i+1; k<=number_stops[v];k++) {
								
								old_arrival_time = arrival_time_stop[v][k];
								arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
								departure_time_stop[v][k] = arrival_time_stop[v][k];

								greatest_ed = INT_MIN;
								//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
								for (int l=0; l<number_passengers_action[v][k];l++) {
									if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
										greatest_ed = earliest_departure[action_passengers[v][k][l]];
								}

								if (greatest_ed > departure_time_stop[v][k])
									departure_time_stop[v][k] = greatest_ed;

								//gives "extra" slack
								//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];

							}

						}

						int current_user_ride_time, difference;
						bool leave_loop = false;
						for (int ii=0; ii<=number_stops[v];ii++) {
							for (int jj=0; jj<number_passengers_action[v][ii];jj++) {
								int save_p = action_passengers[v][ii][jj];
								for (int k=ii+1; k<=number_stops[v];k++) {
									leave_loop = false;
									for (int l=0; l<number_passengers_action[v][k];l++) {
										if (action_passengers[v][k][l] == save_p){
											current_user_ride_time = arrival_time_stop[v][k] - departure_time_stop[v][ii];
											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time -  user_ride_time[save_p];
												//<<"difference: "<<difference<<endl;
												total_user_ride_time += difference;
												//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
												user_ride_time[save_p] = current_user_ride_time;
												l = number_passengers_action[v][k]+1;
												//k = number_stops[best_v]+2; //leave loop
												leave_loop = true;
											}
										}
									}
									if (leave_loop)
										k = number_stops[v]+2;
								}	
							} 
						}

						
						if (count == 1) {
							begin = i;
							//free_capacity[v][i]++;
							i--;
						} else {
							end = i;
						}
						
						break;
					} 
		
				}

				
				for (int l=begin;l<end;l++) {
					free_capacity[v][l]++;
				}
			
				if (count == 2) {

					if (free_capacity[v].size()==2){
						//means the vehicle is empty at depot
						departure_time_stop[v][0] = current_time;
					}

					int min_slack_time_so_far, tightest_arrival_time, possible_new_slack_time;
					min_slack_time_so_far = INT_MAX;
					tightest_arrival_time = INT_MAX;
					//updates the slack times
					for (int k = number_stops[v]; k>=1; k--){

						tightest_arrival_time = INT_MAX;
						for (int l = 0; l < number_passengers_action[v][k]; l++) {
							if (latest_arrival[action_passengers[v][k][l]] < tightest_arrival_time)
								tightest_arrival_time = latest_arrival[action_passengers[v][k][l]];
						}

						if (tightest_arrival_time != INT_MAX)
							slack_time[v][k] = tightest_arrival_time - arrival_time_stop[v][k];
						else
							slack_time[v][k] = 86400;
						if (slack_time[v][k] < min_slack_time_so_far) {
							min_slack_time_so_far = slack_time[v][k];
						} else {
							slack_time[v][k] = min_slack_time_so_far;
						}

					}

					break;
				}
				
			}*/
		}

	}

	//v after the removal of all passengers? or not
	/*for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}*/
}

void empty_vehicle(int v, bool& megaerror, double &temperature, int &type_move){

	if (new_insertions_v.size() > 0)
		new_insertions_v.clear();

	if (new_insertions_p.size() > 0)
		new_insertions_p.clear();

	bool all_accepted_relocate_trip = false;
	int counter = 0;
	int deltaURT;

	deltaURT = 0;
	oldy_urt = total_user_ride_time;
	totalcomputedDELTA = 0;
	
	save_intm_solution();

	int best_v = v;
	/*cout<<"STARTING EMPTYING HEEEERE "<<current_time<<" "<<v<<endl;
	
	for (int i=0; i<=number_stops[best_v];i++) {
		cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
		for (int j=0; j<number_passengers_action[best_v][i];j++) 
			cout<<action_passengers[best_v][i][j]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
		cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
		cout<<"|"<<slack_time[best_v][i]<<"|  ";
		cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
	}

	cout<<"passengers to be removed: "<<endl;
	for (int i = 0; i < passengers_at_vehicle[v].size(); i++){
		cout<<passengers_at_vehicle[v][i]<<" ";
	}
	cout<<endl;
	cout<<"number stops "<<number_stops[v]<<endl;*/

	//bool megaerror = false;


	relocate_all_passengers_vehicle_nn(v, init_temperature, type_move, counter, deltaURT, megaerror);

	//UPDATE USER RIDE TIMES TO SEE IF NOW I CAN COMPUTE DELTA CORRECTLY??????
	
	if (megaerror){
		return;
	}
	bool accept_relocate_trip;
	if (counter == passengers_at_vehicle[v].size()) {
		//improving move regarding URT
		//necessary to remove the passengers from v

		//<<"revert changes1"<<endl;
		for (int i = 0; i<passengers_at_vehicle[v].size();i++){
			int p = passengers_at_vehicle[v][i];
			//cout<<p<<endl;
			remove_passenger_from_vehicle(v, p);
			//update URT
			update_URT(vehicle_assigned[p]);
		}
		see_if_arrival_departure_dont_match(v);
		update_URT(v);

		//<<"oldyxnew: "<<oldy_urt<<" "<<total_user_ride_time<<" "<<(oldy_urt-total_user_ride_time)<<endl; 
		if ((oldy_urt-total_user_ride_time) > 0){
			//cout<<"REMOVEALLPOSITIVE"<<endl;
			type_move = 1;
		} else {
			int x = (double)rand() / (double)RAND_MAX;
			int delta = oldy_urt-total_user_ride_time;

			if (x < exp(-delta / (temperature))) {
				//non improving move is accepted according to SA
				accept_relocate_trip = true;
			} else {
				accept_relocate_trip = false;
			}

			if (accept_relocate_trip) {
				//cout<<"REMOVEALLACCEPT"<<endl;
				type_move = 2;
			} else {
				type_move = 3;
				//<<"back to beginning"<<endl;
				return_intm_solution();
			}
		}
		
	} else {
		type_move = 3;
		//cout<<"revert changes2"<<endl;
		return_intm_solution();
		//revert changes
		/*for (int i=0; i<new_insertions_v.size(); i++){
			int p = new_insertions_p[i];
			cout<<"hh "<<p<<endl;
			remove_passenger_from_vehicle(new_insertions_v[i], p);
			//update URT
			update_URT(new_insertions_v[i]);
			vehicle_assigned[p] = v;
 		}
 		see_if_arrival_departure_dont_match(v);
 		update_URT(v);*/

	}
}

void relocate_passenger(int p, double &temperature, int &type_move){

	//cout<<"before relocated passenger: "<<p<<endl;

	bool accept_relocate_trip = false;
	//compute the removal of passenger p from the solution
	int v = vehicle_assigned[p];
	int decrease_solution = 0;
	int count=0;
	int greatest_ed = INT_MIN;
	int old_arrival_time;


	/*cout<<vehicle_assigned[p]<<endl;
	for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}
	cout<<endl<<endl;*/

	//this can be maybe speed up by keeping a variable of where in the vehicle the passenger is assigned
	/*for (int i = 0; i < number_stops[v]; i++){
		for (int j=0; j<number_passengers_action[v][i];j++) {
			if (action_passengers[v][i][j] == p) {
				decrease_solution += travel_time[stops[v][i-1]][stops[v][i+1]] -travel_time[stops[v][i]][stops[v][i+1]] -travel_time[stops[v][i-1]][stops[v][i]];
				count++;
				break;
			}
		}
		if (count == 2)
			break;
	}*/

	blocked_vehicles[vehicle_assigned[p]] = 1;

	re_insertion(p, accept_relocate_trip, temperature, type_move);

	int begin, end;
	begin = 0;
	end = 0;
	int total_faking_error = 0;
	if (accept_relocate_trip) {
		//cout<<"removeee here SA"<<endl;
		//printf("remove heeeere SA\n");
		//<<"number stops "<<number_stops[v]<<endl;
		//it means the passenger was relocated to another trip 
		
		for (int i=0; i<number_stops[v]; i++){
			for (int j=0; j<number_passengers_action[v][i];j++) {
				//<<"actp "<<action_passengers[v][i][j]<<endl;
				if (action_passengers[v][i][j] == p) {
					count++;
				
					//<<"SAhier1"<<endl;
					//removes previous origin stuff
					if (number_passengers_action[v][i] == 1) {
						number_stops[v]--;

						//cout<<"SAhier2"<<i<<endl;
						stops[v].erase(stops[v].begin() + i);
						action_passengers[v].erase(action_passengers[v].begin() + i);
						number_passengers_action[v].erase(number_passengers_action[v].begin() + i);
						
						arrival_time_stop[v].erase(arrival_time_stop[v].begin() + i);
						departure_time_stop[v].erase(departure_time_stop[v].begin() + i);
						slack_time[v].erase(slack_time[v].begin() + i);
						free_capacity[v].erase(free_capacity[v].begin() + i);

						//<<"SAhier3"<<i<<endl;
						//re-update further arrival and departure times
						for (int k=i; k<=number_stops[v];k++) {
							old_arrival_time = arrival_time_stop[v][k];

							//dont change
							/*if (k == 1) {
								
								if(departure_time_stop[v][k-1] < current_time) {
									
									arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
									departure_time_stop[v][k] = arrival_time_stop[v][k];
								} else {
									departure_time_stop[v][k-1] = current_time;
									arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
									departure_time_stop[v][k] = arrival_time_stop[v][k];
								}
							}*/

							if (k > 1) {
								arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
								departure_time_stop[v][k] = arrival_time_stop[v][k];
							} 
							//<<"xsx: "<<arrival_time_stop[v][k]<<" "<<departure_time_stop[v][k]<<" "<<stops[v][k-1]<<" "<<stops[v][k]<<" "<<travel_time[stops[v][k-1]][stops[v][k]]<<endl;

							greatest_ed = INT_MIN;
							//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
							for (int l=0; l<number_passengers_action[v][k];l++) {
								if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
									greatest_ed = earliest_departure[action_passengers[v][k][l]];
							}

							if (greatest_ed > departure_time_stop[v][k])
								departure_time_stop[v][k] = greatest_ed;

							//gives "extra" slack
							//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];
						}
						

					} else {

						action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
						number_passengers_action[v][i]--;

						//arrival_time_stop[v][i] = //shouldnt change

						greatest_ed = INT_MIN;
						for (int l=0; l<number_passengers_action[v][i];l++) {
							if (earliest_departure[action_passengers[v][i][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[v][i][l]];
						}

						//departure time only changes if the greatest earliest departure < departure time
						//it means that by removing the passenger the bus can leave earlier
						if (greatest_ed < departure_time_stop[v][i]) {
							if (arrival_time_stop[v][i] > greatest_ed){
								departure_time_stop[v][i] = arrival_time_stop[v][i];
							} else {
								departure_time_stop[v][i] = greatest_ed;
							}
						}
						

						//free_capacity[v][i]++;

						//re-update further arrival and departure times
						for (int k=i+1; k<=number_stops[v];k++) {
							
							old_arrival_time = arrival_time_stop[v][k];
							arrival_time_stop[v][k] = departure_time_stop[v][k-1]+travel_time[stops[v][k-1]][stops[v][k]];
							departure_time_stop[v][k] = arrival_time_stop[v][k];

							greatest_ed = INT_MIN;
							//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
							for (int l=0; l<number_passengers_action[v][k];l++) {
								if (earliest_departure[action_passengers[v][k][l]] > greatest_ed)
									greatest_ed = earliest_departure[action_passengers[v][k][l]];
							}

							if (greatest_ed > departure_time_stop[v][k])
								departure_time_stop[v][k] = greatest_ed;

							//gives "extra" slack
							//slack_time[v][k] += old_arrival_time - arrival_time_stop[v][k];

						}

					}


					
		
					
					int current_user_ride_time, difference;
					bool leave_loop = false;
					for (int i=0; i<=number_stops[v];i++) {
						for (int j=0; j<number_passengers_action[v][i];j++) {
							int save_p = action_passengers[v][i][j];
							for (int k=i+1; k<=number_stops[v];k++) {
								leave_loop = false;
								for (int l=0; l<number_passengers_action[v][k];l++) {
									if (action_passengers[v][k][l] == save_p){
										current_user_ride_time = arrival_time_stop[v][k] - departure_time_stop[v][i];
										
										if (count == 2){
											if (current_user_ride_time != user_ride_time_temp[save_p]) {
												cout<<"faking removing error"<<endl;
												cout<<current_user_ride_time<<" "<<user_ride_time_temp[save_p]<<endl;
												cout<<arrival_time_stop[v][k]<<" "<<departure_time_stop[v][i]<<endl;
												total_faking_error += user_ride_time_temp[save_p] - current_user_ride_time;
											}
										

											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time -  user_ride_time[save_p];
												//<<"difference: "<<difference<<endl;
												total_user_ride_time += difference;
												total_difference += difference;
												//<<"COMP DIFFERENCE remove (real) "<<save_p<<" "<<difference<<endl;
												//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
												user_ride_time[save_p] = current_user_ride_time;
												l = number_passengers_action[v][k]+1;
												//k = number_stops[best_v]+2; //leave loop
												leave_loop = true;
											}
										}

									}
								}
								if (leave_loop)
									k = number_stops[v]+2;
							}	
						} 
					}

					
					if (count == 1) {
						begin = i;
						//free_capacity[v][i]++;
						i--;
					} else {
						end = i;
					}
					
					break;
				} 		
			}


			if (count == 2) {
				//<<"oldyxnew: "<<oldy_urt<<" "<<total_user_ride_time<<" "<<(oldy_urt-total_user_ride_time)<<" "<<computedDELTA<<" ;"<<total_difference<<endl; 
				if ((oldy_urt-total_user_ride_time) != computedDELTA){
					//cout<<"COMPUTING ERRROR"<<endl;
					if (total_faking_error != 0)
						cout<<"total faking error: "<<total_faking_error<<endl;
				}
				break;
			}	
		}
		see_if_arrival_departure_dont_match(v);
		update_URT(v);


	}

	for (int l=begin;l<end;l++) {
		free_capacity[v][l]++;
	}
	
	if (count == 2) {

		if (free_capacity[v].size()==2){
			//means the vehicle is empty at depot
			departure_time_stop[v][0] = current_time;
		}

		int min_slack_time_so_far, tightest_arrival_time, possible_new_slack_time;
		min_slack_time_so_far = INT_MAX;
		tightest_arrival_time = INT_MAX;
		//updates the slack times
		for (int k = number_stops[v]; k>=1; k--){

			tightest_arrival_time = INT_MAX;
			for (int l = 0; l < number_passengers_action[v][k]; l++) {
				if (latest_arrival[action_passengers[v][k][l]] < tightest_arrival_time)
					tightest_arrival_time = latest_arrival[action_passengers[v][k][l]];
			}

			if (tightest_arrival_time != INT_MAX)
				slack_time[v][k] = tightest_arrival_time - arrival_time_stop[v][k];
			else
				slack_time[v][k] = 86400;
			if (slack_time[v][k] < min_slack_time_so_far) {
				min_slack_time_so_far = slack_time[v][k];
			} else {
				slack_time[v][k] = min_slack_time_so_far;
			}

		}

		//break;
	}



	/*cout<<"after removing: "<<endl;
	for (int l=0; l<=number_stops[v];l++) {
		cout<<stops[v][l]<<" ("<<number_passengers_action[v][l]<<") "<<" [";
		for (int m=0; m<number_passengers_action[v][l];m++) 
			cout<<action_passengers[v][l][m]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][l]<<"} ";
		cout<<"{"<<departure_time_stop[v][l]<<"} ";
		cout<<"|"<<slack_time[v][l]<<"|  ";
		cout<<"*"<<free_capacity[v][l]<<"*"<<endl;
	}
	cout<<endl;*/

	if (count == 2) {
		v = addedAtV;
		int current_user_ride_time, difference;
		bool leave_loop = false;
		for (int i=0; i<=number_stops[v];i++) {
			for (int j=0; j<number_passengers_action[v][i];j++) {
				int save_p = action_passengers[v][i][j];
				for (int k=i+1; k<=number_stops[v];k++) {
					leave_loop = false;
					for (int l=0; l<number_passengers_action[v][k];l++) {
						if (action_passengers[v][k][l] == save_p){
							current_user_ride_time = arrival_time_stop[v][k] - departure_time_stop[v][i];
							
							//if (count == 2){
							if (current_user_ride_time != user_ride_time_temp[save_p]) {
								cout<<"delta computing error"<<endl;
								cout<<save_p<<" "<<current_user_ride_time<<" "<<user_ride_time_temp[save_p]<<endl;
								cout<<arrival_time_stop[v][k]<<" "<<departure_time_stop[v][i]<<endl;
								total_faking_error += user_ride_time_temp[save_p] - current_user_ride_time;
							}
							leave_loop = true;
							//}
							/*if (current_user_ride_time != user_ride_time[save_p]) {
								difference = current_user_ride_time -  user_ride_time[save_p];
								//<<"difference: "<<difference<<endl;
								total_user_ride_time += difference;
								//<<"total_user_ride_time: "<<total_user_ride_time<<endl;
								user_ride_time[save_p] = current_user_ride_time;
								l = number_passengers_action[v][k]+1;
								//k = number_stops[best_v]+2; //leave loop
								leave_loop = true;
							}*/
						}
					}
					if (leave_loop)
						k = number_stops[v]+2;
				}	
			} 
		}
	}
}

void check_valid_user_ride_times() {
	
	int new_total_user_ride_time = 0;
	int current_user_ride_time, difference;
	bool leave_loop = false;
	for (int v=0; v<total_number_vehicles; v++) {
		for (int i=0; i<=number_stops[v];i++) {
			for (int j=0; j<number_passengers_action[v][i];j++) {
				int save_p = action_passengers[v][i][j];
				for (int k=i+1; k<=number_stops[v];k++) {
					leave_loop = false;
					for (int l=0; l<number_passengers_action[v][k];l++) {
						if (action_passengers[v][k][l] == save_p){
							current_user_ride_time = arrival_time_stop[v][k] - departure_time_stop[v][i];
							new_total_user_ride_time += current_user_ride_time;
							if (current_user_ride_time != user_ride_time[save_p]) {
								//difference = current_user_ride_time -  user_ride_time[save_p];
								
								//user_ride_time[save_p] = current_user_ride_time;
								cout<<"WRONG COMPUTED RIDE TIME!!!! passengerX "<<save_p<<" "<<current_user_ride_time<<" "<<user_ride_time[save_p]<<endl;
								cout<<"ERRORR"<<endl;
								
								l = number_passengers_action[v][k]+1;
								//k = number_stops[v]+2; //leave loop
								leave_loop = true;
								cout<<"leave"<<endl;
							}
						}
					}
					if (leave_loop)
						k = number_stops[v]+2;
				}	
			} 
		}
	}

	for (int i=0; i<maxpassengers; i++){

		if (assigned_to_3rd_party[i] == 1){
			new_total_user_ride_time += user_ride_time[i];
		}
	}

	//<<"URT: "<<new_total_user_ride_time<<" "<<total_user_ride_time<<endl;
}


void print_all_vehicles(){

	for (int v = 0; v<total_number_vehicles;v++) {
		for (int i=0; i<=number_stops[v];i++) {
			cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" [";
			for (int j=0; j<number_passengers_action[v][i];j++) 
				cout<<action_passengers[v][i][j]<<" ";
			cout<<"]  ";

			cout<<"{"<<arrival_time_stop[v][i]<<"} ";
			cout<<"{"<<departure_time_stop[v][i]<<"} ";
			cout<<"|"<<slack_time[v][i]<<"|  ";
			cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
		}
		cout<<endl<<endl;
	}
}

void compute_idle_times(){

	int idle_time_total = 0;


	for (int v=0; v<total_number_vehicles; v++){

		for (int i=1; i<number_stops[v]; i++){

			int idle_time =departure_time_stop[v][i] - arrival_time_stop[v][i];
			if (idle_time < 0)
				cout<<"NEGATIVE IDLE TIMEEEE"<<endl;
			idle_time_total += idle_time;
		}

	}

	cout<<"total idle times: "<<idle_time_total<<endl;
}

void simulated_annealing(int n_allocated) {

	double temperature, elapsed;
	int delta, relocate_p, nrep, no_improvement, type_move, count;
	//type_move 1 - accepeted and improving
	//type_move 2 - accepted and non improving
	//type_move 3 - rejected or not feasible move

	temperature = init_temperature;
	no_improvement = 0; 
	count = 0;
	start_time = get_wall_time();


	save_best_solution();

	while(true){

		nrep = 0;
		do {


			double y = (double)rand() / (double)RAND_MAX;

			if (y <= 0.5) {
				//SWITCH
				relocate_p = rand() % n_allocated;
				if (vehicle_assigned[relocate_p] != -1) {
					if (passengers_departure_time_from_home[relocate_p] >= current_time) {
						//cout<<"relocate passenger SA: "<<relocate_p<<endl;
						relocate_passenger(relocate_p, temperature, type_move);
						if (relocate_p < 0){
							cout<<"MEGAERRORRRR1"<<endl;
							return;
						}
						// <<"vehicle_assigned SA: "<<vehicle_assigned[relocate_p]<<endl;
					}
				}

				/*cout<<"AFTER_SIMPLE RELOCATE"<<endl;
				for (int kk=0;kk<300;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}*/
			} else {

				//REDUCE
				//cout<<"heere"<<endl;
				select_vehicles_havent_that_can_be_turned_empty();
				//cout<<"hieerx";
				//<<"size vehicles at depot "<<vehicles_still_depot.size()<<endl;
				//for (int i=0; i<vehicles_still_depot.size();i++) {

				bool megaerror = false;
				int v;
				if (vehicles_still_depot.size() > 0) {
					int b = rand() % vehicles_still_depot.size();
					v = vehicles_still_depot[b];
				}
				
				if (passengers_at_vehicle[v].size() > 0) {
					empty_vehicle(v, megaerror, temperature, type_move);
				}
				if (megaerror){
					cout<<"MEGAERRORRRR2"<<endl;
					//return 0;
				}
					//relocate_all_passengers_vehicle(v, init_temperature, type_move);
				//}

				//return_best_solution();
				/*cout<<"AFTER_ALL RELOCATE"<<endl;
				for (int kk=0;kk<300;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}*/

			}

			if (type_move == 1) {
				no_improvement = 0;
			}

			if (type_move == 2) {
				no_improvement++;
			}

			if (total_user_ride_time < best_total_user_ride_time) {
				//cout<<"BEEST FOUND SO FAR"<<endl;
				save_best_solution();
				/*for (int kk=0;kk<total_requests;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}*/
				//<<"heerex"<<endl;
			}

			
			if (++count > 25) {
				elapsed = get_wall_time() - start_time;
				//<<"ELAPSED TIME "<<elapsed<<endl;
				if (elapsed > comp_time) {
					return;
				}
				count = 0;
			}

			nrep++;
		} while (nrep < maxnrep);

		maxnrep += increase_rep;
		temperature = init_temperature + lambda * log(1 + no_improvement);


	}
}


int main(int argc, char **argv) {


	comp_time = 0.10;
	start_time = get_wall_time();
	total_number_vehicles = 0;
	for (int i=1; i<argc; i++)
  	{
		if (strcmp(argv[i], "--filename_requests") == 0) {
			input_requests(argv[i+1]);
			cout<<argv[i+1]<<" ";
		} else if (strcmp(argv[i], "--filename_travel_time") == 0) {
			input_travel_time(argv[i+1]);

		} else if (strcmp(argv[i], "--depot") == 0) {
			for (int j = 0; j < number_depots; j++) {
				i++;
				depot[j] = stoi(argv[i]);
			}
		} else if (strcmp(argv[i], "--number_vehicles") == 0) {
			for (int j = 0; j < number_type_vehicles; j++) {
				i++;
				number_vehicles[j] = stoi(argv[i]);
				total_number_vehicles += number_vehicles[j];
			}
		} else if (strcmp(argv[i], "--capacity_vehicles") == 0) {
			for (int j = 0; j < number_type_vehicles; j++) {
				i++;
				maxcapacity[j] = stoi(argv[i]);
			}
		} else if (strcmp(argv[i], "--type_vehicles") == 0) {
			number_type_vehicles = stoi(argv[i+1]);
		} else if (strcmp(argv[i], "--number_depots") == 0) {
			number_depots = stoi(argv[i+1]);
		} else if (strcmp(argv[i], "--init_temperature") == 0) {    		
    		init_temperature = atof(argv[i+1]);
    		i++;
    	} else if (strcmp(argv[i], "--lambda") == 0) {    		
    		lambda = atof(argv[i+1]);
    		i++;
    	} else if (strcmp(argv[i], "--maxnrep") == 0) {
      		maxnrep = atoi(argv[i+1]);
    		i++;
    	} else if (strcmp(argv[i], "--increase_rep") == 0) {
      		increase_rep = atoi(argv[i+1]);
    		i++;
    	} else if (strcmp(argv[i], "--total_requests") == 0) {
      		total_requests = atoi(argv[i+1]);
    		i++;
    	} else if (strcmp(argv[i], "--seed") == 0) {
      		seed = atoi(argv[i+1]);
    		i++;
    	}

	}

	srand(seed);
	cout<<total_requests<<" "<<seed<<" ";

	

	for (int i = 0; i <= maxvehicles; i++) {
   		stops[i].resize(2);
   		number_passengers_action[i].resize(2);
   		arrival_time_stop[i].resize(2);
   		departure_time_stop[i].resize(2);
   		slack_time[i].resize(2);
   		free_capacity[i].resize(2);
   	}

   	for (int i = 0; i <= maxvehicles; i++) {
   		action_passengers[i].resize(2);
   		for (int j = 0; j < 2; j++) {
   			action_passengers[i][j].resize(1);
   		}
   	}

   	for (int i = 0; i < number_depots; i++) {
   		number_vehicles_at_depot[i] = 0;
   	}

   	//for (int i = 0; i <= maxvehicles; i++)
   	//	action_passengers[i].resize(maxpassengers + 1);

   	
	current_time = time_stamp[0]; //time stamp of the first passenger is the current time on the system
	int current_passenger;
	//empty_vehicle = 0;


	for (int i=0; i<maxpassengers; i++){
		route_assigned[i] = 0;
		vehicle_assigned[i] = -1;
		delay[i] = 0;
		assigned_to_3rd_party[i] = 0;
	}

	int k = 0;

	//initialize route of vehicles at the depot
	for (int j=0; j<number_type_vehicles; j++) {

		for (int i=0; i<number_vehicles[j];i++) {
			
			//total_capacity[i] = maxcapacity;
			//max_capacity[i] = maxcapacity;

			arrival_time_stop[k][0] = 0;
			arrival_time_stop[k][1] = 86400;

			departure_time_stop[k][0] = 0;
			departure_time_stop[k][1] = 86400;

			slack_time[k][0] = 86400;
			slack_time[k][1] = 86400;

			//free_capacity[i] = maxcapacity;
			free_capacity[k][0] = maxcapacity[j];
			free_capacity[k][1] = maxcapacity[j];

			blocked_vehicles[k] = 0;

			//assign "randomly" a depot to the vehicle
			vehicle_located_at_depot[k] = rand() % number_depots; 

			vehicles_at_depot[vehicle_located_at_depot[k]][number_vehicles_at_depot[vehicle_located_at_depot[k]]] = k;
			number_vehicles_at_depot[vehicle_located_at_depot[k]]++;

			stops[k][0] = depot[vehicle_located_at_depot[k]];
			stops[k][1] = depot[vehicle_located_at_depot[k]];
			number_stops[k] = 1; 

			current_position[k] = 0;

			vehicle_type[k] = j;

			k++;

		}
	}
	//total_capacity[number_vehicles] = maxcapacity;
	//max_capacity[number_vehicles] = maxcapacity;

	//solution_cost = 0;
	current_passenger = 0;
	served_passengers = 0;
	served_passengers_3party = 0;
	total_served_passengers = 0;

	//initially a quarter of the price of 3rd party (taxi/uber)
	for (int i = 0; i < number_type_vehicles; i++) {
		ODB_booking_fee[i] = 0.41;
		ODB_base_fare[i] = 0.41;
		ODB_per_minute_charge[i] = 0.07;
		ODB_km_charge[i] = 0.20;
		ODB_surge_multiplier[i] = 1;
		ODB_minimum_fare[i] = 1.10;
	}

	tp_booking_fee = 1.63;
	tp_base_fare = 1.65;
	tp_per_minute_charge = 0.26;
	tp_km_charge = 0.47;
	tp_surge_multiplier = 1;
	tp_minimum_fare = 4.38;

	//for (int k=0;k<300;k++){
	//	time_stamp[k] = time_stamp[0];
	//}

	for (int v = 0; v < total_number_vehicles; v++) {
		for (int s = 0; s < maxpassengers; s++) {
			blocked_positions[v][s] = 0;
		}
	}
	
	total_user_ride_time = 0;

	max_flex_delay = 0;

	for (int k=0;k<total_requests;k++){
		latest_arrival[k] = latest_arrival[k] + 3600;
	}

	for (int k=0;k<total_requests;k++){
		current_time = time_stamp[k];
		//<<"current time: "<<current_time<<endl;
		curr_number_insertions = 0;
		//cheapest FEASIBLE insertion
		if (current_passenger < 0) {
			cout<<"MEGAERRORRRR1"<<endl;
			return 0;
		}

		//cout<<"insertion "<<current_passenger<<endl;
		cheapest_insertion_randomized(current_passenger);

		for (int kk=0;kk<k;kk++){
			if (vehicle_assigned[kk] != -1) {
				solution_validation(kk, vehicle_assigned[kk]);
			//served_passengers++;
			}
		}

		//<<"xxxheeerexxxx1"<<endl;
		if (k > total_number_vehicles + 10) {
			simulated_annealing(k);
			return_best_solution();
		}
		//<<"xxxheeerexxxx2"<<endl;

		//to remove all passengers from a vehicle
		//possibly delta is being computed wrong
		/*if (k > total_number_vehicles + 10) {

			save_best_solution();
			//<<"xxxheeerexxxx3"<<endl;
			select_vehicles_havent_that_can_be_turned_empty();
			cout<<"size vehicles at depot "<<vehicles_still_depot.size()<<endl;
			for (int i=0; i<vehicles_still_depot.size();i++) {
				int v = vehicles_still_depot[i];
				bool megaerror = false;
				if (passengers_at_vehicle[v].size() > 0)
					empty_vehicle(v, megaerror, k);
				if (megaerror){
					cout<<"MEGAERRORRRR2"<<endl;
					return 0;
				}
				//relocate_all_passengers_vehicle(v, init_temperature, type_move);
				if (total_user_ride_time < best_total_user_ride_time) {
					cout<<"2BEEST FOUND SO FAR2"<<endl;
					save_best_solution();
				}
			}

			return_best_solution();
			cout<<"AFTER_ALL RELOCATE"<<endl;
			for (int kk=0;kk<k;kk++){
				if (vehicle_assigned[kk] != -1) {
					solution_validation(kk, vehicle_assigned[kk]);
				//served_passengers++;
				}
			}
		}*/

		
		
		//check_valid_user_ride_times();
		/*served_passengers = 0;
		for (int l=0;l<k;l++){
			if (vehicle_assigned[l] != -1) {
				served_passengers++;
			}
		}
		cout << "served passengers ODB " << served_passengers << endl;*/


		/*if (k > total_number_vehicles + 10) {
			for (int l=0;l<3;l++){
				int relocate_p = rand() % k;
				if (vehicle_assigned[relocate_p] != -1)
					relocate_passenger(relocate_p);
			}
			bool valid;
			for (int m=0;m<=k;m++){
				if (vehicle_assigned[m] != -1) {
					valid = solution_validation(m, vehicle_assigned[m]);
					if (not valid)
						return 0;
				}
			}
			
		}*/
		//cheapest_insertion(current_passenger);
		
		//update current position that still modifications will be able to happen
		/*for (int v = 0; v < total_number_vehicles; v++) {
			for (int j = current_position[v]; j < number_stops[v]; j++) {
				if (departure_time_stop[v][j] >= current_time) {
					current_position[v] = j; 
					break;
				}
			}
		}*/
		current_passenger++;
		//<<endl;
		//if (slack_time[19][1] < 0)
		//	break;
	}

	check_valid_user_ride_times();
	//<<"ALL VEHICLES"<<endl;
	//print_all_vehicles();



	

	for (int k=0;k<total_requests;k++){
		if (vehicle_assigned[k] != -1) {
			solution_validation(k, vehicle_assigned[k]);
			served_passengers++;
		}
	}
	/*cout << "served passengers ODB " << served_passengers << endl;
	cout << "served passengers 3rd party: " << served_passengers_3party << endl;*/
	total_served_passengers = served_passengers + served_passengers_3party;
	/*cout << "served passengers total: " << total_served_passengers << endl;
	cout << "solution cost (total user ride time): "<<total_user_ride_time<<endl;
	cout << "BEST solution cost (total user ride time): "<<best_total_user_ride_time<<endl<<endl;*/
	cout << served_passengers << " " << served_passengers_3party << " " << total_served_passengers << " " <<total_user_ride_time << " " << best_total_user_ride_time << endl;
	//compute_idle_times();

	elapsed = get_wall_time() - start_time;	 
}
