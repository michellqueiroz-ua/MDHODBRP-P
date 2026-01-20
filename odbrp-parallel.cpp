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
#include <random>
#include <omp.h>
#include <set>
#include <sys/resource.h>
#include <chrono>
#include <cassert>
//#include<Eigen/Dense>
//using namespace Eigen;
using namespace std;

#define maxvehicles 10000
#define maxpassengers 3000
#define maxstations 6000
#define maxtotalcapacity 40
#define maxtypevehicles 40
#define maxnumberdepots 10
#define number_clusters 4

typedef int listP[21000 + 1];
//typedef int matrixVP[maxvehicles + 1][maxpassengers + 1];
typedef int matrixPV[21000 + 1][maxvehicles + 1];
typedef int matrixVS[maxvehicles + 1][maxstations + 1];
typedef int matrixVC[maxvehicles + 1][maxtotalcapacity*2];
typedef int matrixPS[21000 + 1][maxstations + 1];
typedef int matrixSS[maxstations + 1][maxstations + 1];
typedef int matrixDV[maxnumberdepots + 1][maxvehicles + 1];
typedef int listS[maxstations + 1];
typedef int listV[maxvehicles + 1];
typedef int listD[maxnumberdepots + 1];
typedef int listT[maxtypevehicles + 1];
typedef double listTd[maxtypevehicles + 1];
typedef int matrixVSS[maxvehicles + 1][maxstations + 1][maxstations + 1]; //to know which stations are visited after another (in case its necessary)
//typedef int matrixVPP[maxvehicles + 1][maxpassengers + 1][maxtotalcapacity + 1];
//typedef int matrixPW[maxpassengers + 1][number_clusters + 1];
typedef int matrixVR[maxvehicles + 1][25000];
typedef int matrixVRC[maxvehicles + 1][25000][80];

//int n;

static double comp_time;

static double rejected_request_rate;
static int served_requests_so_far, rejected_requests_so_far;

//intermidate solution saving
vector<vector<int> > intm_stops(maxvehicles + 1);
vector<vector<vector<int> > > intm_action_passengers(maxvehicles + 1);
vector<vector<int> > intm_number_passengers_action(maxvehicles + 1);
vector<vector<int> > intm_arrival_time_stop(maxvehicles + 1); 
vector<vector<int> > intm_departure_time_stop(maxvehicles + 1);
vector<vector<int> > intm_slack_time(maxvehicles + 1); 
vector<vector<int> > intm_free_capacity(maxvehicles + 1);
static listP intm_user_ride_time;
static listV intm_number_stops;
static listP intm_vehicle_assigned, intm_assigned_to_3rd_party;
static listP intm_passengers_departure_time_from_home;
//int intm_served_passengers;
//int intm_served_passengers_3party;
//int intm_total_served_passengers;
//int intm_total_user_ride_time;

//best solution saving
vector<vector<int> > best_stops(maxvehicles + 1);
vector<vector<vector<int> > > best_action_passengers(maxvehicles + 1);
vector<vector<int> > best_number_passengers_action(maxvehicles + 1);
vector<vector<int> > best_arrival_time_stop(maxvehicles + 1); 
vector<vector<int> > best_departure_time_stop(maxvehicles + 1);
vector<vector<int> > best_slack_time(maxvehicles + 1); 
vector<vector<int> > best_free_capacity(maxvehicles + 1);
static listP best_user_ride_time;
static listV best_number_stops;
static listV next_free_position_passenger;
static listP best_vehicle_assigned, best_assigned_to_3rd_party;
static listP best_passengers_departure_time_from_home;

//int best_total_served_passengers;
//int best_served_passengers_3party;
//int best_served_passengers;

static listP vehicle_assigned;
static listP user_ride_time, assigned_to_3rd_party;

bool priority_empty_vehicle = false;

static int served_passengers;
static int served_passengers_3party;
static int total_served_passengers;
static int total_user_ride_time;
//int oldy_urt;
static int best_total_user_ride_time;

static matrixPV blocked_vehicles, removed_passenger;

//matrixVP stopi, stopj; //route of the bus: stopj is visited after stopi)
vector<vector<int> > stops(maxvehicles + 1);
static listV number_stops; //number of stops vehicle v is performing for each route
vector<vector<int> > number_passengers_action(maxvehicles + 1); //store number of passengers performing an action at given vehicle and given stop
vector<vector<int> > arrival_time_stop(maxvehicles + 1); //stores which time the stop was/will be visited by the bus
vector<vector<int> > departure_time_stop(maxvehicles + 1);
vector<vector<int> > slack_time(maxvehicles + 1);
vector<vector<int> > free_capacity(maxvehicles + 1);
vector<vector<vector<int> > > action_passengers(maxvehicles + 1); //store which passengers are performing an action at given vehicle and given stop
vector<vector<int> > passengers_at_vehicle(maxvehicles + 1);

static int total_requests;
static int seed;

vector<int> centroids;
map<int, int> centroids_keys;
vector<vector<int> > clusters(number_clusters);
vector<int> best_tot_cluster_ride_time(number_clusters);

vector<int> passengers_on_hold;

static matrixVC arrival_time_stop_temp, departure_time_stop_temp;
static listP user_ride_time_temp, user_ride_time_start, affected_passengers;
static std::vector<std::chrono::high_resolution_clock::time_point> response_time_request;
// At top with other declarations:
static std::vector<std::chrono::high_resolution_clock::time_point> request_time_stamp;
static std::vector<bool> request_timestamp_captured;

static listV vehicle_type;
static listV current_position;
static listD depot;
static listT number_vehicles;
static listD number_vehicles_at_depot;
static listD number_empty_vehicles_at_depot;
static matrixDV vehicles_at_depot;
static int total_number_vehicles;
static listT maxcapacity;
static listV vehicle_located_at_depot;

static listV mindDist;
static listV cluster;

static int number_type_vehicles;
static int number_depots;

static matrixSS travel_time;
static listS stations_ids;
//map<int, int> station_id_map;

//information about the requests
static listP time_stamp, earliest_departure, latest_departure, latest_arrival, direct_travel_time;
static listP waiting_time;
static listP delay;
static listP passengers_departure_time_from_home;
static matrixPS stops_origin, stops_destination;
static matrixPS walking_time_stops_origin, walking_time_stops_destination;
static listP number_stops_origin, number_stops_destination;
static listP already_opened_vehicle_for_it;
static int number_stations;

matrixPS stops_uneven;
listP number_stops_uneven;

static int extra_travel_time;
static double passengers_per_kilometer, average_travel_time_ratio;
static int total_deadheading_times, total_shared_times, total_per_vehicle_travel_time;
static double average_response_time_new_requests;
static double average_waiting_time;

static int current_time;
static clock_t start_time;
static clock_t begin_time;
static double elapsedf;
static int max_flex_delay;

/*listS saved_arrival_time, saved_departure_time, saved_slack_time;
int saved_number_stops_v;
vector<int> saved_stops2;
vector<int> saved_number_passengers_action2;
vector<std::vector<int> > saved_action_passengers2;
std::vector<int> saved_arrival_time2, saved_departure_time2, saved_slack_time2, saved_free_capacity2;*/

static matrixVS saved_arrival_time, saved_departure_time, saved_slack_time;
static listV saved_number_stops_v;
static matrixVR saved_stops2, saved_number_passengers_action2;
static matrixVR saved_arrival_time2, saved_departure_time2, saved_slack_time2, saved_free_capacity2;
static matrixVRC saved_action_passengers2;

static listTd ODB_booking_fee, ODB_base_fare, ODB_per_minute_charge, ODB_km_charge, ODB_surge_multiplier, ODB_minimum_fare;
static double tp_booking_fee, tp_base_fare, tp_per_minute_charge, tp_km_charge, tp_surge_multiplier, tp_minimum_fare;

//SA parameters
static double init_temperature, lambda;
static int maxnrep, ntrials, increase_rep;

listV avg_occupancy;
int overall_max_capacity, overall_max_capacity4, overall_max_capacity8, overall_max_capacity12;
double overall_occupancy, overall_occupancy4, overall_occupancy8, overall_occupancy12;
listV running_max_capacity;

bool swap_vehicle;

struct Insertions {

	int increase_length;
	int pos_station;
	int sel_station;
	int v;
	int delay;
	bool repeated_station;
	int passengers_departure_time_from_home;
};

//Insertions insertions[25000];
//int curr_number_insertions; 

struct SortClusters {

	int idx_cluster;
	int mean_dist;
	

};


struct SortPassengers {

	int k;
	int time_stamp;
	

};
static SortPassengers sort_passengers[21000 + 1];

static SortClusters sort_clusters[21000 + 1][number_clusters + 1];
static int it_cl_inser[22000];
static int att_inser[22000];

// Fail-fast bounds checks (keeps the crash close to the real bug)
static inline void CHECK_IDX(long long idx, long long size, const char* what, const char* file, int line) {
    if (idx < 0 || idx >= size) {
        std::cerr << "OOB: " << what << " idx=" << idx << " size=" << size
                  << " at " << file << ":" << line << std::endl;
        std::abort();
    }
}
#define CHECK_INDEX(idx, size, what) CHECK_IDX((idx), (size), (what), __FILE__, __LINE__)

bool comparator( Insertions a, Insertions b){
	if(a.increase_length < b.increase_length)
		return 1;
	else 
		return 0;
}

bool comparator2( SortClusters a, SortClusters b){
	if(a.mean_dist < b.mean_dist)
		return 1;
	else 
		return 0;
}

bool comparator3( SortPassengers a, SortPassengers b){
	if(a.time_stamp < b.time_stamp)
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
							//total_user_ride_time += difference;
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

void print_v_vehicle(int v){

	//for (int v = 0; v<total_number_vehicles;v++) {
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
	//}
}

bool solution_validation(int p, int v){


	bool pick_up = true;
	bool valid_solution = true;
	int count = 0;
	for (int i=0; i<number_stops[v];i++) {

		if (number_stops[v] + 1 != departure_time_stop[v].size()){
			cout<<"weird here"<<endl;
			cout<<"vehicle "<<v<<endl;
			cout<<number_stops[v]<<" "<<departure_time_stop[v].size()<<" "<<number_passengers_action[v].size()<<" "<<arrival_time_stop[v].size()<<" "<<slack_time[v].size()<<" "<<free_capacity[v].size()<<endl<<endl;
		}
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

					int slaack = latest_arrival[p] - arrival_time_stop[v][i];

					if (slaack < slack_time[v][i]){
						cout<<"slack computed wrong p: "<<p<<endl;
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
	//<<"passed1 "<<endl;
	if (not valid_solution) {
		
		//<<travel_time[1174][3081]<<" "<<travel_time[3081][2142]<<" "<<travel_time[1174][2142]<<" "<<endl;
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
	//<<"passed2 "<<endl;

	return true;
}

void save_best_solution(int cluster_id){

	best_tot_cluster_ride_time[cluster_id] = 0;
	//<<"A"<<endl;
	for (int j=0;j<clusters[cluster_id].size();j++){

		int v = clusters[cluster_id][j];
		//<<"B"<<endl;
		best_stops[v] = stops[v];
		//<<"C"<<endl;
		//std::copy(std::begin(action_passengers[v]), std::end(action_passengers[v]), std::begin(best_action_passengers[v]));
		best_action_passengers[v] = action_passengers[v];
		//<<"D"<<endl;
		best_number_passengers_action[v] = number_passengers_action[v];
		best_arrival_time_stop[v] = arrival_time_stop[v];
		//<<"E"<<endl;
		best_departure_time_stop[v] = departure_time_stop[v];
		best_slack_time[v] = slack_time[v];
		//xxcout<<"F"<<endl;
		best_free_capacity[v] = free_capacity[v];
		//std::copy(std::begin(number_stops), std::end(number_stops), std::begin(best_number_stops));
		best_number_stops[v] = number_stops[v];
		
		
		for (int p=0;p<total_requests;p++) {
			if (vehicle_assigned[p] == v) {

				best_passengers_departure_time_from_home[p] = passengers_departure_time_from_home[p];
				//std::copy(std::begin(passengers_departure_time_from_home), std::end(passengers_departure_time_from_home), std::begin(best_passengers_departure_time_from_home));
				best_user_ride_time[p] = user_ride_time[p];
				best_tot_cluster_ride_time[cluster_id] += user_ride_time[p];
				//std::copy(std::begin(user_ride_time), std::end(user_ride_time), std::begin(best_user_ride_time));
				best_vehicle_assigned[p] = vehicle_assigned[p];
				//std::copy(std::begin(vehicle_assigned), std::end(vehicle_assigned), std::begin(best_vehicle_assigned));
				best_assigned_to_3rd_party[p] = assigned_to_3rd_party[p];
				//std::copy(std::begin(assigned_to_3rd_party), std::end(assigned_to_3rd_party), std::begin(best_assigned_to_3rd_party));
			}
		}

	}

	/*best_served_passengers = served_passengers;
	best_served_passengers_3party = served_passengers_3party;
	best_total_served_passengers = total_served_passengers;
	best_total_user_ride_time = total_user_ride_time;*/
}

void return_best_solution(int cluster_id){

	for (int j=0;j<clusters[cluster_id].size();j++){

		int v = clusters[cluster_id][j];
		stops[v] = best_stops[v];
		action_passengers[v] = best_action_passengers[v];
		//std::copy(std::begin(best_action_passengers[v]), std::end(best_action_passengers[v]), std::begin(action_passengers[v]));
		number_passengers_action[v] = best_number_passengers_action[v];
		arrival_time_stop[v] = best_arrival_time_stop[v];
		departure_time_stop[v] = best_departure_time_stop[v];
		slack_time[v] = best_slack_time[v];
		free_capacity[v] = best_free_capacity[v];
		number_stops[v] = best_number_stops[v];
		//std::copy(std::begin(best_number_stops), std::end(best_number_stops), std::begin(number_stops));

		for (int p=0;p<total_requests;p++) {
			if (vehicle_assigned[p] == v) {
				passengers_departure_time_from_home[p] = best_passengers_departure_time_from_home[p];
				//std::copy(std::begin(best_passengers_departure_time_from_home), std::end(best_passengers_departure_time_from_home), std::begin(passengers_departure_time_from_home));
				user_ride_time[p] = best_user_ride_time[p];
				//std::copy(std::begin(best_user_ride_time), std::end(best_user_ride_time), std::begin(user_ride_time));
				vehicle_assigned[p] = best_vehicle_assigned[p];
				//std::copy(std::begin(best_vehicle_assigned), std::end(best_vehicle_assigned), std::begin(vehicle_assigned));
				assigned_to_3rd_party[p] = best_assigned_to_3rd_party[p];
				//std::copy(std::begin(best_assigned_to_3rd_party), std::end(best_assigned_to_3rd_party), std::begin(assigned_to_3rd_party));
			}
		}

	}

	/*served_passengers = best_served_passengers;
	served_passengers_3party = best_served_passengers_3party;
	total_served_passengers = best_total_served_passengers;
	total_user_ride_time = best_total_user_ride_time;*/
}

void save_intm_solution(int cluster_id){

	for (int j=0;j<clusters[cluster_id].size();j++){		
		//vehicle info
		int v = clusters[cluster_id][j];
		intm_stops[v] = stops[v];
		//std::copy(std::begin(action_passengers[v]), std::end(action_passengers[v]), std::begin(intm_action_passengers[v]));
		intm_action_passengers[v] = action_passengers[v];
		intm_number_passengers_action[v] = number_passengers_action[v];
		intm_arrival_time_stop[v] = arrival_time_stop[v];
		intm_departure_time_stop[v] = departure_time_stop[v];
		intm_slack_time[v] = slack_time[v];
		intm_free_capacity[v] = free_capacity[v];
		//std::copy(std::begin(number_stops[v]), std::end(number_stops[v]), std::begin(intm_number_stops[v]));
		intm_number_stops[v] = number_stops[v];

		for (int p=0;p<total_requests;p++) {
			if (vehicle_assigned[p] == v) {
				//passengers info
				//std::copy(std::begin(passengers_departure_time_from_home), std::end(passengers_departure_time_from_home), std::begin(intm_passengers_departure_time_from_home));
				intm_passengers_departure_time_from_home[p] = passengers_departure_time_from_home[p];
				//std::copy(std::begin(user_ride_time), std::end(user_ride_time), std::begin(intm_user_ride_time));
				intm_user_ride_time[p] = user_ride_time[p];
				//std::copy(std::begin(vehicle_assigned), std::end(vehicle_assigned), std::begin(intm_vehicle_assigned));
				//std::copy(std::begin(assigned_to_3rd_party), std::end(assigned_to_3rd_party), std::begin(intm_assigned_to_3rd_party));
				intm_vehicle_assigned[p] = vehicle_assigned[p];
				intm_assigned_to_3rd_party[p] = assigned_to_3rd_party[p];

			}
		}

	}

	//overall solution info
	/*intm_served_passengers = served_passengers;
	intm_served_passengers_3party = served_passengers_3party;
	intm_total_served_passengers = total_served_passengers;
	intm_total_user_ride_time = total_user_ride_time;*/
}

void return_intm_solution(int cluster_id){

	for (int j=0;j<clusters[cluster_id].size();j++){

		int v = clusters[cluster_id][j];
		stops[v] = intm_stops[v];
		//std::copy(std::begin(intm_action_passengers[v]), std::end(intm_action_passengers[v]), std::begin(action_passengers[v]));
		action_passengers[v] = intm_action_passengers[v];
		number_passengers_action[v] = intm_number_passengers_action[v];
		arrival_time_stop[v] = intm_arrival_time_stop[v];
		departure_time_stop[v] = intm_departure_time_stop[v];
		slack_time[v] = intm_slack_time[v];
		free_capacity[v] = intm_free_capacity[v];
		number_stops[v] = intm_number_stops[v];
		
		for (int p=0;p<total_requests;p++) {
			if (vehicle_assigned[p] == v) {

				passengers_departure_time_from_home[p] = intm_passengers_departure_time_from_home[p];
				//std::copy(std::begin(intm_passengers_departure_time_from_home), std::end(intm_passengers_departure_time_from_home), std::begin(passengers_departure_time_from_home));
				user_ride_time[p] = intm_user_ride_time[p];
				//std::copy(std::begin(intm_user_ride_time), std::end(intm_user_ride_time), std::begin(user_ride_time));
				vehicle_assigned[p] = intm_vehicle_assigned[p];
				//std::copy(std::begin(intm_vehicle_assigned), std::end(intm_vehicle_assigned), std::begin(vehicle_assigned));
				assigned_to_3rd_party[p] = intm_assigned_to_3rd_party[p];
				//std::copy(std::begin(intm_assigned_to_3rd_party), std::end(intm_assigned_to_3rd_party), std::begin(assigned_to_3rd_party));
			}

		}

	}

	//served_passengers = intm_served_passengers;
	//served_passengers_3party = intm_served_passengers_3party;
	//total_served_passengers = intm_total_served_passengers;
	//total_user_ride_time = intm_total_user_ride_time;
}

void save_vehicle_p(int v){

	saved_number_stops_v[v] = number_stops[v];

	//<<number_stops[v]<<" "<<endl;
	for (int i=0; i<=number_stops[v];i++) {
		saved_stops2[v][i] = stops[v][i];
		saved_number_passengers_action2[v][i] = number_passengers_action[v][i];
		
		//<<number_passengers_action[v][i]<<endl;
		for (int j=0; j<number_passengers_action[v][i];j++) {
			 saved_action_passengers2[v][i][j] = action_passengers[v][i][j];
		}
		
		saved_arrival_time2[v][i] = arrival_time_stop[v][i];
		saved_departure_time2[v][i] = departure_time_stop[v][i];
		saved_slack_time2[v][i] = slack_time[v][i];
		saved_free_capacity2[v][i] = free_capacity[v][i];
	}	
}

void return_save_vehicle_p(int v){

	number_stops[v] = saved_number_stops_v[v];

	while (stops[v].size() != number_stops[v] + 1) {

		if (stops[v].size() > number_stops[v] + 1) {

			stops[v].erase(stops[v].begin());
			number_passengers_action[v].erase(number_passengers_action[v].begin());
			action_passengers[v].erase(action_passengers[v].begin());
			arrival_time_stop[v].erase(arrival_time_stop[v].begin());
			departure_time_stop[v].erase(departure_time_stop[v].begin());
			slack_time[v].erase(slack_time[v].begin());
			free_capacity[v].erase(free_capacity[v].begin());

		} else {

			if (stops[v].size() < number_stops[v] + 1) {

				stops[v].insert(stops[v].begin(), 0);
				number_passengers_action[v].insert(number_passengers_action[v].begin(), 0);
				action_passengers[v].insert(action_passengers[v].begin(), vector<int>());
				action_passengers[v][0].resize(15);
				arrival_time_stop[v].insert(arrival_time_stop[v].begin(), 0);
				departure_time_stop[v].insert(departure_time_stop[v].begin(), 0);
				slack_time[v].insert(slack_time[v].begin(), 0);
				free_capacity[v].insert(free_capacity[v].begin(), 0);

			}
		}

	}


	for (int i=0; i<=number_stops[v];i++) {
		stops[v][i] = saved_stops2[v][i];
		number_passengers_action[v][i] = saved_number_passengers_action2[v][i];
		for (int j=0; j<number_passengers_action[v][i];j++) 
			action_passengers[v][i][j] = saved_action_passengers2[v][i][j];
		
		arrival_time_stop[v][i] = saved_arrival_time2[v][i];
		departure_time_stop[v][i] = saved_departure_time2[v][i];
		slack_time[v][i] = saved_slack_time2[v][i];
		free_capacity[v][i] = saved_free_capacity2[v][i];
	}
}

/*void update_departure_time_empty_vehicles(){

	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v].size() == 2) {
			departure_time_stop[v][0] = current_time;
		}
	}
}*/

void input_travel_time(char *filename) {

	fstream file (filename, ios::in);
	string line, data;
	int s, stop1, stop2;
	//<<filename<<endl;
	if(file.is_open())
	{
		/*getline(file, line);
		stringstream str(line);
		s = 0;
		getline(str, data, ',');

		int count = 0;
		while(getline(str, data, ',')) { //reads the header

			//if (count > 0) {
				stations_ids[s] = stoi(data);
				//<<stations_ids[s]<<" "<<endl;
				//station_map.insert(pair<int, int>(stations_ids[s], s));
				station_id_map[stations_ids[s]] = s;
				s = s + 1;
			//}
			count++;
		}*/
		//<<endl;


		stop1 = 0;
		getline(file, line);

		while(getline(file, line))
		{

			stringstream str(line);
			stop2 = 0;
			//getline(str, data, ',');
			//<<data<<endl;
			
			//count = 0;
			int count = 0;
			while(getline(str, data, ',')) {
				//cout<<data<<" ";
				if (count > 0) {
					travel_time[stop1][stop2] = stoi(data);
					travel_time[stop2][stop1] = stoi(data);
					stop2 = stop2 + 1;
				} 
				count++;
			}
			//cout<<endl<<endl;
			//break;
			//<<stop1<<" "<<stop2<<endl;
			stop1 = stop1 + 1;

		}

		for (int i=0;i<stop1;i++){
			stations_ids[i] = i;
		}
		//<<"count: "<<count<<endl;
		int count2 =1;

		/*for(int i=0;i<stop1;i++) {
			for (int j=0;j<stop1;j++) {
				if (travel_time[i][j] == 0) {
					cout<<i<<" "<<j<<endl;
		  			// << travel_time[i][j] <<',';
		  		}
		  	}
		   cout <<endl;
		}*/

		//<<"xx: "<<travel_time[2950][214]<<" "<<travel_time[214][2716]<<" "<<travel_time[2950][2716]<<endl;

		/*cout<<endl;
		while (count2 > 0) {
		
			count2 = 0;

			for (int i=0;i<stop1;i++){
				//<<i<<endl;
				for (int j=0;j<stop1;j++){
					for(int k=0;k<stop1;k++){
						if (travel_time[i][j] > travel_time[i][k] + travel_time[k][j]){
							travel_time[i][j] = travel_time[i][k] + travel_time[k][j];
							travel_time[j][i] = travel_time[i][j];
							count2++;
							//<<"err "<<i<<" "<<j<<" "<<k<<endl;
						}
					}
				}
			}

			cout<<count2<<" cc"<<endl;

		}


		std::ofstream out("travel_time_updated4.csv");

		for(int i=0;i<stop1;i++) {
		  for (int j=0;j<stop1;j++)
		    out << travel_time[i][j] <<',';
		  out << '\n';
		}*/
		//saveData("travel_time_matrix.csv", travel_time);
		//<<"out"<<endl;

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
		total_requests = 0;
		while(getline(file, line))
		{

			total_requests++;
			stringstream str(line);
			getline(str, data, ',');
			p = stoi(data);
			//<<p<<" ";
			
			getline(str, data, ',');
			time_stamp[p] = stoi(data);
			//<<time_stamp[p]<<" ";

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
			//<<earliest_departure[p]<<" ";
			
			getline(str, data, ',');
			latest_departure[p] = stoi(data);
			//printf("%d\n", latest_departure[p]);
			//<<latest_departure[p]<<" ";

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
			//<<stops_origin[p][s]<<" ";
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
					//<<stops_origin[p][s]<<" ";
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
			//<<walking_time_stops_origin[p][s]<<" ";
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
					//<<walking_time_stops_origin[p][s]<<" ";
					//printf("%d ", stops_origin[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			
			
			getline(str, data, ',');
			latest_arrival[p] = stoi(data);
			//<<latest_arrival[p]<<" ";


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
			//<<stops_destination[p][s]<<" ";
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
					//<<stops_destination[p][s]<<" ";
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
			//<<walking_time_stops_destination[p][s]<<" ";
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
					//<<walking_time_stops_destination[p][s]<<" ";
					//printf("%d ", stops_destination[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			//printf("\n");
			//<<endl;
			getline(str, data, ',');
			getline(str, data, ',');

		}

	} else cout<<"Could not open the file\n";
}

void input_requests_commuting(char *filename) {

	fstream file (filename, ios::in);
	string line, data, stop;
	int p, s;

	int temp_time_stamp;
	int temp_earliest_departure;
	int temp_latest_departure;
	int temp_latest_arrival;

	if(file.is_open())
	{
		getline(file, line);
		stringstream str(line);
		while(getline(str, data, ',')); //reads the header
		//<<data<<endl;
		total_requests = 0;
		while(getline(file, line))
		{

			total_requests++;
			stringstream str(line);
			getline(str, data, ',');
			p = stoi(data);
			if (p < 0 || p > 21000) {
				cerr << "ERROR: passenger id out of range: " << p << endl;
				abort();
			}
			//<<p<<" ";
			
			getline(str, data, ',');
			time_stamp[p] = stoi(data);
			//<<time_stamp[p]<<" ";

			getline(str, data, ',');
			getline(str, data, ',');
			temp_time_stamp = stoi(data);
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
			//<<earliest_departure[p]<<" ";
			
			getline(str, data, ',');
			latest_departure[p] = stoi(data);


			getline(str, data, ',');
			temp_earliest_departure = stoi(data);
			//printf("%d\n", earliest_departure[p]);
			//<<earliest_departure[p]<<" ";
			
			getline(str, data, ',');
			temp_latest_departure = stoi(data);
			//printf("%d\n", latest_departure[p]);
			//<<latest_departure[p]<<" ";

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
			//<<stops_origin[p][s]<<" ";
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
					//<<stops_origin[p][s]<<" ";
					//printf("%d ", stops_origin[p][s]);
					s = s + 1;
					if (s >= maxstations) {
						cerr << "ERROR: too many origin stops for p="<<p<<" (s="<<s<<")"<<endl;
						abort();
					}
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
			//<<walking_time_stops_origin[p][s]<<" ";
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
					//<<walking_time_stops_origin[p][s]<<" ";
					//printf("%d ", stops_origin[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			
			
			getline(str, data, ',');
			latest_arrival[p] = stoi(data);

			getline(str, data, ',');
			temp_latest_arrival = stoi(data);
			//<<latest_arrival[p]<<" ";


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
			//<<stops_destination[p][s]<<" ";
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
					//<<stops_destination[p][s]<<" ";
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
			//<<walking_time_stops_destination[p][s]<<" ";
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
					//<<walking_time_stops_destination[p][s]<<" ";
					//printf("%d ", stops_destination[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			//printf("\n");
			//<<endl;
			getline(str, data, ',');
			getline(str, data, ',');

			double y = (double)rand() / (double)RAND_MAX;

			if (y <= 0.6) {

				time_stamp[p] = temp_time_stamp;
				earliest_departure[p] = temp_earliest_departure;
				latest_departure[p] = temp_latest_departure;
				latest_arrival[p] = temp_latest_arrival;

			}

		}

	} else cout<<"Could not open the file\n";
}

void input_requests_festival(char *filename) {

	fstream file (filename, ios::in);
	string line, data, stop;
	int p, s;

	int temp_time_stamp;
	int temp_earliest_departure;
	int temp_latest_departure;
	int temp_latest_arrival;

	int temp_time_stamp2;
	int temp_earliest_departure2;
	int temp_latest_departure2;
	int temp_latest_arrival2;
	srand(1);
	if(file.is_open())
	{
		getline(file, line);
		stringstream str(line);
		while(getline(str, data, ',')); //reads the header
		//<<data<<endl;
		total_requests = 0;
		while(getline(file, line))
		{

			total_requests++;
			/*if (total_requests > 19060) {
				total_requests--;
				return;
			}*/
			stringstream str(line);
			getline(str, data, ',');
			p = stoi(data);
			//cout<<"p: "<<p<<endl;
			
			getline(str, data, ',');
			time_stamp[p] = stoi(data);
			//cout<<time_stamp[p]<<" ";

			getline(str, data, ',');
			getline(str, data, ',');
			temp_time_stamp = stoi(data);
			//cout<<temp_time_stamp<<" ";
			getline(str, data, ',');
			getline(str, data, ',');
			temp_time_stamp2 = stoi(data);
			//cout<<temp_time_stamp2<<" ";
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
			//cout<<earliest_departure[p]<<" ";
			//printf("%d\n", earliest_departure[p]);
			//<<earliest_departure[p]<<" ";
			
			getline(str, data, ',');
			latest_departure[p] = stoi(data);
			//cout<<latest_departure[p]<<" ";


			getline(str, data, ',');
			temp_earliest_departure = stoi(data);
			//cout<<temp_earliest_departure<<" ";
			//printf("%d\n", earliest_departure[p]);
			//<<earliest_departure[p]<<" ";
			
			getline(str, data, ',');
			temp_latest_departure = stoi(data);
			//cout<<temp_latest_departure<<" ";
			//printf("%d\n", latest_departure[p]);
			//<<latest_departure[p]<<" ";

			getline(str, data, ',');
			temp_earliest_departure2 = stoi(data);
			//cout<<temp_earliest_departure2<<" ";
			//printf("%d\n", earliest_departure[p]);
			//<<earliest_departure[p]<<" ";
			
			getline(str, data, ',');
			temp_latest_departure2 = stoi(data);
			//cout<<temp_latest_departure2<<" ";
			//printf("%d\n", latest_departure[p]);
			//<<latest_departure[p]<<" ";

			getline(str, data, ','); //direct travel time
			direct_travel_time[p] = stoi(data);
			//cout<<direct_travel_time[p]<<" ";
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
			//cout<<stops_origin[p][s]<<" ";
			//<<stops_origin[p][s]<<" ";
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
					//cout<<stops_origin[p][s]<<" ";
					//if (stops_origin[p][s] > 6000) {
					//	cout<<stops_origin[p][s]<<" ";
					//}
					//printf("%d ", stops_origin[p][s]);
					s = s + 1;
					//if (s >= 6000) {
					//	cout<<"error hier "<<" ";
					//}
					if (leave_loop)
						break;
				}
			}
			//printf("\n");
			//cout<<endl;
			number_stops_origin[p] = s;
			//cout<<number_stops_origin[p]<<" ";

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
			//cout<<walking_time_stops_origin[p][s]<<" ";
			//<<walking_time_stops_origin[p][s]<<" ";
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
					//cout<<walking_time_stops_origin[p][s]<<" ";
					//<<walking_time_stops_origin[p][s]<<" ";
					//printf("%d ", stops_origin[p][s]);
					s = s + 1;
					//if (s >= 6000) {
					//	cout<<"error hier "<<" ";
					//}
					if (leave_loop)
						break;
				}
			}
			
			
			getline(str, data, ',');
			latest_arrival[p] = stoi(data);
			//cout<<latest_arrival[p]<<" ";

			getline(str, data, ',');
			temp_latest_arrival = stoi(data);
			//cout<<temp_latest_arrival<<" ";

			getline(str, data, ',');
			temp_latest_arrival2 = stoi(data);
			//cout<<temp_latest_arrival2<<" ";
			//<<latest_arrival[p]<<" ";


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
			//cout<<stops_destination[p][s]<<" ";
			//printf("%d ", stops_destination[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					//cout<<s<<" ";
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					stops_destination[p][s] = stoi(stop);
					//if (stops_destination[p][s] > 6000)
					//cout<<stops_destination[p][s]<<" ";
					//printf("%d ", stops_destination[p][s]);
					s = s + 1;
					/*if (s >= 6000) {
						cout<<"error hier "<<" ";
					}*/
					if (leave_loop)
						break;
				}
			}
			//cout<<endl;
			//printf("\n");
			number_stops_destination[p] = s;
			//cout<<number_stops_destination[p]<<" "<<endl;

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
			//cout<<walking_time_stops_destination[p][s]<<" ";
			//printf("%d ", stops_destination[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					//cout<<s<<" ";
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					walking_time_stops_destination[p][s] = stoi(stop);
					//cout<<walking_time_stops_destination[p][s]<<" ";
					//printf("%d ", stops_destination[p][s]);
					s = s + 1;
					/*if (s >= 6000) {
						cout<<"error hier "<<" ";
					}*/
					if (leave_loop)
						break;
				}
			}
			//cout<<endl;
			//printf("\n");
			//<<endl;
			getline(str, data, ',');
			getline(str, data, ',');
			//cout<<endl;
			double y = (double)rand() / (double)RAND_MAX;

			if ((y >= 0.2) && (y <= 0.6)) {

				/*if (p > 19999) {
						cout<<"error hier "<<" ";
					}*/
				time_stamp[p] = temp_time_stamp;
				earliest_departure[p] = temp_earliest_departure;
				latest_departure[p] = temp_latest_departure;
				latest_arrival[p] = temp_latest_arrival;

			} else {

				if (y > 0.6) {

					/*if (p > 19999) {
						cout<<"error hier "<<" ";
					}*/
					time_stamp[p] = temp_time_stamp2;
					earliest_departure[p] = temp_earliest_departure2;
					latest_departure[p] = temp_latest_departure2;
					latest_arrival[p] = temp_latest_arrival2;


				}


			}

		}

	} else cout<<"Could not open the file\n";
	
	cout<<"exit oxe"<<endl;
}

void input_zones_uneven_demand(char *filename) {

	fstream file (filename, ios::in);
	//cout<<filename<<endl;
	string line, data, stop;
	int p, s;

	if(file.is_open())
	{
		//cout<<"hier"<<endl;
		getline(file, line);
		stringstream str(line);
		while(getline(str, data, ',')); //reads the header
		//<<data<<endl;
		//total_requests = 0;

		p = 0;
		while(getline(file, line))
		{
			//cout<<total_requests<<endl;
			//total_requests++;
			stringstream str(line);
			getline(str, data, ',');
			
			getline(str, data, ',');
			bool leave_loop = false;
			if (data.find(']') != std::string::npos)
					leave_loop = true; 
			data.erase(remove(data.begin(), data.end(), '['), data.end());
			data.erase(remove(data.begin(), data.end(), '"'), data.end());
			data.erase(remove(data.begin(), data.end(), ']'), data.end());
			s = 0;
			stops_uneven[p][s] = stoi(data);
			//printf("%d ", stops_origin[p][s]);
			s = s + 1;
			//stringstream str2(data);
			
			if (not (leave_loop)) {
				while(getline(str, stop, ',')) {
					
					if (stop.find(']') != std::string::npos)
						leave_loop = true; 
					data.erase(remove(data.begin(), data.end(), '"'), data.end());
					data.erase(remove(data.begin(), data.end(), ']'), data.end());
					stops_uneven[p][s] = stoi(stop);
					printf("%d ", stops_uneven[p][s]);
					s = s + 1;
					if (leave_loop)
						break;
				}
			}
			//printf("\n");
			number_stops_uneven[p] = s;
			p++;

		}


	} else cout<<"Could not open the file\n";
	//cout<<"out here"<<endl;
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

void cheapest_origin2_p(int p, int v, int &min_increase_length, int &sel_origin, int &pos_origin, bool &repeated_station, bool &flexibilize_lat_departure_time, Insertions (&insertions_p) [45000], int& curr_number_insertions_p){

	int remove_edge;
	int s_origin, increase;
	int pick_up_time;
	int feasible_insertion = false;
	int new_arrival_time, new_departure_time, new_slack_time, new_arrival_nxt_stop, old_arrival_nxt_stop, old_slack_time;
	int curr_dpt_time, latest_departure_passenger;
	int new_capacity;
	int departure_time_from_home;

	//curr_number_insertions_p = 0;
	//<<"vx "<<v<<endl;
	//look for cheapest insertion of origin
	for (int j=0;j<number_stops_origin[p];j++) {
		s_origin = stops_origin[p][j];
		for (int i=0; i<number_stops[v];i++) {

			//means that a detour can only be made if the bus has not departure yet from the stop
			//only future actions can be modified
			
			//if (i > 0)
			if (departure_time_stop[v][i] > 0)
				curr_dpt_time = departure_time_stop[v][i];
			else {
				curr_dpt_time = earliest_departure[p]-travel_time[stops[v][i]][s_origin];
				if (curr_dpt_time < current_time){
					curr_dpt_time = current_time;
				}
			}
			//<<"testY "<<endl;
			//<<"dptstimes "<<curr_dpt_time<<" "<<current_time<<endl;
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

				//<<"testZ "<<endl;
				//at what time the passenger will have to leave their home to pick up the bus on time?
				departure_time_from_home = new_departure_time-walking_time_stops_origin[p][j];
				//if the departure time from home is greater than or equal to current time, it means it is still a future event, which means it is feasible to happen

				if (old_arrival_nxt_stop == 86400)
					old_arrival_nxt_stop = 0;
				new_slack_time = old_slack_time - (new_arrival_nxt_stop - old_arrival_nxt_stop);
				
				//if (swap_vehicle)
			 		//cout<<"INSERTING IIN"<<i<<" "<<old_slack_time<<endl<<new_arrival_nxt_stop<<endl<<old_arrival_nxt_stop<<endl;
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
			 	
			 	//<<"testX "<<endl;

			 	
			 	if ((new_slack_time >= 0) && (pick_up_time <= latest_departure_passenger) && (new_capacity >= 0) && (departure_time_from_home >= current_time)) {

			 		//if (swap_vehicle)
			 		//	cout<<"co: "<<new_slack_time<<" "<<pick_up_time<<" "<<latest_departure_passenger<<" "<<new_capacity<<" "<<departure_time_from_home<<" "<<current_time<<endl;

			 		insertions_p[curr_number_insertions_p].passengers_departure_time_from_home = departure_time_from_home;
					insertions_p[curr_number_insertions_p].increase_length = increase;
					//min_increase_length = increase;
					//remove_edge = i;
					insertions_p[curr_number_insertions_p].pos_station = i+1;
					//sel_origin = s_origin;
					insertions_p[curr_number_insertions_p].sel_station = s_origin;

					insertions_p[curr_number_insertions_p].v = v;

					//repeated_station = false;
					insertions_p[curr_number_insertions_p].repeated_station = false;

					//check if included station is equal to one of the two stations
					if (s_origin == stops[v][i]){
						insertions_p[curr_number_insertions_p].repeated_station = true;
						insertions_p[curr_number_insertions_p].pos_station = i;
					}

					//check if included station is equal to one of the two stations
					if (s_origin == stops[v][i+1]) {
						insertions_p[curr_number_insertions_p].repeated_station = true;
						insertions_p[curr_number_insertions_p].pos_station = i+1;
					}

					curr_number_insertions_p++;
					if (curr_number_insertions_p > 44995) {
						return;
					}
					//<<"inset: "<<curr_number_insertions_p<<endl;

					//<<s_origin<<" "<<stops[v][i]<<" "<<stops[v][i+1]<<" "<<new_arrival_nxt_stop<<" "<<old_arrival_nxt_stop<<" "<<new_slack_time<<endl;

					//<<"app: true"<<endl;
					//<<"repeated: "<<repeated_station<<endl;
				} else {
					//cout<<"co: "<<new_slack_time<<" "<<pick_up_time<<" "<<latest_departure_passenger<<" "<<new_capacity<<" "<<departure_time_from_home<<" "<<current_time<<endl;
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

void cheapest_origin(int p, int v, int &min_increase_length, int &sel_origin, int &pos_origin, bool &repeated_station, bool &flexibilize_lat_departure_time, int &best_departure_time_from_home){

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
	/*if (empty_running_vehicle){
		bi = number_stops[v] - 1;
	}*/
	//<<number_stops_origin[p]<<" "<<number_stops[v]<<endl;
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
			 	//<<"co: "<<" "<<increase<<" "<<min_increase_length<<" "<<new_slack_time<<" "<<pick_up_time<<" "<<latest_departure_passenger<<" "<<new_capacity<<" "<<departure_time_from_home<<" "<<current_time;
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

void cheapest_destination(int p, int v, int pos_origin, int &min_increase_length, int &sel_destination, int &pos_destination, bool &repeated_station, bool &flexibilize_arrival_time, bool &infeasible_insertion) {

	//put here a check for the origin first then the loop (because of feasibility checks it may not be wise to insert the origin before checking for destination feasibility)
	int s_destination, increase;	
	int remove_edge;
	int drop_off_time;
	int new_arrival_time, new_departure_time, new_slack_time, new_arrival_nxt_stop, old_arrival_nxt_stop, old_slack_time;
	int latest_arrival_passenger, delay_trip;
	int new_capacity;
	bool feasible_insertion_found = false;
	infeasible_insertion = false;

	int min_increase_length_inf = INT_MAX;
	int remove_edge_inf, sel_destination_inf;
	bool repeated_station_inf;
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
			//<<"dpt: "<<drop_off_time<<" "<<latest_arrival_passenger<<endl;
			//<<new_slack_time<<endl;
			//cout<<"cd: "<<" "<<increase<<" "<<min_increase_length<<" "<<new_slack_time<<" "<<drop_off_time<<" "<<latest_arrival_passenger<<" "<<new_capacity<<endl;
			if ((increase < min_increase_length) && (new_slack_time >= 0) && (drop_off_time <= latest_arrival_passenger) && (new_capacity >= 0)) {
				feasible_insertion_found = true;
				infeasible_insertion = false;
				
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
			} else {

				if (not feasible_insertion_found) {

					infeasible_insertion = true;
					if (increase < min_increase_length_inf) {

						min_increase_length_inf = increase;
						remove_edge_inf = i;
						sel_destination_inf = s_destination;

						repeated_station_inf = false;
						//check if included station is equal to one of the two stations
						if (s_destination == stops[v][i]){
							repeated_station_inf = true;
							remove_edge_inf = i;
						}

						//check if included station is equal to one of the two stations
						if (s_destination == stops[v][i+1]) {
							repeated_station_inf = true;
							remove_edge_inf = i+1;
						}

					}

				}

			}
		}
	}

	if ((not feasible_insertion_found) && (infeasible_insertion)) {
		min_increase_length = min_increase_length_inf;
		remove_edge = remove_edge_inf;
		sel_destination = sel_destination_inf;
		repeated_station = repeated_station_inf;
		remove_edge =  remove_edge_inf;
	}

	if (not repeated_station)
		pos_destination = remove_edge+1;
	else
		pos_destination = remove_edge;
}

void cheapest_destination2(int p, int v, int pos_origin, int &min_increase_length, int &sel_destination, int &pos_destination, bool &repeated_station, bool &flexibilize_arrival_time, bool &infeasible_insertion) {

	//put here a check for the origin first then the loop (because of feasibility checks it may not be wise to insert the origin before checking for destination feasibility)
	int s_destination, increase;	
	int remove_edge;
	int drop_off_time;
	int new_arrival_time, new_departure_time, new_slack_time, new_arrival_nxt_stop, old_arrival_nxt_stop, old_slack_time;
	int latest_arrival_passenger, delay_trip;
	int new_capacity;
	bool feasible_insertion_found = false;
	infeasible_insertion = false;

	int min_increase_length_inf = INT_MAX;
	int remove_edge_inf, sel_destination_inf;
	bool repeated_station_inf;
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
			//<<"dpt: "<<drop_off_time<<" "<<latest_arrival_passenger<<endl;
			//<<new_slack_time<<endl;
			
			if ((increase < min_increase_length) && (new_slack_time >= 0) && (drop_off_time <= latest_arrival_passenger) && (new_capacity >= 0)) {
				feasible_insertion_found = true;
				infeasible_insertion = false;
				
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
			} else {
				//cout<<"cd: "<<" "<<increase<<" "<<min_increase_length<<" slack time "<<new_slack_time<<" do time "<<drop_off_time<<" la time "<<latest_arrival_passenger<<" "<<new_capacity<<" "<<latest_arrival[p]<<endl;
				if (not feasible_insertion_found) {

					infeasible_insertion = true;
					if (increase < min_increase_length_inf) {

						min_increase_length_inf = increase;
						remove_edge_inf = i;
						sel_destination_inf = s_destination;

						repeated_station_inf = false;
						//check if included station is equal to one of the two stations
						if (s_destination == stops[v][i]){
							repeated_station_inf = true;
							remove_edge_inf = i;
						}

						//check if included station is equal to one of the two stations
						if (s_destination == stops[v][i+1]) {
							repeated_station_inf = true;
							remove_edge_inf = i+1;
						}

					}

				}

			}
		}
	}

	if ((not feasible_insertion_found) && (infeasible_insertion)) {
		min_increase_length = min_increase_length_inf;
		remove_edge = remove_edge_inf;
		sel_destination = sel_destination_inf;
		repeated_station = repeated_station_inf;
		remove_edge =  remove_edge_inf;
	}

	if (not repeated_station)
		pos_destination = remove_edge+1;
	else
		pos_destination = remove_edge;
}

void filter_vehicles(int p, int c_id, vector<int>& filtered_vehicles_p){

	int v;
	bool leave_loop = false;
	int arrival_time_at_stop;
	for (int i = 0; i < clusters[c_id].size(); i++) {
		v = clusters[c_id][i];
		//if (free_capacity[v].size()>2){//means that vehicle is not empty
			//<<"heere";
			leave_loop = false;
			for (int j=0; j<=number_stops[v];j++) {
				for (int k=0;k<number_stops_origin[p];k++) {
					arrival_time_at_stop = departure_time_stop[v][j]+travel_time[stops[v][j]][stops_origin[p][k]];
					if (arrival_time_at_stop <= latest_arrival[p]) {
						//vehicle is feasible
						filtered_vehicles_p.push_back(v);
						leave_loop = true;
						break;
					}

				}
				if (leave_loop)
					break; 

			}
		//}
	}
}

void filter_vehicles2(int p, int c_id, vector<int>& filtered_vehicles_p){

	int v;
	bool leave_loop = false;
	int arrival_time_at_stop;
	for (int i = 0; i < clusters[c_id].size(); i++) {
		v = clusters[c_id][i];
		if (free_capacity[v].size()>2){//means that vehicle is not empty
			//<<"heere";
			leave_loop = false;
			for (int j=0; j<=number_stops[v];j++) {
				for (int k=0;k<number_stops_origin[p];k++) {
					arrival_time_at_stop = departure_time_stop[v][j]+travel_time[stops[v][j]][stops_origin[p][k]];
					if (arrival_time_at_stop <= latest_arrival[p]) {
						//vehicle is feasible
						filtered_vehicles_p.push_back(v);
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

void filter_vehicles_restricted(int p, int c_id, int vp, vector<int>& filtered_vehicles_p){

	int v;
	bool leave_loop = false;
	int arrival_time_at_stop;
	for (int i = 0; i < clusters[c_id].size(); i++) {
		v = clusters[c_id][i];
		if ((free_capacity[v].size()>2) && (free_capacity[v][0] < free_capacity[vp][0])) {//means that vehicle is not empty
			//<<"heere";
			leave_loop = false;
			for (int j=0; j<=number_stops[v];j++) {
				for (int k=0;k<number_stops_origin[p];k++) {
					arrival_time_at_stop = departure_time_stop[v][j]+travel_time[stops[v][j]][stops_origin[p][k]];
					if (arrival_time_at_stop <= latest_arrival[p]) {
						//vehicle is feasible
						filtered_vehicles_p.push_back(v);
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

/*void select_vehicles_havent_left_depot(vector<int>& vehicles_still_depot){

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
}*/

void select_vehicles_havent_that_can_be_turned_empty(vector<int>& vehicles_still_depot, int cluster_id){

	//<<"SELECT EMPTYING VEHICLES"<<endl;
	if (vehicles_still_depot.size() > 0)
		vehicles_still_depot.clear();
	//<<"hier1"<<endl;
	int v;
	for ( int it = 0; it < clusters[cluster_id].size(); it++) {

		v = clusters[cluster_id][it];

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
					//action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
					action_passengers[v][i][j] = action_passengers[v][i][number_passengers_action[v][i]-1];
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

/*void try_to_insert_in_already_running_vehicle(int p){
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

	//if (filtered_vehicles.size()>0)
	//	filtered_vehicles.clear();
	vector<int> filtered_vehicles_p;
	filter_vehicles2(p, 0, filtered_vehicles_p);

	int fsize = filtered_vehicles_p.size();
	for (int i=0;i<fsize;i++){
		if (vehicle_assigned[p] == filtered_vehicles_p[i]) {
			filtered_vehicles_p.erase(filtered_vehicles_p.begin() + i);
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
	
	while ((not_feasible_insertion) && (iterations < filtered_vehicles_p.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		Insertions insertions_p[55000];
		int curr_number_insertions_p = 0;
		//<<"heeerexx";
		for (int vf=0; vf<filtered_vehicles_p.size();vf++) {
			int v = filtered_vehicles_p[vf];
			
			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			if (blocked_vehicles[p][v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				
				
				//<<"vehiclex: "<<v<<endl;
				cheapest_origin2_p(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, insertions_p, curr_number_insertions_p);
				
			}
		}
		//<<endl;

		
		sort(insertions_p, insertions_p+curr_number_insertions_p, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions_p > 0) {
		
			//int rclsize = 5;
			remaining_insertions = curr_number_insertions_p;
			//if (rclsize < remaining_insertions)
			//	next_replace = rclsize;
			//else
			next_replace = remaining_insertions-1;

			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions_p)) {

				//<<"hiiier"<<endl;
				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				//if (rclsize <= remaining_insertions)
				//	selected_insertion = rand() % rclsize;
				//else
				selected_insertion = rand() % remaining_insertions;

				//<<"selec inser"<<selected_insertion<<endl;
				
				best_min_increase_length = insertions_p[selected_insertion].increase_length;
				best_sel_origin = insertions_p[selected_insertion].sel_station;
				best_pos_origin = insertions_p[selected_insertion].pos_station;
				best_v = insertions_p[selected_insertion].v;
				//<<"vehicley: "<<best_v<<endl;
				best_repeated_station = insertions_p[selected_insertion].repeated_station;
				remaining_insertions--; //updates the number of remaining feasible insertions

				//bring the next insertion to the top 3
				insertions_p[selected_insertion].increase_length = insertions_p[next_replace].increase_length;
				insertions_p[selected_insertion].sel_station = insertions_p[next_replace].sel_station;
				insertions_p[selected_insertion].pos_station = insertions_p[next_replace].pos_station;
				insertions_p[selected_insertion].v = insertions_p[next_replace].v;
				insertions_p[selected_insertion].repeated_station = insertions_p[next_replace].repeated_station;
				next_replace--;

				
				if (not best_repeated_station) {
					//<<"heere3"<<endl;

					//<<best_sel_origin<<" "<<best_min_increase_length<<endl;

					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					//if (action_passengers[best_v].size() < best_pos_origin)
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].resize(15);
					action_passengers[best_v][best_pos_origin][0] = p;
					next_free_position_passenger[best_v] = 1;
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					
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
					if (number_passengers_action[best_v][best_pos_origin] < action_passengers[best_v][best_pos_origin].size()) {
						action_passengers[best_v][best_pos_origin][number_passengers_action[best_v][best_pos_origin]] = p;
						//next_free_position_passenger[best_v]++;
					}
					else 
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin() + number_passengers_action[best_v][best_pos_origin], p);
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
					saved_arrival_time[best_v][i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[best_v][i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					}
					saved_slack_time[best_v][i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[best_v][i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				//vehicle_assigned[p] = best_v;

				

				best_min_increase_length = INT_MAX;
				
				min_increase_length = INT_MAX;
				repeated_station = false;
				sel_destination = -1;
				if (tested_all_vehicles_once)
					flexibilize_arrival_time = true;
				else
					flexibilize_arrival_time = false;

				bool infeasible_insertion;
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
			
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

					passengers_departure_time_from_home[p] = insertions_p[next_replace].passengers_departure_time_from_home;
					
					if (not repeated_station) {
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						//if (action_passengers[best_v].size() < best_pos_origin)
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].resize(15);
						action_passengers[best_v][pos_destination][0] = p;
						//next_free_position_passenger[best_v] = 1;
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
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
						if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
							action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
						} else
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
						number_passengers_action[best_v][pos_destination]++;

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						//if (slc_time < slack_time[best_v][pos_destination]) 
						//	slack_time[best_v][pos_destination] = slc_time;
						int min_slck_time = INT_MAX;
						for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
							int px = action_passengers[best_v][pos_destination][l];
							int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < min_slck_time) {
								min_slck_time = slc_time;
							}
						}
						slack_time[best_v][pos_destination] = min_slck_time;

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
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
						}
						arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
						departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
						slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


						free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


					} else {

						//<<"heere"<<endl;
						//<<action_passengers[best_v][best_pos_origin][1]<<endl;
						//action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
						//next_free_position_passenger[best_v]--;
						number_passengers_action[best_v][best_pos_origin]--;

						arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
						departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
						free_capacity[best_v][best_pos_origin]++;

						//re-update further arrival and departure times
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
						}

					}
				}
				
				//<<"heeere"<<endl;

				//if (best_v == 24) {
					//<<flexibilize_lat_departure_time<<endl;
				//<<"xyz1"<<endl;
 				
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

		
	}

	if (!not_so_big_increase_found) {
		vehicle_assigned[p] = initially_assigned_vehicle;
	}
}*/

void see_if_departure_empty_vehicle_not_zero(int v) {

	if (free_capacity[v].size() == 2) {
		if (departure_time_stop[v][0] > 0) {
			//cout<<"problemss"<<endl;
			departure_time_stop[v][0] = 0;
		}
	}


}

void see_if_arrival_departure_dont_match(int best_v) {

	if (free_capacity[best_v].size() > 2) {
		//<<number_stops[best_v]<<" "<<departure_time_stop[best_v].size()<<endl;
		for (int i = 1; i <= number_stops[best_v]; i++) {

			//if (arrival_time_stop[best_v][i] > current_time)
			if ((arrival_time_stop[best_v][i] > current_time) && (arrival_time_stop[best_v][i] != departure_time_stop[best_v][i])) {

				int greatest_ed = INT_MIN;
				//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
				for (int m=0; m<number_passengers_action[best_v][i];m++) {
					if (earliest_departure[action_passengers[best_v][i][m]] > greatest_ed)
						greatest_ed = earliest_departure[action_passengers[best_v][i][m]];
				}

				if ((greatest_ed != departure_time_stop[best_v][i]) && (arrival_time_stop[best_v][i] >= greatest_ed)) {
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
		//<<stops[v][i]<<" "<<arrival_time_stop_temp[v][i]<<" "<<departure_time_stop_temp[v][i]<<endl;
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
			//<<"diff "<<" "<<arrival_time_stop_temp[v][i]<<" "<<departure_time_stop_temp[v][i]<<endl;
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

	//<<stops[v][pos_destination]<<"dp "<<arrival_time_stop_temp[v][pos_destination]<<" "<<departure_time_stop_temp[v][pos_destination]<<endl;

	if (repeated_station) {

		//<<" "<<departure_time_stop_temp[v][pos_destination]<<" "<<travel_time[sel_destination][stops[v][pos_destination+1]]<<" "<<sel_destination<<" "<<stops[v][pos_destination+1]<<endl;
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
	
	//<<stops[v][pos_destination+1]<<"ps1 "<<arrival_time_stop_temp[v][pos_destination+1]<<" "<<departure_time_stop_temp[v][pos_destination+1]<<endl;

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
			//<<"hiierx3 "<<greatest_ed<<endl;
			departure_time_stop_temp[v][pos_destination+1] = greatest_ed;
		}
	}
	//<<"hier5"<<endl;

	//<<stops[v][pos_destination+1]<<"ps2 "<<arrival_time_stop_temp[v][pos_destination+1]<<" "<<departure_time_stop_temp[v][pos_destination+1]<<endl;
	
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

		//<<stops[v][i]<<" "<<arrival_time_stop_temp[v][i]<<" "<<departure_time_stop_temp[v][i]<<endl;
	}
	//<<"hier6"<<endl;

	//compute here difference in URT
	int current_user_ride_time, difference;
	bool leave_loop = false;
	difference = 0;
	for (int i=0; i<=number_stops[v];i++) {
		for (int j=0; j<number_passengers_action[v][i];j++) {
			int save_p = action_passengers[v][i][j];
			//<<save_p<<endl;
			if (save_p == p){
				user_ride_time_temp[save_p] = arrival_time_stop_temp[v][pos_destination] - departure_time_stop_temp[v][i];
				difference += user_ride_time[save_p] - user_ride_time_temp[save_p];
				//<<"COMP DIFFERENCE add "<<save_p<<" "<<user_ride_time[save_p] - user_ride_time_temp[save_p]<<endl;
			} else {
				for (int k=i+1; k<=number_stops[v];k++) {
					leave_loop = false;
					for (int l=0; l<number_passengers_action[v][k];l++) {
						//<<action_passengers[v][k][l]<<endl;
						if (action_passengers[v][k][l] == save_p){

							if ((k >= pos_destination) && (not repeated_station))  {
								k = k + 1;
							} 

							if ((i >= pos_destination) && (not repeated_station)) {
								i = i + 1;
							}

						
							current_user_ride_time = arrival_time_stop_temp[v][k] - departure_time_stop_temp[v][i];
							
							user_ride_time_temp[save_p] = current_user_ride_time;
							
							//<<"fad2: "<<save_p<<"; "<<current_user_ride_time<<" "<<arrival_time_stop_temp[v][k]<<" "<<departure_time_stop_temp[v][i]<<endl;
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

void re_insertion_to_repair(int p, bool &accept_relocate_trip, int cluster_id){

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
	//if (filtered_vehicles.size()>0)
	//	filtered_vehicles.clear();

	bool infeasible_insertion = false;
	//oldy_urt=total_user_ride_time;
	

	vector<int> filtered_vehicles_p;
	filter_vehicles2(p, cluster_id, filtered_vehicles_p);

	//<<"filtered_vehicles SIZE "<<filtered_vehicles.size()<<endl;
	while ((not_feasible_insertion) && (iterations < filtered_vehicles_p.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		Insertions insertions_p[45000];
		int curr_number_insertions_p = 0;
		
		for (int vf=0; vf<filtered_vehicles_p.size();vf++) {
			int v = filtered_vehicles_p[vf];
			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			if (blocked_vehicles[p][v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				
				see_if_arrival_departure_dont_match(v);
				//cout<<"HERE SA"<<endl;
				if (curr_number_insertions_p < 44955)
					cheapest_origin2_p(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, insertions_p, curr_number_insertions_p);
				
			}
		}
		//<<endl;

		//<<"curr insert2: " << curr_number_insertions<<endl;
		sort(insertions_p, insertions_p+curr_number_insertions_p, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions_p > 0) {
		
			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions_p)) {

				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				best_min_increase_length = insertions_p[iterations2].increase_length;
				best_sel_origin = insertions_p[iterations2].sel_station;
				best_pos_origin = insertions_p[iterations2].pos_station;
				best_v = insertions_p[iterations2].v;
				best_repeated_station = insertions_p[iterations2].repeated_station;

				//<<"few checks "<<endl;
				//<<best_min_increase_length<<" "<<best_sel_origin<<" "<<best_pos_origin<<" "<<best_v<<" "<<best_repeated_station<<endl<<endl;

				if (not best_repeated_station) {
					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].resize(15);
					action_passengers[best_v][best_pos_origin][0] = p;
					//next_free_position_passenger[best_v] = 1;
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
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
					if (number_passengers_action[best_v][best_pos_origin] < action_passengers[best_v][best_pos_origin].size()) {
						action_passengers[best_v][best_pos_origin][number_passengers_action[best_v][best_pos_origin]] = p;
					} else {
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin() + number_passengers_action[best_v][best_pos_origin], p);
					}
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
					saved_arrival_time[best_v][i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[best_v][i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					} else {
						bool update_DPT_time = true;
						int greatest_ed = INT_MIN;
						for (int l = 0; l < number_passengers_action[best_v][i];l++){
							int pp = action_passengers[best_v][i][l];
							if (earliest_departure[pp] > arrival_time_stop[best_v][i])
								update_DPT_time = false;
							if (earliest_departure[pp] > greatest_ed)
								greatest_ed = earliest_departure[pp];
						}

						if (update_DPT_time){
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						} else {
							if (greatest_ed < departure_time_stop[best_v][i])
								departure_time_stop[best_v][i] = greatest_ed;
						}
					}
					saved_slack_time[best_v][i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[best_v][i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				

				//if (best_v == 24) {
				/*cout<<"hierxxx: "<<flexibilize_lat_departure_time<<endl;
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

				
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
			
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

				//if (delay[p] <= max_flex_delay)
				//	accept_delay_trip = true;
				//else 
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
				//int DELTA;
				//int diff_remov_passenger = compute_difference_URT_by_fake_removing_passenger(p);
				//int diff_add_passenger = 0;
				//if ((sel_destination != -1) && (no_violation_capacity))
				//	diff_add_passenger = compute_difference_URT_by_fake_adding_passenger(p, best_v, sel_destination, pos_destination, repeated_station);
				//DELTA = diff_remov_passenger + diff_add_passenger;

				//if (diff_remov_passenger < 0)
				//	cout<<"NEGATIVE ERROR"<<endl;
				//DELTA += diff_remov_passenger;
				//accept_relocate_trip = false;
				//if (DELTA > 0)
				//	accept_relocate_trip = true;

				//printf("DELTA:%d \n", DELTA);

				
				//computedDELTA=DELTA;
				//addedAtV = best_v;
				//<<"oldy and delta "<<oldy_urt<<DELTA<<endl;
				if ((sel_destination != -1) && (no_violation_capacity) && (not infeasible_insertion)) {

					accept_relocate_trip = true;
					passengers_departure_time_from_home[p] = insertions_p[iterations2].passengers_departure_time_from_home;
					//type_move = 1;
					vehicle_assigned[p] = best_v;

					//<<"feasible and reduce fare1"<<endl;
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;

					
					if (not repeated_station) {
						//<<"hierrs"<<endl;
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						action_passengers[best_v][pos_destination].resize(15);
						action_passengers[best_v][pos_destination][0] = p;
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
						//<<"hierNrs1"<<endl;
						//update passenger performing actions on the stops
						if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
							action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
						} else
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
						number_passengers_action[best_v][pos_destination]++;

						

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						//if (slc_time < slack_time[best_v][pos_destination]) 
						//	slack_time[best_v][pos_destination] = slc_time;

						int min_slck_time = INT_MAX;
						for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
							int px = action_passengers[best_v][pos_destination][l];
							int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < min_slck_time) {
								min_slck_time = slc_time;
							}
						}
						slack_time[best_v][pos_destination] = min_slck_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}


					//update further arrival, departure times, and slack times
					for (int i=pos_destination; i<=number_stops[best_v];i++) {
						old_arr_time = arrival_time_stop[best_v][i];
						//<<"aki "<<" "<<departure_time_stop[best_v][i-1]<<" "<<travel_time[stops[best_v][i-1]][stops[best_v][i]]<<" "<<stops[best_v][i-1]<<" "<<stops[best_v][i]<<endl;
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						old_dpt_time = departure_time_stop[best_v][i];
						//if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						//}

						//<<arrival_time_stop[best_v][i]<<" "<<departure_time_stop[best_v][i]<<endl;

						int greatest_ed = INT_MIN;
						//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
						for (int l=0; l<number_passengers_action[best_v][i];l++) {
							if (earliest_departure[action_passengers[best_v][i][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[best_v][i][l]];
						}

						if (greatest_ed > departure_time_stop[best_v][i])
							departure_time_stop[best_v][i] = greatest_ed;

						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						//<<arrival_time_stop[best_v][i]<<" "<<departure_time_stop[best_v][i]<<endl;
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

										/*if (current_user_ride_time != user_ride_time_temp[save_p]) {
											cout<<"crime fake add "<<save_p<<endl;
										}*/
										if (current_user_ride_time != user_ride_time[save_p]) {
											difference = current_user_ride_time -  user_ride_time[save_p];
											//<<"difference: "<<difference<<endl;
											//total_user_ride_time += difference;
											//total_difference += difference;
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

					//move is simply not accepted because it is infeasible

					/*cout<<"BEFORE_SIMPLE RELOCATE"<<endl;
					for (int kk=0;kk<900;kk++){
						if (vehicle_assigned[kk] != -1) {
							solution_validation(kk, vehicle_assigned[kk]);
						//served_passengers++;
						}
					}*/
					//type_move = 3;
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
						
						//<<"hxxx"<<endl;
						stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
						action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
						number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
						

						//re-update further arrival and departure times
						//<<"ns: "<<best_pos_origin<<" "<<number_stops[best_v]<<endl;
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
							//<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
						}
						number_stops[best_v]--;
						arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
						departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
						slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


						free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


					} else {

						//<<"hed1ere"<<endl;
						//<<action_passengers[best_v][best_pos_origin][1]<<endl;
						//action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
						
						number_passengers_action[best_v][best_pos_origin]--;

						arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
						departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
						free_capacity[best_v][best_pos_origin]++;

						//re-update further arrival and departure times
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
						}

					}
					

				}

				/*cout<<"AFTER_SIMPLE UPDATE"<<endl;
				for (int kk=0;kk<900;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}*/

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
			iterations = filtered_vehicles_p.size()+1;
		}

		//<<"hieerxx";
		if (no_feasible_insertion2) {

			//type_move = 3;
			//iterations++;
			iterations = filtered_vehicles_p.size()+1;
		}
		if (iterations >= filtered_vehicles_p.size()) {
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
	//route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[p][i] = 0;
}

void relocate_passenger_to_repair(int p, bool &accept_relocate_trip, int cluster_id){

	//<<"before relocated passenger: "<<p<<endl;

	//compute the removal of passenger p from the solution
	int v = vehicle_assigned[p];
	int decrease_solution = 0;
	int count=0;
	int greatest_ed = INT_MIN;
	int old_arrival_time;


	/*cout<<vehicle_assigned[p]<<endl;
	cout<<"removing passenger x "<<p<<endl;
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

	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[p][i] = 0;

	//<<"vehicle and p :"<<vehicle_assigned[p]<<" "<<p<<endl;
	blocked_vehicles[p][vehicle_assigned[p]] = 1;

	re_insertion_to_repair(p, accept_relocate_trip, cluster_id);

	int begin, end;
	begin = 0;
	end = 0;
	int total_faking_error = 0;
	if (accept_relocate_trip) {
		//<<"removeee here SA"<<endl;
		//printf("remove heeeere SA\n");
		//<<"number stops "<<number_stops[v]<<endl;
		//it means the passenger was relocated to another trip 

		/*for (int l=0; l<=number_stops[v];l++) {
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
		*/
		for (int i=0; i<number_stops[v]; i++){
			for (int j=0; j<number_passengers_action[v][i];j++) {
				//<<"actp "<<action_passengers[v][i][j]<<endl;
				if (action_passengers[v][i][j] == p) {
					count++;
				
					//<<"SAhier1"<<endl;
					//removes previous origin stuff
					if (number_passengers_action[v][i] == 1) {
						number_stops[v]--;

						//<<"REMOVINGhier2"<<i<<endl;
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

						//action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
						action_passengers[v][i][j] = action_passengers[v][i][number_passengers_action[v][i]-1];
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
												//<<"faking removing error"<<endl;
												//<<current_user_ride_time<<" "<<user_ride_time_temp[save_p]<<endl;
												//<<arrival_time_stop[v][k]<<" "<<departure_time_stop[v][i]<<endl;
												total_faking_error += user_ride_time_temp[save_p] - current_user_ride_time;
											}
										

											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time -  user_ride_time[save_p];
												//<<"difference: "<<difference<<endl;
												//total_user_ride_time += difference;
												//total_difference += difference;
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

					/*cout<<number_stops[v]<<" "<<number_passengers_action[v].size()<<" "<<action_passengers[v].size()<<" "<<arrival_time_stop[v].size()<<" "<<departure_time_stop[v].size()<<" "<<slack_time[v].size()<<" "<<free_capacity[v].size()<<endl;
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


			/*if (count == 2) {
				//<<"oldyxnew: "<<oldy_urt<<" "<<total_user_ride_time<<" "<<(oldy_urt-total_user_ride_time)<<" "<<computedDELTA<<" ;"<<total_difference<<endl; 
				if ((oldy_urt-total_user_ride_time) != computedDELTA){
					//<<"COMPUTING ERRROR"<<endl;
					if (total_faking_error != 0)
						cout<<"total faking error: "<<total_faking_error<<endl;
				}
				break;
			}*/	
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



	/*cout<<"removing: "<<endl;
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
}

void remove_from_vehicle(int p, int v){

	int count = 0;
	int old_arrival_time;
	int greatest_ed = INT_MIN;
	int begin, end;
	begin = 0;
	end = 0;
	int total_faking_error = 0;

	/*cout<<"beforing removing"<<endl;
	for (int i=0; i<=number_stops[v];i++) {
		cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" [";
		for (int j=0; j<number_passengers_action[v][i];j++) 
			cout<<action_passengers[v][i][j]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][i]<<"} ";
		cout<<"{"<<departure_time_stop[v][i]<<"} ";
		cout<<"|"<<slack_time[v][i]<<"|  ";
		cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
	}*/

	for (int i=0; i<number_stops[v]; i++){
		for (int j=0; j<number_passengers_action[v][i];j++) {
			//<<"actp "<<action_passengers[v][i][j]<<endl;
			if (action_passengers[v][i][j] == p) {
				count++;
			
				//<<"SAhier1"<<endl;
				//removes previous origin stuff
				if (number_passengers_action[v][i] == 1) {
					number_stops[v]--;

					//<<"SAhier2"<<i<<endl;
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

					//action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
					action_passengers[v][i][j] = action_passengers[v][i][number_passengers_action[v][i]-1];
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
										/*if (current_user_ride_time != user_ride_time_temp[save_p]) {
											cout<<"faking removing error"<<endl;
											cout<<current_user_ride_time<<" "<<user_ride_time_temp[save_p]<<endl;
											cout<<arrival_time_stop[v][k]<<" "<<departure_time_stop[v][i]<<endl;
											total_faking_error += user_ride_time_temp[save_p] - current_user_ride_time;
										}*/
									

										if (current_user_ride_time != user_ride_time[save_p]) {
											difference = current_user_ride_time -  user_ride_time[save_p];
											//<<"difference: "<<difference<<endl;
											//total_user_ride_time += difference;
											//total_difference += difference;
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


		/*if (count == 2) {
			//<<"oldyxnew: "<<oldy_urt<<" "<<total_user_ride_time<<" "<<(oldy_urt-total_user_ride_time)<<" "<<computedDELTA<<" ;"<<total_difference<<endl; 
			if ((oldy_urt-total_user_ride_time) != computedDELTA){
				//<<"COMPUTING ERRROR"<<endl;
				if (total_faking_error != 0)
					cout<<"total faking error: "<<total_faking_error<<endl;
			}
			break;
		}*/	
	}
	see_if_arrival_departure_dont_match(v);
	update_URT(v);

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

	/*cout<<"passenger removed "<<p<<endl;
	for (int i=0; i<=number_stops[v];i++) {
		cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" [";
		for (int j=0; j<number_passengers_action[v][i];j++) 
			cout<<action_passengers[v][i][j]<<" ";
		cout<<"]  ";

		cout<<"{"<<arrival_time_stop[v][i]<<"} ";
		cout<<"{"<<departure_time_stop[v][i]<<"} ";
		cout<<"|"<<slack_time[v][i]<<"|  ";
		cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
	}*/
}

bool repair_solution(int v, int p, int cluster_id){

	int found_p = 0;
	std::vector<int> changed_passengers;
	std::vector<int> to_vehicles;
	listP blocked_passengers;

	for (int j=0; j<total_requests;j++){
		blocked_passengers[j] = 0;
	}
	int i = 1;
	while ((found_p < 2) && (i < number_stops[v])) {

		//<<"stuck"<<endl;
		for (int j=0; j<number_passengers_action[v][i];j++) {

			if (action_passengers[v][i][j] != p) {
				int passenger_relocate = action_passengers[v][i][j];
				int saved_v = vehicle_assigned[passenger_relocate];
				bool accept_relocate_trip = false;

				if (blocked_passengers[passenger_relocate] == 0) {
					blocked_passengers[passenger_relocate] = 1;
					//<<"repair passenger "<<passenger_relocate<<endl;
					relocate_passenger_to_repair(passenger_relocate, accept_relocate_trip, cluster_id);

					if (accept_relocate_trip) {
						//<<"accepted relocate "<<endl;
						changed_passengers.push_back(passenger_relocate);
						to_vehicles.push_back(vehicle_assigned[passenger_relocate]);
						//test if capacity and slack times are respected
						//if so, solution is repaired
						bool repaired_solution = true;
						for (int k=1; k<number_stops[v];k++) {

							if (slack_time[v][k] < 0) {
								repaired_solution = false;
							}

							if (free_capacity[v][k] < 0) {
								repaired_solution = false;
							}

						}

						if (repaired_solution) {
							//<<"REPAIRED "<<endl;
							return true;
						}

					} else {
						vehicle_assigned[passenger_relocate] = saved_v;
					}
				}

				

			} else {
				found_p++;
			}
		}

		i++;

	}

	for (int i=0; i<changed_passengers.size(); i++) {
		//<<"NOT REPAIRED  HERE"<<endl;
		//<<changed_passengers[i]<<" "<<to_vehicles[i]<<endl;
		remove_from_vehicle(changed_passengers[i], to_vehicles[i]);
		//<<"left hier1"<<endl;
		vehicle_assigned[changed_passengers[i]] = v;
		see_if_arrival_departure_dont_match(to_vehicles[i]);
		//<<"left hier2"<<endl;
		update_URT(to_vehicles[i]);
		//<<"left hier3"<<endl;
	}

	//<<"left hier4"<<endl;
	/*cout<<"BEFORE RETURN SAVE VEHICLE"<<endl;
	for (int kk=0;kk<900;kk++){
		if (vehicle_assigned[kk] != -1) {
			solution_validation(kk, vehicle_assigned[kk]);
		//served_passengers++;
		}
	}*/

	return_save_vehicle_p(v);
	//<<"left hier5"<<endl;

	/*cout<<"AFTER RETURN SAVE VEHICLE"<<endl;
	for (int kk=0;kk<900;kk++){
		if (vehicle_assigned[kk] != -1) {
			solution_validation(kk, vehicle_assigned[kk]);
		//served_passengers++;
		}
	}*/
	see_if_arrival_departure_dont_match(v);
	update_URT(v);

	if (changed_passengers.size() > 0) {
		changed_passengers.clear();
		to_vehicles.clear();
	}

	//<<"left hier6"<<endl;

	return false;
}

//this is a randomized version of cheapest insertion
//inserts the passenger at a randomized position between (0, k)



void cheapest_insertion_randomized_parallel(int p, bool accept_infeasible_insertion, int cluster_id){

	if (cluster_id < 0 || cluster_id >= number_clusters) {
        cerr << "ERROR: Invalid cluster_id " << cluster_id << " (max: " << number_clusters-1 << ")" << endl;
        return;
    }
    
    if (p < 0 || p >= total_requests) {
        cerr << "ERROR: Invalid passenger " << p << " (max: " << total_requests-1 << ")" << endl;
        return;
    }
    
    if (clusters[cluster_id].empty()) {
        cerr << "WARNING: Cluster " << cluster_id << " is empty for passenger " << p << endl;
        //serve_passenger_third_party_vehicle(p);
        return;
    }
	/*// Basic runtime guards to help catch out-of-range accesses causing segfaults.
	if (p < 0) {
		cerr << "DEBUG: cheapest_insertion_randomized_parallel called with negative p=" << p << endl;
		return;
	}
	if (cluster_id < 0) {
		cerr << "DEBUG: cheapest_insertion_randomized_parallel called with negative cluster_id=" << cluster_id << " p=" << p << endl;
		return;
	}
	if (cluster_id >= number_clusters) {
		cerr << "DEBUG: cheapest_insertion_randomized_parallel: cluster_id out of range: " << cluster_id << " >= " << number_clusters << " p=" << p << endl;
		return;
	}
	if (p >= total_requests) {
		cerr << "DEBUG: cheapest_insertion_randomized_parallel: p out of range: " << p << " >= total_requests(" << total_requests << ")" << endl;
		return;
	}

	// If clusters[cluster_id] is unexpectedly empty, emit a message (may indicate earlier logic bug)
	if (clusters[cluster_id].empty()) {
		cerr << "DEBUG: cheapest_insertion_randomized_parallel: cluster " << cluster_id << " is empty (p=" << p << ")" << endl;
	}*/

	/*if (p == 5) {
		cout<<p<<"GOT HERE"<<endl;
		return;
	}*/
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
	int best_departure_time_from_home;
	int veh;
	int ttcsd = INT_MAX;

	bool infeasible_insertion = false;
	
	Insertions insertions_p[45000];
	int curr_number_insertions_p = 0;

	vector<int> filtered_vehicles_p;
	
	//check for highest capacitated vehicle from partition that is empty

	//cout<<"pass: "<<p<<endl;
	capacity_best_empty_vehicle = INT_MIN;
	int best_distance_location = INT_MAX;

	bool empty_running_vehicle = false;
	/*for (int j = 0; j < clusters[cluster_id].size(); j++){
		veh = clusters[cluster_id][j];

		if (free_capacity[veh].size() == 2){//means that vehicle is empty
			//<<"hieer"<<endl;
			ttcsd = INT_MAX;
			for (int k=0;k<number_stops_origin[p];k++) {
				if (travel_time[stops_origin[p][k]][depot[vehicle_located_at_depot[veh]]] < ttcsd) {
					ttcsd = travel_time[stops_origin[p][k]][depot[vehicle_located_at_depot[veh]]];
				}
			}

			if (ttcsd < best_distance_location) {
				if (current_time + ttcsd <= latest_arrival[p]) { //important to verify if is feasible according to time window constraints
					//<<"NOPE"<<endl;
					best_empty_vehicle = veh;
					best_distance_location = ttcsd;
					capacity_best_empty_vehicle = free_capacity[veh][0];
				}
				
			} else {
				if (ttcsd == best_distance_location) {
					if (free_capacity[veh][0] > capacity_best_empty_vehicle) {
						if (current_time + ttcsd <= latest_arrival[p]) {
							best_empty_vehicle = veh;
							best_distance_location = ttcsd;
							capacity_best_empty_vehicle = free_capacity[veh][0];	
						}
					}
				}
			}

		} else {
			if (free_capacity[veh].size() > 2) { //the vehicle has passengers assigned but might be empty at some point in time 
				for (int i=number_stops[veh]-1; i<number_stops[veh];i++){
					if (free_capacity[veh][i] == free_capacity[veh][0]) { //means that vehicle is empty

						ttcsd = INT_MAX;
						for (int k=0;k<number_stops_origin[p];k++) {
							if (travel_time[stops[veh][i]][stops_origin[p][k]] < ttcsd) {
								ttcsd = travel_time[stops[veh][i]][stops_origin[p][k]];
							}
						}

						if (ttcsd < best_distance_location) {

							if (departure_time_stop[veh][i] + ttcsd <= latest_arrival[p]) { //important to verify if is feasible according to time window constraints
								empty_running_vehicle = true;
								best_empty_vehicle = veh;
								best_distance_location = ttcsd;
								capacity_best_empty_vehicle = free_capacity[veh][0];
							}

						} else {
							if (ttcsd == best_distance_location) {
								if (free_capacity[veh][0] > capacity_best_empty_vehicle) {

									if (departure_time_stop[veh][i] + ttcsd <= latest_arrival[p]) {
										empty_running_vehicle = true;
										best_empty_vehicle = veh;
										best_distance_location = ttcsd;
										capacity_best_empty_vehicle = free_capacity[veh][0];
									}
								}
							}
						}
					}
				}
			}
		}
	}*/

	if (filtered_vehicles_p.size()>0)
		filtered_vehicles_p.clear();
	//<<"hier 5.1"<<endl;
	int vv;
	/*if ((cluster_id < 0) or (cluster_id >= number_clusters)) {
		cout<<cluster_id<<endl;
		cout<<"exiting here CID PROBLM"<<endl;
		return;
	}*/
	//<<"clusterSIZ: "<<cluster_id<<" "<<clusters[cluster_id].size()<<endl;
	for (int i = 0; i < clusters[cluster_id].size(); i++) {
		vv = clusters[cluster_id][i];
		/*if ((vv > total_number_vehicles) or (vv < 0)) {
			cout<<vv<<endl;
			cout<<"exiting here VV PROBLM"<<endl;
			return;
		}*/
		if (free_capacity[vv].size() == 2){ //means that vehicle is empty and still at the depot
			filtered_vehicles_p.push_back(vv);
		}

	}

	 for (int i = 0; i < clusters[cluster_id].size(); i++) {
        int vv = clusters[cluster_id][i];
        if (vv < 0 || vv >= total_number_vehicles) {
            cerr << "CRITICAL: Invalid vehicle index " << vv << " from cluster " << cluster_id << endl;
            continue;
        }
    }


	//<<"hieerx2xempt"<<" "<<best_empty_vehicle<<" "<<number_stops[best_empty_vehicle]<<endl;
	bool same_station = false;
	//v = best_empty_vehicle;
	int selected_insertion, next_replace, remaining_insertions;
	//inserting in an empty vehicle stays the same as before
	bool not_feasible_insertion = true;
	bool no_feasible_insertion_empty = false;
	//bool emptyfirst = false;
	if (filtered_vehicles_p.size() == 0) {
		v = -1;
	}
	//<<"hier 5.2"<<endl;
	//<<"filtered SIZE: "<<filtered_vehicles_p.size()<<" "<<cluster_id<<endl;
	for (int itv = 0; itv < filtered_vehicles_p.size(); itv++) {

		v = filtered_vehicles_p[itv];
		if (not_feasible_insertion) {
			//<<"yhiery";
			//emptyfirst = true;
			int prv_arr_time_at_origin, prv_dpt_time_at_origin;
			//int old_user_ride_time = total_user_ride_time;

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
			//<<"veh: "<<v<<endl;
			cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);
			//<<"outside here"<<" "<<cluster_id<<endl;
			if (sel_origin != -1) {

				//printf("1234");
				//bool not_repeated_origin;
				if (not repeated_station) {

					//<<v<<" "<<pos_origin<<" "<<sel_origin<<" "<<p<<endl;
					//return;

					/*if ((v > total_number_vehicles) or (v < 0)) {
						cout<<v<<endl;
						cout<<"exiting here PROBLM"<<endl;
						return;
					}
					if (pos_origin-1 < 0){
						cout<<pos_origin<<endl;
						cout<<"exiting here2 PROBLM"<<endl;
						return;
					}
					if ((sel_origin > maxstations) or (sel_origin < 0)){
						cout<<sel_origin<<endl;
						cout<<"exiting here3 PROBLM"<<endl;
						return;
					}

					if ((p > 20000) or (p < 0)){
						cout<<p<<endl;
						cout<<"exiting here4 PROBLM"<<endl;
						return;
					}*/


					same_station = false;
					//<<"sel o"<<sel_origin;
					stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
					
					//update passenger performing actions on the stops
					//<<"hiiieer1"<<endl;
					action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
					action_passengers[v][pos_origin].resize(15);
					action_passengers[v][pos_origin][0] = p;
					//<<"hiiieer2"<<endl;
					//action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
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
					//if (earliest_departure[p] > departure_time_stop[v][pos_origin])
					//	departure_time_stop[v][pos_origin] = earliest_departure[p];

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
					if (number_passengers_action[v][pos_origin] < action_passengers[v][pos_origin].size()) {
						action_passengers[v][pos_origin][number_passengers_action[v][pos_origin]] = p;
					} else
						action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin() + number_passengers_action[v][pos_origin], p);
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


				//<<"hier 5.4"<<endl;
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
				cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
				//<<min_increase_length<<" "<<pos_destination<<endl;
				//<<"hier 5.5"<<endl;
				if ((sel_destination != -1) && (not infeasible_insertion)) {

					/*if ((v > total_number_vehicles) or (v < 0)) {
						cout<<v<<endl;
						cout<<"exiting here PROBLM"<<endl;
						return;
					}
					if (pos_destination-1 < 0){
						cout<<pos_destination<<endl;
						cout<<"exiting here2 PROBLM"<<endl;
						return;
					}
					if ((sel_destination > maxstations) or (sel_destination < 0)){
						cout<<sel_destination<<endl;
						cout<<"exiting here3 PROBLM"<<endl;
						return;
					}

					if ((p > 20000) or (p < 0)){
						cout<<p<<endl;
						cout<<"exiting here4 PROBLM"<<endl;
						return;
					}*/
					
					not_feasible_insertion = false;
					//<<"1hierxx"<<endl;
					passengers_departure_time_from_home[p] = best_departure_time_from_home;
					//<<"pos dest: "<<sel_destination<<endl;
					stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
					
					//update passenger performing actions on the stops
					//<<"hiiieer3"<<endl;
					action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
					action_passengers[v][pos_destination].resize(15);
					action_passengers[v][pos_destination][0] = p;
					//<<"hiiieer2"<<endl;
					//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


					//updating arrival and departure from stops
					int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
					//<<"xx "<<stops[v][pos_destination-1]<<endl;
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
					slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
					
					if (not same_station)
						slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
					else {
						int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
						if (slc_time < slack_time[v][pos_origin])
							slack_time[v][pos_origin] = slc_time;
					}

					prv_capacity = free_capacity[v][pos_destination-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

					//solution_cost += min_increase_length; //updates solution cost
					//updates user ride time
					//<<"heere1"<<endl;
					user_ride_time[p] = travel_time[sel_origin][sel_destination];
					//total_user_ride_time += user_ride_time[p];
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
					//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
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
					cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);

					if (delay[p] <= max_flex_delay)
						accept_delay_trip = true;
					else 
						accept_delay_trip = false;
					if ((sel_destination != -1) && (accept_delay_trip) && (not infeasible_insertion)) {
						
						not_feasible_insertion = false;
						//<<"2hierxx"<<endl;
						passengers_departure_time_from_home[p] = best_departure_time_from_home;
						//<<"pos dest: "<<sel_destination<<endl;
						stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
						
						//update passenger performing actions on the stops
						action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
						action_passengers[v].resize(15);
						action_passengers[v][pos_destination][0] = p;
						//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
						number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


						//updating arrival and departure from stops
						int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
						//<<"xx "<<stops[v][pos_destination-1]<<endl;
						arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
						departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
						slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
						//slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

						if (not same_station)
							slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
						else {
							int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
							if (slc_time < slack_time[v][pos_origin])
								slack_time[v][pos_origin] = slc_time;
						}

						prv_capacity = free_capacity[v][pos_destination-1];
						free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

						//updates solution cost
						//<<"heere2"<<endl;
						user_ride_time[p] = travel_time[sel_origin][sel_destination];
						//total_user_ride_time += user_ride_time[p];
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
						//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
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

						 
						//<<"3hierxx"<<endl;
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

						no_feasible_insertion_empty = true;
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
				//<<"hier 5.55"<<endl;
				cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);
				//<<"hier 5.57"<<" "<<cluster_id<<endl;
				if (sel_origin != -1) {

					if (not repeated_station) {

						/*if ((v > total_number_vehicles) or (v < 0)) {
							cout<<v<<endl;
							cout<<"exiting here PROBLM"<<endl;
							return;
						}
						if (pos_origin-1 < 0){
							cout<<pos_origin<<endl;
							cout<<"exiting here2 PROBLM"<<endl;
							return;
						}
						if ((sel_origin > maxstations) or (sel_origin < 0)){
							cout<<sel_origin<<endl;
							cout<<"exiting here3 PROBLM"<<endl;
							return;
						}

						if ((p > 20000) or (p < 0)){
							cout<<p<<endl;
							cout<<"exiting here4 PROBLM"<<endl;
							return;
						}*/

						same_station = false;
						//<<"sel o"<<sel_origin;
						stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
						
						//update passenger performing actions on the stops
						action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
						action_passengers[v][pos_origin].resize(15);
						action_passengers[v][pos_origin][0] = p;
						//action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
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
						//if (earliest_departure[p] > departure_time_stop[v][pos_origin])
						//	departure_time_stop[v][pos_origin] = earliest_departure[p];

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
						//action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
						if (number_passengers_action[v][pos_origin] < action_passengers[v][pos_origin].size()) {
							action_passengers[v][pos_origin][number_passengers_action[v][pos_origin]] = p;
						} else
							action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin() + number_passengers_action[v][pos_origin], p);
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
					cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
					//<<min_increase_length<<" "<<pos_destination<<endl;
					
					if ((sel_destination != -1) && (not infeasible_insertion)) {

						/*if ((v > total_number_vehicles) or (v < 0)) {
							cout<<v<<endl;
							cout<<"exiting here PROBLM"<<endl;
							return;
						}
						if (pos_destination-1 < 0){
							cout<<pos_destination<<endl;
							cout<<"exiting here2 PROBLM"<<endl;
							return;
						}
						if ((sel_destination > maxstations) or (sel_destination < 0)){
							cout<<sel_destination<<endl;
							cout<<"exiting here3 PROBLM"<<endl;
							return;
						}

						if ((p > 20000) or (p < 0)){
							cout<<p<<endl;
							cout<<"exiting here4 PROBLM"<<endl;
							return;
						}*/

						//<<"4hierxx"<<endl;
						
						not_feasible_insertion = false;
						//<<"pos dest: "<<pos_destination<<" "<<cluster_id<<endl;
						stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
						
						passengers_departure_time_from_home[p] = best_departure_time_from_home;
						//update passenger performing actions on the stops
						action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
						action_passengers[v][pos_destination].resize(15);
						action_passengers[v][pos_destination][0] = p;
						//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
						number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


						//updating arrival and departure from stops
						int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
						//<<"xx "<<stops[v][pos_destination-1]<<endl;
						arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
						departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
						slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
						//slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
						if (not same_station)
							slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
						else {
							int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
							if (slc_time < slack_time[v][pos_origin])
								slack_time[v][pos_origin] = slc_time;
						}

						prv_capacity = free_capacity[v][pos_destination-1];
						free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

						//updates solution cost
						//<<"heere3"<<endl;
						user_ride_time[p] = travel_time[sel_origin][sel_destination];
						//<<"user ride time: "<<travel_time[sel_origin][sel_destination]<<" "<<sel_origin<<" "<<sel_destination<<endl;
						//total_user_ride_time += user_ride_time[p];
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
						//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
						departure_time_stop[v][pos_destination+1] = arrival_time_stop[v][pos_destination+1];

						int min_slack_time_so_far = INT_MAX;	
							for (int i=number_stops[v]; i>=1; i--){
								if (slack_time[v][i] < min_slack_time_so_far) {
									min_slack_time_so_far = slack_time[v][i];
								} else {
									slack_time[v][i] = min_slack_time_so_far;
								}
							}

						//<<"hierx56"<<endl;
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


						//<<"5hierxx"<<endl;
						//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
						//try to insert giving flexibility according to latest_arrivel_time
						min_increase_length = INT_MAX;
						repeated_station = false;
						sel_destination = -1;
						flexibilize_arrival_time = true;
						see_if_arrival_departure_dont_match(v);
						cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);

						if (delay[p] <= max_flex_delay)
							accept_delay_trip = true;
						else 
							accept_delay_trip = false;
						if ((sel_destination != -1) && (accept_delay_trip) && (not infeasible_insertion)) {
							//<<"pos dest: "<<sel_destination<<endl;
							/*if ((v > total_number_vehicles) or (v < 0)) {
								cout<<v<<endl;
								cout<<"exiting here PROBLM"<<endl;
								return;
							}
							if (pos_destination-1 < 0){
								cout<<pos_destination<<endl;
								cout<<"exiting here2 PROBLM"<<endl;
								return;
							}
							if ((sel_destination > maxstations) or (sel_destination < 0)){
								cout<<sel_destination<<endl;
								cout<<"exiting here3 PROBLM"<<endl;
								return;
							}

							if ((p > 20000) or (p < 0)){
								cout<<p<<endl;
								cout<<"exiting here4 PROBLM"<<endl;
								return;
							}*/

							not_feasible_insertion = false;
							stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
							
							passengers_departure_time_from_home[p] = best_departure_time_from_home;
							//update passenger performing actions on the stops
							
							action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
							action_passengers[v][pos_destination].resize(15);
							action_passengers[v][pos_destination][0] = p;
							//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
							number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


							//updating arrival and departure from stops
							int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
							//<<"xx "<<stops[v][pos_destination-1]<<endl;
							arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
							departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
							slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
							//slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
							if (not same_station)
								slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
							else {
								int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
								if (slc_time < slack_time[v][pos_origin])
									slack_time[v][pos_origin] = slc_time;
							}

							prv_capacity = free_capacity[v][pos_destination-1];
							free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

							//updates solution cost
							//<<"heere4"<<endl;
							user_ride_time[p] = travel_time[sel_origin][sel_destination];
							//total_user_ride_time += user_ride_time[p];
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
							//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
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

							 
							//<<"5hierxx"<<endl;
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

								//action_passengers[v][pos_origin].erase(action_passengers[v][pos_origin].begin());

								number_passengers_action[v][pos_origin]--;

								arrival_time_stop[v][pos_origin] = prv_arr_time_at_origin;
								departure_time_stop[v][pos_origin] = prv_dpt_time_at_origin;
								free_capacity[v][pos_origin]++;

							}
							no_feasible_insertion_empty = true;
							//ultimately serve passenger with a 3rd party vehicle
							//serve_passenger_third_party_vehicle(p);
						
						}
					}

					update_URT(v);

				} else {
					no_feasible_insertion_empty = true;
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

		}

	} 

	//running with non-empty vehicle
	if (vehicle_assigned[p] == -1) {
		if ((no_feasible_insertion_empty) or (v == -1)) {

			//if (not emptyfirst) {
			//	cout<<"hereee"<<endl;
			//}
			//no empty vehicles. passenger will be inserted at the cheapest cost amongst all vehicles that is feasible

			//oneoftwo = 0;
			/*if (filtered_vehicles.size()>0)
				filtered_vehicles.clear();*/
			vector<int> filtered_vehicles_p2;
			filter_vehicles2(p, cluster_id, filtered_vehicles_p2);
			//<<"filtered size: "<<filtered_vehicles_p.size()<<endl;
			//cout<<"filtered_vehicles SIZE "<<filtered_vehicles_p2.size()<<" "<<served_passengers<<" "<<cluster_id<<endl;
			//cout<<"clsize "<<clusters[cluster_id].size()<<"p "<<p<<endl;
			bool not_feasible_insertion = true;
			int iterations = 0;
			bool tested_all_vehicles_once = false;

			/*if (filtered_vehicles.size() == 0){
				serve_passenger_third_party_vehicle(p);
			}*/
			//<<"hier 5.7"<<endl;
			std::shuffle(filtered_vehicles_p2.begin(), filtered_vehicles_p2.end(), default_random_engine(current_time));
			while ((not_feasible_insertion) && (iterations < filtered_vehicles_p2.size())) {
				
				int best_pos_origin, best_pos_destination;
				int best_min_increase_length;
				int best_sel_origin, best_sel_destination, best_v;
				bool best_repeated_station;
				
				best_min_increase_length = INT_MAX;
				best_v = -1;
				curr_number_insertions_p = 0;
				//<<"heeereAA";
				for (int vf=0; vf<filtered_vehicles_p2.size();vf++) {
					int v = filtered_vehicles_p2[vf];


					/*cout<<"BEFORE EVERYTHING"<<endl;
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
					cout<<endl;*/

					//<<"veh: "<<v<<"iterations: "<<iterations<<" "<<filtered_vehicles_p.size()<<" "<<total_number_vehicles<<endl;
					min_increase_length = INT_MAX;
					repeated_station = false;
					//(free_capacity[v] > 0) && 
					if (blocked_vehicles[p][v] == 0) {

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

						see_if_arrival_departure_dont_match(v);
						//<<"heereBB"<<endl;
						//<<v<<endl;
						//cout<<"insidere here"<<endl;
						if (curr_number_insertions_p < 44995)
							cheapest_origin2_p(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, insertions_p, curr_number_insertions_p);
						
					}
				}
				//<<endl;
				//<<"hier num insers "<<curr_number_insertions_p<<endl;
				//<<"heereCC";
				//cout<<"curr insert5: " << curr_number_insertions_p<<" "<<flexibilize_lat_departure_time<<endl;
				sort(insertions_p, insertions_p+curr_number_insertions_p, comparator);

				//<<best_v<<endl;
				//if (best_v != -1) {
				bool no_feasible_insertion2 = true;
				int iterations2 = 0;

				if (curr_number_insertions_p > 0) {
				
					//int rclsize = 5;
					remaining_insertions = curr_number_insertions_p;
					//if (rclsize < remaining_insertions)
					//	next_replace = rclsize;
					//else
					next_replace = remaining_insertions-1;

					while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions_p)) {

						//<<"5.87"<<endl;
							
						//<<"remain inser "<<remaining_insertions<<" "<<iterations2<<" "<<curr_number_insertions_p<<endl;
						//<<best_v<<"\n";
						int prv_arr_time_at_origin, prv_dpt_time_at_origin;

						//if (rclsize <= remaining_insertions)
						//	selected_insertion = rand() % rclsize;
						//else
						selected_insertion = rand() % remaining_insertions;

						//<<"sel inser "<<selected_insertion<<" "<<next_replace<<endl;

						//<<"selec inser"<<selected_insertion<<endl;
						//print_v_vehicle(insertions_p[selected_insertion].v);
						best_min_increase_length = insertions_p[selected_insertion].increase_length;
						best_sel_origin = insertions_p[selected_insertion].sel_station;
						best_pos_origin = insertions_p[selected_insertion].pos_station;
						best_v = insertions_p[selected_insertion].v;
						//<<"best v"<<best_v<<endl;

						best_repeated_station = insertions_p[selected_insertion].repeated_station;
						remaining_insertions--; //updates the number of remaining feasible insertions

						//bring the next insertion to the top 3
						insertions_p[selected_insertion].increase_length = insertions_p[next_replace].increase_length;
						insertions_p[selected_insertion].sel_station = insertions_p[next_replace].sel_station;
						insertions_p[selected_insertion].pos_station = insertions_p[next_replace].pos_station;
						insertions_p[selected_insertion].v = insertions_p[next_replace].v;
						insertions_p[selected_insertion].repeated_station = insertions_p[next_replace].repeated_station;
						next_replace--;
						/*if (total_number_vehicles == 0){
							cout<<"megra error hier 1"<<endl;
						}*/

						/*for (int i=0; i<=number_stops[best_v];i++) {
							cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
							for (int j=0; j<number_passengers_action[best_v][i];j++) 
								cout<<action_passengers[best_v][i][j]<<" ";
							cout<<"]  ";

							cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
							cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
							cout<<"|"<<slack_time[best_v][i]<<"|  ";
							cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
						}
						cout<<endl;*/

						//<<"npt passed saved"<<endl;
						/*if (saved_stops2.size() > 0) {
							saved_stops2.clear();
							saved_number_passengers_action2.clear();
							saved_action_passengers2.clear();
							saved_departure_time2.clear();
							saved_slack_time2.clear();
							saved_free_capacity2.clear();
							//<<"size saved: "<<saved_arrival_time2.size()<<endl;
							saved_arrival_time2.clear();
							//<<"size saved: "<<saved_arrival_time2.size()<<endl;
							//for (int i=0; i <){
							///		saved_arrival_time2[i].clear();
							//}
						}*/

						/*cout<<"BEFORE SAVE VEHICLE"<<endl;
						for (int kk=0;kk<900;kk++){
							if (vehicle_assigned[kk] != -1) {
								solution_validation(kk, vehicle_assigned[kk]);
							//served_passengers++;
							}
						}*/
						save_vehicle_p(best_v);

						/*cout<<"AFTER SAVE VEHICLE"<<endl;
						for (int kk=0;kk<900;kk++){
							if (vehicle_assigned[kk] != -1) {
								solution_validation(kk, vehicle_assigned[kk]);
							//served_passengers++;
							}
						}*/
						//<<"passed saved"<<endl;
						//<<best_pos_origin<<" "<<number_stops[best_v]<<" "<<arrival_time_stop[best_v].size()<<" "<<departure_time_stop[best_v].size()<<" "<<slack_time[best_v].size()<<" "<<number_passengers_action[best_v].size()<<" "<<free_capacity[best_v].size()<<endl<<endl;
						if (not best_repeated_station) {
							//<<"heere5.8.2.2.2"<<endl;
							//<<"non rep"<<endl;
							//oneoftwo++;
							//<<best_sel_origin<<" "<<best_min_increase_length<<endl;
							/*cout<<"start"<<endl;
							for (int i=0; i<=number_stops[best_v];i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";
								cout<<endl;
							}
							cout<<endl<<endl;*/
							//<<"best v "<<best_v<<endl;
							//<<best_pos_origin<<" "<<best_sel_origin<<endl;
							stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
							number_stops[best_v]++;

							/*for (int i=0; i<number_stops[best_v]-1;i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";
								cout<<endl;
								
							}
							cout<<endl<<endl;*/

							
							
							
							/*for (int i=0; i<number_stops[best_v]-1;i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";
								cout<<endl;
								
							}
							cout<<endl<<endl;*/

							number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

							//update passenger performing actions on the stops
							action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, std::vector<int>());
							action_passengers[best_v][best_pos_origin].resize(15);
							action_passengers[best_v][best_pos_origin][0] = p;

							//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
							
							/*for (int i=0; i<number_stops[best_v]-1;i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";
								cout<<endl;
							}
							cout<<endl<<endl;*/

							/*for (int i=0; i<=number_stops[best_v];i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";
								cout<<endl;
							}
							cout<<endl<<endl;*/
							//updating arrival and departure from stops
							
							//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;

							//case the vehicle is empty
							if ((departure_time_stop[best_v][best_pos_origin-1] == 0) && (best_pos_origin-1 == 0)) {
								int curr_dpt_time = earliest_departure[p]-travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
								if (curr_dpt_time < current_time){
									curr_dpt_time = current_time;
								}

								if (best_pos_origin == 1)
									departure_time_stop[best_v][0] = curr_dpt_time;
							}

							
							arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
							
							if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
								departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
							else
								departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
							slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
							
							//<<"hier 5.999"<<endl;
							/*for (int i=0; i<=number_stops[best_v];i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";

								cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
								cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
							}
							cout<<endl;*/

							prv_capacity = free_capacity[best_v][best_pos_origin-1];
							free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);

							/*for (int i=0; i<=number_stops[best_v];i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";

								cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
								cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
								cout<<"|"<<slack_time[best_v][i]<<"|  ";
								cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
							}
							cout<<endl;*/

							/*cout<<best_pos_origin<<" "<<endl;
							cout<<"AFTER UPDATES"<<endl;
							for (int kk=0;kk<900;kk++){
								if (vehicle_assigned[kk] != -1) {
									solution_validation(kk, vehicle_assigned[kk]);
								//served_passengers++;
								}
							}*/
							/*if (total_number_vehicles == 0){
								cout<<"megra error hier 2"<<endl;
							}*/	
							
						} else {
							//<<"heere5.88888"<<endl;
							//update passenger performing actions on the stops
							//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
							if (number_passengers_action[best_v][best_pos_origin] < action_passengers[best_v][best_pos_origin].size()) {
								action_passengers[best_v][best_pos_origin][number_passengers_action[best_v][best_pos_origin]] = p;
							} else
								action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin() + number_passengers_action[best_v][best_pos_origin], p);
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
						//<<best_pos_origin<<" "<<number_stops[best_v]<<" "<<arrival_time_stop[best_v].size()<<" "<<departure_time_stop[best_v].size()<<" "<<slack_time[best_v].size()<<" "<<number_passengers_action[best_v].size()<<" "<<free_capacity[best_v].size()<<endl<<endl;
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							//<<"hiier "<<arrival_time_stop[best_v][i]<<endl;
							saved_arrival_time[best_v][i] = arrival_time_stop[best_v][i];
							arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
							saved_departure_time[best_v][i] = departure_time_stop[best_v][i];
							if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
								departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
							}
							saved_slack_time[best_v][i] = slack_time[best_v][i];
							slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[best_v][i];
							//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
						}

						/*if (total_number_vehicles == 0){
							cout<<"megra error hier 3"<<endl;
						}*/
						
						
						//solution_cost += best_min_increase_length;
						//saved_increase = best_min_increase_length;
						
						/*cout<<"AFTER INSERTING ORIGIN"<<endl;
						for (int kk=0;kk<900;kk++){
							if (vehicle_assigned[kk] != -1) {
								solution_validation(kk, vehicle_assigned[kk]);
							//served_passengers++;
							}
						}*/

						vehicle_assigned[p] = best_v;

						//if (best_v == 24) {
						/*cout<<endl;
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
						//<<endl;*/

						best_min_increase_length = INT_MAX;
						
						min_increase_length = INT_MAX;
						repeated_station = false;
						sel_destination = -1;
						if (tested_all_vehicles_once)
							flexibilize_arrival_time = true;
						else
							flexibilize_arrival_time = false;

						//<<"15.872"<<endl;
						cheapest_destination2(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
						//<<"out 15.872"<<endl;
						//cout<<"pos origin: "<<best_pos_origin<<" "<<endl;
						//cout<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
						
						if ((sel_destination != -1)) { 

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

						//if (delay[p] == 0) {
						//cout<<"delay "<<delay[p]<<" "<<max_flex_delay<<endl;
						//cout<<"xxx :"<<sel_destination<<" "<<no_violation_capacity<<" "<<accept_delay_trip<<" "<<infeasible_insertion<<" "<<latest_arrival[p]<<endl;
						//}

						/*if (total_number_vehicles == 0){
							cout<<"megra error hier 4"<<endl;
						}*/
						if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip) && (not infeasible_insertion)) {
							
							
							//<<"742hierxx"<<endl;
							//print_v_vehicle(best_v);
							no_feasible_insertion2 = false;
							not_feasible_insertion = false;

							passengers_departure_time_from_home[p] = insertions_p[next_replace].passengers_departure_time_from_home;
							
							if (not repeated_station) {
								//<<"hier NR"<<endl;
								//oneoftwo++;
								stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
								number_stops[best_v]++;

								//update passenger performing actions on the stops
								action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
								action_passengers[best_v][pos_destination].resize(15);
								action_passengers[best_v][pos_destination][0] = p;
								//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
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
								//<<"hier REP"<<endl;
								//update passenger performing actions on the stops
								//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
								if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
									action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
								} else
									action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
								number_passengers_action[best_v][pos_destination]++;

								//according to the update when the origin is added, if the stop already exists then no need to change
								//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
								//arrival_time_stop[best_v][pos_destination] = arr_time;
								//departure_time_stop[best_v][pos_destination] = arr_time;
								//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
								//if (slc_time < slack_time[best_v][pos_destination]) 
									//slack_time[best_v][pos_destination] = slc_time;

								int min_slck_time = INT_MAX;
								for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
									int px = action_passengers[best_v][pos_destination][l];
									int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
									if (slc_time < min_slck_time) {
										min_slck_time = slc_time;
									}
								}
								slack_time[best_v][pos_destination] = min_slck_time;

								//prv_capacity = free_capacity[best_v][pos_destination];
								//free_capacity[best_v][pos_destination] = prv_capacity+1;

							}

							//<<"outxhier "<<number_stops[best_v]<<endl;
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
							//<<"outxhier1"<<endl;

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
							//<<"outxhier2"<<endl;
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
														//total_user_ride_time += difference;
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
							//<<"outxhier3"<<endl;


							
							
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
							
							/*cout<<"BEFORE DEST INSERT"<<endl;
								for (int kk=0;kk<900;kk++){
									if (vehicle_assigned[kk] != -1) {
										solution_validation(kk, vehicle_assigned[kk]);
									//served_passengers++;
									}
								}*/
							if ((infeasible_insertion) && (accept_infeasible_insertion)) {
								
								//<<"72hierxx"<<endl;	
								/*cout<<"7hierxx"<<endl;
								cout<<"vehicle infinsertion "<<best_v<<endl;
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
								no_feasible_insertion2 = false;
								not_feasible_insertion = false;

								passengers_departure_time_from_home[p] = insertions_p[next_replace].passengers_departure_time_from_home;
								
								//<<"hieer1"<<endl;
								//<<best_v<<" "<<pos_destination<<" "<<number_stops[best_v]<<" "<<arrival_time_stop[best_v].size()<<endl<<endl;
								if (not repeated_station) {
									//oneoftwo++;
									//<<"hieer2"<<endl;
									stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
									number_stops[best_v]++;

									//update passenger performing actions on the stops
									action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
									action_passengers[best_v][pos_destination].resize(15);
									action_passengers[best_v][pos_destination][0] = p;
									//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
									number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);
									//<<"hieer3"<<endl;
									//updating arrival and departure from stops
									int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
									//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
									arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
									departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
									slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

									prv_capacity = free_capacity[best_v][pos_destination-1];
									free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
									//<<"hieer4"<<endl;
								} else {
									//<<"hieer2"<<endl;
									//update passenger performing actions on the stops
									//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
									if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
										action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
									} else
										action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
									number_passengers_action[best_v][pos_destination]++;
									//<<"hieer3"<<endl;
									//according to the update when the origin is added, if the stop already exists then no need to change
									//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
									//arrival_time_stop[best_v][pos_destination] = arr_time;
									//departure_time_stop[best_v][pos_destination] = arr_time;
									//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
									//if (slc_time < slack_time[best_v][pos_destination]) 
										//slack_time[best_v][pos_destination] = slc_time;

									int min_slck_time = INT_MAX;
									for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
										int px = action_passengers[best_v][pos_destination][l];
										int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
										if (slc_time < min_slck_time) {
											min_slck_time = slc_time;
										}
									}
									slack_time[best_v][pos_destination] = min_slck_time;
									//<<"hieer4"<<endl;
									//prv_capacity = free_capacity[best_v][pos_destination];
									//free_capacity[best_v][pos_destination] = prv_capacity+1;

								}

								//<<"hieer5"<<endl;
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
								//<<"hieer6"<<endl;
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
								//<<"hieer7"<<endl;
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
															//total_user_ride_time += difference;
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

								/*cout<<"infeasible added "<<" "<<p<<endl;
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

								/*cout<<"BEFORE REPAIR"<<endl;
								for (int kk=0;kk<900;kk++){
									if (vehicle_assigned[kk] != -1) {
										solution_validation(kk, vehicle_assigned[kk]);
									//served_passengers++;
									}
								}*/

								//try to make it feasible again
								bool repaired_solution;
								repaired_solution = repair_solution(best_v, p, cluster_id);

								if (not repaired_solution) {
									//remove_from_vehicle(best_v, p);
									vehicle_assigned[p] = -1;
								}

								/*cout<<"AFTER INSERTION INFEASIBLE"<<endl;
								for (int kk=0;kk<900;kk++){
									if (vehicle_assigned[kk] != -1) {
										solution_validation(kk, vehicle_assigned[kk]);
									//served_passengers++;
									}
								}*/
								

							} else {
								//<<"omg here"<<endl;
								iterations2++;
								//solution_cost -= saved_increase;
								vehicle_assigned[p] = -1;
								if (sel_destination != -1) {
									for (int i=best_pos_origin+1;i<pos_destination;i++){

										free_capacity[best_v][i]++;
					
									}
								}

								/*if (total_number_vehicles == 0){
									cout<<"megra error hier 5"<<endl;
								}*/

								if (number_passengers_action[best_v][best_pos_origin] == 1) {
									
									stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
									action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
									number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
									

									//re-update further arrival and departure times
									for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
										arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
										departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
										slack_time[best_v][i] = saved_slack_time[best_v][i];
									}
									number_stops[best_v]--;
									arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
									departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
									slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


									free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);

									/*if (total_number_vehicles == 0){
										cout<<"megra error hier 6"<<endl;
									}*/


								} else {

									//<<"heere"<<endl;
									//<<action_passengers[best_v][best_pos_origin][1]<<endl;
									//action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
									
									number_passengers_action[best_v][best_pos_origin]--;

									arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
									departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
									free_capacity[best_v][best_pos_origin]++;

									//re-update further arrival and departure times
									for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
										arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
										departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
										slack_time[best_v][i] = saved_slack_time[best_v][i];
									}

									/*if (total_number_vehicles == 0){
										cout<<"megra error hier 7"<<endl;
									}*/

								}
							}

						}

						//<<"XYZhieer"<<endl;
						//<<"outxhier4"<<endl;
						update_URT(best_v);

						/*if (total_number_vehicles == 0){
							cout<<"megra error hier 8"<<endl;
						}*/
						//print_v_vehicle(best_v);
						//<<"outxhier5"<<endl;

						/*cout<<"URT UPDATED "<<endl;
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
					iterations = filtered_vehicles_p2.size()+1;
				}
				//<<"hier 5.85"<<endl;
				///<<"heereFF";
				if (no_feasible_insertion2) {

					//iterations++;
					iterations = filtered_vehicles_p2.size()+1;
				}
				if (iterations >= filtered_vehicles_p2.size()) {
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
						//<<p<<endl;
						//<<"rejected "<<endl;
					}
				}
			}
		}
	}

	//<<"hieer2"<<endl;
	/*if (vehicle_assigned[p] == -1) {
		//serve_passenger_third_party_vehicle(p);
		if (second_try)
			serve_passenger_third_party_vehicle(p);
		else
			passengers_on_hold.push_back(p);
	}*/

	//route_assigned[p] = 1;
	//reset blocked vehicles structure
	//<<"almost exiting"<<endl;
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[p][i] = 0;
	//if (vehicle_assigned[p] == -1)
		//cout<<"exiting cluster: "<<cluster_id<<endl;
}

void filter_vehicles_type(int type, vector<int>& filtered_vehicles_p) {

	if (type != -1) {
		for (int v = 0; v < total_number_vehicles; v++) {
			if (vehicle_type[v] == type){
				if (free_capacity[v].size()>2){//means that vehicle is not empty
					filtered_vehicles_p.push_back(v);
				}
			}
		}
	} else {
		for (int v = 0; v < total_number_vehicles; v++) {
			//if (vehicle_type[v] == type){
				if (free_capacity[v].size()>2){//means that vehicle is not empty
					filtered_vehicles_p.push_back(v);
				}
			//}
		}

	}

}

void filter_vehicles_type_empty(int type, vector<int>& filtered_vehicles_p) {

	if (type != -1) {
		for (int v = 0; v < total_number_vehicles; v++) {
			if (vehicle_type[v] == type){
				if (free_capacity[v].size()==2){//means that vehicle is not empty
					filtered_vehicles_p.push_back(v);
				}
			}
		}
	} else {
		for (int v = 0; v < total_number_vehicles; v++) {
			//if (vehicle_type[v] == type){
				if (free_capacity[v].size()==2){//means that vehicle is not empty
					filtered_vehicles_p.push_back(v);
				}
			//}
		}

	}

}

void decide_capacity_vehicle_uneven(int p, int &type){

	int which_type_demand = -1;

	bool found_stop = false;

	for (int k=0;k<number_stops_origin[p];k++) {
		int stopk = stops_origin[p][k];
		found_stop = false;
		for (int z = 0; z < 1; z++) {
			for (int nu = 0; nu < number_stops_uneven[z]; nu++) {
				if (stopk == stops_uneven[z][nu]){
					found_stop = true;
					which_type_demand = z;
					break;
				}
			}
			if (found_stop)
				break;
		}
		if (found_stop)
			break;
	}
	

	if (which_type_demand == 0) {
		//means large or medium capacity

		double y = (double)rand() / (double)RAND_MAX;

		if (number_vehicles[2] > 0) {
			if (y <= 0.5) {
				type = 1;
			} else {
				type = 2;
			}
		} else {
			type = 1;
		}
	}


	if (which_type_demand == 1) {
		//means small capacity
		type = 0;
	}

	if (which_type_demand == -1) {
		//RANDOM
		double y = (double)rand() / (double)RAND_MAX;

		if (number_vehicles[2] > 0) {
			if (y <= 0.33) {
				type = 0;
			} else {
				if (y <= 0.66) {
					type = 1;
				} else {
					type = 2;
				}
			} 
		} else {
			if (y <= 0.5) {
				type = 0;
			} else {
				type = 1;
			}
		}
	}


	//call filtered vehicles with the type then
}

void decide_capacity_vehicle_uneven_RANDOM(int p, int &type){

	int which_type_demand = -1;

	if (which_type_demand == -1) {
		//RANDOM
		double y = (double)rand() / (double)RAND_MAX;

		if (number_vehicles[2] > 0) {
			if (y <= 0.33) {
				type = 0;
			} else {
				if (y <= 0.66) {
					type = 1;
				} else {
					type = 2;
				}
			} 
		} else {
			if (y <= 0.5) {
				type = 0;
			} else {
				type = 1;
			}
		}
	}


	//call filtered vehicles with the type then
}

//in this version, we try to insert in a moving vehicle first
//then try empty vehicles
void cheapest_insertion_randomized_parallel_NEW(int p, bool accept_infeasible_insertion, int cluster_id){

	/*if (p == 5) {
		cout<<p<<"GOT HERE"<<endl;
		return;
	}*/
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
	int best_departure_time_from_home;
	int veh;
	int ttcsd = INT_MAX;

	bool infeasible_insertion = false;
	
	Insertions insertions_p[45000];
	int curr_number_insertions_p = 0;

	vector<int> filtered_vehicles_p;

	bool same_station = false;
	//v = best_empty_vehicle;
	int selected_insertion, next_replace, remaining_insertions;
	//inserting in an empty vehicle stays the same as before
	bool not_feasible_insertion = true;
	bool no_feasible_insertion_empty = false;

	int typev;
	decide_capacity_vehicle_uneven_RANDOM(p, typev);
	
	//check for highest capacitated vehicle from partition that is empty

	//cout<<"pass: "<<p<<endl;

	//cout<<"passenger "<<p<<endl;
	//running with non-empty vehicle
	for(int ci=0; ci <= 1;ci++) {
		if (vehicle_assigned[p] == -1) {
			v = -1;
			if ((no_feasible_insertion_empty) or (v == -1)) {

				//if (not emptyfirst) {
				//	cout<<"hereee"<<endl;
				//}
				//no empty vehicles. passenger will be inserted at the cheapest cost amongst all vehicles that is feasible

				//oneoftwo = 0;
				/*if (filtered_vehicles.size()>0)
					filtered_vehicles.clear();*/
				vector<int> filtered_vehicles_p2;
				//filter_vehicles2(p, cluster_id, filtered_vehicles_p2);
				if (ci == 0) {
					filter_vehicles_type(typev, filtered_vehicles_p2);
				} else {
					filter_vehicles_type(-1, filtered_vehicles_p2);
				}

				//cout<<"p2 size "<<filtered_vehicles_p2.size()<<endl;

				bool not_feasible_insertion = true;
				int iterations = 0;
				bool tested_all_vehicles_once = false;

				/*if (filtered_vehicles.size() == 0){
					serve_passenger_third_party_vehicle(p);
				}*/
				//<<"hier 5.7"<<endl;
				std::shuffle(filtered_vehicles_p2.begin(), filtered_vehicles_p2.end(), default_random_engine(current_time));
				//cout<<"FILTERED VEHICLE SIZE: "<<filtered_vehicles_p2.size()<<endl;
				while ((not_feasible_insertion) && (iterations < filtered_vehicles_p2.size())) {
					
					int best_pos_origin, best_pos_destination;
					int best_min_increase_length;
					int best_sel_origin, best_sel_destination, best_v;
					bool best_repeated_station;
					
					best_min_increase_length = INT_MAX;
					best_v = -1;
					curr_number_insertions_p = 0;
					//<<"heeereAA";
					for (int vf=0; vf<filtered_vehicles_p2.size();vf++) {
						int v = filtered_vehicles_p2[vf];


						/*cout<<"BEFORE EVERYTHING"<<endl;
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
						cout<<endl;*/

						//<<"veh: "<<v<<"iterations: "<<iterations<<" "<<filtered_vehicles_p.size()<<" "<<total_number_vehicles<<endl;
						min_increase_length = INT_MAX;
						repeated_station = false;
						//(free_capacity[v] > 0) && 
						if (blocked_vehicles[p][v] == 0) {

							if (tested_all_vehicles_once)
								flexibilize_lat_departure_time = true;
							else
								flexibilize_lat_departure_time = false;

							//flexibilize_lat_departure_time = false;
							
							/*cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time);

							if (min_increase_length < best_min_increase_length) {
								best_min_increase_length = min_increase_length;
								best_sel_origin = sel_origin;
								best_pos_origin = pos_origin;
								best_v = v;
								best_repeated_station = repeated_station;
							}*/

							see_if_arrival_departure_dont_match(v);
							//<<"heereBB"<<endl;
							//<<v<<endl;
							//cout<<"insidere here CONSTRUCTIVEEE"<<endl;
							if (curr_number_insertions_p < 44995)
								cheapest_origin2_p(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, insertions_p, curr_number_insertions_p);
							
						}
					}
					//<<endl;
					//<<"hier num insers "<<curr_number_insertions_p<<endl;
					//<<"heereCC";
					//cout<<"curr insert5: " << curr_number_insertions_p<<" "<<flexibilize_lat_departure_time<<endl;
					sort(insertions_p, insertions_p+curr_number_insertions_p, comparator);

					//<<best_v<<endl;
					//if (best_v != -1) {
					bool no_feasible_insertion2 = true;
					int iterations2 = 0;

					if (curr_number_insertions_p > 0) {
					
						//int rclsize = 5;
						remaining_insertions = curr_number_insertions_p;
						//if (rclsize < remaining_insertions)
						//	next_replace = rclsize;
						//else
						next_replace = remaining_insertions-1;

						while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions_p)) {

							//<<"5.87"<<endl;
								
							//<<"remain inser "<<remaining_insertions<<" "<<iterations2<<" "<<curr_number_insertions_p<<endl;
							//<<best_v<<"\n";
							int prv_arr_time_at_origin, prv_dpt_time_at_origin;

							//if (rclsize <= remaining_insertions)
							//	selected_insertion = rand() % rclsize;
							//else
							selected_insertion = rand() % remaining_insertions;

							//<<"sel inser "<<selected_insertion<<" "<<next_replace<<endl;

							//<<"selec inser"<<selected_insertion<<endl;
							//print_v_vehicle(insertions_p[selected_insertion].v);
							best_min_increase_length = insertions_p[selected_insertion].increase_length;
							best_sel_origin = insertions_p[selected_insertion].sel_station;
							best_pos_origin = insertions_p[selected_insertion].pos_station;
							best_v = insertions_p[selected_insertion].v;
							//<<"best v"<<best_v<<endl;

							best_repeated_station = insertions_p[selected_insertion].repeated_station;
							remaining_insertions--; //updates the number of remaining feasible insertions

							//bring the next insertion to the top 3
							insertions_p[selected_insertion].increase_length = insertions_p[next_replace].increase_length;
							insertions_p[selected_insertion].sel_station = insertions_p[next_replace].sel_station;
							insertions_p[selected_insertion].pos_station = insertions_p[next_replace].pos_station;
							insertions_p[selected_insertion].v = insertions_p[next_replace].v;
							insertions_p[selected_insertion].repeated_station = insertions_p[next_replace].repeated_station;
							next_replace--;
							/*if (total_number_vehicles == 0){
								cout<<"megra error hier 1"<<endl;
							}*/

							/*for (int i=0; i<=number_stops[best_v];i++) {
								cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
								for (int j=0; j<number_passengers_action[best_v][i];j++) 
									cout<<action_passengers[best_v][i][j]<<" ";
								cout<<"]  ";

								cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
								cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
								cout<<"|"<<slack_time[best_v][i]<<"|  ";
								cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
							}
							cout<<endl;*/

							//<<"npt passed saved"<<endl;
							/*if (saved_stops2.size() > 0) {
								saved_stops2.clear();
								saved_number_passengers_action2.clear();
								saved_action_passengers2.clear();
								saved_departure_time2.clear();
								saved_slack_time2.clear();
								saved_free_capacity2.clear();
								//<<"size saved: "<<saved_arrival_time2.size()<<endl;
								saved_arrival_time2.clear();
								//<<"size saved: "<<saved_arrival_time2.size()<<endl;
								//for (int i=0; i <){
								///		saved_arrival_time2[i].clear();
								//}
							}*/

							/*cout<<"BEFORE SAVE VEHICLE"<<endl;
							for (int kk=0;kk<900;kk++){
								if (vehicle_assigned[kk] != -1) {
									solution_validation(kk, vehicle_assigned[kk]);
								//served_passengers++;
								}
							}*/
							save_vehicle_p(best_v);

							/*cout<<"AFTER SAVE VEHICLE"<<endl;
							for (int kk=0;kk<900;kk++){
								if (vehicle_assigned[kk] != -1) {
									solution_validation(kk, vehicle_assigned[kk]);
								//served_passengers++;
								}
							}*/
							//<<"passed saved"<<endl;
							//<<best_pos_origin<<" "<<number_stops[best_v]<<" "<<arrival_time_stop[best_v].size()<<" "<<departure_time_stop[best_v].size()<<" "<<slack_time[best_v].size()<<" "<<number_passengers_action[best_v].size()<<" "<<free_capacity[best_v].size()<<endl<<endl;
							if (not best_repeated_station) {
								//<<"heere5.8.2.2.2"<<endl;
								//<<"non rep"<<endl;
								//oneoftwo++;
								//<<best_sel_origin<<" "<<best_min_increase_length<<endl;
								/*cout<<"start"<<endl;
								for (int i=0; i<=number_stops[best_v];i++) {
									cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
									for (int j=0; j<number_passengers_action[best_v][i];j++) 
										cout<<action_passengers[best_v][i][j]<<" ";
									cout<<"]  ";
									cout<<endl;
								}
								cout<<endl<<endl;*/
								//<<"best v "<<best_v<<endl;
								//<<best_pos_origin<<" "<<best_sel_origin<<endl;
								stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
								number_stops[best_v]++;

								/*for (int i=0; i<number_stops[best_v]-1;i++) {
									cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
									for (int j=0; j<number_passengers_action[best_v][i];j++) 
										cout<<action_passengers[best_v][i][j]<<" ";
									cout<<"]  ";
									cout<<endl;
									
								}
								cout<<endl<<endl;*/

								
								
								
								/*for (int i=0; i<number_stops[best_v]-1;i++) {
									cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
									for (int j=0; j<number_passengers_action[best_v][i];j++) 
										cout<<action_passengers[best_v][i][j]<<" ";
									cout<<"]  ";
									cout<<endl;
									
								}
								cout<<endl<<endl;*/

								number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + best_pos_origin, 1);

								//update passenger performing actions on the stops
								action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, std::vector<int>());
								action_passengers[best_v][best_pos_origin].resize(15);
								action_passengers[best_v][best_pos_origin][0] = p;

								//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
								
								/*for (int i=0; i<number_stops[best_v]-1;i++) {
									cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
									for (int j=0; j<number_passengers_action[best_v][i];j++) 
										cout<<action_passengers[best_v][i][j]<<" ";
									cout<<"]  ";
									cout<<endl;
								}
								cout<<endl<<endl;*/

								/*for (int i=0; i<=number_stops[best_v];i++) {
									cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
									for (int j=0; j<number_passengers_action[best_v][i];j++) 
										cout<<action_passengers[best_v][i][j]<<" ";
									cout<<"]  ";
									cout<<endl;
								}
								cout<<endl<<endl;*/
								//updating arrival and departure from stops
								
								//<<"xx "<<travel_time[stops[v][0]][sel_origin]<<endl;

								//case the vehicle is empty
								if ((departure_time_stop[best_v][best_pos_origin-1] == 0) && (best_pos_origin-1 == 0)) {
									int curr_dpt_time = earliest_departure[p]-travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin];
									if (curr_dpt_time < current_time){
										curr_dpt_time = current_time;
									}

									if (best_pos_origin == 1)
										departure_time_stop[best_v][0] = curr_dpt_time;
								}

								
								arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + best_pos_origin, departure_time_stop[best_v][best_pos_origin-1]+travel_time[stops[best_v][best_pos_origin-1]][best_sel_origin]);
								
								if (earliest_departure[p] > arrival_time_stop[best_v][best_pos_origin])
									departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, earliest_departure[p]);
								else
									departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + best_pos_origin, arrival_time_stop[best_v][best_pos_origin]);
								slack_time[best_v].insert(slack_time[best_v].begin() + best_pos_origin, 86400);
								
								//<<"hier 5.999"<<endl;
								/*for (int i=0; i<=number_stops[best_v];i++) {
									cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
									for (int j=0; j<number_passengers_action[best_v][i];j++) 
										cout<<action_passengers[best_v][i][j]<<" ";
									cout<<"]  ";

									cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
									cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
								}
								cout<<endl;*/

								prv_capacity = free_capacity[best_v][best_pos_origin-1];
								free_capacity[best_v].insert(free_capacity[best_v].begin() + best_pos_origin, prv_capacity-1);

								/*for (int i=0; i<=number_stops[best_v];i++) {
									cout<<stops[best_v][i]<<" ("<<number_passengers_action[best_v][i]<<") "<<" [";
									for (int j=0; j<number_passengers_action[best_v][i];j++) 
										cout<<action_passengers[best_v][i][j]<<" ";
									cout<<"]  ";

									cout<<"{"<<arrival_time_stop[best_v][i]<<"} ";
									cout<<"{"<<departure_time_stop[best_v][i]<<"} ";
									cout<<"|"<<slack_time[best_v][i]<<"|  ";
									cout<<"*"<<free_capacity[best_v][i]<<"*"<<endl;
								}
								cout<<endl;*/

								/*cout<<best_pos_origin<<" "<<endl;
								cout<<"AFTER UPDATES"<<endl;
								for (int kk=0;kk<900;kk++){
									if (vehicle_assigned[kk] != -1) {
										solution_validation(kk, vehicle_assigned[kk]);
									//served_passengers++;
									}
								}*/
								/*if (total_number_vehicles == 0){
									cout<<"megra error hier 2"<<endl;
								}*/	
								
							} else {
								//<<"heere5.88888"<<endl;
								//update passenger performing actions on the stops
								//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
								if (number_passengers_action[best_v][best_pos_origin] < action_passengers[best_v][best_pos_origin].size()) {
									action_passengers[best_v][best_pos_origin][number_passengers_action[best_v][best_pos_origin]] = p;
								} else
									action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin() + number_passengers_action[best_v][best_pos_origin], p);
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
							//<<best_pos_origin<<" "<<number_stops[best_v]<<" "<<arrival_time_stop[best_v].size()<<" "<<departure_time_stop[best_v].size()<<" "<<slack_time[best_v].size()<<" "<<number_passengers_action[best_v].size()<<" "<<free_capacity[best_v].size()<<endl<<endl;
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								//<<"hiier "<<arrival_time_stop[best_v][i]<<endl;
								saved_arrival_time[best_v][i] = arrival_time_stop[best_v][i];
								arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
								saved_departure_time[best_v][i] = departure_time_stop[best_v][i];
								if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
									departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
								}
								saved_slack_time[best_v][i] = slack_time[best_v][i];
								slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[best_v][i];
								//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
							}

							/*if (total_number_vehicles == 0){
								cout<<"megra error hier 3"<<endl;
							}*/
							
							
							//solution_cost += best_min_increase_length;
							//saved_increase = best_min_increase_length;
							
							/*cout<<"AFTER INSERTING ORIGIN"<<endl;
							for (int kk=0;kk<900;kk++){
								if (vehicle_assigned[kk] != -1) {
									solution_validation(kk, vehicle_assigned[kk]);
								//served_passengers++;
								}
							}*/

							vehicle_assigned[p] = best_v;

							//if (best_v == 24) {
							/*cout<<endl;
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
							//<<endl;*/

							best_min_increase_length = INT_MAX;
							
							min_increase_length = INT_MAX;
							repeated_station = false;
							sel_destination = -1;
							if (tested_all_vehicles_once)
								flexibilize_arrival_time = true;
							else
								flexibilize_arrival_time = false;

							flexibilize_arrival_time = false;

							//<<"15.872"<<endl;
							cheapest_destination2(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
							//<<"out 15.872"<<endl;
							//cout<<"pos origin: "<<best_pos_origin<<" "<<endl;
							//cout<<"pos dest: "<<pos_destination<<" sd: "<<sel_destination<<" "<<endl;
							
							if ((sel_destination != -1)) { 

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

							//if (delay[p] == 0) {
							//cout<<"delay "<<delay[p]<<" "<<max_flex_delay<<endl;
							//cout<<"xxx :"<<sel_destination<<" "<<no_violation_capacity<<" "<<accept_delay_trip<<" "<<infeasible_insertion<<" "<<latest_arrival[p]<<endl;
							//}

							/*if (total_number_vehicles == 0){
								cout<<"megra error hier 4"<<endl;
							}*/
							if ((sel_destination != -1) && (no_violation_capacity) && (accept_delay_trip) && (not infeasible_insertion)) {
								
								
								//<<"742hierxx"<<endl;
								//print_v_vehicle(best_v);
								no_feasible_insertion2 = false;
								not_feasible_insertion = false;

								passengers_departure_time_from_home[p] = insertions_p[next_replace].passengers_departure_time_from_home;
								
								if (not repeated_station) {
									//<<"hier NR"<<endl;
									//oneoftwo++;
									stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
									number_stops[best_v]++;

									//update passenger performing actions on the stops
									action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
									action_passengers[best_v][pos_destination].resize(15);
									action_passengers[best_v][pos_destination][0] = p;
									//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
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
									//<<"hier REP"<<endl;
									//update passenger performing actions on the stops
									//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
									if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
										action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
									} else
										action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
									number_passengers_action[best_v][pos_destination]++;

									//according to the update when the origin is added, if the stop already exists then no need to change
									//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
									//arrival_time_stop[best_v][pos_destination] = arr_time;
									//departure_time_stop[best_v][pos_destination] = arr_time;
									//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
									//if (slc_time < slack_time[best_v][pos_destination]) 
										//slack_time[best_v][pos_destination] = slc_time;

									int min_slck_time = INT_MAX;
									for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
										int px = action_passengers[best_v][pos_destination][l];
										int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
										if (slc_time < min_slck_time) {
											min_slck_time = slc_time;
										}
									}
									slack_time[best_v][pos_destination] = min_slck_time;

									//prv_capacity = free_capacity[best_v][pos_destination];
									//free_capacity[best_v][pos_destination] = prv_capacity+1;

								}

								//<<"outxhier "<<number_stops[best_v]<<endl;
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
								//<<"outxhier1"<<endl;

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
								//<<"outxhier2"<<endl;
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
															//total_user_ride_time += difference;
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
								//<<"outxhier3"<<endl;


								
								
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
								
								/*cout<<"BEFORE DEST INSERT"<<endl;
									for (int kk=0;kk<900;kk++){
										if (vehicle_assigned[kk] != -1) {
											solution_validation(kk, vehicle_assigned[kk]);
										//served_passengers++;
										}
									}*/
								if ((infeasible_insertion) && (accept_infeasible_insertion)) {
									
									//<<"72hierxx"<<endl;	
									/*cout<<"7hierxx"<<endl;
									cout<<"vehicle infinsertion "<<best_v<<endl;
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
									no_feasible_insertion2 = false;
									not_feasible_insertion = false;

									passengers_departure_time_from_home[p] = insertions_p[next_replace].passengers_departure_time_from_home;
									
									//<<"hieer1"<<endl;
									//<<best_v<<" "<<pos_destination<<" "<<number_stops[best_v]<<" "<<arrival_time_stop[best_v].size()<<endl<<endl;
									if (not repeated_station) {
										//oneoftwo++;
										//<<"hieer2"<<endl;
										stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
										number_stops[best_v]++;

										//update passenger performing actions on the stops
										action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
										action_passengers[best_v][pos_destination].resize(15);
										action_passengers[best_v][pos_destination][0] = p;
										//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
										number_passengers_action[best_v].insert(number_passengers_action[best_v].begin() + pos_destination, 1);
										//<<"hieer3"<<endl;
										//updating arrival and departure from stops
										int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
										//<<"pd "<<stops[best_v][pos_destination-1]<<" tt"<<travel_time[stops[best_v][pos_destination-1]][sel_destination]<<endl;
										arrival_time_stop[best_v].insert(arrival_time_stop[best_v].begin() + pos_destination, arr_time);
										departure_time_stop[best_v].insert(departure_time_stop[best_v].begin() + pos_destination, arr_time);
										slack_time[best_v].insert(slack_time[best_v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[best_v][pos_destination]);

										prv_capacity = free_capacity[best_v][pos_destination-1];
										free_capacity[best_v].insert(free_capacity[best_v].begin() + pos_destination, prv_capacity+1);
										//<<"hieer4"<<endl;
									} else {
										//<<"hieer2"<<endl;
										//update passenger performing actions on the stops
										//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
										if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
											action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
										} else
											action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
										number_passengers_action[best_v][pos_destination]++;
										//<<"hieer3"<<endl;
										//according to the update when the origin is added, if the stop already exists then no need to change
										//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
										//arrival_time_stop[best_v][pos_destination] = arr_time;
										//departure_time_stop[best_v][pos_destination] = arr_time;
										//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
										//if (slc_time < slack_time[best_v][pos_destination]) 
											//slack_time[best_v][pos_destination] = slc_time;

										int min_slck_time = INT_MAX;
										for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
											int px = action_passengers[best_v][pos_destination][l];
											int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
											if (slc_time < min_slck_time) {
												min_slck_time = slc_time;
											}
										}
										slack_time[best_v][pos_destination] = min_slck_time;
										//<<"hieer4"<<endl;
										//prv_capacity = free_capacity[best_v][pos_destination];
										//free_capacity[best_v][pos_destination] = prv_capacity+1;

									}

									//<<"hieer5"<<endl;
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
									//<<"hieer6"<<endl;
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
									//<<"hieer7"<<endl;
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
																//total_user_ride_time += difference;
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

									/*cout<<"infeasible added "<<" "<<p<<endl;
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

									/*cout<<"BEFORE REPAIR"<<endl;
									for (int kk=0;kk<900;kk++){
										if (vehicle_assigned[kk] != -1) {
											solution_validation(kk, vehicle_assigned[kk]);
										//served_passengers++;
										}
									}*/

									//try to make it feasible again
									bool repaired_solution;
									repaired_solution = repair_solution(best_v, p, cluster_id);

									if (not repaired_solution) {
										//remove_from_vehicle(best_v, p);
										vehicle_assigned[p] = -1;
									}

									/*cout<<"AFTER INSERTION INFEASIBLE"<<endl;
									for (int kk=0;kk<900;kk++){
										if (vehicle_assigned[kk] != -1) {
											solution_validation(kk, vehicle_assigned[kk]);
										//served_passengers++;
										}
									}*/
									

								} else {
									//<<"omg here"<<endl;
									iterations2++;
									//solution_cost -= saved_increase;
									vehicle_assigned[p] = -1;
									if (sel_destination != -1) {
										for (int i=best_pos_origin+1;i<pos_destination;i++){

											free_capacity[best_v][i]++;
						
										}
									}

									/*if (total_number_vehicles == 0){
										cout<<"megra error hier 5"<<endl;
									}*/

									if (number_passengers_action[best_v][best_pos_origin] == 1) {
										
										stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
										action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
										number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
										

										//re-update further arrival and departure times
										for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
											arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
											departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
											slack_time[best_v][i] = saved_slack_time[best_v][i];
										}
										number_stops[best_v]--;
										arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
										departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
										slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


										free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);

										/*if (total_number_vehicles == 0){
											cout<<"megra error hier 6"<<endl;
										}*/


									} else {

										//<<"heere"<<endl;
										//<<action_passengers[best_v][best_pos_origin][1]<<endl;
										//action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
										
										number_passengers_action[best_v][best_pos_origin]--;

										arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
										departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
										free_capacity[best_v][best_pos_origin]++;

										//re-update further arrival and departure times
										for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
											arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
											departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
											slack_time[best_v][i] = saved_slack_time[best_v][i];
										}

										/*if (total_number_vehicles == 0){
											cout<<"megra error hier 7"<<endl;
										}*/

									}
								}

							}

							//<<"XYZhieer"<<endl;
							//<<"outxhier4"<<endl;
							update_URT(best_v);

							/*if (total_number_vehicles == 0){
								cout<<"megra error hier 8"<<endl;
							}*/
							//print_v_vehicle(best_v);
							//<<"outxhier5"<<endl;

							/*cout<<"URT UPDATED "<<endl;
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
						iterations = filtered_vehicles_p2.size()+1;
					}
					//<<"hier 5.85"<<endl;
					///<<"heereFF";
					if (no_feasible_insertion2) {

						//cout<<"NO FEASIBLE INSERTION FOR "<<p<<" "<<vehicle_assigned[p]<<endl;
						//iterations++;
						iterations = filtered_vehicles_p2.size()+1;
					}
					if (iterations >= filtered_vehicles_p2.size()) {
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
							//<<p<<endl;
							//<<"rejected "<<endl;
						}
					}
				}
			}
		}
	}
	//non-empty


	//empty
	capacity_best_empty_vehicle = INT_MIN;
	int best_distance_location = INT_MAX;

	bool empty_running_vehicle = false;
	

	if (filtered_vehicles_p.size()>0)
		filtered_vehicles_p.clear();
	int vv;
	
	//<<"clusterSIZ: "<<cluster_id<<" "<<clusters[cluster_id].size()<<endl;
	/*for (int i = 0; i < clusters[cluster_id].size(); i++) {
		vv = clusters[cluster_id][i];
		
		if (free_capacity[vv].size() == 2){ //means that vehicle is empty and still at the depot
			filtered_vehicles_p.push_back(vv);
		}

	}*/


	//<<"hieerx2xempt"<<" "<<best_empty_vehicle<<" "<<number_stops[best_empty_vehicle]<<endl;
	
	//bool emptyfirst = false;
	//if (filtered_vehicles_p.size() == 0) {
	//	v = -1;
	//}
	

	for(int ci=0; ci <= 1;ci++) {
		if (vehicle_assigned[p] == -1) {
			if (filtered_vehicles_p.size()>0)
				filtered_vehicles_p.clear();
			if (ci == 0) {
				filter_vehicles_type_empty(typev, filtered_vehicles_p);
			} else {
				filter_vehicles_type_empty(-1, filtered_vehicles_p);
			}

			//cout<<"size filter p "<<filtered_vehicles_p.size()<<endl;

			int best_found_user_ride_time = INT_MAX;
			for (int itv = 0; itv < filtered_vehicles_p.size(); itv++) {

				v = filtered_vehicles_p[itv];
				if (not_feasible_insertion) {
					//<<"yhiery";
					//emptyfirst = true;
					int prv_arr_time_at_origin, prv_dpt_time_at_origin;
					//int old_user_ride_time = total_user_ride_time;

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
					//<<"veh: "<<v<<endl;
					cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);
					//<<"outside here"<<" "<<cluster_id<<endl;
					if (sel_origin != -1) {

						//printf("1234");
						//bool not_repeated_origin;
						if (not repeated_station) {

							//<<v<<" "<<pos_origin<<" "<<sel_origin<<" "<<p<<endl;
							//return;

							/*if ((v > total_number_vehicles) or (v < 0)) {
								cout<<v<<endl;
								cout<<"exiting here PROBLM"<<endl;
								return;
							}
							if (pos_origin-1 < 0){
								cout<<pos_origin<<endl;
								cout<<"exiting here2 PROBLM"<<endl;
								return;
							}
							if ((sel_origin > maxstations) or (sel_origin < 0)){
								cout<<sel_origin<<endl;
								cout<<"exiting here3 PROBLM"<<endl;
								return;
							}

							if ((p > 20000) or (p < 0)){
								cout<<p<<endl;
								cout<<"exiting here4 PROBLM"<<endl;
								return;
							}*/


							same_station = false;
							//<<"sel o"<<sel_origin;
							stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
							
							//update passenger performing actions on the stops
							//<<"hiiieer1"<<endl;
							action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
							action_passengers[v][pos_origin].resize(15);
							action_passengers[v][pos_origin][0] = p;
							//<<"hiiieer2"<<endl;
							//action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
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
							//if (earliest_departure[p] > departure_time_stop[v][pos_origin])
							//	departure_time_stop[v][pos_origin] = earliest_departure[p];

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
							if (number_passengers_action[v][pos_origin] < action_passengers[v][pos_origin].size()) {
								action_passengers[v][pos_origin][number_passengers_action[v][pos_origin]] = p;
							} else
								action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin() + number_passengers_action[v][pos_origin], p);
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


						//<<"hier 5.4"<<endl;
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
						cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
						//<<min_increase_length<<" "<<pos_destination<<endl;
						//<<"hier 5.5"<<endl;
						if ((sel_destination != -1) && (not infeasible_insertion)) {

							/*if ((v > total_number_vehicles) or (v < 0)) {
								cout<<v<<endl;
								cout<<"exiting here PROBLM"<<endl;
								return;
							}
							if (pos_destination-1 < 0){
								cout<<pos_destination<<endl;
								cout<<"exiting here2 PROBLM"<<endl;
								return;
							}
							if ((sel_destination > maxstations) or (sel_destination < 0)){
								cout<<sel_destination<<endl;
								cout<<"exiting here3 PROBLM"<<endl;
								return;
							}

							if ((p > 20000) or (p < 0)){
								cout<<p<<endl;
								cout<<"exiting here4 PROBLM"<<endl;
								return;
							}*/
							
							not_feasible_insertion = false;
							//<<"1hierxx"<<endl;
							passengers_departure_time_from_home[p] = best_departure_time_from_home;
							//<<"pos dest: "<<sel_destination<<endl;
							stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
							
							//update passenger performing actions on the stops
							//<<"hiiieer3"<<endl;
							action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
							action_passengers[v][pos_destination].resize(15);
							action_passengers[v][pos_destination][0] = p;
							//<<"hiiieer2"<<endl;
							//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
							number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


							//updating arrival and departure from stops
							int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
							//<<"xx "<<stops[v][pos_destination-1]<<endl;
							arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
							departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
							slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
							
							if (not same_station)
								slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
							else {
								int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
								if (slc_time < slack_time[v][pos_origin])
									slack_time[v][pos_origin] = slc_time;
							}

							prv_capacity = free_capacity[v][pos_destination-1];
							free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

							//solution_cost += min_increase_length; //updates solution cost
							//updates user ride time
							//<<"heere1"<<endl;
							user_ride_time[p] = travel_time[sel_origin][sel_destination];

							//total_user_ride_time += user_ride_time[p];
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
							//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
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

							/*if (user_ride_time[p] < best_found_user_ride_time) {
								best_found_user_ride_time = user_ride_time[p];
								save_vehicle_p(v);
							}*/

							
							
						} else {

							
							//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
							//try to insert giving flexibility according to latest_arrivel_time
							min_increase_length = INT_MAX;
							repeated_station = false;
							sel_destination = -1;
							flexibilize_arrival_time = true;
							see_if_arrival_departure_dont_match(v);	
							cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);

							if (delay[p] <= max_flex_delay)
								accept_delay_trip = true;
							else 
								accept_delay_trip = false;
							if ((sel_destination != -1) && (accept_delay_trip) && (not infeasible_insertion)) {
								
								not_feasible_insertion = false;
								//<<"2hierxx"<<endl;
								passengers_departure_time_from_home[p] = best_departure_time_from_home;
								//<<"pos dest: "<<sel_destination<<endl;
								stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
								
								//update passenger performing actions on the stops
								action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
								action_passengers[v].resize(15);
								action_passengers[v][pos_destination][0] = p;
								//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
								number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


								//updating arrival and departure from stops
								int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
								//<<"xx "<<stops[v][pos_destination-1]<<endl;
								arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
								departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
								slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
								//slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];

								if (not same_station)
									slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
								else {
									int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
									if (slc_time < slack_time[v][pos_origin])
										slack_time[v][pos_origin] = slc_time;
								}

								prv_capacity = free_capacity[v][pos_destination-1];
								free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

								//updates solution cost
								//<<"heere2"<<endl;
								user_ride_time[p] = travel_time[sel_origin][sel_destination];
								//total_user_ride_time += user_ride_time[p];
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
								//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
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

								 
								//<<"3hierxx"<<endl;
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

								no_feasible_insertion_empty = true;
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
						//<<"hier 5.55"<<endl;
						cheapest_origin(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, best_departure_time_from_home);
						//<<"hier 5.57"<<" "<<cluster_id<<endl;
						if (sel_origin != -1) {

							if (not repeated_station) {

								/*if ((v > total_number_vehicles) or (v < 0)) {
									cout<<v<<endl;
									cout<<"exiting here PROBLM"<<endl;
									return;
								}
								if (pos_origin-1 < 0){
									cout<<pos_origin<<endl;
									cout<<"exiting here2 PROBLM"<<endl;
									return;
								}
								if ((sel_origin > maxstations) or (sel_origin < 0)){
									cout<<sel_origin<<endl;
									cout<<"exiting here3 PROBLM"<<endl;
									return;
								}

								if ((p > 20000) or (p < 0)){
									cout<<p<<endl;
									cout<<"exiting here4 PROBLM"<<endl;
									return;
								}*/

								same_station = false;
								//<<"sel o"<<sel_origin;
								stops[v].insert(stops[v].begin() + pos_origin, sel_origin);
								
								//update passenger performing actions on the stops
								action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
								action_passengers[v][pos_origin].resize(15);
								action_passengers[v][pos_origin][0] = p;
								//action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
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
								//if (earliest_departure[p] > departure_time_stop[v][pos_origin])
								//	departure_time_stop[v][pos_origin] = earliest_departure[p];

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
								//action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), p);
								if (number_passengers_action[v][pos_origin] < action_passengers[v][pos_origin].size()) {
									action_passengers[v][pos_origin][number_passengers_action[v][pos_origin]] = p;
								} else
									action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin() + number_passengers_action[v][pos_origin], p);
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
							cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
							//<<min_increase_length<<" "<<pos_destination<<endl;
							
							if ((sel_destination != -1) && (not infeasible_insertion)) {

								/*if ((v > total_number_vehicles) or (v < 0)) {
									cout<<v<<endl;
									cout<<"exiting here PROBLM"<<endl;
									return;
								}
								if (pos_destination-1 < 0){
									cout<<pos_destination<<endl;
									cout<<"exiting here2 PROBLM"<<endl;
									return;
								}
								if ((sel_destination > maxstations) or (sel_destination < 0)){
									cout<<sel_destination<<endl;
									cout<<"exiting here3 PROBLM"<<endl;
									return;
								}

								if ((p > 20000) or (p < 0)){
									cout<<p<<endl;
									cout<<"exiting here4 PROBLM"<<endl;
									return;
								}*/

								//<<"4hierxx"<<endl;
								
								not_feasible_insertion = false;
								//<<"pos dest: "<<pos_destination<<" "<<cluster_id<<endl;
								stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
								
								passengers_departure_time_from_home[p] = best_departure_time_from_home;
								//update passenger performing actions on the stops
								action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
								action_passengers[v][pos_destination].resize(15);
								action_passengers[v][pos_destination][0] = p;
								//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
								number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


								//updating arrival and departure from stops
								int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
								//<<"xx "<<stops[v][pos_destination-1]<<endl;
								arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
								departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
								slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
								//slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
								if (not same_station)
									slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
								else {
									int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
									if (slc_time < slack_time[v][pos_origin])
										slack_time[v][pos_origin] = slc_time;
								}

								prv_capacity = free_capacity[v][pos_destination-1];
								free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

								//updates solution cost
								//<<"heere3"<<endl;
								user_ride_time[p] = travel_time[sel_origin][sel_destination];
								//<<"user ride time: "<<travel_time[sel_origin][sel_destination]<<" "<<sel_origin<<" "<<sel_destination<<endl;
								//total_user_ride_time += user_ride_time[p];
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
								//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
								departure_time_stop[v][pos_destination+1] = arrival_time_stop[v][pos_destination+1];

								int min_slack_time_so_far = INT_MAX;	
									for (int i=number_stops[v]; i>=1; i--){
										if (slack_time[v][i] < min_slack_time_so_far) {
											min_slack_time_so_far = slack_time[v][i];
										} else {
											slack_time[v][i] = min_slack_time_so_far;
										}
									}

								//<<"hierx56"<<endl;
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


								//<<"5hierxx"<<endl;
								//no feasible insertion for destination according to constraints (probably time constrains since the travel time from the depot)
								//try to insert giving flexibility according to latest_arrivel_time
								min_increase_length = INT_MAX;
								repeated_station = false;
								sel_destination = -1;
								flexibilize_arrival_time = true;
								see_if_arrival_departure_dont_match(v);
								cheapest_destination(p, v, pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);

								if (delay[p] <= max_flex_delay)
									accept_delay_trip = true;
								else 
									accept_delay_trip = false;
								if ((sel_destination != -1) && (accept_delay_trip) && (not infeasible_insertion)) {
									//<<"pos dest: "<<sel_destination<<endl;
									/*if ((v > total_number_vehicles) or (v < 0)) {
										cout<<v<<endl;
										cout<<"exiting here PROBLM"<<endl;
										return;
									}
									if (pos_destination-1 < 0){
										cout<<pos_destination<<endl;
										cout<<"exiting here2 PROBLM"<<endl;
										return;
									}
									if ((sel_destination > maxstations) or (sel_destination < 0)){
										cout<<sel_destination<<endl;
										cout<<"exiting here3 PROBLM"<<endl;
										return;
									}

									if ((p > 20000) or (p < 0)){
										cout<<p<<endl;
										cout<<"exiting here4 PROBLM"<<endl;
										return;
									}*/

									not_feasible_insertion = false;
									stops[v].insert(stops[v].begin() + pos_destination, sel_destination);
									
									passengers_departure_time_from_home[p] = best_departure_time_from_home;
									//update passenger performing actions on the stops
									
									action_passengers[v].insert(action_passengers[v].begin() + pos_destination, vector<int>());
									action_passengers[v][pos_destination].resize(15);
									action_passengers[v][pos_destination][0] = p;
									//action_passengers[v][pos_destination].insert(action_passengers[v][pos_destination].begin(), p);
									number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_destination, 1);


									//updating arrival and departure from stops
									int arr_time = departure_time_stop[v][pos_destination-1]+travel_time[stops[v][pos_destination-1]][sel_destination];
									//<<"xx "<<stops[v][pos_destination-1]<<endl;
									arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_destination, arr_time);
									departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_destination, arr_time);
									slack_time[v].insert(slack_time[v].begin() + pos_destination, latest_arrival[p]-arrival_time_stop[v][pos_destination]);
									//slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
									if (not same_station)
										slack_time[v][pos_origin] = latest_arrival[p]-arrival_time_stop[v][pos_destination];
									else {
										int slc_time = latest_arrival[p]-arrival_time_stop[v][pos_destination];
										if (slc_time < slack_time[v][pos_origin])
											slack_time[v][pos_origin] = slc_time;
									}

									prv_capacity = free_capacity[v][pos_destination-1];
									free_capacity[v].insert(free_capacity[v].begin() + pos_destination, prv_capacity+1);

									//updates solution cost
									//<<"heere4"<<endl;
									user_ride_time[p] = travel_time[sel_origin][sel_destination];
									//total_user_ride_time += user_ride_time[p];
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
									//<<"as :"<<stops[v][pos_destination]<<" "<<stops[v][pos_destination+1]<<endl;
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

									 
									//<<"5hierxx"<<endl;
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

										//action_passengers[v][pos_origin].erase(action_passengers[v][pos_origin].begin());

										number_passengers_action[v][pos_origin]--;

										arrival_time_stop[v][pos_origin] = prv_arr_time_at_origin;
										departure_time_stop[v][pos_origin] = prv_dpt_time_at_origin;
										free_capacity[v][pos_origin]++;

									}
									no_feasible_insertion_empty = true;
									//ultimately serve passenger with a 3rd party vehicle
									//serve_passenger_third_party_vehicle(p);
								
								}
							}

							update_URT(v);

						} else {
							no_feasible_insertion_empty = true;
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

				}
			}
		}

	}
	//empty 

	
	//route_assigned[p] = 1;
	//reset blocked vehicles structure
	//<<"almost exiting"<<endl;
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[p][i] = 0;
	//if (vehicle_assigned[p] == -1)
		//cout<<"exiting cluster: "<<cluster_id<<endl;
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

void times_validation(int v) {

	if (number_stops[v] > 2) {
		for (int i=2; i<=number_stops[v];i++) {

			int real_arrival = departure_time_stop[v][i-1]+travel_time[stops[v][i-1]][stops[v][i]];
			if (arrival_time_stop[v][i] != real_arrival){
				cout<<"wrong arrival"<<endl;
				cout<<"pos: "<<i<<endl;
				cout<<"real arrival "<<real_arrival<<" wrong arrival "<<arrival_time_stop[v][i]<<endl;
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
	}
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
						if (x < exp(delta / (temperature))) {
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

void re_insertion(int p, bool &accept_relocate_trip, double &temperature, int &type_move, int cluster_id, int &addedAtV, int vp){

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
	//if (filtered_vehicles.size()>0)
	//	filtered_vehicles.clear();

	bool infeasible_insertion = false;
	//total_difference = 0;
	//oldy_urt=total_user_ride_time;
	//check for closest depot that has an empty vehicle
	//second check for highest capacitated vehicle from that depot
	/*for (int i = 0; i < number_depots; i++) {

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
	}*/

	Insertions insertions_p[45000];
	int curr_number_insertions_p = 0; 

	vector<int> filtered_vehicles_p;
	//if (best_empty_vehicle != -1)
	//	filtered_vehicles_p.push_back(best_empty_vehicle);

	//if (filtered_vehicles.size() == 0)
	
	//filter_vehicles(p, cluster_id, filtered_vehicles_p);
	bool restriced_assignment = false;

	double y = (double)rand() / (double)RAND_MAX;

	if (y <= 0.5) {
		restriced_assignment = true;
	} else {
		restriced_assignment = false;
	}

	restriced_assignment = false;

	if (restriced_assignment) {

		if (filtered_vehicles_p.size()>0)
			filtered_vehicles_p.clear();
		int vv;
		for (int i = 0; i < clusters[cluster_id].size(); i++) {
			vv = clusters[cluster_id][i];

			if ((free_capacity[vv].size() == 2) && (free_capacity[vv][0] < free_capacity[vp][0])) { //means that vehicle is empty and still at the depot
				filtered_vehicles_p.push_back(vv);
			}

		}

		filter_vehicles_restricted(p, cluster_id, vp, filtered_vehicles_p);

		/*cout<<free_capacity[vp][0]<<endl;
		for (int xx = 0; xx < filtered_vehicles.size(); xx++){
			cout<<free_capacity[filtered_vehicles[xx]][0]<<" ";
		}
		cout<<endl;*/


	} else {

		if (filtered_vehicles_p.size()>0)
			filtered_vehicles_p.clear();
		int vv;
		for (int i = 0; i < clusters[cluster_id].size(); i++) {
			vv = clusters[cluster_id][i];

			if (free_capacity[vv].size() == 2){ //means that vehicle is empty and still at the depot
				filtered_vehicles_p.push_back(vv);
			}

		}

		//only attempts to insert in other vehicles if there is no priority to empty vehicles
		if (!priority_empty_vehicle)
			filter_vehicles2(p, cluster_id, filtered_vehicles_p);


	}

	/*if (priority_empty_vehicle) {
		cout<<"filtered vehicles size: "<<filtered_vehicles_p.size()<<endl;
	}*/

	//<<"filtered_vehicles SIZE "<<filtered_vehicles.size()<<endl;
	while ((not_feasible_insertion) && (iterations <= filtered_vehicles_p.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		curr_number_insertions_p = 0;
		
		for (int vf=0; vf<filtered_vehicles_p.size();vf++) {
			int v = filtered_vehicles_p[vf];
			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			if (blocked_vehicles[p][v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				
				see_if_arrival_departure_dont_match(v);
				//cout<<"HEEEERE "<<endl;
				if (curr_number_insertions_p < 44955)
					cheapest_origin2_p(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, insertions_p, curr_number_insertions_p);
				
			}
		}
		//<<endl;

		//<<"curr insert2: " << curr_number_insertions<<endl;
		sort(insertions_p, insertions_p+curr_number_insertions_p, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		/*if (priority_empty_vehicle) {
			cout<<"number insertios "<<curr_number_insertions_p<<endl;
		}*/
		if (curr_number_insertions_p > 0) {
		
			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions_p)) {

				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				best_min_increase_length = insertions_p[iterations2].increase_length;
				best_sel_origin = insertions_p[iterations2].sel_station;
				best_pos_origin = insertions_p[iterations2].pos_station;
				best_v = insertions_p[iterations2].v;
				best_repeated_station = insertions_p[iterations2].repeated_station;

				if (not best_repeated_station) {
					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].resize(15);
					action_passengers[best_v][best_pos_origin][0] = p;
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
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
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					if (number_passengers_action[best_v][best_pos_origin] < action_passengers[best_v][best_pos_origin].size()) {
						action_passengers[best_v][best_pos_origin][number_passengers_action[best_v][best_pos_origin]] = p;
					} else
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin() + number_passengers_action[best_v][best_pos_origin], p);
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
					saved_arrival_time[best_v][i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[best_v][i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					} else {
						bool update_DPT_time = true;
						int greatest_ed = INT_MIN;
						for (int l = 0; l < number_passengers_action[best_v][i];l++){
							int pp = action_passengers[best_v][i][l];
							if (earliest_departure[pp] > arrival_time_stop[best_v][i])
								update_DPT_time = false;
							if (earliest_departure[pp] > greatest_ed)
								greatest_ed = earliest_departure[pp];
						}

						if (update_DPT_time){
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						} else {
							if (greatest_ed < departure_time_stop[best_v][i])
								departure_time_stop[best_v][i] = greatest_ed;
						}
					}
					saved_slack_time[best_v][i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[best_v][i];
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

				
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
			
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

				//if (delay[p] <= max_flex_delay)
				//	accept_delay_trip = true;
				//else 
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

				
				//computedDELTA=DELTA;
				addedAtV = best_v;
				//<<"oldy and delta "<<oldy_urt<<DELTA<<endl;
				if ((sel_destination != -1) && (no_violation_capacity) && (accept_relocate_trip) && (not infeasible_insertion)) {

					passengers_departure_time_from_home[p] = insertions_p[iterations2].passengers_departure_time_from_home;
					type_move = 1;
					vehicle_assigned[p] = best_v;

					//<<"feasible and reduce fare1"<<endl;
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;

					
					if (not repeated_station) {
						//<<"hierrs"<<endl;
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].resize(15);
						action_passengers[best_v][pos_destination][0] = p;
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
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
						//<<"hierNrs1"<<endl;
						//update passenger performing actions on the stops
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						
						if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
							action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
						} else
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
						number_passengers_action[best_v][pos_destination]++;

						

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						//if (slc_time < slack_time[best_v][pos_destination]) 
						//	slack_time[best_v][pos_destination] = slc_time;

						int min_slck_time = INT_MAX;
						for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
							int px = action_passengers[best_v][pos_destination][l];
							int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < min_slck_time) {
								min_slck_time = slc_time;
							}
						}
						slack_time[best_v][pos_destination] = min_slck_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}


					//update further arrival, departure times, and slack times
					for (int i=pos_destination; i<=number_stops[best_v];i++) {
						old_arr_time = arrival_time_stop[best_v][i];
						//<<"aki "<<" "<<departure_time_stop[best_v][i-1]<<" "<<travel_time[stops[best_v][i-1]][stops[best_v][i]]<<" "<<stops[best_v][i-1]<<" "<<stops[best_v][i]<<endl;
						arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
						old_dpt_time = departure_time_stop[best_v][i];
						//if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						//}

						//<<arrival_time_stop[best_v][i]<<" "<<departure_time_stop[best_v][i]<<endl;

						int greatest_ed = INT_MIN;
						//the departure time of a stop can change according to the earliest departure of passengers assigned to that stop
						for (int l=0; l<number_passengers_action[best_v][i];l++) {
							if (earliest_departure[action_passengers[best_v][i][l]] > greatest_ed)
								greatest_ed = earliest_departure[action_passengers[best_v][i][l]];
						}

						if (greatest_ed > departure_time_stop[best_v][i])
							departure_time_stop[best_v][i] = greatest_ed;

						slack_time[best_v][i] -= arrival_time_stop[best_v][i] - old_arr_time;
						//<<arrival_time_stop[best_v][i]<<" "<<departure_time_stop[best_v][i]<<endl;
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

										//if (current_user_ride_time != user_ride_time_temp[save_p]) {
											//<<"crime fake add "<<save_p<<endl;
										//}
										if (current_user_ride_time != user_ride_time[save_p]) {
											difference = current_user_ride_time -  user_ride_time[save_p];
											//<<"difference: "<<difference<<endl;
											//total_user_ride_time += difference;
											//total_difference += difference;
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
						if (x < exp(delta / (temperature))) {
							//non improving move is accepted according to SA
							accept_relocate_trip = true;
						} else {
							accept_relocate_trip = false;
						}
					} else {
						accept_relocate_trip = false;
					} 

					if (accept_relocate_trip) {

						passengers_departure_time_from_home[p] = insertions_p[iterations2].passengers_departure_time_from_home;
						type_move = 2;
						vehicle_assigned[p] = best_v;

						//<<"feasible and accepted according to SA (its NON IMPROVING)"<<endl;
						no_feasible_insertion2 = false;
						not_feasible_insertion = false;
						
						
						if (not repeated_station) {

							//<<"huiishx"<<endl;
							//<<"vehicle "<<best_v<<endl;
							stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
							
							number_stops[best_v]++;


							//update passenger performing actions on the stops
							action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
							
							action_passengers[best_v][pos_destination].resize(15);
							action_passengers[best_v][pos_destination][0] = p;
							//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
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
							//<<"huiishxESe"<<endl;
							//<<"hereeeeLSE"<<endl;
							//update passenger performing actions on the stops
							//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
							if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
								action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
							} else
								action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
							number_passengers_action[best_v][pos_destination]++;

							//according to the update when the origin is added, if the stop already exists then no need to change
							//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
							//arrival_time_stop[best_v][pos_destination] = arr_time;
							//departure_time_stop[best_v][pos_destination] = arr_time;
							//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
							//if (slc_time < slack_time[best_v][pos_destination]) 
							//	slack_time[best_v][pos_destination] = slc_time;
							int min_slck_time = INT_MAX;
							for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
								int px = action_passengers[best_v][pos_destination][l];
								int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
								if (slc_time < min_slck_time) {
									min_slck_time = slc_time;
								}
							}
							slack_time[best_v][pos_destination] = min_slck_time;

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
												//total_user_ride_time += difference;
												//total_difference += difference;
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

						/*cout<<"BEFORE_SIMPLE RELOCATE"<<endl;
						for (int kk=0;kk<900;kk++){
							if (vehicle_assigned[kk] != -1) {
								solution_validation(kk, vehicle_assigned[kk]);
							//served_passengers++;
							}
						}*/
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
							
							//<<"hxxx"<<endl;
							stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
							action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
							number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
							

							//re-update further arrival and departure times
							//<<"ns: "<<best_pos_origin<<" "<<number_stops[best_v]<<endl;
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
								departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
								slack_time[best_v][i] = saved_slack_time[best_v][i];
								//<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
							}
							number_stops[best_v]--;
							arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
							departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
							slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


							free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


						} else {

							//<<"hed1ere"<<endl;
							//<<action_passengers[best_v][best_pos_origin][1]<<endl;
							//action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
							
							number_passengers_action[best_v][best_pos_origin]--;

							arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
							departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
							free_capacity[best_v][best_pos_origin]++;

							//re-update further arrival and departure times
							for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
								arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
								departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
								slack_time[best_v][i] = saved_slack_time[best_v][i];
							}

						}

						if (best_pos_origin == 1) {
							int new_dpt_time = arrival_time_stop[best_v][1]-travel_time[stops[best_v][0]][stops[best_v][1]];
							if (new_dpt_time < current_time){
								new_dpt_time = current_time;
							}

						
							departure_time_stop[best_v][0] = new_dpt_time;
						}
					}

				}

				/*cout<<"AFTER_SIMPLE UPDATE"<<endl;
				for (int kk=0;kk<900;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}*/

				update_URT(best_v);
				//print_v_vehicle(best_v);
				

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
			iterations = filtered_vehicles_p.size()+1;
		}

		//<<"hieerxx";
		if (no_feasible_insertion2) {

			type_move = 3;
			//iterations++;
			iterations = filtered_vehicles_p.size()+1;
		}
		if (iterations >= filtered_vehicles_p.size()) {
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

	//if (priority_empty_vehicle) {
	for (int i = 0; i < clusters[cluster_id].size(); i++) {
		int vv = clusters[cluster_id][i];
		see_if_departure_empty_vehicle_not_zero(vv);
	}
	//}

	//<<"hieerxx2";
	//route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[p][i] = 0;
}

std::vector<int> new_insertions_v;
std::vector<int> new_insertions_p;

void re_insertion_nn(int p, bool &accept_relocate_trip, double &temperature, int &type_move, int &diffURT, int cluster_id, int vp){

	accept_relocate_trip = false; 
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
	//if (filtered_vehicles.size()>0)
	//	filtered_vehicles.clear();
	bool infeasible_insertion = false;
	
	Insertions insertions_p[45000];
	int curr_number_insertions_p = 0;
	vector<int> filtered_vehicles_p;
	//if (best_empty_vehicle != -1)
	//	filtered_vehicles_p.push_back(best_empty_vehicle);

	//if (filtered_vehicles.size() == 0)
	
	//filter_vehicles(p, cluster_id, filtered_vehicles_p);


	bool restriced_assignment = false;

	double y = (double)rand() / (double)RAND_MAX;

	if (y <= 0.5) {
		restriced_assignment = true;
	} else {
		restriced_assignment = false;
	}

	restriced_assignment = false;

	if (restriced_assignment) {

		if (filtered_vehicles_p.size()>0)
			filtered_vehicles_p.clear();
		int vv;
		for (int i = 0; i < clusters[cluster_id].size(); i++) {
			vv = clusters[cluster_id][i];

			if ((free_capacity[vv].size() == 2) && (free_capacity[vv][0] < free_capacity[vp][0])) { //means that vehicle is empty and still at the depot
				filtered_vehicles_p.push_back(vv);
			}

		}

		filter_vehicles_restricted(p, cluster_id, vp, filtered_vehicles_p);

		/*cout<<free_capacity[vp][0]<<endl;
		for (int xx = 0; xx < filtered_vehicles.size(); xx++){
			cout<<free_capacity[filtered_vehicles[xx]][0]<<" ";
		}
		cout<<endl;*/


	} else {

		if (filtered_vehicles_p.size()>0)
			filtered_vehicles_p.clear();

		int vv;
		for (int i = 0; i < clusters[cluster_id].size(); i++) {
			vv = clusters[cluster_id][i];

			if (free_capacity[vv].size() == 2){ //means that vehicle is empty and still at the depot
				filtered_vehicles_p.push_back(vv);
			}

		}

		filter_vehicles2(p, cluster_id, filtered_vehicles_p);


	}
	//<<"here1"<<endl;
	//<<filtered_vehicles_p.size()<<endl;
	//<<"filtered_vehicles SIZE "<<filtered_vehicles.size()<<endl;
	while ((not_feasible_insertion) && (iterations <= filtered_vehicles_p.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		curr_number_insertions_p = 0;
		//<<"here2"<<endl;

		//<<"fps:"<<filtered_vehicles_p.size()<<" "<<p<<" "<<v<<" "<<endl;
		for (int vf=0; vf<filtered_vehicles_p.size();vf++) {
			int v = filtered_vehicles_p[vf];
			//<<"here2.0"<<endl;

			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			//<<"here2.1"<<endl;
			if (blocked_vehicles[p][v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				//<<"here2.2"<<endl;
				see_if_arrival_departure_dont_match(v);
				//cout<<"here2.3"<<endl;
				if (curr_number_insertions_p < 44955)
					cheapest_origin2_p(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, insertions_p, curr_number_insertions_p);
				//<<"here2.4"<<endl;		
			}
			//<<"here2.5"<<endl;
		}
		//<<endl;

		//<<"curr insert3: " << curr_number_insertions<<endl;
		//<<"here3"<<endl;
		sort(insertions_p, insertions_p+curr_number_insertions_p, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions_p > 0) {
			//<<"here4"<<endl;
			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions_p)) {

				//<<best_v<<"\n";
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				best_min_increase_length = insertions_p[iterations2].increase_length;
				best_sel_origin = insertions_p[iterations2].sel_station;
				best_pos_origin = insertions_p[iterations2].pos_station;
				best_v = insertions_p[iterations2].v;
				best_repeated_station = insertions_p[iterations2].repeated_station;
				

				/*cout<<"flex TTSWWDTII: "<<flexibilize_lat_departure_time<<endl;
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
				//<<"here5"<<endl;
				if (not best_repeated_station) {
					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].resize(15);
					action_passengers[best_v][best_pos_origin][0] = p;
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
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
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					if (number_passengers_action[best_v][best_pos_origin] < action_passengers[best_v][best_pos_origin].size()) {
						action_passengers[best_v][best_pos_origin][number_passengers_action[best_v][best_pos_origin]] = p;
					} else
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin() + number_passengers_action[best_v][best_pos_origin], p);
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
				//<<"here6"<<endl;
				//update further arrival, departure times and slack times
				for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
					saved_arrival_time[best_v][i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[best_v][i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					} else {
						bool update_DPT_time = true;
						int greatest_ed = INT_MIN;
						for (int l = 0; l < number_passengers_action[best_v][i];l++){
							int pp = action_passengers[best_v][i][l];
							if (earliest_departure[pp] > arrival_time_stop[best_v][i])
								update_DPT_time = false;
							if (earliest_departure[pp] > greatest_ed)
								greatest_ed = earliest_departure[pp];
						}

						if (update_DPT_time){
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						} else {
							if (greatest_ed < departure_time_stop[best_v][i])
								departure_time_stop[best_v][i] = greatest_ed;
						}
					}
					saved_slack_time[best_v][i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[best_v][i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				

				//if (best_v == 24) {
				/*cout<<"flex RRN: "<<flexibilize_lat_departure_time<<endl;
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
				//<<"here7"<<endl;
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
				//<<"here8"<<endl;
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

				//if (delay[p] <= max_flex_delay)
				//	accept_delay_trip = true;
				//else 
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

				/*int DELTA;
				int diff_remov_passenger = compute_difference_URT_by_fake_removing_passenger(p);
				int diff_add_passenger = 0;
				if ((sel_destination != -1) && (no_violation_capacity))
					diff_add_passenger = compute_difference_URT_by_fake_adding_passenger(p, best_v, sel_destination, pos_destination, repeated_station);
				DELTA = diff_remov_passenger + diff_add_passenger;*/

				//if (diff_remov_passenger < 0)
				//	cout<<"NEGATIVE ERROR"<<endl;
				//DELTA += diff_remov_passenger;
				//accept_relocate_trip = false;
				//if (DELTA > 0)
					accept_relocate_trip = true; //first feasible is accepted

				//printf("DELTA:%d \n", DELTA);
					//<<"here9"<<endl;
				//totalcomputedDELTA += DELTA;
				if ((sel_destination != -1) && (no_violation_capacity) && (accept_relocate_trip) && (not infeasible_insertion)) {

					//diffURT = DELTA;
					new_insertions_v.push_back(best_v);
					new_insertions_p.push_back(p);
					passengers_departure_time_from_home[p] = insertions_p[iterations2].passengers_departure_time_from_home;
					type_move = 1;
					vehicle_assigned[p] = best_v;
					//cout<<"accepted new trip "<<p<<" "<<best_v<<endl;

					//<<"feasible and reduce fare2"<<endl;
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;
					
					if (not repeated_station) {
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].resize(15);
						action_passengers[best_v][pos_destination][0] = p;
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
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
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
							action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
						} else
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
						number_passengers_action[best_v][pos_destination]++;

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						//if (slc_time < slack_time[best_v][pos_destination]) 
						//	slack_time[best_v][pos_destination] = slc_time;
						int min_slck_time = INT_MAX;
						for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
							int px = action_passengers[best_v][pos_destination][l];
							int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < min_slck_time) {
								min_slck_time = slc_time;
							}
						}
						slack_time[best_v][pos_destination] = min_slck_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}

					//<<"here10"<<endl;
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
											//total_user_ride_time += difference;
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
					//<<"here11"<<endl;
					if (sel_destination != -1) {
						for (int i=best_pos_origin+1;i<pos_destination;i++){

							free_capacity[best_v][i]++;
		
						}
					}

					if (number_passengers_action[best_v][best_pos_origin] == 1) {
						
						stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
						action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
						number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
						

						//re-update further arrival and departure times
						//<<"ns: "<<best_pos_origin<<" "<<number_stops[best_v]<<endl;
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
							//<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
						}
						number_stops[best_v]--;
						arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
						departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
						slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


						free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


					} else {

						//<<"heere"<<endl;
						//<<action_passengers[best_v][best_pos_origin][1]<<endl;
						//action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
						
						number_passengers_action[best_v][best_pos_origin]--;

						arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
						departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
						free_capacity[best_v][best_pos_origin]++;

						//re-update further arrival and departure times
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
							//<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
						}
					}

					if (best_pos_origin == 1) {
						int new_dpt_time = arrival_time_stop[best_v][1]-travel_time[stops[best_v][0]][stops[best_v][1]];
						if (new_dpt_time < current_time){
							new_dpt_time = current_time;
						}

					
						departure_time_stop[best_v][0] = new_dpt_time;
					}
				}
				//<<"here12"<<endl;
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
			iterations = filtered_vehicles_p.size()+1;
		}
		//<<"here13"<<endl;
		if (no_feasible_insertion2) {

			type_move = 3;
			//iterations++;
			iterations = filtered_vehicles_p.size()+1;
		}
		//<<"here14"<<endl;
		if (iterations >= filtered_vehicles_p.size()) {
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
	//<<"here15"<<endl;
	//route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[p][i] = 0;
	//<<"here16"<<endl;

	for (int i = 0; i < clusters[cluster_id].size(); i++) {
		int vv = clusters[cluster_id][i];
		see_if_departure_empty_vehicle_not_zero(vv);
	}
}

void relocate_all_passengers_vehicle_nn(int v, double &temperature, int &type_move, int& counter, int& deltaURT, bool&megaerror, int cluster_id){


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
	}
	cout<<endl;*/

	//<<"here1"<<endl;
	for (int kk=0; kk<passengers_at_vehicle[v].size();kk++){
		p = passengers_at_vehicle[v][kk]; //passenger that will try to relocate
		//<<"here2"<<endl;
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

		//if (vehicle_assigned[p] != v)
			//<<"ERROR HYER"<<endl;

		blocked_vehicles[p][vehicle_assigned[p]] = 0;

		accept_relocate_trip = false;
		//<<"here3"<<endl;
		re_insertion_nn(p, accept_relocate_trip, temperature, type_move, diffURT, cluster_id, v);
		//<<"OUT1"<<endl;
		int begin, end;
		begin = 0;
		end = 0;


		if (accept_relocate_trip) {
			//<<"OUT2"<<endl;
			counter++; //one passenger has been succesfully been relocated
			
			count = 0;
			//printf("remove heeeere\n");
			//<<"number stops "<<number_stops[v]<<endl;
			//deltaURT += diffURT;
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

void re_insertion_swap(int p, bool &accept_relocate_trip, double &temperature, int &type_move, int &diffURT, int cluster_id, int vp, int vs){

	accept_relocate_trip = false; 
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
	//if (filtered_vehicles.size()>0)
	//	filtered_vehicles.clear();
	bool infeasible_insertion = false;
	
	Insertions insertions_p[45000];
	int curr_number_insertions_p = 0;
	vector<int> filtered_vehicles_p;
	//if (best_empty_vehicle != -1)
	//	filtered_vehicles_p.push_back(best_empty_vehicle);

	//if (filtered_vehicles.size() == 0)
	
	//filter_vehicles(p, cluster_id, filtered_vehicles_p);


	bool restriced_assignment = false;

	double y = (double)rand() / (double)RAND_MAX;

	if (y <= 0.5) {
		restriced_assignment = true;
	} else {
		restriced_assignment = false;
	}

	restriced_assignment = false;

	if (restriced_assignment) {

		if (filtered_vehicles_p.size()>0)
			filtered_vehicles_p.clear();
		int vv;
		for (int i = 0; i < clusters[cluster_id].size(); i++) {
			vv = clusters[cluster_id][i];

			if ((free_capacity[vv].size() == 2) && (free_capacity[vv][0] < free_capacity[vp][0])) { //means that vehicle is empty and still at the depot
				filtered_vehicles_p.push_back(vv);
			}

		}

		filter_vehicles_restricted(p, cluster_id, vp, filtered_vehicles_p);

		/*cout<<free_capacity[vp][0]<<endl;
		for (int xx = 0; xx < filtered_vehicles.size(); xx++){
			cout<<free_capacity[filtered_vehicles[xx]][0]<<" ";
		}
		cout<<endl;*/


	} else {

		if (filtered_vehicles_p.size()>0)
			filtered_vehicles_p.clear();

		/*int vv;
		for (int i = 0; i < clusters[cluster_id].size(); i++) {
			vv = clusters[cluster_id][i];

			if (free_capacity[vv].size() == 2){ //means that vehicle is empty and still at the depot
				filtered_vehicles_p.push_back(vv);
			}

		}

		filter_vehicles2(p, cluster_id, filtered_vehicles_p);*/
		//vs is the only vehicle it can be assigned
		filtered_vehicles_p.push_back(vs); 


	}
	//cout<<"herecpcpcp "<<vs<<endl;
	//<<filtered_vehicles_p.size()<<endl;
	//<<"filtered_vehicles SIZE "<<filtered_vehicles.size()<<endl;
	while ((not_feasible_insertion) && (iterations <= filtered_vehicles_p.size())) {
		
		int best_pos_origin, best_pos_destination;
		int best_min_increase_length;
		int best_sel_origin, best_sel_destination, best_v;
		bool best_repeated_station;
		
		best_min_increase_length = INT_MAX;
		best_v = -1;
		curr_number_insertions_p = 0;
		//<<"here2"<<endl;

		//<<"fps:"<<filtered_vehicles_p.size()<<" "<<p<<" "<<v<<" "<<endl;
		for (int vf=0; vf<filtered_vehicles_p.size();vf++) {
			int v = filtered_vehicles_p[vf];
			//<<"here2.0"<<endl;

			//<<v<<" "<<free_capacity[v]<<endl;
			min_increase_length = INT_MAX;
			repeated_station = false;
			//(free_capacity[v] > 0) && 
			//<<"here2.1"<<endl;
			if (blocked_vehicles[p][v] == 0) {

				if (tested_all_vehicles_once)
					flexibilize_lat_departure_time = true;
				else
					flexibilize_lat_departure_time = false;
				//<<"here2.2"<<endl;
				see_if_arrival_departure_dont_match(v);
				//cout<<"here2.3"<<endl;
				if (curr_number_insertions_p < 44955)
					cheapest_origin2_p(p, v, min_increase_length, sel_origin, pos_origin, repeated_station, flexibilize_lat_departure_time, insertions_p, curr_number_insertions_p);
				//<<"here2.4"<<endl;		
			}
			//<<"here2.5"<<endl;
		}
		//<<endl;

		//cout<<"curr insert3: " << curr_number_insertions_p<<endl;
		//<<"here3"<<endl;
		sort(insertions_p, insertions_p+curr_number_insertions_p, comparator);

		//<<best_v<<endl;
		//if (best_v != -1) {
		bool no_feasible_insertion2 = true;
		int iterations2 = 0;
		if (curr_number_insertions_p > 0) {
			//<<"here4"<<endl;
			while ((no_feasible_insertion2) && (iterations2 < curr_number_insertions_p)) {

				
				int prv_arr_time_at_origin, prv_dpt_time_at_origin;

				best_min_increase_length = insertions_p[iterations2].increase_length;
				best_sel_origin = insertions_p[iterations2].sel_station;
				best_pos_origin = insertions_p[iterations2].pos_station;
				best_v = insertions_p[iterations2].v;
				best_repeated_station = insertions_p[iterations2].repeated_station;
				//cout<<"CHANGING HERE "<<p<<" "<<best_v<<"\n";

				/*cout<<"flex TTSWWDTII: "<<flexibilize_lat_departure_time<<endl;
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
				//<<"here5"<<endl;
				if (not best_repeated_station) {
					stops[best_v].insert(stops[best_v].begin() + best_pos_origin, best_sel_origin);
					number_stops[best_v]++;

					//update passenger performing actions on the stops
					action_passengers[best_v].insert(action_passengers[best_v].begin() + best_pos_origin, vector<int>());
					action_passengers[best_v][best_pos_origin].resize(15);
					action_passengers[best_v][best_pos_origin][0] = p;
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
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
					//action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin(), p);
					if (number_passengers_action[best_v][best_pos_origin] < action_passengers[best_v][best_pos_origin].size()) {
						action_passengers[best_v][best_pos_origin][number_passengers_action[best_v][best_pos_origin]] = p;
					} else
						action_passengers[best_v][best_pos_origin].insert(action_passengers[best_v][best_pos_origin].begin() + number_passengers_action[best_v][best_pos_origin], p);
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
				//<<"here6"<<endl;
				//update further arrival, departure times and slack times
				for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
					saved_arrival_time[best_v][i] = arrival_time_stop[best_v][i];
					arrival_time_stop[best_v][i] = departure_time_stop[best_v][i-1]+travel_time[stops[best_v][i-1]][stops[best_v][i]];
					saved_departure_time[best_v][i] = departure_time_stop[best_v][i];
					if (arrival_time_stop[best_v][i] > departure_time_stop[best_v][i]) {
						departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
					} else {
						bool update_DPT_time = true;
						int greatest_ed = INT_MIN;
						for (int l = 0; l < number_passengers_action[best_v][i];l++){
							int pp = action_passengers[best_v][i][l];
							if (earliest_departure[pp] > arrival_time_stop[best_v][i])
								update_DPT_time = false;
							if (earliest_departure[pp] > greatest_ed)
								greatest_ed = earliest_departure[pp];
						}

						if (update_DPT_time){
							departure_time_stop[best_v][i] = arrival_time_stop[best_v][i];
						} else {
							if (greatest_ed < departure_time_stop[best_v][i])
								departure_time_stop[best_v][i] = greatest_ed;
						}
					}
					saved_slack_time[best_v][i] = slack_time[best_v][i];
					slack_time[best_v][i] -= arrival_time_stop[best_v][i] - saved_arrival_time[best_v][i];
					//<<slack_time[best_v][i]<<endl<<arrival_time_stop[best_v][i]<<endl<<saved_arrival_time[i]<<endl<<endl;
				}
				
				
				//solution_cost += best_min_increase_length;
				//saved_increase = best_min_increase_length;
				
				

				//if (best_v == 24) {
				/*cout<<"INSERTING ORIGIN SWAP: "<<flexibilize_lat_departure_time<<endl;
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
				//<<"here7"<<endl;
				cheapest_destination(p, best_v, best_pos_origin, min_increase_length, sel_destination, pos_destination, repeated_station, flexibilize_arrival_time, infeasible_insertion);
				//<<"here8"<<endl;
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

				//if (delay[p] <= max_flex_delay)
				//	accept_delay_trip = true;
				//else 
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

				/*int DELTA;
				int diff_remov_passenger = compute_difference_URT_by_fake_removing_passenger(p);
				int diff_add_passenger = 0;
				if ((sel_destination != -1) && (no_violation_capacity))
					diff_add_passenger = compute_difference_URT_by_fake_adding_passenger(p, best_v, sel_destination, pos_destination, repeated_station);
				DELTA = diff_remov_passenger + diff_add_passenger;*/

				//if (diff_remov_passenger < 0)
				//	cout<<"NEGATIVE ERROR"<<endl;
				//DELTA += diff_remov_passenger;
				//accept_relocate_trip = false;
				//if (DELTA > 0)
					accept_relocate_trip = true; //first feasible is accepted

				//printf("DELTA:%d \n", DELTA);
					//<<"here9"<<endl;
				//totalcomputedDELTA += DELTA;
				if ((sel_destination != -1) && (no_violation_capacity) && (accept_relocate_trip) && (not infeasible_insertion)) {

					//diffURT = DELTA;
					new_insertions_v.push_back(best_v);
					new_insertions_p.push_back(p);
					passengers_departure_time_from_home[p] = insertions_p[iterations2].passengers_departure_time_from_home;
					type_move = 1;
					vehicle_assigned[p] = best_v;
					//cout<<"accepted new trip "<<p<<" "<<best_v<<endl;

					//<<"feasible and reduce fare2"<<endl;
					no_feasible_insertion2 = false;
					not_feasible_insertion = false;
					
					if (not repeated_station) {
						stops[best_v].insert(stops[best_v].begin() + pos_destination, sel_destination);
						number_stops[best_v]++;

						//update passenger performing actions on the stops
						action_passengers[best_v].insert(action_passengers[best_v].begin() + pos_destination, vector<int>());
						action_passengers[best_v][pos_destination].resize(15);
						action_passengers[best_v][pos_destination][0] = p;
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
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
						//action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin(), p);
						if (number_passengers_action[best_v][pos_destination] < action_passengers[best_v][pos_destination].size()) {
							action_passengers[best_v][pos_destination][number_passengers_action[best_v][pos_destination]] = p;
						} else
							action_passengers[best_v][pos_destination].insert(action_passengers[best_v][pos_destination].begin() + number_passengers_action[best_v][pos_destination], p);
						number_passengers_action[best_v][pos_destination]++;

						//according to the update when the origin is added, if the stop already exists then no need to change
						//int arr_time = departure_time_stop[best_v][pos_destination-1]+travel_time[stops[best_v][pos_destination-1]][sel_destination];
						//arrival_time_stop[best_v][pos_destination] = arr_time;
						//departure_time_stop[best_v][pos_destination] = arr_time;
						//int slc_time = latest_arrival[p]-arrival_time_stop[best_v][pos_destination];
						//if (slc_time < slack_time[best_v][pos_destination]) 
						//	slack_time[best_v][pos_destination] = slc_time;
						int min_slck_time = INT_MAX;
						for (int l=0;l<number_passengers_action[best_v][pos_destination];l++){
							int px = action_passengers[best_v][pos_destination][l];
							int slc_time = latest_arrival[px]-arrival_time_stop[best_v][pos_destination];
							if (slc_time < min_slck_time) {
								min_slck_time = slc_time;
							}
						}
						slack_time[best_v][pos_destination] = min_slck_time;

						//prv_capacity = free_capacity[best_v][pos_destination];
						//free_capacity[best_v][pos_destination] = prv_capacity+1;

					}

					//<<"here10"<<endl;
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
											//total_user_ride_time += difference;
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
					//<<"here11"<<endl;
					if (sel_destination != -1) {
						for (int i=best_pos_origin+1;i<pos_destination;i++){

							free_capacity[best_v][i]++;
		
						}
					}

					if (number_passengers_action[best_v][best_pos_origin] == 1) {
						
						stops[best_v].erase(stops[best_v].begin() + best_pos_origin);
						action_passengers[best_v].erase(action_passengers[best_v].begin() + best_pos_origin);
						number_passengers_action[best_v].erase(number_passengers_action[best_v].begin() + best_pos_origin);
						

						//re-update further arrival and departure times
						//<<"ns: "<<best_pos_origin<<" "<<number_stops[best_v]<<endl;
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
							//<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
						}
						number_stops[best_v]--;
						arrival_time_stop[best_v].erase(arrival_time_stop[best_v].begin() + best_pos_origin);
						departure_time_stop[best_v].erase(departure_time_stop[best_v].begin() + best_pos_origin);
						slack_time[best_v].erase(slack_time[best_v].begin() + best_pos_origin);


						free_capacity[best_v].erase(free_capacity[best_v].begin() + best_pos_origin);


					} else {

						//<<"heere"<<endl;
						//<<action_passengers[best_v][best_pos_origin][1]<<endl;
						//action_passengers[best_v][best_pos_origin].erase(action_passengers[best_v][best_pos_origin].begin());
						
						number_passengers_action[best_v][best_pos_origin]--;

						arrival_time_stop[best_v][best_pos_origin] = prv_arr_time_at_origin;
						departure_time_stop[best_v][best_pos_origin] = prv_dpt_time_at_origin;
						free_capacity[best_v][best_pos_origin]++;

						//re-update further arrival and departure times
						for (int i=best_pos_origin+1; i<=number_stops[best_v];i++) {
							arrival_time_stop[best_v][i] = saved_arrival_time[best_v][i];
							departure_time_stop[best_v][i] = saved_departure_time[best_v][i];
							slack_time[best_v][i] = saved_slack_time[best_v][i];
							//<<saved_arrival_time[i]<<" "<<saved_departure_time[i]<<" "<<saved_slack_time[i]<<endl;
						}
					}

					if (best_pos_origin == 1) {
						int new_dpt_time = arrival_time_stop[best_v][1]-travel_time[stops[best_v][0]][stops[best_v][1]];
						if (new_dpt_time < current_time){
							new_dpt_time = current_time;
						}

					
						departure_time_stop[best_v][0] = new_dpt_time;
					}
				}
				//<<"here12"<<endl;
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
			iterations = filtered_vehicles_p.size()+1;
		}
		//<<"here13"<<endl;
		if (no_feasible_insertion2) {

			type_move = 3;
			//iterations++;
			iterations = filtered_vehicles_p.size()+1;
		}
		//<<"here14"<<endl;
		if (iterations >= filtered_vehicles_p.size()) {
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
	//<<"here15"<<endl;
	//route_assigned[p] = 1;
	//reset blocked vehicles structure
	for (int i=0;i<total_number_vehicles;i++)
		blocked_vehicles[p][i] = 0;
	//<<"here16"<<endl;

	for (int i = 0; i < clusters[cluster_id].size(); i++) {
		int vv = clusters[cluster_id][i];
		see_if_departure_empty_vehicle_not_zero(vv);
	}
}

void relocate_all_passengers_vehicle_swap(int v, double &temperature, int &type_move, int& counter, int& deltaURT, bool&megaerror, int cluster_id, int vs){


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
	}
	cout<<endl;*/

	//<<"here1"<<endl;
	for (int kk=0; kk<passengers_at_vehicle[v].size();kk++){
		p = passengers_at_vehicle[v][kk]; //passenger that will try to relocate
		//<<"here2"<<endl;
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

		//if (vehicle_assigned[p] != v)
			//<<"ERROR HYER"<<endl;

		blocked_vehicles[p][vehicle_assigned[p]] = 0;
		blocked_vehicles[p][vs] = 0;
		accept_relocate_trip = false;
		//<<"here3"<<endl;
		re_insertion_swap(p, accept_relocate_trip, temperature, type_move, diffURT, cluster_id, v, vs);
		//<<"OUT1"<<endl;
		int begin, end;
		begin = 0;
		end = 0;


		if (accept_relocate_trip) {
			//<<"OUT2"<<endl;
			counter++; //one passenger has been succesfully been relocated
			
			count = 0;
			//printf("remove heeeere\n");
			//<<"number stops "<<number_stops[v]<<endl;
			//deltaURT += diffURT;
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

int compute_cluster_URT(int cluster_id){

	int cluster_URT = 0;

	for (int j=0;j<clusters[cluster_id].size();j++){

		//<<
		int v = clusters[cluster_id][j];
		
		for (int p=0;p<total_requests;p++) {
			if (vehicle_assigned[p] == v) {
				cluster_URT += user_ride_time[p];
			}
		}
	}

	return cluster_URT;
}

void stay_times_consistency_empty_vehicle(int v){

	arrival_time_stop[v][0] = 0;
	arrival_time_stop[v][1] = 86400;

	departure_time_stop[v][0] = 0;
	departure_time_stop[v][1] = 86400;

	slack_time[v][0] = 86400;
	slack_time[v][1] = 86400;

}

void empty_vehicle(int v, bool& megaerror, double &temperature, int &type_move, int cluster_id){

	std::random_device rd;
	std::mt19937 g(rd());

	if (new_insertions_v.size() > 0)
		new_insertions_v.clear();

	if (new_insertions_p.size() > 0)
		new_insertions_p.clear();

	bool all_accepted_relocate_trip = false;
	int counter = 0;
	int deltaURT;

	deltaURT = 0;
	//oldy_urt = total_user_ride_time;

	int prev_URT = compute_cluster_URT(cluster_id);

	
	//totalcomputedDELTA = 0;
	//<<"B"<<endl;
	save_intm_solution(cluster_id);
	//vector<int> edited_vehicles;

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
	}*/

	/*cout<<"passengers to be removed: "<<endl;
	for (int i = 0; i < passengers_at_vehicle[v].size(); i++){
		cout<<passengers_at_vehicle[v][i]<<" ";
	}
	cout<<endl;
	cout<<"number stops "<<number_stops[v]<<endl;*/

	//bool megaerror = false;

	//emptying vehicle before trying to re-insert passengers again
	for (int i = 0; i<passengers_at_vehicle[v].size();i++){
		int p = passengers_at_vehicle[v][i];
		//<<p<<endl;
		remove_passenger_from_vehicle(v, p);
		//update URT
		update_URT(vehicle_assigned[p]);
	}
	see_if_arrival_departure_dont_match(v);
	update_URT(v);
	stay_times_consistency_empty_vehicle(v);
	//cout<<"counter pv1: "<<counter<<" "<<passengers_at_vehicle[v].size()<<endl;
	//randomly shuffle passengers in the vector to insert them randomly
	std::shuffle(passengers_at_vehicle[v].begin(), passengers_at_vehicle[v].end(), g);
	//cout<<"counter pv2: "<<counter<<" "<<passengers_at_vehicle[v].size()<<endl;
	megaerror = false;
	counter = 0;
	relocate_all_passengers_vehicle_nn(v, init_temperature, type_move, counter, deltaURT, megaerror, cluster_id);
	//<<"OUT3"<<endl;
	//UPDATE USER RIDE TIMES TO SEE IF NOW I CAN COMPUTE DELTA CORRECTLY??????
	
	if (megaerror){
		cout<<"mega error?"<<endl;
		return;
	}
	bool accept_relocate_trip;
	//cout<<"counter pv3: "<<counter<<" "<<passengers_at_vehicle[v].size()<<endl;
	if (counter == passengers_at_vehicle[v].size()) {
		
		int new_URT = compute_cluster_URT(cluster_id);
		//<<"oldyxnew: "<<oldy_urt<<" "<<total_user_ride_time<<" "<<(oldy_urt-total_user_ride_time)<<endl; 
		//if ((oldy_urt-total_user_ride_time) > 0){
		if ((prev_URT-new_URT) > 0){
			//<<"REMOVEALLPOSITIVE"<<endl;
			type_move = 1;
		} else {
			int x = (double)rand() / (double)RAND_MAX;
			//int delta = oldy_urt-total_user_ride_time;
			int delta = prev_URT-new_URT;

			if (x < exp(delta / (temperature))) {
				//non improving move is accepted according to SA
				accept_relocate_trip = true;
			} else {
				accept_relocate_trip = false;
			}
			//<<"F0"<<endl;
			if (accept_relocate_trip) {
				//<<"REMOVEALLACCEPT"<<endl;
				type_move = 2;
			} else {
				type_move = 3;
				//<<"back to beginning"<<endl;
				return_intm_solution(cluster_id);
			}
		}
		//<<"F"<<endl;
	} else {
		type_move = 3;
		//<<"revert changes2"<<endl;
		//<<"G"<<endl;
		return_intm_solution(cluster_id);
		//<<"H"<<endl;
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
	//<<"OUT_OUT"<<endl;
}

void swap_sequence(int v1, int v2, bool& megaerror, double &temperature, int &type_move, int cluster_id){

	std::random_device rd;
	std::mt19937 g(rd());

	if (new_insertions_v.size() > 0)
		new_insertions_v.clear();

	if (new_insertions_p.size() > 0)
		new_insertions_p.clear();

	if (passengers_at_vehicle[v1].size()>0)
		passengers_at_vehicle[v1].clear();

	if (passengers_at_vehicle[v2].size()>0)
		passengers_at_vehicle[v2].clear();

	bool all_accepted_relocate_trip = false;
	int counter1 = 0;
	int counter2 = 0;
	int deltaURT;

	deltaURT = 0;
	//oldy_urt = total_user_ride_time;

	int prev_URT = compute_cluster_URT(cluster_id);

	
	//totalcomputedDELTA = 0;
	//<<"B"<<endl;
	save_intm_solution(cluster_id);
	//vector<int> edited_vehicles;

	int best_v = v2;
	/*cout<<"STARTING EMPTYING HEEEERE "<<current_time<<" "<<v1<<endl;
	
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

	/*cout<<"passengers to be removed: "<<endl;
	for (int i = 0; i < passengers_at_vehicle[v].size(); i++){
		cout<<passengers_at_vehicle[v][i]<<" ";
	}
	cout<<endl;
	cout<<"number stops "<<number_stops[v]<<endl;*/

	//bool megaerror = false;

	//randomly decide the start point (sequence 1)
	std::uniform_int_distribution<int> dist1s(1, number_stops[v1]-1);
	int starting_point_sequence1 = dist1s(g);
	//randomly decide the length (sequence 1)
	std::uniform_int_distribution<int> dist1l(1, number_stops[v1]-starting_point_sequence1);
	int length_sequence1 = dist1l(g);

	int p1;
	for (int i=0; i<length_sequence1;i++) {
		for (int j=0; j<number_passengers_action[v1][starting_point_sequence1+i];j++) {
			p1 = action_passengers[v1][starting_point_sequence1+i][j];
			//cout<<"hier"<<p1<<endl;
			if (removed_passenger[p1][v1] == 0) {

				//remove_passenger_from_vehicle(v1, p1);
				passengers_at_vehicle[v1].push_back(p1);
				//update URT
				//update_URT(vehicle_assigned[p1]);
				removed_passenger[p1][v1] = 1;
			}
		}
	}

	//cout<<"testing seg fault1"<<endl;

	for (int i=0;i<passengers_at_vehicle[v1].size();i++){
		p1 = passengers_at_vehicle[v1][i];
		remove_passenger_from_vehicle(v1, p1);
		update_URT(v1);
	}
	best_v = v1;
	/*cout<<"AFTER REMOVAL SEQUENCE HEEEERE "<<endl;
	cout<<starting_point_sequence1<<" "<<length_sequence1<<endl;
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
	//cout<<"testing seg fault2"<<endl;


	//randomly decide the start point (sequence 2)
	std::uniform_int_distribution<int> dist2s(1, number_stops[v2]-1);
	//cout<<number_stops[v2]-1<<endl;
	int starting_point_sequence2 = dist2s(g);
	//randomly decide the length (sequence 2)
	std::uniform_int_distribution<int> dist2l(1, number_stops[v2]-starting_point_sequence2);
	//cout<<number_stops[v2]-starting_point_sequence2<<endl;
	int length_sequence2 = dist2l(g);
	//cout<<"testing seg fault3"<<endl;
	int p2;
	for (int i=0; i<length_sequence2;i++) {
		for (int j=0; j<number_passengers_action[v2][starting_point_sequence2+i];j++) {
			p2 = action_passengers[v2][starting_point_sequence2+i][j];
			if (removed_passenger[p2][v2] == 0) {
				//remove_passenger_from_vehicle(v2, p2);
				passengers_at_vehicle[v2].push_back(p2);
				//update URT
				//update_URT(vehicle_assigned[p2]);
				removed_passenger[p2][v2] = 1;
			}
		}
	}
	//cout<<"testing seg fault4"<<endl;
	//cout<<starting_point_sequence2<<" "<<length_sequence2<<endl;
	for (int i=0;i<passengers_at_vehicle[v2].size();i++){
		
		p2 = passengers_at_vehicle[v2][i];
		//cout<<p2<<endl;
		remove_passenger_from_vehicle(v2, p2);
		//cout<<"pos rem"<<endl;
		update_URT(v2);
		//cout<<"pos urt"<<endl;
	}
	//cout<<"testing seg fault5"<<endl;
	best_v = v2;
	/*cout<<"AFTER REMOVAL SEQUENCE HEEEERE "<<endl;
	
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

	see_if_arrival_departure_dont_match(v1);
	update_URT(v1);
	if (free_capacity[v1].size() == 2)
		stay_times_consistency_empty_vehicle(v1);

	see_if_arrival_departure_dont_match(v2);
	update_URT(v2);
	if (free_capacity[v2].size() == 2)
		stay_times_consistency_empty_vehicle(v2);

	
	//randomly shuffle passengers in the vector to insert them randomly
	std::shuffle(passengers_at_vehicle[v1].begin(), passengers_at_vehicle[v1].end(), g);
	//randomly shuffle passengers in the vector to insert them randomly
	std::shuffle(passengers_at_vehicle[v2].begin(), passengers_at_vehicle[v2].end(), g);
	

	//cout<<"counter pv2: "<<counter<<" "<<passengers_at_vehicle[v].size()<<endl;
	megaerror = false;
	counter1 = 0;
	relocate_all_passengers_vehicle_swap(v1, init_temperature, type_move, counter1, deltaURT, megaerror, cluster_id, v2);
	//cout<<"counter pv1: "<<counter1<<" "<<passengers_at_vehicle[v1].size()<<endl;
	counter2 = 0;
	deltaURT = 0;
	relocate_all_passengers_vehicle_swap(v2, init_temperature, type_move, counter2, deltaURT, megaerror, cluster_id, v1);	
	//cout<<"counter pv1: "<<counter2<<" "<<passengers_at_vehicle[v2].size()<<endl;
	//UPDATE USER RIDE TIMES TO SEE IF NOW I CAN COMPUTE DELTA CORRECTLY??????
	
	if (megaerror){
		cout<<"mega error?"<<endl;
		return;
	}
	bool accept_relocate_trip;
	//cout<<"counter pv3: "<<counter<<" "<<passengers_at_vehicle[v].size()<<endl;
	if ((counter1 == passengers_at_vehicle[v1].size()) && (counter2 == passengers_at_vehicle[v2].size())) {
		
		int new_URT = compute_cluster_URT(cluster_id);
		//<<"oldyxnew: "<<oldy_urt<<" "<<total_user_ride_time<<" "<<(oldy_urt-total_user_ride_time)<<endl; 
		//if ((oldy_urt-total_user_ride_time) > 0){
		if ((prev_URT-new_URT) > 0){
			//<<"REMOVEALLPOSITIVE"<<endl;
			type_move = 1;
			cout<<"counter pv1: "<<counter1<<" "<<passengers_at_vehicle[v1].size()<<endl;
			cout<<"counter pv2: "<<counter2<<" "<<passengers_at_vehicle[v2].size()<<endl;
		} else {
			int x = (double)rand() / (double)RAND_MAX;
			//int delta = oldy_urt-total_user_ride_time;
			int delta = prev_URT-new_URT;

			if (x < exp(delta / (temperature))) {
				//non improving move is accepted according to SA
				accept_relocate_trip = true;
			} else {
				accept_relocate_trip = false;
			}
			//<<"F0"<<endl;
			if (accept_relocate_trip) {
				//<<"REMOVEALLACCEPT"<<endl;
				type_move = 2;
			} else {
				type_move = 3;
				//<<"back to beginning"<<endl;
				return_intm_solution(cluster_id);
			}
		}
		//<<"F"<<endl;
	} else {
		type_move = 3;
		//<<"revert changes2"<<endl;
		//<<"G"<<endl;
		return_intm_solution(cluster_id);
		//<<"H"<<endl;
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
	//<<"OUT_OUT"<<endl;

	if (passengers_at_vehicle[v1].size()>0)
		passengers_at_vehicle[v1].clear();

	if (passengers_at_vehicle[v2].size()>0)
		passengers_at_vehicle[v2].clear();

	for (int i=0;i<total_requests;i++){
		for (int j=0;j<maxvehicles;j++){
			//blocked_vehicles[i][j] = 0;
			removed_passenger[i][j] = 0;
		}
	}
}

void relocate_passenger(int p, double &temperature, int &type_move, int cluster_id){

	//<<"before relocated passenger: "<<p<<endl;

	bool accept_relocate_trip = false;
	//compute the removal of passenger p from the solution
	int v = vehicle_assigned[p];
	int decrease_solution = 0;
	int count=0;
	int greatest_ed = INT_MIN;
	int old_arrival_time;


	/*cout<<vehicle_assigned[p]<<endl;
	cout<<"removing passenger y "<<p<<endl;
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

	blocked_vehicles[p][vehicle_assigned[p]] = 1;

	int addedAtV;
	//<<"A"<<endl;
	//bool priority_empty_vehicle = true;

	re_insertion(p, accept_relocate_trip, temperature, type_move, cluster_id, addedAtV, v);
	/*if (!accept_relocate_trip) {
		priority_empty_vehicle = false;
		re_insertion(p, accept_relocate_trip, temperature, type_move, cluster_id, addedAtV, v);
	}*/
	//<<"B"<<endl;
	int begin, end;
	begin = 0;
	end = 0;
	int total_faking_error = 0;
	if (accept_relocate_trip) {
		//cout<<"accepted relocated / priority empty vehicle "<<priority_empty_vehicle<<endl;
		//cout<<"removeee here SA"<<endl;
		//printf("remove heeeere SA\n");
		//<<"number stops "<<number_stops[v]<<endl;
		//it means the passenger was relocated to another trip 
		//<<"C"<<endl;
		for (int i=0; i<number_stops[v]; i++){
			for (int j=0; j<number_passengers_action[v][i];j++) {
				//<<"actp "<<action_passengers[v][i][j]<<endl;
				if (action_passengers[v][i][j] == p) {
					count++;
				
					//<<"SAhier1"<<endl;
					//removes previous origin stuff
					if (number_passengers_action[v][i] == 1) {
						number_stops[v]--;

						//<<"SAhier2"<<i<<endl;
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

						//action_passengers[v][i].erase(action_passengers[v][i].begin() + j);
						action_passengers[v][i][j] = action_passengers[v][i][number_passengers_action[v][i]-1];
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
												//<<"faking removing error"<<endl;
												//<<current_user_ride_time<<" "<<user_ride_time_temp[save_p]<<endl;
												//<<arrival_time_stop[v][k]<<" "<<departure_time_stop[v][i]<<endl;
												total_faking_error += user_ride_time_temp[save_p] - current_user_ride_time;
											}
										

											if (current_user_ride_time != user_ride_time[save_p]) {
												difference = current_user_ride_time -  user_ride_time[save_p];
												//<<"difference: "<<difference<<endl;
												//total_user_ride_time += difference;
												//total_difference += difference;
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
				//if ((oldy_urt-total_user_ride_time) != computedDELTA){
					//<<"COMPUTING ERRROR"<<endl;
					//if (total_faking_error != 0)
						//<<"total faking error: "<<total_faking_error<<endl;
				//}
				break;
			}	
		}
		//<<"D"<<endl;
		see_if_arrival_departure_dont_match(v);
		update_URT(v);
		//<<"E"<<endl;

	} else {
		vehicle_assigned[p] = v;
	}

	for (int l=begin;l<end;l++) {
		free_capacity[v][l]++;
	}
	//<<"F"<<endl;
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
	//<<"G"<<endl;


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
								//<<"delta computing error"<<endl;
								//<<save_p<<" "<<current_user_ride_time<<" "<<user_ride_time_temp[save_p]<<endl;
								//<<arrival_time_stop[v][k]<<" "<<departure_time_stop[v][i]<<endl;
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
	//<<"H"<<endl;
}

void update_direct_travel_times() {
    // For each passenger, compute the minimum travel time between their available stops
    for (int p = 0; p < total_requests; p++) {
        int min_travel_time = INT_MAX;
        
        // Iterate over all combinations of pickup and delivery stops
        for (int i = 0; i < number_stops_origin[p]; i++) {
            for (int j = 0; j < number_stops_destination[p]; j++) {
                int pickup_stop = stops_origin[p][i];
                int delivery_stop = stops_destination[p][j];
                
                // Compute travel time from this pickup to this delivery
                int travel_time_p_d = travel_time[pickup_stop][delivery_stop];
                
                // Track the minimum travel time across all stop combinations
                if (travel_time_p_d < min_travel_time) {
                    min_travel_time = travel_time_p_d;
                }
            }
        }
        
        // Store the minimum direct travel time for this passenger
        direct_travel_time[p] = min_travel_time;
    }
}

void compute_waiting_times() {
	
	//compute waiting times for each passenger
	for (int p=0; p<total_requests; p++) {
		int waiting_time_p = 0;

		int v = vehicle_assigned[p];
        if (v != -1) {
            // find first stop index on vehicle v where passenger p appears (pickup)
            int pickup_idx = -1;
            for (int i = 0; i <= number_stops[v]; i++) {
                for (int j = 0; j < number_passengers_action[v][i]; j++) {
                    if (action_passengers[v][i][j] == p) {
                        pickup_idx = i;
                        break;
                    }
                }
                if (pickup_idx != -1) break;
            }

            if (pickup_idx != -1) {
                int pickup_time = departure_time_stop[v][pickup_idx];
                waiting_time_p = pickup_time - earliest_departure[p];
                if (waiting_time_p < 0) waiting_time_p = 0;
            }
        }

        // store per-passenger waiting time (assumes waiting_time[] exists globally)
        waiting_time[p] = waiting_time_p;
		average_waiting_time += waiting_time_p;
		

	}
}

void check_valid_user_ride_times() {
	extra_travel_time = 0;
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
							if ((current_user_ride_time != user_ride_time[save_p]) or (current_user_ride_time == 0)) {
								
								extra_travel_time += current_user_ride_time - direct_travel_time[save_p];
								//difference = current_user_ride_time -  user_ride_time[save_p];
								average_travel_time_ratio += (double)current_user_ride_time/direct_travel_time[save_p];
								//user_ride_time[save_p] = current_user_ride_time;
								cout<<"WRONG COMPUTED RIDE TIME!!!! passengerX "<<save_p<<" "<<current_user_ride_time<<" "<<user_ride_time[save_p]<<endl;
								cout<<"ERRORR"<<endl;
								print_v_vehicle(v);
								l = number_passengers_action[v][k]+1;
								//k = number_stops[v]+2; //leave loop
								leave_loop = true;
								//<<"leave"<<endl;
							} else {
								extra_travel_time += current_user_ride_time - direct_travel_time[save_p];
								average_travel_time_ratio += (double)current_user_ride_time/direct_travel_time[save_p];
							}
						}
					}
					if (leave_loop)
						k = number_stops[v]+2;
				}	
			} 
		}
	}

	for (int i=0; i<total_requests; i++){

		if (assigned_to_3rd_party[i] == 1){
			new_total_user_ride_time += user_ride_time[i];
		}
	}

	//<<"URT: "<<new_total_user_ride_time<<" "<<total_user_ride_time<<endl;
}

void reassign_vehicles_to_another_depot(){

	for (int i = 0; i < number_depots; i++) {
		number_empty_vehicles_at_depot[i] = 0;
	}

	for (int v = 0; v<total_number_vehicles;v++) {

		if (free_capacity[v].size() == 2)
			number_empty_vehicles_at_depot[vehicle_located_at_depot[v]]++;

	}

	/*int depot_with_less_vehicles_empty;
	int smaller_number_vehicles_empty = INT_MAX;

	for (int i = 0; i < number_depots; i++) {
		if (number_empty_vehicles_at_depot[i] < smaller_number_vehicles_empty) {
			depot_with_less_vehicles_empty = i;
			smaller_number_vehicles_empty = number_empty_vehicles_at_depot[i];
		}
	}*/

	for (int i = 0; i < number_depots; i++) {

		//depot[i];//the stop
		if (number_empty_vehicles_at_depot[i] < 5) {

			//get more vehicles here bc it will run out
			int closest_depot;
			int closest_distance = INT_MAX;
			//find closest depot with more vehicles
			for (int j = 0; j < number_depots; j++) {
				if ((travel_time[depot[i]][depot[j]] < closest_distance) && (number_empty_vehicles_at_depot[j] > 10)) {
					//balance empty vehicles 

					int difference_v = number_empty_vehicles_at_depot[j] - number_empty_vehicles_at_depot[i];
					int move_v = difference_v/2; //number of vehicles that will be moved
					int count_number_moved = 0;
					for (int v = 0; v<total_number_vehicles;v++) {


						if ((vehicle_located_at_depot[v] == j) && (free_capacity[v].size() == 2)) {

							count_number_moved++;

							stops[v][0] = depot[i];
							stops[v][1] = depot[i];

							vehicle_located_at_depot[v] = i;

							arrival_time_stop[v][0] = current_time+travel_time[depot[j]][depot[i]];
							departure_time_stop[v][0] = arrival_time_stop[v][0]+1;

						}

						if (count_number_moved == move_v) {
							v = total_number_vehicles+1;
						}

					}
				}
			}

		}

	}


	//just to check
	for (int v = 0; v<total_number_vehicles;v++) {

		if (free_capacity[v].size() == 2)
			number_empty_vehicles_at_depot[vehicle_located_at_depot[v]]++;

	}

	
	
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

void compute_passengers_per_kilometers(){

	passengers_per_kilometer = 0;
	int counted_vehicles = 0;
	int number_passengers_vehicle = 0;\
	double kilometers_run;
	for (int v=0; v<total_number_vehicles; v++){

		int total_ride_time_vehicle = arrival_time_stop[v][number_stops[v]] - departure_time_stop[v][0];
		kilometers_run += (double)(total_ride_time_vehicle/180); //travel speeds considered are 20kmh. this may change depending on data

		
		int counter = 0;
		for (int i=1; i<number_stops[v]; i++){
			counter += number_passengers_action[v][i];
		}

		number_passengers_vehicle += counter/2;

	}

	//now compute the average of passengers per kilometer run
	passengers_per_kilometer = (double)(number_passengers_vehicle/kilometers_run);
}

void compute_deadheading_times(){

	total_deadheading_times = 0;
	total_per_vehicle_travel_time = 0;
	total_shared_times = 0;

	for (int v=0; v<total_number_vehicles; v++){


		total_deadheading_times += arrival_time_stop[v][1] - departure_time_stop[v][0]; //first stop the vehicle always runs empty
		int vtype = vehicle_type[v];
		
		total_per_vehicle_travel_time += arrival_time_stop[v][1] - departure_time_stop[v][0];

		for (int i=1; i<number_stops[v]; i++){
			
			total_per_vehicle_travel_time += arrival_time_stop[v][i+1] - departure_time_stop[v][i];

			if (free_capacity[v][i] == maxcapacity[vtype]) {
				//this means the vehicle is gonna run empty to the next stop
				total_deadheading_times += arrival_time_stop[v][i+1] - departure_time_stop[v][i];
			}

			if (maxcapacity[vtype] - free_capacity[v][i] >= 2) {
				//this means the vehicle is running with at least two passengers on board
				total_shared_times += arrival_time_stop[v][i+1] - departure_time_stop[v][i];
			}


		}

	}
}

void check_last_position_route() {

 	//check if last position on route passed the time
 	//and make vehicle wait so new requests can be added afterwords
 	for (int v=0; v<total_number_vehicles; v++){

 		//<<"v: "<<v<<endl;
 		if (number_stops[v] > 2) {

 			if (free_capacity[v][number_stops[v]-1] == free_capacity[v][0]) {
 				if (departure_time_stop[v][number_stops[v]-1] < current_time) {
 					//arrival_time_stop[v][number_stops[v]] = arrival_time_stop[v][number_stops[v]-1] + travel_time[stops[v][number_stops[v]-1]][stops[v][number_stops[v]]]+600;
					departure_time_stop[v][number_stops[v]-1] = current_time+10;
				}

 			} else {
		 		if (departure_time_stop[v][number_stops[v]-1] < current_time) {
		 			//means that already passed
		 			//<<"passed!!!"; 
		 			int prev_stop = stops[v][number_stops[v]-1];
		 			int pos_origin = number_stops[v];
		 			stops[v].insert(stops[v].begin() + pos_origin, prev_stop);
						
					//update passenger performing actions on the stops
					action_passengers[v].insert(action_passengers[v].begin() + pos_origin, vector<int>());
					action_passengers[v][pos_origin].resize(15);
					action_passengers[v][pos_origin][0] = -1;

					//action_passengers[v][pos_origin].insert(action_passengers[v][pos_origin].begin(), -1);
					number_passengers_action[v].insert(number_passengers_action[v].begin() + pos_origin, 0);

					int prev_time = departure_time_stop[v][number_stops[v]-1];
					arrival_time_stop[v].insert(arrival_time_stop[v].begin() + pos_origin, prev_time);
					departure_time_stop[v].insert(departure_time_stop[v].begin() + pos_origin, current_time+1);
					slack_time[v].insert(slack_time[v].begin() + pos_origin, 86400);

					int prv_capacity = free_capacity[v][pos_origin-1];
					free_capacity[v].insert(free_capacity[v].begin() + pos_origin, prv_capacity);
					
					

					number_stops[v]++;
					//updating the arrival time at depot + some waiting time of 10 min
					arrival_time_stop[v][number_stops[v]] = arrival_time_stop[v][number_stops[v]-1] + travel_time[stops[v][number_stops[v]-1]][stops[v][number_stops[v]]]+600;
					departure_time_stop[v][number_stops[v]] = arrival_time_stop[v][number_stops[v]] + 1;
				

					/*for (int i=0; i<=number_stops[v];i++) {
					cout<<stops[v][i]<<" ("<<number_passengers_action[v][i]<<") "<<" [";
						for (int j=0; j<number_passengers_action[v][i];j++) 
							cout<<action_passengers[v][i][j]<<" ";
						cout<<"]  ";

						cout<<"{"<<arrival_time_stop[v][i]<<"} ";
						cout<<"{"<<departure_time_stop[v][i]<<"} ";
						cout<<"|"<<slack_time[v][i]<<"|  ";
						cout<<"*"<<free_capacity[v][i]<<"*"<<endl;
					}
					cout<<endl<<endl;*/
		 		}
	 		}
 		}

 	}
}

// Add before simulated annealing/main SA loops:
void validate_solution_state(int cluster_id) {
    cout << "\n=== VALIDATING CLUSTER " << cluster_id << " ===" << endl;
    
    // Check 1: Every passenger in cluster has valid vehicle
    /*for (int p = 0; p < total_requests; p++) {
        if (vehicle_assigned[p] != -1) {
            int v = vehicle_assigned[p];
            bool v_in_cluster = false;
            for (int cv : clusters[cluster_id]) {
                if (cv == v) {
                    v_in_cluster = true;
                    break;
                }
            }
            if (!v_in_cluster) {
                cerr << "WARNING: Passenger " << p << " assigned to vehicle " << v 
                     << " NOT in cluster " << cluster_id << endl;
            }
        }
    }*/

	// Global vehicle assignment sanity check
	for (int p = 0; p < total_requests; ++p) {
		int v = vehicle_assigned[p];
		if (v == -1) continue;
		if (v < 0 || v >= total_number_vehicles) {
			cerr << "ERROR: passenger " << p << " has invalid vehicle " << v << endl;
		}
	}
    
    // Check 2: Vehicle data structure sizes match
    for (int v : clusters[cluster_id]) {
        if (number_stops[v] > 0) {
            int expected_size = number_stops[v] + 1;
            if (stops[v].size() != expected_size || 
                arrival_time_stop[v].size() != expected_size ||
                free_capacity[v].size() != expected_size) {
                cerr << "ERROR: Vehicle " << v << " data mismatch! stops:" 
                     << stops[v].size() << " arrival:" << arrival_time_stop[v].size() 
                     << " capacity:" << free_capacity[v].size() << " (expected:" << expected_size << ")" << endl;
            }
        }
    }
}

void simulated_annealing(int n_allocated, int cluster_id) {

	double temperature, elapsed;
	int delta, relocate_p, nrep, no_improvement, type_move, count;
	//type_move 1 - accepeted and improving
	//type_move 2 - accepted and non improving
	//type_move 3 - rejected or not feasible move
	std::random_device rd;
	std::mt19937 g(rd());

	//<<"hier8"<<endl;
	temperature = init_temperature;
	no_improvement = 0; 
	count = 0;
	double start_time2 = get_wall_time();
	start_time = std::clock();
	auto start_time3 = std::chrono::high_resolution_clock::now();
	double elapsed3;
	double prev_elapsed;
	int count_static_it = 0;
	
	vector<int> vehicles_still_depot;

	//cout<<"cid: "<<cluster_id<<" "<<clusters[cluster_id].size()<<endl;
	//<<"X"<<endl;
	save_best_solution(cluster_id);
	//<<"Y"<<endl;
	//<<"hier9"<<endl;

	//int veih_cluster = 
	std::vector<int> passengers_in_cluster;

	if (clusters[cluster_id].size() > 1) {
		for (int i=0;i<clusters[cluster_id].size();i++){
			int v = clusters[cluster_id][i];
			//<<v<<" ";
			for (int j=0;j<n_allocated;j++){

				int pid = sort_passengers[j].k;
				if (vehicle_assigned[pid] == v) {
					passengers_in_cluster.push_back(pid);
				}

			}
		}
	}
	//<<"hier10"<<endl;
	//<<endl;
	//<<"size pic: "<<passengers_in_cluster.size()<<endl;

	while(true){

		nrep = 0;
		do {


			double y = (double)rand() / (double)RAND_MAX;
			swap_vehicle = false;

			//randomly decide the neighborhood
			//std::uniform_int_distribution<int> neigh1(1, 2);
			//int neighborhood_chosen = neigh1(g);
			//cout<<neighborhood_chosen<<endl;
			int neighborhood_chosen = 1;
			
			//RELOCATE
			if (neighborhood_chosen == 1) {
				count_static_it++;
				//SWITCH
				//cout<<"PIC: "<<passengers_in_cluster.size()<<endl;
				if (passengers_in_cluster.size() > 0) {
					relocate_p = passengers_in_cluster[rand() % passengers_in_cluster.size()];
					//<<"rel p: "<<relocate_p<<endl;
					if (vehicle_assigned[relocate_p] != -1) {
						if (passengers_departure_time_from_home[relocate_p] >= current_time) {
							//<<"relocate passenger SA: "<<relocate_p<<endl;
							//<<"hier11"<<endl;
							type_move = 3;
							priority_empty_vehicle = true;
							relocate_passenger(relocate_p, temperature, type_move, cluster_id);
							if (type_move == 3) {
								count_static_it++;
								priority_empty_vehicle = false;
								relocate_passenger(relocate_p, temperature, type_move, cluster_id);
							}
							//<<"hier12"<<endl;
							if (relocate_p < 0){
								//<<"MEGAERRORRRR1"<<endl;
								return;
							}
							// <<"vehicle_assigned SA: "<<vehicle_assigned[relocate_p]<<endl;
						}
					}
				} else {
					return;
				}

				/*cout<<"AFTER_SIMPLE RELOCATE"<<endl;
				for (int kk=0;kk<total_requests;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}
				for (int vv=0;vv<total_number_vehicles;vv++){
					times_validation(vv);
				}*/
			}

			//DESTROY REPAIR
			if (neighborhood_chosen == 3) {
				count_static_it++;
				//REDUCE
				
				//the way vehicles are added to vehicles_still_depot needs to be changeds
				select_vehicles_havent_that_can_be_turned_empty(vehicles_still_depot, cluster_id);
				//<<"A"<<endl;
				//<<"hieerx";
				//<<"size vehicles at depot "<<vehicles_still_depot.size()<<endl;
				//for (int i=0; i<vehicles_still_depot.size();i++) {

				bool megaerror = false;
				int v;
				if (vehicles_still_depot.size() > 0) {
					int b = rand() % vehicles_still_depot.size();
					v = vehicles_still_depot[b];

					if (passengers_at_vehicle[v].size() > 0) {
						empty_vehicle(v, megaerror, temperature, type_move, cluster_id);
					}
				}
				//<<"heere1"<<endl;
				
				//<<"no idea1"<<endl;
				//<<"heere2"<<endl;
				if (megaerror){
					//<<"MEGAERRORRRR2"<<endl;
					//return 0;
				}
					//relocate_all_passengers_vehicle(v, init_temperature, type_move);
				//}

				/*cout<<"AFTER_SIMPLE REDUCE"<<endl;
				for (int kk=0;kk<total_requests;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}
				for (int vv=0;vv<total_number_vehicles;vv++){
					times_validation(vv);
				}*/
				
			}

			//SWAP HERE
			if (neighborhood_chosen == 2) {
				count_static_it++;
				//SWAP
				
				swap_vehicle = true;

				bool megaerror = false;
				int v1, v2;
				v1 = -1;
				v2 = -1;
				//if (vehicles_still_depot.size() > 0) {
					while (v1 == v2) {
						int b1 = rand() % clusters[cluster_id].size();
						v1 = clusters[cluster_id][b1];

						int b2 = rand() % clusters[cluster_id].size();
						v2 = clusters[cluster_id][b2];

						if ((v1 != v2) && (free_capacity[v1].size() > 2) && (free_capacity[v2].size() > 2)) {
							swap_sequence(v1, v2, megaerror, temperature, type_move, cluster_id);
						} else {
							v1 = v2;
						}
					}

					
				//}
				//<<"heere1"<<endl;
				
				//<<"no idea1"<<endl;
				//<<"heere2"<<endl;
				if (megaerror){
					//<<"MEGAERRORRRR2"<<endl;
					//return 0;
				}
					//relocate_all_passengers_vehicle(v, init_temperature, type_move);
				//}

				/*cout<<"AFTER_SIMPLE SWAP"<<endl;
				for (int kk=0;kk<total_requests;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}
				for (int vv=0;vv<total_number_vehicles;vv++){
					times_validation(vv);
				}*/
				
			}

			

			if (type_move == 1) {
				no_improvement = 0;
			}

			if (type_move == 2) {
				no_improvement++;
			}
			//<<"hier13"<<endl;
			//<<"no idea2"<<endl;
			int curr_cluster_URT = compute_cluster_URT(cluster_id);
			//<<"no idea3"<<endl;
			//<<cluster_id<<best_tot_cluster_ride_time.size()<<endl;
			//<<"hier14"<<endl;
			if (curr_cluster_URT < best_tot_cluster_ride_time[cluster_id]) {
				//<<"BEEST FOUND SO FAR"<<endl;
				//<<"no idea4"<<endl;
				save_best_solution(cluster_id);
				//<<"no idea5"<<endl;
				/*for (int kk=0;kk<total_requests;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}*/
				//<<"heerex"<<endl;
			}
			//<<"no idea6"<<endl;
			//<<"hier15"<<endl;
			
			if (++count > 25) {
				double elapsed2;
				elapsed2 = get_wall_time() - start_time2;

				elapsed = (double)(std::clock() - start_time)/(double)(CLOCKS_PER_SEC);

				auto end_time3 = std::chrono::high_resolution_clock::now();
				elapsed3 = std::chrono::duration<double>(end_time3 - start_time3).count();
				
				
				
				//<<"ELAPSED TIME "<<elapsed<<endl;
				
				if (elapsed3 - prev_elapsed > 3) {
					cout<<"ELAPSEED "<<elapsed<<" "<<elapsed2<<" "<<elapsed3<<endl;
				}
				//change this static / dyanmic
				if (count_static_it > 200) {
					return;
				}
				/*if (elapsed3 > comp_time) {
					return;
				}*/
				prev_elapsed = elapsed3;
				count = 0;
			}

			nrep++;
		} while (nrep < maxnrep);

		maxnrep += increase_rep;
		temperature = init_temperature + lambda * log(1 + no_improvement);


	}

	if (passengers_in_cluster.size() > 0)
		passengers_in_cluster.clear();
}

int recompute_distance_costs(int old_cluster, int new_cluster, int sum_all_distances){

	int new_sum_all_distance = sum_all_distances;
	int loc_centroid = stops[new_cluster][current_position[new_cluster]];
	int dist;

	for (int j=0; j<total_number_vehicles;j++){

		int loc_vehicle = stops[j][current_position[j]];
		

		int new_min_dist = 999999;
		if (cluster[j] == old_cluster) {
			//recompute a new cluster
			new_min_dist = mindDist[j];
			for (int i=0; i<centroids.size();i++){

				int clusterId = centroids[i];
				int loc_centroid = stops[centroids[i]][current_position[centroids[i]]];

				//int loc_vehicle = stops[j][current_position[j]];
				dist = travel_time[loc_centroid][loc_vehicle];

				if (dist < new_min_dist) {
					new_min_dist = dist;
					//cluster[j] = clusterId;
				}

			}

			//compute the difference between oldmindist and newmindist
			int difference = new_min_dist - mindDist[j];
			new_sum_all_distance += difference;


		} else {

			dist = travel_time[loc_centroid][loc_vehicle];

			new_min_dist = mindDist[j];
			if (dist < mindDist[j]) {
				new_min_dist = dist;
			}

			int difference = new_min_dist - mindDist[j];
			new_sum_all_distance += difference;

		}

	}

	return new_sum_all_distance;
}

void update_distance_costs(int old_cluster, int new_cluster){

	//int new_sum_all_distance = sum_all_distances;
	int loc_centroid = stops[new_cluster][current_position[new_cluster]];
	int dist;

	for (int j=0; j<total_number_vehicles;j++){

		int loc_vehicle = stops[j][current_position[j]];
		

		if (cluster[j] == old_cluster) {
			//recompute a new cluster
			int new_min_dist = 999999;
			for (int i=0; i<centroids.size();i++){

				int clusterId = centroids[i];
				int loc_centroid = stops[centroids[i]][current_position[centroids[i]]];

				//int loc_vehicle = stops[j][current_position[j]];
				dist = travel_time[loc_centroid][loc_vehicle];

				if (dist < mindDist[j]) {
					mindDist[j] = dist;
					cluster[j] = clusterId;
				}

			}

			//compute the difference between oldmindist and newmindist
			//int difference = new_min_dist - mindDist[j];
			//new_sum_all_distance += difference;


		} else {

			dist = travel_time[loc_centroid][loc_vehicle];

			if (dist < mindDist[j]) {
				mindDist[j] = dist;
				cluster[j] = new_cluster;
			}

			//int difference = new_min_dist - mindDist[j];
			//new_sum_all_distance += difference;

		}

	}
}

void update_current_position() {

	//update which positions on the route the vehicles are
	int k;
	for (int j=0; j<total_number_vehicles;j++){

		for (k=0;k<=number_stops[j];k++) {

			if (arrival_time_stop[j][k] > current_time) {
				break;
			}

		}

		current_position[j] = k-1;

	}
}

void randomly_assign_clusters() {

	for (int j=0; j<total_number_vehicles;j++){
		cluster[j] = -1;
	}

	std::vector<int> groupNumbers;
        
    // Generate unique random numbers within the specified range
    for (int i = 0; i < total_number_vehicles; i++) {
        groupNumbers.push_back(i);
    }

    //unsigned xseedx = 0;

    // Shuffle the vector to randomize the order
    std::shuffle(groupNumbers.begin(), groupNumbers.end(), default_random_engine(current_time));

    int subVectorSize = (int)(total_number_vehicles / number_clusters);
    
    // Initialize iterators for the original vector
    int begin = 0;
    int end = subVectorSize;
    //<<"subsize: "<<subVectorSize<<endl;

    // Split the vector into x sub-vectors
    for (int i = 0; i < number_clusters; i++) {
        // Handle the last sub-vector which may be smaller
        if (i == number_clusters - 1) {
            end = groupNumbers.size();
        }

        // Create a sub-vector and add it to the result
        //clusters[i].push_back(std::vector<int>(begin, end));
        for (int itx=begin;itx<end;itx++){
        	clusters[i].push_back(groupNumbers[itx]);
        }

        // Move iterators to the next sub-vector
        begin = end;
        end += subVectorSize;
    }

    for (int i = 0; i < number_clusters; i++) {
    	for (int j=0; j<clusters[i].size();j++){
    		cluster[clusters[i][j]] = i;
    	}
    }


}

void k_medoids(int k, int epochs){

	bool improvement = true;
	int iterations = 0;

	int dist;
	for (int j=0; j<total_number_vehicles;j++){
		mindDist[j] = 999999;
		cluster[j] = -1;
	}

	for (int i=0; i<centroids.size();i++){
		mindDist[centroids[i]] = 0;
		cluster[centroids[i]] = centroids[i];
	}

	//assign each vehicle to its nearest centroids
	for (int i=0; i<centroids.size();i++){

		int clusterId = centroids[i];
		int loc_centroid = stops[centroids[i]][current_position[centroids[i]]];
		
		for (int j=0; j<total_number_vehicles;j++){

			int loc_vehicle = stops[j][current_position[j]];
			//<<"comp: "<<stops[centroids[i]][current_position[centroids[i]]]<<" "<<stops[j][current_position[j]]<<endl;
			dist = travel_time[loc_centroid][loc_vehicle];
			//<<"dist:"<<dist<<endl;
			if (dist < mindDist[j]) {
				mindDist[j] = dist;
				cluster[j] = clusterId;
			} else {
				if (dist == mindDist[j]) {
					double y = (double)rand() / (double)RAND_MAX;
					if (y <= 0.5) {
						mindDist[j] = dist;
						cluster[j] = clusterId;
					}
				}
			}

		}

	}


	while ((improvement) && (iterations < epochs)) {
		iterations++;
		//update centroid
		int sum_all_distances = 0;
		for (int j=0; j<total_number_vehicles;j++){
			sum_all_distances += mindDist[j];
		}

		//<<"dist:"<<sum_all_distances<<endl;
		int new_sum_all_distances;
		int best_sum_all_distances = sum_all_distances;
		int new_best_cluster = -1;

		int remove_cluster = -1;
		for (int i=0; i<centroids.size();i++){
			int clusterId = centroids[i];

			for (int j=0; j<total_number_vehicles;j++){
				if ((cluster[j] == clusterId) && (clusterId != j)) {

					//perform swap and recompute distance costs
					new_sum_all_distances = recompute_distance_costs(clusterId, j, sum_all_distances);
					//<<new_sum_all_distances<<endl;
					if (new_sum_all_distances < best_sum_all_distances) {
						best_sum_all_distances = new_sum_all_distances;
						new_best_cluster = j;
						remove_cluster = i;
					}

				}
			}
		}

		if (new_best_cluster != -1) {
			int old_cluster = centroids[remove_cluster];
			centroids[remove_cluster] = new_best_cluster;
			update_distance_costs(old_cluster, new_best_cluster);
		} else {
			improvement = false;
		}

	}
}

void build_clusters(){

	for (int i=0;i<number_clusters;i++){

		centroids_keys[centroids[i]] = i;
		best_tot_cluster_ride_time[i] = 0;
	}

	int id_centroid;
	for (int i=0; i<total_number_vehicles;i++){

		//id_centroid = centroids_keys[cluster[i]];
		// cluster[i] must be a centroid id, otherwise centroids_keys[...] is wrong
        if (cluster[i] < 0) {
            std::cerr << "ERROR: cluster[" << i << "] is " << cluster[i] << " (unassigned)\n";
            std::abort();
        }
        auto it = centroids_keys.find(cluster[i]);
        if (it == centroids_keys.end()) {
            std::cerr << "ERROR: cluster[" << i << "]=" << cluster[i]
                      << " not found in centroids_keys (bad clustering)\n";
            std::abort();
        }

        id_centroid = it->second;
        //CHECK_INDEX(id_centroid, number_clusters, "id_centroid");
		clusters[id_centroid].push_back(i);

	}

	// DEBUG: Print clusters and check for overlaps
    /*std::set<int> all_vehicles_seen;
    for (int c = 0; c < number_clusters; c++) {
        cout << "Cluster " << c << ": ";
        for (int v : clusters[c]) {
            cout << v << " ";
            // Check if vehicle already appeared in another cluster
            if (all_vehicles_seen.count(v) > 0) {
                cerr << "ERROR: Vehicle " << v << " appears in multiple clusters!" << endl;
            }
            all_vehicles_seen.insert(v);
        }
        cout << endl;
    }
    cout << "Total vehicles in clusters: " << all_vehicles_seen.size() << " (expected: " << total_number_vehicles << ")" << endl;
	if (all_vehicles_seen.size() != total_number_vehicles) {
        cerr << "ERROR: Not all vehicles assigned to clusters or vehicles assigned multiple times!" << endl;
    }*/
}

void compute_mean_distances_request_partitions(int p){

	//CHECK_INDEX(p, total_requests, "p in compute_mean_distances_request_partitions");

	//<<"p:"<<p<<endl;
	int expected_position, k;
	int sum_dist_clusters;
	for (int i=0;i<number_clusters;i++){
		//<<"i: "<<i<<endl;
		sort_clusters[p][i].mean_dist = 0;
		sort_clusters[p][i].idx_cluster = i;
		sum_dist_clusters = 0;
		for (int j=0;j<clusters[i].size();j++){ //for each vehicle
			
			int v = clusters[i][j];
			//<<"v: "<<v<<" "<<total_number_vehicles<<endl;
			//compute the expected position of vehicle at the earliest_departure time of passenger
			for (k=0;k<=number_stops[v];k++){
				//<<"k: "<<k<<endl;
				if (arrival_time_stop[v][k] > earliest_departure[p]) {
					break;
				} 
			}

			expected_position = k-1;
			int stop_expected_position = stops[v][expected_position];

			int avg_tt_origin = 0;
			for (int x=0;x<number_stops_origin[p];x++){
				avg_tt_origin += travel_time[stops_origin[p][x]][stop_expected_position];
			}

			avg_tt_origin = (int)avg_tt_origin/number_stops_origin[p];
			sum_dist_clusters += avg_tt_origin;



		}

		//<<"cluster size: "<<clusters[i].size()<<endl;
		sort_clusters[p][i].mean_dist = (int)sum_dist_clusters/clusters[i].size();

	}

	//sort the indexes
	sort(sort_clusters[p], sort_clusters[p]+number_clusters, comparator2);

	/*for (int i=0; i<number_clusters; i++){
		cout<<sort_clusters[p][i].idx_cluster<<" "<<sort_clusters[p][i].mean_dist<<endl;
	}
	cout<<endl;*/
}

void check_vehicle_occupancy(){

	//int avg_used_capacity;

	//we check for every stop the vehicle made, the used capacity of the vehicle, and then we take an average
	int used_capacity;
	for (int v = 0; v<total_number_vehicles;v++) {
		avg_occupancy[v] = 0;
		running_max_capacity[v] = 0;
		for (int i=1; i<number_stops[v];i++) {
			used_capacity = free_capacity[v][0] - free_capacity[v][i];
			avg_occupancy[v] += used_capacity;
			if (free_capacity[v][i] == 0) {
				running_max_capacity[v]++;
			}
		}
		if (number_stops[v] > 2)
			avg_occupancy[v] = avg_occupancy[v]/(number_stops[v]-2);
		else 
			avg_occupancy[v] = 0;
	}

	overall_occupancy = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		overall_occupancy += avg_occupancy[v];
	}

	overall_occupancy = (double)overall_occupancy/(double)total_number_vehicles;


	//ocupancy per type of vehicle

	overall_occupancy4 = 0;
	int count4 = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v][0] == 4) {
			count4++;
			overall_occupancy4 += avg_occupancy[v];
		}
	}
	if (count4 > 0)
		overall_occupancy4 = (double)overall_occupancy4/(double)count4;
	else
		overall_occupancy4 = 0;

	overall_occupancy8 = 0;
	int count8 = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v][0] == 8) {
			count8++;
			overall_occupancy8 += avg_occupancy[v];
		}
	}
	if (count8 > 0)
		overall_occupancy8 = (double)overall_occupancy8/(double)count8;
	else
		overall_occupancy8 = 0;


	overall_occupancy12 = 0;
	int count12 = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v][0] == 12) {
			count12++;
			overall_occupancy12 += avg_occupancy[v];
		}
	}
	if (count12 > 0)
		overall_occupancy12 = (double)overall_occupancy12/(double)count12;
	else
		overall_occupancy12 = 0;


	//running with maximum capacity
	overall_max_capacity = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		overall_max_capacity += running_max_capacity[v];
	}


	overall_max_capacity4 = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v][0] == 4) {
			overall_max_capacity4 += running_max_capacity[v];
		}

	}

	overall_max_capacity8 = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v][0] == 8) {
			overall_max_capacity8 += running_max_capacity[v];
		}

	}


	overall_max_capacity12 = 0;
	for (int v = 0; v<total_number_vehicles;v++) {
		if (free_capacity[v][0] == 12) {
			overall_max_capacity12 += running_max_capacity[v];
		}

	}

}

void update_arrival_time_depot() {

	for (int v=0; v<total_number_vehicles;v++){
		arrival_time_stop[v][number_stops[v]] = departure_time_stop[v][number_stops[v]-1]+travel_time[stops[v][number_stops[v]-1]][stops[v][number_stops[v]]];
		departure_time_stop[v][number_stops[v]] = arrival_time_stop[v][number_stops[v]];
	}

}

int main(int argc, char **argv) {

	string output_filename;
	string requests_filename;
	comp_time = 0.25;
	//start_time = std::clock();
	//cout<<re
	begin_time = get_wall_time();
	total_number_vehicles = 0;
	average_response_time_new_requests = 0.0;
	average_waiting_time = 0;
	for (int i=1; i<argc; i++)
  	{
		if (strcmp(argv[i], "--filename_requests") == 0) {
			//input_requests_festival(argv[i+1]);
			input_requests_commuting(argv[i+1]);
			//input_requests(argv[i+1]); //concert & nightlife
			requests_filename = argv[i+1];
			cout<<"x"<<total_requests<<" ";
		} else if (strcmp(argv[i], "--filename_travel_time") == 0) {
			//<<"HIER"<<endl;
			input_travel_time(argv[i+1]);
			/*for(int i=0;i<5828;i++) {
			for (int j=0;j<5828;j++) {
				if (travel_time[i][j] == 0) {
					cout<<i<<" "<<j<<endl;
		  			// << travel_time[i][j] <<',';
		  		}
		  	}
		  }*/
			cout<<"leave ttm"<<endl;
		   // <<endl;
		} else if (strcmp(argv[i], "--filename_uneven") == 0) {
			//cout<<"ttm"<<endl;
			input_zones_uneven_demand(argv[i+1]);
			//cout<<"leave ttm"<<endl;
		} else if (strcmp(argv[i], "--output_file") == 0) {
			//<<"ttm"<<endl;
			//input_travel_time(argv[i+1]);
			output_filename = argv[i+1];
			
		} else if (strcmp(argv[i], "--depot") == 0) {
			for (int j = 0; j < number_depots; j++) {
				i++;
				depot[j] = stoi(argv[i]);
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
      		//total_requests = atoi(argv[i+1]);
    		i++;
    	} else if (strcmp(argv[i], "--seed") == 0) {
      		seed = atoi(argv[i+1]);
    		i++;
    	} else if (strcmp(argv[i], "--number_vehicles1") == 0) {
			//for (int j = 0; j < number_type_vehicles; j++) {
			i++;
			float perc = (float)stof(argv[i])/100;
			number_vehicles[0] = ceil(perc*total_requests);
			total_number_vehicles += number_vehicles[0];
			//}
		} else if (strcmp(argv[i], "--number_vehicles2") == 0) {
			//for (int j = 0; j < number_type_vehicles; j++) {
			i++;
			float perc = (float)stof(argv[i])/100;
			number_vehicles[1] = ceil(perc*total_requests);
			total_number_vehicles += number_vehicles[1];
			//}
		} else if (strcmp(argv[i], "--number_vehicles3") == 0) {
			//for (int j = 0; j < number_type_vehicles; j++) {
			i++;
			float perc = (float)stof(argv[i])/100;
			number_vehicles[2] = ceil(perc*total_requests);
			total_number_vehicles += number_vehicles[2];
			//}
		}

	}
	cout << "Number of threads available: " << omp_get_max_threads() << endl;
	//<<"x1"<<total_requests<<" ";
	/*cout<<"AFTER"<<endl;
	for(int i=0;i<5828;i++) {
	for (int j=0;j<5828;j++) {
		if (travel_time[i][j] == 0) {
			cout<<i<<" "<<j<<endl;
  			// << travel_time[i][j] <<',';
  		}
  	}
  }
   cout <<endl;*/
	srand(seed);
	//<<total_requests<<" "<<seed<<" ";

	/*if (total_requests == 300) {
		number_vehicles[0] = 50;
		number_vehicles[1] = 50;
		number_vehicles[2] = 50;
		total_number_vehicles = 150;
	}

	if (total_requests == 600) {
		number_vehicles[0] = 100;
		number_vehicles[1] = 100;
		number_vehicles[2] = 100;
		total_number_vehicles = 300;
	}

	if (total_requests == 900) {
		number_vehicles[0] = 150;
		number_vehicles[1] = 150;
		number_vehicles[2] = 150;
		total_number_vehicles = 450;
	}*/
	

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

	request_time_stamp.resize(total_requests + 1);
	request_timestamp_captured.resize(total_requests + 1, false);

   	//<<"x2 "<<total_requests<<" ";
   	
	current_time = time_stamp[0]; //time stamp of the first passenger is the current time on the system
	int current_passenger;
	//empty_vehicle = 0;


	for (int i=0; i<total_requests; i++){
		//route_assigned[i] = 0;
		vehicle_assigned[i] = -1;
		delay[i] = 0;
		assigned_to_3rd_party[i] = 0;
		att_inser[i] = 0;
		//already_opened_vehicle_for_it[i] = 0;
	}

	//<<"x3 "<<total_requests<<" ";

	int k = 0;

	int next_depot = 0;

	//initialize route of vehicles at the depot
	//<<"staart "<<number_type_vehicles<<" "<<endl;
	
	//remove this
	//total_requests = 14;
	//total_number_vehicles = 6;
	/*for (int i=0; i<total_requests; i++){
		if (number_stops_origin[i] > 3)
			number_stops_origin[i] = 3;
		if (number_stops_destination[i] > 3)
			number_stops_destination[i] = 3;
	}*/

	for (int j=0; j<number_type_vehicles; j++) {
		next_depot = 0;
		for (int i=0; i<number_vehicles[j];i++) {
			//<<number_vehicles[j]<<endl;
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

			
			//assign "randomly" a depot to the vehicle
			//vehicle_located_at_depot[k] = rand() % number_depots; 

			vehicle_located_at_depot[k] = next_depot;
			next_depot++;
			if (next_depot == number_depots) {
				next_depot = 0;
			}

			mindDist[k] = 999999;
			cluster[k] = -1;

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

	//<<"x4 "<<total_requests<<" ";

	for (int i=0;i<total_requests;i++){
		for (int j=0;j<maxvehicles;j++){
			blocked_vehicles[i][j] = 0;
			removed_passenger[i][j] = 0;
		}
	}
	//<<"x5 "<<total_requests<<endl;
	//total_capacity[number_vehicles] = maxcapacity;
	//max_capacity[number_vehicles] = maxcapacity;

	int min_v_at_depot = INT_MAX;
	for (int d = 0; d<number_depots; d++) {
		if (number_vehicles_at_depot[d] < min_v_at_depot){
			min_v_at_depot = number_vehicles_at_depot[d];
		}
	}

	int num_iterations_to_start_reassign = min_v_at_depot - 5;

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

	/*for (int v = 0; v < total_number_vehicles; v++) {
		for (int s = 0; s < maxpassengers; s++) {
			blocked_positions[v][s] = 0;
		}
	}*/
	
	total_user_ride_time = 0;

	max_flex_delay = 0;

	

	//int number_clusters = 4;



	/*if ((total_requests >= 600) && (total_requests <= 2100)) {
		for (int k=0;k<total_requests;k++){
			latest_arrival[k] = latest_arrival[k] + 3600;
		}
	}*/

	served_requests_so_far = 0;
	rejected_requests_so_far = 0;

	//reduce size instance (static exp with model)
	/*total_requests = 3;
	total_number_vehicles = 3;
	for (int i =0; i < total_requests; i++){
		number_stops_origin[i] = 2;
		number_stops_destination[i] = 2;
	}*/
	//comment this later
	
	int it = 0;
	//initialize clusters
	//try to get one from each depot. then comes back from first depot and repeats...
	for (int i=0; i<number_clusters;i++){

		int next_depot = i % number_depots;	
		
		while ((vehicle_located_at_depot[it]) != next_depot) {
			it++;
			if (it >= total_number_vehicles)
				it = 0;

		}
		bool already_centroid = false;
		//i need also to verify if the vehicle is not already a "centroid"
		for (int j=0;j<centroids.size();j++){
			if (centroids[j] == it)
				already_centroid = true;
		}

		if (not already_centroid)
			centroids.push_back(it);
		else
			i--;
	}



	
	/*for (int i=0; i<centroids.size(); i++){
		cout<<centroids[i]<<" "<<vehicle_located_at_depot[centroids[i]]<<endl;
	}*/

	int epochs = 1000;
	//randomly assign a new position to each vehicle
	//this is for concert instances -> they all start at stadium
	/*for (int i = 0; i < total_number_vehicles; i++){
		//for (int j=0;j<number_stops_origin[p];j++) {
		int j = rand() % number_stops_origin[0];
		int s_origin = stops_origin[0][j];
		stops[i][0] = s_origin; 
		//}
	}*/

	//stops[centroids[3]][0] = rand() % 5588; 	
	//<<"hier1"<<endl;
	k_medoids(number_clusters, epochs);

	/*cout<<"new centroids "<<endl;
	for (int i=0; i<centroids.size(); i++){
		cout<<centroids[i]<<" "<<vehicle_located_at_depot[centroids[i]]<<endl;
	}*/

	build_clusters();
	//<<"hier2"<<endl;
	/*for (int i=0;i<number_clusters;i++){

		for (int j=0;j<clusters[i].size();j++){
			cout<<clusters[i][j]<<" ";
		}
		cout<<endl;
	}*/

	/*for(int i=0;i<5828;i++) {
	for (int j=0;j<5828;j++) {
		if (travel_time[i][j] == 0) {
			cout<<i<<" "<<j<<endl;
  			// << travel_time[i][j] <<',';
  		}
  	}
  }
   cout <<endl;*/

	int test_total_distance = 0;
	for (int j=0; j<total_number_vehicles; j++) {

		int loc_centroid = stops[cluster[j]][current_position[cluster[j]]];
		int loc_vehicle = stops[j][current_position[j]];
		test_total_distance += travel_time[loc_centroid][loc_vehicle];
	}
	cout<<"total dist: "<<test_total_distance<<endl;
	for (int p=0; p<min(10,total_requests); p++) {
    	cout<<"  p="<<p<<" #origin="<<number_stops_origin[p]<<" #dest="<<number_stops_destination[p]<<endl;
	}
	vector<int> passengers_to_be_inserted;
	vector<int> passengers_to_be_insertedOLD;

	
	//<<"hier"<<endl;
	//change the current time part to a parameter given value
	//MAIN ALGORITHM

	//std::vector<int> avl_cluster;
	//std::vector<int> del_passenger;
	int del_passenger[number_clusters+1];
	int avl_cluster[number_clusters+1];
	for (int c=0;c<number_clusters;c++) {
		//avl_cluster.push_back(0);
		avl_cluster[c] = 0;
		//del_passenger.push_back(0);
		del_passenger[c] = 0;
	}


	for (int p = 0; p < total_requests; p++) {
		for (int j=0;j<number_stops_origin[p];j++) {
			int s_origin = stops_origin[p][j];
			for (int k=0;k<number_stops_destination[p];k++) {
				int s_destination = stops_destination[p][k];

				if (s_origin == s_destination) {
					cout<<"error "<<endl;
					cout<<p<<" "<<s_origin<<" "<<s_destination<<endl;
				}

			}

		}
	}

	for (int p = 0; p < total_requests; p++) {
		sort_passengers[p].k = p;
		sort_passengers[p].time_stamp = time_stamp[p];
	}

	sort(sort_passengers, sort_passengers+total_requests, comparator3);

	// initialize response_time_request
	response_time_request.resize(total_requests);
	k = 0;
	//current_time = sort_passengers[k].time_stamp;
	current_time = 0; //static

	//current_time = 0;

	//<<"hier3"<<endl;'
	//41400
	//k = total_requests+1;
	//<<k<<" "<<total_requests<<" "<<current_time<<endl;
	//clock_t start_algorithm_time;
	//start_algorithm_time = std::clock();
	auto start_algorithm_time = std::chrono::high_resolution_clock::now();
	auto start_time_chrono = std::chrono::high_resolution_clock::now();
	double p_elapsed_algo_time, difference_elapsed, difference_updated, p_updated_cluster;
	p_elapsed_algo_time = 0;
	p_updated_cluster = current_time;
	int it_updated_cluster = 0;
	int difference_updateds;
	//64800 - festival
	//commuting - 32400
	//nightlife - 88400
	//nightlife2 - 22600 
	//concert 2 - 70400
	//commuting2 - 70400
	//concert - 86600

	int algo_iterations = 0;

	//uncomment static
	while(k < total_requests) { 
		passengers_to_be_inserted.push_back(sort_passengers[k].k);
		//cout<<"p and ts: "<<sort_passengers[k].k<<" "<<sort_passengers[k].time_stamp<<endl;
		//cout<<"k: "<<k<<endl;
		k++;
	}
	//uncomment static

	cout<<requests_filename<<endl;
	
	listP min_travel_time;
	for (int r = 0; r < total_requests; r++){
		
		min_travel_time[r] = INT_MAX;
		if (number_stops_origin[r] < 1 || number_stops_destination[r] < 1) {
			cerr<<"ERROR: request "<<r<<" has empty stops list: "
				<<number_stops_origin[r]<<" "<<number_stops_destination[r]<<endl;
			abort();
		}
		//cout<<"#O #D "<<number_stops_origin[r]<<" "<<number_stops_destination[r]<<endl;
		for (int i = 0; i < number_stops_destination[r]; i++) {
			for (int j = 0; j < number_stops_origin[r]; j++) {
				//cout<<stops_origin[r][j]<<" "<<stops_destination[r][i]<<endl;
				//cout<<travel_time[stops_origin[r][j]][stops_destination[r][i]]<<endl;
				if (travel_time[stops_origin[r][j]][stops_destination[r][i]] < min_travel_time[r]) {
					if (travel_time[stops_origin[r][j]][stops_destination[r][i]] <= 0) {
						travel_time[stops_origin[r][j]][stops_destination[r][i]] = 1;
					}
					min_travel_time[r] = travel_time[stops_origin[r][j]][stops_destination[r][i]];
				}
			}
		}

		//cout<<"min_travel_time "<<r<<" "<<min_travel_time[r]<<endl;

	}

	//ensure feasibility
	for (int k=0;k<total_requests;k++){
		latest_arrival[k] = latest_arrival[k] + 3600;
	}

	//while((k < total_requests) or (current_time < 32400)) {
	while(algo_iterations < 90) { //static
		algo_iterations++;

	//while(current_time < 28800) {
		
		check_last_position_route();
		//<<"hier3.1"<<endl;
		//if (k > num_iterations_to_start_reassign)
		//	reassign_vehicles_to_another_depot();
		
		//if ((current_time >= sort_passengers[k].time_stamp)) {
		if (algo_iterations < 1000) { //static

			// DEBUG: Track per-cluster passenger assignments
			/*for (int c = 0; c < number_clusters; c++) {
				int passengers_in_c = 0;
				for (int p = 0; p < total_requests; p++) {
					if (vehicle_assigned[p] != -1) {
						int v = vehicle_assigned[p];
						// Find which cluster v belongs to
						bool found = false;
						for (int cv : clusters[c]) {
							if (cv == v) {
								passengers_in_c++;
								found = true;
								break;
							}
						}
					}
				}
				cout << "Cluster " << c << ": " << passengers_in_c << " passengers assigned" << endl;
			}*/
			//comment static
			
			/*if (passengers_to_be_inserted.size() > 0) {
				passengers_to_be_inserted.clear();
			}

			//cout<<current_time<<" "<<sort_passengers[k].time_stamp<<" "<<k<<" "<<total_requests<<endl;
			while((current_time >= sort_passengers[k].time_stamp) && (k < total_requests)) { 
				passengers_to_be_inserted.push_back(sort_passengers[k].k);
				k++;
			}*/
			//coment static

			for (int ol=0;ol<passengers_to_be_insertedOLD.size();ol++){
				passengers_to_be_inserted.push_back(passengers_to_be_insertedOLD[ol]);
			}

			if (passengers_to_be_insertedOLD.size() > 0) 
				passengers_to_be_insertedOLD.clear();


		
			int num_threads_for = passengers_to_be_inserted.size();
			if (num_threads_for > number_clusters) {
				num_threads_for = number_clusters;
			}

			

			#pragma omp parallel for num_threads(num_threads_for)
			for (int itx = 0; itx<num_threads_for; itx++) {
				int nxt_p = passengers_to_be_inserted[itx];
				compute_mean_distances_request_partitions(nxt_p);
			}
			

			//maybe sort passengers_to_be_inserted in a way to avoid collision between clusters???

			//<<"hier4"<<endl;
			for (int c=0;c<number_clusters;c++) {
				avl_cluster[c] = c;
			}
			int reinsertions_passenger_old = 0;
			
			/*int num_threads_compute_partitions = passengers_to_be_inserted.size();
			#pragma omp parallel for num_threads(num_threads_compute_partitions)
			for (int px = 0; px<num_threads_compute_partitions; px++) {
				if (px < passengers_to_be_inserted.size()) {
					int nxt_p = passengers_to_be_inserted[px];
					compute_mean_distances_request_partitions(nxt_p);
				}
			}*/

			//cout<<"passengers to be inserted: "<<passengers_to_be_inserted.size()<<endl;
			while (passengers_to_be_inserted.size() > 0) {

				// Optional: detect vector corruption early
				// (size() itself is safe, but corrupted heap can explode anywhere)
				/*if (passengers_to_be_inserted.size() > (size_t)total_requests) {
					std::cerr << "CORRUPTION? passengers_to_be_inserted.size()="
							<< passengers_to_be_inserted.size()
							<< " > total_requests=" << total_requests << std::endl;
					std::abort();
				}*/

				//for (int itx = 0; itx<num_threads_for; itx++) {

					//each passengers will be inserted in one processor
					//start parallelized for	
					#pragma omp parallel for num_threads(num_threads_for)
					for (int px = 0; px<num_threads_for; px++) {

						if (px < passengers_to_be_inserted.size()) {

							// IMPORTANT: whatever code sets nxt_p must be above.
							// Immediately validate nxt_p before using it:
							
							//<<"12size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
							int nxt_p = passengers_to_be_inserted[px];
							//CHECK_INDEX(nxt_p, total_requests, "nxt_p (passenger id)");
							//cout<<"nxt p: "<<nxt_p<<"p: "<<px<<"x"<<"size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
							// Capture timestamp only on first entry for this passenger
							// timestamp capture still must be protected because vector<bool)
							#pragma omp critical(ts_capture)
							{
								if (!request_timestamp_captured[nxt_p]) {
									request_time_stamp[nxt_p] = std::chrono::high_resolution_clock::now();
									request_timestamp_captured[nxt_p] = true;
								}

							}
							//<<"nxp: "<<nxt_p<<endl;
							it_cl_inser[nxt_p] = 0;
							//<<"13size: "<<passengers_to_be_inserted.size()<<" "<<nxt_p<<"ends"<<endl;
							
							compute_mean_distances_request_partitions(nxt_p);
							
							bool continue_this_passenger = true;
							bool accept_infeasible_insertion = false;
							while (continue_this_passenger) {
								//<<"hier5"<<endl;
								//#pragma omp parallel for num_threads(num_threads_for)
								//cout<<"0nxt p: "<<nxt_p<<"p: "<<px<<"x"<<"size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
								//cout<<"cluster av: "<<avl_cluster[px]<<endl;
								
								//cout<<"bf_inser1 "<<avl_cluster[px]<<endl;
								bool entered_here = false;
								bool entered_there = false;
								int best_cluster = sort_clusters[nxt_p][0].idx_cluster;
            
								if (best_cluster < 0 || best_cluster >= number_clusters) {
									cerr << "ERROR: Invalid cluster assignment " << best_cluster << " for passenger " << nxt_p << endl;
								}
								if (avl_cluster[px] == sort_clusters[nxt_p][0].idx_cluster){
									entered_here = true;
									cheapest_insertion_randomized_parallel(nxt_p, accept_infeasible_insertion, avl_cluster[px]);
								} else {
									if (number_clusters > 1) {
										if (avl_cluster[px] == sort_clusters[nxt_p][1].idx_cluster){
											entered_here = true;
											cheapest_insertion_randomized_parallel(nxt_p, accept_infeasible_insertion, avl_cluster[px]);
										} else {
											double y = (double)rand() / (double)RAND_MAX;
											if (y <= 0.5) {
												entered_there = true;
												//entered_here = true;
												cheapest_insertion_randomized_parallel(nxt_p, accept_infeasible_insertion, avl_cluster[px]);
											}
										}
									}
								}

								//cheapest_insertion_randomized_parallel_NEW(nxt_p, accept_infeasible_insertion, avl_cluster[px]);
								
								//cout<<"af_inser1 "<<avl_cluster[px]<<endl;

								//<<"hier5.9"<<endl;
								if (vehicle_assigned[nxt_p] == -1) {
									//if (not (entered_here))
									//	cheapest_insertion_randomized_parallel(nxt_p, accept_infeasible_insertion, avl_cluster[px]);
									//cout<<"1nxt p: "<<nxt_p<<"p: "<<px<<"x"<<"size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
									it_cl_inser[nxt_p]++;
								} else {
									//cout<<"2nxt p: "<<nxt_p<<"p: "<<px<<"x"<<"size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
									continue_this_passenger = false;
									//<<px<<" "<<del_passenger.size()<<endl;
									del_passenger[px] = 1;
									served_requests_so_far++;
									response_time_request[nxt_p] = std::chrono::high_resolution_clock::now();
									auto response_time_duration = std::chrono::duration<double>(response_time_request[nxt_p] - request_time_stamp[nxt_p]);
									double response_time_in_seconds = response_time_duration.count();
									if (response_time_in_seconds < 0) response_time_in_seconds = 0;
									average_response_time_new_requests += response_time_in_seconds;
								}
								//<<"hier6"<<endl;
								//cout<<"3nxt p: "<<nxt_p<<"p: "<<px<<"x"<<"size: "<<passengers_to_be_inserted.size()<<" cid: "<<sort_clusters[nxt_p][it_cl_inser[nxt_p]].idx_cluster<<endl;
								//<<"continue: "<<vehicle_assigned[nxt_p]<<endl;
								if (continue_this_passenger) {
									//if (it_cl_inser[nxt_p] >= number_clusters) {
										att_inser[nxt_p]++;
										
										//if (att_inser[nxt_p] >= 2){
											if (vehicle_assigned[nxt_p] == -1) {
												
												int response_time = current_time - time_stamp[nxt_p];
												
												if (att_inser[nxt_p] < 100) { //static
												//if (response_time < 500) {
													#pragma omp critical(ptbi_old_push)
													{
														passengers_to_be_insertedOLD.push_back(nxt_p);
													}
												}
												
											}
											
											continue_this_passenger = false;
											del_passenger[px] = 1;
										//}
									//}
								}
								//cout<<"4nxt p: "<<nxt_p<<"p: "<<px<<"x"<<"size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
								
							}

						}
					}
					//end parallelized for
					
					for (int c=0;c<number_clusters;c++) {
						avl_cluster[c] = avl_cluster[c]+1;
						avl_cluster[c] = avl_cluster[c]%number_clusters;
						//<<avl_cluster[c]<<" ";
					}
					//<<"hier6.9"<<endl;
					//<<endl;

					for (int c=num_threads_for-1; c>=0;c--) {
						if (del_passenger[c] == 1) {
							//<<"size ptbi: "<<passengers_to_be_inserted.size()<<endl;
							//<<"del p: "<<del_passenger[c]<<endl;
							att_inser[passengers_to_be_inserted[c]] = 0;
							passengers_to_be_inserted.erase(passengers_to_be_inserted.begin() + c);
							//<<c<<endl;
							//<<"5size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
							del_passenger[c] = 0;
								
						}
					}

					if (reinsertions_passenger_old < 2) {
						reinsertions_passenger_old++;
						if (passengers_to_be_insertedOLD.size() > 0) {
							for (int ol=0;ol<passengers_to_be_insertedOLD.size();ol++){
								passengers_to_be_inserted.push_back(passengers_to_be_insertedOLD[ol]);
							}
							passengers_to_be_insertedOLD.clear();
						}
					}
					

					//<<"hier6.93"<<endl;
					//<<"6size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
					num_threads_for = passengers_to_be_inserted.size();
					if (num_threads_for > number_clusters) {
						//<<"7size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
						num_threads_for = number_clusters;
						//<<"8size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
					}
					//<<"9size: "<<passengers_to_be_inserted.size()<<"ends"<<endl;
					//<<"hier6.95"<<endl;

					
				//}

				//shuffle passengers to add some variety
				if (passengers_to_be_inserted.size() > 1)
					std::shuffle(passengers_to_be_inserted.begin(), passengers_to_be_inserted.end(), default_random_engine(current_time));
			}

				
				/*cout<<"AFTER CONSTRUCTIVE"<<endl;
				for (int kk=0;kk<k;kk++){
					if (vehicle_assigned[kk] != -1) {
						solution_validation(kk, vehicle_assigned[kk]);
					//served_passengers++;
					}
				}
				//<<"hieer0"<<endl;
				for (int vv=0;vv<total_number_vehicles;vv++){
					times_validation(vv);
				}*/
				
			
		} 

		//l_elapsed_algo_time = (double)(std::clock() - start_algorithm_time)/(double)(CLOCKS_PER_SEC);

		difference_updated = current_time - p_updated_cluster;
		difference_updateds = algo_iterations - it_updated_cluster; //static

		//comment this static. no need to change partitions
		//if (difference_updated > 50) {
		/*if (number_clusters > 1) {
			if (difference_updateds > 50) {
				//p_updated_cluster = current_time;

				centroids.clear();
				for (int ix=0; ix<number_clusters;ix++){
					clusters[ix].clear();
				}

				double y = (double)rand() / (double)RAND_MAX;

				//y = 0 - static
				if (y <= 0.0) {

					//decide new centroids
					
					for (int ix=0; ix<number_clusters;ix++){

						int cent = rand() % total_number_vehicles; 
						bool already_centroid = false;
						//i need also to verify if the vehicle is not already a "centroid"
						for (int jx=0;jx<centroids.size();jx++){
							if (centroids[jx] == cent)
								already_centroid = true;
						}

						if (not already_centroid)
							centroids.push_back(cent);
						else
							ix--;
					}

					int epochs = 1000;
					
					k_medoids(number_clusters, epochs);

					build_clusters();

				} else {
					randomly_assign_clusters();

				}		
			}
		}*/
		//decide new centroids
		//comment this static. no need to change partitions

		
		//cout<<"hier7sa"<<endl;
		if (k > total_number_vehicles + 1) {
		
			
			//omp_set_num_threads(2);	
			#pragma omp parallel for num_threads(number_clusters)
			for (int c=0; c<number_clusters; c++) {
				//<<k<<endl;
				//<<"passenger: p"<<k<<endl;
				//cout<<"cluster c"<<c<<endl;
				// Before each simulated_annealing call
				//for (int c = 0; c < number_clusters; c++) {
				//validate_solution_state(c);
				//}
				simulated_annealing(k, c);
				//check_valid_user_ride_times();
				//<<"passenger: p"<<k<<endl;
				//cout<<"cluster c2"<<c<<endl;
				return_best_solution(c);
				//<<"out heere"<<endl;

			}
		}
		

		

		//comment this static case
		
		auto this_time = std::chrono::high_resolution_clock::now();
		//l_elapsed_algo_time = (double)(std::clock() - start_algorithm_time)/(double)(CLOCKS_PER_SEC);
		double l_elapsed_algo_time = std::chrono::duration<double>(this_time - start_algorithm_time).count();
		
		//cout<<"ALGO elapsed: "<<l_elapsed_algo_time<<endl;
		difference_elapsed = l_elapsed_algo_time - p_elapsed_algo_time;
		
		difference_elapsed = difference_elapsed*2;
		/*if (difference_elapsed < 1)
			current_time += 1;
		else
			current_time += difference_elapsed;*/

		p_elapsed_algo_time = l_elapsed_algo_time;
		
		//update_current_position();
		
		//comment this static case


		
	}


	//cout<<"END"<<endl;
	update_arrival_time_depot();
	update_direct_travel_times();
	
	check_valid_user_ride_times();
	
	/*for (int i =0; i < total_requests; i++){
		for (int j =0; j < number_stops_origin[i]; j++){
			cout<<stops_origin[j]<<" ";
		}
		cout<<endl;
		for (int j =0; j < number_stops_destination[i]; j++){
			cout<<stops_destination[j]<<" ";
		}
		cout<<endl;
		cout<<endl;
	}*/

	print_all_vehicles();


	for (int k=0;k<total_requests;k++){
		if (vehicle_assigned[k] != -1) {
			solution_validation(k, vehicle_assigned[k]);
			served_passengers++;
		}
	}

	served_passengers_3party = total_requests - served_passengers;

	compute_passengers_per_kilometers();

	compute_deadheading_times();
	compute_waiting_times();

	cout<<"check occu"<<endl;
	check_vehicle_occupancy();

	cout<<served_passengers<<endl;
	double average_extra_travel_time = (double)extra_travel_time/served_passengers;
	average_travel_time_ratio = (double)average_travel_time_ratio/served_passengers;
	
	total_served_passengers = served_passengers + served_passengers_3party;

	double avg_per_vehicle_travel_time = (double)total_per_vehicle_travel_time/total_number_vehicles;

	average_response_time_new_requests = (double)average_response_time_new_requests/served_passengers;
	average_waiting_time = (double)average_waiting_time/served_passengers;

	for (int c=0;c<number_clusters;c++){
		total_user_ride_time += best_tot_cluster_ride_time[c];
	}

	best_total_user_ride_time=0;
	for (int p=0;p<total_requests;p++){
		best_total_user_ride_time += user_ride_time[p];
	}

	double average_user_ride_time = (double)best_total_user_ride_time/(double)served_passengers;
	
	//<<total_user_ride_time<<" "<<best_total_user_ride_time<<endl;
	//<<served_passengers<<"  "<<total_user_ride_time<<endl;
	//<<total_user_ride_time<<endl;
	std::ofstream output_file;
	output_file.open(output_filename, std::ios::app);
	output_file << requests_filename << " " << served_passengers << " " << served_passengers_3party << " " << total_served_passengers << " " << passengers_per_kilometer << " " << average_extra_travel_time << " " << average_travel_time_ratio << " " << total_deadheading_times << " " << total_shared_times << " " << total_user_ride_time << " " << best_total_user_ride_time << " avgurt: " << average_user_ride_time << " " << avg_per_vehicle_travel_time << " " << seed << " " << total_number_vehicles << " " << overall_occupancy << " " << overall_occupancy4 << " " << overall_occupancy8 << " " << overall_occupancy12 << " " << overall_max_capacity << " " << overall_max_capacity4 << " " << overall_max_capacity8 << " " << overall_max_capacity12 << " " << average_waiting_time << " " << average_response_time_new_requests;
	output_file<<" ";
	//previously already commented
	/*cout << "served passengers ODB " << served_passengers << endl;
	cout << "served passengers 3rd party: " << served_passengers_3party << endl;*/
	/*cout << "served passengers total: " << total_served_passengers << endl;
	cout << "solution cost (total user ride time): "<<total_user_ride_time<<endl;
	cout << "BEST solution cost (total user ride time): "<<best_total_user_ride_time<<endl<<endl;*/
	//compute_idle_times();

	//elapsed = get_wall_time() - begin_time;
	auto end_time_chrono = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::seconds>(end_time_chrono - start_time_chrono);

	auto this_time2 = std::chrono::high_resolution_clock::now();
	//l_elapsed_algo_time = (double)(std::clock() - start_algorithm_time)/(double)(CLOCKS_PER_SEC);
	
	elapsedf = std::chrono::duration<double>(this_time2 - start_algorithm_time).count();	 
	output_file << elapsedf << " " << duration.count() << endl;
}
