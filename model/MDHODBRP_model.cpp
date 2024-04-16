//INCLUDE FILES AND GLOBAL VARIABLES

// hide unsafe warnings wrt deprecated input/output commands
#define _CRT_SECURE_NO_WARNINGS 1
#pragma warning(disable: 4996)

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
#include <iomanip>
#include "gurobi_c++.h"
#include <iterator>
using namespace std;

#define maxvehicles 100
#define maxpassengers 3000
#define maxstations 6000
#define maxtotalcapacity 40
#define maxtypevehicles 40
#define maxnumberdepots 10

typedef int listD[maxnumberdepots + 1];
typedef int listT[maxtypevehicles + 1];
typedef int listV[maxvehicles + 1];
typedef int matrixDV[maxnumberdepots + 1][maxvehicles + 1];
typedef int listS[maxstations + 1];
typedef int matrixSS[maxstations + 1][maxstations + 1];
typedef int listP[maxpassengers + 1];
typedef int matrixPS[maxpassengers + 1][maxstations + 1];
typedef int matrixVSS[maxvehicles + 1][maxstations + 1][maxstations + 1];
//using namespace std;

int ts_min, ts_max;
map<int, int> nodes;
listD depot;
listV all_depots;
map<int, int> type_node;
int number_nodes;
listT number_vehicles;
listD number_vehicles_at_depot;
int total_number_vehicles;
listT maxcapacity;
int number_type_vehicles;
int number_depots;
int number_nodes_depots;
listV vehicle_located_at_depot;
listV vehicle_return_to_depot;
matrixDV vehicles_at_depot;
listV vehicle_type;
int total_requests;

map<int, int> station_id_map;
listS stations_ids;
matrixSS travel_time;
string output_filename;
string requests_filename;

listP time_stamp, earliest_departure, latest_departure, earliest_arrival, latest_arrival, direct_travel_time;

matrixPS stops_origin, stops_destination;
matrixPS walking_time_stops_origin, walking_time_stops_destination;
listP number_stops_origin, number_stops_destination;
int number_stations;

listS q;
matrixVSS M, W;


string IntToString (int a)
{
    ostringstream temp;
    temp<<a;
    return temp.str();
}

void input_travel_time(char *filename) {

	fstream file (filename, ios::in);
	string line, data;
	int s, stop1, stop2;
	//<<"here ";
	if(file.is_open())
	{
		/*getline(file, line);
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
		}*/
		//<<endl;


		stop1 = 0;
		getline(file, line);

		while(getline(file, line))
		{

			stringstream str(line);
			stop2 = 0;
			getline(str, data, ',');
			//<<data<<endl;
			
			int count = 0;
			while(getline(str, data, ',')) {
				//cout<<data<<endl;
				if (count > 0) {
					travel_time[stop1][stop2] = stoi(data);
					travel_time[stop2][stop1] = stoi(data);
					stop2 = stop2 + 1;
				} 
				count++;
			}
			//cout<<stop1<<" "<<stop2<<endl;
			stop1 = stop1 + 1;

		}

		for (int i=0;i<stop1;i++){
			stations_ids[i] = i;
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
			//stops_origin[p][s] = stoi(data);
			stops_origin[p][s] = number_nodes;
			nodes[number_nodes] = stoi(data);
			type_node[number_nodes] = 1;
			number_nodes++;
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
					//stops_origin[p][s] = stoi(stop);
					stops_origin[p][s] = number_nodes;
					nodes[number_nodes] = stoi(data);
					type_node[number_nodes] = 1;
					number_nodes++;
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
			//stops_destination[p][s] = stoi(data);
			stops_destination[p][s] = number_nodes;
			nodes[number_nodes] = stoi(data);
			type_node[number_nodes] = 2;
			number_nodes++;
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
					//stops_destination[p][s] = stoi(stop);
					stops_destination[p][s] = number_nodes;
					nodes[number_nodes] = stoi(data);
					type_node[number_nodes] = 2;
					number_nodes++;
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

void input_requests_OLD(char *filename) {

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

void MDHODBRPFR_MODEL(){ 

	double elapsed3;
	
	int max_time = 86400;
	
	GRBEnv env = GRBEnv();

	std::string varName;

	try{

		GRBModel model = GRBModel(env);

		typedef std::vector<std::vector<GRBVar>> IntVarMatrix;
		typedef std::vector<std::vector<GRBVar>> NumVarMatrix;
		typedef std::vector<IntVarMatrix> IntVar3Matrix;

		//decision variables

		//if vehicle b travels from node i to j
		//IntVar3Matrix x(env, total_number_vehicles);
		IntVar3Matrix x(total_number_vehicles, IntVarMatrix(number_nodes, std::vector<GRBVar>(number_nodes)));
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {
				for (int j = 0; j < number_nodes; j++) {
					x[b][i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY);
					varName = "x("+IntToString(b) +","+IntToString(i)+","+IntToString(j)+")";
					x[b][i][j].set(GRB_StringAttr_VarName, varName);
	            	//x[b][i][j].setName(varName.c_str());
            	}
			}	
		}

		//load of vehicle b after serving node i
		//IntVarMatrix Q(env, total_number_vehicles);
		IntVarMatrix Q(total_number_vehicles, std::vector<GRBVar>(number_nodes));
		for (int n = 0; n < total_number_vehicles; n++) {
			for (int j = 0; j < number_nodes; j++) {
				Q[n][j] = model.addVar(0.0, maxtotalcapacity, 0.0, GRB_INTEGER);
			}
		}

		//start service by vehicle b at node n
		//IntVarMatrix T(env, total_number_vehicles);
		IntVarMatrix T(total_number_vehicles, std::vector<GRBVar>(number_nodes));
		for (int n = 0; n < total_number_vehicles; n++) {
			for (int j = 0; j < number_nodes; j++) {
				T[n][j] = model.addVar(0.0, max_time, 0.0, GRB_INTEGER);
			}
		}


		//(1) Objective function
		//minimize total user ride time
		GRBLinExpr objFunc = 0;
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {
				for (int j = 0; j < number_nodes; j++) {
					objFunc += travel_time[nodes[i]][nodes[j]]*x[b][i][j];
				}
			}
			
		}
		//model.add(IloMinimize(env, objFunc));
		//objFunc.end(); 
		model.setObjective(objFunc, GRB_MINIMIZE);

		//(6)
		//every request is served once

		for (int r = 0; r < total_requests; r++){

			GRBLinExpr sum = 0;

			for (int b = 0; b < total_number_vehicles; b++) {
				for (int i = 0; i < number_stops_origin[r]; i++) {
					int nodei = stops_origin[r][i];
					for (int j = 0; j < number_nodes; j++) {
						//cout<<travel_time[nodes[nodei]][nodes[j]]<<endl;
						sum += x[b][nodei][j];
					}
				}
			}
			model.addConstr(sum == 1);
			//model.add(sum == 1);
			//sum.end();

		}

		//(7)
		//picked up and dropped by the same vehicle
		for (int r = 0; r < total_requests; r++){
			for (int b = 0; b < total_number_vehicles; b++) {

				//IloExpr sum(env);
				GRBLinExpr sum = 0;
				for (int i = 0; i < number_stops_origin[r]; i++) {
					int nodei = stops_origin[r][i];
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][nodei][j];
					}
				}


				GRBLinExpr sum2 = 0;
				for (int j = 0; j < number_stops_destination[r]; j++) {
					int nodei = stops_destination[r][j];
					for (int i = 0; i < number_nodes; i++) {
						sum2 += x[b][nodei][i];
					}
				}

				//model.add(sum - sum2 == 0);
				model.addConstr(sum - sum2 == 0);
				//sum.end();
				//sum2.end();

			}
		}

		//(8)
		for (int b = 0; b < total_number_vehicles; b++) {

			GRBLinExpr sum = 0;
			for (int j = 0; j < number_nodes; j++) {
				sum += x[b][vehicle_located_at_depot[b]][j];
			}
			model.addConstr(sum == 1);
			//sum.end();

		}

		//(9)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int r = 0; r < total_requests; r++){
				for (int i = 0; i < number_stops_origin[r]; i++) {

					GRBLinExpr sum = 0;
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][j][stops_origin[r][i]];
					}

					GRBLinExpr sum2 = 0;
					for (int j = 0; j < number_nodes; j++) {
						sum2 += x[b][stops_origin[r][i]][j];
					}
					model.addConstr(sum - sum2 == 0);
					//sum.end();
					//sum2.end();

				}

				for (int i = 0; i < number_stops_destination[r]; i++) {

					GRBLinExpr sum = 0;
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][j][stops_destination[r][i]];
					}

					GRBLinExpr sum2 = 0;
					for (int j = 0; j < number_nodes; j++) {
						sum2 += x[b][stops_destination[r][i]][j];
					}
					model.addConstr(sum - sum2 == 0);
					//sum.end();
					//sum2.end();

				}

			}

		}

		//(10)
		for (int b = 0; b < total_number_vehicles; b++) {

			GRBLinExpr sum = 0;
			for (int j = 0; j < number_nodes; j++) {
				sum += x[b][j][vehicle_return_to_depot[b]];
			}
			model.addConstr(sum == 1);
			//sum.end();

		}

		//(11) *review*
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int r1 = 0; r1 < total_requests; r1++){

				for (int i = 0; i < number_stops_origin[r1]; i++) {

					int i1 = stops_origin[r1][i];
					q[i1] = -1;
					for (int r2 = 0; r2 < total_requests; r2++){

						if (r1 != r2) {
							for (int j = 0; j < number_stops_origin[r2]; j++) {
								int j1 = stops_origin[r2][j];
								M[b][i1][j1] = std::max(0, latest_departure[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_departure[r2]);
								model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
							}
						}

						for (int j = 0; j < number_stops_destination[r2]; j++) {
							int j1 = stops_destination[r2][j];
							M[b][i1][j1] = std::max(0, latest_departure[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_arrival[r2]);
							model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
						}
						//}

					}

					for (int j = 0; j < number_nodes_depots; j++) {
						int j1 = all_depots[j];
						M[b][i1][j1] = std::max(0, latest_departure[r1] + travel_time[nodes[i1]][nodes[j1]] - ts_min);
						model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
					}

				}

				for (int i = 0; i < number_stops_destination[r1]; i++) {

					int i1 = stops_destination[r1][i];
					q[i1] = 1;
					for (int r2 = 0; r2 < total_requests; r2++){

						if (r1 != r2) {

							for (int j = 0; j < number_stops_origin[r2]; j++) {
								int j1 = stops_origin[r2][j];
								M[b][i1][j1] = std::max(0, latest_arrival[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_departure[r2]);
								model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
							}

						
							for (int j = 0; j < number_stops_destination[r2]; j++) {
								int j1 = stops_destination[r2][j];
								M[b][i1][j1] = std::max(0, latest_arrival[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_arrival[r2]);
								model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
							}
						}

					

					}

					for (int j = 0; j < number_nodes_depots; j++) {
						int j1 = all_depots[j];
						M[b][i1][j1] = std::max(0, latest_arrival[r1] + travel_time[nodes[i1]][nodes[j1]] - ts_min);
						model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
					}

				}

				for (int i = 0; i < number_nodes_depots; i++) {
					int i1 = all_depots[i];
					q[i1] = 0;
					for (int r2 = 0; r2 < total_requests; r2++){

						for (int j = 0; j < number_stops_origin[r2]; j++) {
							int j1 = stops_origin[r2][j];
							M[b][i1][j1] = std::max(0, ts_max + travel_time[nodes[i1]][nodes[j1]] - earliest_departure[r2]);
							model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
						}

						for (int j = 0; j < number_stops_destination[r2]; j++) {
							int j1 = stops_destination[r2][j];
							M[b][i1][j1] = std::max(0, ts_max + travel_time[nodes[i1]][nodes[j1]] - earliest_arrival[r2]);
							model.addConstr(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
						}					

					}

					//for (int j = 0; j < number_depots; j++) {
						//int j1 = i1;
						//model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
					//}

				}

			}
		}



		//(12)
		//question here -> if the node isn't served. what T[b][i1] will be? 
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int r = 0; r < total_requests; r++){
				for (int i = 0; i < number_stops_origin[r]; i++) {
					int i1 = stops_origin[r][i];
					GRBLinExpr sum = 0;
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][i1][j];
					}
					model.addConstr(T[b][i1] >= earliest_departure[r]*sum);
					//sum.end();	
				}
			}
		}

		//(13)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int r = 0; r < total_requests; r++){
				for (int i = 0; i < number_stops_destination[r]; i++) {
					int i1 = stops_destination[r][i];
					GRBLinExpr sum = 0;
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][i1][j];
					}
					model.addConstr(T[b][i1] <= latest_arrival[r]*sum);
					//sum.end();	
				}
			}
		}

		//(14)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {
				for (int j = 0; j < number_nodes; j++) {

					W[b][i][j] = std::min(maxcapacity[vehicle_type[b]], maxcapacity[vehicle_type[b]] + q[i]);
					model.addConstr(Q[b][j] >= Q[b][i] + q[i]*x[b][i][j] - W[b][i][j]*(1 - x[b][i][j]));

				}
			}
		}

		//(15)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {

				GRBLinExpr sum = 0;
				for (int j = 0; j < number_nodes; j++) {
					sum += x[b][i][j];
				}
				//int max1 = std::max(0, q[i]);
				model.addConstr(Q[b][i] >= std::max(0, q[i])*sum);
				//sum.end();

			}
		}

		//(16)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {

				GRBLinExpr sum = 0;
				for (int j = 0; j < number_nodes; j++) {
					sum += x[b][i][j];
				}
				//int min1 = std::min(maxcapacity[vehicle_type[b]], maxcapacity[vehicle_type[b]]+q[i]);
				model.addConstr(Q[b][i] <= std::min(maxcapacity[vehicle_type[b]], maxcapacity[vehicle_type[b]]+q[i])*sum);
				//sum.end();

			}
		}


		//---
		//setting to 0 some variables
		bool can_be_part_of_solution = false;
		for (int i = 0; i < number_nodes; i++) {
			can_be_part_of_solution = false;
			for (int r = 0; r < total_requests; r++){
				//check if node belongs to the origin or destination of a request
				for (int j = 0; j < number_stops_origin[r]; j++) {
					int i1 = stops_origin[r][j];
					if (i1 == i) {
						can_be_part_of_solution = true;
					}
				}

				for (int j = 0; j < number_stops_destination[r]; j++) {
					int i1 = stops_destination[r][j];
					if (i1 == i) {
						can_be_part_of_solution = true;
					}
				}

				for (int j = 0; j < number_nodes_depots; j++) {
					int i1 = all_depots[j];
					if (i1 == i) {
						can_be_part_of_solution = true;
					}
				}

			}

			if (not can_be_part_of_solution) {
				for (int b = 0; b < total_number_vehicles; b++) {
					for (int j=0;j<number_nodes;j++) {
						//x[b][i][j].set(GRB_DoubleAttr_Start, 0.0);
						//x[b][j][i].set(GRB_DoubleAttr_Start, 0.0);

						x[b][i][j].set(GRB_DoubleAttr_LB, 0.0); // Lower bound
           		 		x[b][i][j].set(GRB_DoubleAttr_UB, 0.0); // Upper bound

           		 		x[b][j][i].set(GRB_DoubleAttr_LB, 0.0); // Lower bound
           		 		x[b][j][i].set(GRB_DoubleAttr_UB, 0.0); // Upper bound
					}
				}
			}
		}

		
		
		//adding to zero walking variables that are not origin/dest of passenger
		/*for (int p = 0; p < np; p++){
			for (int sf = 0; sf < Aw.size(); sf++){
				//int o = Aw[sf].i;
				//int d = Aw[sf].j;
				if ((origins[p] != Aw[sf].i) && (Aw[sf].typei == 2)) {
					count++;
					model.add(w[p][sf] == 0);
				}
				if ((destinations[p] != Aw[sf].j) && (Aw[sf].typej == 3)) {
					count++;
					model.add(w[p][sf] == 0);
				}
			}
		}*/

		//printf("%d\n", count);
		//IloCplex cplex2(model);
		//cplex2.exportModel("model.lp");
		model.write("model.lp");

		model.set(GRB_DoubleParam_TimeLimit, 3600); // Time limit
        model.set(GRB_DoubleParam_MIPGapAbs, 0.9); // Absolute gap

        //time_t start3 = time(NULL);
        std::time_t start = std::time(nullptr);

        /*if ( !cplex2.solve() ) {
            env.error() << "Failed to optimize LP." << endl;
            throw(-1);
        }*/
        // Enable IIS computation
        //env.set(GRB_IntParam_IISMethod, 1);

        model.optimize();

        /*cout<<"SEE WHICH INFEASIBLE"<<endl;
        if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
            // Compute the IIS
            model.computeIIS();
            model.write("iismodel.ilp");

        }
        cout<<"SEE WHICH INFEASIBLE"<<endl;*/

        /*if (model.get(GRB_IntAttr_Status) == GRB_INFEASIBLE) {
            // Compute the IIS
            model.computeIIS();

            // Get the number of constraints
			int numConstrs = model.get(GRB_IntAttr_NumConstrs);

			// Iterate over constraints to find which ones are in the IIS
			for (int i = 0; i < numConstrs; ++i) {
			    GRBConstr c = model.getConstr(i);
			    if (c.get(GRB_IntAttr_IISConstr) == 1) {
			        std::cout << "Constraint '" << c.get(GRB_StringAttr_ConstrName) << "' is in the IIS." << std::endl;
			    }
			}
        }*/

        if (model.get(GRB_IntAttr_Status) != GRB_OPTIMAL) {
            std::cerr << "Failed to optimize LP." << std::endl;
            throw(-1);
        }

        std::cout << "Solution status = " << model.get(GRB_IntAttr_Status) << endl;
        std::cout << "Solution value = " << model.get(GRB_DoubleAttr_ObjVal) << endl;
        std::cout << "Best upper bound = " << model.get(GRB_DoubleAttr_ObjBound) << endl;

        //double gap = 100*((cplex2.getBestObjValue() - cplex2.getObjValue())/cplex2.getObjValue());
        double gap = (model.get(GRB_DoubleAttr_ObjBound) - model.get(GRB_DoubleAttr_ObjVal)) / model.get(GRB_DoubleAttr_ObjVal) * 100;

        //time_t end3 = time(NULL);
        //elapsed3 = difftime(end3,start3);
        std::time_t end = std::time(nullptr);
        double elapsed = std::difftime(end, start);

        std::cout << "Solution time = " <<  elapsed << endl;

        std::ofstream output_file;
		output_file.open(output_filename, std::ios::app);
		output_file << requests_filename << " " << model.get(GRB_DoubleAttr_ObjVal) << " " << elapsed << endl;

		//printing solution
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {
				for (int j = 0; j < number_nodes; j++) {
					double bX = x[b][i][j].get(GRB_DoubleAttr_X);
					if (bX == 1.0){
						cout<<b<<" "<<nodes[i]<<" "<<nodes[j]<<" "<<travel_time[nodes[i]][nodes[j]]<<endl;
					}
				}
			}
			cout<<endl;
		}

	}

	catch (GRBException e) {
        std::cerr << "Concert exception caught: " << e.getMessage() << endl;
    }

    catch (...) {
        std::cerr << "Unknown exception caught" << endl;
    }

	//env.end();
}


int main(int argc, char **argv) {

	for (int i=1; i<argc; i++)
  	{
		if (strcmp(argv[i], "--filename_requests") == 0) {
			input_requests(argv[i+1]);
			cout<<argv[i+1]<<endl;
			requests_filename = argv[i+1];
		} else if (strcmp(argv[i], "--filename_travel_time") == 0) {
			input_travel_time(argv[i+1]);
		} else if (strcmp(argv[i], "--depot") == 0) {
			for (int j = 0; j < number_depots; j++) {
				i++;
				depot[j] = stoi(argv[i]);
				//depot[j] = number_nodes;
				//nodes[number_nodes] = stoi(argv[i]);
				type_node[depot[j]] = 3;
				//number_nodes++;
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
		} else if (strcmp(argv[i], "--number_vehicles1") == 0) {
			//for (int j = 0; j < number_type_vehicles; j++) {
			i++;
			float perc = (float)stoi(argv[i])/100;
			number_vehicles[0] = perc*total_requests;
			total_number_vehicles += number_vehicles[0];
			//}
		} else if (strcmp(argv[i], "--number_vehicles2") == 0) {
			//for (int j = 0; j < number_type_vehicles; j++) {
			i++;
			float perc = (float)stoi(argv[i])/100;
			number_vehicles[1] = perc*total_requests;
			total_number_vehicles += number_vehicles[1];
			//}
		} else if (strcmp(argv[i], "--number_vehicles3") == 0) {
			//for (int j = 0; j < number_type_vehicles; j++) {
			i++;
			float perc = (float)stoi(argv[i])/100;
			number_vehicles[2] = perc*total_requests;
			total_number_vehicles += number_vehicles[2];
			//}
		} else if (strcmp(argv[i], "--output_file") == 0) {
			//<<"ttm"<<endl;
			//input_travel_time(argv[i+1]);
			output_filename = argv[i+1];
			//<<"leave ttm"<<endl;
		}
	}

	/*for (int i = 0; i < number_depots; i++) {
   		number_vehicles_at_depot[i] = 0;
   	}*/

   	int k = 0;
   	number_nodes_depots = 0;
   	total_requests = 3;
	total_number_vehicles = 3;
	for (int i =0; i < total_requests; i++){
		number_stops_origin[i] = 2;
		number_stops_destination[i] = 2;
	}
   	for (int j=0; j<number_type_vehicles; j++) {

		for (int i=0; i<number_vehicles[j];i++) {
			
			//assign "randomly" a depot to the vehicle
			int depot_i = rand() % number_depots;

			if (k < total_number_vehicles) {
				all_depots[number_nodes_depots] = number_nodes;
				vehicle_located_at_depot[k] = number_nodes; 
				nodes[number_nodes] = depot[depot_i];
				number_nodes++;
				number_nodes_depots++;

				//repeat return node
				all_depots[number_nodes_depots] = number_nodes;
				vehicle_return_to_depot[k] = number_nodes; 
				nodes[number_nodes] = depot[depot_i];
				number_nodes++;
				number_nodes_depots++;
			}

			//vehicles_at_depot[vehicle_located_at_depot[k]][number_vehicles_at_depot[vehicle_located_at_depot[k]]] = k;
			//number_vehicles_at_depot[vehicle_located_at_depot[k]]++;

			vehicle_type[k] = j;

			k++;

		}
	}

	ts_min = 25200;
	ts_max = 32400;


	

	

	/*for (int k=0;k<total_requests;k++){
		latest_arrival[k] = latest_arrival[k] + 1800;
	}*/

	cout<<"success"<<endl;
	cout<<"HIER "<<total_number_vehicles<<" "<<number_stops_origin[0]<<" "<<number_nodes<<endl;
	MDHODBRPFR_MODEL();
	for (int i =0; i < total_requests; i++){
		for (int j =0; j < number_stops_origin[i]; j++){
			cout<<stops_origin[i][j]<<" ";
		}
		cout<<endl;
		for (int j =0; j < number_stops_destination[i]; j++){
			cout<<stops_destination[i][j]<<" ";
		}
		cout<<endl;
		cout<<endl;
	}
	cout<<"FINAL success2"<<endl;
	
	return 0;

}



