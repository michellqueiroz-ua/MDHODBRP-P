#pragma once
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

/*typedef int listP[maxpassengers + 1];
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

extern int n;
extern double comp_time;

extern listV vehicle_type;
extern listD depot;
extern listT number_vehicles;
extern listD number_vehicles_at_depot;
extern matrixDV vehicles_at_depot;
extern int total_number_vehicles;
extern listT maxcapacity;
extern listV vehicle_located_at_depot;
extern int total_served_passengers;
extern int number_type_vehicles;
extern int number_depots;
extern int number_requests;

extern matrixVSS M, W;
extern matrixSS travel_time;
extern listS stations_ids;
extern listS q;
extern map<int, int> station_id_map;

extern map<int, int> nodes;
extern map<int, int> type_node;
extern int number_nodes;

//information about the requests
extern listP time_stamp, earliest_departure, latest_departure, earliest_arrival, latest_arrival, direct_travel_time;
extern listP delay;
extern listP passengers_departure_time_from_home;
extern matrixPS stops_origin, stops_destination;
extern matrixPS walking_time_stops_origin, walking_time_stops_destination;
extern listP number_stops_origin, number_stops_destination;
extern int number_stations;

extern int ts_min, ts_max;*/

extern int ts_min, ts_max;
extern map<int, int> nodes;
extern listD depot;
extern map<int, int> type_node;
extern int number_nodes;
extern listT number_vehicles;
extern listD number_vehicles_at_depot;
extern int total_number_vehicles;
extern listT maxcapacity;
extern int number_type_vehicles;
extern int number_depots;
extern int number_requests;
extern listD number_vehicles_at_depot;
extern listV vehicle_located_at_depot;
extern matrixDV vehicles_at_depot;
extern listV vehicle_type;