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

listV vehicle_type;
listD depot;
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

matrixSS travel_time;
listS stations_ids;
listS q;
map<int, int> station_id_map;

map<int, int> nodes;
map<int, int> type_node;
int number_nodes;

//information about the requests
listP time_stamp, earliest_departure, latest_departure, earliest_arrival, latest_arrival, direct_travel_time;
listP delay;
listP passengers_departure_time_from_home;
matrixPS stops_origin, stops_destination;
matrixPS walking_time_stops_origin, walking_time_stops_destination;
listP number_stops_origin, number_stops_destination;
int number_stations;

int ts_min, ts_max;

