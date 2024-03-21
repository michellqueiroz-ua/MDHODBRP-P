//#include <ilcplex/ilocplex.h>
#include "Global.h"
#include "Input.h"
#include <iomanip>
#include "gurobi_c++.h"
#include <vector>
//using namespace std;



string IntToString (int a)
{
    ostringstream temp;
    temp<<a;
    return temp.str();
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
		IloExpr objFunc(env);
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {
				for (int j = 0; j < number_nodes; j++) {
					objFunc += travel_time[nodes[i]][nodes[j]]*x[b][i][j];
				}
			}
			
		}
		model.add(IloMinimize(env, objFunc));
		objFunc.end(); 


		//(2)
		for (int r = 0; r < number_requests; r++){

			IloExpr sum(env);
			for (int b = 0; b < total_number_vehicles; b++) {
				for (int i = 0; i < number_stops_origin[r]; i++) {
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][stops_origin[r][i]][j];
					}
				}
			}
			model.add(sum == 1);
			sum.end();

		}

		//(3)
		for (int r = 0; r < number_requests; r++){
			for (int b = 0; b < total_number_vehicles; b++) {

				IloExpr sum(env);
				for (int i = 0; i < number_stops_origin[r]; i++) {
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][stops_origin[r][i]][j];
					}
				}


				IloExpr sum2(env);
				for (int j = 0; j < number_stops_destination[r]; j++) {
					for (int i = 0; i < number_nodes; i++) {
						sum2 += x[b][stops_destination[r][i]][j];
					}
				}

				model.add(sum - sum2 == 0);
				sum.end();
				sum2.end();

			}
		}

		//(4)
		for (int b = 0; b < total_number_vehicles; b++) {

			IloExpr sum(env);
			for (int j = 0; j < number_nodes; j++) {
				sum += x[b][vehicle_located_at_depot[b]][j];
			}
			model.add(sum == 1);
			sum.end();

		}

		//(5)
		for (int b = 0; b < total_number_vehicles; b++) {


			for (int r = 0; r < number_requests; r++){
				for (int i = 0; i < number_stops_origin[r]; i++) {

					IloExpr sum(env);
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][j][stops_origin[r][i]];
					}

					IloExpr sum2(env);
					for (int j = 0; j < number_nodes; j++) {
						sum2 += x[b][stops_origin[r][i]][j];
					}
					model.add(sum - sum2 == 0);
					sum.end();
					sum2.end();

				}

				for (int i = 0; i < number_stops_destination[r]; i++) {

					IloExpr sum(env);
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][j][stops_destination[r][i]];
					}

					IloExpr sum2(env);
					for (int j = 0; j < number_nodes; j++) {
						sum2 += x[b][stops_destination[r][i]][j];
					}
					model.add(sum - sum2 == 0);
					sum.end();
					sum2.end();

				}

			}

		}

		//(6)
		for (int b = 0; b < total_number_vehicles; b++) {

			IloExpr sum(env);
			for (int j = 0; j < number_nodes; j++) {
				sum += x[b][j][vehicle_located_at_depot[b]];
			}
			model.add(sum == 1);
			sum.end();

		}

		//(7) *review*
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int r1 = 0; r1 < number_requests; r1++){

				for (int i = 0; i < number_stops_origin[r1]; i++) {

					int i1 = stops_origin[r1][i];
					q[i1] = -1;
					for (int r2 = 0; r2 < number_requests; r2++){

						if (r1 != r2) {
							for (int j = 0; j < number_stops_origin[r2]; j++) {
								int j1 = stops_origin[r2][j];
								M[b][i1][j1] = std::max(0, latest_departure[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_departure[r2]);
								model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
							}

							for (int j = 0; j < number_stops_destination[r2]; j++) {
								int j1 = stops_destination[r2][j];
								M[b][i1][j1] = std::max(0, latest_departure[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_arrival[r2]);
								model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
							}
						}

					}

					for (int j = 0; j < number_depots; j++) {
						int j1 = depot[j];
						M[b][i1][j1] = std::max(0, latest_departure[r1] + travel_time[nodes[i1]][nodes[j1]] - ts_min);
						model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
					}

				}

				for (int i = 0; i < number_stops_destination[r1]; i++) {

					int i1 = stops_destination[r1][i];
					q[i1] = 1;
					for (int r2 = 0; r2 < number_requests; r2++){

						if (r1 != r2) {
							for (int j = 0; j < number_stops_origin[r2]; j++) {
								int j1 = stops_origin[r2][j];
								M[b][i1][j1] = std::max(0, latest_arrival[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_departure[r2]);
								model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
							}

							for (int j = 0; j < number_stops_destination[r2]; j++) {
								int j1 = stops_destination[r2][j];
								M[b][i1][j1] = std::max(0, latest_arrival[r1] + travel_time[nodes[i1]][nodes[j1]] - earliest_arrival[r2]);
								model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
							}
						}

					

					}

					for (int j = 0; j < number_depots; j++) {
						int j1 = depot[j];
						M[b][i1][j1] = std::max(0, latest_arrival[r1] + travel_time[nodes[i1]][nodes[j1]] - ts_min);
						model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
					}

				}

				for (int i = 0; i < number_depots; i++) {
					int i1 = depot[i];
					q[i1] = 0;
					for (int r2 = 0; r2 < number_requests; r2++){

						for (int j = 0; j < number_stops_origin[r2]; j++) {
							int j1 = stops_origin[r2][j];
							M[b][i1][j1] = std::max(0, ts_max + travel_time[nodes[i1]][nodes[j1]] - earliest_departure[r2]);
							model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
						}

						for (int j = 0; j < number_stops_destination[r2]; j++) {
							int j1 = stops_destination[r2][j];
							M[b][i1][j1] = std::max(0, ts_max + travel_time[nodes[i1]][nodes[j1]] - earliest_arrival[r2]);
							model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
						}					

					}

					//for (int j = 0; j < number_depots; j++) {
						//int j1 = i1;
						//model.add(T[b][j1] >= T[b][i1] + travel_time[nodes[i1]][nodes[j1]]*x[b][i1][j1] - M[b][i1][j1]*(1 - x[b][i1][j1]));
					//}

				}

			}
		}



		//(8)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int r = 0; r < number_requests; r++){
				for (int i = 0; i < number_stops_origin[r]; i++) {
					int i1 = stops_origin[r][i];
					IloExpr sum(env);
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][i1][j];
					}
					model.add(T[b][i1] >= earliest_departure[r]*sum);
					sum.end();	
				}
			}
		}

		//(9)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int r = 0; r < number_requests; r++){
				for (int i = 0; i < number_stops_origin[r]; i++) {
					int i1 = stops_destination[r][i];
					IloExpr sum(env);
					for (int j = 0; j < number_nodes; j++) {
						sum += x[b][i1][j];
					}
					model.add(T[b][i1] <= latest_arrival[r]*sum);
					sum.end();	
				}
			}
		}

		//(10)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {
				for (int j = 0; j < number_nodes; j++) {

					W[b][i][j] =  std::min(maxcapacity[vehicle_type[b]], maxcapacity[vehicle_type[b]] + q[i]);
					model.add(Q[b][j] >= Q[b][i] + q[i]*x[b][i][j] - W[b][i][j]*(1 - x[b][i][j]));

				}
			}
		}

		//(11)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {

				IloExpr sum(env);
				for (int j = 0; j < number_nodes; j++) {
					sum += x[b][i][j];
				}
				int max1 = std::max(0, q[i]);
				model.add(Q[b][i] >= max1*sum);
				sum.end();

			}
		}

		//(12)
		for (int b = 0; b < total_number_vehicles; b++) {
			for (int i = 0; i < number_nodes; i++) {

				IloExpr sum(env);
				for (int j = 0; j < number_nodes; j++) {
					sum += x[b][i][j];
				}
				int min1 = std::min(maxcapacity[vehicle_type[b]], maxcapacity[vehicle_type[b]]+q[i]);
				model.add(Q[b][i] <= min1*sum);
				sum.end();

			}
		}


		//---

		
		
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
		IloCplex cplex2(model);
		cplex2.exportModel("model.lp");

		cplex2.setParam(IloCplex::TiLim, 3600);
        cplex2.setParam(IloCplex::EpAGap, 0.9);

        time_t start3 = time(NULL);

        if ( !cplex2.solve() ) {
            env.error() << "Failed to optimize LP." << endl;
            throw(-1);
        }

        env.out() << "Solution status = " << cplex2.getStatus() << endl;
        env.out() << "Solution value = " << cplex2.getObjValue() << endl;
        env.out() << "Best upper bound = " << cplex2.getBestObjValue() << endl;

        double gap = 100*((cplex2.getBestObjValue() - cplex2.getObjValue())/cplex2.getObjValue());

        time_t end3 = time(NULL);
        elapsed3 = difftime(end3,start3);

        env.out() << "Solution time = " <<  elapsed3 << endl;

	}

	catch (IloException& e) {
        cerr << "Concert exception caught: " << e << endl;
    }

    catch (...) {
        cerr << "Unknown exception caught" << endl;
    }

	env.end();
}


int main(int argc, char **argv) {

	for (int i=1; i<argc; i++)
  	{
		if (strcmp(argv[i], "--filename_requests") == 0) {
			input_requests(argv[i+1]);
			cout<<argv[i+1]<<endl;
		} else if (strcmp(argv[i], "--filename_travel_time") == 0) {
			input_travel_time(argv[i+1]);

		} else if (strcmp(argv[i], "--depot") == 0) {
			for (int j = 0; j < number_depots; j++) {
				i++;
				depot[j] = number_nodes;
				nodes[number_nodes] = stoi(argv[i]);
				type_node[number_nodes] = 3;
				number_nodes++;
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
		} else if (strcmp(argv[i], "--number_requests") == 0) {
			number_requests = stoi(argv[i+1]);
		}
	}

	for (int i = 0; i < number_depots; i++) {
   		number_vehicles_at_depot[i] = 0;
   	}

   	int k = 0;
   	for (int j=0; j<number_type_vehicles; j++) {

		for (int i=0; i<number_vehicles[j];i++) {
			
			//assign "randomly" a depot to the vehicle
			vehicle_located_at_depot[k] = rand() % number_depots; 

			vehicles_at_depot[vehicle_located_at_depot[k]][number_vehicles_at_depot[vehicle_located_at_depot[k]]] = k;
			number_vehicles_at_depot[vehicle_located_at_depot[k]]++;

			vehicle_type[k] = j;

			k++;

		}
	}

	ts_min = 25200;
	ts_max = 32400;

	MDHODBRPFR_MODEL();

	return 0;

	
}



