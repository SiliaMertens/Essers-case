#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <numeric>
#include <time.h>
#include <ctime>
#include <chrono>
#include <algorithm>

#include "ProbabilityEstimator.h"
#include "Heuristic.h"
#include "GaussianDistribution.h"
#include "DiscreteDistribution.h"
using namespace std;
//A macro definition for the string gaussian for quick access
#define GAUSSIAN "gaussian"
void show(vector<int> const &input)
{
	cout << "shuffle ";
	for (auto const &i : input)
	{
		std::cout << i << " ";
	}
	cout << "\n";
}
void show_usage(std::string name)
{
	std::cerr << "---------------------------------------------------------------------\n"
			  << "Usage: " << name << " arg1 arg2 arg3 ..... arg 6 | ARGUMENTS (Required)\n"
			  << "---------------------------------------------------------------------\n"
			  << "ARGUMENTS:\n"
			  << "\tdata_file: A text file representing the dataset. E.g. Inputfile_tw2u.txt\n"
			  << "\tcollection_date: A specific day to run the algorithm on. E.g. 3-Sep-2018\n"
			  << "\tcoordinates_file: A text file representing an adjacency matrix of nodes. E.g. distance_matrix3sep.txt\n"
			  << "\tresolution: A string or an integer number representing the resolution of the probability estimator. E.g. gaussian for GaussianDistribution OR 14 for DiscreteDistribution\n"
			  << "\ttime_window_violation_cost: A double number. E.g. 0.5\n"
			  << "\tdriving_time_violation_cost: A double number. E.g. 1000\n"
			  << "Example: " << name << " General_Cargo_LTL_2018_v10072019_input_code\\ adjusted\\ tw.txt 2-Jan-2018 distance_matrix\\ 2\\ jan.txt 14 0.5 1000"
			  << std::endl;
}
// "General_Cargo_LTL_2018_v10072019_input_code adjusted tw.txt" "2-Jan-2018" "distance_matrix 2 jan.txt" 10 1000 10
string data_file;		 //= "Inputcode10customers.txt";
string coordinates_file; // = "distance_matrix10klanten.txt";

double time_window_violation_cost /*= 1*/;	  // = 10;
double driving_time_violation_cost /*= 1*/; // = 1000;
string resolution /*= "14"*/;
double perturbation_percentage = 0.15; //0.2;
double value_no_improvement = 10; //2;

/* Main function to run the algorithm */
int main(int argc, char *argv[])
{
	cout << " ============ Start of execution ======== "
		 << "\n";
	auto start = chrono::high_resolution_clock::now();



	srand(time(NULL));
	/*
	===============================================
	The beginning of the code
	===============================================
	*/

	/* 
	- Initialise the problem.
	- read the data.
	- Set some holder variables for (current/previous/local_best/ILS(iterated local search)) solutions
	*/
	struct problem p;

	if (false)
	{
		std::cerr << "The program expect 8 arguments to be passed.\nCurrently " + to_string(argc - 1) + " are passed \nSee Usage below." << std::endl;
		show_usage(argv[0]);
		return 1;
	}
	else
	{

		try
		{
			data_file = /*argv[1];*/ "C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\Inputfile_experiments_tw8u.txt";
			p.collection_date = /*argv[2];*/ "07-Sep-2018";
			coordinates_file = /*argv[3];*/ "C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\distance_matrix7sep.txt";
			resolution = /*argv[4];*/ "70";
			time_window_violation_cost = /*stod(argv[5]); */ stod("1");
			driving_time_violation_cost = /*stod(argv[6]);*/ stod("1");

			//data_file = /*argv[1];*/ "C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\Inputfile_experiments_tw2u.txt";
			//p.collection_date =/* argv[2]; */"02-Jan-2018";
			//coordinates_file =/* argv[3];*/ "C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\distance_matrix2jan.txt";
			//value_no_improvement = /*stod(argv[4]);*/ stod("2");
			//perturbation_percentage= /*stod(argv[5]);*/  stod("0.1");
			

			//if (resolution == GAUSSIAN)
			//{
			//	cout << "Initial solution with " << resolution << "\n";
			//	p.pe = new GaussianDistribution();
			//}
			//else
			//{
				cout << "Initial solution with " << resolution << " bins Discrete distribution.\n";
				p.pe = new DiscreteDistribution();
			//}
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			show_usage(argv[0]);
			throw std::invalid_argument("main() -> Value is not converted to double or does not exist in the arguments. See the usage.");
		}
	}

	//if (argc > 1) {
	//	data_file = argv[1];
	//	p.collection_date = argv[2];
	//	coordinates_file = argv[3];
	//	perturbation_percentage = stod(argv[4]);
	//	value_no_improvement = stoi(argv[5]);
	//	time_window_violation_cost = stod(argv[6]);
	//	driving_time_violation_cost = stod(argv[7]);
	//	resolution = argv[8];
	//}

	read_data(p);
	read_distance_and_time_matrix(p);

	p.pe->readDistributions("C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\" + p.collection_date, resolution); // read probabilities of 'p.collection_date'

	struct solution s_curr;
	struct solution s_prev;
	struct solution s_local_best;
	struct solution s_total_best;
	struct solution s_ILS_best;
	struct solution s_actual;
	initialize_solution(p, s_curr);
	initialize_solution(p, s_prev);
	initialize_solution(p, s_local_best);
	initialize_solution(p, s_total_best);
	initialize_solution(p, s_ILS_best);
	initialize_solution(p, s_actual);

	struct solution s_recourse;
	initialize_solution(p, s_recourse);

	s_curr.routes[0].route = { 0, 235, 232, 50, 120, 42, 0 };
	s_curr.routes[1].route = { 0, 155, 110, 0 };
	s_curr.routes[2].route = { 0, 181, 200, 153, 109, 119, 197	,	73, 0 };
	s_curr.routes[3].route = { 0, 22, 1, 0 };
	s_curr.routes[4].route = { 0, 31, 26, 55, 183, 0 };
	s_curr.routes[5].route = { 0, 16, 74, 38, 136, 0 };
	s_curr.routes[6].route = { 0, 204, 220, 151, 0 };
	s_curr.routes[7].route = { 0, 184, 68, 40, 89, 0 };
	s_curr.routes[8].route = { 0, 175, 128, 141, 0 };
	s_curr.routes[9].route = { 0, 79, 63, 61, 23, 112, 0 };
	s_curr.routes[10].route = { 0, 64, 102, 0 };
	s_curr.routes[11].route = { 0, 13, 238, 144, 179, 0 };
	s_curr.routes[12].route = { 0, 48, 10, 15, 0 };
	s_curr.routes[13].route = { 0, 190, 188, 104, 0 };
	s_curr.routes[14].route = { 0, 66, 59, 83, 0 };
	s_curr.routes[15].route = { 0, 94, 115, 100, 0 };
	s_curr.routes[16].route = { 0, 164, 134, 37, 142, 0 };
	s_curr.routes[17].route = { 0, 92, 24, 18, 101, 0 };
	s_curr.routes[18].route = { 0, 236, 205, 217, 212, 227, 0 };
	s_curr.routes[19].route = { 0, 52, 34, 53, 0 };
	s_curr.routes[20].route = { 0, 116, 93, 44, 17, 207, 29, 0 };
	s_curr.routes[21].route = { 0, 99, 78, 32, 3, 0 };
	s_curr.routes[22].route = { 0, 171, 173, 172, 0 };
	s_curr.routes[23].route = { 0, 98, 118, 0 };
	s_curr.routes[24].route = { 0, 138, 213, 196, 163, 193, 19	,	0 };
	s_curr.routes[25].route = { 0, 27, 11, 234, 216, 219, 202,	0 };
	s_curr.routes[26].route = { 0, 125, 0 };
	s_curr.routes[27].route = { 0, 145, 84, 30, 0 };
	s_curr.routes[28].route = { 0, 107, 223, 208, 201, 81, 56,	9	, 0 };
	s_curr.routes[29].route = { 0, 182, 191, 25, 7, 0 };
	s_curr.routes[30].route = { 0, 152, 203, 167, 210, 0 };
	s_curr.routes[31].route = { 0, 33, 165, 192, 0 };
	s_curr.routes[32].route = { 0, 177, 0 };
	s_curr.routes[33].route = { 0, 147, 82, 71, 72, 146, 58, 21,	46, 49, 0 };
	s_curr.routes[34].route = { 0, 76, 0 };
	s_curr.routes[35].route = { 0, 41, 117, 2, 5, 0 };
	s_curr.routes[36].route = { 0, 137, 80, 124, 47, 0 };
	s_curr.routes[37].route = { 0, 218, 211, 225, 226, 0 };
	s_curr.routes[38].route = { 0, 87, 88, 90, 0 };
	s_curr.routes[39].route = { 0, 133, 129, 67, 0 };
	s_curr.routes[40].route = { 0, 54, 237, 230, 139, 0 };
	s_curr.routes[41].route = { 0, 149, 121, 111, 96, 97, 0 };
	s_curr.routes[42].route = { 0, 229, 186, 194, 0 };
	s_curr.routes[43].route = { 0, 158, 103, 140, 36, 77, 0 };
	s_curr.routes[44].route = { 0, 168, 169, 239, 8, 0 };
	s_curr.routes[45].route = { 0, 39, 105, 91, 0 };
	s_curr.routes[46].route = { 0, 95, 65, 0 };
	s_curr.routes[47].route = { 0, 209, 131, 170, 126, 0 };
	s_curr.routes[48].route = { 0, 154, 157, 156, 0 };
	s_curr.routes[49].route = { 0, 143, 132, 150, 0 };
	s_curr.routes[50].route = { 0, 160, 14, 108, 113, 75, 62,	0 };
	s_curr.routes[51].route = { 0, 6, 206, 28, 159, 0 };
	s_curr.routes[52].route = { 0, 57, 180, 199, 185, 0 };
	s_curr.routes[53].route = { 0, 174, 176, 0 };
	s_curr.routes[54].route = { 0, 122, 228, 187, 178, 123, 0 };
	s_curr.routes[55].route = { 0, 35, 86, 130, 0 };
	s_curr.routes[56].route = { 0, 12, 0 };
	s_curr.routes[57].route = { 0, 221, 222, 214, 215, 224, 198,	4, 0 };
	s_curr.routes[58].route = { 0, 135, 70, 0 };
	s_curr.routes[59].route = { 0, 195, 45, 20, 0 };
	s_curr.routes[60].route = { 0, 162, 166, 161, 240, 51, 114	,	0 };
	s_curr.routes[61].route = { 0, 189, 231, 69, 43, 85, 148,	0 };
	s_curr.routes[62].route = { 0, 233, 241, 106, 127, 60, 0 };


	//update_solution(s_curr, s_recourse);

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	update_earliest_time(p, s_curr, vehicle_id);

	//	
	//	update_latest_time(p, s_curr, vehicle_id);
	//	update_schedule(p, s_curr, vehicle_id);
	//}

	//
	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	bereken_route_cost_zonder_recourse(p, s_curr, vehicle_id);
	//	bereken_route_cost(p, s_curr, vehicle_id);
	//	calculate_probabilities(p, s_curr, vehicle_id);
	//}

	//

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	bereken_gewogen_route_cost(p, s_curr, s_recourse, vehicle_id);
	//}

	//calculate_total_cost(p, s_curr);

	//cout << "Initial solution with " << s_curr.number_of_vehicles_used << " vehicles and distance " << s_curr.total_distance_cost << " distance_parameter " << s_curr.total_distance_parameter
	//	<< " route duration " << s_curr.total_route_duration << " route duration_parameter " << s_curr.total_route_duration_parameter << " time window violation " << s_curr.total_time_window_violation
	//	<< " time window violation_parameter " << s_curr.total_time_window_violation_parameter
	//	<< " overtime " << s_curr.total_overtime << " overtime_parameter " << s_curr.total_overtime_parameter << "driving time violation " << s_curr.total_driving_time_violation 
	//	<< " driving time violation parameter " << s_curr.total_driving_time_violation_parameter << " total cost " << s_curr.total_cost << "\n";
	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	for (size_t position = 0; position < s_curr.routes[vehicle_id].route.size(); position++) {
	//		cout << s_curr.routes[vehicle_id].route[position] << " ";
	//	}

	//	cout << "\n";
	//}


	update_solution(s_curr, s_recourse);

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		update_earliest_time(p, s_curr, vehicle_id);

		
		update_latest_time(p, s_curr, vehicle_id);
		update_schedule(p, s_curr, vehicle_id);
	}

	update_solution(s_curr, s_actual);
	
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
				actual_demand(p, s_actual, vehicle_id);
	
			}
	
	calculate_total_cost_actualdemand(p, s_actual);
	
	//cout << "total cost actual demand " << s_actual.total_cost << "\n";

	//cout << "TOTAL: Initial_solution_with vehicles " << s_actual.number_of_vehicles_used << " vehicle_cost " << s_actual.vehicle_cost << " total_distance_cost " << s_actual.total_distance_cost << " total_distance_parameter " << s_actual.total_distance_parameter
	//	<< " total_route_duration " << s_actual.total_route_duration << " total_route_duration_parameter " << s_actual.total_route_duration_parameter << " time_window_violation " << s_actual.total_time_window_violation << " time_window_violation_parameter " 
	//	<< s_actual.total_time_window_violation_parameter << " overtime " << s_actual.total_overtime << " overtime_parameter " << s_actual.total_overtime_parameter << " driving_time_violation " << s_actual.total_driving_time_violation 
	//	<< " driving_time_violation_parameter " << s_actual.total_driving_time_violation_parameter << " total_cost " << s_actual.total_cost 
	//	<< " INITIAL: Initial_solution_with_distance_cost " << s_actual.total_distance_cost_without_recourse << " initial_distance_parameter " << s_actual.total_distance_parameter_without_recourse
	//	<< " initial_route_duration " << s_actual.total_route_duration_without_recourse << " initial_route_duration_parameter " << s_actual.total_route_duration_parameter_without_recourse << " initial_total_cost " << s_actual.total_cost_without_recourse
	//	<< " RECOURSE: Initial_solution_with_distance_cost " << s_actual.total_distance_cost_with_recourse << " recourse_distance_parameter " << s_actual.total_distance_parameter_with_recourse
	//	<< " recourse_route_duration " << s_actual.total_route_duration_with_recourse << " recourse_route_duration_parameter " << s_actual.total_route_duration_parameter_with_recourse << " recourse_time_window_violation " << s_actual.total_time_window_violation 
	//	<< " recourse_time_window_violation_parameter " << s_actual.total_time_window_violation_parameter << " recourse_overtime " << s_actual.total_overtime << " recourse_overtime_parameter " << s_actual.total_overtime_parameter << " recourse_driving_time_violation " 
	//	<< s_actual.total_driving_time_violation << " recourse_driving_time_violation_parameter " << s_actual.total_driving_time_violation_parameter << " recourse_total_cost " << s_actual.total_cost_with_recourse << "\n";

	cout << "number_of_vehicles " << s_actual.number_of_vehicles_used << " vehicle_cost " << s_actual.vehicle_cost << " total_distance_cost " << s_actual.total_distance_cost << " initial_distance_cost " << s_actual.total_distance_cost_without_recourse
		<< " recourse_distance_cost " << s_actual.total_distance_cost_with_recourse << " total_distance_parameter " << s_actual.total_distance_parameter << " initial_distance_parameter " << s_actual.total_distance_parameter_without_recourse
		<< " recourse_distance_parameter " << s_actual.total_distance_parameter_with_recourse << " total_route_duration " << s_actual.total_route_duration << " initial_route_duration " << s_actual.total_route_duration_without_recourse
		<< " recourse_route_duration " << s_actual.total_route_duration_with_recourse << " total_route_duration_parameter " << s_actual.total_route_duration_parameter << " initial_route_duration_parameter " 
		<< s_actual.total_route_duration_parameter_without_recourse << " recourse_route_duration_parameter " << s_actual.total_route_duration_parameter_with_recourse << " time_window_violation " << s_actual.total_time_window_violation 
		<< " time_window_violation_parameter " << s_actual.total_time_window_violation_parameter << " overtime " << s_actual.total_overtime << " overtime_parameter " << s_actual.total_overtime_parameter << " driving_time_violation " 
		<< s_actual.total_driving_time_violation << " driving_time_violation_parameter " << s_actual.total_driving_time_violation_parameter << " total_cost " << s_actual.total_cost << " initial_total_cost " << s_actual.total_cost_without_recourse 
		<< " recourse_total_cost " << s_actual.total_cost_with_recourse << "\n";


	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 0; position < s_actual.routes[vehicle_id].route.size(); position++) {
				cout << s_actual.routes[vehicle_id].route[position] << " ";
		}

		cout << "\n";
	}


	/*write_output_file(p, s_curr);*/

	//TODO: Optimise this customer (random) insertion code
	/*
	-----------------------
	START: Initial solution code (Insertion procedure)
	-----------------------
	- Radomise the customers
	- Insert them at best positions (considering distance and COST, see perform_best_insertion())
	*/
	// Code to randomly insert customers at their best-initial-position (e.g. distance based)
	//FIXME: This code block might be not needed (just making a vector of 1 to n_customers)

//	vector<int> customers_to_be_inserted = {};
//
//	for (int customer_id = 1; customer_id <= p.n_customers; customer_id++)
//	{
//
//		customers_to_be_inserted.push_back(customer_id);
//	}
//
//	//cout << "customers to be inserted " << customers_to_be_inserted.size() << "\n";
//
//	for (int i = 0; i < customers_to_be_inserted.size() - 1; i++)
//	{
//		int j = i + rand() % (customers_to_be_inserted.size() - i);
//		swap(customers_to_be_inserted[i], customers_to_be_inserted[j]);
//	}
//
//	show(customers_to_be_inserted);
//	//TODO: Refactor the initial solution code (this block and perform_best_insertion())
//	//FIXME: The block of generating a vector + randomly shuffling + this block,,, are all to do one thing (SORT based on distance (which is done inside the perform_best_insertion))
//	for (int customer_id = 0; customer_id <= customers_to_be_inserted.size() - 1; customer_id++)
//	{
//		perform_best_insertion(p, s_curr, customers_to_be_inserted[customer_id]);
//	}
//
//	// Put current solution as local_best
//	update_solution(s_curr, s_local_best);
//
//	cout << "TOTAL: Initial solution with " << s_local_best.number_of_vehicles_used << " vehicles " << s_local_best.vehicle_cost << " vehicle cost and distance " << s_local_best.total_distance_cost << " distance_parameter " << s_local_best.total_distance_parameter
//		 << " route duration " << s_local_best.total_route_duration << " route duration parameter " << s_local_best.total_route_duration_parameter << " time window violation " << s_local_best.total_time_window_violation << " time window violation parameter " << s_local_best.total_time_window_violation_parameter << " overtime " << s_local_best.total_overtime << " overtime_parameter " << s_local_best.total_overtime_parameter << " driving time violation " << s_local_best.total_driving_time_violation << " driving time violation parameter " << s_local_best.total_driving_time_violation_parameter << " total cost " << s_local_best.total_cost << "\n";
//	cout << "NORMAL: Initial solution with distance " << s_local_best.total_distance_cost_without_recourse << " distance_parameter " << s_local_best.total_distance_parameter_without_recourse
//		<< " route duration " << s_local_best.total_route_duration_without_recourse << " route duration parameter " << s_local_best.total_route_duration_parameter_without_recourse << " total cost " << s_local_best.total_cost_without_recourse << "\n";
//	cout << "RECOURSE: Initial solution with distance " << s_local_best.total_distance_cost_with_recourse << " distance_parameter " << s_local_best.total_distance_parameter_with_recourse
//		<< " route duration " << s_local_best.total_route_duration_with_recourse << " route duration parameter " << s_local_best.total_route_duration_parameter_with_recourse << " time window violation " << s_local_best.total_time_window_violation << " time window violation parameter " << s_local_best.total_time_window_violation_parameter << " overtime " << s_local_best.total_overtime << " overtime_parameter " << s_local_best.total_overtime_parameter << " driving time violation " << s_local_best.total_driving_time_violation << " driving time violation parameter " << s_local_best.total_driving_time_violation_parameter << " total cost " << s_local_best.total_cost_with_recourse << "\n";
//	
//	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
//	{
//		for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
//		{
//			cout << s_local_best.routes[vehicle_id].route[position] << " ";
//		}
//
//		cout << "\n";
//	}
//
//	//update_solution(s_local_best, s_actual);
//
//	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
//	//	actual_demand(p, s_actual, vehicle_id);
//
//	//}
//
//	//calculate_total_cost_actualdemand(p, s_actual);
//
//	//cout << "TOTAL: Initial solution with " << s_actual.number_of_vehicles_used << " vehicles " << s_actual.vehicle_cost << " vehicle cost and distance " << s_actual.total_distance_cost << " distance_parameter " << s_actual.total_distance_parameter
//	//	<< " route duration " << s_actual.total_route_duration << " route duration parameter " << s_actual.total_route_duration_parameter << " time window violation " << s_actual.total_time_window_violation << " time window violation parameter " << s_actual.total_time_window_violation_parameter << " overtime " << s_actual.total_overtime << " overtime_parameter " << s_actual.total_overtime_parameter << " driving time violation " << s_actual.total_driving_time_violation << " driving time violation parameter " << s_actual.total_driving_time_violation_parameter << " total cost " << s_actual.total_cost << "\n";
//	//cout << "NORMAL: Initial solution with distance " << s_actual.total_distance_cost_without_recourse << " distance_parameter " << s_actual.total_distance_parameter_without_recourse
//	//	<< " route duration " << s_actual.total_route_duration_without_recourse << " route duration parameter " << s_actual.total_route_duration_parameter_without_recourse << " total cost " << s_actual.total_cost_without_recourse << "\n";
//	//cout << "RECOURSE: Initial solution with distance " << s_actual.total_distance_cost_with_recourse << " distance_parameter " << s_actual.total_distance_parameter_with_recourse
//	//	<< " route duration " << s_actual.total_route_duration_with_recourse << " route duration parameter " << s_actual.total_route_duration_parameter_with_recourse << " time window violation " << s_actual.total_time_window_violation << " time window violation parameter " << s_actual.total_time_window_violation_parameter << " overtime " << s_actual.total_overtime << " overtime_parameter " << s_actual.total_overtime_parameter << " driving time violation " << s_actual.total_driving_time_violation << " driving time violation parameter " << s_actual.total_driving_time_violation_parameter << " total cost " << s_actual.total_cost_with_recourse << "\n";
//
//
//
//
//	update_solution(s_local_best, s_total_best);
//	update_solution(s_local_best, s_ILS_best);
//
//	cout << "total best " << s_total_best.total_cost << "\n";
//	/*
//	-----------------------
//	END: initial solution  (Insertion procedure)
//	-----------------------
//	*/
//
//	//TODO Optimise Relocate & SWAP & Relocate flows (i.e. the procedure is identical but the functions differs)
//	/*
//	-----------------------
//	START: Relocate flow
//	-----------------------
//	- Remove from initial position and insert at the best possible position, in the same route or other routes.
//		- This will effect/change route lengths (remove VS. insert).
//		- Neighborhood check is done here as well, which means calcuting the cost/impact of this relocate on the other routes as well.
//	- The relocate happens for all routes, for all clients (see inside relocate()). 
//	- The relocate flow stops when there is no more improvments (note: current setup is considering only one relocation at a time)
//	*/
//	// Store the best solution as the previous best.
//
//
///*	while (s_local_best.total_cost < s_prev.total_cost) {*/ // relocate en swap uitvoeren totdat er geen verbeteringen meer gevonden worden 
//
//		update_solution(s_local_best, s_prev);
//		relocate(p, s_prev, s_curr, s_local_best);
//
//		// while loop uitvoeren op RELOCATE totdat er geen verbeteringen meer zijn
//		while (s_local_best.total_cost < s_prev.total_cost)
//		{
//			update_solution(s_local_best, s_prev);
//			relocate(p, s_prev, s_curr, s_local_best);
//			//TODO: Optimise all prints in a function or store them in files.
//			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
//				<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " total cost " << s_local_best.total_cost << "\n";
//			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
//			{
//				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
//				{
//					cout << s_local_best.routes[vehicle_id].route[position] << " ";
//				}
//				cout << "\n";
//			}
//
//			cout << "best cost na relocate " << s_local_best.total_cost << "\n";
//		}
//
//		//Update holder variables with the new results.
//		if (s_local_best.total_cost < s_total_best.total_cost)
//		{
//			update_solution(s_local_best, s_total_best);
//			update_solution(s_local_best, s_ILS_best);
//
//			cout << "total best " << s_total_best.total_cost << "\n";
//			//cout << "ILS best " << s_ILS_best.total_cost << "\n";
//		}
//		/*
//		-----------------------
//		END: Relocate flow
//		-----------------------
//		*/
//
//		/*
//		-----------------------
//		START: SWAP flow
//		-----------------------
//		- Swap two customers from different routes.
//			Differences with relocate:
//				- Never in the same routes.
//				- Current Route length is never change in swap whereas in relocate it does remove from one route and insert in another route (changing the length of both routes).
//		- The swap happens for all routes, for all clients (see inside swap()).
//		- The swap flow stops when there is no more improvments (note: current setup is considering only one relocation at a time)
//		*/
//		// Store the best solution as the previous best.
//
//
//
//		update_solution(s_local_best, s_prev);
//		swap(p, s_prev, s_curr, s_local_best);
//		// while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn
//		while (s_local_best.total_cost < s_prev.total_cost)
//		{ // while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn
//			update_solution(s_local_best, s_prev);
//			swap(p, s_prev, s_curr, s_local_best);
//
//			cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
//				<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " total cost " << s_local_best.total_cost << "\n";
//			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
//			{
//				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
//				{
//					cout << s_local_best.routes[vehicle_id].route[position] << " ";
//				}
//
//				cout << "\n";
//			}
//
//			cout << "best cost na swap " << s_local_best.total_cost << "\n";
//		}
//		//Update holder variables with the new results.
//		if (s_local_best.total_cost < s_total_best.total_cost)
//		{
//			update_solution(s_local_best, s_total_best);
//			update_solution(s_local_best, s_ILS_best);
//
//			cout << "total best " << s_total_best.total_cost << "\n";
//			cout << "ILS best " << s_ILS_best.total_cost << "\n";
//		}
//		/*
//		-----------------------
//		END: SWAP flow
//		-----------------------
//		*/
//
//	//}
//
//	
//
//	/*
//	-----------------------
//	START: Perturbation flow
//	-----------------------
//	- Random percentage of customer is removed from the best solution found so far (found after initial-relocate-swap-relocate).
//	- Removed customers are inserted again by the insertion procedure (first removed, then initial solution wiht perform_best_insertion())
//	- After insertion, the operators are executed again (Relocate - Swap - Relocate)
//	- Current percentage is 30% but this will be a controlled parameter after optimising the code. 
//	TODO: Check with Silia if this is working,, seems to have mistakes in lines from 290 - 355, and 
//	*/
//
//	// Temp variable to check the number of iterations without chage (i.e. Convergence check/Stop condition)
//	int number_of_times_without_improvement = 0;
//
//	cout << "value no improvement " << value_no_improvement << "\n";
//
//	//Iterate until the stop condition is met (e.g. currently, no more imporvments for 2 time/iterations).
//	//for (int iteration = 0; iteration < iterations; iteration++) {
//	while (number_of_times_without_improvement < value_no_improvement)
//	{
//
//		update_solution(s_total_best, s_local_best);
//
//		vector<int> random_customers = {};
//
//		while (random_customers.size() < perturbation_percentage * p.n_customers)
//		{
//
//			int customer = rand() % p.n_customers + 1;
//
//			std::vector<int>::iterator it = find(random_customers.begin(), random_customers.end(), customer);
//			if (it == random_customers.end())
//			{
//				//cout << "curr " << j_curr << "\n";
//				random_customers.push_back(customer);
//			}
//		}
//
//		cout << "verwijderde klanten ";
//		for (size_t i = 0; i < random_customers.size(); i++)
//		{
//			cout << random_customers[i] << " ";
//		}
//
//		cout << "\n";
//
//		s_local_best.route_customer = {};
//		s_local_best.position_customer = {};
//
//		for (size_t i = 0; i < random_customers.size(); i++)
//		{
//			position_removed_customers(p, s_local_best, random_customers[i]);
//			remove_customer(p, s_local_best, s_local_best.route_customer[i], s_local_best.position_customer[i]);
//		}
//
//		cout << "route na verwijdering "
//			 << "\n";
//		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
//		{
//			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
//			{
//				cout << s_local_best.routes[vehicle_id].route[position] << " ";
//			}
//
//			cout << "\n";
//		}
//
//		cout << "\n";
//
//		/*
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-
//		END: Randomly remove a percentage of customer
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-
//		*/
//
//		/*
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//		START:  Insertion procedure of the removed customers
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//		*/
//		for (size_t customer_id = 0; customer_id < random_customers.size(); customer_id++)
//		{ // verwijderde klanten terug invoegen
//			perform_best_insertion(p, s_local_best, random_customers[customer_id]);
//		}
//
//		cout << "New initial solution after perturbation with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
//			 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " total cost " << s_local_best.total_cost << "\n";
//		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
//		{
//			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
//			{
//				cout << s_local_best.routes[vehicle_id].route[position] << " ";
//			}
//
//			cout << "\n";
//		}
//
//		cout << "\n";
//
//		//write_output_file_perturbation(p, s_local_best);
//
//		if (s_local_best.total_cost < s_total_best.total_cost)
//		{
//			update_solution(s_local_best, s_total_best);
//
//			cout << "total best " << s_total_best.total_cost << "\n";
//		}
//		/*
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//		END:  Insertion procedure of the removed customers
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//		*/
//
//		/*
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//		START:  Relocate - Swap flows after the insertion of removed customer
//		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//		*/
//
//		//while (s_local_best.total_cost < s_prev.total_cost) {
//
//			update_solution(s_local_best, s_prev);
//			relocate(p, s_prev, s_curr, s_local_best);
//
//			//cout << "\nNew best solution after first relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
//			//	<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
//			//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
//			//	for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
//			//		cout << s_local_best.routes[vehicle_id].route[position] << " ";
//			//	}
//			//	cout << "\n";
//			//}
//
//			//cout << "best cost na first relocate " << s_local_best.total_cost << "\n";
//
//			//if (s_local_best.total_cost < s_total_best.total_cost) {
//			//	update_solution(s_local_best, s_total_best);
//
//			//	cout << "total best " << s_total_best.total_cost << "\n";
//			//}
//
//			while (s_local_best.total_cost < s_prev.total_cost)
//			{ // while loop uitvoeren totdat er geen verbeteringen meer zijn
//
//				update_solution(s_local_best, s_prev);
//				relocate(p, s_prev, s_curr, s_local_best);
//
//				cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
//					<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " time window violation parameter " << s_local_best.total_time_window_violation_parameter
//					<< " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " driving time violation parameter " << s_local_best.total_driving_time_violation_parameter << " total cost " << s_local_best.total_cost << "\n";
//				for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
//				{
//					for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
//					{
//						cout << s_local_best.routes[vehicle_id].route[position] << " ";
//					}
//
//					cout << "\n";
//				}
//
//				cout << "best cost na relocate " << s_local_best.total_cost << "\n";
//			}
//
//			if (s_local_best.total_cost < s_total_best.total_cost)
//			{
//				update_solution(s_local_best, s_total_best);
//
//				cout << "total best " << s_total_best.total_cost << "\n";
//			}
//
//			if (s_local_best.total_cost < s_ILS_best.total_cost)
//			{
//				update_solution(s_local_best, s_ILS_best);
//
//				cout << "ILS best " << s_ILS_best.total_cost << "\n";
//			}
//
//			update_solution(s_local_best, s_prev);
//			swap(p, s_prev, s_curr, s_local_best);
//
//			while (s_local_best.total_cost < s_prev.total_cost)
//			{
//				update_solution(s_local_best, s_prev);
//				swap(p, s_prev, s_curr, s_local_best);
//
//				cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
//					<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " time window violation parameter " << s_local_best.total_time_window_violation_parameter
//					<< " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " driving time violation parameter " << s_local_best.total_driving_time_violation_parameter << " total cost " << s_local_best.total_cost << "\n";
//				for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
//				{
//					for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
//					{
//						cout << s_local_best.routes[vehicle_id].route[position] << " ";
//					}
//
//					cout << "\n";
//				}
//
//				cout << "best cost na swap " << s_local_best.total_cost << "\n";
//			}
//
//			if (s_local_best.total_cost < s_total_best.total_cost)
//			{
//				update_solution(s_local_best, s_total_best);
//
//				cout << "total best " << s_total_best.total_cost << "\n";
//			}
//
//			if (s_local_best.total_cost < s_ILS_best.total_cost)
//			{
//				update_solution(s_local_best, s_ILS_best);
//
//				cout << "ILS best " << s_ILS_best.total_cost << "\n";
//			}
//
//			/*
//			-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//			END:  Relocate - Swap flows after the insertion of removed customer
//			-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
//			*/
//
//		//}
//	
//
//		//FIXME: Is this ever going to be TRUE?! Related to lone 295
//		if (s_local_best.total_cost < s_total_best.total_cost)
//		{
//			update_solution(s_local_best, s_total_best);
//
//			cout << "total best " << s_total_best.total_cost << "\n";
//		}
//		/* 
//		FIXME: The problem with the current code of the if-else block is related to the Code-Smell of the Remove Percentage Customers.
//		 The code might be working, but it is wrong.
//		 The local_best is being updated with new random remove but not copied properly before.
//		*/
//		//  Update the ILS best solution, if the (current) local_best cost is smaller than the ILS_best cost.
//		if (s_local_best.total_cost < s_ILS_best.total_cost)
//		{
//			update_solution(s_local_best, s_ILS_best);
//
//			cout << "ILS best " << s_ILS_best.total_cost << "\n";
//		}
//		//Otherwise, increase the number of times without improvments
//		else
//		{
//			number_of_times_without_improvement++;
//
//			cout << "number of times without improvement " << number_of_times_without_improvement << "\n";
//		}
//
//		cout << "final ILS best " << s_ILS_best.total_cost << "\n";
//
//
//		cout << "\nFINALE ROUTE ";
//
//		cout << "\nBeste route " << s_ILS_best.number_of_vehicles_used << " vehicles and distance " << s_ILS_best.total_distance_cost
//			<< " route duration " << s_ILS_best.total_route_duration << " time window violation " << s_ILS_best.total_time_window_violation <<
//			" overtime " << s_ILS_best.total_overtime << " driving time violation " << s_ILS_best.total_driving_time_violation << " total cost " << s_ILS_best.total_cost << "\n";
//
//		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
//			for (size_t position = 0; position < s_ILS_best.routes[vehicle_id].route.size(); position++) {
//				cout << s_ILS_best.routes[vehicle_id].route[position] << " ";
//			}
//
//			cout << "\n";
//		}
//
//		cout << "\n";
//
//		update_solution(s_ILS_best, s_actual);
//
//		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
//			actual_demand(p, s_actual, vehicle_id);
//
//		}
//
//		calculate_total_cost_actualdemand(p, s_actual);
//
//		cout << "total cost actual demand " << s_actual.total_cost << "\n";
//
//		auto stop = chrono::high_resolution_clock::now();
//
//		// Get duration. Substart timepoints to
//		// get durarion. To cast it to proper unit
//		// use duration cast method
//		auto duration = chrono::duration_cast<chrono::minutes>(stop - start);
//		cout << "Time taken to run the algorithm is: : " << duration.count() << " minutes" << endl;
//		cout << " ============ End of execution ======== (still waiting to write the results) "
//			 << "\n";
//
//		//write_output_file(p, s_ILS_best);
//		write_output_file(p, s_actual);
//	}
//
//	//std::string file_name = "csv_results_parameter_setting.csv";
//	//write_csv_output(p, s_ILS_best, file_name);
//	 std::string file_name_actual ="csv_results_september_actual_demand_test.csv";
//	 write_csv_output(p, s_actual, file_name_actual);
//	
	return 0;
}
