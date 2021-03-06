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

inline bool ends_with(std::string const& value, std::string const& ending)
{
	if (ending.size() > value.size()) return false;
	return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

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
			  << "Usage: " << name << " arg1 arg2 arg3 ..... arg 8 | ARGUMENTS (Required)\n"
			  << "---------------------------------------------------------------------\n"
			  << "ARGUMENTS:\n"
			  << "\toptimization: type of optimization e.g. deterministic\n"
			  << "\tdata_file: A text file representing the dataset. E.g. Inputfile_tw2u.txt\n"
			  << "\tcollection_date: A specific day to run the algorithm on. E.g. 3-Sep-2018\n"
			  << "\tcoordinates_file: A text file representing an adjacency matrix of nodes. E.g. distance_matrix3sep.txt\n"
			  << "\ttime_window_violation_cost: A double number. E.g. 0.5\n"
			  << "\tdriving_time_violation_cost: A double number. E.g. 1000\n"
			  << "\tdistribution_file: A text file with the distribution, each line is <order_id>,<given_demand>,... where ... are doubles representing a distribution\n"
			  << "Example: " << name << " Stochastic Inputfile_experiments_tw2u0.txt 04-Sep-2018 distance_matrix4sep.txt 0.5 1000 emp_04-Sep-2018_b1370.csv"
			  << std::endl;
}
// "General_Cargo_LTL_2018_v10072019_input_code adjusted tw.txt" "2-Jan-2018" "distance_matrix 2 jan.txt" 10 1000 10 "emp_04-Sep-2018_b137.csv"
string data_file;		 //= "Inputcode10customers.txt";
string coordinates_file; // = "distance_matrix10klanten.txt";

double time_window_violation_cost;	  // = 10;
double driving_time_violation_cost; // = 1000;
string distribution_file;
double perturbation_percentage = 0.15; //0.2;
double value_no_improvement = 10; //2;
string optimization; 

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

	if (argc < 8)
	{
		std::cerr << "The program expect 8 arguments to be passed.\nCurrently " + to_string(argc - 1) + " are passed \nSee Usage below." << std::endl;
		show_usage(argv[0]);
		return 1;
	}
	else
	{

		try
		{
			optimization = argv[1]; /*"Stochastic";*/
			data_file = argv[2]; /*"C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\Inputfile_experiments_tw2u0.txt";*/
			p.collection_date = argv[3]; /*"04-Sep-2018";*/
			coordinates_file = argv[4]; /*"C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\distance_matrix4sep.txt";*/
			time_window_violation_cost = stod(argv[5]);  /*stod("1");*/
			driving_time_violation_cost = stod(argv[6]); /*stod("1");*/
			distribution_file = argv[7]; /*"C:\\Users\\lucp9937\\source\\repos\\SiliaMertens\\Essers-case\\emp_04-Sep-2018_b1370.csv";*/
			
			


			if (ends_with(distribution_file, "gaussian.csv"))
			{
				cout << "Initial solution with " << distribution_file << ", Gaussian distribution\n";
				p.pe = new GaussianDistribution();
			}
			else
			{
				cout << "Initial solution with " << distribution_file << ", Discrete distribution.\n";
				p.pe = new DiscreteDistribution();
			}
		}
		catch (const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			show_usage(argv[0]);
			throw std::invalid_argument("main(), exception during argument parsing. See the usage:");
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

	cout << "Found " << p.n_customers << " customers\n";

	p.pe->readDistributions(distribution_file); // read probabilities of 'p.collection_date'

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

	//TODO: Optimise this customer (random) insertion code
	/*
	-----------------------
	START: Initial solution code (Insertion procedure)(customers_to_be_inserted.size() - i)
	-----------------------
	- Radomise the customers
	- Insert them at best positions (considering distance and COST, see perform_best_insertion())
	*/
	// Code to randomly insert customers at their best-initial-position (e.g. distance based)
	//FIXME: This code block might be not needed (just making a vector of 1 to n_customers)

	vector<int> customers_to_be_inserted = {};

	for (int customer_id = 1; customer_id <= p.n_customers; customer_id++)
	{

		customers_to_be_inserted.push_back(customer_id);
	}

	//cout << "customers to be inserted " << customers_to_be_inserted.size() << "\n";

	for (int i = 0; i < customers_to_be_inserted.size() - 1; i++)
	{
		int j = i + rand() % (customers_to_be_inserted.size() - i);
		swap(customers_to_be_inserted[i], customers_to_be_inserted[j]);
	}

	show(customers_to_be_inserted);
	//TODO: Refactor the initial solution code (this block and perform_best_insertion())
	//FIXME: The block of generating a vector + randomly shuffling + this block,,, are all to do one thing (SORT based on distance (which is done inside the perform_best_insertion))
	for (int customer_id = 0; customer_id <= customers_to_be_inserted.size() - 1; customer_id++)
	{
		perform_best_insertion(p, s_curr, customers_to_be_inserted[customer_id]);
	}

	// Put current solution as local_best
	update_solution(s_curr, s_local_best);

	cout << "Initial solution with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost << " distance_parameter " << s_local_best.total_distance_parameter
		 << " route duration " << s_local_best.total_route_duration << " route duration parameter " << s_local_best.total_route_duration_parameter << " time window violation " << s_local_best.total_time_window_violation << " time window violation parameter " << s_local_best.total_time_window_violation_parameter << " overtime " << s_local_best.total_overtime << " overtime_parameter " << s_local_best.total_overtime_parameter << " driving time violation " << s_local_best.total_driving_time_violation << " driving time violation parameter " << s_local_best.total_driving_time_violation_parameter << " total cost " << s_local_best.total_cost << "\n";

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	//{
	//	for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
	//	{
	//		cout << s_local_best.routes[vehicle_id].route[position] << " ";
	//		//cout << "specified demand " << p.nodes[s_local_best.routes[vehicle_id].route[position]].specified_demand << " ";
	//		//cout << " actual demand " << p.nodes[s_local_best.routes[vehicle_id].route[position]].actual_demand << " ";
	//	
	//	}

	//	//cout << "cost normal " << s_local_best.routes[vehicle_id].weighted_route_cost << " with " << s_local_best.routes[vehicle_id].weighted_route_cost_with_recourse << " without " 
	//	//	<< s_local_best.routes[vehicle_id].weighted_route_cost_without_recourse << " TW " << s_local_best.routes[vehicle_id].weighted_time_window_violation << "\n";

	//	//cout << " route cost " << s_local_best.routes[vehicle_id].route_cost << " no recourse "<< s_local_best.routes[vehicle_id].route_cost_no_recourse << " recourse " << s_local_best.routes[vehicle_id].route_cost_recourse << "\n";

	//	//cout << "\n";
	//}

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	//{
	//	for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
	//	{

	//		cout << s_local_best.routes[vehicle_id].weighted_route_cost << " ";
	//	}

	//	cout << "\n";
	//}

	update_solution(s_local_best, s_total_best);
	update_solution(s_local_best, s_ILS_best);

	cout << "total best " << s_total_best.total_cost << "\n";
	/*
	-----------------------
	END: initial solution  (Insertion procedure)
	-----------------------
	*/

	//TODO Optimise Relocate & SWAP & Relocate flows (i.e. the procedure is identical but the functions differs)
	/*
	-----------------------
	START: Relocate flow
	-----------------------
	- Remove from initial position and insert at the best possible position, in the same route or other routes.
		- This will effect/change route lengths (remove VS. insert).
		- Neighborhood check is done here as well, which means calcuting the cost/impact of this relocate on the other routes as well.
	- The relocate happens for all routes, for all clients (see inside relocate()). 
	- The relocate flow stops when there is no more improvments (note: current setup is considering only one relocation at a time)
	*/
	// Store the best solution as the previous best.


/*	while (s_local_best.total_cost < s_prev.total_cost) {*/ // relocate en swap uitvoeren totdat er geen verbeteringen meer gevonden worden 

		update_solution(s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		// while loop uitvoeren op RELOCATE totdat er geen verbeteringen meer zijn
		while (s_local_best.total_cost < s_prev.total_cost)
		{
			update_solution(s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);
			//TODO: Optimise all prints in a function or store them in files.
			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
			{
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
				{
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}
				cout << "\n";
			}

			cout << "best cost na relocate " << s_local_best.total_cost << "\n";
		}

		//Update holder variables with the new results.
		if (s_local_best.total_cost < s_total_best.total_cost)
		{
			update_solution(s_local_best, s_total_best);
			update_solution(s_local_best, s_ILS_best);

			cout << "total best " << s_total_best.total_cost << "\n";
			//cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}
		/*
		-----------------------
		END: Relocate flow
		-----------------------
		*/

		/*
		-----------------------
		START: SWAP flow
		-----------------------
		- Swap two customers from different routes.
			Differences with relocate:
				- Never in the same routes.
				- Current Route length is never change in swap whereas in relocate it does remove from one route and insert in another route (changing the length of both routes).
		- The swap happens for all routes, for all clients (see inside swap()).
		- The swap flow stops when there is no more improvments (note: current setup is considering only one relocation at a time)
		*/
		// Store the best solution as the previous best.



		update_solution(s_local_best, s_prev);
		swap(p, s_prev, s_curr, s_local_best);
		// while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn
		while (s_local_best.total_cost < s_prev.total_cost)
		{ // while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn
			update_solution(s_local_best, s_prev);
			swap(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
			{
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
				{
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}

				cout << "\n";
			}

			cout << "best cost na swap " << s_local_best.total_cost << "\n";
		}
		//Update holder variables with the new results.
		if (s_local_best.total_cost < s_total_best.total_cost)
		{
			update_solution(s_local_best, s_total_best);
			update_solution(s_local_best, s_ILS_best);

			cout << "total best " << s_total_best.total_cost << "\n";
			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}
		/*
		-----------------------
		END: SWAP flow
		-----------------------
		*/

	//}

	

	/*
	-----------------------
	START: Perturbation flow
	-----------------------
	- Random percentage of customer is removed from the best solution found so far (found after initial-relocate-swap-relocate).
	- Removed customers are inserted again by the insertion procedure (first removed, then initial solution wiht perform_best_insertion())
	- After insertion, the operators are executed again (Relocate - Swap - Relocate)
	- Current percentage is 30% but this will be a controlled parameter after optimising the code. 
	TODO: Check with Silia if this is working,, seems to have mistakes in lines from 290 - 355, and 
	*/

	// Temp variable to check the number of iterations without chage (i.e. Convergence check/Stop condition)
	int number_of_times_without_improvement = 0;

	cout << "value no improvement " << value_no_improvement << "\n";

	//Iterate until the stop condition is met (e.g. currently, no more imporvments for 2 time/iterations).
	//for (int iteration = 0; iteration < iterations; iteration++) {
	while (number_of_times_without_improvement < value_no_improvement)
	{

		update_solution(s_total_best, s_local_best);

		vector<int> random_customers = {};

		while (random_customers.size() < perturbation_percentage * p.n_customers)
		{

			int customer = rand() % p.n_customers + 1;

			std::vector<int>::iterator it = find(random_customers.begin(), random_customers.end(), customer);
			if (it == random_customers.end())
			{
				//cout << "curr " << j_curr << "\n";
				random_customers.push_back(customer);
			}
		}

		cout << "verwijderde klanten ";
		for (size_t i = 0; i < random_customers.size(); i++)
		{
			cout << random_customers[i] << " ";
		}

		cout << "\n";

		s_local_best.route_customer = {};
		s_local_best.position_customer = {};

		for (size_t i = 0; i < random_customers.size(); i++)
		{
			position_removed_customers(p, s_local_best, random_customers[i]);
			remove_customer(p, s_local_best, s_local_best.route_customer[i], s_local_best.position_customer[i]);
		}

		cout << "route na verwijdering "
			 << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
		{
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
			{
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";
		}

		cout << "\n";

		/*
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-
		END: Randomly remove a percentage of customer
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*--*-*-*-
		*/

		/*
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		START:  Insertion procedure of the removed customers
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		*/
		for (size_t customer_id = 0; customer_id < random_customers.size(); customer_id++)
		{ // verwijderde klanten terug invoegen
			perform_best_insertion(p, s_local_best, random_customers[customer_id]);
		}

		cout << "New initial solution after perturbation with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
		{
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
			{
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";
		}

		cout << "\n";

		//write_output_file_perturbation(p, s_local_best);

		if (s_local_best.total_cost < s_total_best.total_cost)
		{
			update_solution(s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}
		/*
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		END:  Insertion procedure of the removed customers
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		*/

		/*
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		START:  Relocate - Swap flows after the insertion of removed customer
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		*/

		//while (s_local_best.total_cost < s_prev.total_cost) {

			update_solution(s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);

			//cout << "\nNew best solution after first relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			//	<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
			//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			//	for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
			//		cout << s_local_best.routes[vehicle_id].route[position] << " ";
			//	}
			//	cout << "\n";
			//}

			//cout << "best cost na first relocate " << s_local_best.total_cost << "\n";

			//if (s_local_best.total_cost < s_total_best.total_cost) {
			//	update_solution(s_local_best, s_total_best);

			//	cout << "total best " << s_total_best.total_cost << "\n";
			//}

			while (s_local_best.total_cost < s_prev.total_cost)
			{ // while loop uitvoeren totdat er geen verbeteringen meer zijn

				update_solution(s_local_best, s_prev);
				relocate(p, s_prev, s_curr, s_local_best);

				cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
					<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " time window violation parameter " << s_local_best.total_time_window_violation_parameter
					<< " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " driving time violation parameter " << s_local_best.total_driving_time_violation_parameter << " total cost " << s_local_best.total_cost << "\n";
				for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
				{
					for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
					{
						cout << s_local_best.routes[vehicle_id].route[position] << " ";
					}

					cout << "\n";
				}

				cout << "best cost na relocate " << s_local_best.total_cost << "\n";
			}

			if (s_local_best.total_cost < s_total_best.total_cost)
			{
				update_solution(s_local_best, s_total_best);

				cout << "total best " << s_total_best.total_cost << "\n";
			}

			if (s_local_best.total_cost < s_ILS_best.total_cost)
			{
				update_solution(s_local_best, s_ILS_best);

				cout << "ILS best " << s_ILS_best.total_cost << "\n";
			}

			update_solution(s_local_best, s_prev);
			swap(p, s_prev, s_curr, s_local_best);

			while (s_local_best.total_cost < s_prev.total_cost)
			{
				update_solution(s_local_best, s_prev);
				swap(p, s_prev, s_curr, s_local_best);

				cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
					<< " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " time window violation parameter " << s_local_best.total_time_window_violation_parameter
					<< " overtime " << s_local_best.total_overtime << " driving time violation " << s_local_best.total_driving_time_violation << " driving time violation parameter " << s_local_best.total_driving_time_violation_parameter << " total cost " << s_local_best.total_cost << "\n";
				for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
				{
					for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
					{
						cout << s_local_best.routes[vehicle_id].route[position] << " ";
					}

					cout << "\n";
				}

				cout << "best cost na swap " << s_local_best.total_cost << "\n";
			}

			if (s_local_best.total_cost < s_total_best.total_cost)
			{
				update_solution(s_local_best, s_total_best);

				cout << "total best " << s_total_best.total_cost << "\n";
			}

			if (s_local_best.total_cost < s_ILS_best.total_cost)
			{
				update_solution(s_local_best, s_ILS_best);

				cout << "ILS best " << s_ILS_best.total_cost << "\n";
			}

			/*
			-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
			END:  Relocate - Swap flows after the insertion of removed customer
			-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
			*/

		//}
	

		//FIXME: Is this ever going to be TRUE?! Related to lone 295
		if (s_local_best.total_cost < s_total_best.total_cost)
		{
			update_solution(s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}
		/* 
		FIXME: The problem with the current code of the if-else block is related to the Code-Smell of the Remove Percentage Customers.
		 The code might be working, but it is wrong.
		 The local_best is being updated with new random remove but not copied properly before.
		*/
		//  Update the ILS best solution, if the (current) local_best cost is smaller than the ILS_best cost.
		if (s_local_best.total_cost < s_ILS_best.total_cost)
		{
			update_solution(s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}
		//Otherwise, increase the number of times without improvments
		else
		{
			number_of_times_without_improvement++;

			cout << "number of times without improvement " << number_of_times_without_improvement << "\n";
		}

		cout << "final ILS best " << s_ILS_best.total_cost << "\n";


		//write_output_file(p, s_ILS_best);
	 } 

		cout << "\nFINALE ROUTE ";

		cout << "\nBeste route " << s_ILS_best.number_of_vehicles_used << " vehicles and distance " << s_ILS_best.total_distance_cost
			<< " route duration " << s_ILS_best.total_route_duration << " time window violation " << s_ILS_best.total_time_window_violation <<
			" overtime " << s_ILS_best.total_overtime << " driving time violation " << s_ILS_best.total_driving_time_violation << " total cost " << s_ILS_best.total_cost << "\n";
		
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_ILS_best.routes[vehicle_id].route.size(); position++) {
				cout << s_ILS_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";
		}

		cout << "\n";

		//write_output_file(p, s_ILS_best);

		update_solution(s_ILS_best, s_actual);

		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			actual_demand(p, s_actual, vehicle_id);

		}

		calculate_total_cost_actualdemand(p, s_actual);

		cout << "total cost actual demand " << s_actual.total_cost << "\n";

		cout << "\nBeste route " << s_actual.number_of_vehicles_used << " vehicles and distance " << s_actual.total_distance_cost
			<< " route duration " << s_actual.total_route_duration << " time window violation " << s_actual.total_time_window_violation <<
			" overtime " << s_actual.total_overtime << " driving time violation " << s_actual.total_driving_time_violation << " total cost " << s_actual.total_cost << "\n";


		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {

			cout << "route" << vehicle_id;
			for (int position = 0; position < s_actual.routes[vehicle_id].route.size(); position++) {
				cout << " " << s_actual.routes[vehicle_id].route[position] << " ";

			}

			cout << "\n";

			cout << "specified_demand ";
			for (int position = 0; position < s_actual.routes[vehicle_id].route.size(); position++) {
				cout << p.nodes[s_actual.routes[vehicle_id].route[position]].specified_demand << " ";
			}

			cout << "\n";

			cout << "sum_specified_demand ";

			double sum_spec_demand = 0.0;

			for (int position = 0; position < s_actual.routes[vehicle_id].route.size(); position++) {
				sum_spec_demand += p.nodes[s_actual.routes[vehicle_id].route[position]].specified_demand;
				cout << sum_spec_demand << " ";
			}

			cout << "\n";

			//output_file << endl << "sum_specified_demand " << ;

			cout << "actual_demand ";
			for (int position = 0; position < s_actual.routes[vehicle_id].route.size(); position++) {
				cout << p.nodes[s_actual.routes[vehicle_id].route[position]].actual_demand << " ";
			}

			cout << "\n";

			cout << "sum_actual_demand ";

			double sum_actual_demand = 0.0;

			for (int position = 0; position < s_actual.routes[vehicle_id].route.size(); position++) {
				sum_actual_demand += p.nodes[s_actual.routes[vehicle_id].route[position]].actual_demand;
				cout << sum_actual_demand << " ";
			}

			cout << "\n";

			/*		cout << "number_of_vehicles " << s_actual.number_of_vehicles_used << "\n";*/

			cout << "Distance_cost: " << s_actual.routes[vehicle_id].distance_cost << " Distance_parameter: " << s_actual.routes[vehicle_id].weighted_distance_parameter
				<< " Route_duration: " << s_actual.routes[vehicle_id].weighted_route_duration << " Route_duration_parameter: " << s_actual.routes[vehicle_id].weighted_route_duration_parameter << " Total_cost: " << s_actual.routes[vehicle_id].weighted_route_cost;

			cout << "\n";

			cout << "Distance_cost_no_recourse: " << s_actual.routes[vehicle_id].distance_cost_no_recourse << " Distance_cost_recourse: " << s_actual.routes[vehicle_id].distance_cost_recourse
				<< " Distance_parameter_no_recourse: " << s_actual.routes[vehicle_id].distance_parameter_no_recourse << " Distance_parameter_recourse: " << s_actual.routes[vehicle_id].distance_parameter_recourse
				<< " Route_duration_no_recourse: " << s_actual.routes[vehicle_id].route_duration_no_recourse << " Route_duration_recourse: " << s_actual.routes[vehicle_id].route_duration_recourse
				<< " Route_duration_parameter_no_recourse: " << s_actual.routes[vehicle_id].route_duration_parameter_no_recourse << " Route_duration_parameter_recourse: " << s_actual.routes[vehicle_id].route_duration_parameter_recourse
				<< " Time_window_violation: " << s_actual.routes[vehicle_id].time_window_violation_recourse << " Time_window_violation_parameter: " << s_actual.routes[vehicle_id].time_window_violation_parameter_recourse
				<< " Overtime: " << s_actual.routes[vehicle_id].overtime_recourse << " Overtime_parameter: " << s_actual.routes[vehicle_id].overtime_recourse
				<< " Driving_time_violation: " << s_actual.routes[vehicle_id].driving_time_violation_recourse << " Driving_time_violation_parameter: " << s_actual.routes[vehicle_id].driving_time_violation_recourse
				<< " Total_cost_no_recourse: " << s_actual.routes[vehicle_id].route_cost_no_recourse << " Total_cost_recourse: " << s_actual.routes[vehicle_id].route_cost_recourse;

			cout << "\n";

	
		
			//write_output_file(p, s_ILS_best);

		}

		double tot_dist_cost = 0.0;
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			tot_dist_cost += s_actual.routes[vehicle_id].distance_cost;
		}

		cout << "tot_dist_cost " << tot_dist_cost << "\n";
		//cout << "total_distance " << s_actual.total_distance_cost << "\n";
		//cout << "total_distance_without_recourse " << s_actual.total_distance_cost_without_recourse << "\n";
		//cout << "total_distance_with_recourse " << s_actual.total_distance_cost_with_recourse << "\n";

		write_output_file_actual_demand(p, s_actual);

		auto stop = chrono::high_resolution_clock::now();

		// Get duration. Substart timepoints to
		// get durarion. To cast it to proper unit
		// use duration cast method
		auto duration = chrono::duration_cast<chrono::minutes>(stop - start);
		cout << "Time taken to run the algorithm is: : " << duration.count() << " minutes" << endl;
		cout << " ============ End of execution ======== (still waiting to write the results) "
			 << "\n";
		
	//}

	//std::string file_name = "csv_results.csv";
	//write_csv_output(p, s_ILS_best, file_name);
	 std::string file_name_actual ="csv_results_actual_demand.csv";
	 write_csv_output(p, s_actual, file_name_actual);
	
	return 0;
}
