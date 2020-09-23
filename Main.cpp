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

using namespace std;

void show(vector<int> const &input)
{
	cout << "shuffle ";
	for (auto const &i : input)
	{
		std::cout << i << " ";
	}
	cout << "\n";
}

string data_file = "General_Cargo_LTL_2018_v10072019_input_code adjusted tw.txt";
string coordinates_file = "distance_matrix 2 jan.txt";

double time_window_violation_cost = 10;
double allowable_operating_time_cost = 1000;

/* Main function to run the algorithm */
int main(int argc, char *argv[])
{
	cout << " ============ Start of execution ======== "
		 << "\n";
	auto start = chrono::high_resolution_clock::now();

	double perturbation_percentage = 0.2;
	int value_no_improvement = 2;

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

	p.collection_date = "2-Jan-2018";

	//if (argc > 1) {

	//	time_window_violation_cost = atof(argv[1]);
	//	allowable_operating_time_cost = atof(argv[2]);
	//}

	//if (argc > 1) {
	//	perturbation_percentage = atoi(argv[1]);
	//	value_no_improvement = atoi(argv[2]);

	//}

	read_data(p);
	read_distance_and_time_matrix(p);

	p.pe.readDistributions(p.collection_date); // read probabilities of 'p.collection_date'

	struct solution s_curr;
	struct solution s_prev;
	struct solution s_local_best;
	struct solution s_total_best;
	struct solution s_ILS_best;
	initialize_solution(p, s_curr);
	initialize_solution(p, s_prev);
	initialize_solution(p, s_local_best);
	initialize_solution(p, s_total_best);
	initialize_solution(p, s_ILS_best);

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
	update_solution(p, s_curr, s_local_best);

	cout << "Initial solution with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
		 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	{
		for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
		{
			cout << s_local_best.routes[vehicle_id].route[position] << " ";
		}

		cout << "\n";
	}

	update_solution(p, s_local_best, s_total_best);
	update_solution(p, s_local_best, s_ILS_best);
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

	update_solution(p, s_local_best, s_prev);
	relocate(p, s_prev, s_curr, s_local_best);

	// while loop uitvoeren op RELOCATE totdat er geen verbeteringen meer zijn
	while (s_local_best.total_cost < s_prev.total_cost)
	{ // while loop uitvoeren op RELOCATE totdat er geen verbeteringen meer zijn
		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);
		//TODO: Optimise all prints in a function or store them in files.
		cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
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
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

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
	update_solution(p, s_local_best, s_prev);
	swap(p, s_prev, s_curr, s_local_best);
	// while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn
	while (s_local_best.total_cost < s_prev.total_cost)
	{ // while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn
		update_solution(p, s_local_best, s_prev);
		swap(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
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
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
		cout << "ILS best " << s_ILS_best.total_cost << "\n";
	}
	/*
	-----------------------
	END: SWAP flow
	-----------------------
	*/
	/*
	-----------------------
	Start: Second Relocate flow
	-----------------------
	*/

	update_solution(p, s_local_best, s_prev);
	relocate(p, s_prev, s_curr, s_local_best);

	cout << "\nNew best solution after first relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
		 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	{
		for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++)
		{
			cout << s_local_best.routes[vehicle_id].route[position] << " ";
		}
		cout << "\n";
	}

	cout << "best cost na first relocate " << s_local_best.total_cost << "\n";

	if (s_local_best.total_cost < s_total_best.total_cost)
	{
		update_solution(p, s_local_best, s_total_best);

		cout << "total best " << s_total_best.total_cost << "\n";
	}

	while (s_local_best.total_cost < s_prev.total_cost)
	{ // while loop op RELOCATE uitvoeren totdat er geen verbeteringen meer zijn

		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
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
	{ // moet dit hier ook telkens, of pas na de laatste relocate?
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
		//cout << "ILS best " << s_ILS_best.total_cost << "\n";
	}
	/*
	-----------------------
	END: Second Relocate flow
	-----------------------
	*/

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

		update_solution(p, s_total_best, s_local_best);

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
			 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
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
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}
		/*
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		END:  Insertion procedure of the removed customers
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		*/

		/*
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		START:  Relocate - Swap - Relocate flows after the insertion of removed customer
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		*/
		update_solution(p, s_local_best, s_prev);
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
		//	update_solution(p, s_local_best, s_total_best);

		//	cout << "total best " << s_total_best.total_cost << "\n";
		//}

		while (s_local_best.total_cost < s_prev.total_cost)
		{ // while loop uitvoeren totdat er geen verbeteringen meer zijn

			update_solution(p, s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
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
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost)
		{
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}

		update_solution(p, s_local_best, s_prev);
		swap(p, s_prev, s_curr, s_local_best);

		while (s_local_best.total_cost < s_prev.total_cost)
		{
			update_solution(p, s_local_best, s_prev);
			swap(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
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
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost)
		{
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}

		update_solution(p, s_local_best, s_prev);
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
		//	update_solution(p, s_local_best, s_total_best);

		//	cout << "total best " << s_total_best.total_cost << "\n";
		//}

		while (s_local_best.total_cost < s_prev.total_cost)
		{ // while loop uitvoeren totdat er geen verbeteringen meer zijn

			update_solution(p, s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				 << " route duration " << s_local_best.total_route_duration << " time window violation " << s_local_best.total_time_window_violation << " overtime " << s_local_best.total_overtime << " allowable operating time " << s_local_best.total_driving_time << " total cost " << s_local_best.total_cost << "\n";
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
		/*
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		END:  Relocate - Swap - Relocate flows after the insertion of removed customer
		-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-*-
		*/

		//FIXME: Is this ever going to be TRUE?! Related to lone 295
		if (s_local_best.total_cost < s_total_best.total_cost)
		{
			update_solution(p, s_local_best, s_total_best);

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
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}
		//Otherwise, increase the number of times without improvments
		else
		{
			number_of_times_without_improvement++;

			cout << "number of times without improvement " << number_of_times_without_improvement << "\n";
		}

		cout << "final ILS best " << s_ILS_best.total_cost << "\n";

		auto stop = chrono::high_resolution_clock::now();

		// Get duration. Substart timepoints to
		// get durarion. To cast it to proper unit
		// use duration cast method
		auto duration = chrono::duration_cast<chrono::minutes>(stop - start);
		cout << "Time taken to run the algorithm is: : " << duration.count() << " minutes" << endl;
		cout << " ============ End of execution ======== (still waiting to write the results) "
			 << "\n";

		write_output_file(p, s_ILS_best);
	}
}
