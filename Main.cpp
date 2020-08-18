#include <iostream>
#include <cstdlib>
#include <string>
#include <vector>
#include <numeric>
#include <time.h>
#include <ctime> 
#include <chrono>

#include "ProbabilityEstimator.h"
#include "Heuristic.h"

using namespace std;

void show(vector<int> const& input) {
	cout << "shuffle ";
	for (auto const& i : input) {
		std::cout << i << " ";
	}
	cout << "\n";
}

int value_no_improvement = 2;

/* Main function to run the algorithm */
int main(int argc, char* argv[]) {

	srand(time(NULL));

	auto start = chrono::system_clock::now();

	struct problem p;
	p.collection_date = "2-Jan-2018"; // set a collection date
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


	vector<int> customers_to_be_inserted = {};

	for (int customer_id = 1; customer_id <= p.n_customers; customer_id++) {

		customers_to_be_inserted.push_back(customer_id);

	}

	//cout << "customers to be inserted " << customers_to_be_inserted.size() << "\n";

	for (int i = 0; i < customers_to_be_inserted.size() - 1; i++) {
		int j = i + rand() % (customers_to_be_inserted.size() - i);
		swap(customers_to_be_inserted[i], customers_to_be_inserted[j]);
	}

	show(customers_to_be_inserted);

	for (int customer_id = 0; customer_id <= customers_to_be_inserted.size() - 1; customer_id++) {

		perform_best_insertion(p, s_curr, customers_to_be_inserted[customer_id]);

	}

	//for (int customer_id = 1; customer_id <= p.n_customers; customer_id++) {
	//	perform_best_insertion(p, s_curr, customer_id);
	//}

	update_solution(p, s_curr, s_local_best);

	cout << "Initial solution with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
		<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
			cout << s_local_best.routes[vehicle_id].route[position] << " ";
		}

		cout << "\n";
	}

	update_solution(p, s_local_best, s_total_best);
	update_solution(p, s_local_best, s_ILS_best);

	cout << "total best " << s_total_best.total_cost << "\n";
	//cout << "ILS best " << s_ILS_best.total_cost << "\n";

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


	while (s_local_best.total_cost < s_prev.total_cost) { // while loop uitvoeren op RELOCATE totdat er geen verbeteringen meer zijn 

		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}
			cout << "\n";
		}

		cout << "best cost na relocate " << s_local_best.total_cost << "\n";
	}

	if (s_local_best.total_cost < s_total_best.total_cost) {
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
		//cout << "ILS best " << s_ILS_best.total_cost << "\n";
	}

	update_solution(p, s_local_best, s_prev);
	swap(p, s_prev, s_curr, s_local_best);

	while (s_local_best.total_cost < s_prev.total_cost) {// while loop uitvoeren op SWAP totdat er geen verbeteringen meer zijn 
		update_solution(p, s_local_best, s_prev);
		swap(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";
		}

		cout << "best cost na swap " << s_local_best.total_cost << "\n";
	}

	if (s_local_best.total_cost < s_total_best.total_cost) {
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
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


	while (s_local_best.total_cost < s_prev.total_cost) { // while loop op RELOCATE uitvoeren totdat er geen verbeteringen meer zijn 

		update_solution(p, s_local_best, s_prev);
		relocate(p, s_prev, s_curr, s_local_best);

		cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}
			cout << "\n";
		}

		cout << "best cost na relocate " << s_local_best.total_cost << "\n";
	}

	if (s_local_best.total_cost < s_total_best.total_cost) { // moet dit hier ook telkens, of pas na de laatste relocate? 
		update_solution(p, s_local_best, s_total_best);
		update_solution(p, s_local_best, s_ILS_best);

		cout << "total best " << s_total_best.total_cost << "\n";
		//cout << "ILS best " << s_ILS_best.total_cost << "\n";
	}


	int number_of_times_without_improvement = 0;

	cout << "value no improvement " << value_no_improvement << "\n";

	//for (int iteration = 0; iteration < iterations; iteration++) {
	while (number_of_times_without_improvement < value_no_improvement) {

		update_solution(p, s_total_best, s_local_best);

		vector<int> random_customers = {};

		while (random_customers.size() < 0.3 * p.n_customers) {

			int customer = rand() % p.n_customers + 1;

			vector<int>::iterator it;

			it = find(random_customers.begin(), random_customers.end(), customer);
			if (it == random_customers.end()) {
				//cout << "curr " << j_curr << "\n";
				random_customers.push_back(customer);
			}

		}

		cout << "verwijderde klanten ";
		for (size_t i = 0; i < random_customers.size(); i++) {
			cout << random_customers[i] << " ";
		}

		cout << "\n";

		s_local_best.route_customer = {};
		s_local_best.position_customer = {};

		for (size_t i = 0; i < random_customers.size(); i++) {
			position_removed_customers(p, s_local_best, random_customers[i]);
			remove_customer(p, s_local_best, s_local_best.route_customer[i], s_local_best.position_customer[i]);
		}



		cout << "route na verwijdering " << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";

		}

		cout << "\n";



		for (size_t customer_id = 0; customer_id < random_customers.size(); customer_id++) { // verwijderde klanten terug invoegen 
			perform_best_insertion(p, s_local_best, random_customers[customer_id]);
		}

		cout << "New initial solution after perturbation with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
			<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
		for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
				cout << s_local_best.routes[vehicle_id].route[position] << " ";
			}

			cout << "\n";

		}

		cout << "\n";

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
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

		while (s_local_best.total_cost < s_prev.total_cost) { // while loop uitvoeren totdat er geen verbeteringen meer zijn 

			update_solution(p, s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}

				cout << "\n";
			}

			cout << "best cost na relocate " << s_local_best.total_cost << "\n";

		}

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost) {
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}

		update_solution(p, s_local_best, s_prev);
		swap(p, s_prev, s_curr, s_local_best);

		while (s_local_best.total_cost < s_prev.total_cost) {
			update_solution(p, s_local_best, s_prev);
			swap(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after swap with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}

				cout << "\n";
			}

			cout << "best cost na swap " << s_local_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost) {
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

		while (s_local_best.total_cost < s_prev.total_cost) { // while loop uitvoeren totdat er geen verbeteringen meer zijn 

			update_solution(p, s_local_best, s_prev);
			relocate(p, s_prev, s_curr, s_local_best);

			cout << "\nNew best solution after relocate with " << s_local_best.number_of_vehicles_used << " vehicles and distance " << s_local_best.total_distance_cost
				<< " route duration " << s_local_best.total_route_duration << " total cost " << s_local_best.total_cost << "\n";
			for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
				for (size_t position = 0; position < s_local_best.routes[vehicle_id].route.size(); position++) {
					cout << s_local_best.routes[vehicle_id].route[position] << " ";
				}

				cout << "\n";
			}

			cout << "best cost na relocate " << s_local_best.total_cost << "\n";

		}

		if (s_local_best.total_cost < s_total_best.total_cost) {
			update_solution(p, s_local_best, s_total_best);

			cout << "total best " << s_total_best.total_cost << "\n";
		}

		if (s_local_best.total_cost < s_ILS_best.total_cost) {
			update_solution(p, s_local_best, s_ILS_best);

			cout << "ILS best " << s_ILS_best.total_cost << "\n";
		}

		else {
			number_of_times_without_improvement++;

			cout << "number of times without improvement " << number_of_times_without_improvement << "\n";
		}


		auto end = std::chrono::system_clock::now();

		std::chrono::duration<double> elapsed_seconds = end - start;
		std::time_t end_time = std::chrono::system_clock::to_time_t(end);

		//cout << "finished computation at " << ctime(&end_time) << "\n";
		//cout << "elapsed time: " << elapsed_seconds.count() << "s\n";

		write_output_file(p, s_ILS_best);
	}
	


}