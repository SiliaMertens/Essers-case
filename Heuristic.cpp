// BASIC HEURISTIC FOR THE CAPACITATED VEHICLE ROUTING PROBLEM WITH TIME WINDOWS (CVRPTW) //

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cfloat>
#include <iterator>
#include <numeric>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <random>
#include <string>
#include <time.h>
#include <vector>
#include <ctime> 

#include "ProbabilityEstimator.h"
#include "Heuristic.h"

using namespace std;


/* Input file */
string data_file = "General_Cargo_LTL_2018_v10072019_input_code adjusted tw.txt";  
string coordinates_file = "distance_matrix 2 jan.txt";

void read_data(problem& p) {

	ifstream infile;
    cout << "Reading data_file '" << data_file << "'\n";
	infile.open(data_file);
    if (!infile.is_open())
        cerr << "Error, failed to open data_file '" << data_file << "'\n";

	string line, collection_date, unit_description, delivery_date;
	long long int order_nr;
	int high_sec_indicator, temp_indicator, ADR_indicator;
	string country;
	int relation_nr, postal_code;
	string street, town;
	int lower_tw, upper_tw;
	string flow_of_goods, transport_type;
	double demand;
	int service_dur, index_count = 1;
	bool is_collection_date_visited = false;

	p.n_customers = 0;
	p.n_lines = 61072; // number of lines in the text file
	p.nodes = new node[p.n_lines];

	infile >> p.n_vehicles >> p.vehicle_cap >> p.max_operating_time >> p.max_route_duration;

	cout << "vehicles " << p.n_vehicles << "\n";

	infile >> p.nodes[0].depot_country // depot information
		>> p.nodes[0].depot_postal_code >> p.nodes[0].depot_street >> p.nodes[0].depot_town >> p.nodes[0].order_nr >> p.nodes[0].demand >> p.nodes[0].lower_tw >> p.nodes[0].upper_tw >> p.nodes[0].service_dur;

	//cout << "depot country " << p.nodes[0].depot_country << " " << p.nodes[0].depot_street << " " << p.nodes[0].demand << "\n";
	//Get a full line until the line-break "\n"
	while (getline(infile, line))
	{
		//Make a stream of strings from the line.
		stringstream sstrm(line);
		//Get the collection_date.
		sstrm >> collection_date;
		if (collection_date == p.collection_date)
		{
			try
			{
				//Get the rest of the data.
				sstrm >> unit_description >> delivery_date >> order_nr >> high_sec_indicator >> temp_indicator >> ADR_indicator >> relation_nr >> country >> postal_code >> street >> town >> lower_tw >> upper_tw >> flow_of_goods >> transport_type >> demand >> service_dur;

				// here the collection date is selected which you will use to run the algorithm. The variables are put in the struct.
				p.nodes[index_count].collection_date = collection_date;
				p.nodes[index_count].unit_description = unit_description;
				p.nodes[index_count].delivery_date = delivery_date;
				p.nodes[index_count].order_nr = order_nr;
				p.nodes[index_count].high_security_indicator = high_sec_indicator;
				p.nodes[index_count].temperature_controlled_indicator = temp_indicator;
				p.nodes[index_count].ADR_indicator = ADR_indicator;
				p.nodes[index_count].pick_up_relation_nr = relation_nr;
				p.nodes[index_count].pick_up_country = country;
				p.nodes[index_count].pick_up_postal_code = postal_code;
				p.nodes[index_count].pick_up_street = street;
				p.nodes[index_count].pick_up_town = town;
				p.nodes[index_count].lower_tw = lower_tw;
				p.nodes[index_count].upper_tw = upper_tw;
				p.nodes[index_count].flow_of_goods = flow_of_goods;
				p.nodes[index_count].transport_type = transport_type;
				p.nodes[index_count].demand = demand;
				p.nodes[index_count].service_dur = service_dur;

				//Temprary counter for the index. Hence, p.n_customers can also be used (i.e. they are both the same increment).

				//cout << "collectiondate " << p.nodes[index_count].collection_date << " ordernr " << p.nodes[index_count].order_nr << " lower tw " << p.nodes[index_count].lower_tw << "upper tw" << p.nodes[index_count].upper_tw << " demand " << p.nodes[index_count].demand << "\n";

				index_count++;
				// To break later if other day is reached.
				is_collection_date_visited = true;
				p.n_customers++; // the number of customers that are present in one day is calculated here
			}
			catch (const std::exception& e)
			{
				throw std::runtime_error("read_data() --> Error at line " + std::to_string(index_count + 1) + " --> Error is: " + e.what());
			}
		}
		else if (is_collection_date_visited)
		{
			//This is in case the collection date has been reached and processed
			//The data will be already collected in previous iterations
			//Therefore we break the rest of the loop as it is not needed.
			break;
		}
		else
		{
			//Collection date is not equal to the current_line_collection_date
			//The collection_date is still not reached, therefore continue next line.
			continue;
		}
	}
	//cout << "customers " << p.n_customers << "\n";
	p.n_nodes = p.n_customers + 1; // number of customers + depot

	cout << "customers " << p.n_customers << "\n";
	cout << "nodes " << p.n_nodes << "\n";

	infile.close();
}

void read_distance_and_time_matrix(struct problem& p) { // dit moet via Graphhopper, afstanden die daar berekend zijn, aanroepen. Bestand inlezen, berekeningen met x-coord en y-coord moet niet meer gedaan worden. 

	p.distance_matrix = new double[(long long)p.n_nodes * p.n_nodes];
	p.time_matrix = new double[(long long)p.n_nodes * p.n_nodes];
	// waarschuwing Warning C26451: Arithmetic overflow: Using operator '%operator%' on a %size1% byte value and then casting the result to a %size2% byte value. Cast the value to the wider type before calling operator '%operator%' to avoid overflow
	// door long long valt deze waarschuwing weg 

	ifstream infile;
	infile.open(coordinates_file);

	for (int i = 0; i < p.n_nodes * p.n_nodes; i++) {
		int a = 0;
		int b = 0;
		infile >> a;
		infile >> b;
		infile >> p.distance_matrix[a * p.n_nodes + b];
		infile >> p.time_matrix[a * p.n_nodes + b];

		//cout << a << " " << b << " distance " << p.distance_matrix[a * p.n_nodes + b] << " time " << p.time_matrix[a * p.n_nodes + b] << "\n";

	}

}

void initialize_solution(struct problem& p, struct solution& s) {

	s.total_distance_cost = 0.0;
	s.number_of_vehicles_used = 0;
	s.total_route_duration = 0.0;
	s.total_cost = 0.0;
	s.route_customer = {};
	s.position_customer = {};

	s.routes = new route[p.n_vehicles];

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		s.routes[vehicle_id].route = { 0, 0 };
		s.routes[vehicle_id].load = { 0, 0 };
		s.routes[vehicle_id].earliest_time = { p.nodes[0].lower_tw, p.nodes[0].lower_tw };
		s.routes[vehicle_id].latest_time = { p.nodes[p.n_nodes - 1].upper_tw, p.nodes[p.n_nodes - 1].upper_tw };
		s.routes[vehicle_id].schedule = { s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1], s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1] };
		s.routes[vehicle_id].distance_cost = 0.0;
		s.routes[vehicle_id].route_used = 0;
		s.routes[vehicle_id].route_duration = 0.0;
		s.routes[vehicle_id].time_window_violiation = 0.0;
		s.routes[vehicle_id].overtime = 0.0;
		s.routes[vehicle_id].driving_time = 0.0;
		s.routes[vehicle_id].route_cost = 0.0;
		s.routes[vehicle_id].weighted_route_cost = 0.0;
		s.routes[vehicle_id].weighted_distance_cost = 0.0;
		s.routes[vehicle_id].weighted_route_duration = 0.0;
		s.routes[vehicle_id].departure_time = 0.0;
	}
}

void update_solution(struct problem& p, struct solution& s1, struct solution& s2) {

	s2.total_distance_cost = s1.total_distance_cost;
	s2.number_of_vehicles_used = s1.number_of_vehicles_used;
	s2.total_route_duration = s1.total_route_duration;
	s2.total_cost = s1.total_cost;

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		s2.routes[vehicle_id].route = s1.routes[vehicle_id].route;
		s2.routes[vehicle_id].load = s1.routes[vehicle_id].load;
		s2.routes[vehicle_id].earliest_time = s1.routes[vehicle_id].earliest_time;
		s2.routes[vehicle_id].latest_time = s1.routes[vehicle_id].latest_time;
		s2.routes[vehicle_id].schedule = s1.routes[vehicle_id].schedule;
		s2.routes[vehicle_id].distance_cost = s1.routes[vehicle_id].distance_cost;
		s2.routes[vehicle_id].route_used = s1.routes[vehicle_id].route_used;
		s2.routes[vehicle_id].route_duration = s1.routes[vehicle_id].route_duration;
		s2.routes[vehicle_id].time_window_violiation = s1.routes[vehicle_id].time_window_violiation;
		s2.routes[vehicle_id].overtime = s1.routes[vehicle_id].overtime;
		s2.routes[vehicle_id].driving_time = s1.routes[vehicle_id].driving_time;
		s2.routes[vehicle_id].route_cost = s1.routes[vehicle_id].route_cost;
		s2.routes[vehicle_id].weighted_route_cost = s1.routes[vehicle_id].weighted_route_cost;
		s2.routes[vehicle_id].weighted_route_cost = s1.routes[vehicle_id].weighted_route_cost;
		s2.routes[vehicle_id].weighted_distance_cost = s1.routes[vehicle_id].weighted_distance_cost;
		s2.routes[vehicle_id].weighted_route_duration = s1.routes[vehicle_id].weighted_route_duration;
		s2.routes[vehicle_id].departure_time = s1.routes[vehicle_id].departure_time;
	}
}

void change_update_solution_1(problem& p, solution& s1, solution& s2, int vehicle1) {

	s2.routes[vehicle1].route = s1.routes[vehicle1].route;
	s2.routes[vehicle1].load = s1.routes[vehicle1].load;
	s2.routes[vehicle1].earliest_time = s1.routes[vehicle1].earliest_time;
	s2.routes[vehicle1].latest_time = s1.routes[vehicle1].latest_time;
	s2.routes[vehicle1].schedule = s1.routes[vehicle1].schedule;
	s2.routes[vehicle1].distance_cost = s1.routes[vehicle1].distance_cost;
	s2.routes[vehicle1].route_used = s1.routes[vehicle1].route_used;
	s2.routes[vehicle1].route_duration = s1.routes[vehicle1].route_duration;
	s2.routes[vehicle1].time_window_violiation = s1.routes[vehicle1].time_window_violiation;
	s2.routes[vehicle1].overtime = s1.routes[vehicle1].overtime;
	s2.routes[vehicle1].driving_time = s1.routes[vehicle1].driving_time;
	s2.routes[vehicle1].route_cost = s1.routes[vehicle1].route_cost;
	s2.routes[vehicle1].weighted_route_cost = s1.routes[vehicle1].weighted_route_cost;
	s2.routes[vehicle1].weighted_route_cost = s1.routes[vehicle1].weighted_route_cost;
	s2.routes[vehicle1].weighted_distance_cost = s1.routes[vehicle1].weighted_distance_cost;
	s2.routes[vehicle1].weighted_route_duration = s1.routes[vehicle1].weighted_route_duration;
	s2.routes[vehicle1].departure_time = s1.routes[vehicle1].departure_time;


}

void change_update_solution_2(problem& p, solution& s1, int vehicle1, int vehicle2) {
	s1.routes[vehicle2].route = s1.routes[vehicle1].route;
	s1.routes[vehicle2].load = s1.routes[vehicle1].load;
	s1.routes[vehicle2].earliest_time = s1.routes[vehicle1].earliest_time;
	s1.routes[vehicle2].latest_time = s1.routes[vehicle1].latest_time;
	s1.routes[vehicle2].schedule = s1.routes[vehicle1].schedule;
	s1.routes[vehicle2].distance_cost = s1.routes[vehicle1].distance_cost;
	s1.routes[vehicle2].route_used = s1.routes[vehicle1].route_used;
	s1.routes[vehicle2].route_duration = s1.routes[vehicle1].route_duration;
	s1.routes[vehicle2].time_window_violiation = s1.routes[vehicle1].time_window_violiation;
	s1.routes[vehicle2].overtime = s1.routes[vehicle1].overtime;
	s1.routes[vehicle2].driving_time = s1.routes[vehicle1].driving_time;
	s1.routes[vehicle2].route_cost = s1.routes[vehicle1].route_cost;
	s1.routes[vehicle2].weighted_route_cost = s1.routes[vehicle1].weighted_route_cost;
	s1.routes[vehicle2].weighted_route_cost = s1.routes[vehicle1].weighted_route_cost;
	s1.routes[vehicle2].weighted_distance_cost = s1.routes[vehicle1].weighted_distance_cost;
	s1.routes[vehicle2].weighted_route_duration = s1.routes[vehicle1].weighted_route_duration;
	s1.routes[vehicle2].departure_time = s1.routes[vehicle1].departure_time;
}

void change_update_solution_3(problem& p, solution& s1, solution& s2, int vehicle1, int vehicle2) {
	s1.routes[vehicle2].route = s2.routes[vehicle1].route;
	s1.routes[vehicle2].load = s2.routes[vehicle1].load;
	s1.routes[vehicle2].earliest_time = s2.routes[vehicle1].earliest_time;
	s1.routes[vehicle2].latest_time = s2.routes[vehicle1].latest_time;
	s1.routes[vehicle2].schedule = s2.routes[vehicle1].schedule;
	s1.routes[vehicle2].distance_cost = s2.routes[vehicle1].distance_cost;
	s1.routes[vehicle2].route_used = s2.routes[vehicle1].route_used;
	s1.routes[vehicle2].route_duration = s2.routes[vehicle1].route_duration;
	s1.routes[vehicle2].time_window_violiation = s2.routes[vehicle1].time_window_violiation;
	s1.routes[vehicle2].overtime = s2.routes[vehicle1].overtime;
	s1.routes[vehicle2].driving_time = s2.routes[vehicle1].driving_time;
	s1.routes[vehicle2].route_cost = s2.routes[vehicle1].route_cost;
	s1.routes[vehicle2].weighted_route_cost = s2.routes[vehicle1].weighted_route_cost;
	s1.routes[vehicle2].weighted_route_cost = s2.routes[vehicle1].weighted_route_cost;
	s1.routes[vehicle2].weighted_distance_cost = s2.routes[vehicle1].weighted_distance_cost;
	s1.routes[vehicle2].weighted_route_duration = s2.routes[vehicle1].weighted_route_duration;
	s1.routes[vehicle2].departure_time = s2.routes[vehicle1].departure_time;
}

vector<int> position_removed_customers(problem& p, solution& s, int customer_id) {

	//cout << "customer " << customer_id << "\n";

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
			if (customer_id == s.routes[vehicle_id].route[position]) {
				int route = vehicle_id;
				int route_position = position;

				s.route_customer.push_back(route);
				s.position_customer.push_back(position);


			}
		}
	}

	//for (int i = 0; i < s.route_customer.size(); i++) {
	//	cout << "route " << s.route_customer[i] << " ";
	//}

	//for (int i = 0; i < s.position_customer.size(); i++) {
	//	cout << "position " << s.position_customer[i] << "\n";
	//}

	return s.route_customer;
	return s.position_customer;

}

void bereken_route_cost_zonder_recourse(problem& p, solution& s, int vehicle_id) {

	s.routes[vehicle_id].route_used = 0;
	s.routes[vehicle_id].route_cost = 0;

	//for (size_t i = 0; i < s.routes[vehicle_id].route.size(); i++) {
	//	cout << s.routes[vehicle_id].route[i] << " ";
	//}

	//cout << "\n";

	if (s.routes[vehicle_id].route.size() == 2) {
		s.routes[vehicle_id].route_cost = 0.0;
		s.routes[vehicle_id].route_used = 0;
	}

	else {
		s.routes[vehicle_id].route_cost = fixed_vehicle_cost;
		s.routes[vehicle_id].route_used = 1;
	}

	s.routes[vehicle_id].distance_cost = 0.0;

	for (size_t i = 0; i < s.routes[vehicle_id].route.size() - 1; i++) {
		s.routes[vehicle_id].distance_cost += p.distance_matrix[s.routes[vehicle_id].route[i] * p.n_nodes + s.routes[vehicle_id].route[i + 1]];

		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].distance_cost /** km_cost*/;
	}

	//if (s.routes[vehicle_id].distance_cost > p.max_operating_time) {
	//	s.routes[vehicle_id].driving_time = s.routes[vehicle_id].distance_cost - p.max_operating_time;
	//	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].driving_time;
	//}

	//cout << "distance " << s.routes[vehicle_id].distance_cost << "\n";

	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen 

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--) {
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[(long long)position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
			p.time_matrix[s.routes[vehicle_id].route[position] * p.n_nodes + s.routes[vehicle_id].route[(long long)position + 1]];
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw) {
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}

	s.routes[vehicle_id].departure_time = s.routes[vehicle_id].schedule[0];

	//cout << "departure time " << s.routes[vehicle_id].departure_time << "\n";

	s.routes[vehicle_id].route_duration = s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0];

	//cout << "route duration " << s.routes[vehicle_id].route_duration << "\n";

	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].route_duration /** driver_cost*/;

	//for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
	//	if (s.routes[vehicle_id].earliest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw) {
	//		s.routes[vehicle_id].time_window_violiation += s.routes[vehicle_id].earliest_time[position] - p.nodes[s.routes[vehicle_id].route[position]].upper_tw;

	//		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].time_window_violiation;
	//	}
	//}

	/*cout << "time window violation " << s.routes[vehicle_id].time_window_violiation << "\n";



	if (s.routes[vehicle_id].route_duration > p.max_route_duration) {
		s.routes[vehicle_id].overtime = s.routes[vehicle_id].route_duration - p.max_route_duration;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].overtime /** overtime_cost*/;
	//}

	/*cout << "overtime " << s.routes[vehicle_id].overtime << "\n";*/

}

void bereken_route_cost(problem& p, solution& s, int vehicle_id) {

	s.routes[vehicle_id].route_used = 0;
	s.routes[vehicle_id].route_cost = 0;

	//for (size_t i = 0; i < s.routes[vehicle_id].route.size(); i++) {
	//	cout << s.routes[vehicle_id].route[i] << " ";
	//}

	//cout << "\n";

	if (s.routes[vehicle_id].route.size() == 2) {
		s.routes[vehicle_id].route_cost = 0.0;
		s.routes[vehicle_id].route_used = 0;
	}

	else {
		s.routes[vehicle_id].route_cost = fixed_vehicle_cost;
		s.routes[vehicle_id].route_used = 1;
	}

	s.routes[vehicle_id].distance_cost = 0.0;

	for (size_t i = 0; i < s.routes[vehicle_id].route.size() - 1; i++) {
		s.routes[vehicle_id].distance_cost += p.distance_matrix[s.routes[vehicle_id].route[i] * p.n_nodes + s.routes[vehicle_id].route[i + 1]];
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].distance_cost /** km_cost*/;
	}

	if (s.routes[vehicle_id].distance_cost > p.max_operating_time) {
		s.routes[vehicle_id].driving_time = s.routes[vehicle_id].distance_cost - p.max_operating_time;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].driving_time; 
	}

	//cout << "distance " << s.routes[vehicle_id].distance_cost << "\n";

	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen 

	s.routes[vehicle_id].schedule[0] = s.routes[vehicle_id].departure_time;

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[(long long)position - 1] + p.nodes[s.routes[vehicle_id].route[(long long)position - 1]].service_dur +
			p.time_matrix[s.routes[vehicle_id].route[(long long)position - 1] * p.n_nodes + s.routes[vehicle_id].route[position]];
		if (s.routes[vehicle_id].schedule[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw) {
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].lower_tw;
		}
	}


	s.routes[vehicle_id].route_duration = s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0];

	/*cout << "route duration " << s.routes[vehicle_id].route_duration << "\n";*/

	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].route_duration /** driver_cost*/;

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
		if (s.routes[vehicle_id].earliest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw) {
			s.routes[vehicle_id].time_window_violiation += s.routes[vehicle_id].earliest_time[position] - p.nodes[s.routes[vehicle_id].route[position]].upper_tw;

			s.routes[vehicle_id].route_cost += s.routes[vehicle_id].time_window_violiation;
		}
	}

	//cout << "time window violation " << s.routes[vehicle_id].time_window_violiation << "\n";

	if (s.routes[vehicle_id].route_duration > p.max_route_duration) {
		s.routes[vehicle_id].overtime = s.routes[vehicle_id].route_duration - p.max_route_duration;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].overtime; /** overtime_cost*/;

	}

}

vector<double> calculate_probabilities(problem& p, solution& s, int vehicle_id)
{ // the probabilities that were put in the two vectors are used here as input. Furthermore, a specific route (with a certain vehicle id) is used as input as well.  

	s.routes[vehicle_id].probability = {}; // initially, there are no probabilities in the vector

	vector<double> failure = probability_of_failure(p, s, vehicle_id); // the vector probability of failure in the previous function is used here.

	// For example: route 0 1 2 3 0, for this route there are different scenarios where a failure can occur, each with a corresponding probability which is calculated here.

	double probability_of_failure_in_route = 1.00;
	double probability_of_no_failure_in_route = 1.00;

	for (size_t i = 1; i < s.routes[vehicle_id].route.size() - 1; i++)
	{

		probability_of_no_failure_in_route *= (1 - failure[i]); // this represents the probability of having no failure in the route
																// in the example: 1 - 0 (probability of failure at customer 1) - 0.05 (probability of failure at customer 2) - 0.095 (probability of failure at customer 3) = 0.085
	}

	s.routes[vehicle_id].probability.push_back(probability_of_no_failure_in_route);

	// the probability of having no failure is put here in the first element of the vector

	for (size_t i = 1; i < s.routes[vehicle_id].route.size() - 1; i++)
	{

		probability_of_failure_in_route = (1 - failure[i - 1]) * probability_of_failure_in_route;
		s.routes[vehicle_id].probability.push_back(failure[i] * probability_of_failure_in_route);
		// here the probabilities of failure at every customer are put in the next elements of the vector. It is assumed that only one failure can occur in a route.
		// Therefore, the probability of failure at the second customer is calculated by probability of failure at customer 2 times the probability of having no failure at customer 1.
		// in the example: probability of failure at customer 1 = 0
		// probability of failure at customer 2 = 0.05 * 1 (no failure at customer 1)
		// probability of failure at customer 3 = 0.10 * 0.95 (no failure at customer 1 and 2)
	}

	//for (size_t i = 0; i < s.routes[vehicle_id].probability.size(); i++) {
	//	cout << "prob " << s.routes[vehicle_id].probability[i] << "\n";
	//}

	return s.routes[vehicle_id].probability;

	// the probabilities calculated in this function are used in a further stage to calculate the route costs based on the different scenarios that can happen
	// i.e. : no failure: 0 1 2 3 0 * corresponding probability, failure at second customer: 0 1 2 0 2 3 0 * corresponding probability, ...
}

void construct_failure_routes(problem& p, solution& s1, solution& s2, int vehicle_id, int position) {

	update_solution(p, s1, s2);

	insert_customer(p, s2, 0, vehicle_id, position + 1); // terug naar depot, invoegen in de route
	insert_customer(p, s2, s1.routes[vehicle_id].route[position], vehicle_id, position + 2);

}

void bereken_gewogen_route_cost(problem& p, solution& s1, solution s2, int vehicle_id) { // here, the weighted cost for one vehicle is calculated. As already stated, there are different possibilities where failures can occur in a route. All the different possibilities with their corresponding probabilities are combined here, resulting in a weighted cost

	update_solution(p, s1, s2);
	vector<double> violation_risk = {};
	s1.routes[vehicle_id].weighted_route_cost = 0.0;
	s1.routes[vehicle_id].weighted_distance_cost = 0.0;
	s1.routes[vehicle_id].weighted_route_duration = 0.0;
	s1.routes[vehicle_id].route_used = 0;
	violation_risk = calculate_probabilities(p, s1, vehicle_id);

	//cout << "violation risk ";
	//for (size_t i = 0; i < s1.routes[vehicle_id].probability.size(); i++) {

	//	cout << violation_risk[i] << " ";
	//}

	//cout << "\n";

	//cout << "size prob " << s1.routes[vehicle_id].probability.size() << "\n";

	bereken_route_cost_zonder_recourse(p, s1, vehicle_id); // the routecost when there is no failure is calculated here 
	s1.routes[vehicle_id].weighted_route_cost += s1.routes[vehicle_id].route_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_distance_cost += s1.routes[vehicle_id].distance_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_route_duration += s1.routes[vehicle_id].route_duration * violation_risk[0];

	//cout << "route cost " << s1.routes[vehicle_id].route_cost << "\n";

	for (size_t index = 0; index < s1.routes[vehicle_id].probability.size() - 1; index++) {
		construct_failure_routes(p, s1, s2, vehicle_id, index);

		bereken_route_cost(p, s2, vehicle_id);

		s1.routes[vehicle_id].weighted_route_cost += s2.routes[vehicle_id].route_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_distance_cost += s2.routes[vehicle_id].distance_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_route_duration += s2.routes[vehicle_id].route_duration * violation_risk[index + 1];
		//cout << "route cost (1) " << s2.routes[vehicle_id].route_cost << "\n";
	}
	//cout << "\n";

	//cout << "totale gewogen kost vehicle: " << vehicle_id << " kost " << s1.routes[vehicle_id].weighted_route_cost << "\n";
	//cout << "\n";
}

void calculate_total_cost(problem& p, solution& s) {

	s.total_distance_cost = 0.0;
	s.total_cost = 0.0;
	s.number_of_vehicles_used = 0;
	s.total_route_duration = 0.0;


	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		s.total_distance_cost += s.routes[vehicle_id].weighted_distance_cost;
		s.total_cost += s.routes[vehicle_id].weighted_route_cost;
		s.number_of_vehicles_used += s.routes[vehicle_id].route_used;
		s.total_route_duration += s.routes[vehicle_id].weighted_route_duration;

	}

	//cout << "totale afstand: " << s.total_distance_cost << "\n";
	//cout << "totale routeduur: " << s.total_route_duration << "\n";
	//cout << "totale kost: " << s.total_cost << "\n";
	//cout << "vehicles used " << s.number_of_vehicles_used << "\n";
}

void change(problem& p, struct solution& s, int vehicle1, int vehicle2) {

	struct solution s_try;
	initialize_solution(p, s_try);

	change_update_solution_1(p, s, s_try, vehicle1);

	change_update_solution_2(p, s, vehicle2, vehicle1);

	change_update_solution_3(p, s, s_try, vehicle1, vehicle2);
	
	delete[] s_try.routes;
}

int last_route(problem& p, solution& s) {

	int last_vehicle = 0;

	for (int vehicle_id = p.n_vehicles - 1; vehicle_id >= 0; vehicle_id--) {
		if (s.routes[vehicle_id].route.size() > 2) {
			last_vehicle = vehicle_id;

			return last_vehicle;
		}
	}
}

void relocate(struct problem& p, struct solution& s_prev, struct solution& s_curr, struct solution& s_best) {

	struct solution s_recourse;
	initialize_solution(p, s_recourse);

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) { // over alle routes loopen
		for (size_t position = 1; position < s_prev.routes[vehicle_id].route.size() - 1; position++) { // over alle klantposities loopen
			/*cout << "vehicle " << vehicle_id << " prev route size " << s_prev.routes[vehicle_id].route.size() << "\n";*/
			int customer_id = s_prev.routes[vehicle_id].route[position];
			/*cout << "customer_id " << customer_id << "\n";*/
			update_solution(p, s_prev, s_curr);
			remove_customer(p, s_curr, vehicle_id, position);
			update_solution(p, s_curr, s_recourse);
			bereken_gewogen_route_cost(p, s_curr, s_recourse, vehicle_id);
			calculate_total_cost(p, s_curr);

			update_earliest_time(p, s_curr, vehicle_id);
			update_latest_time(p, s_curr, vehicle_id);
			update_schedule(p, s_curr, vehicle_id);

			if (s_curr.routes[vehicle_id].route.size() == 2) {
				change(p, s_curr, vehicle_id, last_route(p, s_curr));
			}

			perform_best_insertion(p, s_curr, customer_id);
			//cout << "customer_id insertion " << customer_id << "\n";

			if (s_best.total_cost > s_curr.total_cost) {
				update_solution(p, s_curr, s_best);
				update_load(p, s_best, vehicle_id);
				update_earliest_time(p, s_best, vehicle_id);
				update_latest_time(p, s_best, vehicle_id);
				update_schedule(p, s_best, vehicle_id);

			}
		}
	}
	
	delete[] s_recourse.routes;
}

void swap(struct problem& p, struct solution& s1, struct solution& s2, struct solution& s3) {

	//cout << "best cost " << s3.total_cost << "\n";

	struct solution s_recourse;
	initialize_solution(p, s_recourse);

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) { // over alle routes loopen
		for (size_t position = 1; position < s1.routes[vehicle_id].route.size() - 1; position++) { // over alle klantposities loopen

			for (int insert_vehicle_id = 0; insert_vehicle_id < p.n_vehicles; insert_vehicle_id++) {
				for (size_t insert_position = 1; insert_position < s1.routes[insert_vehicle_id].route.size() - 1; insert_position++) {

					if (insert_vehicle_id != vehicle_id) {

						int customer_id = s1.routes[vehicle_id].route[position];
						update_solution(p, s1, s2);
						remove_customer(p, s2, vehicle_id, position); // klant uit de route halen 
						update_solution(p, s2, s_recourse);
						bereken_gewogen_route_cost(p, s2, s_recourse, vehicle_id);
						calculate_total_cost(p, s2);
						update_earliest_time(p, s2, vehicle_id);
						update_latest_time(p, s2, vehicle_id);
						update_schedule(p, s2, vehicle_id);

						//cout << "removed customer vehicle " << vehicle_id << " position " << position << " customer " << customer_id << "\n";

						//cout << "after first remove \n";
						//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
						//	for (size_t position = 0; position < s2.routes[vehicle_id].route.size(); position++) {
						//		cout << s2.routes[vehicle_id].route[position] << " ";
						//	}

						//	cout << "\n";

						//}

						//cout << "\n";

						int removed_customer = s2.routes[insert_vehicle_id].route[insert_position];
						remove_customer(p, s2, insert_vehicle_id, insert_position);
						update_solution(p, s2, s_recourse);
						bereken_gewogen_route_cost(p, s2, s_recourse, insert_vehicle_id);
						calculate_total_cost(p, s2);
						update_earliest_time(p, s2, insert_vehicle_id);
						update_latest_time(p, s2, insert_vehicle_id);
						update_schedule(p, s2, insert_vehicle_id);

						//cout << "second removed customer vehicle " << insert_vehicle_id << " position " << insert_position << " customer " << removed_customer << "\n";

						insert_customer(p, s2, customer_id, insert_vehicle_id, insert_position);

						int predecessor_id = s2.routes[insert_vehicle_id].route[(long long)insert_position - 1];
						int successor_id = s2.routes[insert_vehicle_id].route[insert_position];

						if (s2.routes[insert_vehicle_id].earliest_time[insert_position - 1] + p.nodes[s2.routes[insert_vehicle_id].route[insert_position - 1]].service_dur +
							p.time_matrix[predecessor_id * p.n_nodes + customer_id] <= p.nodes[customer_id].upper_tw &&
							s2.routes[insert_vehicle_id].latest_time[insert_position] - p.nodes[customer_id].service_dur -
							p.time_matrix[customer_id * p.n_nodes + successor_id] >= p.nodes[customer_id].lower_tw) {
							if (check_schedule(p, s2, insert_vehicle_id) == true) {
								if (check_load(p, s2, insert_vehicle_id) == true) {

									update_solution(p, s2, s_recourse);

									bereken_gewogen_route_cost(p, s2, s_recourse, insert_vehicle_id);
									calculate_total_cost(p, s2);

									//cout << "inserted customer " << customer_id << " vehicle " << insert_vehicle_id << " position " << insert_position << " cost " << s2.total_cost << "\n";

									//cout << "solution voor perform best insertion for swap \n";
									//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
									//	for (size_t position = 0; position < s2.routes[vehicle_id].route.size(); position++) {
									//		cout << s2.routes[vehicle_id].route[position] << " ";
									//	}

									//	cout << "\n";

									//}

									//cout << "\n";

									perform_best_insertion_for_swap(p, s2, removed_customer, vehicle_id);

									if (s2.possible_insertion == 1) {

										if (s3.total_cost > s2.total_cost) {
											update_solution(p, s2, s3);
											update_load(p, s3, vehicle_id);
											update_earliest_time(p, s3, vehicle_id);
											update_latest_time(p, s3, vehicle_id);
											update_schedule(p, s3, vehicle_id);

											//cout << "best solution \n";
											//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
											//	for (size_t position = 0; position < s3.routes[vehicle_id].route.size(); position++) {
											//		cout << s3.routes[vehicle_id].route[position] << " ";
											//	}

											//	cout << "\n";

											//}

											//cout << "\n";
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
	delete[] s_recourse.routes;
}

void remove_customer(struct problem& p, struct solution& s, int vehicle_id, int position) {

	int customer_id = s.routes[vehicle_id].route[position];

	s.routes[vehicle_id].route.erase(s.routes[vehicle_id].route.begin() + position);

	update_load(p, s, vehicle_id);
	update_earliest_time(p, s, vehicle_id);
	update_latest_time(p, s, vehicle_id);
	update_schedule(p, s, vehicle_id);

}

void insert_customer(struct problem& p, struct solution& s, int customer_id, int vehicle_id, int position) {

	//cout << "customer id " << customer_id << " position " << position << "\n";
	s.routes[vehicle_id].route.insert(s.routes[vehicle_id].route.begin() + position, customer_id);

	update_load(p, s, vehicle_id);
	update_earliest_time(p, s, vehicle_id);
	update_latest_time(p, s, vehicle_id);
	update_schedule(p, s, vehicle_id);

}

void perform_best_insertion_for_swap(struct problem& p, struct solution& s, int customer_id, int vehicle_id) {

	s.possible_insertion = 1;
	double best_cost = DBL_MAX;
	int best_vehicle_id = -1;
	int best_position = -1;
	struct solution s_try;
	initialize_solution(p, s_try);
	update_solution(p, s, s_try);

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
			s_try.routes[vehicle_id].route[position] = s.routes[vehicle_id].route[position];
			s_try.routes[vehicle_id].route.resize(s.routes[vehicle_id].route.size());
		}
	}

	int check = 0;

	struct solution s_recourse;
	initialize_solution(p, s_recourse);

	vector<int> customers_to_be_inserted = {};

	int count_no_insertion = 0;

	/* Check all insertion positions in all vehicles */
	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {

		int predecessor_id = s.routes[vehicle_id].route[(long long)position - 1];
		int successor_id = s.routes[vehicle_id].route[position];


		if (check == 0) {

			// lijst bijhouden van klanten die ingevoegd moeten worden, dit random door elkaar gooien zodat de klanten om een random manier ingevoegd worden 
			insert_customer(p, s_try, customer_id, vehicle_id, position);

			if (s_try.routes[vehicle_id].earliest_time[position - 1] + p.nodes[s_try.routes[vehicle_id].route[position - 1]].service_dur +
				p.time_matrix[predecessor_id * p.n_nodes + customer_id] <= p.nodes[customer_id].upper_tw &&
				s_try.routes[vehicle_id].latest_time[position] - p.nodes[customer_id].service_dur -
				p.time_matrix[customer_id * p.n_nodes + successor_id] >= p.nodes[s_try.routes[vehicle_id].route[position + 1]].lower_tw) {
				if (check_schedule(p, s_try, vehicle_id) == true) {
					if (check_load(p, s_try, vehicle_id) == true) {

						update_solution(p, s_try, s_recourse);

						bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
						calculate_total_cost(p, s_try);

						//cout << "inserted customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost <<  "\n"; 

						if (s_try.total_cost < best_cost) {

							best_cost = s_try.total_cost;
							best_position = position;

							//cout << "klant " << customer_id << " position " << best_position << " cost " << best_cost << "\n";

						}
					}
					else {
						count_no_insertion++;
						//cout << "count no insertion " << count_no_insertion << "\n";
					}
				}
			}

			//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
			//	for (size_t position = 0; position < s_try.routes[vehicle_id].route.size(); position++) {
			//		cout << s_try.routes[vehicle_id].route[position] << " ";
			//	}

			//	cout << "\n";
			//}

			//cout << "\n";

			remove_customer(p, s_try, vehicle_id, position);
			bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
		}

		if (s.routes[vehicle_id].route.size() == 2) {
			check = 1;
		}

		/*cout << "removed customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost << "\n"; */

	}

	if (s_try.routes[vehicle_id].route.size() - 1 == count_no_insertion || best_position == - 1) {
		//cout << "no insertion possible \n";
		s.possible_insertion = 0;
	}

	else { // hier klant invoegen op beste positie 
		insert_customer(p, s, customer_id, vehicle_id, best_position);

		update_solution(p, s, s_recourse);

		bereken_gewogen_route_cost(p, s, s_recourse, vehicle_id);
		calculate_total_cost(p, s);

		s.possible_insertion = 1;

		//cout << "definitieve route \n";
		//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		//	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
		//		cout << s.routes[vehicle_id].route[position] << " ";
		//	}

		//	cout << "\n";

		//}

		//cout << "\n";

	}

	//cout << "best customer " << customer_id << " vehicle " << best_vehicle_id << " position " << best_position << " cost " << best_cost << "\n";
	delete[] s_try.routes;
	delete[] s_recourse.routes;
}

void perform_best_insertion(struct problem& p, struct solution& s, int customer_id) {

	double best_cost = DBL_MAX;
	int best_vehicle_id = -1;
	int best_position = -1;
	struct solution s_try;
	initialize_solution(p, s_try);
	update_solution(p, s, s_try);

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
			s_try.routes[vehicle_id].route[position] = s.routes[vehicle_id].route[position];
			s_try.routes[vehicle_id].route.resize(s.routes[vehicle_id].route.size());
		}
	}

	int check = 0;

	struct solution s_recourse;
	initialize_solution(p, s_recourse);



	/* Check all insertion positions in all vehicles */
	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {

			int predecessor_id = s.routes[vehicle_id].route[(long long)position - 1];
			int successor_id = s.routes[vehicle_id].route[position];

			//cout << "vehicle id " << vehicle_id << "\n";

			if (check == 0) {

				// lijst bijhouden van klanten die ingevoegd moeten worden, dit random door elkaar gooien zodat de klanten om een random manier ingevoegd worden 
				insert_customer(p, s_try, customer_id, vehicle_id, position);

				if (s_try.routes[vehicle_id].earliest_time[position - 1] + p.nodes[s_try.routes[vehicle_id].route[position - 1]].service_dur +
					p.time_matrix[predecessor_id * p.n_nodes + customer_id] <= p.nodes[customer_id].upper_tw &&
					s_try.routes[vehicle_id].latest_time[position] - p.nodes[customer_id].service_dur -
					p.time_matrix[customer_id * p.n_nodes + successor_id] >= p.nodes[s_try.routes[vehicle_id].route[position + 1]].lower_tw) {
					if (check_schedule(p, s_try, vehicle_id) == true) {
						if (check_load(p, s_try, vehicle_id) == true) {

							update_solution(p, s_try, s_recourse);

							bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
							calculate_total_cost(p, s_try);

							//cout << "inserted customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost <<  "\n"; 
							
							//cout << "stry total cost " << s_try.total_cost << " best cost " << best_cost << "\n";
							if (s_try.total_cost < best_cost) {

								best_cost = s_try.total_cost;
								best_vehicle_id = vehicle_id;
								best_position = position;
								 
								//cout << "best position (1) " << best_position << "\n";
								//cout << "best vehicle id (1) " << best_vehicle_id << "\n";

								//cout << "klant " << customer_id << " vehicle " << best_vehicle_id << " position " << best_position << " cost " << best_cost << "\n"; 

							}
						}
					}
				}

				remove_customer(p, s_try, vehicle_id, position);
				bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
			}

			if (s.routes[vehicle_id].route.size() == 2) {
				check = 1;
			}

			/*cout << "removed customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost << "\n"; */

		}
	}
	//cout << "best position (2) " << best_position << "\n";
	//cout << "best vehicle id (2) " << best_vehicle_id << "\n";
	insert_customer(p, s, customer_id, best_vehicle_id, best_position);

	update_solution(p, s, s_recourse);

	bereken_gewogen_route_cost(p, s, s_recourse, best_vehicle_id);
	calculate_total_cost(p, s);

	//cout << "best customer " << customer_id << " vehicle " << best_vehicle_id << " position " << best_position << " cost " << best_cost << "\n";

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
	//	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
	//		cout << s.routes[vehicle_id].route[position] << " ";
	//	}

	//	cout << "\n";
	//}

	//cout << "\n";
	
	delete[] s_try.routes;

}

vector<double> probability_of_failure(problem& p, solution& s, int vehicle_id) { // this function requires the size of a route as input 

	//TODO Improve this function AKA.
	//- Less functional call (reduce switching time)
	//- Encapsulate everything with p.pe.jointCdf(p); --> mutual definition problem?!

	//for (int vehicle_id = 0; vehicle_id < 5; vehicle_id++) {
	//	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
	//		cout << s.routes[vehicle_id].route[position] << " ";
	//	}
	//	cout << "\n";
	//}

	//cout << "\n";

	// Initialise with 0 for depot point.
	vector<double> failure{ 0.0 }; // the different probabilities are put in a vector

	if (s.routes[vehicle_id].route.size() == 2) {

		failure.push_back(0.0);
	}

	else {

		vector<double>::iterator iter = failure.begin();
		// Should be equal to the number of clients in the current route.
		std::vector<std::string> customersIDs;

		//cout << "vehicle " << vehicle_id << "\n";

		//cout << "ordernr " << p.nodes[s.routes[vehicle_id].route[0]].order_nr << "customer " << s.routes[vehicle_id].route[0] << "\n";
		//cout << "ordernr " << p.nodes[s.routes[vehicle_id].route[1]].order_nr << "customer " << s.routes[vehicle_id].route[1] << "\n";
		//cout << "ordernr " << p.nodes[s.routes[vehicle_id].route[2]].order_nr << "customer " << s.routes[vehicle_id].route[2] << "\n";
		//cout << "size route " << s.routes[vehicle_id].route.size() << "\n";
		for (int i = 1; i < s.routes[vehicle_id].route.size() - 1; i++) // hier wordt wel geen rekening gehouden met de veronderstelling dat er geen failure mogelijk is bij de eerste klant in de route 
		{

			customersIDs.push_back(std::to_string(p.nodes[s.routes[vehicle_id].route[i]].order_nr));
		}

		//cout << "size " << customersIDs.size() << "\n";

		//for (int i = 0; i < customersIDs.size(); i++) {
		//	cout << "customersIDs " << customersIDs[i] << " ";
		//}

		//cout << "\n";

		std::vector<std::vector<double>> emplDists = p.pe.getEmpricialDistributions(customersIDs);
		vector<double> jointCdfRes = p.pe.jointCDF(emplDists);
		failure.insert(failure.end(), jointCdfRes.begin(), jointCdfRes.end());
		// End with 0 for depot point.
		failure.push_back(0.0);
	}

	return failure;

	//vector<double> failure = {}; // the different probabilities are put in a vector 
	//failure.push_back(0);
	//for (int i = 0; i < s.routes[vehicle_id].route.size() - 2; i++) { // here it is assumed that the probability of failure at every customer in the route is the same, this needs to be changed of course in what you find.
	//	failure.push_back(0.05 * i);

	//}

	//failure.push_back(0);

	////for (int i = 0; i < routesize; i++) {
	////	cout << kans[i] << "\n";
	////}
	//return failure;
}

bool check_load(struct problem& p, struct solution& s, int vehicle_id) {

	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
		if (s.routes[vehicle_id].load[position] > p.vehicle_cap) {
			return false;
		};
	}

	return true;
}

bool check_schedule(struct problem& p, struct solution& s, int vehicle_id) {

	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw + 0.0001 ||
			s.routes[vehicle_id].schedule[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw - 0.0001) {
			return false;
		};
	}

	return true;
}

void update_load(struct problem& p, struct solution& s, int vehicle_id) {

	s.routes[vehicle_id].load.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].load[0] = 0;

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
		s.routes[vehicle_id].load[position] = s.routes[vehicle_id].load[(long long)position - 1] + p.nodes[s.routes[vehicle_id].route[position]].demand;
	}

}

void update_schedule(struct problem& p, struct solution& s, int vehicle_id) {


	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen 

	//cout << "schedule " << s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] << "\n";

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--) {
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[(long long)position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
			p.time_matrix[s.routes[vehicle_id].route[position] * p.n_nodes + s.routes[vehicle_id].route[(long long)position + 1]];
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw) {
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}
}

void update_earliest_time(struct problem& p, struct solution& s, int vehicle_id) {

	s.routes[vehicle_id].earliest_time.resize(s.routes[vehicle_id].route.size());

	//cout << "lower tw for update earliest time " << p.nodes[0].lower_tw << "\n";
	s.routes[vehicle_id].earliest_time[0] = p.nodes[0].lower_tw;

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
		s.routes[vehicle_id].earliest_time[position] = s.routes[vehicle_id].earliest_time[(long long)position - 1] + p.nodes[s.routes[vehicle_id].route[(long long)position - 1]].service_dur +
			p.time_matrix[s.routes[vehicle_id].route[(long long)position - 1] * p.n_nodes + s.routes[vehicle_id].route[position]];
		if (s.routes[vehicle_id].earliest_time[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw) {
			s.routes[vehicle_id].earliest_time[position] = p.nodes[s.routes[vehicle_id].route[position]].lower_tw;
		}
	}

	/*cout << "earliest time " << s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1] << "\n";*/
}

void update_latest_time(struct problem& p, struct solution& s, int vehicle_id) {

	s.routes[vehicle_id].latest_time.resize(s.routes[vehicle_id].route.size());

	//cout << "p.n_nodes (- 1) " << p.n_nodes - 1 << "\n";
	//cout << "upper tw for update latest time " << p.nodes[0].upper_tw << "\n";

	s.routes[vehicle_id].latest_time[s.routes[vehicle_id].route.size() - 1] = p.nodes[0].upper_tw;// hier vanaf vanachter beginnen tellen
		


	/*cout << "latest time 1 " << s.routes[vehicle_id].latest_time[s.routes[vehicle_id].route.size() - 1] << "\n";*/

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--) {
		s.routes[vehicle_id].latest_time[position] = s.routes[vehicle_id].latest_time[(long long)position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
			p.time_matrix[s.routes[vehicle_id].route[position] * p.n_nodes + s.routes[vehicle_id].route[(long long)position + 1]];
		if (s.routes[vehicle_id].latest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw) {
			s.routes[vehicle_id].latest_time[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}

	
	//	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
	//		cout << s.routes[vehicle_id].route[position] << " ";
	//	}

	//cout << "\n";

	//cout << "latest time ";
	//for (int i = 0; i < s.routes[vehicle_id].route.size(); i++) {
	//	cout << s.routes[vehicle_id].latest_time[i] << " "; 
	//}

	//cout << "\n";
	
}

void write_output_file(struct problem& p, struct solution& s) {

	ofstream output_file;

	output_file.open(("Results " + data_file + ".txt"), std::ios_base::app);

	output_file << "Data file: " << data_file << endl << "Vehicles: " << s.number_of_vehicles_used << " Distance: " << s.total_distance_cost
		<< " Route Duration: " << s.total_route_duration << " Total Cost: " << s.total_cost << "\n";

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {

		output_file << endl << "- route " << vehicle_id << ": ";
		for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			output_file << s.routes[vehicle_id].route[position] << " ";
		}
		output_file;
	}

	output_file << endl << endl;
	output_file.close();
}

//void represent_solution(struct problem& p, struct solution& s) {
//
//	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
//
//		cout << "Route " << vehicle_id << ": ";
//		for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
//			cout << s.routes[vehicle_id].route[position] << " (" << s.routes[vehicle_id].load[position] << ", " << s.routes[vehicle_id].schedule[position] << ") ";
//		}
//		cout << endl << endl;
//	}
//}
