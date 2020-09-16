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
    if (!infile.is_open()){
        throw std::runtime_error("unable to open file: " + data_file);
	}

	string line, collection_date, unit_description, delivery_date;
	unsigned int order_nr;
	int high_sec_indicator, temp_indicator, ADR_indicator;
	string country;
	int relation_nr, postal_code;
	string street, town;
	int lower_tw, upper_tw, service_dur;
	string flow_of_goods, transport_type;
	double demand;
	int index_count = 1;
	bool is_collection_date_visited = false;

	p.n_customers = 0;

	infile >> p.n_vehicles >> p.vehicle_cap >> p.max_operating_time >> p.max_route_duration;

	cout << "vehicles " << p.n_vehicles << "\n";
	node current_node;

	//TODO: Check if the depot.order_nr is correct and needed?!! Also the demand, lower_tw, upper_tw, service_dur
	infile >> current_node.depot_country // depot information
		>> current_node.depot_postal_code >> current_node.depot_street >> current_node.depot_town >> current_node.order_nr >> current_node.demand >> current_node.lower_tw >> current_node.upper_tw >> current_node.service_dur;
	//Push the depot node in the vector
	p.nodes.push_back(current_node);
	
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
				//Reset the current_node variable to be assigned new values
				current_node = {};
				//Get the rest of the data.
				sstrm >> unit_description >> delivery_date >> order_nr >> high_sec_indicator >> temp_indicator >> ADR_indicator >> relation_nr >> country >> postal_code >> street >> town >> lower_tw >> upper_tw >> flow_of_goods >> transport_type >> demand >> service_dur;

				// here the collection date is selected which you will use to run the algorithm. The variables are put in the struct.

				current_node.collection_date = collection_date;
				current_node.unit_description = unit_description;
				current_node.delivery_date = delivery_date;
				current_node.order_nr = order_nr;
				current_node.high_security_indicator = high_sec_indicator;
				current_node.temperature_controlled_indicator = temp_indicator;
				current_node.ADR_indicator = ADR_indicator;
				current_node.pick_up_relation_nr = relation_nr;
				current_node.pick_up_country = country;
				current_node.pick_up_postal_code = postal_code;
				current_node.pick_up_street = street;
				current_node.pick_up_town = town;
				current_node.lower_tw = lower_tw;
				current_node.upper_tw = upper_tw;
				current_node.flow_of_goods = flow_of_goods;
				current_node.transport_type = transport_type;
				current_node.demand = demand;
				current_node.service_dur = service_dur;

				//Temprary counter for the index. Hence, p.n_customers can also be used (i.e. they are both the same increment).

				p.nodes.push_back(current_node);
				//Keep count only to show the error if exist
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
	for(const auto &n : p.nodes){
		cout << "ordernumber " << n.order_nr << "\n";
	}
	

	infile.close();
}

void read_distance_and_time_matrix(struct problem& p) { // A text file is being read where the distances (in km) and times (in seconds) for every customer is represented (for example: customer1 - customer2 distance = 100, time = 60)
	// dit moet via Graphhopper, afstanden die daar berekend zijn, aanroepen. Bestand inlezen, berekeningen met x-coord en y-coord moet niet meer gedaan worden. 

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

	}

}

void initialize_solution(struct problem& p, struct solution& s) { // Here everything is initialized at zero, you use this in the beginning of the main to set everything to zero

	s.total_distance_cost = 0.0;
	s.number_of_vehicles_used = 0;
	s.total_route_duration = 0.0;
	s.total_cost = 0.0;
	s.route_customer = {};
	s.position_customer = {};

	//TODO: clean
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
//FIXME: Optimise the following 4 block of code.
//REFACTOR: Try memcpy or copy-constructors (i.e. change solution to a class)
void update_solution(struct problem& p, struct solution& s1, struct solution& s2) { // In this update_solution everything is actually copied from one solution to the other one (s2 = s1)

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

void change_update_solution_1(problem& p, solution& s1, solution& s2, int vehicle1) { // This is similar to the previous function (update solution) but only for one vehicle (not a loop over all vehicles). 
	//This function is used in the function change later on in the code, but this will be explained later on

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

void change_update_solution_2(problem& p, solution& s1, int vehicle1, int vehicle2) { // This one is also similar to the previous one, the difference here: s1(veh2) = s1(veh1) compared to s2(veh1) = s1(veh1) in the previous, also used in the change function
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

void change_update_solution_3(problem& p, solution& s1, solution& s2, int vehicle1, int vehicle2) { // Again similar to the previous one, now s1(veh2) = s2(veh1), also used in the change function
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
//FIXME: This function is wrong (returns twice!!!?)
vector<int> position_removed_customers(problem& p, solution& s, int customer_id) { // This function is made to determine the route and the position of the customers that will be removed in the perturbation (random % of customers that will be removed)

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
			if (customer_id == s.routes[vehicle_id].route[position]) {
				//FIXME: These 2 variables does nothing....
				int route = vehicle_id;
				int route_position = position;
				//TODO: W
				s.route_customer.push_back(route);
				s.position_customer.push_back(position);


			}
		}
	}

	return s.route_customer;
	return s.position_customer;

}

void bereken_route_cost_zonder_recourse(problem& p, solution& s, int vehicle_id) { // Here all the costs are calculated, but here the recourse cost is not taken into account. So actually here it is 'assumed' that there is no recourse cost, which will be used to calculate the total cost when no recourse action is needed

	s.routes[vehicle_id].route_used = 0;
	s.routes[vehicle_id].route_cost = 0;

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
		s.routes[vehicle_id].distance_cost += (p.distance_matrix[s.routes[vehicle_id].route[i] * p.n_nodes + s.routes[vehicle_id].route[i + 1]]) * km_cost;

		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].distance_cost;
	}

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

	s.routes[vehicle_id].route_duration = (s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0]) * driver_cost;

	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].route_duration;

}

void bereken_route_cost(problem& p, solution& s, int vehicle_id) { // similar to the previous one, but here also the recourse components are included (e.g. time window violation, overtime)

	s.routes[vehicle_id].route_used = 0;
	s.routes[vehicle_id].route_cost = 0;

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
		s.routes[vehicle_id].distance_cost += (p.distance_matrix[s.routes[vehicle_id].route[i] * p.n_nodes + s.routes[vehicle_id].route[i + 1]]) * km_cost;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].distance_cost;
	}

	if (s.routes[vehicle_id].distance_cost > p.max_operating_time) {
		s.routes[vehicle_id].driving_time = (s.routes[vehicle_id].distance_cost - p.max_operating_time) * allowable_operating_time_cost;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].driving_time; 
	}

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


	s.routes[vehicle_id].route_duration = (s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0]) * driver_cost;

	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].route_duration;

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
		if (s.routes[vehicle_id].earliest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw) {
			s.routes[vehicle_id].time_window_violiation += (s.routes[vehicle_id].earliest_time[position] - p.nodes[s.routes[vehicle_id].route[position]].upper_tw) * time_window_violation_cost;

			s.routes[vehicle_id].route_cost += s.routes[vehicle_id].time_window_violiation;
		}
	}


	if (s.routes[vehicle_id].route_duration > p.max_route_duration) {
		s.routes[vehicle_id].overtime = s.routes[vehicle_id].route_duration - p.max_route_duration * overtime_cost;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].overtime; 

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


	return s.routes[vehicle_id].probability;

	// the probabilities calculated in this function are used in a further stage to calculate the route costs based on the different scenarios that can happen
	// i.e. : no failure: 0 1 2 3 0 * corresponding probability, failure at second customer: 0 1 2 0 2 3 0 * corresponding probability, ...
}

void construct_failure_routes(problem& p, solution& s1, solution& s2, int vehicle_id, int position) { // When there is a failure, the vehicle has to return to the depot and go back to the customer again, this is represented in this function

	update_solution(p, s1, s2);

	insert_customer(p, s2, 0, vehicle_id, position + 1); // terug naar depot, invoegen in de route
	insert_customer(p, s2, s1.routes[vehicle_id].route[position], vehicle_id, position + 2);

}

void bereken_gewogen_route_cost(problem& p, solution& s1, solution s2, int vehicle_id) { // here, the weighted cost for one vehicle is calculated. As already stated, there are different possibilities where failures can occur in a route (as well as no failure). All the different possibilities with their corresponding probabilities are combined here, resulting in a weighted cost

	update_solution(p, s1, s2);
	vector<double> violation_risk = {};
	s1.routes[vehicle_id].weighted_route_cost = 0.0;
	s1.routes[vehicle_id].weighted_distance_cost = 0.0;
	s1.routes[vehicle_id].weighted_route_duration = 0.0;
	s1.routes[vehicle_id].weighted_time_window_violation = 0.0;
	s1.routes[vehicle_id].weighted_overtime = 0.0;
	s1.routes[vehicle_id].weighted_driving_time = 0.0;
	s1.routes[vehicle_id].route_used = 0;
	violation_risk = calculate_probabilities(p, s1, vehicle_id);

	bereken_route_cost_zonder_recourse(p, s1, vehicle_id); // the routecost when there is no failure is calculated here 
	s1.routes[vehicle_id].weighted_route_cost += s1.routes[vehicle_id].route_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_distance_cost += s1.routes[vehicle_id].distance_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_route_duration += s1.routes[vehicle_id].route_duration * violation_risk[0];


	for (size_t index = 0; index < s1.routes[vehicle_id].probability.size() - 1; index++) {
		construct_failure_routes(p, s1, s2, vehicle_id, index);

		bereken_route_cost(p, s2, vehicle_id);

		s1.routes[vehicle_id].weighted_route_cost += s2.routes[vehicle_id].route_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_distance_cost += s2.routes[vehicle_id].distance_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_route_duration += s2.routes[vehicle_id].route_duration * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_time_window_violation += s2.routes[vehicle_id].time_window_violiation * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_overtime += s2.routes[vehicle_id].overtime * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_driving_time += s2.routes[vehicle_id].driving_time * violation_risk[index + 1];
	}
}

void calculate_total_cost(problem& p, solution& s) { // In the previous function, the weighted cost was calculated for every route, in this function, the total cost for all the routes is calculated (just the sum of all the individual route costs)

	s.total_distance_cost = 0.0;
	s.total_cost = 0.0;
	s.number_of_vehicles_used = 0;
	s.total_route_duration = 0.0;


	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		s.total_distance_cost += s.routes[vehicle_id].weighted_distance_cost;
		s.total_cost += s.routes[vehicle_id].weighted_route_cost;
		s.number_of_vehicles_used += s.routes[vehicle_id].route_used;
		s.total_route_duration += s.routes[vehicle_id].weighted_route_duration;
		s.total_time_window_violation += s.routes[vehicle_id].weighted_time_window_violation;
		s.total_overtime += s.routes[vehicle_id].weighted_overtime;
		s.total_driving_time += s.routes[vehicle_id].weighted_driving_time;


	}

	//cout << "totale afstand: " << s.total_distance_cost << "\n";
}

void change(problem& p, struct solution& s, int vehicle1, int vehicle2) { // this function is used in the relocate function, where I will explain the reason for it 

	struct solution s_try;
	initialize_solution(p, s_try);

	change_update_solution_1(p, s, s_try, vehicle1);

	change_update_solution_2(p, s, vehicle2, vehicle1);

	change_update_solution_3(p, s, s_try, vehicle1, vehicle2);
	
	delete[] s_try.routes;
}

int last_route(problem& p, solution& s) {// this function is used in the relocate function, where I will explain the reason for it 

	int last_vehicle = 0;

	for (int vehicle_id = p.n_vehicles - 1; vehicle_id >= 0; vehicle_id--) {
		if (s.routes[vehicle_id].route.size() > 2) {
			last_vehicle = vehicle_id;

			return last_vehicle;
		}
	}
}

void relocate(struct problem& p, struct solution& s_prev, struct solution& s_curr, struct solution& s_best) { // Relocate function, as explained in PDF methodology file

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
			} // You check every position in every route. However, you have more vehicles than needed, so at one point you have different empty routes, so it is not necessary to check all these empty routes (an empty route has length 2: 0 - 0).
			// However, sometimes it can be that there is an empty route in the middle of all the routes, so if you move this to the end of all routes, you don't need to check all the empty routes

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

void swap(struct problem& p, struct solution& s1, struct solution& s2, struct solution& s3) { // Swap operator as explained in PDF methodology file

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

void remove_customer(struct problem& p, struct solution& s, int vehicle_id, int position) { // This function is used (among others) in the swap and relocate function to remove a customer from the solution, when this is done, the load, earliest time (ET), latest time (LT) and schedule needs to be updated

	int customer_id = s.routes[vehicle_id].route[position];

	s.routes[vehicle_id].route.erase(s.routes[vehicle_id].route.begin() + position);

	update_load(p, s, vehicle_id);
	update_earliest_time(p, s, vehicle_id);
	update_latest_time(p, s, vehicle_id);
	update_schedule(p, s, vehicle_id);

}

void insert_customer(struct problem& p, struct solution& s, int customer_id, int vehicle_id, int position) { // the same as the previous one, only here a customer is inserted instead of removed

	//cout << "customer id " << customer_id << " position " << position << "\n";
	s.routes[vehicle_id].route.insert(s.routes[vehicle_id].route.begin() + position, customer_id);

	update_load(p, s, vehicle_id);
	update_earliest_time(p, s, vehicle_id);
	update_latest_time(p, s, vehicle_id);
	update_schedule(p, s, vehicle_id);

}

void perform_best_insertion_for_swap(struct problem& p, struct solution& s, int customer_id, int vehicle_id) { // Similar to the perform_best_insertion function, only this one is used in the swap operator, because you insert a customer again in the route where you initially removed one (see explanation swap operator)

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
//REFACTOR: Seperate the initial insertion code from the COST calculations
void perform_best_insertion(struct problem& p, struct solution& s, int customer_id) { // Function to check the best route and position to insert a customer (taking into account the load, schedule, ... conditions)

	double best_cost = DBL_MAX;
	int best_vehicle_id = -1;
	int best_position = -1;
	struct solution s_try;
	initialize_solution(p, s_try);
	update_solution(p, s, s_try);
	
	//REFACTOR: Try to use Merge_SORT 
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
	// If the code mistkinly got a route {0, 0} which has no clients but only starts and ends at depot.
	if (s.routes[vehicle_id].route.size() == 2) {
		// End with 0 for depot point.
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
		std::vector<std::vector<double>> emplDists = p.pe.getEmpricialDistributions(customersIDs);
		vector<double> jointCdfRes = p.pe.jointCDF(emplDists);
		failure.insert(failure.end(), jointCdfRes.begin(), jointCdfRes.end());
		// End with 0 for depot point.
		failure.push_back(0.0);
	}

	return failure;

}

bool check_load(struct problem& p, struct solution& s, int vehicle_id) { // In this function, it is checked whether the total load exceeds the capacity of the vehicle (13.6 load meter) or not

	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
		if (s.routes[vehicle_id].load[position] > p.vehicle_cap) {
			return false;
		};
	}

	return true;
}

bool check_schedule(struct problem& p, struct solution& s, int vehicle_id) { // In this function, it is checked whether the time you are at a customer to serve this customer satisfies that customer's time window or not

	for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw + 0.0001 ||
			s.routes[vehicle_id].schedule[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw - 0.0001) {
			return false;
		};
	}

	return true;
}

void update_load(struct problem& p, struct solution& s, int vehicle_id) { // When you insert or remove a customer, you need to update the load, since the route changes 

	s.routes[vehicle_id].load.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].load[0] = 0;

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
		s.routes[vehicle_id].load[position] = s.routes[vehicle_id].load[(long long)position - 1] + p.nodes[s.routes[vehicle_id].route[position]].demand;
	}

}

void update_schedule(struct problem& p, struct solution& s, int vehicle_id) { // Also when you insert or remove a customer, the schedule changes as well 


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

void update_earliest_time(struct problem& p, struct solution& s, int vehicle_id) { // similar as before for the earliest time 

	s.routes[vehicle_id].earliest_time.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].earliest_time[0] = p.nodes[0].lower_tw;

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {
		s.routes[vehicle_id].earliest_time[position] = s.routes[vehicle_id].earliest_time[(long long)position - 1] + p.nodes[s.routes[vehicle_id].route[(long long)position - 1]].service_dur +
			p.time_matrix[s.routes[vehicle_id].route[(long long)position - 1] * p.n_nodes + s.routes[vehicle_id].route[position]];
		if (s.routes[vehicle_id].earliest_time[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw) {
			s.routes[vehicle_id].earliest_time[position] = p.nodes[s.routes[vehicle_id].route[position]].lower_tw;
		}
	}
}

void update_latest_time(struct problem& p, struct solution& s, int vehicle_id) { // similar as before for the latest time 

	s.routes[vehicle_id].latest_time.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].latest_time[s.routes[vehicle_id].route.size() - 1] = p.nodes[0].upper_tw;// hier vanaf vanachter beginnen tellen

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--) {
		s.routes[vehicle_id].latest_time[position] = s.routes[vehicle_id].latest_time[(long long)position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
			p.time_matrix[s.routes[vehicle_id].route[position] * p.n_nodes + s.routes[vehicle_id].route[(long long)position + 1]];
		if (s.routes[vehicle_id].latest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw) {
			s.routes[vehicle_id].latest_time[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}
	
}

void write_output_file(struct problem& p, struct solution& s) { // Just to write the output to a text file 

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

