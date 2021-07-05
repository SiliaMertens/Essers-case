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

extern string data_file;
extern string coordinates_file;
extern string distribution_file;
extern string optimization; 

void read_data(problem &p)
{

	ifstream infile;
	infile.open(data_file);
	if (!infile.is_open())
	{
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
	double specified_demand, actual_demand;
	int index_count = 1;
	bool is_collection_date_visited = false;

	p.n_customers = 0;

	infile >> p.n_vehicles >> p.vehicle_cap >> p.max_driving_time >> p.max_route_duration;

	cout << "vehicles " << p.n_vehicles << "\n";
	node current_node;

	//TODO: Check if the depot.order_nr is correct and needed?!! Also the demand, lower_tw, upper_tw, service_dur
	infile >> current_node.depot_country // depot information
		>> current_node.depot_postal_code >> current_node.depot_street >> current_node.depot_town >> current_node.order_nr >> current_node.specified_demand >> current_node.actual_demand >> current_node.lower_tw >> current_node.upper_tw >> current_node.service_dur;
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
				sstrm >> unit_description >> delivery_date >> order_nr >> high_sec_indicator >> temp_indicator >> ADR_indicator >> relation_nr >> country >> postal_code >> street >> town >> lower_tw >> upper_tw >> flow_of_goods >> transport_type >> specified_demand >> actual_demand >> service_dur;

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
				current_node.specified_demand = specified_demand;
				current_node.actual_demand = actual_demand;
				current_node.service_dur = service_dur;

				
				//cout << "collection date " << current_node.collection_date << "\n";
				//cout << "unit description " << current_node.unit_description << "\n";
				//cout << "delivery date " << current_node.delivery_date << "\n";

				//Temprary counter for the index. Hence, p.n_customers can also be used (i.e. they are both the same increment).

				p.nodes.push_back(current_node);
				//Keep count only to show the error if exist
				index_count++;
				// To break later if other day is reached.
				is_collection_date_visited = true;
				p.n_customers++; // the number of customers that are present in one day is calculated here

			}
			catch (const std::exception &e)
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

	//cout << "customers " << p.n_customers << "\n";
	//cout << "nodes " << p.n_nodes << "\n";

	infile.close();
}

void read_distance_and_time_matrix(problem &p)
{ // dit moet via Graphhopper, afstanden die daar berekend zijn, aanroepen. Bestand inlezen, berekeningen met x-coord en y-coord moet niet meer gedaan worden.

	// p.distance_matrix = new double[(long long)p.n_nodes * p.n_nodes];
	// p.time_matrix = new double[(long long)p.n_nodes * p.n_nodes];
	// waarschuwing Warning C26451: Arithmetic overflow: Using operator '%operator%' on a %size1% byte value and then casting the result to a %size2% byte value. Cast the value to the wider type before calling operator '%operator%' to avoid overflow
	// door long long valt deze waarschuwing weg

	ifstream infile;
	infile.open(coordinates_file);
	if (!infile.is_open())
	{
		throw std::runtime_error("unable to open file: " + coordinates_file);
	}

	vector<double> current_distance_vector;
	vector<double> current_time_vector;
	string line;
	int curr_a = 0, a = 0, b = 0;
	double a_b_distance = 0.0, a_b_time = 0.0;
	while (getline(infile, line))
	{
		stringstream sstrm(line);
		sstrm >> a >> b >> a_b_distance >> a_b_time;
		if (curr_a != a)
		{
			// To add local vecotrs (current node with all relations to other ndoes)
			p.distance_matrix.emplace_back(current_distance_vector);
			current_distance_vector.clear();
			p.time_matrix.emplace_back(current_time_vector);
			current_time_vector.clear();
			curr_a = a;
		}

		current_distance_vector.push_back(a_b_distance);
		current_time_vector.push_back(a_b_time);

		for (int i = 0; i < current_distance_vector.size(); i++) {
			/*cout << "distance " << current_distance_vector[i] << "\n";*/
		}
	}


	// To add the last vecotr as there are no new line to check curr_a == a (last node with all relations to other ndoes)
	p.distance_matrix.emplace_back(current_distance_vector);
	current_distance_vector.clear();
	p.time_matrix.emplace_back(current_time_vector);
	current_time_vector.clear();
	infile.close();

}

void initialize_solution(problem &p, solution &s)
{
	//TODO: Refactor this function to be included in the s initialiser.

	s.total_distance_cost = 0.0;
	s.total_distance_parameter = 0.0;
	s.number_of_vehicles_used = 0;
	s.vehicle_cost = 0.0;
	s.total_route_duration = 0.0;
	s.total_route_duration_parameter = 0.0;
	s.total_time_window_violation = 0.0;
	s.total_time_window_violation_parameter = 0.0;
	s.total_overtime = 0.0;
	s.total_overtime_parameter = 0.0;
	s.total_driving_time_violation = 0.0;
	s.total_driving_time_violation_parameter = 0.0;
	s.total_cost = 0.0;
	s.total_distance_cost_without_recourse = 0.0;
	s.total_distance_parameter_without_recourse = 0.0;
	s.total_route_duration_without_recourse = 0.0;
	s.total_route_duration_parameter_without_recourse = 0.0;
	s.total_cost_without_recourse = 0.0;
	s.total_distance_cost_with_recourse = 0.0;
	s.total_distance_parameter_with_recourse = 0.0;
	s.total_route_duration_with_recourse = 0.0;
	s.total_route_duration_parameter_with_recourse = 0.0;
	s.total_cost_with_recourse = 0.0;
	s.route_customer = {};
	s.position_customer = {};

	//TODO: clean
	// cout << "I am Here \n";
	route route_instance;
	route_instance.route = {0, 0};
	route_instance.load = {0, 0};
	route_instance.load_actualdemand = { 0, 0 };
	route_instance.earliest_time = {p.nodes[0].lower_tw, p.nodes[0].lower_tw};
	route_instance.latest_time = {p.nodes[p.n_nodes - 1].upper_tw, p.nodes[p.n_nodes - 1].upper_tw};
	route_instance.schedule = {0, 0};
	route_instance.distance_cost = 0.0;
	route_instance.distance_parameter = 0.0;
	route_instance.driving_time = 0.0;
	route_instance.route_used = 0;
	route_instance.route_duration = 0.0;
	route_instance.route_duration_parameter = 0.0;
	route_instance.time_window_violation = 0.0;
	route_instance.time_window_violation_parameter = 0.0;
	route_instance.overtime = 0.0;
	route_instance.overtime_parameter = 0.0;
	route_instance.driving_time_violation = 0.0;
	route_instance.driving_time_violation_parameter = 0.0;
	route_instance.route_cost = 0.0;

	route_instance.distance_cost_no_recourse = 0.0;
	route_instance.distance_parameter_no_recourse = 0.0;
	route_instance.driving_time_no_recourse = 0.0;
	route_instance.route_used_no_recourse = 0;
	route_instance.route_duration_no_recourse = 0.0;
	route_instance.route_duration_parameter_no_recourse = 0.0;
	route_instance.route_cost_no_recourse = 0.0;

	route_instance.distance_cost_recourse = 0.0;
	route_instance.distance_parameter_recourse = 0.0;
	route_instance.driving_time_recourse = 0.0;
	route_instance.route_used_recourse = 0;
	route_instance.route_duration_recourse = 0.0;
	route_instance.route_duration_parameter_recourse = 0.0;
	route_instance.time_window_violation_recourse = 0.0;
	route_instance.time_window_violation_parameter_recourse = 0.0;
	route_instance.overtime_recourse = 0.0;
	route_instance.overtime_parameter_recourse = 0.0;
	route_instance.driving_time_violation_recourse = 0.0;
	route_instance.driving_time_violation_parameter_recourse = 0.0;
	route_instance.route_cost_recourse = 0.0;

	route_instance.weighted_route_cost = 0.0;
	route_instance.weighted_distance_cost = 0.0;
	route_instance.weighted_distance_parameter = 0.0;
	route_instance.weighted_route_duration = 0.0;
	route_instance.weighted_route_duration_parameter = 0.0;
	route_instance.weighted_time_window_violation = 0.0;
	route_instance.weighted_time_window_violation_parameter = 0.0;
	route_instance.weighted_overtime = 0.0;
	route_instance.weighted_overtime_parameter = 0.0;
	route_instance.weighted_driving_time_violation = 0.0;
	route_instance.weighted_driving_time_violation_parameter = 0.0;
	route_instance.weighted_route_cost_without_recourse = 0.0;
	route_instance.weighted_distance_cost_without_recourse = 0.0;
	route_instance.weighted_distance_parameter_without_recourse = 0.0;
	route_instance.weighted_route_duration_without_recourse = 0.0;
	route_instance.weighted_route_duration_parameter_without_recourse = 0.0;
	route_instance.weighted_route_cost_with_recourse = 0.0;
	route_instance.weighted_distance_cost_with_recourse = 0.0;
	route_instance.weighted_distance_parameter_with_recourse = 0.0;
	route_instance.weighted_route_duration_with_recourse = 0.0;
	route_instance.weighted_route_duration_parameter_with_recourse = 0.0;
	route_instance.departure_time = 0.0;

	//Initialise (bootstrap) the s.routes through assign operator
	s.routes.assign(p.n_vehicles, route_instance);
	route_instance = {};
}
//FIXME: Optimise the following 4 block of code.
//REFACTOR: Try memcpy or copy-constructors (i.e. change solution to a class)
void update_solution(solution &s1, solution &s2)
{
	s2.total_distance_cost = s1.total_distance_cost;
	s2.total_distance_parameter = s1.total_distance_parameter;
	s2.number_of_vehicles_used = s1.number_of_vehicles_used;
	s2.vehicle_cost = s1.vehicle_cost;
	s2.total_route_duration = s1.total_route_duration;
	s2.total_route_duration_parameter = s1.total_route_duration_parameter;
	s2.total_time_window_violation = s1.total_time_window_violation;
	s2.total_time_window_violation_parameter = s1.total_time_window_violation_parameter;
	s2.total_overtime = s1.total_overtime;
	s2.total_overtime_parameter = s1.total_overtime_parameter;
	s2.total_driving_time_violation = s1.total_driving_time_violation;
	s2.total_driving_time_violation_parameter = s1.total_driving_time_violation_parameter;
	s2.total_cost = s1.total_cost;
	s2.total_distance_cost_without_recourse = s1.total_distance_cost_without_recourse;
	s2.total_distance_parameter_without_recourse = s1.total_distance_parameter_without_recourse;
	s2.total_route_duration_without_recourse = s1.total_route_duration_without_recourse;
	s2.total_route_duration_parameter_without_recourse = s1.total_route_duration_parameter_without_recourse;
	s2.total_cost_without_recourse = s1.total_cost_without_recourse;
	s2.total_distance_cost_with_recourse = s1.total_distance_cost_with_recourse;
	s2.total_distance_parameter_with_recourse = s1.total_distance_parameter_with_recourse;
	s2.total_route_duration_with_recourse = s1.total_route_duration_with_recourse;
	s2.total_route_duration_parameter_with_recourse = s1.total_route_duration_parameter_with_recourse;
	s2.total_cost_with_recourse = s1.total_cost_with_recourse;
	//Vector to vector copy c++11 (copy & move semantics)
	s2.routes = s1.routes;

}

void change_update_solution_vehicle(solution &s1, solution &s2, int vehicle1, int vehicle2)
{
	s1.routes[vehicle2] = s2.routes[vehicle1];
	// s1.routes[vehicle2].departure_time = s2.routes[vehicle1].departure_time;
}

void position_removed_customers(problem &p, solution &s, int customer_id)
{
	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	{
		for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
		{
			if (customer_id == s.routes[vehicle_id].route[position])
			{
				int route = vehicle_id;

				s.route_customer.push_back(route);
				s.position_customer.push_back(position);
				//To break the loops and end the function if customer has been found.
				return;
			}
		}
	}
}

void bereken_route_cost_zonder_recourse(problem &p, solution &s, int vehicle_id)
{

	s.routes[vehicle_id].route_used = 0;
	s.routes[vehicle_id].route_cost = 0;

	if (s.routes[vehicle_id].route.size() == 2)
	{
		s.routes[vehicle_id].route_cost = 0.0;
		s.routes[vehicle_id].route_used = 0;
	}

	else
	{
		s.routes[vehicle_id].route_cost = fixed_vehicle_cost;
		s.routes[vehicle_id].route_used = 1;


	}

	s.routes[vehicle_id].distance_cost = 0.0;
	s.routes[vehicle_id].distance_parameter = 0.0;

	for (int i = 0; i < s.routes[vehicle_id].route.size() - 1; i++)
	{
		double temp_distance_param = p.distance_matrix[s.routes[vehicle_id].route[i]][s.routes[vehicle_id].route[i + 1]];
		s.routes[vehicle_id].distance_cost += temp_distance_param * km_cost;
		s.routes[vehicle_id].distance_parameter += temp_distance_param;
	}
	
	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].distance_cost;

	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--)
	{
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
												  p.time_matrix[s.routes[vehicle_id].route[position]][s.routes[vehicle_id].route[position + 1]];
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
		{
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}

	s.routes[vehicle_id].departure_time = s.routes[vehicle_id].schedule[0];
	double temp_duration_parameter = s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0];
	s.routes[vehicle_id].route_duration = temp_duration_parameter * driver_cost;
	s.routes[vehicle_id].route_duration_parameter = temp_duration_parameter;

	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].route_duration;
}

void bereken_route_cost_zonder_recourse_actual_demand(problem& p, solution& s, int vehicle_id)
{

	s.routes[vehicle_id].route_used_no_recourse = 0;
	s.routes[vehicle_id].route_cost_no_recourse = 0;

	if (s.routes[vehicle_id].route.size() == 2)
	{
		s.routes[vehicle_id].route_cost_no_recourse = 0.0;
		s.routes[vehicle_id].route_used_no_recourse = 0;
	}

	else
	{
		s.routes[vehicle_id].route_cost_no_recourse = fixed_vehicle_cost;
		s.routes[vehicle_id].route_used_no_recourse = 1;
	}

	s.routes[vehicle_id].distance_cost_no_recourse = 0.0;
	s.routes[vehicle_id].distance_parameter_no_recourse = 0.0;

	for (int i = 0; i < s.routes[vehicle_id].route.size() - 1; i++)
	{
		double temp_distance_param = p.distance_matrix[s.routes[vehicle_id].route[i]][s.routes[vehicle_id].route[i + 1]];
		s.routes[vehicle_id].distance_cost_no_recourse += temp_distance_param * km_cost;
		s.routes[vehicle_id].distance_parameter_no_recourse += temp_distance_param;

		//cout << "distance cost " << s.routes[vehicle_id].distance_cost << "\n";
		//cout << "distance parameter " << s.routes[vehicle_id].distance_parameter << "\n";
	}

	s.routes[vehicle_id].route_cost_no_recourse += s.routes[vehicle_id].distance_cost_no_recourse;

	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--)
	{
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
			p.time_matrix[s.routes[vehicle_id].route[position]][s.routes[vehicle_id].route[position + 1]];
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
		{
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}

	s.routes[vehicle_id].departure_time = s.routes[vehicle_id].schedule[0];
	double temp_duration_parameter = s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0];
	s.routes[vehicle_id].route_duration_no_recourse = temp_duration_parameter * driver_cost;
	s.routes[vehicle_id].route_duration_parameter_no_recourse = temp_duration_parameter;

	s.routes[vehicle_id].route_cost_no_recourse += s.routes[vehicle_id].route_duration_no_recourse;
}




void bereken_route_cost(problem &p, solution &s, int vehicle_id)
{ // similar to the previous one, but here also the recourse components are included (e.g. time window violation, overtime)

	s.routes[vehicle_id].route_used = 0;
	s.routes[vehicle_id].route_cost = 0;

	if (s.routes[vehicle_id].route.size() == 2)
	{
		s.routes[vehicle_id].route_cost = 0.0;
		s.routes[vehicle_id].route_used = 0;
	}

	else
	{
		s.routes[vehicle_id].route_cost = fixed_vehicle_cost;
		s.routes[vehicle_id].route_used = 1;
	}

	s.routes[vehicle_id].distance_cost = 0.0;
	s.routes[vehicle_id].distance_parameter = 0.0;

	for (int i = 0; i < s.routes[vehicle_id].route.size() - 1; i++)
	{
		double temp_distance_parameter = p.distance_matrix[s.routes[vehicle_id].route[i]][s.routes[vehicle_id].route[i + 1]];
		s.routes[vehicle_id].distance_cost += temp_distance_parameter * km_cost;
		s.routes[vehicle_id].distance_parameter += temp_distance_parameter;

		s.routes[vehicle_id].driving_time += p.time_matrix[s.routes[vehicle_id].route[i]][s.routes[vehicle_id].route[i + 1]];
	}
	
	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].distance_cost;
	
	if (s.routes[vehicle_id].driving_time > p.max_driving_time)
	{
		double current_driving_time_violation = s.routes[vehicle_id].driving_time - p.max_driving_time;
		s.routes[vehicle_id].driving_time_violation = current_driving_time_violation * driving_time_violation_cost;
		s.routes[vehicle_id].driving_time_violation_parameter = current_driving_time_violation;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].driving_time_violation;
	}

	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen

	s.routes[vehicle_id].schedule[0] = s.routes[vehicle_id].departure_time;

	for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
	{
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[position - 1] + p.nodes[s.routes[vehicle_id].route[position - 1]].service_dur +
												  p.time_matrix[s.routes[vehicle_id].route[position - 1]][s.routes[vehicle_id].route[position]];
		if (s.routes[vehicle_id].schedule[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw)
		{
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].lower_tw;
		}

		if (s.routes[vehicle_id].earliest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
		{
			double temp_time_window_violation_parameter = s.routes[vehicle_id].earliest_time[position] - p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
			s.routes[vehicle_id].time_window_violation += temp_time_window_violation_parameter * time_window_violation_cost;
			s.routes[vehicle_id].time_window_violation_parameter += temp_time_window_violation_parameter;

			s.routes[vehicle_id].route_cost += s.routes[vehicle_id].time_window_violation;
		}
	}
	double temp_route_duration_parameter = s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0];
	s.routes[vehicle_id].route_duration = temp_route_duration_parameter * driver_cost;
	s.routes[vehicle_id].route_duration_parameter = temp_route_duration_parameter;

	//cout << "route duration " << s.routes[vehicle_id].route_duration << "\n";

	s.routes[vehicle_id].route_cost += s.routes[vehicle_id].route_duration;

	 //for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
	 //{
	 //	if (s.routes[vehicle_id].earliest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
	 //	{
	 //		double temp_time_window_violation_parameter = s.routes[vehicle_id].earliest_time[position] - p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
	 //		s.routes[vehicle_id].time_window_violation += temp_time_window_violation_parameter * time_window_violation_cost;
	 //		s.routes[vehicle_id].time_window_violation_parameter += temp_time_window_violation_parameter;

	 //		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].time_window_violation;
	 //	}
	 //}

	//cout << "time window violation " << s.routes[vehicle_id].time_window_violation << "\n";

	if (s.routes[vehicle_id].route_duration > p.max_route_duration)
	{
		double temp_overtime_parameter = s.routes[vehicle_id].route_duration - p.max_route_duration;
		s.routes[vehicle_id].overtime = temp_overtime_parameter * overtime_cost;
		s.routes[vehicle_id].overtime_parameter = temp_overtime_parameter;
		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].overtime;
	}
}

void bereken_route_cost_actual_demand(problem& p, solution& s, int vehicle_id)
{ // similar to the previous one, but here also the recourse components are included (e.g. time window violation, overtime)

	s.routes[vehicle_id].route_used_recourse = 0;
	s.routes[vehicle_id].route_cost_recourse = 0;

	if (s.routes[vehicle_id].route.size() == 2)
	{
		s.routes[vehicle_id].route_cost_recourse = 0.0;
		s.routes[vehicle_id].route_used_recourse = 0;
	}

	else
	{
		s.routes[vehicle_id].route_cost_recourse = fixed_vehicle_cost;
		s.routes[vehicle_id].route_used_recourse = 1;
	}

	s.routes[vehicle_id].distance_cost_recourse = 0.0;
	s.routes[vehicle_id].distance_parameter_recourse = 0.0;

	for (int i = 0; i < s.routes[vehicle_id].route.size() - 1; i++)
	{
		double temp_distance_parameter = p.distance_matrix[s.routes[vehicle_id].route[i]][s.routes[vehicle_id].route[i + 1]];
		s.routes[vehicle_id].distance_cost_recourse += temp_distance_parameter * km_cost;
		s.routes[vehicle_id].distance_parameter_recourse += temp_distance_parameter;

		s.routes[vehicle_id].driving_time_recourse += p.time_matrix[s.routes[vehicle_id].route[i]][s.routes[vehicle_id].route[i + 1]];
	}

	s.routes[vehicle_id].route_cost_recourse += s.routes[vehicle_id].distance_cost_recourse;

	if (s.routes[vehicle_id].driving_time_recourse > p.max_driving_time)
	{
		double current_driving_time_violation = s.routes[vehicle_id].driving_time_recourse - p.max_driving_time;
		s.routes[vehicle_id].driving_time_violation_recourse = current_driving_time_violation * driving_time_violation_cost;
		s.routes[vehicle_id].driving_time_violation_parameter_recourse = current_driving_time_violation;
		s.routes[vehicle_id].route_cost_recourse += s.routes[vehicle_id].driving_time_violation_recourse;
	}

	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen

	s.routes[vehicle_id].schedule[0] = s.routes[vehicle_id].departure_time;

	for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
	{
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[position - 1] + p.nodes[s.routes[vehicle_id].route[position - 1]].service_dur +
			p.time_matrix[s.routes[vehicle_id].route[position - 1]][s.routes[vehicle_id].route[position]];
		if (s.routes[vehicle_id].schedule[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw)
		{
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].lower_tw;
		}

		if (s.routes[vehicle_id].earliest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
		{
			double temp_time_window_violation_parameter = s.routes[vehicle_id].earliest_time[position] - p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
			s.routes[vehicle_id].time_window_violation_recourse += temp_time_window_violation_parameter * time_window_violation_cost;
			s.routes[vehicle_id].time_window_violation_parameter_recourse += temp_time_window_violation_parameter;

			s.routes[vehicle_id].route_cost_recourse += s.routes[vehicle_id].time_window_violation_recourse;
		}
	}
	double temp_route_duration_parameter = s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] - s.routes[vehicle_id].schedule[0];
	s.routes[vehicle_id].route_duration_recourse = temp_route_duration_parameter * driver_cost;
	s.routes[vehicle_id].route_duration_parameter_recourse = temp_route_duration_parameter;

	//cout << "route duration " << s.routes[vehicle_id].route_duration << "\n";

	s.routes[vehicle_id].route_cost_recourse += s.routes[vehicle_id].route_duration_recourse;

	// for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
	// {
	// 	if (s.routes[vehicle_id].earliest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
	// 	{
	// 		double temp_time_window_violation_parameter = s.routes[vehicle_id].earliest_time[position] - p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
	// 		s.routes[vehicle_id].time_window_violation += temp_time_window_violation_parameter * time_window_violation_cost;
	// 		s.routes[vehicle_id].time_window_violation_parameter += temp_time_window_violation_parameter;

	// 		s.routes[vehicle_id].route_cost += s.routes[vehicle_id].time_window_violation;
	// 	}
	// }

	//cout << "time window violation " << s.routes[vehicle_id].time_window_violation << "\n";

	if (s.routes[vehicle_id].route_duration_recourse > p.max_route_duration)
	{
		double temp_overtime_parameter = s.routes[vehicle_id].route_duration_recourse - p.max_route_duration;
		s.routes[vehicle_id].overtime_recourse = temp_overtime_parameter * overtime_cost;
		s.routes[vehicle_id].overtime_parameter_recourse = temp_overtime_parameter;
		s.routes[vehicle_id].route_cost_recourse += s.routes[vehicle_id].overtime_recourse;
	}
}

vector<double> calculate_probabilities(problem &p, solution &s, int vehicle_id)
{ // the probabilities that were put in the two vectors are used here as input. Furthermore, a specific route (with a certain vehicle id) is used as input as well.

	s.routes[vehicle_id].probability = {}; // initially, there are no probabilities in the vector

	if (optimization == "Stochastic") {
		vector<double> failure = probability_of_failure(p, s, vehicle_id); // the vector probability of failure in the previous function is used here.



		// For example: route 0 1 2 3 0, for this route there are different scenarios where a failure can occur, each with a corresponding probability which is calculated here.

	// double probability_of_failure_in_route = 1.00;
		double probability_of_no_failure_in_route = 1.00;
		/*Change by ahmed after discussing with Silia..
		This should be probability_of_no_failure_in_route = 1 - the probability of failure at the last customer in the route before returing to the depot.
		The reason is that in our code we are already calculating the Join Cummulative Distribution which represents the failure as the following:
		failure_at_customer1, failure_at_customer1_and_2, failure_at_customer1_and_2_3, failure_at_customer1_and_2_3_4_N
		Then just reconstructing the probability vector */
		// for (int i = 1; i < s.routes[vehicle_id].route.size() - 1; i++)
		// {

		// 	probability_of_no_failure_in_route *= (1 - failure[i]); // this represents the probability of having no failure in the route
		// 															// in the example: 1 - 0 (probability of failure at customer 1) - 0.05 (probability of failure at customer 2) - 0.095 (probability of failure at customer 3) = 0.085
		// }

		// the probability of having no failure is put here in the first element of the vector

		// for (int i = 1; i < s.routes[vehicle_id].route.size() - 1; i++)
		// {

		// 	probability_of_failure_in_route = (1 - failure[i - 1]) * probability_of_failure_in_route;
		// 	s.routes[vehicle_id].probability.push_back(failure[i] * probability_of_failure_in_route);
		// 	// here the probabilities of failure at every customer are put in the next elements of the vector. It is assumed that only one failure can occur in a route.
		// 	// Therefore, the probability of failure at the second customer is calculated by probability of failure at customer 2 times the probability of having no failure at customer 1.
		// 	// in the example: probability of failure at customer 1 = 0
		// 	// probability of failure at customer 2 = 0.05 * 1 (no failure at customer 1)
		// 	// probability of failure at customer 3 = 0.10 * 0.95 (no failure at customer 1 and 2)
		// }
		probability_of_no_failure_in_route = 1 - failure[failure.size() - 2];

		s.routes[vehicle_id].probability.push_back(probability_of_no_failure_in_route);
		//To Slice the route vector to execlude depot_node
		auto first = failure.begin() + 1;
		auto last = failure.end() - 1;
		s.routes[vehicle_id].probability.insert(s.routes[vehicle_id].probability.end(), first, last);
		return s.routes[vehicle_id].probability;

		// the probabilities calculated in this function are used in a further stage to calculate the route costs based on the different scenarios that can happen
		// i.e. : no failure: 0 1 2 3 0 * corresponding probability, failure at second customer: 0 1 2 0 2 3 0 * corresponding probability, ...
	}

	else {
		
		vector<double>failure = { 1,1,1 };

		double probability_of_no_failure_in_route = 1.00;

		probability_of_no_failure_in_route = 1 - failure[failure.size() - 2];

		s.routes[vehicle_id].probability.push_back(probability_of_no_failure_in_route);
		auto first = failure.begin() + 1;
		auto last = failure.end() - 1;
		s.routes[vehicle_id].probability.insert(s.routes[vehicle_id].probability.end(), first, last);
		return s.routes[vehicle_id].probability;

	}
	
	
}

void construct_failure_routes(problem &p, solution &s1, solution &s2, int vehicle_id, int position)
{

	// change_update_solution_1(p, s1, s2, vehicle_id);
	change_update_solution_vehicle(s2, s1, vehicle_id, vehicle_id);

	insert_customer(p, s2, 0, vehicle_id, position + 1); // terug naar depot, invoegen in de route
	insert_customer(p, s2, s1.routes[vehicle_id].route[position], vehicle_id, position + 2);
}

void bereken_gewogen_route_cost(problem &p, solution &s1, solution &s2, int vehicle_id)
{ // here, the weighted cost for one vehicle is calculated. As already stated, there are different possibilities where failures can occur in a route. All the different possibilities with their corresponding probabilities are combined here, resulting in a weighted cost

	// change_update_solution_1(p, s1, s2, vehicle_id);
	change_update_solution_vehicle(s2, s1, vehicle_id, vehicle_id);
	vector<double> violation_risk = {};
	s1.routes[vehicle_id].weighted_route_cost = 0.0;
	s1.routes[vehicle_id].weighted_distance_cost = 0.0;
	s1.routes[vehicle_id].weighted_distance_parameter = 0.0;
	s1.routes[vehicle_id].weighted_route_duration = 0.0;
	s1.routes[vehicle_id].weighted_route_duration_parameter = 0.0;
	s1.routes[vehicle_id].weighted_time_window_violation = 0.0;
	s1.routes[vehicle_id].weighted_time_window_violation_parameter = 0.0;
	s1.routes[vehicle_id].weighted_overtime = 0.0;
	s1.routes[vehicle_id].weighted_overtime_parameter = 0.0;
	s1.routes[vehicle_id].weighted_driving_time_violation = 0.0;
	s1.routes[vehicle_id].weighted_driving_time_violation_parameter = 0.0;
	s1.routes[vehicle_id].route_used = 0;
	s1.routes[vehicle_id].weighted_route_cost_without_recourse = 0.0;
	s1.routes[vehicle_id].weighted_distance_cost_without_recourse = 0.0;
	s1.routes[vehicle_id].weighted_distance_parameter_without_recourse = 0.0;
	s1.routes[vehicle_id].weighted_route_duration_without_recourse = 0.0;
	s1.routes[vehicle_id].weighted_route_duration_parameter_without_recourse = 0.0;
	s1.routes[vehicle_id].weighted_route_cost_with_recourse = 0.0;
	s1.routes[vehicle_id].weighted_distance_cost_with_recourse = 0.0;
	s1.routes[vehicle_id].weighted_distance_parameter_with_recourse = 0.0;
	s1.routes[vehicle_id].weighted_route_duration_with_recourse = 0.0;
	s1.routes[vehicle_id].weighted_route_duration_parameter_with_recourse = 0.0;
	violation_risk = calculate_probabilities(p, s1, vehicle_id);

	bereken_route_cost_zonder_recourse(p, s1, vehicle_id); // the routecost when there is no failure is calculated here
	s1.routes[vehicle_id].weighted_route_cost += s1.routes[vehicle_id].route_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_distance_cost += s1.routes[vehicle_id].distance_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_distance_parameter += s1.routes[vehicle_id].distance_parameter * violation_risk[0];
	s1.routes[vehicle_id].weighted_route_duration += s1.routes[vehicle_id].route_duration * violation_risk[0];
	s1.routes[vehicle_id].weighted_route_duration_parameter += s1.routes[vehicle_id].route_duration_parameter * violation_risk[0];

	s1.routes[vehicle_id].weighted_route_cost_without_recourse += s1.routes[vehicle_id].route_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_distance_cost_without_recourse += s1.routes[vehicle_id].distance_cost * violation_risk[0];
	s1.routes[vehicle_id].weighted_distance_parameter_without_recourse += s1.routes[vehicle_id].distance_parameter * violation_risk[0];
	s1.routes[vehicle_id].weighted_route_duration_without_recourse += s1.routes[vehicle_id].route_duration * violation_risk[0];
	s1.routes[vehicle_id].weighted_route_duration_parameter_without_recourse += s1.routes[vehicle_id].route_duration_parameter * violation_risk[0];

	//cout << "weighted route cost without recourse " << s1.routes[vehicle_id].weighted_route_cost_without_recourse << "\n";

	for (int index = 0; index < s1.routes[vehicle_id].probability.size() - 1; index++)
	{
		construct_failure_routes(p, s1, s2, vehicle_id, index);

		bereken_route_cost(p, s2, vehicle_id);

		s1.routes[vehicle_id].weighted_route_cost += s2.routes[vehicle_id].route_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_distance_cost += s2.routes[vehicle_id].distance_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_distance_parameter += s2.routes[vehicle_id].distance_parameter * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_route_duration += s2.routes[vehicle_id].route_duration * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_route_duration_parameter += s2.routes[vehicle_id].route_duration_parameter * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_time_window_violation += s2.routes[vehicle_id].time_window_violation * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_time_window_violation_parameter += s2.routes[vehicle_id].time_window_violation_parameter * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_overtime += s2.routes[vehicle_id].overtime * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_overtime_parameter += s2.routes[vehicle_id].overtime_parameter * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_driving_time_violation += s2.routes[vehicle_id].driving_time_violation * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_driving_time_violation_parameter += s2.routes[vehicle_id].driving_time_violation_parameter * violation_risk[index + 1];

		s1.routes[vehicle_id].weighted_route_cost_with_recourse += s2.routes[vehicle_id].route_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_distance_cost_with_recourse += s2.routes[vehicle_id].distance_cost * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_distance_parameter_with_recourse += s2.routes[vehicle_id].distance_parameter * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_route_duration_with_recourse += s2.routes[vehicle_id].route_duration * violation_risk[index + 1];
		s1.routes[vehicle_id].weighted_route_duration_parameter_with_recourse += s2.routes[vehicle_id].route_duration_parameter * violation_risk[index + 1];

		//cout << "weighted route cost with recourse " << s1.routes[vehicle_id].weighted_route_cost_with_recourse << "\n";
		//cout << "weighted route cost " << s1.routes[vehicle_id].weighted_route_cost << "\n";
	}
}


void calculate_total_cost(problem &p, solution &s)
{

	s.total_cost = 0.0;
	s.total_distance_cost = 0.0;
	s.total_distance_parameter = 0.0;
	s.number_of_vehicles_used = 0;
	s.vehicle_cost = 0.0;
	s.total_route_duration = 0.0;
	s.total_route_duration_parameter = 0.0;
	s.total_time_window_violation = 0.0;
	s.total_time_window_violation_parameter = 0.0;
	s.total_overtime = 0.0;
	s.total_overtime_parameter = 0.0;
	s.total_driving_time_violation = 0.0;
	s.total_driving_time_violation_parameter = 0.0;
	s.total_cost_without_recourse = 0.0;
	s.total_distance_cost_without_recourse = 0.0;
	s.total_distance_parameter_without_recourse = 0.0;
	s.total_route_duration_without_recourse = 0.0;
	s.total_route_duration_parameter_without_recourse = 0.0;
	s.total_cost_with_recourse = 0.0;
	s.total_distance_cost_with_recourse = 0.0;
	s.total_distance_parameter_with_recourse = 0.0;
	s.total_route_duration_with_recourse = 0.0;
	s.total_route_duration_parameter_with_recourse = 0.0;

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	{
		s.total_cost += s.routes[vehicle_id].weighted_route_cost;
		s.total_distance_cost += s.routes[vehicle_id].weighted_distance_cost;
		s.total_distance_parameter += s.routes[vehicle_id].weighted_distance_parameter;
		s.number_of_vehicles_used += s.routes[vehicle_id].route_used;
		s.vehicle_cost += s.routes[vehicle_id].route_used * fixed_vehicle_cost;
		s.total_route_duration += s.routes[vehicle_id].weighted_route_duration;
		s.total_route_duration_parameter += s.routes[vehicle_id].weighted_route_duration_parameter;
		s.total_time_window_violation += s.routes[vehicle_id].weighted_time_window_violation;
		s.total_time_window_violation_parameter += s.routes[vehicle_id].weighted_time_window_violation_parameter;
		s.total_overtime += s.routes[vehicle_id].weighted_overtime;
		s.total_overtime_parameter += s.routes[vehicle_id].weighted_overtime_parameter;
		s.total_driving_time_violation += s.routes[vehicle_id].weighted_driving_time_violation;
		s.total_driving_time_violation_parameter += s.routes[vehicle_id].weighted_driving_time_violation_parameter;


		s.total_cost_without_recourse += s.routes[vehicle_id].weighted_route_cost_without_recourse;
		s.total_distance_cost_without_recourse += s.routes[vehicle_id].weighted_distance_cost_without_recourse;
		s.total_distance_parameter_without_recourse += s.routes[vehicle_id].weighted_distance_parameter_without_recourse;
		s.total_route_duration_without_recourse += s.routes[vehicle_id].weighted_route_duration_without_recourse;
		s.total_route_duration_parameter_without_recourse += s.routes[vehicle_id].weighted_route_duration_parameter_without_recourse;

		s.total_cost_with_recourse += s.routes[vehicle_id].weighted_route_cost_with_recourse;
		s.total_distance_cost_with_recourse += s.routes[vehicle_id].weighted_distance_cost_with_recourse;
		s.total_distance_parameter_with_recourse += s.routes[vehicle_id].weighted_distance_parameter_with_recourse;
		s.total_route_duration_with_recourse += s.routes[vehicle_id].weighted_route_duration_with_recourse;
		s.total_route_duration_parameter_with_recourse += s.routes[vehicle_id].weighted_route_duration_parameter_with_recourse;

		//cout << "total cost without recourse " << s.total_cost_without_recourse << "\n";
		//cout << "total cost with recourse " << s.total_cost_with_recourse << "\n";
		//cout << "total cost " << s.total_cost << "\n";
		//cout << "total distance cost without recourse " << s.total_distance_cost_without_recourse << "\n";
		//cout << "total distance cost with recourse " << s.total_distance_cost_with_recourse << "\n";
		//cout << "total distance cost " << s.total_distance_cost << "\n";
	}

	//cout << "totale afstand: " << s.total_distance_cost << "\n";
	//cout << "overtime " << s.total_overtime << "\n";
}

void change(problem &p, solution &s, int vehicle1, int vehicle2)
{

	struct solution s_try;
	initialize_solution(p, s_try);

	// change_update_solution_1(p, s, s_try, vehicle1);
	change_update_solution_vehicle(s_try, s, vehicle1, vehicle1);

	// change_update_solution_2(p, s, vehicle2, vehicle1);
	change_update_solution_vehicle(s, s, vehicle2, vehicle1);

	// change_update_solution_3(p, s, s_try, vehicle1, vehicle2);
	change_update_solution_vehicle(s, s_try, vehicle1, vehicle2);
}

int last_route(problem &p, solution &s)
{

	int last_vehicle = 0;

	for (int vehicle_id = p.n_vehicles - 1; vehicle_id >= 0; vehicle_id--)
	{
		if (s.routes[vehicle_id].route.size() > 2)
		{
			last_vehicle = vehicle_id;

			return last_vehicle;
		}
	}
	return last_vehicle;
}

void relocate(problem &p, solution &s_prev, solution &s_curr, solution &s_best)
{

	solution s_recourse;
	initialize_solution(p, s_recourse);

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	{ // over alle routes loopen
		if (s_prev.routes[vehicle_id].route.size() <= 3)
		{
			// '0 0' or '0 x 0': no relocate necessary
			continue;
		}

		for (int position = 1; position < s_prev.routes[vehicle_id].route.size() - 1; position++)
		{ // over alle klantposities loopen
			/*cout << "vehicle " << vehicle_id << " prev route size " << s_prev.routes[vehicle_id].route.size() << "\n";*/
			int customer_id = s_prev.routes[vehicle_id].route[position];
			/*cout << "customer_id " << customer_id << "\n";*/
			update_solution(s_prev, s_curr);
			remove_customer(p, s_curr, vehicle_id, position);
			//update_solution(p, s_curr, s_recourse);
			bereken_gewogen_route_cost(p, s_curr, s_recourse, vehicle_id);
			calculate_total_cost(p, s_curr);

			update_earliest_time(p, s_curr, vehicle_id);
			update_latest_time(p, s_curr, vehicle_id);
			update_schedule(p, s_curr, vehicle_id);

			if (s_curr.routes[vehicle_id].route.size() == 2)
			{
				change(p, s_curr, vehicle_id, last_route(p, s_curr));
			}

			perform_best_insertion(p, s_curr, customer_id);
			//cout << "customer_id insertion " << customer_id << "\n";

			if (s_best.total_cost > s_curr.total_cost)
			{
				update_solution(s_curr, s_best);
				update_load(p, s_best, vehicle_id);
				update_earliest_time(p, s_best, vehicle_id);
				update_latest_time(p, s_best, vehicle_id);
				update_schedule(p, s_best, vehicle_id);
			}
		}
	}
}

void swap(problem &p, solution &s1, solution &s2, solution &s3)
{

	solution s_recourse;
	initialize_solution(p, s_recourse);

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	{ // over alle routes loopen
		for (int position = 1; position < s1.routes[vehicle_id].route.size() - 1; position++)
		{ // over alle klantposities loopen

			for (int insert_vehicle_id = 0; insert_vehicle_id < p.n_vehicles; insert_vehicle_id++)
			{
				for (int insert_position = 1; insert_position < s1.routes[insert_vehicle_id].route.size() - 1; insert_position++)
				{

					if (insert_vehicle_id != vehicle_id)
					{

						int customer_id = s1.routes[vehicle_id].route[position];
						update_solution(s1, s2);
						remove_customer(p, s2, vehicle_id, position); // klant uit de route halen
						//update_solution(p, s2, s_recourse);
						bereken_gewogen_route_cost(p, s2, s_recourse, vehicle_id);
						calculate_total_cost(p, s2);
						update_earliest_time(p, s2, vehicle_id);
						update_latest_time(p, s2, vehicle_id);
						update_schedule(p, s2, vehicle_id);

						//cout << "removed customer vehicle " << vehicle_id << " position " << position << " customer " << customer_id << "\n";

						int removed_customer = s2.routes[insert_vehicle_id].route[insert_position];
						remove_customer(p, s2, insert_vehicle_id, insert_position);
						//update_solution(p, s2, s_recourse);
						bereken_gewogen_route_cost(p, s2, s_recourse, insert_vehicle_id);
						calculate_total_cost(p, s2);
						update_earliest_time(p, s2, insert_vehicle_id);
						update_latest_time(p, s2, insert_vehicle_id);
						update_schedule(p, s2, insert_vehicle_id);

						//cout << "second removed customer vehicle " << insert_vehicle_id << " position " << insert_position << " customer " << removed_customer << "\n";

						insert_customer(p, s2, customer_id, insert_vehicle_id, insert_position);

						int predecessor_id = s2.routes[insert_vehicle_id].route[insert_position - 1];
						int successor_id = s2.routes[insert_vehicle_id].route[insert_position];

						if (s2.routes[insert_vehicle_id].earliest_time[insert_position - 1] + p.nodes[s2.routes[insert_vehicle_id].route[insert_position - 1]].service_dur +
									p.time_matrix[predecessor_id][customer_id] <=
								p.nodes[customer_id].upper_tw &&
							s2.routes[insert_vehicle_id].latest_time[insert_position] - p.nodes[customer_id].service_dur -
									p.time_matrix[customer_id][successor_id] >=
								p.nodes[customer_id].lower_tw)
						{
							if (check_schedule(p, s2, insert_vehicle_id) == true)
							{
								if (check_load(p, s2, insert_vehicle_id) == true)
								{

									//update_solution(p, s2, s_recourse);

									bereken_gewogen_route_cost(p, s2, s_recourse, insert_vehicle_id);
									calculate_total_cost(p, s2);

									//cout << "inserted customer " << customer_id << " vehicle " << insert_vehicle_id << " position " << insert_position << " cost " << s2.total_cost << "\n";

									perform_best_insertion_for_swap(p, s2, removed_customer, vehicle_id);

									if (s2.possible_insertion == 1)
									{

										if (s3.total_cost > s2.total_cost)
										{
											update_solution(s2, s3);
											update_load(p, s3, vehicle_id);
											update_earliest_time(p, s3, vehicle_id);
											update_latest_time(p, s3, vehicle_id);
											update_schedule(p, s3, vehicle_id);
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
}

void remove_customer(problem &p, solution &s, int vehicle_id, int position)
{

	// int customer_id = s.routes[vehicle_id].route[position];

	s.routes[vehicle_id].route.erase(s.routes[vehicle_id].route.begin() + position);

	update_load(p, s, vehicle_id);
	update_earliest_time(p, s, vehicle_id);
	update_latest_time(p, s, vehicle_id);
	update_schedule(p, s, vehicle_id);
}

void insert_customer(problem &p, solution &s, int customer_id, int vehicle_id, int position)
{
	//cout << "customer id " << customer_id << " vehicle id " << vehicle_id << " position " << position << "\n";
	s.routes[vehicle_id].route.insert(s.routes[vehicle_id].route.begin() + position, customer_id);

	update_load(p, s, vehicle_id);
	update_earliest_time(p, s, vehicle_id);
	update_latest_time(p, s, vehicle_id);
	update_schedule(p, s, vehicle_id);
}

void perform_best_insertion_for_swap(problem &p, solution &s, int customer_id, int vehicle_id)
{

	s.possible_insertion = 1;
	double best_cost = DBL_MAX;
	//int best_vehicle_id = -1;
	int best_position = -1;
	struct solution s_try;
	initialize_solution(p, s_try);
	update_solution(s, s_try);

	int check = 0;

	struct solution s_recourse;
	initialize_solution(p, s_recourse);

	vector<int> customers_to_be_inserted = {};

	int count_no_insertion = 0;

	/* Check all insertion positions in all vehicles */
	for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
	{

		int predecessor_id = s.routes[vehicle_id].route[position - 1];
		int successor_id = s.routes[vehicle_id].route[position];

		if (check == 0)
		{

			// lijst bijhouden van klanten die ingevoegd moeten worden, dit random door elkaar gooien zodat de klanten om een random manier ingevoegd worden
			insert_customer(p, s_try, customer_id, vehicle_id, position);

			if (s_try.routes[vehicle_id].earliest_time[position - 1] + p.nodes[s_try.routes[vehicle_id].route[position - 1]].service_dur +
						p.time_matrix[predecessor_id][customer_id] <=
					p.nodes[customer_id].upper_tw &&
				s_try.routes[vehicle_id].latest_time[position] - p.nodes[customer_id].service_dur -
						p.time_matrix[customer_id][successor_id] >=
					p.nodes[s_try.routes[vehicle_id].route[position + 1]].lower_tw)
			{
				if (check_schedule(p, s_try, vehicle_id) == true)
				{
					if (check_load(p, s_try, vehicle_id) == true)
					{

						//update_solution(p, s_try, s_recourse);

						bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
						calculate_total_cost(p, s_try);

						//cout << "inserted customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost <<  "\n";

						if (s_try.total_cost < best_cost)
						{

							best_cost = s_try.total_cost;
							//best_vehicle_id = vehicle_id;
							best_position = position;

							//cout << "best position (1) " << best_position << "\n";
							//cout << "(best) vehicle id (1) " << vehicle_id << "\n";

							//cout << "klant " << customer_id << " position " << best_position << " cost " << best_cost << "\n";
						}
					}
					else
					{
						count_no_insertion++;
						//cout << "count no insertion " << count_no_insertion << "\n";
					}
				}
			}

			remove_customer(p, s_try, vehicle_id, position);
			// XXX Onderstaande lijn 30% van rekentijd, maar niet nodig?? de gewogen cost wordt hereberekend na insertion als die voorwaardes satisfied heirboven...
            // bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
		}

		if (s.routes[vehicle_id].route.size() == 2)
		{
			check = 1;
		}

		/*cout << "removed customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost << "\n"; */
	}

	if (s_try.routes[vehicle_id].route.size() - 1 == count_no_insertion || best_position == -1)
	{
		//cout << "no insertion possible \n";
		s.possible_insertion = 0;
	}

	else
	{ // hier klant invoegen op beste positie

		insert_customer(p, s, customer_id, vehicle_id, best_position);

		//update_solution(p, s, s_recourse);

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
}

void perform_best_insertion(problem &p, solution &s, int customer_id)
{

	//cout << "Customer id perform best insertion " << customer_id << "\n";

	s.possible_insertion = 1;
	double best_cost = DBL_MAX;
	int best_vehicle_id = -1;
	int best_position = -1;
	struct solution s_try;
	initialize_solution(p, s_try);
	update_solution(s, s_try);

	int check = 0;

	struct solution s_recourse;
	initialize_solution(p, s_recourse);

	
	/* Check all insertion positions in all vehicles */
	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	{
		for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
		{

			int predecessor_id = s.routes[vehicle_id].route[position - 1];
			int successor_id = s.routes[vehicle_id].route[position];

			//cout << "vehicle id " << vehicle_id << "\n";

			if (check == 0)
			{

				// lijst bijhouden van klanten die ingevoegd moeten worden, dit random door elkaar gooien zodat de klanten om een random manier ingevoegd worden
				insert_customer(p, s_try, customer_id, vehicle_id, position);

				if (s_try.routes[vehicle_id].earliest_time[position - 1] + p.nodes[s_try.routes[vehicle_id].route[position - 1]].service_dur +
							p.time_matrix[predecessor_id][customer_id] <=
						p.nodes[customer_id].upper_tw &&
					s_try.routes[vehicle_id].latest_time[position] - p.nodes[customer_id].service_dur -
							p.time_matrix[customer_id][successor_id] >=
						p.nodes[s_try.routes[vehicle_id].route[position + 1]].lower_tw)
				{
					if (check_schedule(p, s_try, vehicle_id) == true)
					{
						if (check_load(p, s_try, vehicle_id) == true)
						{

							//update_solution(p, s_try, s_recourse);

							bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
							calculate_total_cost(p, s_try);

							//cout << "inserted customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost <<  "\n";

							//cout << "stry total cost " << s_try.total_cost << " best cost " << best_cost << "\n";
							if (s_try.total_cost < best_cost)
							{

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
                // XXX Onderstaande lijn 30% van rekentijd, maar niet nodig?? de gewogen cost wordt hereberekend na insertion als die voorwaardes satisfied heirboven...
				// bereken_gewogen_route_cost(p, s_try, s_recourse, vehicle_id);
			}

			if (s.routes[vehicle_id].route.size() == 2)
			{
				check = 1;
			}

			/*cout << "removed customer " << customer_id << " vehicle " << vehicle_id << " position " << position << " cost " << s_try.total_cost << "\n"; */
		}
	}

	if (best_vehicle_id == -1 && best_position == -1) {
		s.possible_insertion = 0;
	}

	else {
		//cout << "best position (2) " << best_position << "\n";
		//cout << "best vehicle id (2) " << best_vehicle_id << "\n";
		insert_customer(p, s, customer_id, best_vehicle_id, best_position);

		//update_solution(p, s, s_recourse);

		bereken_gewogen_route_cost(p, s, s_recourse, best_vehicle_id);
		calculate_total_cost(p, s);

		s.possible_insertion = 1;
		//cout << "best customer " << customer_id << " vehicle " << best_vehicle_id << " position " << best_position << " cost " << best_cost << "\n";
	}

}

vector<double> probability_of_failure(problem &p, solution &s, int vehicle_id)
{ // this function requires the size of a route as input

	//TODO Improve this function AKA.
	//- Less functional call (reduce switching time)
	//- Encapsulate everything with p.pe.jointCdf(p); --> mutual definition problem?!

	// Initialise with 0 for depot point.
	vector<double> failure{0.0}; // the different probabilities are put in a vector
	//If route was initialised wrongly with only depot point as start and stop {0,0}
	if (s.routes[vehicle_id].route.size() == 2)
	{

		failure.push_back(0.0);
	}

	else
	{

		//To Slice the route vector to execlude depot_node
		auto first = s.routes[vehicle_id].route.begin() + 1;
		auto last = s.routes[vehicle_id].route.end() - 1;
		// Should be equal to the number of clients in the current route.
		std::vector<int> customersIDs(first, last);
		// for (int i = 0, max = s.routes[vehicle_id].route.size(); i < max; i++)
		// {
		// 	cout << " original route: " << s.routes[vehicle_id].route[i] << endl;
		// }
		std::vector<std::vector<double>> emplDists = p.pe->getEmpricialDistributions(customersIDs);
		vector<double> jointCdfRes = p.pe->jointCDF(emplDists);
		failure.insert(failure.end(), jointCdfRes.begin(), jointCdfRes.end());
		// End with 0 for depot point.
		failure.push_back(0.0);
	}

	return failure;
}

int position_failure(problem& p, solution& s, int vehicle_id) {


	//update_load(p, s, vehicle_id);

	s.routes[vehicle_id].load_actualdemand.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].load_actualdemand[0] = 0;

	//cout << "ROUTE ";
	//for (size_t position = 0; position < s.routes[vehicle_id].route.size(); position++) {
	//	cout << s.routes[vehicle_id].route[position] << " ";
	//}

	//cout << "\n";

	for (size_t position = 1; position < s.routes[vehicle_id].route.size(); position++) {

		s.routes[vehicle_id].load_actualdemand[position] = s.routes[vehicle_id].load_actualdemand[position - 1] + p.nodes[s.routes[vehicle_id].route[position]].actual_demand;

		//cout << "position " << position << " load actual demand " << s.routes[vehicle_id].load_actualdemand[position] << "\n";

		if (s.routes[vehicle_id].load_actualdemand[position] > p.vehicle_cap) {

			//cout << "position failure " << position << "\n";
			s.position_failure = position;
			//cout << "s.position failure " << s.position_failure << "\n";
			return s.position_failure;

		}

	}

	//cout << "no failure";
	//cout << "\n";

	return -1;

}

void actual_demand(problem& p, solution& s, int vehicle_id) {

	if (position_failure(p, s, vehicle_id) == -1) {
		bereken_route_cost_zonder_recourse_actual_demand(p, s, vehicle_id);

		//cout << "route cost " << s.routes[vehicle_id].route_cost << "\n";
	}

	else if (position_failure(p, s, vehicle_id) == s.position_failure) {

		construct_failure_routes(p, s, s, vehicle_id, s.position_failure);
		bereken_route_cost_actual_demand(p, s, vehicle_id);

	}


}

void calculate_total_cost_actualdemand(problem& p, solution& s) {

	s.total_cost = 0.0;
	s.total_distance_cost = 0.0;
	s.total_distance_parameter = 0.0;
	s.number_of_vehicles_used = 0;
	s.vehicle_cost = 0.0;
	s.total_route_duration = 0.0;
	s.total_route_duration_parameter = 0.0;
	s.total_time_window_violation = 0.0;
	s.total_time_window_violation_parameter = 0.0;
	s.total_overtime = 0.0;
	s.total_overtime_parameter = 0.0;
	s.total_driving_time_violation = 0.0;
	s.total_driving_time_violation_parameter = 0.0;
	s.total_cost_without_recourse = 0.0;
	s.total_distance_cost_without_recourse = 0.0;
	s.total_distance_parameter_without_recourse = 0.0;
	s.total_route_duration_without_recourse = 0.0;
	s.total_route_duration_parameter_without_recourse = 0.0;
	s.total_cost_with_recourse = 0.0;
	s.total_distance_cost_with_recourse = 0.0;
	s.total_distance_parameter_with_recourse = 0.0;
	s.total_route_duration_with_recourse = 0.0;
	s.total_route_duration_parameter_with_recourse = 0.0;

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		//s.total_cost += s.routes[vehicle_id].route_cost;
		//s.total_distance_cost += s.routes[vehicle_id].distance_cost;
		//s.total_distance_parameter += s.routes[vehicle_id].distance_parameter;
		//s.number_of_vehicles_used += s.routes[vehicle_id].route_used;
		//s.total_route_duration += s.routes[vehicle_id].route_duration;
		//s.total_route_duration_parameter += s.routes[vehicle_id].route_duration_parameter;
		//s.total_time_window_violation += s.routes[vehicle_id].time_window_violation;
		//s.total_time_window_violation_parameter += s.routes[vehicle_id].time_window_violation_parameter;
		//s.total_overtime += s.routes[vehicle_id].overtime;
		//s.total_overtime_parameter += s.routes[vehicle_id].overtime_parameter;
		//s.total_driving_time_violation += s.routes[vehicle_id].driving_time_violation;
		//s.total_driving_time_violation_parameter += s.routes[vehicle_id].driving_time_violation_parameter;

		s.number_of_vehicles_used += s.routes[vehicle_id].route_used_no_recourse + s.routes[vehicle_id].route_used_recourse; // ok? 

		s.vehicle_cost += (s.routes[vehicle_id].route_used_no_recourse + s.routes[vehicle_id].route_used_recourse) * fixed_vehicle_cost; // ok? 
		s.total_cost_without_recourse += s.routes[vehicle_id].route_cost_no_recourse;
		s.total_distance_cost_without_recourse += s.routes[vehicle_id].distance_cost_no_recourse;
		s.total_distance_parameter_without_recourse += s.routes[vehicle_id].distance_parameter_no_recourse;
		s.total_route_duration_without_recourse += s.routes[vehicle_id].route_duration_no_recourse;
		s.total_route_duration_parameter_without_recourse += s.routes[vehicle_id].route_duration_parameter_no_recourse;

		s.total_cost_with_recourse += s.routes[vehicle_id].route_cost_recourse;
		s.total_distance_cost_with_recourse += s.routes[vehicle_id].distance_cost_recourse;
		s.total_distance_parameter_with_recourse += s.routes[vehicle_id].distance_parameter_recourse;
		s.total_route_duration_with_recourse += s.routes[vehicle_id].route_duration_recourse;
		s.total_route_duration_parameter_with_recourse += s.routes[vehicle_id].route_duration_parameter_recourse;
		s.total_time_window_violation += s.routes[vehicle_id].time_window_violation_recourse;
		s.total_time_window_violation_parameter += s.routes[vehicle_id].time_window_violation_parameter_recourse;
		s.total_overtime += s.routes[vehicle_id].overtime_recourse;
		s.total_overtime_parameter += s.routes[vehicle_id].overtime_parameter_recourse;
		s.total_driving_time_violation += s.routes[vehicle_id].driving_time_violation_recourse;
		s.total_driving_time_violation_parameter += s.routes[vehicle_id].driving_time_violation_parameter_recourse;

		s.total_cost = s.total_cost_without_recourse + s.total_cost_with_recourse;
		s.total_distance_cost = s.total_distance_cost_without_recourse + s.total_distance_cost_with_recourse;
		s.total_distance_parameter = s.total_distance_parameter_without_recourse + s.total_distance_parameter_with_recourse;
		s.total_route_duration = s.total_route_duration_without_recourse + s.total_route_duration_with_recourse;
		s.total_route_duration_parameter = s.total_route_duration_parameter_without_recourse + s.total_route_duration_parameter_with_recourse;



	}

	//cout << "totale afstand: " << s.total_distance_cost << "\n";
	//cout << "overtime " << s.total_overtime << "\n";
}

bool check_load(problem &p, solution &s, int vehicle_id)
{

	for (int position = 0; position < s.routes[vehicle_id].route.size(); position++)
	{
		if (s.routes[vehicle_id].load[position] > p.vehicle_cap)
		{
			return false;
		};
	}

	return true;
}

bool check_schedule(problem &p, solution &s, int vehicle_id)
{

	for (int position = 0; position < s.routes[vehicle_id].route.size(); position++)
	{
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw + 0.0001 ||
			s.routes[vehicle_id].schedule[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw - 0.0001)
		{
			return false;
		}
	}

	return true;
}

void update_load(problem &p, solution &s, int vehicle_id)
{

	s.routes[vehicle_id].load.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].load[0] = 0;

	for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
	{
		s.routes[vehicle_id].load[position] = s.routes[vehicle_id].load[position - 1] + p.nodes[s.routes[vehicle_id].route[position]].specified_demand;
	}
}

void update_schedule(problem &p, solution &s, int vehicle_id)
{

	s.routes[vehicle_id].schedule.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] = s.routes[vehicle_id].earliest_time[s.routes[vehicle_id].route.size() - 1]; // hier vanaf vanachter beginnen tellen

	//cout << "schedule " << s.routes[vehicle_id].schedule[s.routes[vehicle_id].route.size() - 1] << "\n";

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--)
	{
		s.routes[vehicle_id].schedule[position] = s.routes[vehicle_id].schedule[position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
												  p.time_matrix[s.routes[vehicle_id].route[position]][s.routes[vehicle_id].route[position + 1]];
		if (s.routes[vehicle_id].schedule[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
		{
			s.routes[vehicle_id].schedule[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}
}

void update_earliest_time(problem &p, solution &s, int vehicle_id)
{

	s.routes[vehicle_id].earliest_time.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].earliest_time[0] = p.nodes[0].lower_tw;

	for (int position = 1; position < s.routes[vehicle_id].route.size(); position++)
	{
		s.routes[vehicle_id].earliest_time[position] = s.routes[vehicle_id].earliest_time[position - 1] + p.nodes[s.routes[vehicle_id].route[position - 1]].service_dur +
													   p.time_matrix[s.routes[vehicle_id].route[position - 1]][s.routes[vehicle_id].route[position]];
		if (s.routes[vehicle_id].earliest_time[position] < p.nodes[s.routes[vehicle_id].route[position]].lower_tw)
		{
			s.routes[vehicle_id].earliest_time[position] = p.nodes[s.routes[vehicle_id].route[position]].lower_tw;
		}
	}
}

void update_latest_time(problem &p, solution &s, int vehicle_id)
{

	s.routes[vehicle_id].latest_time.resize(s.routes[vehicle_id].route.size());

	s.routes[vehicle_id].latest_time[s.routes[vehicle_id].route.size() - 1] = p.nodes[0].upper_tw; // hier vanaf vanachter beginnen tellen

	for (int position = s.routes[vehicle_id].route.size() - 2; position >= 0; position--)
	{
		s.routes[vehicle_id].latest_time[position] = s.routes[vehicle_id].latest_time[position + 1] - p.nodes[s.routes[vehicle_id].route[position]].service_dur -
													 p.time_matrix[s.routes[vehicle_id].route[position]][s.routes[vehicle_id].route[position + 1]];
		if (s.routes[vehicle_id].latest_time[position] > p.nodes[s.routes[vehicle_id].route[position]].upper_tw)
		{
			s.routes[vehicle_id].latest_time[position] = p.nodes[s.routes[vehicle_id].route[position]].upper_tw;
		}
	}
}

void write_output_file(problem &p, solution &s)
{ // nog toevoegen over welk scenario (experiment) het gaat

	ofstream output_file;

	// toevoegen: totale kost vlak voor perturbatie

	output_file.open(("Results without recourse " + data_file + " day " + p.collection_date + " coordinates file " + coordinates_file + ".txt"), std::ios_base::app);
	if (!output_file.is_open())
	{
		throw std::runtime_error("unable to open file: " + ("Results without recourse" + data_file + " day " + p.collection_date + " coordinates file " + coordinates_file + ".txt"));
	}

	

	output_file << "Data file: " << data_file
		<< " Day " << p.collection_date << " coordinates_file " << coordinates_file /*<< "stopping_criteria " << value_no_improvement << " perturbation_percentage " << perturbation_percentage*/ /*<< " resolution " << resolution */<< " TW_violation_penalty_cost " << time_window_violation_cost << " driving_time_violation_cost " << driving_time_violation_cost
		<< " Vehicles: " << s.number_of_vehicles_used << " Vehicle_cost " << s.vehicle_cost << " Total_Distance: " << s.total_distance_cost << " Initial_Distance : " << s.total_distance_cost_without_recourse << " Recourse_Distance : " << s.total_distance_cost_with_recourse
		<< " Total_Distance_parameter: " << s.total_distance_parameter << " Initial_Distance_parameter: " << s.total_distance_parameter_without_recourse << " Recourse_Distance_parameter: " << s.total_distance_parameter_with_recourse
		<< " Total_Route_Duration: " << s.total_route_duration << " Initial_Route_Duration: " << s.total_route_duration_without_recourse << " Recourse_Route_Duration: " << s.total_route_duration_with_recourse
		<< " Total_Route_Duration_parameter: " << s.total_route_duration_parameter << " Initial_Route_Duration_parameter: " << s.total_route_duration_parameter_without_recourse << " Recourse_Route_Duration_parameter: " << s.total_route_duration_parameter_with_recourse
		<< " TW_violation: " << s.total_time_window_violation << " TW_violation_parameter: " << s.total_time_window_violation_parameter << " Overtime: " << s.total_overtime << " Overtime_parameter: " << s.total_overtime_parameter
		<< " Driving_time_violation: " << s.total_driving_time_violation_parameter << " Driving_time_violation_parameter: " << s.total_driving_time_violation_parameter
		<< " Total_Cost: " << s.total_cost << " Intial_Total_Cost: " << s.total_cost_without_recourse << " Recourse_Total_Cost: " << s.total_cost_with_recourse << "\n";

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {

		output_file << endl << "- route " << vehicle_id << " ";
		for (int position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			output_file << s.routes[vehicle_id].route[position] << " ";
		}

		cout << "\n";

		output_file << endl << "specified_demand ";
		for (int position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			output_file << p.nodes[s.routes[vehicle_id].route[position]].specified_demand << " ";
		}

		cout << "\n";

		output_file << endl << "Distance_cost: " << s.routes[vehicle_id].distance_cost << " Distance_parameter: " << s.routes[vehicle_id].weighted_distance_parameter
			<< " Route_duration: " << s.routes[vehicle_id].weighted_route_duration << " Route_duration_parameter: " << s.routes[vehicle_id].weighted_route_duration_parameter << " Total_cost: " << s.routes[vehicle_id].weighted_route_cost;
		output_file;
	}

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	//{

	//	output_file << endl
	//				<< "s_curr.routes[" << vehicle_id << "].route = { ";
	//	for (int position = 0; position < s.routes[vehicle_id].route.size() - 1; position++)
	//	{
	//		output_file << s.routes[vehicle_id].route[position] << ", ";
	//	}
	//	output_file << s.routes[vehicle_id].route[0] << " };";
	//}

	output_file << endl
		<< endl;
	output_file.close();
}

void write_output_file_actual_demand(problem& p, solution& s)
{ // nog toevoegen over welk scenario (experiment) het gaat

	ofstream output_file;

	// toevoegen: totale kost vlak voor perturbatie

	output_file.open(("Results recourse afterwards actual demand " + data_file + " day " + p.collection_date + " coordinates file " + coordinates_file + ".txt"), std::ios_base::app);
	if (!output_file.is_open())
	{
		throw std::runtime_error("unable to open file: " + ("Results recourse afterwards actual demand" + data_file + " day " + p.collection_date + " coordinates file " + coordinates_file + ".txt"));
	}

	//oude versie!!!
	//output_file << "Data file: " << data_file
	//	<< " Day " << p.collection_date << " coordinates_file " << coordinates_file /*<< "stopping_criteria " << value_no_improvement << " perturbation_percentage " << perturbation_percentage*/ /*<< " resolution " << resolution */ << " TW_violation_penalty_cost " << time_window_violation_cost << " driving_time_violation_cost " << driving_time_violation_cost
	//	<< " Vehicles: " << s.number_of_vehicles_used << " Vehicle_cost " << s.vehicle_cost << " Total_Distance: " << s.total_distance_cost << " Initial_Distance : " << s.total_distance_cost_without_recourse << " Recourse_Distance : " << s.total_distance_cost_with_recourse
	//	<< " Total_Distance_parameter: " << s.total_distance_parameter << " Initial_Distance_parameter: " << s.total_distance_parameter_without_recourse << " Recourse_Distance_parameter: " << s.total_distance_parameter_with_recourse
	//	<< " Total_Route_Duration: " << s.total_route_duration << " Initial_Route_Duration: " << s.total_route_duration_without_recourse << " Recourse_Route_Duration: " << s.total_route_duration_with_recourse
	//	<< " Total_Route_Duration_parameter: " << s.total_route_duration_parameter << " Initial_Route_Duration_parameter: " << s.total_route_duration_parameter_without_recourse << " Recourse_Route_Duration_parameter: " << s.total_route_duration_parameter_with_recourse
	//	<< " TW_violation: " << s.total_time_window_violation << " TW_violation_parameter: " << s.total_time_window_violation_parameter << " Overtime: " << s.total_overtime << " Overtime_parameter: " << s.total_overtime_parameter
	//	<< " Driving_time_violation: " << s.total_driving_time_violation_parameter << " Driving_time_violation_parameter: " << s.total_driving_time_violation_parameter
	//	<< " Total_Cost: " << s.total_cost << " Intial_Total_Cost: " << s.total_cost_without_recourse << " Recourse_Total_Cost: " << s.total_cost_with_recourse << "\n";

	double tot_dist_cost_zonder_recourse = 0.0;
	double tot_dist_par_zonder_recourse = 0.0;
	double tot_route_dur_zonder_recourse = 0.0;
	double tot_route_dur_par_zonder_recourse = 0.0;
	double tot_cost_zonder_recourse = 0.0;

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		tot_dist_cost_zonder_recourse += s.routes[vehicle_id].distance_cost;
		tot_dist_par_zonder_recourse += s.routes[vehicle_id].distance_parameter;
		tot_route_dur_zonder_recourse += s.routes[vehicle_id].route_duration;
		tot_route_dur_par_zonder_recourse += s.routes[vehicle_id].route_duration_parameter;
		tot_cost_zonder_recourse += s.routes[vehicle_id].route_cost; 
	}

	output_file << "Data file: " << data_file
		<< " Day " << p.collection_date << " coordinates_file " << coordinates_file /*<< "stopping_criteria " << value_no_improvement << " perturbation_percentage " << perturbation_percentage*/ /*<< " resolution " << resolution */ << " TW_violation_penalty_cost " << time_window_violation_cost << " driving_time_violation_cost " << driving_time_violation_cost
		<< " Vehicles: " << s.number_of_vehicles_used << " Vehicle_cost " << s.vehicle_cost << " Distance_cost_zonder_recourse " << tot_dist_cost_zonder_recourse << " Distance_cost_met_recourse: " << s.total_distance_cost
		<< " Distance_parameter_zonder_recourse " << tot_dist_par_zonder_recourse << " Distance_parameter_met_recourse: " << s.total_distance_parameter 
		<< " Route_duration_zonder_recourse " << tot_route_dur_zonder_recourse << " Route_Duration_met_recourse: " << s.total_route_duration 
		<< " Route_duration_parameter_zonder_recourse " << tot_route_dur_par_zonder_recourse << " Route_Duration_parameter_met_recourse: " << s.total_route_duration_parameter
		<< " TW_violation: " << s.total_time_window_violation << " TW_violation_parameter: " << s.total_time_window_violation_parameter << " Overtime: " << s.total_overtime << " Overtime_parameter: " << s.total_overtime_parameter
		<< " Driving_time_violation: " << s.total_driving_time_violation_parameter << " Driving_time_violation_parameter: " << s.total_driving_time_violation_parameter
		<< " Total_Cost_zonder_recourse: " << tot_cost_zonder_recourse << " Total_Cost_met_recourse: " << s.total_cost << "\n";



	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {

		output_file << endl << "route" << vehicle_id;
		for (int position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			output_file << " "<< s.routes[vehicle_id].route[position] << " ";

		}

		cout << "\n";

		output_file << endl << "specified_demand ";
		for (int position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			output_file << p.nodes[s.routes[vehicle_id].route[position]].specified_demand << " ";
		}

		cout << "\n";

		output_file << endl << "sum_specified_demand ";
		double sum_spec_demand = 0.0;

		for (int position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			sum_spec_demand += p.nodes[s.routes[vehicle_id].route[position]].specified_demand;
			output_file << sum_spec_demand << " ";
		}

		cout << "\n";

		//output_file << endl << "sum_specified_demand " << ;

		output_file << endl << "actual_demand ";
		for (int position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			output_file << p.nodes[s.routes[vehicle_id].route[position]].actual_demand << " ";
		}

		cout << "\n";

		output_file << endl << "sum_actual_demand ";
		double sum_act_demand = 0.0;

		for (int position = 0; position < s.routes[vehicle_id].route.size(); position++) {
			sum_act_demand += p.nodes[s.routes[vehicle_id].route[position]].actual_demand;
			output_file << sum_act_demand << " ";
		}

		cout << "\n";

		output_file << endl << "Distance_cost: " << s.routes[vehicle_id].distance_cost << " Distance_parameter: " << s.routes[vehicle_id].weighted_distance_parameter
			<< " Route_duration: " << s.routes[vehicle_id].weighted_route_duration << " Route_duration_parameter: " << s.routes[vehicle_id].weighted_route_duration_parameter << " Total_cost: " << s.routes[vehicle_id].weighted_route_cost;

		cout << "\n";

		output_file << endl << "Distance_cost_no_recourse: " << s.routes[vehicle_id].distance_cost_no_recourse << " Distance_cost_recourse: " << s.routes[vehicle_id].distance_cost_recourse
			<< " Distance_parameter_no_recourse: " << s.routes[vehicle_id].distance_parameter_no_recourse << " Distance_parameter_recourse: " << s.routes[vehicle_id].distance_parameter_recourse
			<< " Route_duration_no_recourse: " << s.routes[vehicle_id].route_duration_no_recourse << " Route_duration_recourse: " << s.routes[vehicle_id].route_duration_recourse
			<< " Route_duration_parameter_no_recourse: " << s.routes[vehicle_id].route_duration_parameter_no_recourse << " Route_duration_parameter_recourse: " << s.routes[vehicle_id].route_duration_parameter_recourse
			<< " Time_window_violation: " << s.routes[vehicle_id].time_window_violation_recourse << " Time_window_violation_parameter: " << s.routes[vehicle_id].time_window_violation_parameter_recourse
			<< " Overtime: " << s.routes[vehicle_id].overtime_recourse << " Overtime_parameter: " << s.routes[vehicle_id].overtime_recourse
			<< " Driving_time_violation: " << s.routes[vehicle_id].driving_time_violation_recourse << " Driving_time_violation_parameter: " << s.routes[vehicle_id].driving_time_violation_recourse
			<< " Total_cost_no_recourse: " << s.routes[vehicle_id].route_cost_no_recourse << " Total_cost_recourse: " << s.routes[vehicle_id].route_cost_recourse;
		output_file;

		
	}

	//for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++)
	//{

	//	output_file << endl
	//				<< "s_curr.routes[" << vehicle_id << "].route = { ";
	//	for (int position = 0; position < s.routes[vehicle_id].route.size() - 1; position++)
	//	{
	//		output_file << s.routes[vehicle_id].route[position] << ", ";
	//	}
	//	output_file << s.routes[vehicle_id].route[0] << " };";
	//}

	output_file << endl
		<< endl;
	output_file.close();
}

void write_csv_output(problem &p, solution &s, std::string f_name)
{


	fstream output_file;

	// toevoegen: totale kost vlak voor perturbatie
	std::string file_name = f_name;
	// Open file as Input to check if the file exist or not
	output_file.open(file_name, std::ios::in);
	if (!output_file.is_open())
	{
		//If the files as input could not open, then it does not exist.
		//Try to make an output file with the name provided.
		output_file.open(file_name, std::ios::out);
		if (!output_file.is_open())
		{
			//If the system couldnot generate (create the file) and open it then there is a problem.
			throw std::runtime_error("unable to open file: " + (file_name));
		}
		//If the ourput file could be created and opened.
		//Put the header (only once when the file is just created)
		output_file << "data_file,"
					<< "collection_date,"
					<< "coordinates_file,"
					<< "distribution_file,"
					<< "TW_violation_cost,"
					<< "driving_time_violation_cost,"
					<< "number_of_vehicles,"
					<< "vehicle_cost,"
					<< "Distance_cost_zonder_recourse,"
					<< "Distance_cost_met_recourse," 
					<< "Distance_parameter_zonder_recourse,"
					<< "Distance_parameter_met_recourse,"
					<< "total_route_duration_zonder_recourse,"
					<< "total_route_duration_met_recourse,"
					<< "total_route_duration_parameter_zonder_recourse,"
					<< "total_route_duration_parameter_met_recourse,"
					<< "time_window_violation,"
					<< "time_window_violation_parameter,"
					<< "overtime,"
					<< "overtime_parameter,"
					<< "driving_time_violation,"
					<< "driving_time_violation_parameter,"
					<< "total_cost_met_recourse,"
					<< "total_cost_zonder_recourse,"
					<< "\n";
	}
	//If the file as input could be opened, that means the file exist. 
	//In this case, close the InputStream and start an appened stream instead.
	else
	{
		output_file.close();
		output_file.open(file_name, std::ios::app);
		if (!output_file.is_open())
		{
			throw std::runtime_error("unable to open file: " + (file_name));
		}
	}

	double tot_dist_cost_zonder_recourse = 0.0;
	double tot_dist_par_zonder_recourse = 0.0;
	double tot_route_dur_zonder_recourse = 0.0;
	double tot_route_dur_par_zonder_recourse = 0.0;
	double tot_cost_zonder_recourse = 0.0;

	for (int vehicle_id = 0; vehicle_id < p.n_vehicles; vehicle_id++) {
		tot_dist_cost_zonder_recourse += s.routes[vehicle_id].distance_cost;
		tot_dist_par_zonder_recourse += s.routes[vehicle_id].distance_parameter;
		tot_route_dur_zonder_recourse += s.routes[vehicle_id].route_duration;
		tot_route_dur_par_zonder_recourse += s.routes[vehicle_id].route_duration_parameter;
		tot_cost_zonder_recourse += s.routes[vehicle_id].route_cost;
	}

	//Here the file will be exist and 
	output_file << data_file << ","
				<< p.collection_date << ","
				<< coordinates_file << ","
				<< distribution_file << ","
				<< time_window_violation_cost << ","
				<< driving_time_violation_cost << ","
				<< s.number_of_vehicles_used << ","
				<< s.vehicle_cost << ","
				<< tot_dist_cost_zonder_recourse << ","
				<< s.total_distance_cost << "," // distance cost met recourse
				<< tot_dist_par_zonder_recourse << ","
				<< s.total_distance_parameter << ","
				<< tot_route_dur_zonder_recourse << ","
				<< s.total_route_duration << ","
				<< tot_route_dur_par_zonder_recourse << ","
				<< s.total_route_duration_parameter << ","
				<< s.total_time_window_violation << ","
				<< s.total_time_window_violation_parameter << ","
				<< s.total_overtime << ","
				<< s.total_overtime_parameter << ","
				<< s.total_driving_time_violation << ","
				<< s.total_driving_time_violation_parameter << ","
				<< tot_cost_zonder_recourse << "," 
				<< s.total_cost << ","
	
				<< "\n";
	output_file.close();
}
