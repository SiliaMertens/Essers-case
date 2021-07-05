#ifndef HEADER_H
#define HEADER_H

#include <map>
#include "ProbabilityEstimator.h"

#pragma once
#pragma once

const double fixed_vehicle_cost = 102.24;
const double km_cost = 0.5834;
const double driver_cost = 0.4166666666666667;
const double overtime_cost = 0.0463333333333333;
extern double time_window_violation_cost;
extern double driving_time_violation_cost;
extern double value_no_improvement;
extern double perturbation_percentage;

class ProbabilityEstimator; // forward declaration

/* Define structs */
struct node
{
	std::string unit_description;
	std::string collection_date;
	std::string delivery_date;
	unsigned int order_nr;
	int high_security_indicator;
	int temperature_controlled_indicator;
	int ADR_indicator;
	int pick_up_relation_nr;
	int pick_up_postal_code;
	std::string pick_up_country;
	std::string pick_up_street;
	std::string pick_up_town;
	double lower_tw;
	double upper_tw;
	std::string flow_of_goods;
	std::string transport_type;
	double specified_demand;
	double actual_demand;
	double service_dur;
	int depot_postal_code;
	std::string depot_country;
	std::string depot_street;
	std::string depot_town;
};
struct problem
{
	std::string collection_date;
	std::vector<node> nodes;
	int n_nodes;
	int n_customers;
	int n_vehicles;
	double vehicle_cap;
	int max_driving_time;
	int max_route_duration;
	std::vector<std::vector<double>> distance_matrix;
	std::vector<std::vector<double>> time_matrix;
	ProbabilityEstimator* pe;
    std::map<const std::vector<int>, std::vector<double> > probmap;
};
struct route
{
	std::vector<int> route;
	std::vector<double> load;
	std::vector<double> load_actualdemand;
	std::vector<double> schedule;
	std::vector<double> earliest_time;
	std::vector<double> latest_time;
	double distance_cost;
	double distance_parameter;
	double driving_time;
	int route_used;
	double route_duration;
	double route_duration_parameter;
	double time_window_violation;
	double time_window_violation_parameter;
	double overtime;
	double overtime_parameter;
	double driving_time_violation;
	double driving_time_violation_parameter;
	double route_cost;

	double distance_cost_no_recourse;
	double distance_parameter_no_recourse;
	double driving_time_no_recourse;
	int route_used_no_recourse;
	double route_duration_no_recourse;
	double route_duration_parameter_no_recourse;
	double route_cost_no_recourse;

	double distance_cost_recourse;
	double distance_parameter_recourse;
	double driving_time_recourse;
	int route_used_recourse;
	double route_duration_recourse;
	double route_duration_parameter_recourse;
	double time_window_violation_recourse;
	double time_window_violation_parameter_recourse;
	double overtime_recourse;
	double overtime_parameter_recourse;
	double driving_time_violation_recourse;
	double driving_time_violation_parameter_recourse;
	double route_cost_recourse;

	double weighted_route_cost;
	double weighted_distance_cost;
	double weighted_distance_parameter;
	double weighted_route_duration;
	double weighted_route_duration_parameter;
	double weighted_time_window_violation;
	double weighted_time_window_violation_parameter;
	double weighted_overtime;
	double weighted_overtime_parameter;
	double weighted_driving_time_violation;
	double weighted_driving_time_violation_parameter;
	double weighted_route_cost_without_recourse;
	double weighted_distance_cost_without_recourse;
	double weighted_distance_parameter_without_recourse;
	double weighted_route_duration_without_recourse;
	double weighted_route_duration_parameter_without_recourse;
	double weighted_route_cost_with_recourse;
	double weighted_distance_cost_with_recourse;
	double weighted_distance_parameter_with_recourse;
	double weighted_route_duration_with_recourse;
	double weighted_route_duration_parameter_with_recourse;
	std::vector<double> probability;
	double departure_time;
};
struct solution
{
	std::vector<route> routes;
	double total_distance_cost;
	double total_distance_parameter;
	int number_of_vehicles_used;
	double vehicle_cost;
	double total_route_duration;
	double total_route_duration_parameter;
	double total_cost;
	double total_time_window_violation;
	double total_time_window_violation_parameter;
	double total_overtime;
	double total_overtime_parameter;
	double total_driving_time_violation;
	double total_driving_time_violation_parameter;
	double total_distance_cost_without_recourse;
	double total_distance_parameter_without_recourse;
	double total_route_duration_without_recourse;
	double total_route_duration_parameter_without_recourse;
	double total_cost_without_recourse;
	double total_distance_cost_with_recourse;
	double total_distance_parameter_with_recourse;
	double total_route_duration_with_recourse;
	double total_route_duration_parameter_with_recourse;
	double total_cost_with_recourse;
	std::vector<int> route_customer;
	std::vector<int> position_customer;
	int position_failure;
	bool possible_insertion;
	
};

/* Preprocessing functions */
void read_data(problem &p);
void read_distance_and_time_matrix(struct problem &p);

/* Solution management functions */
void initialize_solution(problem &p, solution &s);
void update_solution(solution &s1, solution &s2);

void change_update_solution_vehicle(solution &s1, solution &s2, int vehicle1, int vehicle2);
void position_removed_customers(problem &p, solution &s, int customer_id);
void bereken_route_cost_zonder_recourse(problem &p, solution &s, int vehicle_id);
void bereken_route_cost(problem &p, solution &s, int vehicle_id);
void bereken_route_cost_zonder_recourse_actual_demand(problem& p, solution& s, int vehicle_id);
void bereken_route_cost_actual_demand(problem& p, solution& s, int vehicle_id);
std::vector<double> calculate_probabilities(problem &p, solution &s, int vehicle_id);
void construct_failure_routes(problem &p, solution &s1, solution &s2, int vehicle_id, int position);
void bereken_gewogen_route_cost(problem &p, solution &s1, solution &s2, int vehicle_id);
void calculate_total_cost(problem &p, solution &s);
void change(problem &p, struct solution &s, int vehicle1, int vehicle2);
int last_route(problem &p, solution &s);
void actual_demand(problem& p, solution& s, int vehicle_id);
int position_failure(problem& p, solution& s, int vehicle_id);
void calculate_total_cost_actualdemand(problem& p, solution& s);

/* Local search operators */
void relocate(struct problem &p, struct solution &s1, struct solution &s2, struct solution &s3);
void swap(struct problem &p, struct solution &s_prev, struct solution &s_curr, struct solution &s_best);
//void function_relocate(problem& p, solution& s_local_best, solution& s_prev, solution& s_curr, solution& s_total_best, solution& s_ILS_best);
//void function_swap(problem& p, solution& s_local_best, solution& s_prev, solution& s_curr, solution& s_total_best, solution& s_ILS_best);

/* Supporting functions */
void remove_customer(problem &p, struct solution &s, int vehicle_id, int position);
void insert_customer(problem &p, struct solution &s, int customer_id, int vehicle_id, int position);
void perform_best_insertion(problem &p, solution &s, int customer_id);
void perform_best_insertion_for_swap(struct problem &p, struct solution &s, int customer_id, int vehicle_id);
void update_load(problem &p, solution &s, int vehicle_id);
void update_schedule(problem &p, solution &s, int vehicle_id);
void update_earliest_time(problem &p, solution &s, int vehicle_id);
void update_latest_time(problem &p, solution &s, int vehicle_id);

/* Feasibility checks */
bool check_load(problem &p, solution &s, int vehicle_id);
bool check_schedule(problem &p, solution &s, int vehicle_id);
std::vector<double> probability_of_failure(problem &p, solution &s, int vehicle_id);

/* Output */
void write_output_file(problem &p, solution &s);
void write_output_file_actual_demand(problem& p, solution& s);
void write_csv_output(problem& p, solution& s, std::string f_name);

#endif
