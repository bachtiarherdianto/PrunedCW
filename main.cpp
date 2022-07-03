/* ******************************************************** */
#include <cmath>
#include <cassert>
#include <list>
#include <set>
#include <queue>
#include <algorithm>
#include <iostream>
#include <vector>
#include <chrono>
#include <iomanip>
#include <unordered_set>

#define PRUNING 100

class ParseInstance{
private:
    int num_nodes = 0;
    int optimum_num_vehicle = 0;
    double capa_vehicle = 0.0;

    std::vector<double> x_coord;
    std::vector<double> y_coord;
    std::vector<double> demand;

    ParseInstance(int N, double Q) :
    num_nodes(N), capa_vehicle(Q){
        this->x_coord.resize(this->num_nodes);
        this->y_coord.resize(this->num_nodes);
        this->demand.resize(this->num_nodes);
        for(int i = 0; i < this->num_nodes; ++i) { this->demand[i] = 0; }
    }

public:
    ParseInstance() = default;

    ~ParseInstance() = default;

    int get_num_nodes() const{ return this->num_nodes; }

    int get_optimum_num_vehicle() const{ return this->optimum_num_vehicle; }

    double get_capacity() const{ return this->capa_vehicle; }

    std::vector<double> get_demand(){ return this->demand; }

    std::vector<double> get_x(){ return this->x_coord; }

    std::vector<double> get_y(){ return this->y_coord; }

    static ParseInstance read(const char* filepath){
        FILE* file;
        int num_nodes_file;
        double vehicle_capa_file;
        double coord_x_file;
        double coord_y_file;
        double demand_node_file;
        int key_node;
        file = fopen(filepath, "r");

        if (file == nullptr) {
            printf("ERROR! Instance file not found\n\nUsage: main <path-to-instance>\n\n");
            exit(1);
        }

        fscanf(file, "%*[^\n]\n", NULL); // NAME: Skip a line
        fscanf(file, "%*[^\n]\n", NULL); // COMMENT: Skip a line
        fscanf(file, "%*[^\n]\n", NULL); // TYPE: Skip a line
        fscanf(file, "DIMENSION : %d\n", &num_nodes_file);  // read the number of nodes in the instance
        fscanf(file, "%*[^\n]\n", NULL); // EDGE_WEIGHT_TYPE: Skip a line
        fscanf(file, "CAPACITY : %lf\n", &vehicle_capa_file);   // read the capacity of the vehicles
        fscanf(file, "%*[^\n]\n", NULL); // NODE_COORD_SECTION: Skip a line

        ParseInstance read_instance(num_nodes_file, vehicle_capa_file);

        for (int i = 0; i < num_nodes_file; ++i) {  // read the coordinates of the nodes
            fscanf(file, "%d %lf %lf\n", &key_node, &coord_x_file, &coord_y_file);
            read_instance.x_coord[i] = coord_x_file;
            read_instance.y_coord[i] = coord_y_file;
        }

        fscanf(file, "%*[^\n]\n", NULL);    // DEMAND_SECTION: Skip a line

        double sum_demand = 0;
        for (int i = 0; i < num_nodes_file; ++i) {  // read the demands of the nodes
            fscanf(file, "%d %lf\n", &key_node, &demand_node_file);
            read_instance.demand[i] = demand_node_file;
            sum_demand += read_instance.demand[i];
        }

        /* Bachtiar's contribution */
        read_instance.optimum_num_vehicle = std::ceil(sum_demand / read_instance.capa_vehicle);   // simple bin packing algorithm

        std::cout << "CVRP instance's information\n";
        std::cout << "- Instance's path         : " << std::string(filepath) << std::endl;
        std::cout << "- Number of depot         : " << 1 << std::endl;
        std::cout << "- Number of customers     : " << read_instance.get_num_nodes() - 1 << std::endl;
        std::cout << "- Vehicle's capacity      : " << std::setprecision(3) << read_instance.get_capacity() << std::endl;
        std::cout << "- Ideal number of routes  : " << read_instance.get_optimum_num_vehicle() << std::endl;

        fclose(file);
        return read_instance;
    }
};

struct compare_min{ bool operator()( std::pair<int, double> const& a, std::pair<int, double> const& b){
        return a.second > b.second;
    }
};

struct compare_max{ bool operator()(std::pair<std::pair<int, int>, double> const& a, std::pair<std::pair<int, int>, double> const& b){
        return a.second < b.second;
    }
};

class Solution{
private:
    struct node;

    struct route_data{
        struct node* first_node;
        struct node* last_node;
        int number_of_nodes_contained;
        double total_demand_served;
        double total_cost;
        double total_cost_backward;
        int key;
        int depot;
    };

    struct node{
        node* left;
        node* right;
        route_data* route;
        int index_segment;
        double acc_demand;  // accumulated total demand
        double acc_cost;    // accumulated total cost
        double acc_cost_bckwrd; // accumulated total cost backward
        int key;
    };

    int n_nearness;
    int n_node;
    int i_depot;
    double total_cost;
    double C;

    std::vector<int> unused_route;
    std::vector<int> node_list;
    std::vector<double> X;
    std::vector<double> Y;
    std::vector<double> D;
    std::set<int> used_route;
    std::vector<std::vector<int>> nearness;
    std::vector<std::vector<double>> savings;
    std::vector<std::vector<double>> nearness_cost;
    std::vector<route_data> set_route;
    std::vector<node> set_node;

public:
    Solution(int n, std::vector<double> x, std::vector<double> y, std::vector<double> d, double c) :
    n_node(n), D(std::move(d)), X(std::move(x)), Y(std::move(y)), C(c){
        // setup initial value
        this->i_depot = 0;
        this->total_cost = 0;

        // prepare data structure
        if (this->set_node.size() != this->n_node){
            this->set_node.resize(this->n_node, node());
            this->set_route.resize(this->n_node, route_data());
            this->unused_route.resize(this->n_node, 0);
        } else{
            this->unused_route.resize(this->n_node, 0);
        }

        for (int i = 0; i < this->n_node; ++i) {
            this->set_node[i].key = i;
            this->set_route[i].key = i;
            this->set_route[i].depot = this->i_depot;
            this->set_route[i].number_of_nodes_contained = 0;
            this->unused_route[i] = this->n_node - i - 1;
            this->node_list.push_back(i);
        }

        // connect the nodes and the routes
        int index_segment_tmp = 1;
        int i_n = 0;        // index node
        int i_r = 0;        // index route
        while (i_n < this->n_node){
            if (this->node_list[i_n] != this->i_depot){
                if (index_segment_tmp == 1){
                    i_r = this->unused_route.back();
                    this->unused_route.pop_back();
                    this->used_route.insert(i_r);
                    this->set_route[i_r].key = i_r;
                    this->set_route[i_r].number_of_nodes_contained = 1;     // add the first node

                    // update index route
                    this->set_route[i_r].first_node = &this->set_node[this->node_list[i_n]];
                    this->set_route[i_r].last_node = &this->set_node[this->node_list[i_n]];

                    // update index node
                    this->set_node[this->node_list[i_n]].route = &this->set_route[i_r];
                    this->set_node[this->node_list[i_n]].index_segment = index_segment_tmp;
                    this->set_node[this->node_list[i_n]].acc_demand = this->D[this->node_list[i_n]];
                    this->set_node[this->node_list[i_n]].acc_cost = this->get_cost(this->set_route[i_r].depot, this->node_list[i_n]);
                } else{ // check possibility to add new node
                    bool is_new_node_can_be_added = true;   // variable to decide whether we can add new node or make new route
                    if (this->set_route[i_r].last_node->acc_demand + this->D[this->node_list[i_n]] > this->C){
                        is_new_node_can_be_added = false;
                    }

                    if (is_new_node_can_be_added){   // add new node to existing route
                        this->set_node[this->node_list[i_n]].route = &this->set_route[i_r];
                        this->set_node[this->node_list[i_n]].index_segment = index_segment_tmp;
                        this->set_node[this->node_list[i_n]].acc_demand = this->set_route[i_r].last_node->acc_demand + this->D[this->node_list[i_n]];
                        this->set_node[this->node_list[i_n]].acc_cost = this->set_route[i_r].last_node->acc_cost + this->get_cost(set_route[i_r].last_node->key, this->node_list[i_n]);

                        Solution::connect_two_nodes(this->set_route[i_r].last_node, &this->set_node[node_list[i_n]]);

                        ++this->set_route[i_r].number_of_nodes_contained;
                        this->set_route[i_r].last_node = &this->set_node[node_list[i_n]];
                    } else{     // close the previous route
                        this->set_route[i_r].total_demand_served = this->set_route[i_r].last_node->acc_demand;
                        this->set_route[i_r].total_cost = this->set_route[i_r].last_node->acc_cost + this->get_cost(this->set_route[i_r].last_node->key, this->set_route[i_r].depot);
                        this->total_cost += this->set_route[i_r].total_cost;

                        // make new routes
                        i_r = this->unused_route.back();
                        this->unused_route.pop_back();
                        this->used_route.insert(i_r);
                        this->set_route[i_r].key = i_r;

                        // add the first node
                        index_segment_tmp = 1;
                        this->set_route[i_r].number_of_nodes_contained = 1;
                        this->set_route[i_r].first_node = &this->set_node[node_list[i_n]];
                        this->set_route[i_r].last_node = &this->set_node[node_list[i_n]];

                        this->set_node[this->node_list[i_n]].route = &this->set_route[i_r];
                        this->set_node[this->node_list[i_n]].index_segment = index_segment_tmp;
                        this->set_node[this->node_list[i_n]].acc_demand = this->D[node_list[i_n]];
                        this->set_node[this->node_list[i_n]].acc_cost = this->get_cost(this->set_route[i_r].depot, this->node_list[i_n]);
                        this->set_node[this->node_list[i_n]].left = &this->set_node[this->set_route[i_r].depot];
                    }
                }

                ++index_segment_tmp; // update index
            }

            ++i_n; // update index

            if (i_n == this->n_node){   // if it last node in the instances, close the previous (the last) route
                this->set_route[i_r].total_demand_served = this->set_route[i_r].last_node->acc_demand;
                this->set_route[i_r].total_cost = this->set_route[i_r].last_node->acc_cost + this->get_cost(this->set_route[i_r].last_node->key, this->set_route[i_r].depot);
                this->set_route[i_r].last_node->right = &this->set_node[this->set_route[i_r].depot];
                this->total_cost += this->set_route[i_r].total_cost;
            }
        }

        this->update_accumulation_final();

        /* Bachtiar's contribution */
        this->n_nearness = std::min<int>(this->n_node, PRUNING);     // pruning 100-nearest nodes
        this->nearness.resize(this->n_node);
        this->nearness_cost.resize(this->n_node);
        for (int i = 0; i < this->n_node; ++i) {
            this->nearness[i].resize(this->n_nearness);
            this->nearness_cost[i].resize(this->n_nearness);
        }

        for (int i = 0; i < this->n_node; ++i) {
            auto Q = std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, compare_min>();
            for (int j = 0; j < this->n_node; ++j) {
                if (i != j){ Q.push(std::make_pair(j, (this->get_cost(i, j)))); }
            }

            for (int j = 0; j < this->n_nearness; ++j) {
                this->nearness[i][j] = Q.top().first;
                this->nearness_cost[i][j] = Q.top().second;
                Q.pop();
            }
        }

        this->savings.resize(this->n_node);
        for (int i = 0; i < this->n_node; ++i) {this->savings[i].resize(this->n_nearness);}
        for (int i = 1; i < this->n_node; ++i) {
            for (int j = 0; j < this->n_nearness; ++j) {    // calculate neighbor's saving table
                if (this->nearness[i][j] > 0) {
                    this->savings[i][j] = this->get_cost(i, 0) + this->get_cost(0, this->nearness[i][j]) - this->get_cost(i, this->nearness[i][j]);
                }
            }
        }
    }

    ~Solution() = default;

    double get_cost(int n1, int n2){
        auto cost = static_cast<double>(sqrt(pow((this->X[n1] - this->X[n2]), 2) + pow((this->Y[n1] - this->Y[n2]), 2)));
        return round(cost);
    }

    double get_total_cost() const{ return this->total_cost; }

    static void connect_two_nodes(node* node_before, node* node_after){
        node_before->right = node_after;
        node_after->left = node_before;
    }

    void free_route(route_data* curr_route){
        this->unused_route.push_back(curr_route->key);
        auto itr = this->used_route.find(curr_route->key);
        if (itr != this->used_route.end()){
            this->used_route.erase(itr);
        }
    }

    void update_accumulation(node* starting_node, bool update_cost, bool update_demand, bool update_index, bool update_route){
        node* curr_node;
        route_data* curr_route = starting_node->route;
        if (update_cost){
            curr_node = starting_node;
            if (curr_node->key == curr_route->first_node->key){
                curr_node->acc_cost = this->get_cost(curr_route->depot, curr_node->key);
            } else{
                curr_node->acc_cost = curr_node->left->acc_cost + this->get_cost(curr_node->left->key, curr_node->key);
            }

            while (curr_node->key != curr_route->last_node->key){
                curr_node = curr_node->right;
                curr_node->acc_cost = curr_node->left->acc_cost + this->get_cost(curr_node->left->key, curr_node->key);
            }

            this->total_cost -= curr_route->total_cost;
            curr_route->total_cost = curr_route->last_node->acc_cost + this->get_cost(curr_route->last_node->key, curr_route->depot);
            this->total_cost += curr_route->total_cost;
        }

        if (update_demand){
            curr_node = starting_node;
            if (curr_node == curr_route->first_node){
                curr_node->acc_demand = this->D[curr_node->key];
            } else{
                curr_node->acc_demand = curr_node->left->acc_demand + this->D[curr_node->key];
            }

            while (curr_node->key != curr_route->last_node->key){
                curr_node = curr_node->right;
                curr_node->acc_demand = curr_node->left->acc_demand + this->D[curr_node->key];
            }

            curr_route->total_demand_served = curr_route->last_node->acc_demand;
        }

        if (update_index){
            curr_node = starting_node;
            if (curr_node == curr_route->first_node){
                curr_node->index_segment = 1;
            } else{
                curr_node->index_segment = curr_node->left->index_segment + 1;
            }

            while (curr_node->key != curr_route->last_node->key){
                curr_node = curr_node->right;
                curr_node->index_segment = curr_node->left->index_segment + 1;
            }

            curr_route->number_of_nodes_contained = curr_route->last_node->index_segment - curr_route->first_node->index_segment + 1;
        }

        if (update_route){
            curr_node = starting_node;
            curr_node->route = curr_route;
            while (curr_node->key != curr_route->last_node->key){
                curr_node = curr_node->right;
                curr_node->route = curr_route;
            }
        }

        bool is_update_backward = true;   // backward calculation
        if (is_update_backward){
            bool is_update_cost_backward = true;
            node* node_bckwrd;
            route_data* route_bckwrd = curr_node->route;
            if (is_update_cost_backward){
                node_bckwrd = route_bckwrd->last_node;
                node_bckwrd->acc_cost_bckwrd = this->get_cost(route_bckwrd->depot, node_bckwrd->key);
                while (node_bckwrd->key != route_bckwrd->first_node->key){
                    node_bckwrd = node_bckwrd->left;
                    node_bckwrd->acc_cost_bckwrd = node_bckwrd->right->acc_cost_bckwrd + this->get_cost(node_bckwrd->right->key, node_bckwrd->key);
                }

                route_bckwrd->total_cost_backward = route_bckwrd->first_node->acc_cost_bckwrd + this->get_cost(route_bckwrd->first_node->key, route_bckwrd->depot);
            }
        }
    }

    void update_accumulation_final(){
        for (int itr : this->used_route) {
            this->update_accumulation(this->set_route[itr].first_node, true, true, true, true);
        }

        this->total_cost = 0;
        for (int itr : this->used_route) {
            this->total_cost += this->set_route[itr].total_cost;
        }
    }

    void update_solution(std::vector<int> curr_solution) {
        this->total_cost = 0;
        if (this->set_node.size() != this->n_node) { // prepare data structure
            this->set_route.clear();
            this->set_node.clear();
            this->set_node.resize(this->n_node, node());
            this->set_route.resize(this->n_node, route_data());
            this->unused_route.resize(this->n_node, 0);
        } else {
            this->unused_route.resize(this->n_node, 0);
        }

        for (int i = 0; i < this->n_node; ++i) {
            this->set_node[i].key = i;
            this->set_route[i].key = i;
            this->set_route[i].depot = this->i_depot;
            this->set_route[i].number_of_nodes_contained = 0;
            this->unused_route[i] = this->n_node - i - 1;
        }

        this->used_route.clear();

        int i_r = 0;     // index route
        for (unsigned int i_n = 0; i_n < curr_solution.size(); ++i_n) {       // index node
            if (curr_solution[i_n] > 0) {  // connect the nodes and the routes
                Solution::connect_two_nodes(&this->set_node[curr_solution[i_n - 1]], &this->set_node[curr_solution[i_n]]);
            }

            if (curr_solution[i_n] == 0) {
                if (i_n > 0) { this->set_route[i_r].last_node = &this->set_node[curr_solution[i_n - 1]]; }    // close the previous route

                if (i_n < curr_solution.size() - 1) {  // add a new route
                    i_r = this->unused_route.back();
                    this->unused_route.pop_back();
                    this->used_route.insert(i_r);
                    this->set_route[i_r].key = i_r;
                    this->set_route[i_r].first_node = &this->set_node[curr_solution[i_n + 1]];
                }
            } else {
                this->set_node[curr_solution[i_n]].route = &this->set_route[i_r];
            }
        }

        this->update_accumulation_final();
    }

    void two_opt_intra_segment(node* node1, node* node2){
        if (node1->index_segment > node2->index_segment){
            std::swap(node1, node2);
        }

        node* node1_left;
        bool node1_first_in_route1 = false;
        if (node1->route->first_node->key == node1->key){ node1_first_in_route1 = true; }
        else{ node1_left = node1->left; }

        node* node2_right;
        bool node2_first_in_route2 = false;
        if (node2->route->last_node->key == node2->key){ node2_first_in_route2 = true; }
        else{ node2_right = node2->right; }

        node* node2_acc = node2;
        auto list_2opt_segment = std::vector<int>();
        list_2opt_segment.push_back(node2->key);
        while (node2_acc->key != node1->key) {    // collect all the segment
            node2_acc = node2_acc->left;
            list_2opt_segment.push_back(node2_acc->key);
        }

        route_data* route_1 = node1->route;     // connect the nodes
        if (node1_first_in_route1){ route_1->first_node = node2; }
        else{ Solution::connect_two_nodes(node1_left, node2); }

        for (int i = 0; i < list_2opt_segment.size() - 1; ++i) {
            Solution::connect_two_nodes(&set_node[list_2opt_segment[i]], &set_node[list_2opt_segment[i + 1]]);
        }

        if (node2_first_in_route2){ route_1->last_node = node1; }
        else{ Solution::connect_two_nodes(node1, node2_right); }

        this->update_accumulation(node2, true, true, true, false);
    }

    void two_opt_inter_segment(node* node1, node* node2){
        node* node1_right;
        bool node1_last_in_route1 = false;
        route_data* route_1 = node1->route;
        if (route_1->last_node->key == node1->key){ node1_last_in_route1 = true; }
        else{ node1_right = node1->right; }

        node* node2_left;
        route_data* route_2 = node2->route;
        bool node2_first_in_route2 = false;
        if (route_2->first_node->key == node2->key){ node2_first_in_route2 = true; }
        else{ node2_left = node2->left; }

        node* last_node_route1 = route_1->last_node;
        node* last_node_route2 = route_2->last_node;

        Solution::connect_two_nodes(node1, node2);
        node2->route = route_1;
        route_1->last_node = last_node_route2;
        this->update_accumulation(node2, true, true, true, true);

        if (node1_last_in_route1) {
            if (node2_first_in_route2) {
                free_route(route_2);
                this->total_cost = 0;
                for (int itr : this->used_route) {
                    this->total_cost += this->set_route[itr].total_cost;
                }
            } else {
                route_2->last_node = node2_left;
                this->update_accumulation(node2_left, true, true, true, true);
            }
        } else {
            if (node2_first_in_route2) {
                route_2->first_node = node1_right;
                route_2->last_node = last_node_route1;
                node1_right->route = route_2;
                this->update_accumulation(node1_right, true, true, true, true);
            } else {
                Solution::connect_two_nodes(node2_left, node1_right);
                route_2->last_node = last_node_route1;
                this->update_accumulation(node2_left, true, true, true, true);
            }
        }
    }
    
    bool two_opt_feasibility(int node1, int node2, double *modified_cost, bool feasible = true){
        route_data* route_1 = this->set_node[node1].route;
        route_data* route_2 = this->set_node[node2].route;

        if (route_1->key == route_2->key) {   // 2-opt intra route
            if (this->set_node[node1].index_segment > this->set_node[node2].index_segment){
                std::swap(node1, node2);
            }

            if (node1 == node2){ feasible = false; }     // two nodes must not same

            double curr_cost = 0;
            if (feasible) {
                if (route_1->first_node->key == node1){
                    curr_cost = this->get_cost(route_1->depot, node2);
                }else{
                    curr_cost = this->set_node[node1].left->acc_cost + this->get_cost(this->set_node[node1].left->key, node2);
                }

                curr_cost += this->set_node[node1].acc_cost_bckwrd - this->set_node[node2].acc_cost_bckwrd;
                if (route_1->last_node->key == node2){
                    curr_cost += this->get_cost(node1, route_1->depot);
                }else{
                    curr_cost += this->get_cost(node1, set_node[node2].right->key) + route_2->total_cost - this->set_node[set_node[node2].right->key].acc_cost;
                }
            }

            *modified_cost += curr_cost - route_1->total_cost;
        } else {
            if (feasible){
                if (route_2->first_node->key == node2) {
                    if (this->set_node[node1].acc_demand + route_2->total_demand_served > this->C){ // new route (2) are not excess vehicle capacity
                        feasible = false;
                    }

                    if (route_1->total_demand_served - this->set_node[node1].acc_demand > this->C){ // new route (1) are not excess vehicle capacity
                        feasible = false;
                    }

                } else {
                    if (this->set_node[node1].acc_demand + route_2->total_demand_served - this->set_node[node2].left->acc_demand > this->C){ // new route (2) are not excess vehicle capacity
                        feasible = false;
                    }

                    if (this->set_node[node2].acc_demand + route_1->total_demand_served - this->set_node[node1].acc_demand > this->C){ // new route (1) are not excess vehicle capacity
                        feasible = false;
                    }
                }
            }

            double route1_curr_cost;
            double route2_curr_cost;
            if (feasible) {
                route1_curr_cost = this->set_node[node1].acc_cost + this->get_cost(node1, node2) + route_2->total_cost - this->set_node[node2].acc_cost;
                if (route_2->first_node->key == node2) {
                    if (route_1->last_node->key == node1){
                        route2_curr_cost = 0;
                    }else{
                        route2_curr_cost = this->get_cost(route_1->depot, this->set_node[node1].right->key) + route_1->total_cost - this->set_node[node1].right->acc_cost;
                    }
                } else {
                    if (route_1->last_node->key == node1){
                        route2_curr_cost = this->set_node[node2].left->acc_cost + this->get_cost(this->set_node[node2].left->key, route_2->depot);
                    }else{
                        route2_curr_cost = this->set_node[node2].left->acc_cost + this->get_cost(this->set_node[node2].left->key, this->set_node[node1].right->key) + route_1->total_cost - this->set_node[node1].right->acc_cost;
                    }
                }
            }

            if (feasible) { 
                *modified_cost += route1_curr_cost + route2_curr_cost - route_1->total_cost - route_2->total_cost; 
            }
        }

        if (feasible){
            return true;
        }

        return false;
    }

    /* Bachtiar's contribution */
    void solve(){
        auto t_start = std::chrono::high_resolution_clock::now();
        auto node_list_temp_cw = std::vector<int>(2 * (this->n_node - 1) + 1, 0);
        for (auto i = 1; i < this->n_node; ++i) { node_list_temp_cw[2 * (i - 1) + 1] = i; }
        this->update_solution(node_list_temp_cw);   // initial solution cw algorithm

        auto Q = std::priority_queue<std::pair< std::pair <int, int>, double>, std::vector<std::pair < std::pair<int, int>, double>>, compare_max>();   // make tree that store and sort the saving value
        for (auto i = 1; i < this->n_node; ++i) {
            for (auto j = 0; j < this->n_nearness; ++j) {        // pruning 100-nearest nodes
                if (this->nearness[i][j] > 0) {
                    Q.push(std::make_pair(std::make_pair(i, this->nearness[i][j]), this->savings[i][j]));
                }
            }
        }

        auto potential_edge = std::pair<std::pair <int, int>, double>();
        auto modified_total_cost = 0.0;
        while (Q.size() > 0) {  // loop one-by one until all have been tested beware feasibility
            potential_edge = Q.top();
            auto index_potential_node1 = potential_edge.first.first;
            auto index_potential_node2 = potential_edge.first.second;
            auto route_temp1 = this->set_node[index_potential_node1].route;
            auto route_temp2 = this->set_node[index_potential_node2].route;
            if (route_temp1->key != route_temp2->key) { // check they must be in different route
                if (route_temp1->number_of_nodes_contained > 1) {
                    if (route_temp1->first_node->key == index_potential_node1 && route_temp2->first_node->key == index_potential_node2) {
                        two_opt_intra_segment(route_temp1->first_node, route_temp1->last_node);
                    } else if (route_temp1->last_node->key == index_potential_node1 && route_temp2->last_node->key == index_potential_node2) {
                        two_opt_intra_segment(route_temp1->first_node, route_temp1->last_node);
                    }
                }

                if (route_temp1->last_node->key == index_potential_node1 && route_temp2->first_node->key == index_potential_node2) {
                    if (two_opt_feasibility(index_potential_node1, index_potential_node2, &modified_total_cost)) {
                        two_opt_inter_segment(&this->set_node[index_potential_node1], &this->set_node[index_potential_node2]);
                    }
                } else if (route_temp1->first_node->key == index_potential_node1 && route_temp2->last_node->key == index_potential_node2) { // if node i is the last in the route and node j is the first in its route
                    if (two_opt_feasibility(index_potential_node2, index_potential_node1, &modified_total_cost)) {
                        two_opt_inter_segment(&this->set_node[index_potential_node2], &this->set_node[index_potential_node1]);
                    }
                }
            }

            Q.pop();    // remove the top edge
        }

        this->update_accumulation_final();  // update objective function
        auto t_end = std::chrono::high_resolution_clock::now();
        auto best_total_cost = this->get_total_cost();

        std::cout << "\nComputation result\n";
        std::cout << "- CW algorithm's solution : distance = " << std::setprecision(12) << this->total_cost << ", no. of routes = " << this->used_route.size() << std::endl;
        std::cout << "- Elapsed time            : " << std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count() << " milliseconds\n";
    }
 };


auto main(int argc, char *argv[]) -> int {
    auto infile = ParseInstance::read(argv[1]);

    Solution cvrp_solver(infile.get_num_nodes(), infile.get_x(), infile.get_y(), infile.get_demand(), infile.get_capacity());

    cvrp_solver.solve();

    return 0;
}
/* ******************************************************** */
