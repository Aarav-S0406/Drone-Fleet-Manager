#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "drone_system.h"

// 1. Sorting & Searching
void merge_sort_parcels(Parcel arr[], int l, int r);
void quick_sort_drones(Drone arr[], int low, int high);
int binary_search_parcel_priority(Parcel arr[], int n, int target_priority);
int recursive_binary_search(Parcel arr[], int low, int high, int target);

// 2. Divide and Conquer
double max_subarray_sum(double arr[], int low, int high, int *start, int *end);
long long fast_exponentiation(long long base, long long exp, long long mod);
int find_celebrity(int matrix[MAX_NODES][MAX_NODES], int n);
void strassen_matrix_mult(int n, double A[MAX_NODES][MAX_NODES], double B[MAX_NODES][MAX_NODES], double C[MAX_NODES][MAX_NODES]);

// 3. Greedy Methods
int activity_selection(int start[], int finish[], int n, int selected[]);
double fractional_knapsack(double capacity, Parcel parcels[], int n);
void huffman_coding(char data[], int freq[], int size);
void dijkstra(WorldState *world, int src, int dist[], int prev[]);
void prim_mst(WorldState *world);
void kruskal_mst(WorldState *world);

// 4. Dynamic Programming
double knapsack_01_dp(int W, double wt[], double val[], int n);
int lcs_length(char *X, char *Y, int m, int n);
double bellman_ford(WorldState *world, int src, int dist[]);
double floyd_warshall(WorldState *world, double dist[MAX_NODES][MAX_NODES]);
double tsp_dp(int mask, int pos, int n, double cost[MAX_NODES][MAX_NODES], int **memo);

// 5. Backtracking & Branch and Bound
bool solve_n_queens_util(int board[MAX_NODES][MAX_NODES], int col, int n);
bool subset_sum(int set[], int n, int sum, int subset[], int subset_size);
void knapsack_backtracking(int i, double current_weight, double current_val, double max_wt, double wt[], double val[], int n, double *max_val);

// 6. Flow and Matching
int ford_fulkerson(int graph[MAX_NODES][MAX_NODES], int s, int t, int n);
int max_bipartite_matching(int bp_graph[MAX_NODES][MAX_NODES], int m, int n, int match[]);

// 7. Number Theory
long long chinese_remainder_theorem(long long num[], long long rem[], int k);

// 8. String Matching
void naive_string_match(char *text, char *pattern);
void rabin_karp_match(char *text, char *pattern, int q);
void kmp_match(char *text, char *pattern);

#endif