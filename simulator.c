#include "drone_system.h"
#include "algorithms.h"

void init_world(WorldState *world) {
    world->node_count = 0;
    world->edge_count = 0;
    world->drone_count = 0;
    world->parcel_count = 0;
}

void load_sample_world(WorldState *world) {
    init_world(world);

    // Setup Nodes (Hubs & Customers)
    world->nodes[0] = (Node){0, "Central Hub", 0.0, 0.0, false};
    world->nodes[1] = (Node){1, "Sector A Delivery", 10.0, 15.0, false};
    world->nodes[2] = (Node){2, "Sector B Restricted Zone", 20.0, 5.0, true}; // No-fly zone
    world->nodes[3] = (Node){3, "Sector C Outpost", 30.0, 25.0, false};
    world->node_count = 4;

    // Setup Edges & Weights (Distances)
    world->edges[0] = (Edge){0, 1, 18.0, false};
    world->edges[1] = (Edge){1, 2, 14.0, true}; // Blocked by no-fly zone
    world->edges[2] = (Edge){0, 2, 22.0, false};
    world->edges[3] = (Edge){2, 3, 20.0, false};
    world->edges[4] = (Edge){1, 3, 25.0, false};
    world->edge_count = 5;

    // Setup Drones
    world->drones[0] = (Drone){101, 100.0, 100.0, 5.0, 0, true};
    world->drones[1] = (Drone){102, 150.0, 150.0, 8.0, 0, true};
    world->drone_count = 2;

    // Setup Parcels
    world->parcels[0] = (Parcel){1, 0, 1, 2.5, 9, 30, false};
    world->parcels[1] = (Parcel){2, 0, 3, 4.0, 7, 45, false};
    world->parcels[2] = (Parcel){3, 1, 3, 1.5, 5, 60, false};
    world->parcel_count = 3;

    printf("[System] Loaded Autonomous Drone Logistics World with %d nodes, %d drones, %d parcels.\n",
           world->node_count, world->drone_count, world->parcel_count);
}

void simulate_normal_operation(WorldState *world) {
    printf("\n=== SCENARIO 1: NORMAL OPERATION (Dijkstra + Greedy Allocation) ===\n");
    int dist[MAX_NODES], prev[MAX_NODES];
    dijkstra(world, 0, dist, prev);
    for (int i = 0; i < world->node_count; i++) {
        if (dist[i] == INT_MAX)
            printf("Node %d (%s): unreachable (restricted or disconnected)\n", i, world->nodes[i].name);
        else
            printf("Shortest distance from Central Hub to Node %d (%s): %d units\n",
                   i, world->nodes[i].name, dist[i]);
    }
    merge_sort_parcels(world->parcels, 0, world->parcel_count - 1);
    printf("Parcels prioritized and scheduled successfully via Merge Sort.\n");
    prim_mst(world);
    kruskal_mst(world);
}

void simulate_constraint_shock(WorldState *world) {
    printf("\n=== SCENARIO 2: CONSTRAINT SHOCK (No-Fly Zone Obstruction & Max Flow) ===\n");
    int n = world->node_count > 4 ? world->node_count : 4;
    int graph[MAX_NODES][MAX_NODES] = {{0}};
    graph[0][1] = 10;
    graph[0][2] = 5;
    graph[1][2] = 15;
    graph[1][3] = 10;
    graph[2][3] = 10;
    int max_f = ford_fulkerson(graph, 0, 3, n);
    printf("Maximum feasible fleet delivery throughput under constraint shock: %d units/hr\n", max_f);

    int bp_graph[MAX_NODES][MAX_NODES] = {{0}};
    bp_graph[0][0] = 1; bp_graph[0][1] = 1;
    bp_graph[1][1] = 1; bp_graph[1][2] = 1;
    int match[MAX_NODES];
    int matched = max_bipartite_matching(bp_graph, 2, 3, match);
    printf("Maximum drone-to-parcel feasible assignments: %d\n", matched);
}

void simulate_stress_test(WorldState *world) {
    printf("\n=== SCENARIO 3: LARGE-DATA STRESS CASE (DP Knapsack & Telemetry Coding) ===\n");
    double wt[] = {2.5, 4.0, 1.5, 3.0};
    double val[] = {90.0, 75.0, 50.0, 60.0};
    double max_val = knapsack_01_dp(6, wt, val, 4);
    printf("Optimal 0/1 Knapsack Parcel Value for Drone Capacity 6.0kg: %.2f\n", max_val);

    char syms[] = {'T', 'A', 'K', 'E'};
    int freqs[] = {12, 5, 3, 2};
    huffman_coding(syms, freqs, 4);
    printf("Active world parcels tracked: %d\n", world->parcel_count);
}

void compare_payload_strategies_and_export(WorldState *world) {
    double capacity = 6.0;
    double wt[] = {2.5, 4.0, 1.5, 3.0};
    double val[] = {90.0, 75.0, 50.0, 60.0};
    int n = 4;

    Parcel greedy_copy[4];
    for (int i = 0; i < n; i++) {
        greedy_copy[i].parcel_id = i + 1;
        greedy_copy[i].weight = wt[i];
        greedy_copy[i].priority = (int)val[i];
    }

    clock_t t0 = clock();
    double greedy_score = fractional_knapsack(capacity, greedy_copy, n);
    clock_t t1 = clock();

    clock_t t2 = clock();
    double dp_score = knapsack_01_dp((int)capacity, wt, val, n);
    clock_t t3 = clock();

    double greedy_ms = ((double)(t1 - t0) / CLOCKS_PER_SEC) * 1000.0;
    double dp_ms = ((double)(t3 - t2) / CLOCKS_PER_SEC) * 1000.0;

    printf("\n=== SAME-INPUT ALGORITHMIC COMPARISON (Payload Cap: %.1f kg) ===\n", capacity);
    printf("Fractional Knapsack (Greedy): Score=%.2f | Time=%.4f ms | Space=O(1) aux\n",
           greedy_score, greedy_ms);
    printf("0/1 Knapsack (DP):            Score=%.2f | Time=%.4f ms | Space=O(n*W)\n",
           dp_score, dp_ms);
    printf("Note: Fractional allows split parcels; 0/1 DP enforces whole-parcel delivery.\n");

    FILE *fp = fopen("mission_report.json", "w");
    if (fp != NULL) {
        fprintf(fp, "{\n  \"project\": \"Project 38 - Drone Delivery Fleet Mission Planner\",\n");
        fprintf(fp, "  \"nodes\": %d,\n  \"drones\": %d,\n  \"parcels\": %d,\n",
                world->node_count, world->drone_count, world->parcel_count);
        fprintf(fp, "  \"comparison\": {\n    \"greedy_fractional_value\": %.2f,\n", greedy_score);
        fprintf(fp, "    \"dp_01_value\": %.2f\n  }\n}\n", dp_score);
        fclose(fp);
        printf("[Export] Saved mission summary to mission_report.json\n");
    }
}

void run_menu(WorldState *world) {
    int choice;
    do {
        printf("\n=========================================\n");
        printf(" DRONE DELIVERY FLEET MISSION PLANNER (PBL-38)\n");
        printf("=========================================\n");
        printf("1. Load / Reset Sample Mission World\n");
        printf("2. Run Normal Operation Simulation (Dijkstra + MergeSort + MST)\n");
        printf("3. Run Constraint Shock Simulation (Ford-Fulkerson + Matching)\n");
        printf("4. Run Large-Data Stress Test (0/1 DP Knapsack + Huffman)\n");
        printf("5. Run String Matching Telemetry Verification (Naive / Rabin-Karp / KMP)\n");
        printf("6. Compare Payload Strategies & Export JSON Report\n");
        printf("7. Exit\n");
        printf("Enter your choice: ");
        if (scanf("%d", &choice) != 1) break;

        switch (choice) {
            case 1: load_sample_world(world); break;
            case 2: simulate_normal_operation(world); break;
            case 3: simulate_constraint_shock(world); break;
            case 4: simulate_stress_test(world); break;
            case 5: {
                char text[] = "DRONE_LOG_SUCCESS_TAKEOFF_ALTITUDE_50M";
                char pattern[] = "SUCCESS";
                printf("Running String Matching algorithms on telemetry logs:\n");
                naive_string_match(text, pattern);
                rabin_karp_match(text, pattern, 101);
                kmp_match(text, pattern);
                break;
            }
            case 6: compare_payload_strategies_and_export(world); break;
            case 7: printf("Exiting Drone Fleet Mission Planner\n"); break;
            default: printf("Invalid choice. Try again.\n");
        }
    } while (choice != 7);
}