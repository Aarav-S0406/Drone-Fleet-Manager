#include "algorithms.h"

// --- 1. SORTING & SEARCHING ---
static void merge(Parcel arr[], int l, int m, int r) {
    int i, j, k;
    int n1 = m - l + 1;
    int n2 = r - m;
    Parcel *L = (Parcel *)malloc((size_t)n1 * sizeof(Parcel));
    Parcel *R = (Parcel *)malloc((size_t)n2 * sizeof(Parcel));
    for (i = 0; i < n1; i++) L[i] = arr[l + i];
    for (j = 0; j < n2; j++) R[j] = arr[m + 1 + j];
    i = 0; j = 0; k = l;
    while (i < n1 && j < n2) {
        if (L[i].priority >= R[j].priority) arr[k++] = L[i++];
        else arr[k++] = R[j++];
    }
    while (i < n1) arr[k++] = L[i++];
    while (j < n2) arr[k++] = R[j++];
    free(L); free(R);
}

void merge_sort_parcels(Parcel arr[], int l, int r) {
    if (l < r) {
        int m = l + (r - l) / 2;
        merge_sort_parcels(arr, l, m);
        merge_sort_parcels(arr, m + 1, r);
        merge(arr, l, m, r);
    }
}

static int partition_drones(Drone arr[], int low, int high) {
    double pivot = arr[high].max_battery;
    int i = (low - 1);
    for (int j = low; j <= high - 1; j++) {
        if (arr[j].max_battery >= pivot) {
            i++;
            Drone temp = arr[i]; arr[i] = arr[j]; arr[j] = temp;
        }
    }
    Drone temp = arr[i + 1]; arr[i + 1] = arr[high]; arr[high] = temp;
    return (i + 1);
}

void quick_sort_drones(Drone arr[], int low, int high) {
    if (low < high) {
        int pi = partition_drones(arr, low, high);
        quick_sort_drones(arr, low, pi - 1);
        quick_sort_drones(arr, pi + 1, high);
    }
}

int binary_search_parcel_priority(Parcel arr[], int n, int target_priority) {
    int left = 0, right = n - 1;
    while (left <= right) {
        int mid = left + (right - left) / 2;
        if (arr[mid].priority == target_priority) return mid;
        if (arr[mid].priority < target_priority) right = mid - 1;
        else left = mid + 1;
    }
    return -1;
}

int recursive_binary_search(Parcel arr[], int low, int high, int target) {
    /* Requires parcels sorted by priority in descending order, as produced
       by merge_sort_parcels(). */
    if (low > high) return -1;
    int mid = low + (high - low) / 2;
    if (arr[mid].priority == target) return mid;
    if (arr[mid].priority < target)
        return recursive_binary_search(arr, low, mid - 1, target);
    return recursive_binary_search(arr, mid + 1, high, target);
}

// --- 2. DIVIDE AND CONQUER ---
static double max_crossing_sum(double arr[], int low, int mid, int high, int *start, int *end) {
    double sum = 0;
    double left_sum = -1e9;
    int max_left = mid;
    for (int i = mid; i >= low; i--) {
        sum += arr[i];
        if (sum > left_sum) { left_sum = sum; max_left = i; }
    }
    sum = 0;
    double right_sum = -1e9;
    int max_right = mid + 1;
    for (int i = mid + 1; i <= high; i++) {
        sum += arr[i];
        if (sum > right_sum) { right_sum = sum; max_right = i; }
    }
    *start = max_left; *end = max_right;
    return left_sum + right_sum;
}

double max_subarray_sum(double arr[], int low, int high, int *start, int *end) {
    if (low == high) { *start = low; *end = high; return arr[low]; }
    int mid = (low + high) / 2;
    int left_start, left_end, right_start, right_end, cross_start, cross_end;
    double left_sum = max_subarray_sum(arr, low, mid, &left_start, &left_end);
    double right_sum = max_subarray_sum(arr, mid + 1, high, &right_start, &right_end);
    double cross_sum = max_crossing_sum(arr, low, mid, high, &cross_start, &cross_end);

    if (left_sum >= right_sum && left_sum >= cross_sum) {
        *start = left_start; *end = left_end; return left_sum;
    } else if (right_sum >= left_sum && right_sum >= cross_sum) {
        *start = right_start; *end = right_end; return right_sum;
    } else {
        *start = cross_start; *end = cross_end; return cross_sum;
    }
}

long long fast_exponentiation(long long base, long long exp, long long mod) {
    if (mod <= 0 || exp < 0) return -1;
    long long res = 1;
    base = base % mod;
    while (exp > 0) {
        if (exp % 2 == 1) res = (res * base) % mod;
        exp = exp >> 1;
        base = (base * base) % mod;
    }
    return res;
}

int find_celebrity(int matrix[MAX_NODES][MAX_NODES], int n) {
    if (matrix == NULL || n <= 0 || n > MAX_NODES) return -1;
    int candidate = 0;
    for (int i = 1; i < n; i++) {
        if (matrix[candidate][i]) candidate = i;
    }
    for (int i = 0; i < n; i++) {
        if (i != candidate && (matrix[candidate][i] || !matrix[i][candidate])) return -1;
    }
    return candidate;
}

void strassen_matrix_mult(int n, double A[MAX_NODES][MAX_NODES], double B[MAX_NODES][MAX_NODES], double C[MAX_NODES][MAX_NODES]) {
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            C[i][j] = 0;
            for (int k = 0; k < n; k++) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }
}

// --- 3. GREEDY METHODS ---
int activity_selection(int start[], int finish[], int n, int selected[]) {
    if (n <= 0 || start == NULL || finish == NULL || selected == NULL) return 0;
    int count = 0;
    int i = 0;
    selected[count++] = i;
    for (int j = 1; j < n; j++) {
        if (start[j] >= finish[i]) {
            selected[count++] = j;
            i = j;
        }
    }
    return count;
}

double fractional_knapsack(double capacity, Parcel parcels[], int n) {
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            double r1 = (double)parcels[i].priority / parcels[i].weight;
            double r2 = (double)parcels[j].priority / parcels[j].weight;
            if (r1 < r2) {
                Parcel temp = parcels[i]; parcels[i] = parcels[j]; parcels[j] = temp;
            }
        }
    }
    double total_value = 0.0;
    for (int i = 0; i < n; i++) {
        if (capacity >= parcels[i].weight) {
            capacity -= parcels[i].weight;
            total_value += parcels[i].priority;
        } else {
            total_value += parcels[i].priority * (capacity / parcels[i].weight);
            break;
        }
    }
    return total_value;
}

void huffman_coding(char data[], int freq[], int size) {
    if (size <= 0 || data == NULL || freq == NULL) {
        printf("[Huffman Coding] Invalid or empty telemetry input.\n");
        return;
    }
    int total_freq = 0;
    for (int i = 0; i < size; i++) total_freq += freq[i];

    printf("[Huffman Coding] Symbols: %d | Total weight/freq: %d\n", size, total_freq);
    for (int i = 0; i < size; i++) {
        int code_len = (size == 1) ? 1 : (i + 1);
        if (code_len >= size) code_len = size - 1;
        if (code_len < 1) code_len = 1;
        printf("  Symbol '%c' (freq=%d) -> est. prefix code len: %d bits\n",
               data[i], freq[i], code_len);
    }
}

static bool edge_is_usable(const WorldState *world, const Edge *edge) {
    if (edge->blocked) return false;
    if (edge->u < 0 || edge->u >= world->node_count ||
        edge->v < 0 || edge->v >= world->node_count) return false;
    if (world->nodes[edge->u].is_no_fly_zone ||
        world->nodes[edge->v].is_no_fly_zone) return false;
    return true;
}

void dijkstra(WorldState *world, int src, int dist[], int prev[]) {
    int n = world->node_count;
    bool sptSet[MAX_NODES] = {false};
    if (n < 0 || n > MAX_NODES) return;
    for (int i = 0; i < n; i++) {
        dist[i] = INT_MAX;
        prev[i] = -1;
    }
    if (src < 0 || src >= n || world->nodes[src].is_no_fly_zone) return;

    dist[src] = 0;
    for (int count = 0; count < n; count++) {
        int u = -1;
        int min_dist = INT_MAX;
        for (int v = 0; v < n; v++) {
            if (!sptSet[v] && dist[v] < min_dist) {
                min_dist = dist[v];
                u = v;
            }
        }
        if (u == -1) break; /* Remaining nodes are unreachable. */
        sptSet[u] = true;

        for (int e = 0; e < world->edge_count; e++) {
            Edge *edge = &world->edges[e];
            if (!edge_is_usable(world, edge)) continue;
            int v = -1;
            if (edge->u == u) v = edge->v;
            else if (edge->v == u) v = edge->u;
            if (v >= 0 && !sptSet[v] && edge->weight >= 0.0 &&
                edge->weight <= INT_MAX && dist[u] <= INT_MAX - (int)edge->weight) {
                int candidate = dist[u] + (int)edge->weight;
                if (candidate < dist[v]) {
                    dist[v] = candidate;
                    prev[v] = u;
                }
            }
        }
    }
}

void prim_mst(WorldState *world) {
    int V = world->node_count;
    if (V <= 0 || V > MAX_NODES) return;
    double key[MAX_NODES];
    bool in_mst[MAX_NODES] = {false};
    int parent[MAX_NODES];
    int active_nodes = 0, start_node = -1;

    for (int i = 0; i < V; i++) {
        key[i] = 1e100;
        parent[i] = -1;
        if (!world->nodes[i].is_no_fly_zone) {
            active_nodes++;
            if (start_node == -1) start_node = i;
        }
    }
    if (active_nodes == 0) {
        printf("[Prim's MST] No flyable nodes are available.\n");
        return;
    }
    key[start_node] = 0.0;

    double total_cost = 0.0;
    int edges_used = 0;
    for (int count = 0; count < active_nodes; count++) {
        int u = -1;
        double min_val = 1e100;
        for (int v = 0; v < V; v++) {
            if (!world->nodes[v].is_no_fly_zone && !in_mst[v] && key[v] < min_val) {
                min_val = key[v];
                u = v;
            }
        }
        if (u == -1) break; /* A remaining component is disconnected. */
        in_mst[u] = true;
        if (parent[u] != -1) {
            total_cost += key[u];
            edges_used++;
        }
        for (int e = 0; e < world->edge_count; e++) {
            Edge *edge = &world->edges[e];
            if (!edge_is_usable(world, edge)) continue;
            int v = (edge->u == u) ? edge->v : ((edge->v == u) ? edge->u : -1);
            if (v != -1 && !in_mst[v] && edge->weight < key[v]) {
                key[v] = edge->weight;
                parent[v] = u;
            }
        }
    }
    if (edges_used == active_nodes - 1)
        printf("[Prim's MST] Flyable network connected: %d edges | Total cost: %.2f\n", edges_used, total_cost);
    else
        printf("[Prim's MST] Network disconnected: spanning forest has %d/%d edges | Cost: %.2f\n",
               edges_used, active_nodes - 1, total_cost);
}

static int find_set(int parent[], int x) {
    if (parent[x] == x) return x;
    return parent[x] = find_set(parent, parent[x]);
}

void kruskal_mst(WorldState *world) {
    int V = world->node_count;
    if (V <= 0 || V > MAX_NODES) return;
    int parent[MAX_NODES];
    int active_nodes = 0;
    for (int i = 0; i < V; i++) {
        parent[i] = i;
        if (!world->nodes[i].is_no_fly_zone) active_nodes++;
    }

    int active_count = 0;
    Edge sorted_edges[MAX_NODES * MAX_NODES];
    for (int i = 0; i < world->edge_count; i++) {
        if (edge_is_usable(world, &world->edges[i]) &&
            active_count < MAX_NODES * MAX_NODES)
            sorted_edges[active_count++] = world->edges[i];
    }
    for (int i = 0; i < active_count - 1; i++) {
        for (int j = i + 1; j < active_count; j++) {
            if (sorted_edges[j].weight < sorted_edges[i].weight) {
                Edge tmp = sorted_edges[i];
                sorted_edges[i] = sorted_edges[j];
                sorted_edges[j] = tmp;
            }
        }
    }

    double mst_cost = 0.0;
    int edges_used = 0;
    for (int i = 0; i < active_count && edges_used < active_nodes - 1; i++) {
        int ru = find_set(parent, sorted_edges[i].u);
        int rv = find_set(parent, sorted_edges[i].v);
        if (ru != rv) {
            parent[ru] = rv;
            mst_cost += sorted_edges[i].weight;
            edges_used++;
        }
    }
    if (edges_used == active_nodes - 1)
        printf("[Kruskal's MST] Flyable network connected: %d edges | Total cost: %.2f\n", edges_used, mst_cost);
    else
        printf("[Kruskal's MST] Network disconnected: spanning forest has %d/%d edges | Cost: %.2f\n",
               edges_used, active_nodes > 0 ? active_nodes - 1 : 0, mst_cost);
}

// --- 4. DYNAMIC PROGRAMMING ---
double knapsack_01_dp(int W, double wt[], double val[], int n) {
    int scale = 10;
    int cap = W * scale;
    if (cap <= 0 || n <= 0) return 0.0;

    double **K = (double **)malloc((size_t)(n + 1) * sizeof(double *));
    for (int i = 0; i <= n; i++) {
        K[i] = (double *)calloc((size_t)(cap + 1), sizeof(double));
    }

    for (int i = 1; i <= n; i++) {
        int item_w = (int)llround(wt[i - 1] * scale);
        for (int w = 0; w <= cap; w++) {
            if (item_w >= 0 && item_w <= w) {
                double include_val = val[i - 1] + K[i - 1][w - item_w];
                double exclude_val = K[i - 1][w];
                K[i][w] = (include_val > exclude_val) ? include_val : exclude_val;
            } else {
                K[i][w] = K[i - 1][w];
            }
        }
    }

    double res = K[n][cap];
    for (int i = 0; i <= n; i++) free(K[i]);
    free(K);
    return res;
}

int lcs_length(char *X, char *Y, int m, int n) {
    if (X == NULL || Y == NULL || m < 0 || n < 0) return -1;
    int **L = (int **)calloc((size_t)(m + 1), sizeof(int *));
    if (L == NULL) return -1;
    for (int i = 0; i <= m; i++) {
        L[i] = (int *)calloc((size_t)(n + 1), sizeof(int));
        if (L[i] == NULL) {
            for (int j = 0; j < i; j++) free(L[j]);
            free(L);
            return -1;
        }
    }
    for (int i = 1; i <= m; i++) {
        for (int j = 1; j <= n; j++) {
            if (X[i - 1] == Y[j - 1]) L[i][j] = L[i - 1][j - 1] + 1;
            else L[i][j] = (L[i - 1][j] > L[i][j - 1]) ? L[i - 1][j] : L[i][j - 1];
        }
    }
    int res = L[m][n];
    for (int i = 0; i <= m; i++) free(L[i]);
    free(L);
    return res;
}

double bellman_ford(WorldState *world, int src, int dist[]) {
    int V = world->node_count;
    if (V <= 0 || V > MAX_NODES || src < 0 || src >= V ||
        world->nodes[src].is_no_fly_zone) return -1.0;
    for (int i = 0; i < V; i++) dist[i] = INT_MAX;
    dist[src] = 0;

    for (int pass = 1; pass < V; pass++) {
        bool changed = false;
        for (int j = 0; j < world->edge_count; j++) {
            Edge *edge = &world->edges[j];
            if (!edge_is_usable(world, edge) || edge->weight < 0.0 ||
                edge->weight > INT_MAX) continue;
            int u = edge->u, v = edge->v;
            int w = (int)edge->weight;
            if (dist[u] != INT_MAX && dist[u] <= INT_MAX - w && dist[u] + w < dist[v]) {
                dist[v] = dist[u] + w;
                changed = true;
            }
            if (dist[v] != INT_MAX && dist[v] <= INT_MAX - w && dist[v] + w < dist[u]) {
                dist[u] = dist[v] + w;
                changed = true;
            }
        }
        if (!changed) break;
    }
    /* A negative return value signals a negative cycle; negative edges are
       currently rejected above because this simulator models distances. */
    return 0.0;
}

double floyd_warshall(WorldState *world, double dist[MAX_NODES][MAX_NODES]) {
    int V = world->node_count;
    if (V < 0 || V > MAX_NODES) return -1.0;
    const double INF = 1e100;
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++)
            dist[i][j] = (i == j) ? 0.0 : INF;
    }
    for (int e = 0; e < world->edge_count; e++) {
        Edge *edge = &world->edges[e];
        if (!edge_is_usable(world, edge) || edge->weight < 0.0) continue;
        if (edge->weight < dist[edge->u][edge->v]) {
            dist[edge->u][edge->v] = edge->weight;
            dist[edge->v][edge->u] = edge->weight;
        }
    }
    for (int k = 0; k < V; k++)
        for (int i = 0; i < V; i++)
            for (int j = 0; j < V; j++)
                if (dist[i][k] < INF && dist[k][j] < INF &&
                    dist[i][k] + dist[k][j] < dist[i][j])
                    dist[i][j] = dist[i][k] + dist[k][j];
    return 0.0;
}

double tsp_dp(int mask, int pos, int n, double cost[MAX_NODES][MAX_NODES], int **memo) {
    if (mask == (1 << n) - 1) return cost[pos][0];
    if (memo[mask][pos] != -1) return memo[mask][pos];
    double ans = 1e9;
    for (int city = 0; city < n; city++) {
        if ((mask & (1 << city)) == 0) {
            double newAns = cost[pos][city] + tsp_dp(mask | (1 << city), city, n, cost, memo);
            if (newAns < ans) ans = newAns;
        }
    }
    return memo[mask][pos] = (int)ans;
}

// --- 5. BACKTRACKING & BRANCH AND BOUND ---
static bool is_safe_queen(int board[MAX_NODES][MAX_NODES], int row, int col, int n) {
    int i, j;
    for (i = 0; i < col; i++) if (board[row][i]) return false;
    for (i = row, j = col; i >= 0 && j >= 0; i--, j--) if (board[i][j]) return false;
    for (i = row, j = col; j >= 0 && i < n; i++, j--) if (board[i][j]) return false;
    return true;
}

bool solve_n_queens_util(int board[MAX_NODES][MAX_NODES], int col, int n) {
    if (col >= n) return true;
    for (int i = 0; i < n; i++) {
        if (is_safe_queen(board, i, col, n)) {
            board[i][col] = 1;
            if (solve_n_queens_util(board, col + 1, n)) return true;
            board[i][col] = 0;
        }
    }
    return false;
}

bool subset_sum(int set[], int n, int sum, int subset[], int subset_size) {
    if (sum == 0) return true;
    if (n == 0 && sum != 0) return false;
    if (set[n - 1] > sum) return subset_sum(set, n - 1, sum, subset, subset_size);
    return subset_sum(set, n - 1, sum, subset, subset_size) ||
           subset_sum(set, n - 1, sum - set[n - 1], subset, subset_size);
}

void knapsack_backtracking(int i, double current_weight, double current_val, double max_wt, double wt[], double val[], int n, double *max_val) {
    if (current_weight <= max_wt && current_val > *max_val) *max_val = current_val;
    if (i == n || current_weight >= max_wt) return;
    knapsack_backtracking(i + 1, current_weight + wt[i], current_val + val[i], max_wt, wt, val, n, max_val);
    knapsack_backtracking(i + 1, current_weight, current_val, max_wt, wt, val, n, max_val);
}

// --- 6. FLOW AND MATCHING ---
static bool bfs_flow(int rGraph[MAX_NODES][MAX_NODES], int s, int t, int parent[], int n) {
    bool visited[MAX_NODES] = {false};
    int queue[MAX_NODES], head = 0, tail = 0;
    queue[tail++] = s; visited[s] = true; parent[s] = -1;
    while (head < tail) {
        int u = queue[head++];
        for (int v = 0; v < n; v++) {
            if (!visited[v] && rGraph[u][v] > 0) {
                queue[tail++] = v; parent[v] = u; visited[v] = true;
            }
        }
    }
    return visited[t];
}

int ford_fulkerson(int graph[MAX_NODES][MAX_NODES], int s, int t, int n) {
    int u, v;
    int rGraph[MAX_NODES][MAX_NODES];
    for (u = 0; u < n; u++)
        for (v = 0; v < n; v++)
            rGraph[u][v] = graph[u][v];
    int parent[MAX_NODES];
    int max_flow = 0;
    while (bfs_flow(rGraph, s, t, parent, n)) {
        int path_flow = INT_MAX;
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            if (rGraph[u][v] < path_flow) path_flow = rGraph[u][v];
        }
        for (v = t; v != s; v = parent[v]) {
            u = parent[v];
            rGraph[u][v] -= path_flow;
            rGraph[v][u] += path_flow;
        }
        max_flow += path_flow;
    }
    return max_flow;
}

static bool bpm_dfs(int bp_graph[MAX_NODES][MAX_NODES], int u, bool seen[], int match[], int n) {
    for (int v = 0; v < n; v++) {
        if (bp_graph[u][v] && !seen[v]) {
            seen[v] = true;
            if (match[v] < 0 || bpm_dfs(bp_graph, match[v], seen, match, n)) {
                match[v] = u;
                return true;
            }
        }
    }
    return false;
}

int max_bipartite_matching(int bp_graph[MAX_NODES][MAX_NODES], int m, int n, int match[]) {
    int count = 0;
    for (int i = 0; i < n; i++) match[i] = -1;
    for (int u = 0; u < m; u++) {
        bool seen[MAX_NODES] = {false};
        if (bpm_dfs(bp_graph, u, seen, match, n)) {
            count++;
        }
    }
    return count;
}

// --- 7. NUMBER THEORY ---
long long chinese_remainder_theorem(long long num[], long long rem[], int k) {
    long long prod = 1;
    for (int i = 0; i < k; i++) prod *= num[i];
    long long result = 0;
    for (int i = 0; i < k; i++) {
        long long pp = prod / num[i];
        long long inv = fast_exponentiation(pp, num[i] - 2, num[i]);
        result += rem[i] * pp * inv;
    }
    return result % prod;
}

// --- 8. STRING MATCHING ---
void naive_string_match(char *text, char *pattern) {
    int n = (int)strlen(text);
    int m = (int)strlen(pattern);
    if (m == 0 || m > n) return;
    for (int i = 0; i <= n - m; i++) {
        int j;
        for (j = 0; j < m; j++) if (text[i + j] != pattern[j]) break;
        if (j == m) printf("[Naive Match] Found pattern at index %d\n", i);
    }
}

void rabin_karp_match(char *text, char *pattern, int q) {
    int d = 256;
    int n = (int)strlen(text);
    int m = (int)strlen(pattern);
    if (q <= 0 || m == 0 || m > n) return;
    int i, j;
    int p = 0, t = 0, h = 1;
    for (i = 0; i < m - 1; i++) h = (h * d) % q;
    for (i = 0; i < m; i++) {
        p = (d * p + pattern[i]) % q;
        t = (d * t + text[i]) % q;
    }
    for (i = 0; i <= n - m; i++) {
        if (p == t) {
            for (j = 0; j < m; j++) if (text[i + j] != pattern[j]) break;
            if (j == m) printf("[Rabin-Karp] Found pattern at index %d\n", i);
        }
        if (i < n - m) {
            t = (d * (t - text[i] * h) + text[i + m]) % q;
            if (t < 0) t = (t + q);
        }
    }
}

void kmp_match(char *text, char *pattern) {
    int m = (int)strlen(pattern);
    int n = (int)strlen(text);
    if (m == 0 || m > n || m > MAX_STRING) return;
    int lps[MAX_STRING];
    int len = 0; lps[0] = 0;
    int i = 1;
    while (i < m) {
        if (pattern[i] == pattern[len]) { len++; lps[i] = len; i++; }
        else {
            if (len != 0) len = lps[len - 1];
            else { lps[i] = 0; i++; }
        }
    }
    i = 0; int j = 0;
    while (i < n) {
        if (pattern[j] == text[i]) { i++; j++; }
        if (j == m) { printf("[KMP Match] Found pattern at index %d\n", i - j); j = lps[j - 1]; }
        else if (i < n && pattern[j] != text[i]) {
            if (j != 0) j = lps[j - 1];
            else i++;
        }
    }
}