#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#define N 10
#define M 20
#define MIN_W 1
#define MAX_W 100
#define INF 1000000000

/* 인접 행렬: w[u][v] == 0 이면 간선 없음 (가중치는 항상 1 이상) */
static int w[N][N];

static int rand_range(int lo, int hi) {
    return lo + rand() % (hi - lo + 1);
}

/* 연결된 가중치 무방향 단순 그래프 생성 */
static void generate_graph(void) {
    int order[N], i, count = 0;

    /* 1) 무작위 신장 트리로 연결성 보장 (간선 N-1개) */
    for (i = 0; i < N; i++) order[i] = i;
    for (i = N - 1; i > 0; i--) {          /* Fisher-Yates 셔플 */
        int j = rand() % (i + 1);
        int t = order[i]; order[i] = order[j]; order[j] = t;
    }
    for (i = 1; i < N; i++) {
        int u = order[i];
        int v = order[rand() % i];         /* 이미 연결된 정점 중 하나 */
        int wt = rand_range(MIN_W, MAX_W);
        w[u][v] = w[v][u] = wt;
        count++;
    }

    /* 2) 자기 간선/중복 간선 없이 나머지 간선 추가 */
    while (count < M) {
        int u = rand() % N;
        int v = rand() % N;
        if (u == v) continue;              /* self-loop 금지 */
        if (w[u][v] != 0) continue;        /* 중복 간선 금지 */
        w[u][v] = w[v][u] = rand_range(MIN_W, MAX_W);
        count++;
    }
}

/* src에서 모든 정점까지 Dijkstra, prev[]에 이전 정점 저장 */
static void dijkstra(int src, int dist[N], int prev[N]) {
    int visited[N] = {0};
    int i, k;

    for (i = 0; i < N; i++) { dist[i] = INF; prev[i] = -1; }
    dist[src] = 0;

    for (k = 0; k < N; k++) {
        int u = -1;
        for (i = 0; i < N; i++)
            if (!visited[i] && (u == -1 || dist[i] < dist[u])) u = i;
        if (u == -1 || dist[u] == INF) break;
        visited[u] = 1;

        for (i = 0; i < N; i++) {
            if (w[u][i] != 0 && !visited[i] && dist[u] + w[u][i] < dist[i]) {
                dist[i] = dist[u] + w[u][i];
                prev[i] = u;
            }
        }
    }
}

/* prev[]를 따라 dst -> src 순으로 거슬러 올라가 경로 복원 */
static int restore_path(const int prev[N], int src, int dst, int path[N]) {
    int tmp[N], len = 0, cur = dst, i;
    while (cur != -1) {
        tmp[len++] = cur;
        cur = prev[cur];
    }
    if (tmp[len - 1] != src) return 0;     /* 경로 없음 */
    for (i = 0; i < len; i++) path[i] = tmp[len - 1 - i];
    return len;
}

/* 연결성 검사 (DFS) */
static int is_connected(void) {
    int seen[N] = {0}, stack[N], top = 0, cnt = 1, i;
    stack[top++] = 0;
    seen[0] = 1;
    while (top > 0) {
        int u = stack[--top];
        for (i = 0; i < N; i++) {
            if (w[u][i] != 0 && !seen[i]) {
                seen[i] = 1;
                cnt++;
                stack[top++] = i;
            }
        }
    }
    return cnt == N;
}

int main(void) {
    int u, v, i;
    int edge_cnt = 0, loop_free = 1, sym_ok = 1, weight_ok = 1;
    int pair_cnt = 0, dist_ok = 1;

    srand((unsigned)time(NULL));
    generate_graph();

    /* ---- 그래프 출력 ---- */
    for (u = 0; u < N; u++)
        for (v = u + 1; v < N; v++)
            if (w[u][v] != 0) edge_cnt++;

    printf("Graph\n");
    printf("Vertices : %d\n", N);
    printf("Edges    : %d\n\n", edge_cnt);
    printf("Edges\n");
    for (u = 0; u < N; u++)
        for (v = u + 1; v < N; v++)
            if (w[u][v] != 0)
                printf("(%d, %d) Weight: %d\n", u, v, w[u][v]);
    printf("\n");

    /* ---- 모든 정점 쌍 최단경로 (각 정점에서 Dijkstra 반복) ---- */
    printf("Shortest Paths\n\n");
    for (u = 0; u < N; u++) {
        int dist[N], prev[N];
        dijkstra(u, dist, prev);

        for (v = u + 1; v < N; v++) {          /* u < v 인 경우만 */
            int path[N];
            int len = restore_path(prev, u, v, path);
            int sum = 0;

            for (i = 0; i + 1 < len; i++)
                sum += w[path[i]][path[i + 1]];
            if (len == 0 || sum != dist[v]) dist_ok = 0;

            printf("%d -> %d\n", u, v);
            printf("Path     : ");
            for (i = 0; i < len; i++) {
                if (i > 0) printf(" -> ");
                printf("%d", path[i]);
            }
            printf("\nDistance : %d\n\n", dist[v]);
            pair_cnt++;
        }
    }
    printf("Total vertex pairs : %d\n\n", pair_cnt);

    /* ---- 확인 사항 ---- */
    for (u = 0; u < N; u++) {
        if (w[u][u] != 0) loop_free = 0;
        for (v = 0; v < N; v++) {
            if (w[u][v] != w[v][u]) sym_ok = 0;
            if (w[u][v] != 0 && (w[u][v] < MIN_W || w[u][v] > MAX_W))
                weight_ok = 0;
        }
    }

    printf("Verification\n");
    printf("정점 10개          : %s\n", N == 10 ? "True" : "False");
    printf("간선 20개          : %s\n", edge_cnt == 20 ? "True" : "False");
    printf("가중치 1~100       : %s\n", weight_ok ? "True" : "False");
    printf("자기 간선/중복 없음: %s\n", (loop_free && sym_ok) ? "True" : "False");
    printf("연결 그래프        : %s\n", is_connected() ? "True" : "False");
    printf("정점 쌍 45개 출력  : %s\n", pair_cnt == 45 ? "True" : "False");
    printf("거리=경로 가중치 합: %s\n", dist_ok ? "True" : "False");

    return 0;
}
