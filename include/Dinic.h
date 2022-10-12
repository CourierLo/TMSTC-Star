#ifndef _DINIC_H
#define _DINIC_H

#include <iostream>
#include <cstdio>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <cstring>

using std::queue; 
using std::vector;
using std::priority_queue;
using std::cout;
using std::endl;
using std::pair;

#define SOURCE 0
#define HORZ(i, j) (m * (i) + (j) + 1)
#define VERT(i, j) ((n - 1) * m + n * (j) + (i) + 1)
#define SINK ((n - 1) * m + n * (m - 1) + 1)
#define RESHAPE(i, j) (int)((i) * Map[0].size() + (j))

typedef vector<vector<int>> Mat;
typedef pair<int, int> P;
const int nmax = 1000000 + 10;
const int INF = 0x3f3f3f3f;

class Dinic {
    int head[nmax], cur[nmax], d[nmax];
    bool vis[nmax], iscut[nmax];
    int tot, n, m, s, t;
    bool pts[nmax];
    vector<int> fa;

    struct edge {
        int nxt, to, w, cap, flow;
    } e[nmax << 1];

    struct tree_edge {
        int from;
        int to;
        int cost;
        bool operator < (const tree_edge& e) const {
            return cost > e.cost;
        }
    };

public:
    void init(int n) {
        this->n = n;
        this->tot = 0;
        memset(head, -1, sizeof head);
        memset(iscut, 0, sizeof iscut);
        memset(pts, false, sizeof pts);
    }

    void add_edge(int u, int v, int c) {
        e[tot].to = v, e[tot].cap = c, e[tot].flow = 0;
        e[tot].nxt = head[u];
        head[u] = tot++;

        e[tot].to = u, e[tot].cap = c, e[tot].flow = c;
        e[tot].nxt = head[v];
        head[v] = tot++;
    }

    bool BFS() {
        memset(vis, 0, sizeof(vis));
        queue<int>Q;
        vis[s] = 1; d[s] = 0;
        Q.push(s);
        while (!Q.empty()) {
            int u = Q.front(); Q.pop();
            for (int i = head[u]; i != -1; i = e[i].nxt) {
                int v = e[i].to;
                if (!vis[v] && e[i].cap > e[i].flow) {
                    vis[v] = 1;
                    d[v] = d[u] + 1;
                    Q.push(v);
                }
            }
        }
        return vis[t];
    }

    int DFS(int x, int a) {
        if (x == t || a == 0) return a;
        int Flow = 0, f;
        for (int& i = cur[x]; i != -1; i = e[i].nxt) {
            int v = e[i].to;
            if (d[v] == d[x] + 1 && (f = DFS(v, std::min(a, e[i].cap - e[i].flow))) > 0) {
                Flow += f;
                e[i].flow += f;
                e[i ^ 1].flow -= f;
                a -= f;
                if (a == 0) break;
            }
        }
        return Flow;
    }

    int Maxflow(int s, int t) {
        this->s = s, this->t = t;
        int Flow = 0;
        while (BFS()) {
            for (int i = 0; i <= n; i++) cur[i] = head[i];
            while (int once = DFS(s, INF)) Flow += once;
        }
        return Flow;
    }

    void get_cut(int u) {
        iscut[u] = true;
        for (int i = head[u]; i != -1; i = e[i].nxt) {
            int v = e[i].to;
            if (!iscut[v] && e[i].cap > e[i].flow)
                get_cut(v);
        }
        return;
    }

    P getEdgeCoor(int index, Mat& Map) {
        int n = Map.size(), m = Map[0].size();
        if (index <= (n - 1) * m)    return { (index - 1) / m, (index - 1) % m };
        else  return { (index - 1 - (n - 1) * m) % n,  (index - 1 - (n - 1) * m) / n };
    }

    Mat dinic_solver(Mat& Map);
        // /*while (scanf_s("%d%d%d", &R, &C, &N) != EOF && (R || C || N)) {
        //     dinic.init(R + C + 5);
        //     int r, c;
        //     for (int i = 1; i <= N; i++) {
        //         scanf_s("%d%d", &r, &c);
        //         dinic.add_edge(r, c + R, 1);
        //     }
        //     for (int i = 1; i <= R; i++) dinic.add_edge(0, i, 1);
        //     for (int i = 1; i <= C; i++) dinic.add_edge(i + R, C + R + 1, 1);
        //     int ans = dinic.Maxflow(0, C + R + 1);
        //     printf("%d ", ans);
        //     dinic.get_cut(0);
        //     for (int i = 1; i <= R; i++) if (!dinic.iscut[i])  printf("r%d ", i);
        //     for (int i = 1; i <= C; i++) if (dinic.iscut[i + R]) printf("c%d ", i);
        //     printf("\n");
        // }*/
        // clock_t start, finish;
        // start = clock();
        // int n = Map.size(), m = Map[0].size();

        // dinic.init(SINK + 5);
        // int res = 0;
        // for (int i = 0; i < n; ++i) {
        //     for (int j = 0; j < m; ++j) {
        //         if (Map[i][j]) {
        //             res++;
        //             // connect source and sink
        //             if (i > 0 && Map[i - 1][j]) {
        //                 res--;
        //                 pts[HORZ(i - 1, j)] = true;
        //                 dinic.add_edge(SOURCE, HORZ(i - 1, j), 1);
        //             }
        //             if (j > 0 && Map[i][j - 1]) {
        //                 res--;
        //                 pts[VERT(i, j - 1)] = true;
        //                 dinic.add_edge(VERT(i, j - 1), SINK, 1);
        //             }

        //             // middle
        //             if (i < n - 1 && j < m - 1 && Map[i + 1][j] && Map[i][j + 1]) {
        //                 dinic.add_edge(HORZ(i, j), VERT(i, j), 1);
        //                 pts[HORZ(i, j)] = pts[VERT(i, j)] = true;
        //             }
        //             if (i > 0 && j < m - 1 && Map[i - 1][j] && Map[i][j + 1]) {
        //                 dinic.add_edge(HORZ(i - 1, j), VERT(i, j), 1);
        //                 pts[HORZ(i - 1, j)] = pts[VERT(i, j)] = true;
        //             }
        //             if (i < n - 1 && j > 0 && Map[i + 1][j] && Map[i][j - 1]) {
        //                 dinic.add_edge(HORZ(i, j), VERT(i, j - 1), 1);
        //                 pts[HORZ(i, j)] = pts[VERT(i, j - 1)] = true;
        //             }
        //             if (i > 0 && j > 0 && Map[i - 1][j] && Map[i][j - 1]) {
        //                 dinic.add_edge(HORZ(i - 1, j), VERT(i, j - 1), 1);
        //                 pts[HORZ(i - 1, j)] = pts[VERT(i, j - 1)] = true;
        //             }
        //         }
        //     }
        // }
        // cout << res + dinic.Maxflow(SOURCE, SINK) << "\n";
        // dinic.get_cut(SOURCE);

        // int tot = Map.size() * Map[0].size();
        // fa.resize(tot + 5);
        // for (int i = 0; i < tot; ++i)	fa[i] = i;
        // Mat MST(tot, vector<int>{});

        // // horizontal node, make vertical edges
        // for (int i = 1; i <= (n - 1) * m; ++i) {
        //     P p = getEdgeCoor(i, Map);
        //     if (dinic.iscut[i] && pts[i]) {
        //         // cout << i << " H: " << p.first << ", " << p.second << "\n";
        //         MST[RESHAPE(p.first, p.second)].push_back(RESHAPE(p.first + 1, p.second));
        //         MST[RESHAPE(p.first + 1, p.second)].push_back(RESHAPE(p.first, p.second));
        //         unite(RESHAPE(p.first, p.second), RESHAPE(p.first + 1, p.second));
        //     }
        // }

        // // vertical node, make horizontal edges
        // for (int i = 1; i <= n * (m - 1); ++i) {
        //     P p = getEdgeCoor(i + (n - 1) * m, Map);
        //     if (!dinic.iscut[i + (n - 1) * m] && pts[i + (n - 1) * m]) {
        //         // cout << i + (n - 1) * m << " V: " << p.first << ", " << p.second << "\n";
        //         MST[RESHAPE(p.first, p.second)].push_back(RESHAPE(p.first, p.second + 1));
        //         MST[RESHAPE(p.first, p.second + 1)].push_back(RESHAPE(p.first, p.second));
        //         unite(RESHAPE(p.first, p.second), RESHAPE(p.first, p.second + 1));
        //     }
        // }

        // mergeMST(MST, Map);
        // //cout << pts.size() << endl;
        // finish = clock();

        // int totalTurns = 0;
        // for (int i = 0; i < MST.size(); ++i) {
        //     totalTurns += getVertexVal(MST, i);
        // }
        // cout << "rect dinic MST number of turns: " << totalTurns << endl;

        // checkMST(MST, Map);

        // cout << "dinic total used time: " << finish - start << "\n";
        // cout << "-------------------Dinic Solver End-------------------\n\n";

        // return MST;
    

    bool checkMST(Mat& graph, Mat& Map) {
        int vertexNum = 0;
        for (int i = 0; i < Map.size(); ++i)
            for (int j = 0; j < Map[0].size(); ++j)
                if (Map[i][j])	vertexNum++;

        int edgeNum = 0;
        for (int i = 0; i < graph.size(); ++i)	edgeNum += graph[i].size();
        edgeNum /= 2;

        cout << "The number of edges: " << edgeNum << endl;
        cout << "The number of vertexes: " << vertexNum << endl;
        if (edgeNum != vertexNum - 1)	return false;
        return true;
    }

    void mergeMST(Mat& graph, Mat& Map) {
        vector<tree_edge> edges;   //双向边
        priority_queue<tree_edge> que;
        for (int i = 0; i < Map.size(); ++i) {
            for (int j = 0; j < Map[0].size(); ++j) {
                if (j < Map[0].size() - 1)
                    if (!same(RESHAPE(i, j), RESHAPE(i, j + 1)) && Map[i][j] && Map[i][j + 1])	edges.push_back({ RESHAPE(i, j), RESHAPE(i, j + 1), 0 });

                if (i < Map.size() - 1)
                    if (!same(RESHAPE(i, j), RESHAPE(i + 1, j)) && Map[i][j] && Map[i + 1][j])	edges.push_back({ RESHAPE(i, j), RESHAPE(i + 1, j), 0 });
            }
        }

        for (int i = 0; i < edges.size(); ++i) {
            // 求每条边带来的增量
            // 度为零，增量为0；度为1，增量-2或0；度为2，增量0或2；度为3，增量2；度为4不可连接。 算边的两端的增量
            edges[i].cost = getEdgeVal(graph, edges[i].from, edges[i].to);
            que.push(edges[i]);
        }

        //sort(edge.begin(), edge.end());
        // 当然可以提前结束
        //int cnt = 0;
        while (!que.empty()) {
            tree_edge curEdge = que.top();  que.pop();
            int v1 = curEdge.from, v2 = curEdge.to;

            if (same(v1, v2))	continue;

            int curVal = getEdgeVal(graph, v1, v2);
            if (curEdge.cost != curVal) {
                curEdge.cost = curVal;
                que.push(curEdge);
                continue;
            }

            // 可以继续优化，随着MST连接，某些边的cost会发生变化，可以多用一个heap维护之
            graph[v1].push_back(v2);
            graph[v2].push_back(v1);
            unite(v1, v2);

            //cnt++;
        }

    }

    // utility function
    int find(int x) {
        return fa[x] == x ? x : fa[x] = find(fa[x]);
    }

    bool same(int x, int y) {
        return find(x) == find(y);
    }

    void unite(int x, int y) {
        x = find(x);  y = find(y);
        if (x == y)	return;
        fa[x] = y;		// 不用rank作平衡了
    }

    bool isSameLine(int a, int b, int c) {
        return a + c == 2 * b;
    }

    int getEdgeVal(Mat& graph, int v1, int v2) {
        int cost = 0;
        if (graph[v1].size() == 1) {
            if (isSameLine(graph[v1][0], v1, v2))	cost -= 2;
        }
        else if (graph[v1].size() == 2) {
            if (isSameLine(graph[v1][0], v1, graph[v1][1]))	cost += 2;
        }
        else if (graph[v1].size() == 3) {
            cost += 2;
        }

        if (graph[v2].size() == 1) {
            if (isSameLine(v1, v2, graph[v2][0]))	cost -= 2;
        }
        else if (graph[v2].size() == 2) {
            if (isSameLine(graph[v2][0], v2, graph[v2][1]))	cost += 2;
        }
        else if (graph[v1].size() == 3) {
            cost += 2;
        }

        return cost;
    }

    int getVertexVal(Mat& graph, int v) {
        if (graph[v].size() == 0)	return 0;
        else if (graph[v].size() == 1)	return 2;
        else if (graph[v].size() == 2) {
            return isSameLine(graph[v][0], v, graph[v][1]) ? 0 : 2;
        }
        else if (graph[v].size() == 3) return 2;
        else  return 4;
    }

} dinic;

#endif