#ifndef _DINIC_H
#define _DINIC_H

#include <iostream>
#include <cstdio>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <cstring>
#include <map>

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

    struct Brick{
        vector<int> pts;
        vector<P> corner{{}, {}, {}, {}};
        vector<P> mid_pts{{}, {}};
    };

    struct tree_edge {
        int from;
        int to;
        int cost;
        bool operator < (const tree_edge& e) const {
            return cost > e.cost;
        }
    };

public:
    vector<Brick> bricks;
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

    Mat dinic_solver(Mat& Map, bool merge);

    void formBricksForMTSP(Mat& Map){
        bricks.clear();
        int cnt = 0;
        std::map<int, int> label;
        for(int i = 0; i < Map.size() * Map[0].size(); ++i){
            if(!Map[i / Map[0].size()][i % Map[0].size()])  continue;

            int parent = find(i);
            if(label.count(parent)){
                bricks[label[parent]].pts.push_back(i);
            } else {
                bricks.push_back({});
                label[parent] = cnt;
                bricks[cnt].pts.push_back(i);
                cnt++;
            }
        }

        cout << "The number of bricks: " << bricks.size() << "\n";
        for(int i = 0; i < bricks.size(); ++i){
            if(bricks[i].pts.size() == 1){
                // single cell
                int x = bricks[i].pts[0] / Map[0].size();
                int y = bricks[i].pts[0] % Map[0].size();
                bricks[i].corner[0] = { 2 * x, 2 * y };
                bricks[i].corner[1] = { 2 * x, 2 * y + 1 };
                bricks[i].corner[2] = { 2 * x + 1, 2 * y };
                bricks[i].corner[3] = { 2 * x + 1, 2 * y + 1 };
            } else {
                sort(bricks[i].pts.begin(), bricks[i].pts.end());
                // vertical or horizontal ? 
                int x1 = bricks[i].pts[0] / Map[0].size();
                int x2 = bricks[i].pts.back() / Map[0].size();
                int y1 = bricks[i].pts[0] % Map[0].size();
                int y2 = bricks[i].pts.back() % Map[0].size();
                if(x1 == x2){
                    // horizontal
                    bricks[i].corner[0] = { 2 * x1, 2 * y1 };
                    bricks[i].corner[1] = { 2 * x2, 2 * y2 + 1 };
                    bricks[i].corner[2] = { 2 * x1 + 1, 2 * y1 };
                    bricks[i].corner[3] = { 2 * x2 + 1, 2 * y2 + 1 };
                } else {
                    // vertical
                    bricks[i].corner[0] = { 2 * x1, 2 * y1 };
                    bricks[i].corner[1] = { 2 * x2 + 1, 2 * y2};
                    bricks[i].corner[2] = { 2 * x1, 2 * y1 + 1 };
                    bricks[i].corner[3] = { 2 * x2 + 1, 2 * y2 + 1 };
                }

                // compute mid pts
                bricks[i].mid_pts[0] = { (bricks[i].corner[0].first + bricks[i].corner[1].first) / 2,
                                         (bricks[i].corner[0].second + bricks[i].corner[1].second) / 2 };
                bricks[i].mid_pts[1] = { (bricks[i].corner[2].first + bricks[i].corner[3].first) / 2,
                                         (bricks[i].corner[2].second + bricks[i].corner[3].second) / 2 };
            }
        }
    }

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

            graph[v1].push_back(v2);
            graph[v2].push_back(v1);
            unite(v1, v2);
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