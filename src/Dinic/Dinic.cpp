#include <Dinic.h>

Mat Dinic::dinic_solver(Mat& Map) {
    /*while (scanf_s("%d%d%d", &R, &C, &N) != EOF && (R || C || N)) {
        dinic.init(R + C + 5);
        int r, c;
        for (int i = 1; i <= N; i++) {
            scanf_s("%d%d", &r, &c);
            dinic.add_edge(r, c + R, 1);
        }
        for (int i = 1; i <= R; i++) dinic.add_edge(0, i, 1);
        for (int i = 1; i <= C; i++) dinic.add_edge(i + R, C + R + 1, 1);
        int ans = dinic.Maxflow(0, C + R + 1);
        printf("%d ", ans);
        dinic.get_cut(0);
        for (int i = 1; i <= R; i++) if (!dinic.iscut[i])  printf("r%d ", i);
        for (int i = 1; i <= C; i++) if (dinic.iscut[i + R]) printf("c%d ", i);
        printf("\n");
    }*/
    clock_t start, finish;
    start = clock();
    int n = Map.size(), m = Map[0].size();

    dinic.init(SINK + 5);
    int res = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            if (Map[i][j]) {
                res++;
                // connect source and sink
                if (i > 0 && Map[i - 1][j]) {
                    res--;
                    pts[HORZ(i - 1, j)] = true;
                    dinic.add_edge(SOURCE, HORZ(i - 1, j), 1);
                }
                if (j > 0 && Map[i][j - 1]) {
                    res--;
                    pts[VERT(i, j - 1)] = true;
                    dinic.add_edge(VERT(i, j - 1), SINK, 1);
                }

                // middle
                if (i < n - 1 && j < m - 1 && Map[i + 1][j] && Map[i][j + 1]) {
                    dinic.add_edge(HORZ(i, j), VERT(i, j), 1);
                    pts[HORZ(i, j)] = pts[VERT(i, j)] = true;
                }
                if (i > 0 && j < m - 1 && Map[i - 1][j] && Map[i][j + 1]) {
                    dinic.add_edge(HORZ(i - 1, j), VERT(i, j), 1);
                    pts[HORZ(i - 1, j)] = pts[VERT(i, j)] = true;
                }
                if (i < n - 1 && j > 0 && Map[i + 1][j] && Map[i][j - 1]) {
                    dinic.add_edge(HORZ(i, j), VERT(i, j - 1), 1);
                    pts[HORZ(i, j)] = pts[VERT(i, j - 1)] = true;
                }
                if (i > 0 && j > 0 && Map[i - 1][j] && Map[i][j - 1]) {
                    dinic.add_edge(HORZ(i - 1, j), VERT(i, j - 1), 1);
                    pts[HORZ(i - 1, j)] = pts[VERT(i, j - 1)] = true;
                }
            }
        }
    }
    cout << res + dinic.Maxflow(SOURCE, SINK) << "\n";
    dinic.get_cut(SOURCE);

    int tot = Map.size() * Map[0].size();
    fa.resize(tot + 5);
    for (int i = 0; i < tot; ++i)	fa[i] = i;
    Mat MST(tot, vector<int>{});

    // horizontal node, make vertical edges
    for (int i = 1; i <= (n - 1) * m; ++i) {
        P p = getEdgeCoor(i, Map);
        if (dinic.iscut[i] && pts[i]) {
            // cout << i << " H: " << p.first << ", " << p.second << "\n";
            MST[RESHAPE(p.first, p.second)].push_back(RESHAPE(p.first + 1, p.second));
            MST[RESHAPE(p.first + 1, p.second)].push_back(RESHAPE(p.first, p.second));
            unite(RESHAPE(p.first, p.second), RESHAPE(p.first + 1, p.second));
        }
    }

    // vertical node, make horizontal edges
    for (int i = 1; i <= n * (m - 1); ++i) {
        P p = getEdgeCoor(i + (n - 1) * m, Map);
        if (!dinic.iscut[i + (n - 1) * m] && pts[i + (n - 1) * m]) {
            // cout << i + (n - 1) * m << " V: " << p.first << ", " << p.second << "\n";
            MST[RESHAPE(p.first, p.second)].push_back(RESHAPE(p.first, p.second + 1));
            MST[RESHAPE(p.first, p.second + 1)].push_back(RESHAPE(p.first, p.second));
            unite(RESHAPE(p.first, p.second), RESHAPE(p.first, p.second + 1));
        }
    }

    mergeMST(MST, Map);
    //cout << pts.size() << endl;
    finish = clock();

    int totalTurns = 0;
    for (int i = 0; i < MST.size(); ++i) {
        totalTurns += getVertexVal(MST, i);
    }
    cout << "rect dinic MST number of turns: " << totalTurns << endl;

    checkMST(MST, Map);

    cout << "dinic total used time: " << finish - start << "\n";
    cout << "-------------------Dinic Solver End-------------------\n\n";

    return MST;
}