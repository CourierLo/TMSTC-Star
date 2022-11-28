#include <m_TSP.h>
#include <UnionFind.h>

void MTSP::GANextGeneration(){
    cur_gen++;
    selection();
    crossover();
    mutation();
    getBestIndividual();
}

void MTSP::selection(){
    Mat parents;
    parents.push_back(ppl[cur_best_id]);
    parents.push_back(best_mtsp);
    parents.push_back(reverseMutate(best_mtsp));
    parents.push_back((best_mtsp));

    setRoulette();

    for(int i = 4; i < ppl.size(); ++i){
        parents.push_back(ppl[wheelOut(_random())]);
    }
    ppl = parents;
}

void MTSP::crossover(){
    vector<int> cross_id;
    for(int i = 0; i < ppl.size(); ++i){
        if((_random() < conf.crossover_pb))   cross_id.push_back(i);
    }

    // std::random_device rd;
    // std::mt19937 g(rd());
    std::shuffle(cross_id.begin(), cross_id.end(), generator);
    for(int i = 0, j = cross_id.size() - 1; i < j; i += 2){
        crossoverOnce(cross_id[i], cross_id[i + 1]);
    }

}

void MTSP::crossoverOnce(int x, int y){
    vector<int> ch1, ch2;
    ch1 = getChild(true, x, y);
    ch2 = getChild(false, x, y);
    ppl[x] = ch1;
    ppl[y] = ch2;
}

vector<int> MTSP::getChild(bool forward, int x, int y){
    vector<int> child, px, py;
    px = ppl[x];
    py = ppl[y];
    vector<int>::iterator dx, dy;       // maybe a bi-rect link would be faster
    int pos = _randomInt(px.size());
    int c = px[pos];
    child.push_back(c);
    while(px.size() > 1){
        dx = find(px.begin(), px.end(), c);
        dy = find(py.begin(), py.end(), c);
        if(forward){
            dx = dx + 1 == px.end() ? px.begin() : dx + 1;
            dy = dy + 1 == py.end() ? py.begin() : dy + 1;
        } else {
            dx = dx == px.begin() ? px.end() - 1 : dx - 1;
            dy = dy == py.begin() ? py.end() - 1 : dy - 1;
        }
        int t = c;
        c = dis[c][*dx] < dis[c][*dy] ? *dx : *dy;
        px.erase(find(px.begin(), px.end(), t));
        py.erase(find(py.begin(), py.end(), t));
        child.push_back(c);
    }
    return child;
}

void MTSP::mutation(){
    for(int i = 0; i < ppl.size(); ++i){
        if(_random() < conf.mutation_pb){
            if(_random() > 0.5)  ppl[i] = concatMutate(ppl[i]);
            else  ppl[i] = reverseMutate(ppl[i]);

            i--;  // Again
        }
    }
}

void MTSP::setRoulette(){
    for(int i = 0; i < vals.size(); ++i)    fitness_vals[i] = 1.0 / vals[i];

    double sum = 0;
    for(int i = 0; i < vals.size(); ++i)        sum += fitness_vals[i];
    for(int i = 0; i < roulette.size(); ++i)    roulette[i] = fitness_vals[i] / sum;
    for(int i = 1; i < roulette.size(); ++i)    roulette[i] += roulette[i - 1];
}

int MTSP::wheelOut(double rand){
    for(int i = 0; i < roulette.size(); ++i){
        if(rand <= roulette[i]) return i;
    }
}

double MTSP::_random(){
    // std::random_device rd;
    // std::mt19937 generator(rd());
    std::uniform_real_distribution<double> unif(0.0, 1.0);
    return unif(generator);
}
 
int MTSP::_randomInt(int bound){
    return (int)(_random() * bound);
}

vector<int> MTSP::reverseMutate(vector<int> individual){
    mutation_time++;
    
    int n, m;
    do{
        m = _randomInt(individual.size() - 2);
        n = _randomInt(individual.size());
    } while (m >= n);

    for(int i = 0, j = (n - m + 1) / 2; i < j; i++){
        std::swap(individual[m + i], individual[n - i]);
    }
    
    return individual;
}

vector<int> MTSP::concatMutate(vector<int> individual){
    mutation_time++;
    int m, n;
    do{
        m = _randomInt(individual.size() / 2);
        n = _randomInt(individual.size());
    } while (m >= n);

    vector<int> splice;
    for(int i = m; i < n; ++i)                  splice.push_back(individual[i]);
    for(int i = 0; i < m; ++i)                  splice.push_back(individual[i]);
    for(int i = n; i < individual.size(); ++i)  splice.push_back(individual[i]);

    return splice;
}

void MTSP::getBestIndividual(){
    for(int i = 0; i < ppl.size(); ++i)
        vals[i] = evaluate(ppl[i]);

    cur_best_val = vals[0];
    cur_best_id = 0;
    for(int i = 1; i < vals.size(); ++i){
        if(cur_best_val > vals[i]){
            cur_best_val = vals[i];
            cur_best_id = i;
        }
    }

    if(best_val == -1 || best_val > cur_best_val){
        best_mtsp = ppl[cur_best_id];
        best_val = cur_best_val;
        unchanged_gens = 0;
    } else {
        unchanged_gens++;
    }
}

double MTSP::evaluate(vector<int> &individual){
    vector<double> sum_tbl(conf.sales_men, 0.0);
    uint64_t tot = 0;
    int cur = 0;

    // cout << "individual: ";
    // for(int i = 0; i < individual.size(); ++i)  
    //     cout << individual[i] << " ";
    // cout << "\n";

    vector<int>::iterator st, ed;
    st = find(individual.begin(), individual.end(), 0);
    ed = find(st + 1, individual.end(), 0);

    // compute inner loop
    while(ed != individual.end()){
        for(vector<int>::iterator it = st + 1; it < ed - 1; it++){
            sum_tbl[cur] += dis[*it][*(it + 1)];
            tot += dis[*it][*(it + 1)];
        }
        // loop
        // sum_tbl[cur] += dis[*(st + 1)][*(ed - 1)];
        // tot += dis[*(st + 1)][*(ed - 1)];

        st = ed;
        ed = find(st + 1, individual.end(), 0);
        cur++;
    }

    // compute tail + head
    st = st + 1 == individual.end() ? individual.begin() : st + 1;
    while(*st != 0){
        ed = st + 1 == individual.end() ? individual.begin() : st + 1;
        if(*ed == 0){
            // if loop, adds here
            break;
        }
        sum_tbl[cur] += dis[*st][*ed];
        tot += dis[*st][*ed];
        st = st + 1 == individual.end() ? individual.begin() : st + 1;
    }
    // return larger values for unfair distribution of distances between salesmen.
    double L2 = 0.0;
    for(int i = 0; i < sum_tbl.size(); ++i) L2 += sum_tbl[i] * sum_tbl[i];

    return L2 / tot;
}

Mat MTSP::MTSP2Path(){
    Mat path_idx(conf.sales_men);
    Mat path_idx_full(conf.sales_men);
    Mat path_pts_idx(conf.sales_men);
    vector<int>::iterator st, ed;
    UnionFind uf(best_mtsp.size());
    st = find(best_mtsp.begin(), best_mtsp.end(), 0);
    ed = find(st + 1, best_mtsp.end(), 0);
    
    int cur = 0;
    int idx = 0;
    while(ed != best_mtsp.end()){
        for(vector<int>::iterator it = st + 1; it < ed; it++){
            idx = pts[*it].first * cmw + pts[*it].second;
            path_idx[cur].push_back(idx);
            path_pts_idx[cur].push_back(*it);
        }
        
        st = ed;
        ed = find(st + 1, best_mtsp.end(), 0);
        cur++;
    }
    
    // tail + head, right order
    st = st + 1 == best_mtsp.end() ? best_mtsp.begin() : st + 1;
    while(*st != 0){
        idx = pts[*st].first * cmw + pts[*st].second;
        path_idx[cur].push_back(idx);
        path_pts_idx[cur].push_back(*st);
        st = st + 1 == best_mtsp.end() ? best_mtsp.begin() : st + 1;
    }

    // form a loop ? 
    // for(int i = 0; i < conf.sales_men; ++i){
    //     path_idx[i].push_back(path_idx[i][0]);
    // }

    // if a path starts or ends from a mid point, add its neighbor
    for(int i = 0; i < path_pts_idx.size(); ++i){
        for(int j = 0; j < path_pts_idx[i].size() - 1; ++j){
            uf.unite(path_pts_idx[i][j], path_pts_idx[i][j + 1]);
        }
    }

    for(int i = 0; i < path_pts_idx.size(); ++i){
        int ft = path_pts_idx[i].front();
        int bk = path_pts_idx[i].back();
        int pass_id1, pass_id2;
        // assume only one neighbor not in the same set, otherwise the best_val
        // is greater than 1000
        if(mid.count(ft)){
            pass_id1 = mid_pts[mid[ft]].pass_id1;
            pass_id2 = mid_pts[mid[ft]].pass_id2;
            if(uf.find(ft) != uf.find(pass_id1)){
                idx = pts[pass_id1].first * cmw + pts[pass_id1].second;
                path_idx[i].insert(path_idx[i].begin(), idx);
            }
            else if(uf.find(ft) != uf.find(pass_id2)){
                idx = pts[pass_id2].first * cmw + pts[pass_id2].second;
                path_idx[i].insert(path_idx[i].begin(), idx);
            }
        }

        if(mid.count(bk)){
            pass_id1 = mid_pts[mid[bk]].pass_id1;
            pass_id2 = mid_pts[mid[bk]].pass_id2;
            if(uf.find(bk) != uf.find(pass_id1)){
                idx = pts[pass_id1].first * cmw + pts[pass_id1].second;
                path_idx[i].push_back(idx);
            }
            else if(uf.find(bk) != uf.find(pass_id2)){
                idx = pts[pass_id2].first * cmw + pts[pass_id2].second;
                path_idx[i].push_back(idx);
            }
        }
    }

    // 补上中间的点！！
    // 注意补充的点不能穿过障碍物
    for(int i = 0; i < path_idx.size(); ++i){
        for(int j = 0; j < path_idx[i].size() - 1; ++j){
            // if not in the same brick, just ignore，no need to fill in
            int x1 = path_idx[i][j] / cmw;
            int y1 = path_idx[i][j] % cmw;
            int x2 = path_idx[i][j + 1] / cmw;
            int y2 = path_idx[i][j + 1] % cmw;
            path_idx_full[i].push_back(path_idx[i][j]);
            bool flag = false;
            if(x1 == x2){
                if(y2 > y1){
                    if(pt_brickid[path_idx[i][j]] == pt_brickid[path_idx[i][j + 1]]){
                        for(int y = y1 + 1; y < y2; ++y)    
                            path_idx_full[i].push_back(x1 * cmw + y);
                    }
                } else {
                    if(pt_brickid[path_idx[i][j]] == pt_brickid[path_idx[i][j + 1]]){
                        for(int y = y1 - 1; y > y2; --y)
                            path_idx_full[i].push_back(x1 * cmw + y);
                    }
                }
            } else if(y1 == y2){
                if(x2 > x1){
                    if(pt_brickid[path_idx[i][j]] == pt_brickid[path_idx[i][j + 1]]){
                        for(int x = x1 + 1; x < x2; ++x)
                            path_idx_full[i].push_back(x * cmw + y1);
                    }
                } else {
                    if(pt_brickid[path_idx[i][j]] == pt_brickid[path_idx[i][j + 1]]){
                        for(int x = x1 - 1; x > x1; --x)
                            path_idx_full[i].push_back(x * cmw + y1);
                    }
                }
            } else {
                continue;
            }
        }
        path_idx_full[i].push_back(path_idx[i].back());
    }

    return path_idx_full;
}

void MTSP::showpath(){
    for(int i = 0; i < best_mtsp.size(); ++i){
        cout << "(" << pts[best_mtsp[i]].first << ", " << pts[best_mtsp[i]].second << ") -> ";
    }
    cout << endl;
}