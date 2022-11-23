#ifndef _M_TSP_H
#define _M_TSP_H

#include <iostream>
#include <cstdio>
#include <queue>
#include <algorithm>
#include <unordered_set>
#include <cstring>
#include <map>
#include <Dinic.h>
#include <cmath>
#include <random>

using std::queue; 
using std::vector;
using std::priority_queue;
using std::cout;
using std::endl;
using std::pair;

#define UNREACHABLE 10000000

struct GA_CONF{
    double crossover_pb;
    double mutation_pb;
    double elite_rate;
    int sales_men;
    int max_iter;
    int ppl_sz;
    int unchanged_iter;
};

// We apply genetic algorithm for solving mTSP problem here
class MTSP{
    vector<P> pts;
    Mat ppl;
    vector<vector<uint64_t>> dis;
    GA_CONF conf;
    vector<double> vals, fitness_vals, roulette;
    int mutation_time = 0;

    vector<int> best_mtsp;  // best mstp through history
    uint64_t cur_best_val;
    int cur_best_id;
    int generations = 0;

    std::random_device rd;
    std::mt19937 generator;

    struct mid_pt{
        int id;
        int pass_id1, pass_id2;  // 两个端点的id
    };
    vector<mid_pt> mid_pts;

    vector<int> depot;
    bool coverAndReturn;
    std::map<int, int> mid;  // pts idx to mid_pts idx
    int cmw;
public:
    int cur_gen = 0;
    int unchanged_gens = 0;
    uint64_t best_val = -1;
    std::map<int, int> pt_brickid;

    MTSP(GA_CONF _conf, Dinic &dinic, vector<int> &_depot, int _cmw, bool _coverAndReturn) : 
        conf(_conf), depot(_depot), cmw(_cmw), coverAndReturn(_coverAndReturn) {
        ppl.resize(conf.ppl_sz);
        vals.resize(conf.ppl_sz);
        fitness_vals.resize(conf.ppl_sz);
        roulette.resize(conf.ppl_sz);
        mid.clear();
        pt_brickid.clear();
        generator = std::mt19937(rd());
        // 只用brick的corner以及mid点进行建图
        // start from one, 0 for seperator
        pts.push_back({ -1, -1 });
        for(int i = 0; i < dinic.bricks.size(); ++i){
            for(int j = 0; j < 4; ++j){
                pts.push_back(dinic.bricks[i].corner[j]);

                int pt_id = dinic.bricks[i].corner[j].first * cmw + dinic.bricks[i].corner[j].second;
                pt_brickid[pt_id] = i;
            }

            // have mid pts
            if(dinic.bricks[i].pts.size() > 1){
                mid_pts.push_back((struct mid_pt){ pts.size(), pts.size() - 4, pts.size() - 3 });
                mid_pts.push_back((struct mid_pt){ pts.size() + 1, pts.size() - 2, pts.size() - 1 });
                pts.push_back(dinic.bricks[i].mid_pts[0]);
                pts.push_back(dinic.bricks[i].mid_pts[1]);
                
                mid[pts.size() - 2] = mid_pts.size() - 2;
                mid[pts.size() - 1] = mid_pts.size() - 1;

                int mid_pt_id1 = dinic.bricks[i].mid_pts[0].first * cmw + dinic.bricks[i].mid_pts[0].second;
                int mid_pt_id2 = dinic.bricks[i].mid_pts[1].first * cmw + dinic.bricks[i].mid_pts[1].second;
                pt_brickid[mid_pt_id1] = i;
                pt_brickid[mid_pt_id2] = i;
            }
        }

        // compute dis
        dis.resize(pts.size(), vector<uint64_t>(pts.size(), 0));
        for(int i = 0; i < pts.size(); ++i){
            for(int j = 0; j < pts.size(); ++j){
                dis[i][j] = distance(pts[i], pts[j]);
            }
        }

        for(int i = 0; i < dis.size(); ++i)
            dis[0][i] = dis[i][0] = UNREACHABLE;
        
        // large penalty for an empty loop
        for(int i = 0; i < dis.size(); ++i)
            dis[i][i] = UNREACHABLE;

        // mid point to it's non neighbor will be unreachable
        // we need to use the actual shortest path
        for(int i = 0; i < mid_pts.size(); ++i){
            for(int j = 0; j < pts.size(); ++j){
                // maybe add a penalty here
                if(j != mid_pts[i].pass_id1 && j != mid_pts[i].pass_id2 && j != mid_pts[i].id)
                    dis[mid_pts[i].id][j] = dis[j][mid_pts[i].id] = UNREACHABLE;
                else if(j == mid_pts[i].pass_id1 || j == mid_pts[i].pass_id2){
                    dis[mid_pts[i].id][j] = dis[j][mid_pts[i].id] = distance(pts[mid_pts[i].id], pts[j]);   // must be 0. Otherwise GA bad result 
                    // = distance(pts[mid_pts[i].id], pts[j])
                }
            }
        }

        // for(int i = 0; i < dis.size(); ++i){
        //     for(int j = 0; j < dis[0].size(); ++j){
        //         cout << dis[i][j] << " ";
        //     }
        //     cout << "\n";
        // }

        for(int i = 0; i < ppl.size(); ++i){
            randomIndividual(ppl[i]);
        }

        getBestIndividual();
    }

    uint64_t distance(P x, P y){
        return (uint64_t)ceil(sqrt(1.0 * (x.first - y.first) * (x.first - y.first) + 
                         1.0 * (x.second - y.second) * (x.second - y.second)));
    }

    void randomIndividual(vector<int>& individual){
        for(int i = 1; i < pts.size(); ++i)
            individual.push_back(i);

        for(int i = 0; i < conf.sales_men; ++i)
            individual.push_back(0);

        // std::random_device rd;
        // std::mt19937 g(rd());

        std::shuffle(individual.begin(), individual.end(), generator);
    }

    // evaluate one chromosome, using L2 value to specify the unbalance
    double evaluate(vector<int>& individual);
    // evolution
    void GANextGeneration();

    void selection();

    void crossover();

    void crossoverOnce(int x, int y);

    void mutation();
    // select the best chromosome from the population
    void getBestIndividual();

    vector<int> reverseMutate(vector<int> individual);

    vector<int> concatMutate(vector<int> individual);

    void setRoulette();

    double _random();

    int _randomInt(int bound);

    int wheelOut(double rand);

    vector<int> getChild(bool foward, int x, int y);

    // generate path
    Mat MTSP2Path();

    void showpath();
};


#endif