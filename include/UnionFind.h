#include <vector>

using std::vector;

class UnionFind{
    vector<int> fa;

public:
    UnionFind(){

    }
    
    UnionFind(int sz){
        fa.resize(sz + 5, 0);
        for(int i = 0; i <= sz; ++i)    fa[i] = i;
    }

    void init(int sz){
        fa.resize(sz + 5, 0);
        for(int i = 0; i <= sz; ++i)    fa[i] = i;
    }

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
};