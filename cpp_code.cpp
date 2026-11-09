// Blantyre Water Pipe Network: Dijkstra shortest paths + Kruskal MST
// Version for Dev-C++ 5.11 (old GCC 4.9, works with the default settings).
// One single file. The graph is built in, so no input file is needed.
// No special compiler options are needed: just press F11 (Compile and Run).
// Weights are travel time in minutes. Replace with Google Maps values.

#include <algorithm>
#include <climits>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

struct Edge {
    int u, v, w;
};

map<string, int> id;
vector<string> name;
vector<Edge> edges;
vector< vector< pair<int, int> > > adj;

// Built-in graph: "NodeA NodeB weight" (use _ instead of spaces)
const char* BUILT_IN =
    "Chichiri_Reservoir Blantyre_CBD 8\n"
    "Chichiri_Reservoir Mount_Pleasant 7\n"
    "Mudi_Reservoir Blantyre_CBD 10\n"
    "Mudi_Reservoir Namiwawa 6\n"
    "Mudi_Reservoir Sunnyside 7\n"
    "Blantyre_CBD Sunnyside 5\n"
    "Blantyre_CBD Namiwawa 6\n"
    "Blantyre_CBD Mount_Pleasant 9\n"
    "Blantyre_CBD QECH 6\n"
    "Blantyre_CBD Ndirande 12\n"
    "Blantyre_CBD Limbe 15\n"
    "Blantyre_CBD Manase 10\n"
    "Namiwawa Mount_Pleasant 5\n"
    "Sunnyside QECH 4\n"
    "QECH Soche 6\n"
    "Soche Mount_Pleasant 8\n"
    "Ndirande Manase 7\n"
    "Ndirande Chilomoni 10\n"
    "Ndirande Chitawira 8\n"
    "Manase Chilomoni 6\n"
    "Chitawira Limbe 6\n"
    "Chitawira Nancholi 5\n"
    "Nancholi Limbe 7\n"
    "Nancholi Chilomoni 9\n"
    "Limbe Chirimba 8\n"
    "Limbe Naperi 9\n"
    "Limbe Chigumula 12\n"
    "Limbe Bangwe 14\n"
    "Limbe Chileka 20\n"
    "Chirimba Chileka 22\n"
    "Naperi Chigumula 6\n"
    "Bangwe Chigumula 7\n"
    "Bangwe Zingwangwa 8\n"
    "Zingwangwa Soche 12\n"
    "Soche Kanjedza 10\n"
    "Kanjedza Kameza 8\n"
    "Kameza Zingwangwa 10\n";

int getId(const string& s) {
    map<string, int>::iterator it = id.find(s);
    if (it != id.end()) return it->second;
    int k = (int)name.size();
    id[s] = k;
    name.push_back(s);
    adj.push_back(vector< pair<int, int> >());
    return k;
}

bool loadFrom(istream& in, const string& source) {
    string line;
    int lineNo = 0;
    while (getline(in, line)) {
        lineNo++;
        if (line.empty() || line[0] == '#') continue;
        istringstream ss(line);
        string a, b;
        int w;
        if (!(ss >> a >> b >> w)) {
            cerr << "Error: bad format on line " << lineNo << " of " << source << "\n";
            return false;
        }
        if (w < 0) {
            cerr << "Error: negative weight on line " << lineNo << " (Dijkstra needs weights >= 0)\n";
            return false;
        }
        int u = getId(a);
        int v = getId(b);
        Edge e;
        e.u = u;
        e.v = v;
        e.w = w;
        edges.push_back(e);
        adj[u].push_back(make_pair(v, w));
        adj[v].push_back(make_pair(u, w));
    }
    return true;
}

vector<int> dijkstra(int src, int target) {
    int n = (int)name.size();
    vector<int> dist(n, INT_MAX), prev(n, -1);
    vector<bool> done(n, false);
    priority_queue< pair<int, int>, vector< pair<int, int> >, greater< pair<int, int> > > pq;
    dist[src] = 0;
    pq.push(make_pair(0, src));
    int step = 0;
    vector<int> selectedPath;
    cout << "\n--- Dijkstra steps (visit order) ---\n";
    while (!pq.empty()) {
        int d = pq.top().first;
        int u = pq.top().second;
        pq.pop();
        if (done[u]) continue;
        done[u] = true;
        step++;
        cout << "Step " << step << ": visit " << name[u] << " (dist " << d << ")\n";
        for (size_t k = 0; k < adj[u].size(); k++) {
            int v = adj[u][k].first;
            int w = adj[u][k].second;
            if (!done[v] && d + w < dist[v]) {
                dist[v] = d + w;
                prev[v] = u;
                pq.push(make_pair(dist[v], v));
                cout << "    update " << name[v] << " = " << dist[v] << " via " << name[u] << "\n";
            }
        }
    }
    cout << "\n--- Shortest times from " << name[src] << " ---\n";
    for (int i = 0; i < n; i++) {
        cout << name[i] << ": ";
        if (dist[i] == INT_MAX) cout << "unreachable\n";
        else cout << dist[i] << " min\n";
    }
    if (target >= 0) {
        cout << "\n--- Path to " << name[target] << " ---\n";
        if (dist[target] == INT_MAX) {
            cout << "No path exists.\n";
        } else {
            for (int x = target; x != -1; x = prev[x]) selectedPath.push_back(x);
            reverse(selectedPath.begin(), selectedPath.end());
            for (size_t i = 0; i < selectedPath.size(); i++) {
                cout << name[selectedPath[i]];
                if (i + 1 < selectedPath.size()) cout << " -> ";
            }
            cout << "\nTotal: " << dist[target] << " min\n";
        }
    }
    return selectedPath;
}

int findSet(vector<int>& p, int x) {
    if (p[x] == x) return x;
    p[x] = findSet(p, p[x]);
    return p[x];
}

bool edgeLess(const Edge& a, const Edge& b) {
    return a.w < b.w;
}

void kruskal(const vector<int>& route) {
    int n = (int)name.size();
    vector<Edge> sorted;
    bool targetSpecific = !route.empty();
    vector<bool> included(n, !targetSpecific);
    if (targetSpecific) {
        for (size_t i = 0; i < route.size(); i++) included[route[i]] = true;
    }
    for (size_t i = 0; i < edges.size(); i++) {
        if (included[edges[i].u] && included[edges[i].v]) sorted.push_back(edges[i]);
    }
    if (!targetSpecific) sorted = edges;
    stable_sort(sorted.begin(), sorted.end(), edgeLess);
    vector<int> parent(n);
    for (int i = 0; i < n; i++) parent[i] = i;
    int total = 0, count = 0;
    if (targetSpecific)
        cout << "\n--- Kruskal MST for selected source-target route nodes ---\n";
    else
        cout << "\n--- Kruskal MST for the entire network ---\n";
    for (size_t i = 0; i < sorted.size(); i++) {
        const Edge& e = sorted[i];
        int a = findSet(parent, e.u);
        int b = findSet(parent, e.v);
        if (a != b) {
            parent[a] = b;
            total += e.w;
            count++;
            cout << "Take " << name[e.u] << " - " << name[e.v] << " (" << e.w << ")\n";
        } else {
            cout << "Skip " << name[e.u] << " - " << name[e.v] << " (" << e.w << ") makes a cycle\n";
        }
    }
    cout << "MST edges: " << count << ", total weight: " << total << "\n";
    int expectedEdges = targetSpecific ? (int)route.size() - 1 : n - 1;
    if (count != expectedEdges) cout << "Warning: selected graph is not connected, this is a spanning forest.\n";
}

int main(int argc, char** argv) {
    // 1. Load the graph (built-in, or from a file given as 3rd argument)
    if (argc > 3) {
        ifstream f(argv[3]);
        if (!f) {
            cerr << "Error: cannot open file " << argv[3] << "\n";
            return 1;
        }
        if (!loadFrom(f, argv[3])) return 1;
    } else {
        istringstream builtIn(BUILT_IN);
        if (!loadFrom(builtIn, "built-in graph")) return 1;
    }
    cout << "Loaded " << name.size() << " nodes and " << edges.size() << " edges.\n";

    // 2. Get source and target
    string s, t;
    bool interactive = (argc <= 1);
    if (!interactive) {
        s = argv[1];
        if (argc > 2) t = argv[2];
    } else {
        cout << "\nPlaces:\n";
        for (size_t i = 0; i < name.size(); i++) cout << "  " << name[i] << "\n";
        cout << "\nSource node: ";
        cin >> s;
        cout << "Target node (or - for none): ";
        cin >> t;
        if (t == "-") t = "";
    }
    if (id.find(s) == id.end()) {
        cerr << "Error: unknown source node '" << s << "'\n";
        if (interactive) { cout << "Press Enter to close..."; cin.ignore(10000, '\n'); cin.get(); }
        return 1;
    }
    int target = -1;
    if (!t.empty()) {
        if (id.find(t) == id.end()) {
            cerr << "Error: unknown target node '" << t << "'\n";
            if (interactive) { cout << "Press Enter to close..."; cin.ignore(10000, '\n'); cin.get(); }
            return 1;
        }
        target = id[t];
    }

    // 3. Run both algorithms
    vector<int> selectedRoute = dijkstra(id[s], target);
    kruskal(selectedRoute);

    // Keep the Dev-C++ console window open
    if (interactive) {
        cout << "\nPress Enter to close...";
        cin.ignore(10000, '\n');
        cin.get();
    }
    return 0;
}
