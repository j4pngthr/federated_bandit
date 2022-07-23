#include"global.hpp"

int start_vld_id = 20, end_vld_id = 97;
int N;

void getRealTrace(vector<vector<pii> >& contact_nodes) {
    N = end_vld_id - start_vld_id + 1;
    string str = "contacts.Exp"; // 入力ファイル
    {
        // str += (simu_id < 3 ? to_string(simu_id + 1) : "6");
        str += "6";
        str += ".txt";
        ifstream ifs(str);
    }

    string line;
    vector<vector<int> > v; // ノードとコンタクト時間

    // コンタクトを2周する
    // まずは最初と最後のコンタクトの時間を得る
    int last_contact_ut = 0;
    while (getline(ifs, line)) {
        istringstream stream(line);
        vector<int> v2;
        while (getline(stream, str, ' ')) {
            if (str[0] >= '0' && str[0] <= '9') {
                v2.eb(stoi(str));
            }
        }

        --v2[0]; --v2[1];
        int node1 = v2[0], node2 = v2[1], contact_ut= v2[2];
        // if (node1 == node2 || !isOriVldId(node1) || !isOriVldId(node2)) continue;

        chmax(last_contact_ut, contact_ut);
        chmin(first_contact_ut, contact_ut);

        v2[0] -= start_vld_id; v2[1] -= start_vld_id;
        v.eb(v2);
    }

    rep(i, sz(v)) {
        int node1 = v[i][0], node2 = v[i][1], contact_ut = v[i][2];
        contact_nodes[ut].eb(node1, node2);
    }
}
