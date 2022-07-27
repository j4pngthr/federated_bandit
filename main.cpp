#include"global.hpp"
#include"getRealTrace.hpp"

int N = 40, T = 100000; // extern -> global.hpp

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);
    cout.tie(0);
    cout << fixed << setprecision(12);

    // firstly, get N, T
    vector<vector<pii> > contact_nodes;
    getRealTrace(contact_nodes);

    vector<Agent> agt(N);

    // レバーの定義
    vector<Lever> lv(M);
    // 最初にすべて1回ずつ引く
    rep(k, M) {
        rep(i, N) {
            ++agt[i].n[k][0];
            agt[i].X[k][0] = lv[k].play();
            agt[i].theta[k][0] = agt[i].X_tilde[k][0] = agt[i].X[k][0];
        }
    }

    rep3(t, 1, T) {
        if (t % 100 == 1) cerr << t << endl;

        rep(i, N) {
            vector<int> A;
            rep(k, M) {
                agt[i].n[k][t] = agt[i].n[k][t - 1];
                rep(j, sz(agt[i].e)) {
                    // agt[i].n_tilde[k][t + 1] = max(agt[i].n[k][t], agt[j].n_tilde[k][t]);
                }
                if (agt[i].n[k][t] < agt[i].n_tilde[k][t] - N) { // line 6
                    A.emplace_back(k);
                }
            }
            int arm_id = -1;
            double max_val = 0;
            if (A.empty()) { // line 7
                rep(k, M) {
                    double alpha1 = 64;
                    rep(i, 17) alpha1 /= N;
                    double C = sqrt(2 * N / agt[i].n[k][t] * log(t)) + alpha1;
                    agt[i].Q[k][t] = agt[i].theta[k][t - 1] + C;
                    if (chmax(max_val, agt[i].Q[k][t])) {
                        arm_id = k;
                    }
                }
            } else { // line 12
                arm_id = rand() % sz(A);
            }
            // observe arm a_i(t)
            agt[i].a[t] = arm_id;
            // get X
            agt[i].X[arm_id][t] = lv[arm_id].play();
            ++agt[i].n[arm_id][t];
            // update X_tilde
            rep(k, M) {
                agt[i].X_tilde[k][t] = accumulate(all(agt[i].X[k]), 0.0) / t;
            }
        }

        // 時刻tに接触する2ノード
        vector<int> used(N);
        for (pii p : contact_nodes[t]) {
            int i = p.F, j = p.S;
            used[i] = 1; used[j] = 1;
            rep(k, M) {
                agt[i].theta[k][t] = (agt[i].theta[k][t - 1] + agt[j].theta[k][t - 1]) / 2 + agt[i].X_tilde[k][t] - agt[i].X_tilde[k][t - 1];
            }
        }
        rep(i, N) {
            if (used[i]) continue;
            rep(k, M) {
                agt[i].theta[k][t] = agt[i].theta[k][t - 1] + agt[i].X_tilde[k][t] - agt[i].X_tilde[k][t - 1];
            }
        }

        // 報酬を出力
        rep(i, N) {
            rep(k, M) {
                double r = agt[i].X[k][t];
                if (r < eps) continue;
                cerr << "i t r " << i << " " << t << " " << r << endl;
            }
        }
    }
}
