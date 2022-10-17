#include"global.hpp"
#include"getRealTrace.hpp"


const int output_range = 100;
int N = 78, T = 100000; // extern -> global.hpp, not const
int cnt = 0, cnt0 = 0; // count of contacts

int main() {
    ios::sync_with_stdio(false);
    cin.tie(0);
    cout.tie(0);
    cout << fixed << setprecision(12);

    vector<Agent> agt(N);

    // firstly, get N, T
    vector<vector<pii> > contact_nodes;
    getRealTrace(contact_nodes, agt); // agtに隣接ノードの情報を持たせる
    // {
    //     cerr << "sz(contact_nodes) ";
    //     int cnt = 0;
    //     rep(i, sz(contact_nodes)) cnt += sz(contact_nodes[i]);
    //     cerr << cnt << endl;
    // }

    // レバーの定義
    vector<Lever> lv(M); // muが求まる
    // cerr << "lever" << endl; rep(k, M) cerr << k << " " << lv[k].getMu() << endl; // k番目のレバーの報酬の平均値

    // 真の値を求める
    double true_val = 0;
    rep(k, M) true_val += lv[k].getMu();
    true_val /= M;
    cerr << "true_val " << true_val << endl;

    // 最初にすべて1回ずつ引く, t=0
    rep(k, M) {
        rep(i, N) {
            agt[i].X[k][0] = lv[k].play(); // ノードiがレバーkを時刻0に引いて得る報酬X
            ++agt[i].n[k][0]; // レバーを引いた回数n
            agt[i].theta[k][0] = agt[i].X[k][0]; // SIGMETRIC2021, algorithm 1, 報酬の推測値
            agt[i].X_tilde[k][0] = agt[i].X[k][0]; // 報酬の平均値
        }
    }

    {
        string filename = "rewards.txt";
        ofstream ofs(filename, ios::out);

        ofs << 0 << " " << 0 << endl;
    }

    double sum = 0;
    rep3(t, 1, T) {
        if (t == 4000) return 0;
        if (t % output_range == 0) cerr << "t " << t << endl;

        rep(i, N) {
            vector<int> A; // line 3
            rep(k, M) {
                agt[i].n[k][t] = agt[i].n[k][t - 1]; // line 4
                for (int j : agt[i].e) { // 隣接ノード, line 5
                    agt[i].n_tilde[k][t + 1] = max(agt[i].n[k][t], agt[j].n_tilde[k][t]); // すぐ下でのみ使う
                }
                if (agt[i].n[k][t] < agt[i].n_tilde[k][t] - N) { // line 6
                    A.emplace_back(k);
                }
            }

            int arm_id = -1;
            double max_val = -100000.0;
            int flg = 0;
            if (A.empty()) { // arm_idを決める, line 7
                rep(k, M) {
                    double alpha1 = 64.0; // Cの1つ上の式, p13
                    rep(i, 17) alpha1 /= N;
                    agt[i].C[k][t] = sqrt(2.0 * N / agt[i].n[k][t] * log(t)) + alpha1; // p13
                    // agt[i].C[k][t] = sqrt(1.0 / agt[i].n[k][t] * log(t)); // log_e(t)
                    agt[i].Q[k][t] = agt[i].theta[k][t - 1] + agt[i].C[k][t];
                    if (chmax(max_val, agt[i].Q[k][t])) {
                        arm_id = k;
                        flg = 1;
                    }
                }
            } else { // line 12
                flg = 2;
                arm_id = A[rand() % sz(A)];
            }
            agt[i].a[t] = arm_id;
            // cerr << t << " " << i << " " << flg << " " << arm_id << endl;
            // if (t % output_range == 0) {
            //     cerr << t << " " << i << " " << flg << " " << arm_id << endl;
            // }

            // get X, line 15
            agt[i].X[arm_id][t] = lv[arm_id].play();
            sum += agt[i].X[arm_id][t];
            // update X_tilde, line 15
            rep(_t, t) { // (2), p6
                int k = agt[i].a[_t];
                agt[i].X_tilde[k][t] += agt[i].X[k][_t];
            }
            rep(k, M) {
                agt[i].X_tilde[k][t] *= 1.0 / agt[i].n[k][t];
            }
            ++agt[i].n[arm_id][t]; // line 16
        }

        // 時刻tに接触する2ノード
        vector<int> used(N);
        for (pii p : contact_nodes[t]) {
            int i = p.F, j = p.S;
            if (i > j) swap(i, j);

            used[i] = 1; used[j] = 1;
            rep(k, M) { // line 20
                agt[i].theta[k][t] = (agt[i].theta[k][t - 1] + agt[j].theta[k][t - 1]) / 2.0 + agt[i].X_tilde[k][t] - agt[i].X_tilde[k][t - 1];
            }

            if (i == 0) ++cnt0;
            ++cnt;
        }
        rep(i, N) {
            if (used[i]) continue;
            rep(k, M) {
                agt[i].theta[k][t] = agt[i].theta[k][t - 1] + agt[i].X_tilde[k][t] - agt[i].X_tilde[k][t - 1];
            }
        }

        // 時刻ごとの各エージェントの報酬の平均を出力
        if (t % output_range == 0) {
            string filename = "rewards.txt";
            ofstream ofs(filename, ios::app);

            ofs << t << " " << sum / N / t << endl;
            // cerr << "cnt0 " << cnt0 << endl;

            // {
            //     cerr << t << endl;
            //     rep(k, M) cerr << agt[0].theta[k][t - 1] << " "; cerr << endl;
            //     rep(k, M) cerr << agt[0].C[k][t] << " "; cerr << endl;
            //     rep(k, M) cerr << agt[0].Q[k][t] << " "; cerr << endl;
            // }
        }















    // // 最初にすべて1回ずつ引く, t=0
    // // 第二引数 0->t+1, 1->t, 2->t-1
    // rep(k, M) {
    //     rep(i, N) {
    //         ++agt[i].n[k][1];
    //         agt[i].X[k][1] = lv[k].play();
    //         agt[i].theta[k][1] = agt[i].X_tilde[k][1] = agt[i].X[k][1]; // SIGMETRIC2021, algorithm 1
    //     }
    // }
    //
    // rep3(t, 1, T) {
    //     if (t % 1000 == 1) cerr << "t " << t << endl;
    //
    //     // t+1<-t, t<-t-1
    //     rep(i, N) {
    //         rep(k, M) {
    //             rep(_t, 2) {
    //                 agt[i].n[k][_t] = agt[i].n[k][_t + 1];
    //                 agt[i].n_tilde[k][_t] = agt[i].n[k][_t + 1];
    //                 agt[i].X[k][_t] = agt[i].n[k][_t + 1];
    //                 agt[i].X_tilde[k][_t] = agt[i].n[k][_t + 1];
    //                 agt[i].theta[k][_t] = agt[i].n[k][_t + 1];
    //                 agt[i].Q[k][_t] = agt[i].n[k][_t + 1];
    //             }
    //         }
    //     }
    //
    //     double ave = 0; // 報酬の平均
    //     rep(i, N) {
    //         vector<int> A;
    //         rep(k, M) {
    //             for (int j : agt[i].e) { // 隣接ノード, line 5
    //                 agt[i].n_tilde[k][0] = max(agt[i].n[k][1], agt[j].n_tilde[k][1]);
    //             }
    //             if (agt[i].n[k][1] < agt[i].n_tilde[k][1] - N) { // line 6
    //                 A.emplace_back(k);
    //             }
    //         }
    //
    //         int arm_id = -1;
    //         double max_val = -100000.0;
    //         if (A.empty()) { // line 7
    //             rep(k, M) {
    //                 double alpha1 = 64;
    //                 rep(i, 17) alpha1 /= N;
    //                 double C = sqrt(2.0 * N / agt[i].n[k][1] * log(t)) + alpha1;
    //                 agt[i].Q[k][1] = agt[i].theta[k][2] + C;
    //                 if (chmax(max_val, agt[i].Q[k][1])) {
    //                     arm_id = k;
    //                 }
    //             }
    //         } else { // line 12
    //             arm_id = A[rand() % sz(A)];
    //         }
    //         // get X, line 15
    //         agt[i].X[arm_id][1] = lv[arm_id].play();
    //         ave += agt[i].X[arm_id][1];
    //         // update X_tilde, line 15
    //         rep(k, M) { // (2), p6
    //             // * (agt[i].X_tilde[k][0] * (t - 1) + agt[i].X[k][1]);
    //             // 終わってない
    //             agt[i].X_tilde[k][1] *= 1.0 / agt[i].n[k][1];
    //         }
    //         ++agt[i].n[arm_id][1]; // line 16
    //     }
    //
    //     // 時刻tに接触する2ノード
    //     vector<int> used(N);
    //     for (pii p : contact_nodes[t]) {
    //         int i = p.F, j = p.S;
    //         used[i] = 1; used[j] = 1;
    //         rep(k, M) { // line 20
    //             agt[i].theta[k][1] = (agt[i].theta[k][2] + agt[j].theta[k][2]) / 2 + agt[i].X_tilde[k][1] - agt[i].X_tilde[k][2];
    //         }
    //     }
    //     rep(i, N) {
    //         if (used[i]) continue;
    //         rep(k, M) {
    //             agt[i].theta[k][1] = agt[i].theta[k][2] + agt[i].X_tilde[k][1] - agt[i].X_tilde[k][2];
    //         }
    //     }
    //
    //     // 時刻ごとの各エージェントの報酬の平均を出力
    //     if (t % 1000 == 1) {
    //         string filename = "rewards.txt";
    //         ofstream ofs(filename, (t == 1 ? ios::out : ios::app));
    //
    //         ofs << t << " " << ave / N << endl;
    //     }
    }
}
