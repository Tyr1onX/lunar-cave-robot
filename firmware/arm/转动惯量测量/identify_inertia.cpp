// identify_inertia.cpp -- joint equivalent-inertia identification from a CSV log
//
// build:  g++ -std=c++17 -O2 -Wall identify_inertia.cpp -o identify_inertia.exe
// run  :  identify_inertia.exe J1.csv 1 -o report_j1.txt
//           argv[1] csv    argv[2] joint index (1..7)
//           -o file   write the report into a file (UTF-8/ASCII, safer than shell redirection)
//
// Model (single axis, other axes locked):
//     tau = J*alpha + B*omega + Fc*sgn(omega) + b
//   alpha comes from a local quadratic fit of the *unwrapped* position
//   (samples are unevenly spaced in time, so the fit uses real timestamps).
//   Two stages are reported: per-segment (independent J,B,Fc,b) and global
//   (one J per segment, B/Fc/b shared by all data).
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <cctype>
#include <algorithm>
#include <utility>
#include <string>
#include <vector>
using namespace std;

static const double PI = 3.14159265358979323846;
static const int DOF = 7;

// ---- geometry model constants, keep in sync with variables.cpp / roboarm_kinematics.cpp
static const double MASS[DOF] = { 0.0, 1.2, 1.5, 0.0, 0.8, 0.0, 0.0 };
static const double LENGTH[DOF] = { 0.0, 0.320, 0.325, 0.0, 0.213, 0.0, 0.0 };
static const double OFFSET[DOF] = { -0.14122, -2.38192, 1.00439, -2.80760, 0.62223, -0.77208, 3.11051 };
static const double THETA_OFF[DOF] = { 0.0, 2.65289, 4.22348, 3.14159, 4.57276, 0.0, 0.0 };
static const double LINK_ALPHA[DOF] = { 0.0, 1.57080, 3.14159, 1.57080, 1.57080, 1.57080, 0.0 };
static const double AL0[DOF][3] = { {0,0,1},{1,0,0},{0,-1,0},{0,0,1},{0,-1,0},{0,0,1},{0,0,1} };

static const int HALF = 8;              // half window of the local quadratic fit (+-8 samples)
static const double VEL_MOVE = 0.05;    // rad/s   "moving" threshold
static const double ACC_MOVE = 0.5;     // rad/s^2 "moving" threshold
static const int MIN_PTS = 60;          // minimum samples per segment
static const double MIN_DTH = 0.15;     // minimum travel of a usable segment [rad]

// ---------------- linear algebra ----------------
static bool SolveN(vector<vector<double> > A, vector<double> b, vector<double>& x)
{
    int n = (int)A.size();
    for (int c = 0; c < n; ++c)
    {
        int piv = c;
        for (int r = c + 1; r < n; ++r) if (fabs(A[r][c]) > fabs(A[piv][c])) piv = r;
        if (fabs(A[piv][c]) < 1e-12) return false;
        swap(A[c], A[piv]);
        swap(b[c], b[piv]);
        double d = A[c][c];
        for (int k = c; k < n; ++k) A[c][k] /= d;
        b[c] /= d;
        for (int r = 0; r < n; ++r)
        {
            if (r == c) continue;
            double f = A[r][c];
            if (f == 0.0) continue;
            for (int k = c; k < n; ++k) A[r][k] -= f * A[c][k];
            b[r] -= f * b[c];
        }
    }
    x = b;
    return true;
}

static bool InvN(const vector<vector<double> >& A, vector<vector<double> >& out)
{
    int n = (int)A.size();
    vector<vector<double> > M(n, vector<double>(2 * n, 0.0));
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j) M[i][j] = A[i][j];
        M[i][n + i] = 1.0;
    }
    for (int c = 0; c < n; ++c)
    {
        int piv = c;
        for (int r = c + 1; r < n; ++r) if (fabs(M[r][c]) > fabs(M[piv][c])) piv = r;
        if (fabs(M[piv][c]) < 1e-12) return false;
        swap(M[c], M[piv]);
        double d = M[c][c];
        for (int k = 0; k < 2 * n; ++k) M[c][k] /= d;
        for (int r = 0; r < n; ++r)
        {
            if (r == c) continue;
            double f = M[r][c];
            if (f == 0.0) continue;
            for (int k = 0; k < 2 * n; ++k) M[r][k] -= f * M[c][k];
        }
    }
    out.assign(n, vector<double>(n, 0.0));
    for (int i = 0; i < n; ++i) for (int j = 0; j < n; ++j) out[i][j] = M[i][n + j];
    return true;
}

// debugger exports names like "j1_left.pos @ 0x20000940" -> strip the address part
static string ColName(const string& c)
{
    size_t p = c.find_first_of(" \t@");
    return p == string::npos ? c : c.substr(0, p);
}

static bool LoadCsv(const char* path, vector<string>& cols, vector<vector<double> >& data)
{
    FILE* f = fopen(path, "rb");
    if (!f) return false;
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= 0) { fclose(f); return false; }
    string s;
    s.resize((size_t)sz);
    size_t got = fread(&s[0], 1, (size_t)sz, f);
    fclose(f);
    s.resize(got);

    size_t p = 0;
    bool first = true;
    while (p <= s.size())
    {
        size_t e = s.find('\n', p);
        if (e == string::npos) e = s.size();
        string line = s.substr(p, e - p);
        p = e + 1;
        while (!line.empty() && (line[line.size() - 1] == '\r' || line[line.size() - 1] == ' '))
            line.erase(line.size() - 1);
        if (line.empty()) { if (p > s.size()) break; continue; }

        vector<string> fld;
        size_t q = 0;
        while (true)
        {
            size_t c = line.find(',', q);
            if (c == string::npos) { fld.push_back(line.substr(q)); break; }
            fld.push_back(line.substr(q, c - q));
            q = c + 1;
        }
        if (first)
        {
            first = false;
            bool numeric = true;
            for (size_t i = 0; i < fld.size(); ++i)
            {
                char* end = 0;
                strtod(fld[i].c_str(), &end);
                if (end == fld[i].c_str()) { numeric = false; break; }
            }
            if (!numeric) { cols = fld; continue; }
        }
        vector<double> row;
        bool ok = true;
        for (size_t i = 0; i < fld.size(); ++i)
        {
            char* end = 0;
            double v = strtod(fld[i].c_str(), &end);
            if (end == fld[i].c_str()) { ok = false; break; }
            row.push_back(v);
        }
        if (ok && row.size() >= 2) data.push_back(row);
    }
    return !data.empty();
}

static int NCol(const vector<vector<double> >& data)
{
    int m = 1 << 30;
    for (size_t i = 0; i < data.size(); ++i) m = min(m, (int)data[i].size());
    return m == (1 << 30) ? 0 : m;
}

static int ColIndex(const vector<string>& cols, const vector<vector<double> >& data, int joint, const char* kind)
{
    int nc = NCol(data);
    char tag[16];
    sprintf(tag, "j%d", joint);
    string suf = string(".") + kind;
    for (size_t i = 0; i < cols.size() && (int)i < nc; ++i)
    {
        string nm = ColName(cols[i]);
        if (nm.size() <= strlen(tag) + suf.size()) continue;
        if (nm.compare(0, strlen(tag), tag) != 0) continue;
        char nx = nm[strlen(tag)];
        if (nx != '_' && nx != '.') continue;                 // do not let "j1" match "j10"
        if (nm.compare(nm.size() - suf.size(), suf.size(), suf) != 0) continue;
        return (int)i;
    }
    // A header is present -> never guess by position, otherwise pos of one joint
    // can silently be mixed with vel/torque of another one.
    if (!cols.empty()) return -1;
    // positional fallback (only for headerless files)
    if (joint == 1)
    {
        if (!strcmp(kind, "pos") && nc > 1) return 1;
        if (!strcmp(kind, "vel") && nc > 2) return 2;
        if (!strcmp(kind, "torque") && nc > 3) return 3;
    }
    if (!strcmp(kind, "pos") && 3 + joint < nc) return 3 + joint;
    return -1;
}

static vector<double> Unwrap(const vector<double>& raw)
{
    vector<double> out(raw.size(), 0.0);
    if (raw.empty()) return out;
    out[0] = raw[0];
    double off = 0.0;
    for (size_t i = 1; i < raw.size(); ++i)
    {
        double d = raw[i] - raw[i - 1];
        int guard = 0;
        while (d > PI && guard++ < 100) { off -= 2.0 * PI; d -= 2.0 * PI; }
        guard = 0;
        while (d < -PI && guard++ < 100) { off += 2.0 * PI; d += 2.0 * PI; }
        out[i] = raw[i] + off;
    }
    return out;
}

// y(t) = a0 + a1*(t-tc) + a2*(t-tc)^2 over [i0,i1]
static bool LocalQuad(const vector<double>& t, const vector<double>& y, int i0, int i1, int ic,
                      double& v, double& d1, double& d2)
{
    int n = i1 - i0 + 1;
    if (n < 5) return false;
    double tc = t[ic];
    vector<vector<double> > A(3, vector<double>(3, 0.0));
    vector<double> b(3, 0.0);
    for (int i = i0; i <= i1; ++i)
    {
        double u = t[i] - tc;
        double pw[3] = { 1.0, u, u * u };
        for (int r = 0; r < 3; ++r)
        {
            for (int c = 0; c < 3; ++c) A[r][c] += pw[r] * pw[c];
            b[r] += pw[r] * y[i];
        }
    }
    vector<double> x;
    if (!SolveN(A, b, x)) return false;
    v = x[0];
    d1 = x[1];
    d2 = 2.0 * x[2];
    return true;
}

// ---------------- geometry model (same point-mass model as the firmware) ----------------
static void MulRV(const double R[3][3], const double v[3], double o[3])
{
    for (int r = 0; r < 3; ++r) o[r] = R[r][0] * v[0] + R[r][1] * v[1] + R[r][2] * v[2];
}
static void MulRR(const double A[3][3], const double B[3][3], double O[3][3])
{
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            O[r][c] = A[r][0] * B[0][c] + A[r][1] * B[1][c] + A[r][2] * B[2][c];
}
static void Cross(const double a[3], const double b[3], double o[3])
{
    o[0] = a[1] * b[2] - a[2] * b[1];
    o[1] = a[2] * b[0] - a[0] * b[2];
    o[2] = a[0] * b[1] - a[1] * b[0];
}

// chain geometry of the last ModelTau() call, used by the closed-form diagonal check
static double gJP[DOF + 1][3], gCOM[DOF][3], gAX[DOF][3];

// M_ii in closed form: sum of m_k * d_k^2 over the links distal to the joint,
// d_k = distance from the joint axis line to the link COM. This is exactly what the
// firmware adds as the diagonal correction, so it must equal ModelTau(pos,joint,0).
static double ModelDiagClosed(int joint)
{
    int i = joint - 1;
    double s = 0;
    for (int k = i; k < DOF; ++k)
    {
        double r[3] = { gCOM[k][0] - gJP[i][0], gCOM[k][1] - gJP[i][1], gCOM[k][2] - gJP[i][2] };
        double r2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        double rz = r[0] * gAX[i][0] + r[1] * gAX[i][1] + r[2] * gAX[i][2];
        s += MASS[k] * (r2 - rz * rz);
    }
    return s;
}

// Model prediction, mirroring the firmware (roboarm_kinematics.cpp) exactly.
//   mode 0: M_jj  -> unit angular acceleration on the joint axis (inertia term)
//   mode 1: tau_g -> gravity torque about the joint axis (static load)
static double ModelTau(const double pos[DOF], int joint, int mode)
{
    double th[DOF];
    for (int i = 0; i < DOF; ++i) th[i] = THETA_OFF[i] + pos[i] - OFFSET[i];

    double R[3][3] = { {1,0,0},{0,1,0},{0,0,1} };
    double dir[DOF][3], ax[DOF][3];
    for (int i = 0; i < DOF; ++i)
    {
        double ct = cos(th[i]), st = sin(th[i]);
        double ca = cos(LINK_ALPHA[i]), sa = sin(LINK_ALPHA[i]);
        double M[3][3] = { { ct, -st, 0.0 },{ st * ca, ct * ca, -sa },{ st * sa, ct * sa, ca } };
        double Rn[3][3];
        MulRR(R, M, Rn);
        memcpy(R, Rn, sizeof(Rn));
        for (int k = 0; k < 3; ++k) ax[i][k] = R[k][2];       // joint axis = 3rd column of R
        MulRV(R, AL0[i], dir[i]);                              // link direction g = R * mdh.axis
        double nr = sqrt(dir[i][0] * dir[i][0] + dir[i][1] * dir[i][1] + dir[i][2] * dir[i][2]) + 1e-12;
        for (int k = 0; k < 3; ++k) dir[i][k] /= nr;
    }
    double jp[DOF + 1][3], com[DOF][3], rc[DOF][3];
    double o[3] = { 0,0,0 };
    for (int i = 0; i < DOF; ++i)
    {
        for (int k = 0; k < 3; ++k)
        {
            jp[i][k] = o[k];
            com[i][k] = o[k] + 0.5 * LENGTH[i] * dir[i][k];
            rc[i][k] = com[i][k] - jp[i][k];
        }
        for (int k = 0; k < 3; ++k) o[k] += LENGTH[i] * dir[i][k];
    }
    for (int k = 0; k < 3; ++k) jp[DOF][k] = o[k];
    for (int i = 0; i <= DOF; ++i) for (int k = 0; k < 3; ++k) gJP[i][k] = jp[i][k];
    for (int i = 0; i < DOF; ++i)
        for (int k = 0; k < 3; ++k) { gCOM[i][k] = com[i][k]; gAX[i][k] = ax[i][k]; }

    int jt = joint - 1;
    double om[3] = { 0,0,0 }, al[3] = { 0,0,0 }, ao[3] = { 0,0,0 };
    double fd[DOF][3];
    for (int i = 0; i < DOF; ++i)
    {
        double ro[3] = { 0,0,0 };
        if (i > 0) for (int c = 0; c < 3; ++c) ro[c] = jp[i][c] - jp[i - 1][c];
        double tmp[3], t2[3];
        Cross(al, ro, tmp); Cross(om, ro, t2); Cross(om, t2, t2);
        for (int c = 0; c < 3; ++c) ao[c] += tmp[c] + t2[c];
        for (int c = 0; c < 3; ++c) al[c] += (mode == 0 && i == jt ? ax[i][c] : 0.0);
        Cross(al, rc[i], tmp); Cross(om, rc[i], t2); Cross(om, t2, t2);
        double ac[3];
        for (int c = 0; c < 3; ++c) ac[c] = ao[c] + tmp[c] + t2[c];
        if (mode == 0)
        {
            for (int c = 0; c < 3; ++c) fd[i][c] = -MASS[i] * ac[c];
        }
        else
        {
            fd[i][0] = 0.0;
            fd[i][1] = 0.0;
            fd[i][2] = -MASS[i] * 9.81;      // base frame is Z-up
        }
    }
    double f[3] = { 0,0,0 }, n[3] = { 0,0,0 }, tau = 0.0;
    for (int i = DOF - 1; i >= 0; --i)
    {
        double ro[3], tmp[3];
        for (int c = 0; c < 3; ++c) ro[c] = jp[i + 1][c] - jp[i][c];
        Cross(ro, f, tmp);
        for (int c = 0; c < 3; ++c) n[c] += tmp[c];
        for (int c = 0; c < 3; ++c) f[c] += fd[i][c];
        Cross(rc[i], fd[i], tmp);
        for (int c = 0; c < 3; ++c) n[c] += tmp[c];
        if (i == jt) tau = -(ax[i][0] * n[0] + ax[i][1] * n[1] + ax[i][2] * n[2]);
    }
    return tau;
}

// numerical d(tau_g)/dq of the model: tells how much a pose-dependent gravity
// term can leak into the identified J (it acts exactly like a spring K = -dTg/dq)
static double ModelGrad(const double pos[DOF], int joint)
{
    const double d = 0.01;
    double p[DOF], m[DOF];
    for (int i = 0; i < DOF; ++i) { p[i] = pos[i]; m[i] = pos[i]; }
    p[joint - 1] += d;
    m[joint - 1] -= d;
    return (ModelTau(p, joint, 1) - ModelTau(m, joint, 1)) / (2.0 * d);
}

/* design matrix layout used by the segment fits:
     0 = alpha, 1 = omega, [theta - mean], [sgn(omega)], last = 1  */
static void FillRow(double* row, int m, int iJ, int iB, int iK, int iFc, int ib,
                    double alpha, double omega, double thc, double sgn)
{
    for (int p = 0; p < m; ++p) row[p] = 0.0;
    row[iJ] = alpha;
    row[iB] = omega;
    if (iK >= 0) row[iK] = thc;
    if (iFc >= 0) row[iFc] = sgn;
    row[ib] = 1.0;
}

int main(int argc, char** argv)
{
    if (argc < 2) { printf("usage: identify_inertia.exe csv [joint] [-k] [-o out]\n"); return 2; }
    const char* path = argv[1];
    int joint = 0;      // 0 = auto detect which joint has .vel + .torque
    bool useK = false;  // add a theta column (only useful with multi-tone excitation)
    bool subGravity = false;  // subtract the model gravity feedforward from the measured torque
    string outPath;
    for (int i = 2; i < argc; ++i)
    {
        if (!strcmp(argv[i], "-o") && i + 1 < argc) outPath = argv[++i];
        else if (!strcmp(argv[i], "-k")) useK = true;
        else if (!strcmp(argv[i], "-g")) subGravity = true;
        else if (isdigit((unsigned char)argv[i][0])) joint = atoi(argv[i]);
    }
    if (!outPath.empty()) { FILE* r = freopen(outPath.c_str(), "w", stdout); (void)r; }

    vector<string> cols;
    vector<vector<double> > data;
    if (!LoadCsv(path, cols, data)) { printf("cannot read: %s\n", path); return 1; }
    int n = (int)data.size();
    int nc = NCol(data);
    if (n < 200) { printf("too few rows: %d\n", n); return 1; }

    printf("file %s   rows %d   cols %d\n", path, n, nc);
    printf("columns:");
    for (int i = 0; i < nc; ++i)
        printf(" [%d]%s", i, i < (int)cols.size() ? ColName(cols[i]).c_str() : "?");
    printf("\n");

    if (joint == 0)
    {
        for (int j = 1; j <= DOF; ++j)
            if (ColIndex(cols, data, j, "vel") >= 0 && ColIndex(cols, data, j, "torque") >= 0) { joint = j; break; }
        printf("auto-detected tested joint: j%d\n", joint);
        if (joint == 0) { printf("no joint has both .vel and .torque columns\n"); return 1; }
    }

    int ip = ColIndex(cols, data, joint, "pos");
    int iv = ColIndex(cols, data, joint, "vel");
    int it = ColIndex(cols, data, joint, "torque");
    printf("map: j%d.pos=%d j%d.vel=%d j%d.torque=%d%s\n", joint, ip, joint, iv, joint, it,
           it < 0 ? "   (no torque column!)" : "");
    if (ip < 0 || it < 0) return 1;

    vector<double> t(n), raw(n);
    for (int i = 0; i < n; ++i) { t[i] = data[i][0]; raw[i] = data[i][ip]; }
    printf("t = %.3f .. %.3f s (%.1f s)\n", t[0], t[n - 1], t[n - 1] - t[0]);
    double dtm = 0, dtmin = 1e9, dtmax = 0;
    for (int i = 1; i < n; ++i)
    {
        double d = t[i] - t[i - 1];
        dtm += d;
        if (d < dtmin) dtmin = d;
        if (d > dtmax) dtmax = d;
    }
    printf("dt: mean %.5f  min %.5f  max %.5f s\n", dtm / (n - 1), dtmin, dtmax);

    // Debugger exports are often concatenated batches -> timestamps can jump
    // backwards. Mark those samples so that no fit window ever spans a jump.
    vector<char> bad(n, 0);
    int njump = 0;
    for (int i = 1; i < n; ++i)
    {
        double d = t[i] - t[i - 1];
        if (d <= 0.0 || d > 0.1) { bad[i] = 1; ++njump; }
    }
    printf("time jumps: %d (windows never span a jump)\n", njump);

    // A log often starts before the first CAN feedback arrives: every joint reads
    // exactly 0. Those samples do not describe the mechanism -> mark them bad too,
    // otherwise the 0 -> real-pose step looks like a huge acceleration.
    int nzero = 0;
    for (int i = 0; i < n; ++i)
    {
        bool allZero = true;
        for (int j = 1; j <= DOF && allZero; ++j)
        {
            int ci = ColIndex(cols, data, j, "pos");
            if (ci >= 0 && fabs(data[i][ci]) > 1e-9) allZero = false;
        }
        if (allZero) { bad[i] = 1; ++nzero; }
    }
    if (nzero) printf("samples with all joints reading 0 (no CAN feedback yet): %d\n", nzero);

    vector<double> pos = Unwrap(raw);
    vector<double> th(n, 0), om(n, 0), al(n, 0), alv(n, 0);
    vector<double> vel(n, 0);
    for (int i = 0; i < n; ++i) if (iv >= 0) vel[i] = data[i][iv];
    for (int i = 0; i < n; ++i)
    {
        int i0 = i - HALF < 0 ? 0 : i - HALF;
        int i1 = i + HALF > n - 1 ? n - 1 : i + HALF;
        for (int q = i0; q <= i1; ++q)          // keep the window inside one block
        {
            if (!bad[q]) continue;
            if (q <= i) i0 = q + 1;
            else i1 = q - 1;
        }
        double d1, d2;
        if (i1 - i0 >= 5 && LocalQuad(t, pos, i0, i1, i, th[i], d1, d2)) { om[i] = d1; al[i] = d2; }
        else { th[i] = pos[i]; om[i] = al[i] = 0.0; }
        if (iv >= 0 && i1 - i0 >= 5) alv[i] = LocalQuad(t, vel, i0, i1, i, d1, d1, d2) ? d2 : 0.0;
        else alv[i] = 0.0;
    }
    if (iv >= 0)
    {
        double sq = 0, sb = 0;
        for (int i = HALF; i < n - HALF; ++i) { double e = vel[i] - om[i]; sq += e * e; sb += vel[i] * vel[i]; }
        printf("fit-vel vs reported-vel: RMS diff %.4f / reported RMS %.4f rad/s\n",
               sqrt(sq / (n - 2 * HALF)), sqrt(sb / (n - 2 * HALF)));
    }
    double mth = 0, mom = 0, mal = 0, mtau = 0;
    for (int i = 0; i < n; ++i)
    {
        mth = max(mth, fabs(th[i]));
        mom = max(mom, fabs(om[i]));
        mal = max(mal, fabs(al[i]));
        mtau = max(mtau, fabs(data[i][it]));
    }
    printf("peaks: |th| %.3f rad  |om| %.3f rad/s  |al| %.2f rad/s^2  |tau| %.3f N.m\n", mth, mom, mal, mtau);

    // position columns of every joint (pose output + model comparison)
    vector<int> opCol(DOF + 1, -1);
    vector<vector<double> > opos(DOF + 1);
    for (int j = 1; j <= DOF; ++j)
    {
        int ci = ColIndex(cols, data, j, "pos");
        opCol[j] = ci;
        if (ci >= 0)
        {
            vector<double> rw(n);
            for (int i = 0; i < n; ++i) rw[i] = data[i][ci];
            opos[j] = Unwrap(rw);
        }
    }

    // Per-sample model gravity torque. The firmware feeds exactly this value as
    // tg, so the measured torque already contains it; -g removes it before the fit.
    vector<double> tau(n, 0.0), poseBuf(DOF, 0.0);
    double rmsTg = 0;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < DOF; ++j) poseBuf[j] = (opCol[j + 1] >= 0) ? opos[j + 1][i] : 0.0;
        double tgm = ModelTau(&poseBuf[0], joint, 1);
        rmsTg += tgm * tgm;
        tau[i] = data[i][it] - (subGravity ? tgm : 0.0);
    }
    printf("model gravity: rms tau_g = %.4f N.m%s\n", sqrt(rmsTg / n),
           subGravity ? "   (subtracted from the measured torque)" : "");

    // ---- motion segments ----
    vector<pair<int, int> > segs;
    bool inSeg = false;
    int s0 = 0;
    for (int i = 0; i < n; ++i)
    {
        bool mv = fabs(om[i]) > VEL_MOVE || fabs(al[i]) > ACC_MOVE;
        if (mv && !inSeg) { inSeg = true; s0 = i; }
        if (!mv && inSeg) { inSeg = false; if (i - s0 >= MIN_PTS) segs.push_back(make_pair(s0, i - 1)); }
    }
    if (inSeg && n - 1 - s0 >= MIN_PTS) segs.push_back(make_pair(s0, n - 1));

    printf("\nsegments: %d\n", (int)segs.size());
    printf("\nseg  t0(s)     pts  dTheta rmsA  rmsW  J(alpha_pos) +- sd    J(alpha_vel)    B       K       Fc      b      RMS    R2   Mmodel  J/M    tauG_mod\n");

    vector<int> ta0, ta1;
    vector<char> tbidir;
    vector<double> tJ, tJv, tW, tM;

    for (int k = 0; k < (int)segs.size(); ++k)
    {
        int a0 = segs[k].first + HALF, a1 = segs[k].second - HALF;
        int cnt = a1 - a0 + 1;
        if (cnt < 30) continue;

        double lo = 1e30, hi = -1e30, sa = 0, sw = 0;
        int npos = 0, nneg = 0;
        for (int i = a0; i <= a1; ++i)
        {
            lo = min(lo, th[i]);
            hi = max(hi, th[i]);
            sa += al[i] * al[i];
            sw += om[i] * om[i];
            if (om[i] > 0.02) ++npos;
            if (om[i] < -0.02) ++nneg;
        }
        double dth = hi - lo;
        double rmsa = sqrt(sa / cnt), rmsw = sqrt(sw / cnt);
        bool bidir = (npos > cnt / 5) && (nneg > cnt / 5);

        // per-segment least squares: tau = J*a + B*w [+ K*(th-thm)] [+ Fc*sgn(w)] + b
        int m = 2 + (useK ? 1 : 0) + (bidir ? 1 : 0) + 1;
        int iJ = 0, iB = 1, iK = useK ? 2 : -1;
        int iFc = bidir ? (useK ? 3 : 2) : -1;
        int ib = m - 1;
        double thm = 0;
        for (int i = a0; i <= a1; ++i) thm += th[i];
        thm /= cnt;

        vector<vector<double> > A(m, vector<double>(m, 0.0));
        vector<double> b(m, 0.0), y(cnt, 0.0), row(m, 0.0);
        for (int i = a0; i <= a1; ++i)
        {
            int r = i - a0;
            FillRow(&row[0], m, iJ, iB, iK, iFc, ib, al[i], om[i], th[i] - thm,
                    (om[i] > 0 ? 1.0 : -1.0));
            y[r] = tau[i];
            for (int p = 0; p < m; ++p)
            {
                for (int q = 0; q < m; ++q) A[p][q] += row[p] * row[q];
                b[p] += row[p] * y[r];
            }
        }
        vector<double> coef;
        if (!SolveN(A, b, coef)) { printf("%3d  %.3f  fit failed\n", k + 1, t[segs[k].first]); continue; }
        vector<vector<double> > Ai;
        bool haveInv = InvN(A, Ai);
        double ym = 0;
        for (int r = 0; r < cnt; ++r) ym += y[r];
        ym /= cnt;
        double sse = 0, sst = 0;
        for (int i = a0; i <= a1; ++i)
        {
            int r = i - a0;
            FillRow(&row[0], m, iJ, iB, iK, iFc, ib, al[i], om[i], th[i] - thm,
                    (om[i] > 0 ? 1.0 : -1.0));
            double pre = 0;
            for (int p = 0; p < m; ++p) pre += coef[p] * row[p];
            sse += (y[r] - pre) * (y[r] - pre);
            sst += (y[r] - ym) * (y[r] - ym);
        }
        double s2 = cnt > m ? sse / (cnt - m) : 0.0;
        double sdJ = haveInv ? sqrt(fabs(s2 * Ai[iJ][iJ])) : 0.0;
        double rms = sqrt(sse / cnt);
        double r2 = sst > 0 ? 1.0 - sse / sst : 0.0;

        // cross-check using alpha obtained from the reported velocity
        double Jv = 0.0;
        {
            vector<vector<double> > A2(m, vector<double>(m, 0.0));
            vector<double> b2(m, 0.0), row2(m, 0.0);
            for (int i = a0; i <= a1; ++i)
            {
                int r = i - a0;
                FillRow(&row2[0], m, iJ, iB, iK, iFc, ib, alv[i], om[i], th[i] - thm,
                        (om[i] > 0 ? 1.0 : -1.0));
                for (int p = 0; p < m; ++p)
                {
                    for (int q = 0; q < m; ++q) A2[p][q] += row2[p] * row2[q];
                    b2[p] += row2[p] * y[r];
                }
            }
            vector<double> c2;
            if (SolveN(A2, b2, c2)) Jv = c2[iJ];
        }

        double Kv = (iK >= 0) ? coef[iK] : 0.0;
        double Fcc = (iFc >= 0) ? coef[iFc] : 0.0;
        double bb = coef[ib];

        double stau = 0, ptau = 0;
        for (int i = a0; i <= a1; ++i)
        {
            double av = fabs(tau[i]);
            stau += tau[i] * tau[i];
            if (av > ptau) ptau = av;
        }
        double rmsTau = sqrt(stau / cnt);
        double jShare = fabs(coef[iJ]) * rmsa;

        // correlation of the alpha and omega columns: near 1 means the fit cannot
        // tell inertia from damping apart
        double ma = 0, mw = 0, va = 0, vw = 0, vaw = 0;
        for (int i = a0; i <= a1; ++i) { ma += al[i]; mw += om[i]; }
        ma /= cnt;
        mw /= cnt;
        for (int i = a0; i <= a1; ++i)
        {
            va += (al[i] - ma) * (al[i] - ma);
            vw += (om[i] - mw) * (om[i] - mw);
            vaw += (al[i] - ma) * (om[i] - mw);
        }
        double corrAW = (va > 1e-12 && vw > 1e-12) ? vaw / sqrt(va * vw) : 0.0;

        // A segment only counts when the inertial term is a real part of the torque
        // (absolute rms thresholds would wrongly reject slow but well excited axes).
        // If damping/friction explains most of the torque, J and B trade off and the
        // resulting J is meaningless even though the fit looks fine.
        double fricShare = rmsTau > 1e-9 ? fabs(coef[iB]) * rmsw / rmsTau : 0.0;
        bool trust = (dth >= MIN_DTH) && (rmsa > 0.3) && (rmsTau > 1e-9) &&
                     (jShare / rmsTau > 0.25) && (fricShare < 0.8);

        int mid = a0 + cnt / 2;
        double pose[DOF];
        for (int j = 0; j < DOF; ++j)
            pose[j] = (opCol[j + 1] >= 0) ? opos[j + 1][mid] : 0.0;
        double Mmod = ModelTau(pose, joint, 0);
        double Mclosed = ModelDiagClosed(joint);      // same pose -> must equal Mmod
        double tauG = ModelTau(pose, joint, 1);
        double gradG = ModelGrad(pose, joint);

        printf("%3d  %7.3f  %4d  %6.3f %5.2f %6.3f  %9.4f +- %-6.4f %10.4f %8.3f %8.3f %7.3f %8.4f %7.4f %5.3f %8.4f %5.2f %8.4f%s\n",
               k + 1, t[segs[k].first], cnt, dth, rmsa, rmsw, coef[iJ], sdJ, Jv,
               coef[iB], Kv, Fcc, bb, rms, r2, Mmod,
               fabs(Mmod) > 1e-9 ? fabs(coef[iJ] / Mmod) : 0.0, tauG,
               trust ? "" : "  <-low excitation");
        printf("     other-axis travel(rad):");
        for (int j = 2; j <= DOF; ++j)
        {
            if (j == joint) continue;
            if (opCol[j] < 0) continue;
            double l2 = 1e30, h2 = -1e30;
            for (int i = a0; i <= a1; ++i) { l2 = min(l2, opos[j][i]); h2 = max(h2, opos[j][i]); }
            printf(" j%d %.4f", j, h2 - l2);
        }
        printf("\n");
        printf("     pose(mid, rad):");
        for (int j = 1; j <= DOF; ++j) printf(" %.3f", pose[j - 1]);
        printf("\n");
        printf("     model: M=%.4f (closed form %.4f)  tauG=%.4f  dTg/dq=%.4f  -> |J leak| = |dTg/dq|/rmsW^2 = %.3f%s\n",
               Mmod, Mclosed, tauG, gradG, rmsw > 1e-6 ? fabs(gradG) / (rmsw * rmsw) : 0.0,
               subGravity ? "  (leak already removed by -g)" : "  <-- need -g or 2-tone excitation");
        printf("     torque: rms=%.4f peak=%.4f | inertial J*rmsA=%.4f (%.0f%% of rms)"
               " | friction B*rmsW=%.4f | corr(a,w)=%.2f -> %s\n",
               rmsTau, ptau, jShare,
               rmsTau > 1e-9 ? 100.0 * jShare / rmsTau : 0.0,
               fabs(coef[iB]) * rmsw, corrAW,
               trust ? "J is identifiable" : "J NOT identifiable from this log");

        if (trust)
        {
            ta0.push_back(a0);
            ta1.push_back(a1);
            tbidir.push_back(bidir ? 1 : 0);
            tJ.push_back(coef[iJ]);
            tJv.push_back(Jv);
            tW.push_back(rmsa);
            tM.push_back(Mmod);
        }
    }

    int K = (int)tJ.size();
    if (K == 0) { printf("\nno usable segment\n"); return 0; }

    double wsum = 0, who = 0;
    for (int k = 0; k < K; ++k) { wsum += tW[k]; who += tJ[k] * tW[k]; }
    printf("\nusable segments: %d, J per segment: ", K);
    for (int k = 0; k < K; ++k) printf("%.3f ", tJ[k]);
    printf("\n  J excit-weighted mean = %.4f kg.m^2  (range %.4f .. %.4f)\n",
           who / wsum, *min_element(tJ.begin(), tJ.end()), *max_element(tJ.begin(), tJ.end()));

    // ---- global fit: one J per segment, B/Fc/b shared ----
    bool useFc = false;
    for (int k = 0; k < K; ++k) if (tbidir[k]) useFc = true;
    for (int attempt = 0; attempt < 2; ++attempt)
    {
        int m = K + 2 + (useFc ? 1 : 0);
        int iB = K, iFc = K + 1, ib = useFc ? K + 2 : K + 1;
        vector<vector<double> > A(m, vector<double>(m, 0.0));
        vector<double> b(m, 0.0), row(m, 0.0);
        int ntot = 0;
        for (int k = 0; k < K; ++k)
            for (int i = ta0[k]; i <= ta1[k]; ++i)
            {
                for (int p = 0; p < m; ++p) row[p] = 0.0;
                row[k] = al[i];
                row[iB] = om[i];
                if (useFc) row[iFc] = (om[i] > 0 ? 1.0 : -1.0);
                row[ib] = 1.0;
                double yv = tau[i];
                for (int p = 0; p < m; ++p)
                {
                    for (int q = 0; q < m; ++q) A[p][q] += row[p] * row[q];
                    b[p] += row[p] * yv;
                }
                ++ntot;
            }
        vector<double> coef;
        if (!SolveN(A, b, coef))
        {
            if (useFc) { useFc = false; continue; }
            printf("\nglobal fit failed\n");
            break;
        }
        vector<vector<double> > Ai;
        bool haveInv = InvN(A, Ai);
        double sse = 0;
        for (int k = 0; k < K; ++k)
            for (int i = ta0[k]; i <= ta1[k]; ++i)
            {
                double pre = coef[k] * al[i] + coef[iB] * om[i] + coef[ib]
                           + (useFc ? (om[i] > 0 ? coef[iFc] : -coef[iFc]) : 0.0);
                double e = tau[i] - pre;
                sse += e * e;
            }
        printf("\nglobal fit (per-segment J, shared %s): residual RMS = %.4f N.m\n",
               useFc ? "B/Fc/b" : "B/b", sqrt(sse / ntot));
        printf("seg  J(global)   sd      J(per-seg)\n");
        for (int k = 0; k < K; ++k)
        {
            double sdJ = haveInv ? sqrt(fabs((sse / (ntot - m)) * Ai[k][k])) : 0.0;
            printf("%3d  %8.4f  %7.4f   %8.4f\n", k + 1, coef[k], sdJ, tJ[k]);
        }
        printf("shared: B = %.4f N.m.s/rad", coef[iB]);
        if (useFc) printf("  Fc = %.4f N.m", coef[iFc]);
        printf("  b = %.4f N.m\n", coef[ib]);

        // single J over all usable data, for reference
        vector<vector<double> > A3(3, vector<double>(3, 0.0));
        vector<double> b3(3, 0.0);
        for (int k = 0; k < K; ++k)
            for (int i = ta0[k]; i <= ta1[k]; ++i)
            {
                double X2[3] = { al[i], om[i], 1.0 };
                for (int p = 0; p < 3; ++p)
                {
                    for (int q = 0; q < 3; ++q) A3[p][q] += X2[p] * X2[q];
                    b3[p] += X2[p] * tau[i];
                }
            }
        vector<double> c3;
        if (SolveN(A3, b3, c3))
            printf("single J over all data = %.4f kg.m^2 (B=%.4f, b=%.4f)\n", c3[0], c3[1], c3[2]);
        break;
    }

    // ---- model check: does the model reproduce how J changes with the pose? ----
    // Fit  J_measured(k) = a * M_model(pose_k) + c  over the usable segments.
    //   a ~ how much the modelled masses must be scaled (geometry assumed right)
    //   c ~ constant inertia the model cannot contain (base, rotor, harness)
    // j2/j3 gravity will give m*r while this gives m*r^2 -> together they separate
    // mass from length, which J alone can never do.
    if (K >= 3)
    {
        double mM = 0, mJ = 0;
        for (int k = 0; k < K; ++k) { mM += tM[k]; mJ += tJ[k]; }
        mM /= K;
        mJ /= K;
        double sxx = 0, sxy = 0, syy = 0;
        for (int k = 0; k < K; ++k)
        {
            sxx += (tM[k] - mM) * (tM[k] - mM);
            sxy += (tM[k] - mM) * (tJ[k] - mJ);
            syy += (tJ[k] - mJ) * (tJ[k] - mJ);
        }
        printf("\nmodel check: J_k = a*M_model(pose_k) + c\n");
        if (sxx > 1e-12)
        {
            double a = sxy / sxx;
            double c = mJ - a * mM;
            double sse = 0;
            for (int k = 0; k < K; ++k)
            {
                double e = tJ[k] - (a * tM[k] + c);
                sse += e * e;
            }
            printf("  a = %.3f (so kMass should be ~a times larger)   c = %.4f kg.m^2"
                   "   R2 = %.4f   resid RMS = %.4f\n",
                   a, c, syy > 1e-12 ? 1.0 - sse / syy : 0.0, sqrt(sse / K));
            printf("  M_model %.4f .. %.4f   J_measured %.4f .. %.4f  (kg.m^2)\n",
                   *min_element(tM.begin(), tM.end()), *max_element(tM.begin(), tM.end()),
                   *min_element(tJ.begin(), tJ.end()), *max_element(tJ.begin(), tJ.end()));
        }
        else
        {
            printf("  (model M barely changes over these poses: scale and offset are not separable)\n");
        }
    }

    // ---- static holds: measured torque vs the model gravity torque ----
    // At rest tau_meas is the gravity load at that pose (+ friction/harness bias),
    // so this table validates kMass/kLength directly: no dynamics, no excitation.
    {
        printf("\nstatic holds (|w|<0.01 rad/s): measured torque vs model gravity torque\n");
        printf("     t0(s)      n  tau_meas  tau_model     diff   dTg/dq   pose(j1..j6)\n");
        int i = 0, nw = 0;
        while (i < n)
        {
            if (bad[i] || !(fabs(om[i]) < 0.01 && fabs(al[i]) < 0.1)) { ++i; continue; }
            int j2 = i;
            double s = 0;
            while (j2 < n && !bad[j2] && fabs(om[j2]) < 0.01 && fabs(al[j2]) < 0.1) { s += data[j2][it]; ++j2; }
            int cntw = j2 - i;
            if (cntw >= 40)
            {
                int mid = i + cntw / 2;
                double pose[DOF];
                for (int q = 0; q < DOF; ++q) pose[q] = (opCol[q + 1] >= 0) ? opos[q + 1][mid] : 0.0;
                double tg = ModelTau(pose, joint, 1);
                double tm = s / cntw;
                printf("%3d  %8.3f %6d  %8.4f  %9.4f  %7.4f  %8.4f  ", ++nw, t[i], cntw, tm, tg, tm - tg, ModelGrad(pose, joint));
                for (int q = 1; q <= 6; ++q) printf(" %.3f", pose[q - 1]);
                printf("\n");
            }
            i = j2;
        }
        if (nw == 0) printf("     (no static window with >= 40 samples)\n");
    }
    return 0;
}
