/* @author: YueLin */

#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Eigen>

class Trajectory
{
    private:
        int D, S;
        std::vector<double> times;
        std::vector<Eigen::MatrixXd> coefficients;
    
    public:
        Trajectory() = default;
        Trajectory(int s): D(s * 2 - 1), S(s) {}
    
    private:
        int index(double*);
        Eigen::VectorXd derivative(double, const int, const int);
    
    public:
        /* Get the number of pieces */
        int size() const {return times.size();}

        /* Clear trajectory */
        void clear() {times.clear(); coefficients.clear();}

        /* Get the total duration of the trajectory */
        double duration() {double d = 0; for(double t: times) d += t; return d;}

        /* Get the derivative on the trajectory at time */
        Eigen::VectorXd derivative(double time, const int n){
            int idx = index(&time); return derivative(time, n, idx);
        }

        /* Set durations and coefficients */
        void set(std::vector<double>& t, std::vector<Eigen::MatrixXd>& c){
            times.assign(t.begin(), t.end());
            coefficients.assign(c.begin(), c.end());
        }

        /* Get the point on the trajectory at time */
        Eigen::VectorXd pos(double time) {return derivative(time, 0);}
        /* Get the velocity on the trajectory at time */
        Eigen::VectorXd vel(double time) {return derivative(time, 1);}
        /* Get the acceleration on the trajectory at time */
        Eigen::VectorXd acc(double time) {return derivative(time, 2);}

        /* Get the point on the i-th piece at time */
        Eigen::VectorXd pos(double time, int i) {return derivative(time, 0, i);}
        /* Get the velocity on the i-th piece at time */
        Eigen::VectorXd vel(double time, int i) {return derivative(time, 1, i);}
        /* Get the acceleration on the i-th piece at time */
        Eigen::VectorXd acc(double time, int i) {return derivative(time, 2, i);}
};

class MINCO
{
    private:
        int n;                         // The number of pieces
        int dim;                       // Dimensions of trajectory
        int S, N;                      // S order, N = 2S
        double* a;                     // Banded system
        int** factorial;               // factorial[i][j] = i! / j!
        Eigen::MatrixXi q;             // q(i, j) = (i+S)!(j+S)! / (i!j!(i+j+1))
        Eigen::MatrixXd b;
        Eigen::VectorXd* times;
        Eigen::MatrixXd head, tail;
    
    public:
        ~MINCO();
        MINCO(const int, const int);
    
    private:
        int index(int i, int j) {return (i - j + N) * N * n + j;}
    
    public:
        /* Initialize MINCO */
        void initialize(Eigen::MatrixXd&, Eigen::MatrixXd&, int);
        
        /* Set points and times of MINCO */
        void set(Eigen::MatrixXd&, Eigen::VectorXd&);

        /* Get MINCO trajectory */
        void get(Trajectory*);

        /* Diffeomorphic transformation of time */
        void diffeomorphism(const Eigen::VectorXd&, Eigen::VectorXd&, bool);

        /* Calculate trajectory cost and gradients with time regularization */
        double cost(Eigen::MatrixXd&, Eigen::VectorXd&, double, double);

        /* Propogate gradients to points and times */
        void propogate(Eigen::MatrixXd, Eigen::VectorXd&,
                       Eigen::MatrixXd&, Eigen::VectorXd&);

        /* Propagate the gradient of t to tau */
        void propogate(Eigen::VectorXd&, Eigen::VectorXd&, Eigen::VectorXd&);

        /* Get matrix b */
        const Eigen::MatrixXd& coefficients() const {return b;}

        /* Get the number of pieces */
        int pieces() const {return n;}

        /* Get order = 2s - 1 */
        int order() {return N - 1;}

        /* Calculate n! / m! */
        int div(int n, int m) {return factorial[n][m];}
};

int Trajectory::index(double* time)
{
    int idx = 0;
    const int N = size() - 1;
    while(idx < N && *time > times[idx])
        *time -= times[idx++];
    return idx;
}

Eigen::VectorXd Trajectory::derivative(double time, const int n, const int idx)
{
    int s = D - n;
    assert(s >= 0);
    double p = time;
    Eigen::VectorXd d = coefficients[idx].col(s);
    for(int order = n + 1; s; ++order, p *= time)
    {
        int c = 1;
        for(int i = 0; i < n; i++)
            c *= order + i;
        d += c * p * coefficients[idx].col(--s);
    }
    return d;
}

MINCO::~MINCO()
{
    if(a != nullptr) delete[] a;
    for(int i = 0; i < N; i++)
        delete[] factorial[i];
    delete[] factorial;
    delete[] times;
}

MINCO::MINCO(const int s = 3, const int d = 3): dim(d), a(nullptr)
{
    /* Set the order */
    N = (S = s) << 1;

    /* Allocate space for time vectors */
    times = new Eigen::VectorXd[N];

    /* Allocate space for the factorial matrix */
    factorial = new int*[N];
    for(int i = 0; i < N; i++)
    {
        factorial[i] = new int[N];
        std::fill_n(factorial[i], N, 0);
    }

    /* Initialize the factorial matrix */
    for(int i = *factorial[0] = 1; i < N; i++)
    {
        *factorial[i] = i ** factorial[i - 1];
        for(int j = 1; j <= i; j++)
            factorial[i][j] = *factorial[i] / *factorial[j];
    }

    /* Initialize matrix Q */
    q = Eigen::MatrixXi::Zero(S, S);
    for(int i = 0; i < s; i++)
        for(int j = 0; j < s; j++)
            q(i, j) = factorial[i + s][i] * factorial[j + s][j] / (i + j + 1);
}

void MINCO::initialize(Eigen::MatrixXd& start, Eigen::MatrixXd& end, int p)
{
    n = p;
    tail = end;
    head = start;

    b.resize(N * n, dim);
    if(a != nullptr) delete[] a;
    a = new double[n * N * (N * 2 + 1)];
    std::fill_n(a, n * N * (N * 2 + 1), 0.);

    for(p = 0; p < N - 1; p++)
        times[p].resize(n);
    times[0].setOnes();
}

void MINCO::set(Eigen::MatrixXd& points, Eigen::VectorXd& durations)
{
    /* Calculate times */
    for(int i = 1; i < N; i++)
        times[i] = times[i - 1].cwiseProduct(durations);
    
    /* Calculate matrix b */
    b.setZero();
    for(int i = 0; i < S; i++)
        b.row(i) = head.col(i).transpose();
    for(int i = 0; i < n - 1; i++)
        b.row(N * i + N - 1) = points.col(i).transpose();
    for(int i = 0; i < S; i++)
        b.row(N * n - S + i) = tail.col(i).transpose();
    
    /* Calculate array a */
    std::fill_n(a, n * N * (N * 2 + 1), 0.);
    for(int i = 0; i < S; i++)
        a[index(i, i)] = *factorial[i];
    for(int i = 0; i < n - 1; i++)
    {
        int idx, offset = N * i;
        
        for(int s = 0, len = S; s < S - 1; s++, len--)
        {
            idx = S + s + offset;
            for(int j = 0; j < len; j++)
                a[index(idx, idx + j)] = times[j][i] * factorial[S + s + j][j];
            a[index(idx, idx + N)] = -*factorial[S + s];
        }

        idx = offset + N - 1;
        for(int j = 0; j < N; j++)
            a[index(idx, offset + j)] = times[j][i];

        for(int s = 0; s < S; s++)
        {
            idx = N + s + offset;
            for(int j = s; j < N; j++)
                a[index(idx, offset + j)] = times[j - s][i] 
                                           * factorial[j][j - s];
            a[index(idx, idx)] = -*factorial[s];
        }
    }
    for(int i = S, offset = N * n; i; i--)
        for(int j = i + S, s = 0; j; j--)
            a[index(offset - i, offset - j)] = times[s++][n - 1] 
                                             * factorial[N - j][N - j + i - S];
    
    /* Banded LU factorization */
    const int M = n * N - 1;
    for(int k = 0; k < M; k++)
    {
        double c = a[index(k, k)];
        int s = std::min(k + N, M);
        for(int i = k + 1; i <= s; i++)
        {
            int idx = index(i, k);
            if(a[idx]) a[idx] /= c;
        }
        for(int j = k + 1; j <= s; j++)
            if((c = a[index(k, j)]))
                for(int i = k + 1; i <= s; i++)
                {
                    int idx = index(i, k);
                    if(a[idx]) a[index(i, j)] -= a[idx] * c;
                }
    }

    /* Slove b */
    for(int j = 0; j <= M; j++)
    {
        int s = std::min(j + N, M);
        for(int i = j + 1; i <= s; i++)
        {
            int idx = index(i, j);
            if(a[idx]) b.row(i) -= a[idx] * b.row(j);
        }
    }
    for(int j = M; j >= 0; j--)
    {
        b.row(j) /= a[index(j, j)];
        for(int i = std::max(j - N, 0); i <= j - 1; i++)
        {
            int idx = index(i, j);
            if(a[idx]) b.row(i) -= a[idx] * b.row(j);
        }
    }
}

void MINCO::get(Trajectory* path)
{
    std::vector<double> t(n);
    std::vector<Eigen::MatrixXd> c(n);
    for(int i = 0; i < n; i++)
    {
        t[i] = times[1][i];
        c[i] = b.block(i * N, 0, N, dim).transpose().rowwise().reverse();
    }
    path->set(t, c);
}

void MINCO::diffeomorphism(const Eigen::VectorXd& input,
                           Eigen::VectorXd& output, bool forward)
{
    output.resize(n);

    /* Forward tau -> t */
    if(forward) for(int i = 0; i < n; i++)
        output[i] = input[i] > 0.
                  ? (0.5 * input[i] + 1.) * input[i] + 1.
                  : 1. / ((0.5 * input[i] - 1.) * input[i] + 1.);
    
    /* Backward t -> tau */
    else for(int i = 0; i < n; i++)
        output[i] = input[i] > 1.
                  ? std::sqrt(2. * input[i] - 1.) - 1.
                  : 1. - std::sqrt(2. / input[i] - 1.);
}

double MINCO::cost(Eigen::MatrixXd& dc, Eigen::VectorXd& dt,
                   double weight, double regular)
{
    /* Calculate trajectory cost partial gradient by coefficients */
    dc.resize(N * n, dim); dc.setZero();
    for(int p = 0; p < n; p++)
    {
        int idx = p * N + S;
        for(int i = 0; i < S; i++)
            for(int j = 0; j < S; j++)
                dc.row(idx + i) += 2 * weight * q(i, j) * times[i + j + 1][p] 
                *b.row(idx + j);
    }
    
    /* Calculate trajectory cost and its partial gradient by times */
    dt.resize(n);
    dt.setZero();
    double e, energy = 0;
    for(int p = 0; p < n; p++)
    {
        int idx = p * N + S;
        for(int i = 0; i < S; i++)
            for(int j = i, t = i; j >= 0; j--)
            {
                e = weight
                  * q(i, i - j) * (j? 2: 1)
                  * b.row(idx + i).dot(b.row(idx + i - j)); 
                dt[p] += (t + 1) * e * times[t][p];
                energy += e * times[++t][p];
            }

        /* Time regularization term */
        energy += regular * times[1][p];
        dt[p] += regular;
    }
    return energy;
}

void MINCO::propogate(Eigen::MatrixXd dc, Eigen::VectorXd& dt0,
                      Eigen::MatrixXd& dp, Eigen::VectorXd& dt)
{
    /* Propogate gradient to points */
    dp.resize(dim, n - 1);
    const int M = N * n - 1;
    for(int j = 0; j <= M; j++)
    {
        int s = std::min(j + N, M);
        dc.row(j) /= a[index(j, j)];
        for(int i = j + 1; i <= s; i++)
        {
            int idx = index(j, i);
            if(a[idx]) dc.row(i) -= a[idx] * dc.row(j);
        }
    }
    for(int j = M; j >= 0; j--)
    {
        int s = std::max(j - N, 0);
        for(int i = s; i <= j - 1; i++)
        {
            int idx = index(j, i);
            if(a[idx]) dc.row(i) -= a[idx] * dc.row(j);
        }
    }
    for(int i = 0; i < n - 1; i++)
        dp.col(i) = dc.row(N * (i + 1) - 1).transpose();
    
    /* Propogate gradient to times */
    dt.resize(n);
    int offset = 0;
    Eigen::MatrixXd m = Eigen::MatrixXd::Zero(N, dim);
    for(int p = 0; p < n - 1; offset = (m.setZero(), N * ++p))
    {
        for(int i = 1, s = N - 1; i < N - 1; s--, i++)
        {
            int idx = (i + S - 1) % N;
            for(int j = 0; j < s; j++)
                m.row(idx) -= times[j][p]
                            * factorial[i + j][j] 
                            * b.row(offset + i + j);
        }
        m.row(S - 1) = m.row(S);
        dt[p] = m.cwiseProduct(dc.block(offset + S, 0, N, dim)).sum();
    }
    for(int i = 1, s = N - 1; i <= S; s--, i++)
        for(int j = 0; j < s; j++)
            m.row(i - 1) -= times[j][n - 1] 
                          * factorial[i + j][j]
                          * b.row(offset + i + j);
    dt[n - 1] = m.block(0, 0, S, dim).cwiseProduct(
        dc.block(n * N - S, 0, S, dim)
    ).sum(); dt += dt0;
}

void MINCO::propogate(Eigen::VectorXd& tau, 
                      Eigen::VectorXd& input,
                      Eigen::VectorXd& output)
{
    for(int i = 0; i < n; i++)
        output[i] = tau[i] > 0
                  ? input[i] * (1. + tau[i])
                  : input[i] * (1. - tau[i])
                  / std::pow(tau[i] * (tau[i] * 0.5 - 1.) + 1., 2);
}