/* @author: YueLin */

#include "planner/minco.hpp"

namespace gfm_planner
{
    PerceptionAwareOptimizer::PerceptionAwareOptimizer(
        GeometricFeatureMetric* gfm,
        MINCO* trajectory, ESDF* sdf,
        double delta, double step, int memory,
        double ds, double time, double range, int k,
        double lambda_s, double rho, double lambda_l, double lambda_p,
        double max_vel, double max_acc, double max_omega, double max_alpha
    ){
        /* Objects */
        esdf = sdf;
        metric = gfm;
        minco = trajectory;

        safe = ds;        // Safe distance
        kappa = k;        // Interval for numerical integration
        fov = range;      // Scanning range of LiDAR
        duration = time;  // The maximum duration of each piece

        /* Optimization parameters */
        bfgs.delta = delta;
        bfgs.g_epsilon = 0;
        bfgs.min_step = step;
        bfgs.mem_size = memory;

        /* Weights of cost */
        wt = rho;
        ws = lambda_s;
        wl = lambda_l;
        wp = lambda_p;

        /* Kinematic limits */
        vm[0] = max_vel;
        vm[1] = max_omega;
        am[0] = max_acc * max_acc;
        am[1] = max_alpha * max_alpha;
    }

    double PerceptionAwareOptimizer::optimize(Trajectory* trajectory)
    {
        /* Initialize */
        Eigen::VectorXd var(minco->pieces() * 4 - 3);
        forward(var, tau, points);

        /* Optimization */
        double value;
        lbfgs::lbfgs_optimize(var, value, cost, nullptr, nullptr, this, bfgs);

        /* Get the optimal points and times */
        backward(var);
        minco->diffeomorphism(tau, times, true);

        /* Set the optimal parameters of MINCO */
        minco->set(points, times);
        minco->get(trajectory);
        return value;
    }

    bool PerceptionAwareOptimizer::setup(std::vector<SE2>& path,
                                         Eigen::MatrixXd* states)
    {
        int n = path.size();
        if(--n <= 0) return false;

        /* Get t and q of MINCO */
        std::vector<double> time;
        std::vector<double> q[3];
        double x, y, z, t, d, v = 1 / vm[0];
        int omega = vm[1] * duration * 32 / PI;
        double x0 = states[0](0, 0) = path[0].x;
        double y0 = states[0](1, 0) = path[0].y;
        double z0 = states[0](2, 0) = path[0].yaw;
        for(int p = 0; p < n; x0 = x, y0 = y)
        {
            z = 0;
            x = path[++p].x; y = path[p].y;
            t = 2 * v * std::hypot(x - x0, y - y0);
            for(int piece = 1; (d = piece * duration) < t; piece++)
            {
                double m = metric->max();
                time.push_back(duration);
                q[0].push_back(x0 + d * (x - x0) / t);
                q[1].push_back(y0 + d * (y - y0) / t);
                int px = q[0].back() * metric->resolution;
                int py = q[1].back() * metric->resolution;
                for(int i = -omega, pz = z0 * 32 / PI; i <= omega; i++)
                {
                    int index = (pz + i) & 0x3F;
                    if((d = metric->evaluate(px, py, index, fov * 32)) < m)
                        z = index, m = d;
                }
                q[2].push_back(z0 = z * PI * 0.03125);
            }
            q[0].push_back(x);
            q[1].push_back(y);
            q[2].push_back(path[p].yaw);
            time.push_back(duration + t - d);
        }
        states[1](0, 0) = x;
        states[1](1, 0) = y;
        states[1](2, 0) = path.back().yaw;

        /* Initialize times and points */
        n = time.size();
        times.resize(n);
        points.resize(3, n - 1);

        /* Initialize gradients */
        dt.resize(n);
        dv.resize(n);
        dp.resize(3, n - 1);
        dc.resize(n-- * (minco->order() + 1) * 2, 3);

        /* Set times and points */
        z0 = path[0].yaw;
        for(int p = 0; p < n; p++)
        {
            times[p] = time[p];
            for(int axis = 0; axis < 3; axis++)
                points(axis, p) = q[axis][p];
            if(std::fabs(q[2][p] - z0) > PI)
                points(2, p) += 2 * PI * std::floor(
                    (z0 - q[2][p]) / (PI * 2) + 0.5
                );
            z0 = points(2, p);
        }
        times[n] = time[n];
        if(std::fabs(states[1](2, 0) - z0) > PI)
            states[1](2, 0) += 2 * PI * std::floor(
                (z0 - states[1](2, 0)) / (PI * 2) + 0.5
            );

        /* Initialize MINCO */
        minco->initialize(states[0], states[1], n + 1);
        minco->diffeomorphism(times, tau, false);
        return true;
    }

    inline void PerceptionAwareOptimizer::forward(Eigen::VectorXd& data,
                                                  Eigen::VectorXd& vector,
                                                  Eigen::MatrixXd& matrix)
    {
        const int n = minco->pieces() - 1;
        for(int axis = 0; axis < 3; axis++)
            data.segment(n * (axis + 1) + 1, n) 
            = matrix.block(axis, 0, 1, n).transpose();
        data.head(n + 1) = vector;
    }

    inline void PerceptionAwareOptimizer::backward(const Eigen::VectorXd& x)
    {
        const int n = minco->pieces() - 1;
        for(int axis = 0; axis < 3; axis++)
            points.block(axis, 0, 1, n) = x.segment(
                n * (axis + 1) + 1, n
            ).transpose();
        tau = x.head(n + 1);
    }

    inline void PerceptionAwareOptimizer::penalty(double* constraint, 
                                                  double* localization)
    {
        double g, p;
        double costs[2] = {0, 0};
        const double k = 1. / kappa;
        const int S = minco->order() + 1;
        const int N = minco->pieces();
        const int P = S >> 1;

        Eigen::Vector3d grad;
        Eigen::Vector2d point;
        Eigen::Vector3d states[4];
        std::vector<double> t(S, 1);
        const Eigen::MatrixXd& b = minco->coefficients();
        Eigen::MatrixX4d beta = Eigen::MatrixX4d::Zero(S, 4);

        for(int i = 0; i < N; i++)
        {
            const Eigen::MatrixXd& c = b.block(i * S, 0, S, 3).transpose();
            for(int j = 0; j <= kappa; j++)
            {
                for(int s = 1; s < S; s++)
                    t[s] = t[s - 1] * j * k * times[i];
                for(int order = 0; order <= 3; order++)
                {
                    for(int div = 0, s = order; s < S; div++, s++)
                        beta(s, order) = t[div] * minco->div(s, div);
                    states[order] = c * beta.col(order);
                }
                
                g = wp * k * (!j || j == kappa? 0.5: 1);

                /* Velocity penalty */
                point = states[1].head(2);
                if((p = point.squaredNorm() - vm[0] * vm[0]) > 0)
                {
                    costs[1] += g * p * times[i];
                    dc.block(i * S, 0, S, 2) += 2 * g * times[i]
                                              * beta.col(1) * point.transpose();
                    dt[i] += g * (p + 2 * t[1] * point.dot(states[2].head(2)));
                }

                /* Angular velocity penalty */
                point[0] = states[1][2];
                if((p = point[0] * point[0] - vm[1] * vm[1]) > 0)
                {
                    costs[1] += g * p * times[i];
                    dc.block(i * S, 2, S, 1) += 2 * g 
                                              * times[i] 
                                              * point[0]
                                              * beta.col(1);
                    dt[i] += g * (p + 2 * t[1] * point[0] * states[2][2]);
                }

                /* Acceleration penalty */
                point = states[2].head(2);
                if((p = point.squaredNorm() - am[0]) > 0)
                {
                    costs[1] += g * p * times[i];
                    dc.block(i * S, 0, S, 2) += 2 * g * times[i]
                                              * beta.col(2) * point.transpose();
                    dt[i] += g * (p + 2 * t[1] * point.dot(states[3].head(2)));
                }

                /* Angular acceleration penalty */
                point[0] = states[2][2];
                if((p = point[0] * point[0] - am[1]) > 0)
                {
                    costs[1] += g * p * times[i];
                    dc.block(i * S, 2, S, 1) += 2 * g 
                                              * times[i] 
                                              * point[0]
                                              * beta.col(2);
                    dt[i] += g * (p + 2 * t[1] * point[0] * states[3][2]);
                }

                /* Safety penalty */
                if((p = safe - esdf->get(states[0], grad)) > 0)
                {
                    g *= std::pow(p, P - 1);
                    dc.block(i * S, 0, S, 3) -= P * g 
                                              * times[i] 
                                              * beta.col(0)
                                              * grad.transpose();
                    dt[i] += g * (p - P * t[1] * grad.dot(states[1]));
                    costs[1] += p * times[i] * g;
                }

                /* Localization cost */
                g = wl * k * (!j || j == kappa? 0.5: 1);
                p = metric->evaluate(states[0], grad, fov);
                dc.block(i * S, 0, S, 3) += g
                                          * times[i]
                                          * beta.col(0)
                                          * grad.transpose();
                dt[i] += g * (p + t[1] * grad.dot(states[1]));
                costs[0] += p * g * times[i];
            }
        }
        *localization = *costs;
        *constraint = *(costs + 1);
    }

    double PerceptionAwareOptimizer::cost(void* perception_optimizer, 
                                          const Eigen::VectorXd& x, 
                                          Eigen::VectorXd& gradient)
    {
        double costs[3];
        
        PerceptionAwareOptimizer* 
        optim = reinterpret_cast<
        PerceptionAwareOptimizer*
        > (perception_optimizer);

        optim->backward(x);

        optim->minco->diffeomorphism(optim->tau, optim->times, true);
        optim->minco->set(optim->points, optim->times);

        *costs = optim->minco->cost(optim->dc, optim->dt, optim->ws, optim->wt);
        optim->penalty(costs + 2, costs + 1);

        // std::cout << "energy: " << costs[0] << std::endl;
        // std::cout << "metric: " << costs[1] << std::endl;
        // std::cout << "penalty: " << costs[2] << std::endl;
        
        optim->minco->propogate(optim->dc, optim->dt, optim->dp, optim->dv);
        optim->minco->propogate(optim->tau, optim->dv, optim->dv);

        optim->forward(gradient, optim->dv, optim->dp);

        return costs[0] + costs[1] + costs[2];
    }
}
