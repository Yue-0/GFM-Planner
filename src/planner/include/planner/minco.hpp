/* @author: YueLin */

#include "planner/metric.hpp"

#include "utils/se2.hpp"
#include "utils/esdf.hpp"
#include "utils/trajectory.hpp"

namespace gfm_planner
{
    class PerceptionAwareOptimizer
    {
        private:
            /* Objects */
            ESDF* esdf;
            MINCO* minco;
            GeometricFeatureMetric* metric;

            /* Vectors */
            Eigen::MatrixXd points, dc, dp;
            Eigen::VectorXd tau, times, dt, dv;

            /* Parameters */
            double fov;                    // Scanning range of LiDAR
            double safe;                   // Safe distance to obstacles
            double duration;               // The maximum duration of each piece
            double vm[2], am[2];           // Maximum velocity and acceleration
            double ws, wt, wl, wp;         // Weights of cost
            
            /* For L-BFGS optimization */
            int past, mem, iterations, kappa;
            Eigen::VectorXd limit, memory, pf;
            double eps, steps, delta, epsilon, wolfe, armijo;
        
        public:
            PerceptionAwareOptimizer(
                GeometricFeatureMetric*, MINCO*, ESDF*,

                /* Constraint parameters */
                double, double, double,

                /* L-BFGS parameters */
                int, int, int, int, 
                double, double, double, double, double, double,

                /* Weights of cost */
                double, double, double, double,

                /* Kinematic limits */
                double, double, double, double
            );

            double optimize(Trajectory*);
            bool setup(std::vector<SE2>&, Eigen::MatrixXd*);
        
        private:
            inline void penalty(double*, double*);
            inline void forward(Eigen::VectorXd&,
                                Eigen::VectorXd&,
                                Eigen::MatrixXd&);
            inline void backward(const Eigen::VectorXd&);
            
            inline double cost(const Eigen::VectorXd&, Eigen::VectorXd&);

            inline double lbfgs(Eigen::VectorXd&);

            inline bool search(Eigen::VectorXd&, double*, 
                               Eigen::VectorXd&, double*,
                               const Eigen::VectorXd&,
                               const Eigen::VectorXd&,
                               const Eigen::VectorXd&);

            bool convergance(const Eigen::VectorXd& x, const Eigen::VectorXd& g)
            {
                return epsilon >= g.cwiseAbs().maxCoeff() / std::max(
                    x.cwiseAbs().maxCoeff(), 1.
                );
            }
    };
}
