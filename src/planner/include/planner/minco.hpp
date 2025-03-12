/* @author: YueLin */

#include "planner/metric.hpp"

#include "utils/lbfgs"
#include "utils/se2.hpp"
#include "utils/esdf.hpp"
#include "utils/trajectory.hpp"

namespace gfm_planner
{
    class PerceptionAwareOptimizer
    {
        public:
            /* Objects */
            ESDF* esdf;
            MINCO* minco;
            GeometricFeatureMetric* metric;

            /* Vectors */
            Eigen::MatrixXd points, dc, dp;
            Eigen::VectorXd tau, times, dt, dv;

            /* Parameters */
            int kappa;                     // Interval for numerical integration
            double fov;                    // Scanning range of LiDAR
            double safe;                   // Safe distance to obstacles
            double duration;               // The maximum duration of each piece
            double vm[2], am[2];           // Maximum velocity and acceleration
            double ws, wt, wl, wp;         // Weights of cost
            lbfgs::lbfgs_parameter_t bfgs;
        
        public:
            PerceptionAwareOptimizer(
                GeometricFeatureMetric*,
                MINCO*, ESDF*,
                double, double, int,              // Optimization parameters
                double, double, double, int,      // Constraint parameters
                double, double, double, double,   // Weights of cost
                double, double, double, double    // Kinematic limits
            );

            double optimize(Trajectory*);
            bool setup(std::vector<SE2>&, Eigen::MatrixXd*);
        
        private:
            inline void penalty(double*, double*);
            inline void forward(Eigen::VectorXd&,
                                Eigen::VectorXd&,
                                Eigen::MatrixXd&);
            inline void backward(const Eigen::VectorXd&);
            
            static double cost(void*, const Eigen::VectorXd&, Eigen::VectorXd&);
    };
}
