/* @author: YueLin, Zhxx */

#include <vector>

#include <opencv2/opencv.hpp>

#include "utils/se2.hpp"
#include "planner/metric.hpp"

namespace gfm_planner
{
    class PerceptionAwareAStar
    {
        private:
            int fov;
            double alpha;
        
        private:
            bool* visited;
            double *g, *h;
            int *parent, *a;
            int H, W, x0, y0, z0, xg, yg, zg;
        
        public:
            ~PerceptionAwareAStar();
            PerceptionAwareAStar(double, double, int, int);
        
        private:
            void initialize(int, bool);

            double f(double cost) {return cost + alpha * h[encode(x0, y0)];}

            int encode(int x, int y) {return x + y * W;}
            void decode(int idx) {y0 = idx / W; x0 = idx - y0 * W; z0 = a[idx];}

        public:
            void heuristic(SE2, GeometricFeatureMetric*);
            std::vector<SE2> plan(GeometricFeatureMetric*, cv::Mat&, SE2, SE2);
    };

    void sampling(float, const cv::Mat&, std::vector<SE2>&);
}