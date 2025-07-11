/* @author: YueLin, Zhxx */

#include <cfloat>
#include <algorithm>

#include "utils/bfs.hpp"
#include "planner/astar.hpp"
#include "utils/dijkstra.hpp"

namespace gfm_planner
{
    PerceptionAwareAStar::~PerceptionAwareAStar()
    {
        delete[] a;
        delete[] g;
        delete[] h;
        delete[] parent;
        delete[] visited;
    }

    PerceptionAwareAStar::PerceptionAwareAStar(double range, double weight,
                                               int height, int width):
                                               fov(32 * range), 
                                               alpha(weight), 
                                               H(height), 
                                               W(width)
    {
        const int N = height * width;
        visited = new bool[N];
        parent = new int[N];
        g = new double[N];
        h = new double[N];
        a = new int[N];
    }

    void PerceptionAwareAStar::initialize(int size, bool pre)
    {
        for(int i = 0; i < size; i++)
        {
            visited[i] = false;
            if(pre) h[i] = size;
            else
            {
                a[i] = 0;
                g[i] = size;
                parent[i] = -1;
            }
        }
    }

    void PerceptionAwareAStar::heuristic(SE2 goal, GeometricFeatureMetric* gfm)
    {
        /* Initialize */
        initialize(H * W, true);
        std::priority_queue<std::pair<double, int>> queue;

        /* Set point */
        goal.integer(gfm->resolution, &xg, &yg, &zg);
        bool obstacle = zero(gfm->max() - gfm->evaluate(
            x0 = std::max(std::min(xg, W - 1), 0), 
            y0 = std::max(std::min(yg, H - 1), 0)
        ));

        /* Push the first point into the queue */
        int index = encode(x0, y0);
        queue.push(std::make_pair(h[index] = 0, index));

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a point */
            decode(index = queue.top().second);
            visited[index] = true; queue.pop();

            /* Expand the point */
            int xm = std::min(W, x0 + 2);
            int ym = std::min(H, y0 + 2);
            for(int y = std::max(y0 - 1, 0); y < ym; y++)
                for(int x = std::max(x0 - 1, 0); x < xm; x++)
                {
                    int idx = encode(x, y);
                    if(visited[idx]) continue;
                    
                    /* Calculate cost value */
                    double cost = gfm->evaluate(x, y);
                    if(!obstacle && zero(cost - gfm->max())) continue;

                    /* Update the point */
                    cost += h[index];
                    if(cost < h[idx])
                    {
                        h[idx] = cost;
                        queue.push(std::make_pair(-cost, idx));
                    }
                }
        }
    }

    std::vector<SE2> PerceptionAwareAStar::plan(
        GeometricFeatureMetric* metric,
        cv::Mat& map, SE2 start, SE2 goal
    ){
        /* Initialize arrays and queue */
        int index;
        std::vector<SE2> path;
        initialize(map.cols * map.rows, false);
        std::priority_queue<std::pair<double, int>> queue;

        /* Set start point and goal point */
        start.integer(metric->resolution, &x0, &y0, &z0);
        goal.integer(metric->resolution, &xg, &yg, &zg);
        x0 = std::max(std::min(x0, W - 1), 0);
        y0 = std::max(std::min(y0, H - 1), 0);
        xg = std::max(std::min(xg, W - 1), 0);
        yg = std::max(std::min(yg, H - 1), 0);
        if(map.at<uchar>(y0, x0))
        {
            std::pair<int, int> corrected = bfs(map, x0, y0);
            x0 = corrected.first, y0 = corrected.second;
        }
        if(map.at<uchar>(yg, xg))
        {
            std::pair<int, int> corrected = bfs(map, xg, yg);
            xg = corrected.first, yg = corrected.second;
        }

        /* Push the first point into the queue */
        a[index = encode(x0, y0)] = z0;
        g[index] = metric->evaluate(x0, y0, z0, fov);
        queue.push(std::make_pair(-f(g[index]), index));

        /* Main loop */
        while(!queue.empty())
        {
            /* Dequeue a point */
            decode(index = queue.top().second); queue.pop();

            /* Check visited */
            if(visited[index])
                continue;
            visited[index] = true;

            /* If found a path */
            if(x0 == xg && y0 == yg)
            {
                path.push_back(goal);
                float resolution = 1 / metric->resolution;
                while((index = parent[index]) != -1)
                {
                    decode(index);
                    path.push_back(SE2(
                        x0 * resolution, 
                        y0 * resolution, 
                        z0 * PI * 0.03125
                    ));
                }
                path.pop_back();
                path.push_back(start);
                std::reverse(path.begin(), path.end());
                break;
            }

            /* Expand the point */
            int xm = std::min(W, x0 + 2);
            int ym = std::min(H, y0 + 2);
            for(int y = std::max(y0 - 1, 0); y < ym; y++)
                for(int x = std::max(x0 - 1, 0); x < xm; x++)
                {
                    int idx = encode(x, y);
                    if(visited[idx] || map.at<uchar>(y, x))
                        continue;
                    
                    /* Calculate cost value */
                    int n, z = z0;
                    double m, cost = 1;
                    for(int dz = -1; dz <= 1; dz++)
                    {
                        n = (z0 + dz) & 0x3F;
                        m = metric->evaluate(x, y, n, fov);
                        if(m < cost) {z = n; cost = m;}
                    }

                    /* Update the point */
                    cost += g[index];
                    if(cost < g[idx])
                    {
                        a[idx] = z;
                        g[idx] = cost;
                        parent[idx] = index;
                        queue.push(std::make_pair(-f(cost), idx));
                    }
                }
        }
        return path;
    }

    void sampling(float resolution, const cv::Mat& map, std::vector<SE2>& path)
    {
        int N = path.size();
        if(N < 3) return;

        /* Get keypoints */
        std::vector<SE2> keypoints;
        keypoints.push_back(path[0]);
        for(int x1, x2, x3, y1, y2, y3, i = 3; i < N; i++)
        {
            path[i - 3].integer(resolution, &x1, &y1);
            path[i - 2].integer(resolution, &x2, &y2);
            path[i - 1].integer(resolution, &x3, &y3);
            if((x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2))
                keypoints.push_back(path[i - 2]);
        }
        keypoints.push_back(path.back());

        /* Construct graph */
        N = keypoints.size();
        cv::Mat graph = cv::Mat(N, N, CV_32FC1) * 0;
        for(int i = 0; i < N; i++)
        {
            cv::Point p1;
            keypoints[i].integer(resolution, &p1.x, &p1.y);
            for(int j = i + 1; j < N; j++)
            {
                cv::Point p2;
                bool connected = true;
                keypoints[j].integer(resolution, &p2.x, &p2.y);
                if(j - i > 1)
                {
                    cv::LineIterator line(map, p1, p2);
                    for(int k = 0; k < line.count; k++)
                    {
                        cv::Point p = line++.pos();
                        if(map.at<uchar>(p.y, p.x))
                        {
                            connected = false; break;
                        }
                    }
                }
                if(connected)
                {
                    p2 -= p1;
                    graph.at<float>(i, j) = std::sqrt(p2.dot(p2));
                }
            }
        }

        /* Sampling keypoints */
        std::vector<int> indexes = dijkstra(graph);
        path.resize(N = indexes.size());
        for(int i = 0; i < N; i++)
            path[i] = keypoints[indexes[i]];
    }
}
