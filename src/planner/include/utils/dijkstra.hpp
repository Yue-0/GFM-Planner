/* @author: YueLin */

#pragma once

#include <queue>
#include <cfloat>
#include <vector>
#include <utility>

#include "opencv2/opencv.hpp"

/* Dijkstra's Algorithm */
std::vector<int> dijkstra(const cv::Mat& graph)
{
    /* Constant */
    const int N = graph.cols;
    assert(graph.cols == graph.rows);

    /* Result */
    std::vector<int> path;

    /* Initialize arrays and queue */
    int* parent = new int[N];
    bool* visit = new bool[N];
    float* dist = new float[N];
    std::fill_n(parent, N, -1);
    std::fill_n(visit, N, false);
    std::fill_n(dist, N, FLT_MAX);
    std::priority_queue<std::pair<float, int>> queue;

    /* Push the first node into the queue */
    queue.push(std::make_pair(dist[0] = 0, 0));

    /* Main loop */
    while(!queue.empty())
    {
        /* Dequeue a node */
        int u = queue.top().second; queue.pop();

        /* If found a path */
        if(u == N - 1)
        {
            do path.push_back(u);
            while((u = parent[u]) >= 0);
            std::reverse(path.begin(), path.end());
            break;
        }

        /* Expand the node */
        for(int v = u + 1; v < N; v++)
            if(!visit[v] && graph.at<float>(u, v) > 1e-2)
            {
                float d = dist[u] + graph.at<float>(u, v);
                if(d < dist[v])
                {
                    dist[v] = d;
                    parent[v] = u;
                    queue.push(std::make_pair(-d, v));
                }
            }
        visit[u] = true;
    }

    delete[] parent;
    delete[] visit;
    delete[] dist;
    return path;
}