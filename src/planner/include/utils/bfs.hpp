/* @author: YueLin */

#pragma once

#include <cmath>
#include <queue>
#include <utility>

#include "opencv2/opencv.hpp"

/* Breadth-First Search Algorithm */
std::pair<int, int> bfs(const cv::Mat& map, int xs, int ys)
{
    /* Constants */
    const int H = map.rows, W = map.cols;

    /* Correct xs and ys */
    xs = std::min(std::max(0, xs), W - 1);
    ys = std::min(std::max(0, ys), H - 1);

    /* Initialize queue and array */
    bool** visited = new bool*[H];
    for(int y = 0; y < H; y++)
    {
        visited[y] = new bool[W];
        std::fill_n(visited[y], W, false);
    }
    std::queue<std::pair<int, int>> queue;

    /* Push the first point into the queue */
    queue.push(std::make_pair(xs, ys));
    visited[ys][xs] = true;

    /* Main loop */
    while(!queue.empty())
    {
        /* Dequeue a point */
        int y0 = queue.front().second;
        int x0 = queue.front().first; 
        queue.pop();
        
        /* Expand the point */
        int xm = std::min(W, x0 + 2);
        int ym = std::min(H, y0 + 2);
        for(int y = std::max(y0 - 1, 0); y < ym; y++)
            for(int x = std::max(x0 - 1, 0); x < xm; x++)
            {
                if(visited[y][x])
                    continue;
                visited[y][x] = true;
                if(!map.at<uchar>(y, x))
                {
                    xs = x; ys = y; goto del;
                }
                queue.push(std::make_pair(x, y));
            }
    }

    del:
        for(int y = 0; y < H; y++)
            delete[] visited[y];
        delete[] visited;
    return std::make_pair(xs, ys);
}