#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <vector>
#include <queue>
#include <algorithm>
#include "WallMap.h"
#include "CellTracker.h"

const uint8_t FF_INF = 255;

struct Pos {
    int r, c;
};

// Two separate distance tables: 
// 1. Live distances updated dynamically during the survey phase.
// 2. Frozen optimal distances used as the A* heuristic.
extern uint8_t live_dist[MAZE_SIZE][MAZE_SIZE];
extern uint8_t flood_dist[MAZE_SIZE][MAZE_SIZE]; 
extern std::vector<MazeHeading> ff_path_moves;

// ==========================================
// GOAL DEFINITIONS
// ==========================================
// Custom target set to top-right corner
inline std::vector<Pos> getGoalCells() {
    return {{5, 5}};
}

inline bool isGoal(int r, int c) {
    for (Pos p : getGoalCells()) {
        if (p.r == r && p.c == c) return true;
    }
    return false;
}

// ==========================================
// PHASE 1: LIVE FLOODFILL (EXPLORATION)
// ==========================================
inline void ffBuildLiveFlood(const std::vector<Pos>& targets) {
    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            live_dist[r][c] = FF_INF;
        }
    }

    std::queue<Pos> q;
    for (auto t : targets) {
        if (t.r >= 0 && t.r < MAZE_SIZE && t.c >= 0 && t.c < MAZE_SIZE) {
            live_dist[t.r][t.c] = 0;
            q.push(t);
        }
    }

    // Multi-source BFS through known open corridors
    while (!q.empty()) {
        Pos curr = q.front();
        q.pop();

        uint8_t d = live_dist[curr.r][curr.c];

        if (canMove(curr.r, curr.c, HEADING_NORTH) && live_dist[curr.r + 1][curr.c] == FF_INF) {
            live_dist[curr.r + 1][curr.c] = d + 1;
            q.push({curr.r + 1, curr.c});
        }
        if (canMove(curr.r, curr.c, HEADING_SOUTH) && live_dist[curr.r - 1][curr.c] == FF_INF) {
            live_dist[curr.r - 1][curr.c] = d + 1;
            q.push({curr.r - 1, curr.c});
        }
        if (canMove(curr.r, curr.c, HEADING_EAST) && live_dist[curr.r][curr.c + 1] == FF_INF) {
            live_dist[curr.r][curr.c + 1] = d + 1;
            q.push({curr.r, curr.c + 1});
        }
        if (canMove(curr.r, curr.c, HEADING_WEST) && live_dist[curr.r][curr.c - 1] == FF_INF) {
            live_dist[curr.r][curr.c - 1] = d + 1;
            q.push({curr.r, curr.c - 1});
        }
    }
}

inline MazeHeading ffGetExploreMove(int r, int c, MazeHeading currentHeading) {
    uint8_t best_val = FF_INF;
    MazeHeading best_dir = currentHeading; // Fallback to straight

    auto tryDir = [&](MazeHeading h, int nr, int nc) {
        if (canMove(r, c, h)) {
            if (live_dist[nr][nc] < best_val) {
                best_val = live_dist[nr][nc];
                best_dir = h;
            } else if (live_dist[nr][nc] == best_val && h == currentHeading) {
                best_dir = h; // Tie-breaker: prefer continuing straight to avoid zig-zags
            }
        }
    };

    tryDir(HEADING_NORTH, r + 1, c);
    tryDir(HEADING_EAST, r, c + 1);
    tryDir(HEADING_SOUTH, r - 1, c);
    tryDir(HEADING_WEST, r, c - 1);

    return best_dir;
}

// ==========================================
// PHASE 2: A* HEURISTIC (POST-EXPLORATION)
// ==========================================
inline void ffBuildHeuristic(const std::vector<Pos>& targets) {
    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            flood_dist[r][c] = FF_INF;
        }
    }

    std::queue<Pos> q;
    for (auto t : targets) {
        if (t.r >= 0 && t.r < MAZE_SIZE && t.c >= 0 && t.c < MAZE_SIZE) {
            flood_dist[t.r][t.c] = 0;
            q.push(t);
        }
    }

    while (!q.empty()) {
        Pos curr = q.front();
        q.pop();

        uint8_t d = flood_dist[curr.r][curr.c];

        if (canMove(curr.r, curr.c, HEADING_NORTH) && flood_dist[curr.r + 1][curr.c] == FF_INF) {
            flood_dist[curr.r + 1][curr.c] = d + 1; q.push({curr.r + 1, curr.c});
        }
        if (canMove(curr.r, curr.c, HEADING_SOUTH) && flood_dist[curr.r - 1][curr.c] == FF_INF) {
            flood_dist[curr.r - 1][curr.c] = d + 1; q.push({curr.r - 1, curr.c});
        }
        if (canMove(curr.r, curr.c, HEADING_EAST) && flood_dist[curr.r][curr.c + 1] == FF_INF) {
            flood_dist[curr.r][curr.c + 1] = d + 1; q.push({curr.r, curr.c + 1});
        }
        if (canMove(curr.r, curr.c, HEADING_WEST) && flood_dist[curr.r][curr.c - 1] == FF_INF) {
            flood_dist[curr.r][curr.c - 1] = d + 1; q.push({curr.r, curr.c - 1});
        }
    }
}

// ==========================================
// A* PATH PLANNER
// ==========================================
struct AStarNode {
    int r, c, f, g;
    bool operator>(const AStarNode& other) const { return f > other.f; }
};

inline bool ffComputePath(int start_r, int start_c, const std::vector<Pos>& targets) {
    ff_path_moves.clear();

    for (auto& t : targets) {
        if (start_r == t.r && start_c == t.c) return true;
    }

    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openSet;
    int gScore[MAZE_SIZE][MAZE_SIZE];
    Pos cameFrom[MAZE_SIZE][MAZE_SIZE];
    bool closedSet[MAZE_SIZE][MAZE_SIZE] = {false};

    for (int r = 0; r < MAZE_SIZE; r++) {
        for (int c = 0; c < MAZE_SIZE; c++) {
            gScore[r][c] = 9999;
            cameFrom[r][c] = {-1, -1};
        }
    }

    gScore[start_r][start_c] = 0;
    openSet.push({start_r, start_c, flood_dist[start_r][start_c], 0});

    Pos endPos = {-1, -1};

    while (!openSet.empty()) {
        AStarNode curr = openSet.top();
        openSet.pop();

        if (closedSet[curr.r][curr.c]) continue;
        closedSet[curr.r][curr.c] = true;

        bool isGoalNode = false;
        for (auto& t : targets) {
            if (curr.r == t.r && curr.c == t.c) {
                isGoalNode = true;
                endPos = {curr.r, curr.c};
                break;
            }
        }
        if (isGoalNode) break;

        auto tryEdge = [&](int nr, int nc, MazeHeading dir) {
            if (canMove(curr.r, curr.c, dir)) {
                if (closedSet[nr][nc]) return;
                int tg = curr.g + 1;
                if (tg < gScore[nr][nc]) {
                    gScore[nr][nc] = tg;
                    cameFrom[nr][nc] = {curr.r, curr.c};
                    int f = tg + flood_dist[nr][nc]; // A* perfect heuristic
                    openSet.push({nr, nc, f, tg});
                }
            }
        };

        tryEdge(curr.r + 1, curr.c, HEADING_NORTH);
        tryEdge(curr.r, curr.c + 1, HEADING_EAST);
        tryEdge(curr.r - 1, curr.c, HEADING_SOUTH);
        tryEdge(curr.r, curr.c - 1, HEADING_WEST);
    }

    if (endPos.r == -1) return false;

    // Reconstruct coordinate path
    std::vector<Pos> path;
    Pos curr = endPos;
    while (curr.r != start_r || curr.c != start_c) {
        path.push_back(curr);
        curr = cameFrom[curr.r][curr.c];
    }
    path.push_back({start_r, start_c});
    std::reverse(path.begin(), path.end());

    // Convert coordinates to directional moves for Solver.h
    for (size_t i = 0; i < path.size() - 1; i++) {
        Pos a = path[i];
        Pos b = path[i + 1];
        if (b.r > a.r) ff_path_moves.push_back(HEADING_NORTH);
        else if (b.r < a.r) ff_path_moves.push_back(HEADING_SOUTH);
        else if (b.c > a.c) ff_path_moves.push_back(HEADING_EAST);
        else if (b.c < a.c) ff_path_moves.push_back(HEADING_WEST);
    }

    return true;
}

// ==========================================
// INTERFACE REQUIRED BY SOLVER.H
// ==========================================
inline int ffGetPathLength() {
    return ff_path_moves.size();
}

inline MazeHeading ffGetMove(int index) {
    if (index < 0 || index >= (int)ff_path_moves.size()) return HEADING_NORTH;
    return ff_path_moves[index];
}

#endif