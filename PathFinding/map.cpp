#include "map.h"
#include <queue>
#include <algorithm>

Map::Map(int width, int height)
    : width(width), height(height)
{
    map_grid.resize(height, std::vector<bool>(width, false));

    const int box_left = 8;
    const int box_right = 28;
    const int box_top = 10;
    const int box_bottom = 24;

    for (int x = box_left + 1; x <= box_right + 1; ++x)
    {
        map_grid[box_top][x] = true;
        map_grid[box_top + 1][x] = true;
    }

    for (int x = box_left + 1; x <= box_right + 1; ++x)
    {
        map_grid[box_bottom][x] = true;
        map_grid[box_bottom + 1][x] = true;
    }

    for (int y = box_top; y <= box_bottom + 1; ++y)
    {
        map_grid[y][box_right] = true;
        map_grid[y][box_right + 1] = true;
    }
}

Map::~Map()
{
    if (texture)
        SDL_DestroyTexture(texture);
}

bool Map::is_wall(int x, int y) const
{
    if (x < 0 || y < 0 || x >= width || y >= height)
        return true;
    return map_grid[y][x];
}

void Map::bake_texture(SDL_Renderer* renderer)
{
    int tex_w = width * cell_size;
    int tex_h = height * cell_size;

    texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_RGBA8888,
        SDL_TEXTUREACCESS_TARGET,
        tex_w, tex_h);
    if (!texture) {
        SDL_Log("创建纹理失败: %s", SDL_GetError());
        return;
    }

    SDL_Texture* old_target = SDL_GetRenderTarget(renderer);
    SDL_SetRenderTarget(renderer, texture);

    // 1. 深色背景
    SDL_SetRenderDrawColor(renderer, 30, 30, 40, 255);
    SDL_RenderClear(renderer);

    // 2. 绘制墙壁（暗青色/深灰色）
    SDL_SetRenderDrawColor(renderer, 70, 80, 90, 255);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (map_grid[y][x]) {
                SDL_FRect rect = {
                    static_cast<float>(x * cell_size),
                    static_cast<float>(y * cell_size),
                    static_cast<float>(cell_size),
                    static_cast<float>(cell_size)
                };
                SDL_RenderFillRect(renderer, &rect);
            }
        }
    }

    // 3. 绘制每个可通行格子的方向箭头（青色）
    SDL_SetRenderDrawColor(renderer, 0, 200, 200, 255);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            if (map_grid[y][x]) continue;               // 墙壁跳过
            Vector2 dir = flow_field[y][x];
            if (dir.x == 0 && dir.y == 0) continue;     // 目标点或无法到达

            float centerX = (x + 0.5f) * cell_size;
            float centerY = (y + 0.5f) * cell_size;
            float length = cell_size * 0.6f;            // 箭头长度
            float headLen = cell_size * 0.2f;           // 箭头头部长度

            Vector2 norm = dir.normalize();
            Vector2 start = Vector2(centerX, centerY) - norm * (length * 0.4f); // 起点稍偏后
            Vector2 end = Vector2(centerX, centerY) + norm * (length * 0.4f);

            // 画主线
            SDL_RenderLine(renderer, start.x, start.y, end.x, end.y);

            // 画箭头头部（两个小斜线）
            Vector2 perp = Vector2(-norm.y, norm.x);    // 垂直方向
            Vector2 arrowBase = end - norm * headLen;
            Vector2 left = arrowBase - perp * (headLen * 0.5f);
            Vector2 right = arrowBase + perp * (headLen * 0.5f);
            SDL_RenderLine(renderer, end.x, end.y, left.x, left.y);
            SDL_RenderLine(renderer, end.x, end.y, right.x, right.y);
        }
    }

    // 4. 绘制目标格子（亮白色高亮）
    SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
    SDL_FRect goal_rect = {
        static_cast<float>(goal_pos.x * cell_size),
        static_cast<float>(goal_pos.y * cell_size),
        static_cast<float>(cell_size),
        static_cast<float>(cell_size)
    };
    SDL_RenderFillRect(renderer, &goal_rect);

    // 5. 绘制网格线（半透灰色）
    SDL_SetRenderDrawColor(renderer, 100, 100, 120, 180);
    for (int x = 0; x <= width; ++x) {
        SDL_RenderLine(renderer,
            static_cast<float>(x * cell_size), 0.0f,
            static_cast<float>(x * cell_size), static_cast<float>(tex_h));
    }
    for (int y = 0; y <= height; ++y) {
        SDL_RenderLine(renderer,
            0.0f, static_cast<float>(y * cell_size),
            static_cast<float>(tex_w), static_cast<float>(y * cell_size));
    }

    SDL_SetRenderTarget(renderer, old_target);
}

void Map::on_render(SDL_Renderer* renderer)
{
    SDL_FRect rect;
    rect.x = rect.y = 0;
    rect.w = float(width * cell_size);
    rect.h = float(height * cell_size);
    SDL_RenderTexture(renderer, texture, &rect, &rect);
}

void Map::generate_flow_field()
{
    const float INF = 1e20f;
    const float SQRT2 = 1.41421356237f;

    std::vector<std::vector<float>> dist_field(height, std::vector<float>(width, INF));
    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            if (is_wall(x, y))
                dist_field[y][x] = -1.0f;

    int gx = (int)goal_pos.x, gy = (int)goal_pos.y;
    if (is_wall(gx, gy)) {
        SDL_Log("错误：目标点位于墙上，无法生成流场！");
        return;
    }

    using State = std::pair<float, std::pair<int, int>>;
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;

    dist_field[gy][gx] = 0.0f;
    pq.push({ 0.0f, {gx, gy} });

    const int dx[8] = { 1, -1, 0, 0, 1, 1, -1, -1 };
    const int dy[8] = { 0, 0, 1, -1, 1, -1, 1, -1 };
    const float cost[8] = { 1.0f, 1.0f, 1.0f, 1.0f, SQRT2, SQRT2, SQRT2, SQRT2 };

    while (!pq.empty())
    {
        auto [cur_dist, pos] = pq.top();
        pq.pop();
        int cx = pos.first, cy = pos.second;
        if (cur_dist > dist_field[cy][cx]) continue;

        for (int i = 0; i < 8; ++i)
        {
            int nx = cx + dx[i], ny = cy + dy[i];
            if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
            if (dist_field[ny][nx] < 0.0f) continue; // 障碍物

            float new_dist = cur_dist + cost[i];
            if (new_dist < dist_field[ny][nx])
            {
                dist_field[ny][nx] = new_dist;
                pq.push({ new_dist, {nx, ny} });
            }
        }
    }

    flow_field.resize(height, std::vector<Vector2>(width));
    Vector2 dir_vectors[8];
    for (int i = 0; i < 8; ++i)
    {
        dir_vectors[i] = Vector2((float)dx[i], (float)dy[i]).normalize();
    }

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            if (dist_field[y][x] < 0.0f || dist_field[y][x] >= INF / 2)
            {
                flow_field[y][x] = Vector2(0, 0);
                continue;
            }
            if (dist_field[y][x] == 0.0f)
            {
                flow_field[y][x] = Vector2(0, 0);
                continue;
            }

            float best_dist = dist_field[y][x];
            int best_idx = -1;
            for (int i = 0; i < 8; ++i)
            {
                int nx = x + dx[i], ny = y + dy[i];
                if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue;
                float d = dist_field[ny][nx];
                if (d >= 0 && d < best_dist)
                {
                    best_dist = d;
                    best_idx = i;
                }
            }
            if (best_idx >= 0)
                flow_field[y][x] = dir_vectors[best_idx];
            else
                flow_field[y][x] = Vector2(0, 0);
        }
    }
}

// 传入的是世界坐标（像素），单位矩形宽高（像素）
Vector2 Map::get_flow_direction(const Vector2& world_pos, int unit_w, int unit_h) const
{
    // 单位矩形的世界坐标边界（假设 world_pos 是左上角）
    float left = world_pos.x;
    float right = world_pos.x + unit_w;
    float top = world_pos.y;
    float bottom = world_pos.y + unit_h;

    int gx1 = int(left) / cell_size;
    int gy1 = int(top) / cell_size;
    int gx2 = int(right - 1) / cell_size;
    int gy2 = int(bottom - 1) / cell_size;

    gx1 = std::max(0, gx1);
    gy1 = std::max(0, gy1);
    gx2 = std::min(width - 1, gx2);
    gy2 = std::min(height - 1, gy2);

    Vector2 sum_dir(0, 0);
    float total_area = 0.0f;

    for (int gy = gy1; gy <= gy2; ++gy)
    {
        for (int gx = gx1; gx <= gx2; ++gx)
        {
            float cell_left = gx * cell_size;
            float cell_right = cell_left + cell_size;
            float cell_top = gy * cell_size;
            float cell_bottom = cell_top + cell_size;

            float overlap_left = std::max(left, cell_left);
            float overlap_right = std::min(right, cell_right);
            float overlap_top = std::max(top, cell_top);
            float overlap_bottom = std::min(bottom, cell_bottom);

            float overlap_w = overlap_right - overlap_left;
            float overlap_h = overlap_bottom - overlap_top;
            if (overlap_w <= 0 || overlap_h <= 0)
                continue;

            float area = overlap_w * overlap_h;
            Vector2 dir = flow_field[gy][gx];
            if (dir.x == 0 && dir.y == 0)
                continue;

            sum_dir = sum_dir + dir * area;
            total_area += area;
        }
    }

    if (total_area > 0.0f)
    {
        sum_dir = sum_dir * (1.0f / total_area);
        sum_dir = sum_dir.normalize();
    }
    return sum_dir;
}