#include "unit.h"
#include <cmath>
#include <random>

// 射线与 AABB 矩形的交点距离（t 值，从射线起点开始）
// 射线起点 origin，方向 dir（需归一化），矩形为 world 坐标轴对齐矩形
// 返回值：如果相交，返回最小正 t 值；否则返回 -1
static float ray_intersect_rect(const Vector2& origin, const Vector2& dir,
    float left, float top, float right, float bottom)
{
    // 计算 t 区间
    float t1 = (left - origin.x) / dir.x;
    float t2 = (right - origin.x) / dir.x;
    if (dir.x == 0) { t1 = -1e30f; t2 = 1e30f; }
    float tmin = std::min(t1, t2);
    float tmax = std::max(t1, t2);

    float t3 = (top - origin.y) / dir.y;
    float t4 = (bottom - origin.y) / dir.y;
    if (dir.y == 0) { t3 = -1e30f; t4 = 1e30f; }
    tmin = std::max(tmin, std::min(t3, t4));
    tmax = std::min(tmax, std::max(t3, t4));

    if (tmax >= tmin && tmax > 0)
        return (tmin > 0) ? tmin : tmax;   // 取最近的正交点
    return -1.0f;
}

void Unit::on_render(SDL_Renderer* renderer, int cell_size)
{
    float px = position.x;
    float py = position.y;
    float w = static_cast<float>(WIDTH);
    float h = static_cast<float>(HEIGHT);
    SDL_FRect outer = { px, py, w, h };
    SDL_SetRenderDrawColor(renderer, 200, 0, 0, 255);
    SDL_RenderFillRect(renderer, &outer);

    float centerX = px + w * 0.5f;
    float centerY = py + h * 0.5f;

    Vector2 dir = velocity.normalize();
    float offsetX = dir.x * (w * 0.25f);
    float offsetY = dir.y * (h * 0.25f);

    float rectW = w * 0.5f;
    float rectH = h * 0.25f;
    SDL_FRect inner = { centerX + offsetX - rectW * 0.5f,
                        centerY + offsetY - rectH * 0.5f,
                        rectW, rectH };
    SDL_SetRenderDrawColor(renderer, 255, 200, 100, 255);
    SDL_RenderFillRect(renderer, &inner);
}

void Unit::on_update(float delta, const Map& map, const std::vector<Unit>& all_units)
{
    // 如果已经停靠完毕，不再移动
    if (arrived) return;

    Vector2 worldPos = get_position();
    Vector2 gridPos = worldPos / map.get_cell_size();
    Vector2 goalGrid = map.get_goal();                 // 格子坐标目标点
    float dist_to_goal = (gridPos - goalGrid).length();

    // 阈值：进入终点区域（距离 < 1.5 格）时开始寻找停靠点
    const float ARRIVAL_THRESHOLD = 10.0f;
    if (dist_to_goal < ARRIVAL_THRESHOLD) {
        // 首次进入时分配停靠格子
        if (parkedCell.x < 0) {
            parkedCell = find_parking_spot(map, all_units);
        }
        // 计算停靠点的世界坐标（格子中心）
        Vector2 targetWorld = (parkedCell + Vector2(0.5f, 0.5f)) * map.get_cell_size();
        // 调整为单位矩形的左上角坐标
        targetWorld = targetWorld - Vector2(WIDTH * 0.5f, HEIGHT * 0.5f);
        Vector2 to_target = targetWorld - worldPos;
        float dist_to_park = to_target.length();

        if (dist_to_park < 2.0f) {
            // 到达停靠点，停稳
            set_position(targetWorld);
            set_velocity(Vector2(0, 0));
            arrived = true;
        }
        else {
            // 向停靠点移动
            Vector2 dir = to_target.normalize();
            set_velocity(dir * speed);
            worldPos += get_velocity() * delta;
            set_position(worldPos);
        }
        return;
    }

    // ========== 未到达终点区域，正常寻路 ==========
    // 1. 流场主方向
    Vector2 flow_dir = map.get_flow_direction(worldPos, WIDTH, HEIGHT);
    // 近距离直接指向目标点（世界坐标）
    Vector2 goalWorld = (goalGrid + Vector2(0.5f, 0.5f)) * map.get_cell_size();
    if (dist_to_goal < 2.0f) {
        flow_dir = (goalWorld - worldPos).normalize();
    }

    // 2. 墙体斥力
    Vector2 wall_repulsion = compute_wall_repulsion(worldPos, map);
    // 3. 单位间分离力
    Vector2 separation = compute_separation(worldPos, map, all_units);

    // 合力
    Vector2 combined = flow_dir + wall_repulsion + separation;
    if (combined.length() > 0.01f)
        combined = combined.normalize();
    else
        combined = flow_dir;   // 保底

    set_velocity(combined * speed);
    worldPos += get_velocity() * delta;
    set_position(worldPos);
}

Vector2 Unit::find_parking_spot(const Map& map, const std::vector<Unit>& all_units) const
{
    int cell_size = map.get_cell_size();
    Vector2 goalGrid = map.get_goal();                // 目标格子坐标
    int gx = (int)goalGrid.x, gy = (int)goalGrid.y;

    // 搜索半径：从 1 格逐步扩大（避免过大范围）
    for (int r = 1; r <= 3; ++r) {
        for (int dy = -r; dy <= r; ++dy) {
            for (int dx = -r; dx <= r; ++dx) {
                int nx = gx + dx, ny = gy + dy;
                // 跳过墙壁
                if (map.is_wall(nx, ny)) continue;
                // 跳过超出地图边界
                if (nx < 0 || ny < 0 || nx >= map.get_width() || ny >= map.get_height()) continue;

                // 检查是否有其他已到达的单位占用了这个格子
                bool occupied = false;
                for (const auto& other : all_units) {
                    if (&other == this) continue;
                    if (other.is_arrived()) {
                        Vector2 oc = other.get_parked_cell();
                        if ((int)oc.x == nx && (int)oc.y == ny) {
                            occupied = true;
                            break;
                        }
                    }
                }
                if (!occupied) {
                    return Vector2((float)nx, (float)ny);
                }
            }
        }
    }
    // 如果周围都被占满，就使用目标点本身（允许轻度重叠）
    return goalGrid;
}

Vector2 Unit::compute_wall_repulsion(const Vector2& pos, const Map& map) const
{
    Vector2 repulsion(0, 0);

    // 参数配置
    const float RAY_LENGTH = 2.5f;          // 射线长度（格子数）
    const float ANGLE_OFFSET = 30.0f;       // 左右偏转角度（度）
    const float REPULSION_STRENGTH = 1.0f;  // 斥力强度（与距离平方成反比的系数）
     
    int cell_size = map.get_cell_size();
    Vector2 center = pos + Vector2(WIDTH * 0.5f, HEIGHT * 0.5f);   // 单位中心点

    // 速度方向（若无速度，默认朝X正方向）
    Vector2 forward = velocity;
    if (forward.length() < 0.01f) forward = Vector2(1, 0);
    forward = forward.normalize();

    // 构建三条射线方向
    float rad = ANGLE_OFFSET * 3.14159265f / 180.0f;
    Vector2 dirs[3] = {
        forward,
        Vector2(forward.x * cos(rad) - forward.y * sin(rad), forward.x * sin(rad) + forward.y * cos(rad)), // 左偏
        Vector2(forward.x * cos(-rad) - forward.y * sin(-rad), forward.x * sin(-rad) + forward.y * cos(-rad)) // 右偏
    };

    // 检测每条射线
    for (int i = 0; i < 3; ++i) {
        Vector2 dir = dirs[i];
        float closest_t = RAY_LENGTH * cell_size;   // 最大距离（像素）
        bool hit = false;
        
        // 确定需要检查的格子范围（射线覆盖的矩形区域）
        Vector2 end = center + dir * (RAY_LENGTH * cell_size);
        float left = std::min(center.x, end.x);
        float right = std::max(center.x, end.x);
        float top = std::min(center.y, end.y);
        float bottom = std::max(center.y, end.y);

        int minx = std::max(0, (int)(left / cell_size) - 1);
        int maxx = std::min(map.get_width() - 1, (int)(right / cell_size) + 1);
        int miny = std::max(0, (int)(top / cell_size) - 1);
        int maxy = std::min(map.get_height() - 1, (int)(bottom / cell_size) + 1);

        for (int gy = miny; gy <= maxy; ++gy) {
            for (int gx = minx; gx <= maxx; ++gx) { 
                if (!map.is_wall(gx, gy)) continue;
                // 墙体世界矩形
                float left = gx * cell_size;
                float right = left + cell_size;
                float top = gy * cell_size;
                float bottom = top + cell_size;
                float t = ray_intersect_rect(center, dir, left, top, right, bottom);
                if (t > 0 && t < closest_t) {
                    closest_t = t;
                    hit = true;
                }
            }
        }

        if (hit) {
            float distance_grid = closest_t / cell_size;
            float force = REPULSION_STRENGTH / (distance_grid * distance_grid);
            // 斥力方向：垂直于射线方向（选择固定侧：根据偏转方向决定）
            // 这里使用：左射线产生向右的力，右射线产生向左的力，中射线产生侧向随机？为了稳定，中射线可产生两个侧向的合成或忽略。
            // 简单方式：计算从交点指向单位中心的向量（背离墙体）
            Vector2 point = center + dir * closest_t;
            Vector2 away = (center - point).normalize();
            repulsion = repulsion + away * force;
        }
    }

    return repulsion;
}

Vector2 Unit::compute_separation(const Vector2& pos, const Map& map, const std::vector<Unit>& all_units) const
{
    Vector2 separation(0, 0);
    const float SEPARATION_RADIUS = 1.5f;        // 碰撞半径（格子单位）
    const float BASE_REPULSION = 5.0f;           // 基础斥力强度

    int cell_size = map.get_cell_size();
    float myPriority = get_speed();               // 速度越快优先级越高

    for (const auto& other : all_units) {
        if (&other == this) continue;

        float otherPriority = other.get_speed();
        Vector2 otherPos = other.get_position();
        float dx = pos.x - otherPos.x;
        float dy = pos.y - otherPos.y;
        float dist_px = sqrt(dx * dx + dy * dy);
        float dist_grid = dist_px / cell_size;
        if (dist_grid > SEPARATION_RADIUS) continue;
        if (dist_grid < 0.01f) dist_grid = 0.01f;

        // 优先级因子：自己优先级越高，受到的斥力越小；若自己优先级远高于对方，甚至可以产生“推挤”力（负斥力）
        float factor = 1.0f;
        if (myPriority > otherPriority) {
            // 我优先级高，斥力减弱；甚至可以变成吸引（推挤）
            float ratio = otherPriority / myPriority;  // 0~1
            factor = 0.2f * ratio;   // 最小0，最大0.2
            // 可选：推挤效果：若我优先级高很多且距离很近，产生向前的力（推动对方）
            if (dist_grid < 0.8f && myPriority > otherPriority * 1.2f) {
                // 推挤：力的方向是从我指向对方（即把对方往前推），但这里只影响自己，所以表现为自己继续向前，不产生斥力。
                // 为了简化，依然让factor很小，不额外加力。
            }
        }
        else {
            // 对方优先级高，我需要让路，斥力增强
            float ratio = myPriority / otherPriority;  // 0~1
            factor = 1.0f + (1.0f - ratio) * 2.0f;    // 最大3倍
            factor = std::min(factor, 3.0f);
        }

        float force_mag = BASE_REPULSION / (dist_grid * dist_grid) * factor;
        Vector2 direction = Vector2(dx, dy).normalize(); // 远离对方的方向
        separation = separation + direction * force_mag;
    }
    return separation;
}