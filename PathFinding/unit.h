#ifndef _UNIT_H_
#define _UNIT_H_

#include "vector2.h"
#include "map.h"
#include <SDL3/SDL.h>

class Unit
{
public:
    Unit(const Vector2& world_pos)
        : position(world_pos), velocity(0, 0) {
    }

    Vector2 get_position() const { return position; }
    void set_position(const Vector2& pos) { position = pos; }

    Vector2 get_velocity() const { return velocity; }
    void set_velocity(const Vector2& vel) { velocity = vel; }

    void on_render(SDL_Renderer* renderer, int cell_size);

    void on_update(float delta, const Map& map, const std::vector<Unit>& all_units);

    void set_speed(float s) { speed = s; }
    bool is_arrived() const { return arrived; }
    Vector2 get_parked_cell() const { return parkedCell; }

    float get_speed() const { return speed; }

private:
    Vector2 position;   // 世界坐标（像素）
    Vector2 velocity;   // 像素/秒
    static constexpr int WIDTH = 30;
    static constexpr int HEIGHT = 30;
    float speed = 100.0f;   // 像素/秒
    bool arrived = false;                     // 是否已完成停靠
    Vector2 parkedCell = Vector2(-1, -1);     // 停靠的格子坐标（格子单位）

    Vector2 find_parking_spot(const Map& map, const std::vector<Unit>& all_units) const;
    Vector2 compute_wall_repulsion(const Vector2& pos, const Map& map) const;
    Vector2 compute_separation(const Vector2& pos, const Map& map, const std::vector<Unit>& all_units) const;
};

#endif // !_UNIT_H_

