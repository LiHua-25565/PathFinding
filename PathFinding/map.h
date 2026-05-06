#ifndef _MAP_H_
#define _MAP_H_

#include <vector>
#include "vector2.h"
#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>

class Map
{
public:
    Map(int width = 1280 / 20, int height = 720 / 20);
    ~Map();

    void bake_texture(SDL_Renderer* renderer);
    void on_render(SDL_Renderer* renderer);
    bool is_wall(int x, int y) const;
    void generate_flow_field();
    Vector2 get_goal() const { return goal_pos; }  // 格子坐标
    Vector2 get_flow_direction(const Vector2& world_pos, int unit_w, int unit_h) const;
    int get_width() const { return width; }
    int get_height() const { return height; }

    int get_cell_size() const { return cell_size; }

private:
    SDL_Texture* texture = nullptr;

    int width = 0;
    int height = 0;
    std::vector<std::vector<bool>> map_grid;      // 地图网格
    std::vector<std::vector<Vector2>> flow_field; // 地图寻路向量场

    int cell_size = 20;
    Vector2 goal_pos = { 50, 18 };
};


#endif // !_MAP_H_
