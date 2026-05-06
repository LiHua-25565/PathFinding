#include <SDL3/SDL.h>
#include <SDL3_image/SDL_image.h>
#include "map.h"
#include "unit.h"
#include <thread>
#include <chrono>
#include <vector>
#include <random>

std::vector<Unit> unit_list;
Map map(64, 36);

void on_update(float delta)
{
    static float accumulator = 0.0f;
    accumulator += delta;
    const float SPAWN_INTERVAL = 1.5f;

    if (accumulator >= SPAWN_INTERVAL)
    {
        accumulator -= SPAWN_INTERVAL;

        const int box_left = 8;
        const int box_right = 28;
        const int box_top = 10;
        const int box_bottom = 24;

        // ========== 框内部区域 ==========
        int x_min = box_left + 1;
        int x_max = box_right - 1;
        int y_min = box_top + 2;
        int y_max = box_bottom - 1;

        if (x_min > x_max || y_min > y_max) return;

        int width = x_max - x_min + 1;
        int height = y_max - y_min + 1;

        int left1_x = x_min, right1_x = x_min + width / 3;
        int left1_y = y_min, right1_y = y_min + height / 3;

        int left2_x = x_min, right2_x = x_min + width / 3;
        int left2_y = y_max - height / 3, right2_y = y_max;

        int left3_x = x_min + width / 3, right3_x = x_max - width / 3;
        int left3_y = y_min + height / 3, right3_y = y_max - height / 3;

        if (right1_x < left1_x) right1_x = left1_x;
        if (right1_y < left1_y) right1_y = left1_y;
        if (right2_x < left2_x) right2_x = left2_x;
        if (right2_y < left2_y) right2_y = left2_y;
        if (right3_x < left3_x) right3_x = left3_x;
        if (right3_y < left3_y) right3_y = left3_y;

        // ========== 框外左侧区域 ==========
        int outside_left_start = 0;
        int outside_left_end = box_left - 1;   // 左侧外部 x 范围
        int outside_top_start = 0;
        int outside_top_end = box_top - 1;     // 上部外部 y 范围
        int outside_bottom_start = box_bottom + 1;
        int outside_bottom_end = map.get_height() - 1;  // 下部外部 y 范围

        // 有效性检查（如果范围无效则跳过该区域的生成）
        bool generate_top = (outside_left_start <= outside_left_end && outside_top_start <= outside_top_end);
        bool generate_bottom = (outside_left_start <= outside_left_end && outside_bottom_start <= outside_bottom_end);

        static std::mt19937 rng(std::random_device{}());

        auto get_random_empty_cell = [&](int lx, int ly, int rx, int ry) -> Vector2 {
            for (int attempt = 0; attempt < 20; ++attempt) {
                int rand_x = std::uniform_int_distribution<int>(lx, rx)(rng);
                int rand_y = std::uniform_int_distribution<int>(ly, ry)(rng);
                if (!map.is_wall(rand_x, rand_y))
                    return Vector2(static_cast<float>(rand_x), static_cast<float>(rand_y));
            }
            return Vector2(static_cast<float>((lx + rx) / 2), static_cast<float>((ly + ry) / 2));
            };

        // 框内三个区域生成
        Vector2 pos1_cell = get_random_empty_cell(left1_x, left1_y, right1_x, right1_y);
        Vector2 pos2_cell = get_random_empty_cell(left2_x, left2_y, right2_x, right2_y);
        Vector2 pos3_cell = get_random_empty_cell(left3_x, left3_y, right3_x, right3_y);

        // 框外左上（如果有效）
        Vector2 pos4_cell;
        if (generate_top) {
            pos4_cell = get_random_empty_cell(outside_left_start, outside_top_start,
                outside_left_end, outside_top_end);
        }

        // 框外左下（如果有效）
        Vector2 pos5_cell;
        if (generate_bottom) {
            pos5_cell = get_random_empty_cell(outside_left_start, outside_bottom_start,
                outside_left_end, outside_bottom_end);
        }

        int cs = map.get_cell_size();
        unit_list.emplace_back(Vector2(pos1_cell.x * cs, pos1_cell.y * cs));
        unit_list.emplace_back(Vector2(pos2_cell.x * cs, pos2_cell.y * cs));
        unit_list.emplace_back(Vector2(pos3_cell.x * cs, pos3_cell.y * cs));
        if (generate_top)
        {
            unit_list.emplace_back(Vector2(pos4_cell.x * cs, pos4_cell.y * cs));
            unit_list.back().set_speed(150.0f);
        }
        if (generate_bottom)
        {
            unit_list.emplace_back(Vector2(pos5_cell.x * cs, pos5_cell.y * cs));
            unit_list.back().set_speed(150.0f);
        }
    }
}

int main(int argc, char* argv[])
{
    using namespace std::chrono;
    SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);

    SDL_Window* window = SDL_CreateWindow("PathFinding", 1280, 720, 0);
    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);

    SDL_SetRenderLogicalPresentation(renderer, 1280, 720, SDL_LOGICAL_PRESENTATION_LETTERBOX);

    SDL_Event event;
    bool is_fullscreen = false;
    bool is_quit = false;

    const nanoseconds frame_duration(1000000000 / 144);
    steady_clock::time_point last_tick = steady_clock::now();

    map.generate_flow_field();  // 生成流场
    map.bake_texture(renderer);

    while (!is_quit)
    {
        while (SDL_PollEvent(&event))
        {
            switch (event.type)
            {
            case SDL_EVENT_QUIT:
                is_quit = true;
                break;
            case SDL_EVENT_KEY_DOWN:
                if (event.key.key == SDLK_F11)
                {
                    is_fullscreen = !is_fullscreen;
                    SDL_SetWindowFullscreen(window, is_fullscreen);
                }
                if (event.key.key == SDLK_ESCAPE)
                {
                    is_fullscreen = false;
                    SDL_SetWindowFullscreen(window, is_fullscreen);
                }
                break;
            }
            SDL_ConvertEventToRenderCoordinates(renderer, &event);
        }

        steady_clock::time_point frame_start = steady_clock::now();
        float delta = duration<float>(frame_start - last_tick).count();

        // 更新单位
        for (auto& unit : unit_list)
        {
            unit.on_update(delta, map, unit_list);
        }
        on_update(delta);

        // 渲染
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        map.on_render(renderer);
        int cell_size = map.get_cell_size();

        for (auto& unit : unit_list)
        {
            unit.on_render(renderer,cell_size);
        }

        SDL_RenderPresent(renderer);

        last_tick = frame_start;
        nanoseconds sleep_duration = frame_duration - (steady_clock::now() - frame_start);
        if (sleep_duration > nanoseconds(0))
            std::this_thread::sleep_for(sleep_duration);
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}