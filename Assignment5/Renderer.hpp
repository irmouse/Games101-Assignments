#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;  // 光线方程：orig + t*dir 原点+时间*方向向量
    uint32_t index;
    Vector2f uv;
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};