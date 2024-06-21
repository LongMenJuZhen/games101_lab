// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //顺时针构建边向量
    Eigen::Vector2f sideAtoB = {_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y()};
    Eigen::Vector2f sideBtoC = {_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y()};
    Eigen::Vector2f sideCtoA = {_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y()};
    //构建点和顶点的向量
    Eigen::Vector2f AtoPoint = {x - _v[0].x(), y - _v[0].y()};
    Eigen::Vector2f BtoPoint = {x - _v[1].x(), y - _v[1].y()};
    Eigen::Vector2f CtoPoint = {x - _v[2].x(), y - _v[2].y()};
    //计算三个顶点对应的二者叉乘
    float crossA = sideAtoB.x() * AtoPoint.y() - sideAtoB.y() * AtoPoint.x();
    float crossB = sideBtoC.x() * BtoPoint.y() - sideBtoC.y() * BtoPoint.x();
    float crossC = sideCtoA.x() * CtoPoint.y() - sideCtoA.y() * CtoPoint.x();
    //判断三个向量的叉乘结果的符号相同，据此判断点是否在三角形内
    if (crossA > 0 && crossB > 0 && crossC > 0)
    {
        return true;
    }
    else if (crossA < 0 && crossB < 0 && crossC < 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
static bool insideTriangle_f(float x, float y, const Vector3f* _v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    //顺时针构建边向量
    Eigen::Vector2f sideAtoB = {_v[1].x() - _v[0].x(), _v[1].y() - _v[0].y()};
    Eigen::Vector2f sideBtoC = {_v[2].x() - _v[1].x(), _v[2].y() - _v[1].y()};
    Eigen::Vector2f sideCtoA = {_v[0].x() - _v[2].x(), _v[0].y() - _v[2].y()};
    //构建点和顶点的向量
    Eigen::Vector2f AtoPoint = {x - _v[0].x(), y - _v[0].y()};
    Eigen::Vector2f BtoPoint = {x - _v[1].x(), y - _v[1].y()};
    Eigen::Vector2f CtoPoint = {x - _v[2].x(), y - _v[2].y()};
    //计算三个顶点对应的二者叉乘
    float crossA = sideAtoB.x() * AtoPoint.y() - sideAtoB.y() * AtoPoint.x();
    float crossB = sideBtoC.x() * BtoPoint.y() - sideBtoC.y() * BtoPoint.x();
    float crossC = sideCtoA.x() * CtoPoint.y() - sideCtoA.y() * CtoPoint.x();
    //判断三个向量的叉乘结果的符号相同，据此判断点是否在三角形内
    if (crossA > 0 && crossB > 0 && crossC > 0)
    {
        return true;
    }
    else if (crossA < 0 && crossB < 0 && crossC < 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}
static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // TODO : Find out the bounding box of current triangle.
    //确定包围盒
    int x_min = std::min(std::min(v[0].x(), v[1].x()), v[2].x());
    int x_max = std::max(std::max(v[0].x(), v[1].x()), v[2].x());
    int y_min = std::min(std::min(v[0].y(), v[1].y()), v[2].y());
    int y_max = std::max(std::max(v[0].y(), v[1].y()), v[2].y());
    // iterate through the pixel and find if the current pixel is inside the triangle
    // // 遍历包围盒里的每个像素(非提高做法)
    // for (int x = x_min; x <= x_max; x++){
    //     for (int y = y_min; y <= y_max; y++){
    //         //判断是否在三角形里
    //         if (insideTriangle(x, y, t.v)){
    //             //看看是不是在最前面的
    //             // If so, use the following code to get the interpolated z value.
    //             auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //             float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //             float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //             z_interpolated *= w_reciprocal;
    //             int index = get_index(x, y);
    //             //没有c++基础，这个藏在头文件的depth_buf给我整不会了
    //             if (z_interpolated < depth_buf[index]){
    //                 //刷新深度
    //                 depth_buf[index] = z_interpolated;
    //                 // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
    //                 //设置像素颜色
    //                 set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
    //             }
    //         }
    //     }
    // }
    // //遍历包围盒里的每个像素(提高做法)
    for (int x = x_min; x <= x_max; x++){
        for (int y = y_min; y <= y_max; y++){
            //四个采样点
            float x1 = x + 0.25;
            float y1 = y + 0.25;
            float x2 = x - 0.25;
            float y2 = y - 0.25;
            float x3 = x + 0.25;
            float y3 = y - 0.25;
            float x4 = x - 0.25;
            float y4 = y + 0.25;
            std::vector<float> x_list = {x1, x2, x3, x4};
            std::vector<float> y_list = {y1, y2, y3, y4};

            int count = 0;
            for (int i = 0; i < 4; i++){
                if (insideTriangle_f(x_list[i], y_list[i], t.v)){
                    count += 1;
                }
            }

            int index = get_index(x, y);
            

            if (count>0){
                count_buf[index] = true;
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (z_interpolated < depth_buf[index]){

                    depth_buf[index] = z_interpolated;
                    
                    int count_down = get_index(x, y-1);
                    int count_up = get_index(x, y+1);
                    int count_left = get_index(x-1, y);
                    int count_right = get_index(x+1, y);
                    
                    if (count_buf[count_down] && count_buf[count_up] && count_buf[count_left] && count_buf[count_right]){
                        set_pixel(Eigen::Vector3f(x, y, 1), t.getColor());
                    }
                    else{
                        
                        set_pixel(Eigen::Vector3f(x, y, 1), t.getColor()*count/4);
                    }
                }
            }
            
        }
    }
}
        
        
            
            
    

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    //加一个count_buf
    count_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on