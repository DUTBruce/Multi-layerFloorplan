//
// Created by 11367 on 2023/5/30.
//

#include "Config.hpp"

class Block
{
public:
    string name;
    int layersize;  //共几层
    int layer;  //表示目前处于第几层，0 到 layersize-1
    COORD_TYPE x, y;   //坐标（相对于根节点(0,0)）的偏移量
    vector<COORD_TYPE> width, height;   //放于第i层时的宽和高
    //vector<int> area;   //放于第i层时的面积

    //在树上的结构信息，修改树的操作通过这几个变量完成
    int left, right, parent;    //左右子块和父块编号，-1表示空
    bool is_from_left;  //是否是父亲的左孩子，用来计算坐标
    //bool is_rotated;  不考虑引脚时只有两种摆放，考虑引脚时废弃

    int rotated_angle;  //顺时针的旋转角度，初始为0度
    //Contour contour;    //每个块都维护一个到目前块的等高线，代价会太大，未实现

    int pins_num;   //该模块的引脚数，各层引脚数一样， pins_num = pins_coor[i].size()
    vector<vector<pair<COORD_TYPE,COORD_TYPE>>> pins_coor;    //pins_coor[i][j]表示第i层引脚j的相对坐标（相对模块坐标），编号 j 从 0 到 pins_num-1

    Block(int layersize = 1, string s = "null")
    {
        this->layersize = layersize;
        layer = 0;
        width.resize(layersize, 0);
        height.resize(layersize, 0);
        pins_coor.resize(layersize);
        name = s;
        x = y = 0;
        left = right = parent = -1;
        rotated_angle = 0;
        is_from_left = false;
    }
    void add_layer_info(int l, COORD_TYPE w, COORD_TYPE h)       //添加l层的宽度w和高度h信息
    {
        assert(l < layersize && w > 0 && h > 0);
        width[l] = w;
        height[l] = h;
    }
    void add_pins_info(int l, vector<pair<COORD_TYPE,COORD_TYPE>> pins_coor_thislayyer) //添加l层的引脚
    {
        pins_coor[l] = pins_coor_thislayyer;
    }
    void rotate()
    {
        rotated_angle = (rotated_angle+90) % 360;
    }
    COORD_TYPE get_width()
    {
        return rotated_angle%180==0? width[layer]: height[layer];
    }
    COORD_TYPE get_height()
    {
        return rotated_angle%180==0? height[layer]: width[layer];
    }
    COORD_TYPE area()
    {
        assert(width[layer] < COORD_TYPE_MAX / height[layer]);   //保证不越界
        return width[layer] * height[layer];
    }
    pair<COORD_TYPE,COORD_TYPE> get_pin_coor(int pin_id)  //引脚pin_id的坐标，每顺时针旋转90°，坐下角坐标就逆时针转移到下一个顶点
    {
        pair<COORD_TYPE,COORD_TYPE> coor; //相对于初始点的坐标
        pair<COORD_TYPE,COORD_TYPE> modify_offset; //修正偏移量，旋转后左下坐标会逆时针转移到下一个顶点
        COORD_TYPE x0 = pins_coor[layer][pin_id].first;
        COORD_TYPE y0 = pins_coor[layer][pin_id].second;
        switch (rotated_angle) {
            case 0:
                coor.first = x0;
                coor.second = y0;
                modify_offset = {0,0};
                break;
            case 90:
                coor.first = y0;
                coor.second = -x0;
                modify_offset = make_pair(0,width[layer]);
                break;
            case 180:
                coor.first = -x0;
                coor.second = -y0;
                modify_offset = make_pair(width[layer],height[layer]);
                break;
            case 270:
                coor.first = -y0;
                coor.second = x0;
                modify_offset = make_pair(height[layer],0);
                break;
            default:
                std::cout << "Invalid rotation degree." << std::endl;
                break;
        }
        //旋转之后 + 对初始点坐标进行修正
        return make_pair(coor.first+modify_offset.first + x, coor.second+modify_offset.second + y);
    };
};
