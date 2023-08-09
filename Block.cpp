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
    vector<vector<COORD_TYPE>> width, height;   //[i][j]表示放于第i层形状j的宽和高
    //vector<int> area;   //放于第i层时的面积
    bool is_macro;  //是为宏模块还是标准单元
    int shape_num;         //形状数量
    int shape_id;    //当前是第几个形状

    //在树上的结构信息，修改树的操作通过这几个变量完成
    int left, right, parent;    //左右子块和父块编号，-1表示空
    bool is_from_left;  //是否是父亲的左孩子，用来计算坐标
    //bool is_rotated;  不考虑引脚时只有两种摆放，考虑引脚时废弃

    int rotated_angle;  //顺时针的旋转角度，初始为0度
    //Contour contour;    //每个块都维护一个到目前块的等高线，代价会太大，未实现

    int pins_num;   //该模块的引脚数，各层引脚数一样， pins_num = pins_coor[i].size()
    vector<vector<pair<COORD_TYPE,COORD_TYPE>>> pins_coor;    //pins_coor[i][j]表示第i层引脚j的相对坐标（相对模块坐标），编号 j 从 0 到 pins_num-1
    COORD_TYPE exceed_outline_area;

    Block(int layersize = 1, string s = "null", bool is_macro = true, int shape_num = 1)
    {
        this->layersize = layersize;
        name = s;
        this->is_macro = is_macro;
        this->shape_num = shape_num;

        //初始化block
        layer = 0;
        width.resize(layersize, vector<COORD_TYPE>(shape_num, 0));
        height.resize(layersize, vector<COORD_TYPE>(shape_num, 0));
        pins_coor.resize(layersize);
        rotated_angle = 0;
        shape_id = 0;

        //初始化block在树中的结构
        x = y = 0;
        left = right = parent = -1;
        is_from_left = false;
    }
    void add_layer_info(int l, vector<COORD_TYPE> w, vector<COORD_TYPE> h)       //添加l层的宽度w和高度h信息
    {
        assert(l < layersize);
        assert(w.size() == h.size() && w.size() == shape_num);
        width[l] = w;
        height[l] = h;
    }
    void add_pins_info(int l, vector<pair<COORD_TYPE,COORD_TYPE>> pins_coor_thislayyer) //添加l层的引脚
    {
        pins_coor[l] = pins_coor_thislayyer;
    }
    void rotateOrFlexSize(int code = 0)   //0表示正常情况，选择rotate或者调整模块尺寸FlexSize都可能， 1表示仅旋转，2表示只调整模块尺寸
    {
        if(code == 0)
        {
            int random = rand() % 2;
            if(random == 0 || shape_num == 1)      //旋转
            {
                rotated_angle = (rotated_angle+90) % 360;
            }
            else if(random == 1)    //调整尺寸
            {
                shape_id = (shape_id + 1) % shape_num;
            }
        }
        else if(code == 1)
        {
            rotated_angle = (rotated_angle+90) % 360;
        }
        else
        {
            shape_id = (shape_id + 1) % shape_num;
        }
    }
    const COORD_TYPE get_width()
    {
        return rotated_angle%180==0? width[layer][shape_id]: height[layer][shape_id];
    }
    const COORD_TYPE get_height()
    {
        return rotated_angle%180==0? height[layer][shape_id]: width[layer][shape_id];
    }
    const COORD_TYPE area()
    {
        return area(layer);
    }
    const COORD_TYPE area(int l)
    {
        assert(width[l][shape_id] < COORD_TYPE_MAX / height[l][shape_id]);   //保证不越界
        return width[l][shape_id] * height[l][shape_id];
    }
    pair<COORD_TYPE,COORD_TYPE> get_pin_coor(int pin_id)  //引脚pin_id的坐标，每顺时针旋转90°，坐下角坐标就逆时针转移到下一个顶点
    {
        if(is_macro)
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
                    modify_offset = make_pair(0,width[layer][shape_id]);
                    break;
                case 180:
                    coor.first = -x0;
                    coor.second = -y0;
                    modify_offset = make_pair(width[layer][shape_id],height[layer][shape_id]);
                    break;
                case 270:
                    coor.first = -y0;
                    coor.second = x0;
                    modify_offset = make_pair(height[layer][shape_id],0);
                    break;
                default:
                    std::cout << "Invalid rotation degree." << std::endl;
                    break;
            }
            //旋转之后 + 对初始点坐标进行修正
            return make_pair(coor.first+modify_offset.first + x, coor.second+modify_offset.second + y);
        }
        else    //缩核模块，中心坐标视为引脚坐标
        {
            assert(pin_id == 0);    //缩核模块数据结构只有一个引脚
            return make_pair(x + get_width()/2, y + get_height()/2);
        }
    };

};
