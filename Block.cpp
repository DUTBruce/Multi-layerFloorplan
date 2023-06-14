//
// Created by 11367 on 2023/5/30.
//

#include "Config.hpp"

class Block
{
public:
    string name;
    int layersize;  //������
    int layer;  //��ʾĿǰ���ڵڼ��㣬0 �� layersize-1
    COORD_TYPE x, y;   //���꣨����ڸ��ڵ�(0,0)����ƫ����
    vector<COORD_TYPE> width, height;   //���ڵ�i��ʱ�Ŀ�͸�
    //vector<int> area;   //���ڵ�i��ʱ�����

    //�����ϵĽṹ��Ϣ���޸����Ĳ���ͨ���⼸���������
    int left, right, parent;    //�����ӿ�͸����ţ�-1��ʾ��
    bool is_from_left;  //�Ƿ��Ǹ��׵����ӣ�������������
    //bool is_rotated;  ����������ʱֻ�����ְڷţ���������ʱ����

    int rotated_angle;  //˳ʱ�����ת�Ƕȣ���ʼΪ0��
    //Contour contour;    //ÿ���鶼ά��һ����Ŀǰ��ĵȸ��ߣ����ۻ�̫��δʵ��

    int pins_num;   //��ģ���������������������һ���� pins_num = pins_coor[i].size()
    vector<vector<pair<COORD_TYPE,COORD_TYPE>>> pins_coor;    //pins_coor[i][j]��ʾ��i������j��������꣨���ģ�����꣩����� j �� 0 �� pins_num-1

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
    void add_layer_info(int l, COORD_TYPE w, COORD_TYPE h)       //���l��Ŀ��w�͸߶�h��Ϣ
    {
        assert(l < layersize && w > 0 && h > 0);
        width[l] = w;
        height[l] = h;
    }
    void add_pins_info(int l, vector<pair<COORD_TYPE,COORD_TYPE>> pins_coor_thislayyer) //���l�������
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
        assert(width[layer] < COORD_TYPE_MAX / height[layer]);   //��֤��Խ��
        return width[layer] * height[layer];
    }
    pair<COORD_TYPE,COORD_TYPE> get_pin_coor(int pin_id)  //����pin_id�����꣬ÿ˳ʱ����ת90�㣬���½��������ʱ��ת�Ƶ���һ������
    {
        pair<COORD_TYPE,COORD_TYPE> coor; //����ڳ�ʼ�������
        pair<COORD_TYPE,COORD_TYPE> modify_offset; //����ƫ��������ת�������������ʱ��ת�Ƶ���һ������
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
        //��ת֮�� + �Գ�ʼ�������������
        return make_pair(coor.first+modify_offset.first + x, coor.second+modify_offset.second + y);
    };
};
