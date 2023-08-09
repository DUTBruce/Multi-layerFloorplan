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
    vector<vector<COORD_TYPE>> width, height;   //[i][j]��ʾ���ڵ�i����״j�Ŀ�͸�
    //vector<int> area;   //���ڵ�i��ʱ�����
    bool is_macro;  //��Ϊ��ģ�黹�Ǳ�׼��Ԫ
    int shape_num;         //��״����
    int shape_id;    //��ǰ�ǵڼ�����״

    //�����ϵĽṹ��Ϣ���޸����Ĳ���ͨ���⼸���������
    int left, right, parent;    //�����ӿ�͸����ţ�-1��ʾ��
    bool is_from_left;  //�Ƿ��Ǹ��׵����ӣ�������������
    //bool is_rotated;  ����������ʱֻ�����ְڷţ���������ʱ����

    int rotated_angle;  //˳ʱ�����ת�Ƕȣ���ʼΪ0��
    //Contour contour;    //ÿ���鶼ά��һ����Ŀǰ��ĵȸ��ߣ����ۻ�̫��δʵ��

    int pins_num;   //��ģ���������������������һ���� pins_num = pins_coor[i].size()
    vector<vector<pair<COORD_TYPE,COORD_TYPE>>> pins_coor;    //pins_coor[i][j]��ʾ��i������j��������꣨���ģ�����꣩����� j �� 0 �� pins_num-1
    COORD_TYPE exceed_outline_area;

    Block(int layersize = 1, string s = "null", bool is_macro = true, int shape_num = 1)
    {
        this->layersize = layersize;
        name = s;
        this->is_macro = is_macro;
        this->shape_num = shape_num;

        //��ʼ��block
        layer = 0;
        width.resize(layersize, vector<COORD_TYPE>(shape_num, 0));
        height.resize(layersize, vector<COORD_TYPE>(shape_num, 0));
        pins_coor.resize(layersize);
        rotated_angle = 0;
        shape_id = 0;

        //��ʼ��block�����еĽṹ
        x = y = 0;
        left = right = parent = -1;
        is_from_left = false;
    }
    void add_layer_info(int l, vector<COORD_TYPE> w, vector<COORD_TYPE> h)       //���l��Ŀ��w�͸߶�h��Ϣ
    {
        assert(l < layersize);
        assert(w.size() == h.size() && w.size() == shape_num);
        width[l] = w;
        height[l] = h;
    }
    void add_pins_info(int l, vector<pair<COORD_TYPE,COORD_TYPE>> pins_coor_thislayyer) //���l�������
    {
        pins_coor[l] = pins_coor_thislayyer;
    }
    void rotateOrFlexSize(int code = 0)   //0��ʾ���������ѡ��rotate���ߵ���ģ��ߴ�FlexSize�����ܣ� 1��ʾ����ת��2��ʾֻ����ģ��ߴ�
    {
        if(code == 0)
        {
            int random = rand() % 2;
            if(random == 0 || shape_num == 1)      //��ת
            {
                rotated_angle = (rotated_angle+90) % 360;
            }
            else if(random == 1)    //�����ߴ�
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
        assert(width[l][shape_id] < COORD_TYPE_MAX / height[l][shape_id]);   //��֤��Խ��
        return width[l][shape_id] * height[l][shape_id];
    }
    pair<COORD_TYPE,COORD_TYPE> get_pin_coor(int pin_id)  //����pin_id�����꣬ÿ˳ʱ����ת90�㣬���½��������ʱ��ת�Ƶ���һ������
    {
        if(is_macro)
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
            //��ת֮�� + �Գ�ʼ�������������
            return make_pair(coor.first+modify_offset.first + x, coor.second+modify_offset.second + y);
        }
        else    //����ģ�飬����������Ϊ��������
        {
            assert(pin_id == 0);    //����ģ�����ݽṹֻ��һ������
            return make_pair(x + get_width()/2, y + get_height()/2);
        }
    };

};
