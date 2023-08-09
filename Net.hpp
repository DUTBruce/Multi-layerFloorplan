//
// Created by 11367 on 2023/6/1.
//

#ifndef SMARTFLOORPLAN_NER_H
#define SMARTFLOORPLAN_NER_H

#include <vector>

class Net
{
public:
    int degree;     //������Ķȣ����Ӷ���ģ�飩
    std::vector<int> connected_blocks; //0��degree-1����ʾ������block_id
    std::vector<int> connected_pins;  //0��degree-1����ʾ��Ӧ��pin_id
    int weight=1;     //������Ȩ�أ������ʱ���Ȩ�أ�Ĭ��Ϊ1������ģ������Ȩ�ػ����1
};


#endif //SMARTFLOORPLAN_NER_H
