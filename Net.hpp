//
// Created by 11367 on 2023/6/1.
//

#ifndef SMARTFLOORPLAN_NER_H
#define SMARTFLOORPLAN_NER_H

#include <vector>

class Net
{
public:
    int degree;     //该网表的度（连接多少模块）
    std::vector<int> connected_blocks; //0到degree-1，表示相连的block_id
    std::vector<int> connected_pins;  //0到degree-1，表示对应的pin_id
};


#endif //SMARTFLOORPLAN_NER_H
