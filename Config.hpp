
#ifndef CONFIG_HPP
#define CONFIG_HPP
#include <iostream>
#include <random>
#include <ctime>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>

#include <unordered_set>
#include <cassert>
#include <stack>
#include <signal.h>
#include <unistd.h>
#include<iostream>
#include<fstream>
#include<vector>
#include <set>
#include<list>
#include<stack>
#include<unordered_map>
#include<cstdlib> //rand()函数头文件
#include<ctime> //clock()函数头文件
#include<assert.h>
#include<cmath>
#include<string>

#define FixedOutline true       //是否为固定轮廓
#define IfUtilizationLimit true //是否有利用率限制
#define ShrinkNet //是否为缩核算例
#define DeleteBlocksNotInNets
#define COORD_TYPE long long int    //坐标类型
#define COORD_TYPE_MAX LONG_LONG_MAX

using namespace std;
class Config
{
public:
    int layyer_max = 2;     //共有几层Floorplan
    double alpha = 0;       //代价函数的面积系数（面积/线长所占比重），0-1
    double gamma = 1;       //轮廓约束系数，任意指定值，没有固定轮廓约束时gamma=0
    string path; //= "C:\\Users\\11367\\Desktop\\algorithm\\floorplanning_iccad2023_btree\\Deploy\\Instance\\";
    string instance; //= "ICCAD2023_floorplanning_case2";
    string block_file, net_file, output_file;
    unsigned int random_seed; //std::random_device{}(); // 随机的随机数种子，初始化std::mt19937 rng(cfg.random_seed)，使用用rng();
    clock_t start_ms = clock();
    float time_limit;
    Config(){
        path = "stdio"; //控制台标准输入输出
        random_seed = 0;    //std::random_device{}()
        time_limit = 300;
    }
    Config(string path, string instance, unsigned int random_seed, float time_limit):
            random_seed(random_seed), path(path), instance(instance), time_limit(time_limit){
        block_file = path + instance + ".blocks";
        net_file = path + instance + ".nets";
        output_file = path + instance + "_seed=" + to_string(random_seed) + ".output";
    }
    Config(string blocks_file, string nets_file, string output_file, unsigned int random_seed, float time_limit):
            block_file(blocks_file), net_file(nets_file), output_file(output_file), random_seed(random_seed), time_limit(time_limit){
    }
};
#endif