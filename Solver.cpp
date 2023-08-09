

#include "Net.hpp"
#include "Config.hpp"
#include "BStarTree.cpp"

//#define SUBMIT
#ifndef INT_MAX
#define INT_MAX 2147483647
#endif

class Solver    //即Floorplan算法框架
{
public:
    //数据结构
    //vector<Block> blocks_cur, blocks_pre, blocks_best;  //当前操作的模块，保存的上一次的模块（作为恢复的副本），以及最好解的模块
    BStarTree cur_tree, pre_tree, best_tree;    //每个tree内部包括layer_size层，block数量总和等于b_num
    vector<Block> initial_blocks;
    vector<Net> nets;
    //vector<int> blocks_tree_number;     //blocks_cur[i]所在的树，可以直接存在block中，已用blocks_cur[block_id].layer代替
    int layer_size;    //几层的floorplanning
    unordered_map<string, int> blocks_name_id;   //块的名字到编号（0到b_num-1）的映射
    Config _cfg;
    int b_num;          //总共的block数量
    unordered_set<int> nets_include_blocks;  //所有网表包含的模块数量，便于删除不在网表中的模块

    //约束信息
    COORD_TYPE outline_width = 23000, outline_height = 19000;    //固定轮廓的宽和高，在读取时赋值，在初始化树时存入树中
    vector<double> userate_max;     //各层的最大利用率限制

    //参数信息
    double alpha, gamma;  //代价函数的面积系数（面积/线长所占比重），轮廓约束系数
    double initial_average_area, initial_average_wirelength;   //初始化时确定的平均面积和线长，便于计算多目标Cost() = area/avg + wirelength/avg;
    double initial_average_exceed_outline_area; //初始化时平均的超出轮廓代价，等于超出的轮廓面积（宏观）+超出轮廓的模块面积（细化）
    double initial_wirelength, initial_time, initial_best_wirelength;
    int initial_iter;
    double cost_pre, cost_cur, best_cost;
    double dead_sapce_rate;
    /*算法评测信息*/
    int initial_num_perturbs;
    vector<COORD_TYPE> total_block_area;    //模块都放在i层的面积
    vector<int> blocks_bigest_layer;        //模块面积最大的那一层
    vector<int> layer_bigest_blocks_num;    //第i层面积最大的模块数量
    long long int iter=0, best_iter=0, adjustLegalOutline_fail_times=0;
    int reject_time=0;
    int accept_inferior_time=0;
    int update_best_time=0;
    time_t start_ms, end_ms;
    time_t outline_adjust_ms = 0;

    Solver(Config &cfg)
    {
        _cfg = cfg;
        _cfg.start_ms = start_ms = clock();
        //start_ms = _cfg.start_ms;
        layer_size = _cfg.layyer_max;
        alpha = _cfg.alpha;
        gamma = _cfg.gamma;
        //best_tree.resize(layer_size);
    }
    bool run()
    {
        srand(_cfg.random_seed);
        return solve(_cfg.block_file, _cfg.net_file, _cfg.output_file);
    }
    bool solve(string blockpath, string netpath, string output_file)
    {
        if(!Initialization(blockpath, netpath))
            return false;
        //OutputBlocks(std::cout, best_tree.blocks);
        //OutputTrees(std::cout,best_tree);
        cout<< "1, time cost: "<< (double)(clock()-start_ms) / CLOCKS_PER_SEC << endl;

        cout<<"search running..."<<endl;
        LocalSearch(best_tree, b_num*300, _cfg.time_limit);
        //OutputBlocks(std::cout, best_tree.blocks);
        //OutputTrees(std::cout,best_tree);
        cout<< "2, time cost: "<< (double)(clock()-start_ms) / CLOCKS_PER_SEC << endl;
        AdjustLegalOutline(best_tree, b_num*30, b_num*b_num);
        cout<< "3 over, time cost: "<< (double)(clock()-start_ms) / CLOCKS_PER_SEC << endl;


//        SA();
//        cout << "SA is over" << endl;
        end_ms = clock();
//        OutputBlocks(std::cout, blocks_cur);`
//        OutputTrees(std::cout,cur_tree);
//        OutputBlocks(std::cout, best_tree.blocks);
//        OutputTrees(std::cout,best_tree);
        Output(output_file);
        return true;
    }

    void SA()
    {
        /*cur_tree.Initialization();
        cur_tree.Pack();
        best_tree = pre_tree = cur_tree;
        best_cost = cost_pre = Cost();*/
        /*double initial_p = 0.99;
        double t = ( - best_cost / log(initial_p) );
        cout<<"t: "<<t<<endl<<endl;
        double t_min = t / 1e3;*/
        best_tree = pre_tree = cur_tree;
        double t = 1e3;
        double t_min = 1e-4;
        double lamda = 0.85;         //退火系数，迭代 100 * num_iterations 次
        int num_iterations = b_num * b_num; //b_num * b_num * 30;   //每个温度下的迭代次数
        double deta_cost;
        iter = best_iter = 0;
        reject_time = 0;
        accept_inferior_time = 0;
        update_best_time=0;
        while(t > t_min)
        {

            for(int i=0; i<num_iterations && (double)(clock()-start_ms)/CLOCKS_PER_SEC < _cfg.time_limit; i++)
            {
                cur_tree.Perturb();
                cur_tree.Pack();
                //OutputBlocks(cout, cur_tree.blocks);
                cost_cur = Cost(cur_tree);
                deta_cost = cost_cur - cost_pre ;
                //cout<<"deta_cost: "<<deta_cost<<endl;
                //cout<<"cost_pre: "<<cost_pre<<", best_cost: "<<best_cost<<endl;
                if(deta_cost <= 0)               //为什么是<=0，和<0有何区别：因为=0的时候接受劣解概率公式结果为1，会一直接受新解
                {
                    //cout<<"cost_cur: "<<cost_cur<<", best_cost: "<<best_cost<<endl;
                    if(cost_cur < best_cost)    //更新最优解，若为固定轮廓则调整后若满足才更新
                    {
                        if(FixedOutline == true){
                            if(AdjustLegalOutline(cur_tree, b_num * b_num, b_num) == false)
                            {
                                //调整轮廓失败，放弃此次最优解，&并加入禁忌？how？
                                adjustLegalOutline_fail_times ++;
                                continue;
                            }
                            else
                            {
                                cost_cur = Cost(cur_tree);
                                if(cost_cur >= best_cost)
                                    continue;
                            }
                        }
                        best_cost = cost_cur;
                        best_tree = cur_tree;
                        //cout<<"best_cost: "<<best_cost<<endl;
                        best_iter = iter;
                        update_best_time++;
                    }
                    cost_pre = cost_cur;
                    pre_tree = cur_tree;
                }
                else if(exp( - deta_cost / t) > rand()%10000/10000.0)   //满足模拟退火接受劣解概率
                {
                    cost_pre = cost_cur;
                    pre_tree = cur_tree;
                    accept_inferior_time++;
                    /*if(iter % 100 == 0)
                    {
                        cout<<"accept, "<<"deta_cost: "<<deta_cost<<" t: "<<t<<" exp( - deta_cost / t): "<<exp( - deta_cost / t)<<endl;
                    }*/
                }
                else
                {
                    cur_tree = pre_tree;            //不接受新解，从 S(i+1) 回滚到 S(i)
                    reject_time++;
                    //cout<<"reject in iter: "<<iter<<endl;
                }
                iter += i;
            }
            t = t * lamda;
        }
        //dead_sapce_rate = (float)best_tree.Area() / total_block_area - 1;
    }
    void LocalSearch(BStarTree& _tree, int max_search_fail_tolerability, float time_limt)   //搜索到局部最优，只接受让代价更小的解
    {
        bool is_debug = false;
        //time_t ad_start_ms = clock();
        BStarTree pre_tree;  //记录tree信息便于调整和回退
        pre_tree = _tree;
        double best_cost = Cost(_tree);
        double cur_cost;
        int cur_search_fails = 0;   //连续扰动失败次数（等于max_search_fail_tolerability表示已收敛）
        while(cur_search_fails < max_search_fail_tolerability && (double)(clock()-start_ms)/CLOCKS_PER_SEC < time_limt)
        {
            _tree.Perturb();
            _tree.Pack();
            cur_cost = Cost(_tree);
            if(cur_cost < best_cost)
            {
                //后续,在这里加个解禁忌，避免进入同样的不合法最优解
                if(AdjustLegalOutline(_tree, b_num*30, b_num*b_num))
                {
                    cur_cost = Cost(_tree); //调整后的cost
                    if(cur_cost < best_cost)
                    {
                        best_cost = cur_cost;
                        pre_tree = _tree;
                        cur_search_fails = 0;
                        update_best_time++;
                    }
                    if(is_debug)
                        cout << "best cost update: " << best_cost << endl;
                }
                else
                {
                    cout << "LocalSearch AdjustLegalOutline() failed" << endl;
                }
            }
            else    //没搜到更好解，回退，记录失败次数
            {
                _tree = pre_tree;
                cur_search_fails++;
            }
        }
    }
//    void ConvSA()
//    {
//        double conv_rate = 1;    //定义收敛率，拒绝率>=收敛率时搜索完毕
//        double initial_p = 0.999;
//        double total_deta_cost=0, average_deta_cost;
//        for(int i=0; i<cur_tree.tree_b_num * cur_tree.tree_b_num; i++)
//        {
//            cur_tree.PerturbAndPack();
//            cur_tree.Pack();
//            cost_cur = Cost();
//            total_deta_cost += abs(cost_cur - cost_pre);
//            cost_pre = cost_cur;
//        }
//        average_deta_cost = total_deta_cost / (cur_tree.tree_b_num * cur_tree.tree_b_num);
//        double t = ( - average_deta_cost / log(initial_p) );//初始温度
//        cout<<"average_deta_cost: "<<average_deta_cost<<endl;
//        cout<<"initial t: "<<t<<endl<<endl;
//        //double t = 1e3;
//        //double t_min = 1e-3;
//        double lamda = 0.85;         //退火系数
//        int num_iterations = 30 * cur_tree.tree_b_num * cur_tree.tree_b_num;   //每个温度下的迭代次数
//        vector<bool> recent_isrejected; //数组实现循环链表的功能
//        recent_isrejected.resize(num_iterations, 0);
//        //cout<<"recent_isrejected.size(): "<<recent_isrejected.size()<<endl;
//        double deta_cost;
//        iter = best_iter = 0;
//        reject_time = 0;
//        accept_inferior_time = 0;
//        update_best_time=0;
//        best_tree = cur_tree;
//        best_cost = cost_pre = cost_cur = Cost();
//        double recent_reject_time = 0;  //最近劣解的拒绝次数
//        double reject_rate = recent_reject_time / recent_isrejected.size();
//        int iter_reject = 0;          //拒绝循环列表的当前位置
//        while(reject_rate < conv_rate)
//        {
//            for(int i=0; i<num_iterations; i++)
//            {
//                cur_tree.PerturbAndPack();
//                cur_tree.Pack();
//                cost_cur = Cost();
//                deta_cost = cost_cur - cost_pre;
//                bool isrejected = 0;        //该次解是否拒绝
//                if(deta_cost <= 0)
//                {
//                    if(deta_cost == 0)              //deta_cost==0也归入拒绝解，不然拒绝率无法达到收敛率
//                        isrejected = 1;
//                    else if(cost_cur < best_cost)    //更新最优解
//                    {
//                        best_cost = cost_cur;
//                        best_tree = cur_tree;
//                        //cout<<"best_cost: "<<best_cost<<endl;
//                        best_iter = i;
//                        update_best_time++;
//                    }
//                    cost_pre = cost_cur;
//                    pre_tree = cur_tree;
//                }
//                else if(exp( - deta_cost / t) > rand()%10000/10000.0)   //满足模拟退火接受劣解概率
//                {
//                    cost_pre = cost_cur;
//                    pre_tree = cur_tree;
//                    accept_inferior_time++;
//                }
//                else            //不接受新解，从 S(i+1) 回滚到 S(i)，并记录
//                {
//                    cur_tree = pre_tree;
//                    reject_time++;
//                    isrejected = 1;             //记录拒绝信息
//                }
//                recent_reject_time += isrejected - recent_isrejected[iter_reject];
//                recent_isrejected[iter_reject] = isrejected;
//                iter_reject = (iter_reject + 1) % recent_isrejected.size();
//                /*if(isrejected)
//                    cout<<"iter_reject: "<<iter_reject<<" recent_reject_time: "<<recent_reject_time<<" iter:"<<iter<<"accept_inferior_time: "<<accept_inferior_time<<endl;
//                */
//            }
//            t = t * lamda;
//            iter += num_iterations;
//            reject_rate = recent_reject_time / recent_isrejected.size();
//            /*if(reject_rate > 0.90)
//                cout<<"iter: "<<iter<<" reject_rate: "<<reject_rate<<endl;*/
//        }
//        /*for(int i=0; i<num_iterations; i++)
//        {
//            cout<<"recent_isrejected "<<i<<": "<<recent_isrejected[i]<<endl;
//        }*/
//        cout<<endl;
//        cout<<"final t: "<<t<<endl;
//        dead_sapce_rate = (float)best_tree.Area() / total_block_area - 1;
//
//    }
//    void PerturbAndPack(vector<BStarTree> &tree, vector<Block> &blocks) //o(h) 在解层面对所有树实行扰动，对相应影响的树进行布局Pack（实际操作实现的在模块层面进行扰动）
//    //6.15新增，Perturb后自动pack扰动的树
//    //6.20新增已解决问题: 移动模块时可能将树移为空，树为空时不会再移入模块。已解决：增加把模块移到对应树根节点之前的操作，也就是新增to的选择
//    {
//        assert(tree[0].blocks == &blocks);
//        bool is_debug = false;
//        if(is_debug)
//            cout << "begin perturb" << endl;
//
//        /*在树层面进行扰动
//        int random = rand() % (2*layer_size - 1);  //扰动的动作，从0到layyersize-1对应一棵树的扰动，其余动作对应树间扰动
//        //1. 动作i在0-layyersize-1中，扰动树i
//        if(random < layer_size || layer_size == 1)
//        {
//            cur_tree[layer_size].PerturbAndPack();
//        }
//        //2. 剩余2*layer_size - 1种情况，进行树间扰动：移动模块从一个树到另一个树。2层的时候为1/3概率
//        else
//        {
//
//        }*/
//
//        //模块层面进行扰动
//        int perturb_move_size = 3;  //扰动操作的数量
//        int random = rand() % perturb_move_size;
//        switch(random)
//        {
//            case 0: //旋转模块 o(1)
//            {
//                int block_id = rand()%b_num;
//                if(is_debug)
//                    cout<<"rotateOrFlexSize "<<block_id<<endl;
//                RotateBlock(tree, blocks, block_id);
//                break;
//            }
//            case 1: //移动模块（删除并插入） o(h)
//            {
//                int from = rand()%b_num;
//                int to = rand() % (b_num + layer_size);      //模块移动的去向：除了1.所有模块后面，还有2.每层布局的根节点之前（成为新的根结点）
//                while(to == from)
//                    to = rand()%b_num;
//                if(is_debug)
//                    cout<<"move "<<from<<" to "<<to<<endl;
//                MoveBlocks(tree, blocks, from, to);
//                break;
//            }
//            case 2: //交换模块（两边同时删除和插入） o(1)
//            {
//                int id_1 = rand()%b_num;
//                int id_2 = rand()%b_num;
//                while(id_2 == id_1)
//                    id_2 = rand()%b_num;
//                if(is_debug)
//                    cout<<"swap "<<id_1<<" and "<<id_2<<endl;
//                SwapBlocks(tree, blocks, id_1, id_2);
//                break;
//            }
//        }
//        if(is_debug)
//            cout << "complete perturb" << endl;
//    }
//    void RotateBlock(vector<BStarTree> &tree, vector<Block> &blocks, int block_id)
//    {
//        tree[blocks[block_id].layer].RotateBlock(block_id);
//        tree[blocks[block_id].layer].Pack();
//    }
//    void MoveBlocks(vector<BStarTree> &tree, vector<Block> &blocks, int from, int to)
//    {
//        //cout << "begin delete" << endl;
//        int layer_from = blocks[from].layer;    //预先存一下所在层，不然后续更改后blocks_cur[from].layer会变为to的layer
//        tree[layer_from].DeleteBlock(from);
//        int layer_to;
//        if(to < b_num)  //1.正常移动到模块to后面
//        {
//            layer_to = blocks[to].layer;
//            //cout << "begin insert" << endl;
//            tree[layer_to].InsertBlock(from, to);
//        }
//        else    //2. 模块移动到布局 to - b_num (0到layyer_size-1) 的根节点之前
//        {
//            layer_to = to - b_num;
//            tree[layer_to].InsertRoot(from);
//        }
//        tree[layer_from].Pack();
//        if(layer_from != layer_to)
//            tree[layer_to].Pack();
//    }
//    void SwapBlocks(vector<BStarTree> &tree, vector<Block> &blocks, int id_1, int id_2)  //o(1)
//    {
//        bool is_debug = false;
//        if(id_1 == 3 && id_2 == 2)
//        {
//            //is_debug = true;
//        }
//        Block& block_1 = blocks[id_1];      //记得引用！不然是临时变量，修改完了没用！
//        Block& block_2 = blocks[id_2];
//        int root_1 = tree[block_1.layer].root;
//        int root_2 = tree[block_2.layer].root;
//        swap(block_1.left, block_2.left);
//        swap(block_1.right, block_2.right);
//        swap(block_1.parent, block_2.parent);
//        swap(block_1.is_from_left, block_2.is_from_left);
//        swap(block_1.layer, block_2.layer);
//        //特殊情况处理
//        //1. 两节点相邻时，更正错误信息
//        if(block_1.left == id_1)
//        {
//            block_1.left = id_2;
//            block_2.parent = id_1;
//        }
//        else if(block_1.right == id_1)
//        {
//            block_1.right = id_2;
//            block_2.parent = id_1;
//        }
//        else if(block_1.parent == id_1)
//        {
//            block_1.parent = id_2;
//            if(block_1.is_from_left)
//                block_2.left = id_1;
//            else
//                block_2.right = id_1;
//        }
//        //2. 更新父节点和子节点指向信息
//        if(block_1.parent != -1)
//        {
//            if(block_1.is_from_left)
//                blocks[block_1.parent].left = id_1;
//            else
//                blocks[block_1.parent].right = id_1;
//        }
//        if(block_1.left != -1)
//            blocks[block_1.left].parent = id_1;
//        if(block_1.right != -1)
//            blocks[block_1.right].parent = id_1;
//        if(block_2.parent != -1)
//        {
//            if(block_2.is_from_left)
//                blocks[block_2.parent].left = id_2;
//            else
//                blocks[block_2.parent].right = id_2;
//        }
//        if(block_2.left != -1)
//            blocks[block_2.left].parent = id_2;
//        if(block_2.right != -1)
//            blocks[block_2.right].parent = id_2;
//        //3. 如果为根节点则更新相应tree的root
//        if(is_debug) {
//            cout<< "root_1: " << root_1 << ", root_2: " << root_2 << endl;
//            cout << "id_1: " << id_1 << ", cur_tree[block_1.layer].root:" << cur_tree[block_1.layer].root << endl;
//            cout << "id_2: " << id_2 << ", cur_tree[block_2.layer].root:" << cur_tree[block_2.layer].root << endl;
//        }
//        //bug在于下面这俩if判断会相互抵消，又回到了原来，所以需要提前存下root或者用else分类讨论
//        if(id_1 == root_1)
//            tree[block_2.layer].root = id_2;
//        if(id_2 == root_2)
//            tree[block_1.layer].root = id_1;
//        tree[blocks[id_1].layer].Pack();
//        if(blocks[id_1].layer != blocks[id_2].layer)
//            tree[blocks[id_2].layer].Pack();
//
//        if(is_debug) {
//            cout << "id_1: " << id_1 << ", cur_tree[block_1.layer].root:" << cur_tree[block_1.layer].root << endl;
//            cout << "id_2: " << id_2 << ", cur_tree[block_2.layer].root:" << cur_tree[block_2.layer].root << endl;
//            OutputBlocks(cout, cur_tree.blocks);
//            OutputTrees(cout, cur_tree);
//        }
//    }
//    void Pack()
//    {
//        for(int it = 0; it < layer_size; it++)
//            cur_tree[it].Pack();
//    }
    bool Initialization(string blockpath, string netpath)       //初始化树结构和平均线长平均面积等信息
    {
        bool is_debug = false;

        //1. 读取并初始化为layyersize个完全二叉树
        //blocks_cur.resize(_cfg.layyer_max, Block()); 为什么我会这么写，感觉有问题
#ifdef ShrinkNet
        ReadFromShrinkNetFile(blockpath, netpath);
#else
        ReadFromFile(blockpath, netpath);
#endif
        /*cur_tree.b_num = 6;
        cur_tree.blocks_cur = {Block(1,1),Block(2,2),Block(2,1),Block(1,1),Block(1,1),Block(3,2)};
        Net::nets = {{0,1,3}, {2,4,5}, {0,1,2,3,4,5}};*/
        total_block_area.resize(layer_size, 0);
        blocks_bigest_layer.resize(b_num, 0);   //初始设为0层最大，然后后面层对比
        layer_bigest_blocks_num.resize(layer_size, 0);
        for(int ib=0; ib<b_num; ib++)    //记录所有块总面积
        {
            Block &b = initial_blocks[ib];
            for(int l=0; l<layer_size; l++)
                total_block_area[l] += b.area(l);
            for(int l=1; l<layer_size; l++)
            {
                if(b.area(l) > b.area(blocks_bigest_layer[ib]))
                    blocks_bigest_layer[ib] = l;
            }
            layer_bigest_blocks_num[blocks_bigest_layer[ib]] ++;
        }
        for(int l=0; l<layer_size; l++)
            cout << "layer " << l << ", total_block_area:" << total_block_area[l] <<", layer_bigest_blocks_num: " << layer_bigest_blocks_num[l] << endl;
        Initial_blocks();
        if(is_debug)
        {
            cout << "Initial_blocks() is over" << endl;
        }
        Initial_trees();
        //OutputBlocks(std::cout, blocks_cur);  //pack前，坐标还都是(0,0)
        cur_tree.Pack();
        if(is_debug)
        {
            cout << "Initial_trees() is over" << endl;
        }
        if(FixedOutline == true){
            int perturb_times = b_num * b_num;
            int max_adjust_tolerability_iteration = b_num * 30;
            while(AdjustLegalOutline(cur_tree, max_adjust_tolerability_iteration, perturb_times) == false
                  && (double)(clock()-start_ms)/CLOCKS_PER_SEC < _cfg.time_limit ) //不成功则逐渐加大扰动力度
            {
                cout << "Initialtion AdjustLegalOutline() failed, continue trying.." << endl;
                srand(rand());                //cur_tree = _tree;
                perturb_times *= 2;
                for(int i=0; i < perturb_times; i++)
                {
                    cur_tree.Perturb();
                }
                cur_tree.Pack();
            }
            if((double)(clock()-start_ms)/CLOCKS_PER_SEC >= _cfg.time_limit)
                return false;
        }
        //blocks_best = blocks_pre = blocks_cur;  //bug已修正，之前blocks_best未用pack后的blocks_cur赋值
        best_tree = pre_tree = cur_tree;
        if(is_debug)
        {
            cout << "AdjustLegalOutline() is over, time_cost: " <<  double(clock() - start_ms) / CLOCKS_PER_SEC
            <<"s, infomation as follow: " << endl;
            OutputBlocks(std::cout, cur_tree.blocks);  //pack后，坐标在放置的位置上
            OutputTrees(std::cout, cur_tree);
        }


        COORD_TYPE initial_area = TotalArea(cur_tree);
        initial_wirelength = TotalWireLength(cur_tree.blocks);
        COORD_TYPE initial_exceed_outline_area = TotalExceedOutlineArea(cur_tree) == 0 ? 1 : TotalExceedOutlineArea(cur_tree); //防止除数为0
        double initial_best_cost = 1 + gamma; //随机初始化过程的最好代价 alpha * cur_area/initial_area + (1-alpha) *  cur_wirelength/initial_wirelength + gamma * cur_exceed_outline_area/initial_exceed_outline_area
        long long total_area = 0;
        double total_wirelength = 0;
        COORD_TYPE  total_exceed_outline_area = initial_exceed_outline_area;

        initial_num_perturbs = b_num * 30;  //随机初始化的次数（单目标的话选取其中最好的解，多目标时不方便量化，不一定是最好的解）
        COORD_TYPE cur_area;
        double cur_wirelength;
        double cur_cost;
        COORD_TYPE cur_exceed_outline_area;
        for(initial_iter=0; initial_iter < initial_num_perturbs
            && (double)(clock()-start_ms)/CLOCKS_PER_SEC < _cfg.time_limit; initial_iter++)
        {
            cur_tree.Perturb();
            if(is_debug && IfUtilizationLimit == true){   //看perturb之后pack之前的blocks_area面积计算的是否正确，如果正确会和pack后的一致
                OutputTrees(std::cout, cur_tree);
            }
            cur_tree.Pack();
            if(is_debug){
                OutputBlocks(std::cout, cur_tree.blocks);
                OutputTrees(std::cout, cur_tree);
            }
            cur_area = TotalArea(cur_tree);
            cur_wirelength = TotalWireLength(cur_tree.blocks);
            cur_exceed_outline_area = TotalExceedOutlineArea(cur_tree);
            cur_cost = alpha * cur_area/initial_area + (1 - alpha) *  cur_wirelength/initial_wirelength + gamma * cur_exceed_outline_area/initial_exceed_outline_area;
            if(cur_cost < initial_best_cost)
            {
                if(FixedOutline == true && AdjustLegalOutline(cur_tree, b_num * 30, b_num) == false) {
                    cout << "initial_best AdjustLegalOutline failed! " << endl;
                }
                else
                {
                    initial_best_cost = cur_cost;
                    best_tree = pre_tree = cur_tree;
                }
            }
            total_area += cur_area;
            total_wirelength += cur_wirelength;
            total_exceed_outline_area += cur_exceed_outline_area;
        }
//        cout<<"total_area: "<<total_area<<endl;
//        cout<<"total_wirelength: "<<total_wirelength<<endl;
        initial_average_area = (double)total_area / initial_num_perturbs;
        initial_average_wirelength = (double)total_wirelength / initial_num_perturbs;
        initial_average_exceed_outline_area = (double)total_exceed_outline_area / (initial_num_perturbs+1);
        initial_best_wirelength = TotalWireLength(best_tree.blocks);
        cout<<"initial_area: " << initial_area <<endl;
        cout<<"initial_wirelength: " << initial_wirelength <<endl;
        cout<<"initial_exceed_outline_area: " << initial_exceed_outline_area <<endl;
        cout<< "initial_average_area: "<<initial_average_area<<endl;
        cout << "initial_average_wirelength: " << initial_average_wirelength << endl;
        cout << "initial_average_exceed_outline_area: " << initial_average_exceed_outline_area <<endl;
        cout<<"initial best area: "<< TotalArea(best_tree)<<endl;
        cout<<"initial_best_wirelength: "<<initial_best_wirelength<<endl;
        cout<<"last perturb area: " << cur_area<<endl;
        cout<<"last perturb wirelength: " << cur_wirelength<<endl;
        cout<<"initial_best_cost: "<<initial_best_cost<<endl;
        cout<<"initial last cost: " << cur_cost << endl;
        pre_tree = cur_tree = best_tree;
        best_cost = cost_pre = Cost(cur_tree);
        cout<<"now best_cost: " << best_cost <<endl;
        cout<<endl;
        initial_time = double(clock() - start_ms) / CLOCKS_PER_SEC;
        return true;
    }
    void Initial_blocks()
    {
        for(int ib=0; ib < b_num; ib++)
        {
            initial_blocks[ib].parent = initial_blocks[ib].left = initial_blocks[ib].right = -1;
            initial_blocks[ib].is_from_left = false;
            /*旋转模块复原*/
            initial_blocks[ib].rotated_angle = 0;
        }
        //blocks_best = blocks_pre = blocks_cur;
    }
    void Initial_trees()
    {
//        //每个block随机选择放入哪个树（未实现）
//        for(int it=0; it<layer_size; it++)
//        {
//
//        }
//        for(int ib=0; ib < b_num; ib++)
//        {
//            int random_tree = rand() % layer_size;
//            blocks_tree_number[ib] = random_tree;
//            cur_tree[random_tree].
//        }

        //0. 初始化树空间
        //cur_tree.resize(layer_size, BStarTree(blocks_cur));
        //chatgpt告诉我对于具有引用类型成员变量的类，它们不能被默认构造或复制，因此在使用 vector 的 resize() 函数时会导致问题。得按下面的初始化
//        cur_tree.reserve(layer_size);  // 提前分配足够的空间
//        best_tree.reserve(layer_size);
//        pre_tree.reserve(layer_size);
//        for (int i = 0; i < layer_size; i++) {
//            // 逐个初始化BStarTree对象
//            cur_tree.emplace_back(&blocks_cur, i, outline_width, outline_height);
//            best_tree.emplace_back(&blocks_best, i, outline_width, outline_height);
//            pre_tree.emplace_back(&blocks_pre, i, outline_width, outline_height);
//        }
        cur_tree = BStarTree(initial_blocks, layer_size, outline_width, outline_height);
        if(IfUtilizationLimit == true)
        {
            if( !cur_tree.initial_tree_struct_with_useratio(userate_max) )
            {
                cerr << "failed in initial_tree_struct_with_useratio()" << endl;
                exit(3);
            }
//            OutputTrees(cout,cur_tree);
//            cur_tree.Pack();
//            OutputTrees(cout,cur_tree);
//            exit(0);
        }
        else
        {
            cur_tree.initial_tree_struct();
        }

//        //1. 每个树平均放置若干block节点（以完全二叉树按层生成节点）
//        int ave_tree_blocksnum = b_num / layer_size;
//        //1.x 特殊情况
//        if(ave_tree_blocksnum == 0) //树比blocks多，前b_num个树每个放一个
//        {
//            cerr << "wait for coding." << endl;
//        }
//        //1.1 正常情况，前layyer_size-1个树平均放
//        for(int it=0; it < layer_size - 1; it++)
//        {
//            cur_tree[it].initial_blocks(it*ave_tree_blocksnum, it+ave_tree_blocksnum);
//        }
//        //1.2 最后一个树放剩下所有blocks
//        cur_tree[layer_size - 1].initial_blocks((layer_size - 1) * ave_tree_blocksnum, b_num);
//
//        best_tree = pre_tree = cur_tree;

        /*测试初始化*/
        //Pack();
        //OutputBlocks(std::cout, blocks_cur);
        /*2, dfs初始化坐标//和等高线contour
        stack<int> s;
        blocks_cur[root].x = 0;
        blocks_cur[root].y = 0;
        int cur = root;
        //访问root
        //cout<<cur<<": l"<<endl;
        while(blocks_cur[cur].left != -1) //访问左孩子节点，右子树入栈
        {
            if(blocks_cur[cur].right != -1)
                s.push(blocks_cur[cur].right);
            cur = blocks_cur[cur].left;
            //访问左孩子节点;
            //cout<<cur<<": l"<<endl;
        }
        //访问其他结点
        while(!s.empty())
        {
            cur = s.top();
            s.pop();
            //访问右孩子节点
            //cout<<cur<<": r"<<endl;
            while(blocks_cur[cur].left != -1) //访问左孩子节点，右子树入栈
            {
                if(blocks_cur[cur].right != -1)
                    s.push(blocks_cur[cur].right);
                cur = blocks_cur[cur].left;
                //访问左孩子节点;
                //cout<<cur<<": l"<<endl;
            }
        }*/
    }
    static bool MoveFstream_to_FindStr(ifstream& fin, string str)
    {
        string s;
        fin>>s;
        while(s!=str && s!="")
        {
            getline(fin, s);
            fin>>s;
        }
        return s!="";
    }
    //todo 改为从结构体中传数据，数据需要读入initial_blocks和nets中
    void ReadFromShrinkNetFile(string blockpath, string netpath)
    {
        bool is_debug = false;
        //1. 读取blocks信息和block上的引脚信息
        ifstream fin(blockpath);
        if(!fin)
        {
            cout<<blockpath<<" blocksfile open error!"<<endl;
            exit(-1);
        }
        fin>>outline_width>>outline_height;
        if(IfUtilizationLimit == true)
        {
            userate_max.resize(layer_size);
            for(int i=0; i<layer_size; i++){
                fin>>userate_max[i];
                userate_max[i] /= 100;
            }
        }
        fin>>b_num;
        if(is_debug)
            cout<<"b_num: "<<b_num<<", outline_width: "<<outline_width<<", outline_height: "<<outline_height<<endl;
        initial_blocks.clear();
        initial_blocks.resize(b_num);
        //1.1 分layyer_size层读取blocks信息和block上的引脚信息
        string str, str_next, line, block_name, block_type;
        COORD_TYPE shape_num, width, height;
        char dummy; // ignore parameter
        COORD_TYPE dum;
        for(int cur_layyer = 0; cur_layyer < layer_size; cur_layyer++)
        {
            for(int i=0; i<b_num; i++)
            {
                //1.1.1 读取当前的block长宽信息
                Block &cur_block = initial_blocks[i];
                fin>>block_name;
                if(is_debug)
                    cout << "block_name: " << block_name <<endl;
                fin>>block_type;        //Y表示宏模块，形状固定引脚多个；N表示非宏模块（标准单元），形状多个引脚固定（按模块中心估算）
                if( !(block_type=="Y" || block_type=="N"))  //检查block_type读取的正确性
                {
                    cerr << "block_type read error: " << block_type << endl;
                    exit(3);
                }
                bool is_macro;
                if(block_type=="Y") //读取宏模块
                {
                    is_macro = true;
                }
                else //读取非宏模块（标准单元），形状多个引脚固定（按模块中心估算）
                {
                    is_macro = false;
                }
                //读取形状数量、长、宽
                fin >> shape_num;
                assert(shape_num > 0);
                vector<COORD_TYPE> widths(shape_num);
                vector<COORD_TYPE> heights(shape_num);
                for(int i=0; i<shape_num; i++)
                {
                    fin  >> width >> height;
                    assert(width > 0 && height > 0);
                    widths[i] = width;
                    heights[i] = height;
                }
                //判断该模块是第一次读取还是第i次读取第i层的，哦哦可以直接按layyer_max读取
                //第一次读取
                if( cur_layyer==0 )
                {
                    assert(blocks_name_id.count(block_name)==0);
                    cur_block = Block(layer_size, block_name, is_macro ,shape_num);
                    cur_block.add_layer_info(0, widths, heights);
                    blocks_name_id[block_name] = i;
                }
                    //非首次读取
                else
                {
                    assert(blocks_name_id.count(block_name) == cur_layyer);
                    cur_block.add_layer_info(cur_layyer, widths, heights);
                }

                //1.1.2 读取当前block在该层的引脚坐标(x_pin, y_pin)...，共pins_num个引脚
                int pins_num;
                fin >> pins_num;
                if(is_debug)
                    cout << "pins_num:" << pins_num << endl;
                vector<pair<COORD_TYPE,COORD_TYPE>> pins_coor_thislayyer;    //模块在该层的引脚坐标
                COORD_TYPE x_pin, y_pin;
                for(int pi = 0; pi < pins_num; pi++)
                {
                    fin >> dummy >> x_pin >> dummy >> y_pin >> dummy;
                    if(is_debug)
                        cout << "x_pin: " << x_pin <<", y_pin>: " << y_pin << endl;
                    pins_coor_thislayyer.push_back(make_pair(x_pin,y_pin));
                }
                cur_block.add_pins_info(cur_layyer, pins_coor_thislayyer);
                cur_block.pins_num = pins_num;
                assert(pins_num == cur_block.pins_coor[cur_layyer].size());
            }
        }
        assert(initial_blocks.size() == b_num);
        fin.close();
        //2. 读取nets信息
        if(netpath.empty())
            return;
        fin.open(netpath);
        if(!fin)
        {
            cout<<netpath<<" netsfile open error!"<<endl;
            exit(-1);
        }
        int num_nets, net_degree, net_weight;    //net数目， 对应net的block数目、权重weight
        fin>>num_nets;
        if(is_debug)
            cout<<"num_nets: "<<num_nets<<endl;
        nets.clear();
        for(int ni=0; ni<num_nets && !fin.eof(); ni++)   //读取每个net的对应信息
        {
            Net cur_net;        //当前读取的net
            fin>>net_degree>>net_weight;
            cur_net.weight = net_weight;
            if(is_debug)
                cout << "net_degree: " << net_degree << ", net_weight: " << net_weight <<endl;
            int final_net_degree = net_degree;
            for(int i=0; i<net_degree; i++)
            {
                fin>>str;   //网表包含的block名字
                if(str=="#")    //过滤注释信息
                {
                    getline(fin, line);
                    i--;
                    continue;
                }
                // 读取该网表包含的block和引脚
                int block_id = blocks_name_id[str];
#ifdef DeleteBlocksNotInNets
                nets_include_blocks.insert(block_id);   //该网表包含的模块
#endif
                int pin_id;
                fin >> pin_id;
                if(is_debug)
                    cout<<"str: "<<str<<", "<<"block_id: "<<block_id<<", pins_id: "<<pin_id<<endl;
                assert(pin_id >= 0 && pin_id < initial_blocks[block_id].pins_num);
                if(block_id == -1)    //不在考虑范围内的引脚
                {
                    cerr << str << " pin is not in range" << endl;
                    final_net_degree--;
                }
                else            //block[id]上的引脚，记录id到网表中
                {
                    cur_net.connected_blocks.push_back(block_id);
                    cur_net.connected_pins.push_back(pin_id);
                }
                getline(fin, line);
            }
            assert(final_net_degree == cur_net.connected_blocks.size());
            //cout<<"final_block_num: "<<final_block_num<<endl<<endl;
            if(final_net_degree > 1)
            {
                cur_net.degree = final_net_degree;
                nets.push_back(cur_net);
            }
            else
            {
                cerr << "final_block_num <= 1, please check" << endl;
                num_nets--;
            }
        }
        assert(num_nets == nets.size());
        if(is_debug)
            cout<<"final_num_nets: "<<num_nets<<endl<<endl;
        fin.close();
#ifdef DeleteBlocksNotInNets    //删除不在任何网表的模块，！并且改变原网表指向的block（不然指向的是原来的block，会越界）
        bool is_debug_delete = false;
        if(is_debug_delete)
            cout << "InNetsBlockNum: " << nets_include_blocks.size() << ", doing DeleteBlocksNotInNets" <<endl;
        vector<Block> temp_blocks = initial_blocks;
        vector<int> blocks_id_old_to_new(b_num, -1);   //旧block_id到新id的转换，改变后续网表的connected_blocks
        initial_blocks.clear();
        for(int i=0; i < temp_blocks.size(); i++)
        {
            if( nets_include_blocks.count(i) )
            {
                if(is_debug_delete)
                    cout << temp_blocks[i].name << " is in nets" << endl;
                initial_blocks.push_back(temp_blocks[i]);
                blocks_id_old_to_new[i] = initial_blocks.size() - 1;
            }
            else if(is_debug_delete)
                cout << temp_blocks[i].name << " is not in nets" << endl;
        }
        b_num = initial_blocks.size();
        assert(initial_blocks.size() == nets_include_blocks.size());\
        //2. 更正网表
        for(auto &net: nets)//for(auto net: nets) 这样不行，改变的是临时变量net，需要加引用
        {
            for(int i = 0; i < net.connected_blocks.size(); i++)
            {
                int old_block_id = net.connected_blocks[i];
                if( blocks_id_old_to_new[old_block_id] < 0 || blocks_id_old_to_new[old_block_id] > b_num)
                {
                    cerr << "something wrong in DeleteBlocksNotInNets" << endl;
                    exit(3);
                }
                else
                {
                    net.connected_blocks[i] = blocks_id_old_to_new[old_block_id];
                }
            }
        }
        if(is_debug_delete)
            cout << "DeleteBlocksNotInNets is over" << endl;
#endif
    }
    void ReadFromFile(string blockpath, string netpath = "")
    {
        bool is_debug = false;
        //1. 读取blocks信息和block上的引脚信息
        ifstream fin(blockpath);
        if(!fin)
        {
            cout<<blockpath<<" blocksfile open error!"<<endl;
            exit(-1);
        }
        string str, str_next, line, block_name, block_type;
        if(!MoveFstream_to_FindStr(fin,"NumHardRectilinearBlocks"))
        {
            cout<<"Hardblocks read error in file: "<<blockpath<<endl;
        }
        fin>>str;   //过滤 : 分隔符
        fin>>b_num;
        cout<<"b_num: "<<b_num<<endl;
        initial_blocks.clear();
        initial_blocks.resize(b_num);
        //1.1 分layyer_size层读取blocks信息和block上的引脚信息
        for(int cur_layyer = 0; cur_layyer < layer_size; cur_layyer++)
        {
            for(int i=0; i<b_num; i++)
            {
                //1.1.1 读取当前的block长宽信息
                Block &cur_block = initial_blocks[i];
                fin>>block_name;
                if(is_debug)
                    cout << "block_name: " << block_name <<endl;
                fin>>block_type;
                if( !(block_type=="hardrectilinear" || block_type=="softrectangular"))  //检查block_type读取的正确性
                {
                    cerr << "block_type read error: " << block_type << endl;
                }
                COORD_TYPE x0, y0, x1, y1;
                char dummy; // ignore parameter
                COORD_TYPE dum;
                if(block_type=="hardrectilinear") //读取硬模块
                {
                    //读取" 4 (x0, y0) (%d, %d) (x1, y1) (%d, %d)"
                    fin >> dum >> dummy >> x0 >> dummy >> y0 >> dummy >> dummy >> dum >> dummy >> dum >> dummy >> dummy
                        >>  x1 >> dummy >> y1 >> dummy >> dummy >> dum >> dummy >> dum >> dummy;

                    //判断该模块是第一次读取还是第i次读取第i层的，哦哦可以直接按layyer_max读取
                    //第一次读取
                    if( cur_layyer==0 )
                    {
                        assert(blocks_name_id.count(block_name)==0);
                        cur_block = Block(layer_size, block_name);
                        cur_block.add_layer_info(0, {x1 - x0}, {y1 - y0});
                        blocks_name_id[block_name] = i;
                    }
                        //非首次读取
                    else
                    {
                        assert(blocks_name_id.count(block_name) == cur_layyer);
                        cur_block.add_layer_info(cur_layyer, {x1 - x0}, {y1 - y0});
                    }

                }
                else //软模块softrectangular，暂未实现
                {

                }
                //1.1.2 读取当前block在该层的引脚坐标(x_pin, y_pin)...，共pins_num个引脚
                int pins_num;
                fin >> pins_num;
                if(is_debug)
                    cout << "pins_num:" << pins_num << endl;
                vector<pair<COORD_TYPE,COORD_TYPE>> pins_coor_thislayyer;    //模块在该层的引脚坐标
                COORD_TYPE x_pin, y_pin;
                for(int pi = 0; pi < pins_num; pi++)
                {
                    fin >> dummy >> x_pin >> dummy >> y_pin >> dummy;
                    if(is_debug)
                        cout << "x_pin: " << x_pin <<", y_pin>: " << y_pin << endl;
                    pins_coor_thislayyer.push_back(make_pair(x_pin,y_pin));
                }
                cur_block.add_pins_info(cur_layyer, pins_coor_thislayyer);
                cur_block.pins_num = pins_num;
                assert(pins_num == cur_block.pins_coor[cur_layyer].size());
            }
        }
        assert(initial_blocks.size() == b_num);
        fin.close();
        //2. 读取nets信息
        if(netpath.empty())
            return;
        fin.open(netpath);
        if(!fin)
        {
            cout<<netpath<<" netsfile open error!"<<endl;
            exit(-1);
        }
        if(!MoveFstream_to_FindStr(fin,"NumNets"))
        {
            cout<<"NumNets read error in file: "<<blockpath<<endl;
        }
        fin>>str;   //过滤:分隔符
        int num_nets, numpins, net_degree;    //net数目， 所有net的pins数目，对应net的block数目
        fin>>num_nets;
        cout<<"num_nets: "<<num_nets<<endl;
        nets.clear();
        while(MoveFstream_to_FindStr(fin,"NetDegree"))  //读取每个net的对应信息
        {
            Net cur_net;        //当前读取的net
            fin>>str;   //过滤:分隔符
            fin>>net_degree;
            if(is_debug)
                cout<<"net_degree: "<<net_degree<<endl;
            int final_net_degree = net_degree;
            for(int i=0; i<net_degree; i++)
            {
                fin>>str;   //网表包含的block名字
                if(str=="#")    //过滤注释信息
                {
                    getline(fin, line);
                    i--;
                    continue;
                }
                // 读取该网表包含的block和引脚
                int block_id = blocks_name_id[str];
                int pin_id;
                fin >> pin_id;
                if(is_debug)
                    cout<<"str: "<<str<<", "<<"block_id: "<<block_id<<", pins_id: "<<pin_id<<endl;
                assert(pin_id >= 0 && pin_id < initial_blocks[block_id].pins_num);
                if(block_id == -1)    //不在考虑范围内的引脚
                {
                    cerr << str << " pin is not in range" << endl;
                    final_net_degree--;
                }
                else            //block[id]上的引脚，记录id到网表中
                {
                    cur_net.connected_blocks.push_back(block_id);
                    cur_net.connected_pins.push_back(pin_id);
                }
                getline(fin, line);
            }
            assert(final_net_degree == cur_net.connected_blocks.size());
            //cout<<"final_block_num: "<<final_block_num<<endl<<endl;
            if(final_net_degree > 1)
            {
                cur_net.degree = final_net_degree;
                nets.push_back(cur_net);
            }
            else
            {
                cerr << "final_block_num <= 1, please check" << endl;
                num_nets--;
            }
        }
        assert(num_nets == nets.size());
        if(is_debug)
            cout<<"final_num_nets: "<<num_nets<<endl<<endl;
        fin.close();
    }
    void TestBlocks_name_id()
    {
        cout<<"TestBlocks_name_id, input block name: (input 0 end)"<<endl;
        string name;
        cin>>name;
        while(name!="0")
        {
            int id = blocks_name_id[name];
            Block &b = initial_blocks[id];
            cout<<"id:"<<id<<" width:"<<b.get_width()<<" height:"<<b.get_height()<<endl;
            cin>>name;
        }
    }
    COORD_TYPE BlocksArea()
    {
        COORD_TYPE blocks_area = 0;
        for(Block b : initial_blocks)
            blocks_area += b.area();
        return blocks_area;
    }
    void OutputBlocks(ostream &out, vector<Block> &b)
    {
        if(!out)
        {
            cerr << "OutputBlocks error" << endl;
            return;
        }
        out<<"name,"
            <<"x,"
            <<"y,"
            <<"layer,"
            <<"rotated_angle,"
           <<"width,"
           <<"height,"
           <<"area,"
           << "exceed_outline_area,"
           <<"left,"
           <<"right,"
           <<"parent,"
           <<"is_from_left,"
           <<endl;
        for(int i=0; i<b_num; i++)
            out << b[i].name << ","
                << b[i].x << ","
                << b[i].y << ","
                << b[i].layer << ","
                << b[i].rotated_angle << ","
                << b[i].get_width() << ","
                << b[i].get_height() << ","
                << b[i].area() << ","
                << b[i].exceed_outline_area << ","
                << b[i].left << ","
                << b[i].right << ","
                << b[i].parent << ","
                << b[i].is_from_left << ","
               <<endl;
        out << "Wirelength: " << TotalWireLength(b) << endl;
    }
    void OutputTrees(ostream &out, BStarTree &t)
    {
        t.OutputLayerInfo(out);
    }
//    double FillingRate(vector<BStarTree>& _tree)
//    {
//
//    }
    bool AdjustLegalOutline(BStarTree& _tree, int max_adjust_tolerability_iteration, int perturb_times)   //perturb后pack的时候有bug，可能是浅复制的原因，已重构代码
    //调整布图结构，向着超出轮廓面积（代价）更小的解移动，直到 1.成功，代价为0 或 2.失败，连续扰动最大容忍次数也没得到更好解
    {
        bool is_debug = false;
        time_t ad_start_ms = clock();
        BStarTree best_tree, cur_tree;  //记录tree信息便于调整和回退
        best_tree = cur_tree = _tree;
        COORD_TYPE best_exceed_outline_area =  TotalExceedOutlineArea(_tree);
        while(best_exceed_outline_area > 0)
        {
            if(is_debug)
                cout<<endl<<"AdjustLegalOutline Perturb and Pack"<<endl;
            if(is_debug)
                cout << "best_exceed_outline_area: " << best_exceed_outline_area << endl;
            cur_tree.Perturb();
            cur_tree.Pack();
            COORD_TYPE cur_exceed_outline_area = TotalExceedOutlineArea(cur_tree);
            if(is_debug)
                cout << "cur_exceed_outline_area: " << cur_exceed_outline_area << endl;
            //扰动，遇到代价更小的解接收，否则回退，继续探索解空间直到达到最大容忍次数
            int i = 0;
            while(cur_exceed_outline_area >= best_exceed_outline_area && i < max_adjust_tolerability_iteration)
            {
                if((double)(clock()-start_ms)/CLOCKS_PER_SEC > _cfg.time_limit)
                {
                    cerr << "over time_limit, AdjustLegalOutline() failed!" << endl;
                    return false;
                }
                cur_tree = best_tree;
                cur_tree.Perturb();
                cur_tree.Pack();
                cur_exceed_outline_area = TotalExceedOutlineArea(cur_tree);
                if(is_debug)
                    cout << "cur_exceed_outline_area: " << cur_exceed_outline_area
                    << ", best_exceed_outline_area: " << best_exceed_outline_area << endl
                    << (cur_exceed_outline_area >= best_exceed_outline_area) <<(i < max_adjust_tolerability_iteration) << endl;
                i++;
            }
            //接受更好解，直到代价降为0
            if(i != max_adjust_tolerability_iteration)
            {
                //cout<< "i: " << i << ", max_adjust_tolerability_iteration: " << max_adjust_tolerability_iteration << endl;
                best_tree = cur_tree;
                best_exceed_outline_area = cur_exceed_outline_area;
                //cout <<  "best_exceed_outline_area: " << best_exceed_outline_area << endl;
            }
            else    //局部收敛仍未满足轮廓约束，则1.逐渐加大扰动力度直到获得合法解或超时 2.返回false，在上层选择是否继续操作，此处选择2
            {
                outline_adjust_ms += (clock() - ad_start_ms);
                return false;
//                cout << "AdjustLegalOutline() failed, continue trying.." << endl;
//                srand(rand());                //cur_tree = _tree;
//                perturb_times *= 2;
//                //max_adjust_tolerability_iteration *= 2;   7.29取消最大调整次数的增大，不然每次掉进局部搜索时间可能翻倍
//                for(int i=0; i < perturb_times; i++)
//                {
//                    cur_tree.Perturb();     //！连续perturb时层内模块面积blocks_area没调整导致利用率出现错误。
//                }
//                cur_tree.Pack();
//                best_exceed_outline_area =  TotalExceedOutlineArea(cur_tree);
            }
        }
        //调整成功
        _tree = best_tree;
        outline_adjust_ms += (clock() - ad_start_ms);
        return true;
    }
    const int UnLegalLayer(BStarTree& _tree)
    {
        for(int it = 0; it < layer_size; it++)
        {
            if(_tree.ExceedOutlineArea(it) > 0)
                return it;
        }
        return -1;
    }
    COORD_TYPE TotalExceedOutlineArea(BStarTree& _tree)
    {
        COORD_TYPE total_exceedoutline_area = 0;
        for(int layer = 0; layer < layer_size; layer++)
            total_exceedoutline_area += _tree.ExceedOutlineArea(layer);
        return total_exceedoutline_area;
    }
    COORD_TYPE TotalArea(BStarTree& _tree)
    {
        COORD_TYPE total_area = 0;
        for(int layer = 0; layer < layer_size; layer++)
            total_area += _tree.Area(layer);
        return total_area;
    }
    /*double TotalWireLength(vector<Block>& _blocks)  //半周长 max|xi - xj| + max|yi - yj|   o(net)
    {
        原半周长
        double sum_half_perimeter = 0;
        for(auto net: nets)
        {
            double x_max = INT_MIN, x_min = INT_MAX;
            double y_max = INT_MIN, y_min = INT_MAX;
            for(int i=0; i<net.degree; i++)     //该网表的所有block
            {
                int block_id = net.connected_blocks[i];
                int pin_id = net.connected_pins[i];
                pair<COORD_TYPE, COORD_TYPE> pin_coor = blocks_cur[block_id].get_pin_coor(pin_id);   //获得引脚坐标
                COORD_TYPE pin_x = pin_coor.first;
                COORD_TYPE pin_y = pin_coor.second;

                int layyer = blocks_cur[block_id].layer;      //当前block在第几层（0层或1层）

                if(pin_x < x_min)
                    x_min = pin_x;
                if(pin_x > x_max)
                    x_max = pin_x;
                if(pin_y < y_min)
                    y_min = pin_y;
                if(pin_y > y_max)
                    y_max = pin_y;
            }
            sum_half_perimeter += (x_max - x_min + y_max - y_min);
        }
        return sum_half_perimeter;
    }*/
    double TotalWireLength(vector<Block>& _blocks)  //半周长 max|xi - xj| + max|yi - yj|   o(net)
    {
        bool is_debug = false;
        double sum_half_perimeter = 0;
        if(is_debug)
            cout << "wirelength step 0: "<< endl;
        for(auto net: nets)
        {
            if(is_debug)
                cout << "wirelength step 1: "<< endl;
            double x_max[2] = {INT_MIN, INT_MIN}, x_min[2] = {INT_MAX, INT_MAX};
            double y_max[2] = {INT_MIN, INT_MIN}, y_min[2] = {INT_MAX, INT_MAX};
            for(int i=0; i<net.degree; i++)     //该网表的所有block
            {
                int block_id = net.connected_blocks[i];
                int pin_id = net.connected_pins[i];
                if(is_debug){
                    cout << "wirelength step 1.1: "<< endl;
                    cout << "block_id: " << block_id << ", block_name: " << _blocks[net.connected_blocks[i]].name
                        << "， pin_id: " << pin_id << endl;
                }
                pair<COORD_TYPE, COORD_TYPE> pin_coor = _blocks[block_id].get_pin_coor(pin_id);   //获得引脚坐标
                COORD_TYPE pin_x = pin_coor.first;
                COORD_TYPE pin_y = pin_coor.second;
                if(is_debug)
                    cout << "pin " << i << ". x: " << pin_x << ", y: " << pin_y << endl;

                int layer = _blocks[block_id].layer;      //当前block在第几层（0层或1层）

                if(pin_x < x_min[layer])
                    x_min[layer] = pin_x;
                if(pin_x > x_max[layer])
                    x_max[layer] = pin_x;
                if(pin_y < y_min[layer])
                    y_min[layer] = pin_y;
                if(pin_y > y_max[layer])
                    y_max[layer] = pin_y;
            }
            if (x_min[0] == INT_MAX || x_min[1] == INT_MAX) {
                int l = (x_min[0] == INT_MAX)?  1 : 0;
                sum_half_perimeter += (x_max[l] - x_min[l] + y_max[l] - y_min[l]);
            } else {
                sum_half_perimeter += net.weight * (max(x_max[0] - x_min[1], 0.0) + max(y_max[0] - y_min[1], 0.0));
                sum_half_perimeter += net.weight * (max(x_max[1] - x_min[0], 0.0) + max(y_max[1] - y_min[0], 0.0));
            }
            if(is_debug)
                cout << "sum_half_perimeter: " << sum_half_perimeter << endl;
        }
        return sum_half_perimeter;
    }

    double Cost(BStarTree& _tree)
    //alpha * TotalArea(_tree) / initial_average_area + (1 - alpha) * TotalWireLength(_tree.blocks) / initial_average_wirelength + gamma * TotalExceedOutlineArea(_tree)/initial_average_exceed_outline_area;
    {
        double cost = 0;
        //用if语句，特殊情况下可以减少计算
        if(alpha != 0)
            cost += alpha * TotalArea(_tree) / initial_average_area;
        if(alpha != 1)
            cost += (1 - alpha) * TotalWireLength(_tree.blocks) / initial_average_wirelength;
        if(gamma != 0)
            cost += gamma * TotalExceedOutlineArea(_tree)/initial_average_exceed_outline_area;
        return cost;
    }
    double Time()
    {
        return (double)(end_ms - start_ms)/CLOCKS_PER_SEC;
    }
    //todo 数据改为输出到结构体中，主要是函数OutputBlocks中的部分有用信息
    void Output(string output_file)
    {
        cout << endl << _cfg.instance <<" instance completed" << endl;
        cout<<"random seed: " << _cfg.random_seed << endl;
        cout <<"best wirelength: " << TotalWireLength(best_tree.blocks) << endl;
        //fout<<"dead_sapce_rate: "<<dead_sapce_rate<<endl;
        cout<<"initial_num_perturbs: "<<initial_num_perturbs<<endl;
        cout<<"iter: "<<iter<<endl<<"best_iter: "<<best_iter<<endl<<"best_cost: "<<best_cost<<endl\
            <<"reject_time: "<<reject_time<<endl<<"accept_inferior_time: "<<accept_inferior_time<<endl\
            <<"update_best_time: "<<update_best_time<<endl<<"time cost: "<<Time()<<endl<<endl;
        ofstream fout(output_file);
        if(!fout)
        {
            cout<<"output error in file: "<<output_file<<endl;
            exit(-1);
        }
        OutputBlocks(fout, best_tree.blocks);
        OutputTrees(fout, best_tree);
        fout <<"random seed: " << _cfg.random_seed << endl;
        fout <<"alpha: " << _cfg.alpha << endl;
        fout <<"gamma: " << _cfg.gamma << endl;
        fout <<"best wirelength: " << TotalWireLength(best_tree.blocks) << endl;
        fout <<"initial_num_perturbs: "<<initial_num_perturbs<<endl;
        fout << "iter: " << iter << endl\
            << "best_iter: " << best_iter << endl\
            << "best_cost: " << best_cost << endl\
            << "reject_time: " << reject_time << endl\
            << "accept_inferior_time: " << accept_inferior_time << endl\
            << "update_best_time: " << update_best_time << endl\
            << "adjustLegalOutline_fail_times: " << adjustLegalOutline_fail_times << endl
            << "outline_adjust_time: " << double(outline_adjust_ms) / CLOCKS_PER_SEC << endl
            << "initial_wirelength: " << initial_wirelength << endl
            << "initial_average_wirelength: " << initial_average_wirelength << endl
            << initial_best_wirelength << ","
            <<"initial_iter: " << initial_iter<<endl
            <<"initial_time: "<<initial_time<<endl
            <<"total time cost: "<<Time()<<endl;
        fout.close();
    }
    void record_log(string log_file)
    {
        ofstream fout(log_file, ios::app);
        if(!fout)
        {
            cout<<"output error in file: "<<log_file<<endl;
            exit(-1);
        }
        fout.seekp(0, ios::end);
        if (fout.tellp() <= 0) {
            fout << "Instance,"
                    "Alpha,"
                    "Gamma,"
                    "RandSeed,"
                    "BestWireLength,"
                    "Time,"
                    "Iteration,"
                    "BestIter,"
                    "BestCost,"
                    "reject_times,"
                    "accept_inferior_times,"
                    "update_best_times,"
                    "outline_fail_times,"
                    "outline_adjust_time,"
                    "InitialWireLength,"
                    "InitialAvgWireLength,"
                    "InitialBestWireLength,"
                    "InitialTime,"
                    "InitialIteration," ;
            for(int i=0; i < layer_size; i++)
            {
                fout << "layer_" + to_string(i) + "_width,"
                    << "layer_" + to_string(i) + "_height,"
                    << "layer_" + to_string(i) + "_fillrate,"
                    << "layer_" + to_string(i) + "_userate,";
                //<< best_tree[i].width_ << ","
            }
            fout << endl;
        }
        fout << _cfg.instance << ","
            << _cfg.alpha << ","
            << _cfg.gamma << ","
                << _cfg.random_seed << ","
                << TotalWireLength(best_tree.blocks) << ","
                << Time() << ","
                << iter << ","
                << best_iter << ","
                << best_cost << ","
                << reject_time << ","
                << accept_inferior_time << ","
                << update_best_time << ","
                << adjustLegalOutline_fail_times << ","
                << double(outline_adjust_ms) / CLOCKS_PER_SEC << ","
                << initial_wirelength << ","
                << initial_average_wirelength << ","
                << initial_best_wirelength << ","
            << initial_time << ","
            << initial_iter << "," ;
        for(int i=0; i < layer_size; i++)
        {
            fout << best_tree.width_[i] << ","
                 << best_tree.height_[i] << ","
                 << best_tree.FillingRate(i)  << ","
                <<best_tree.UtilizationRate(i) << ",";
        }
        fout << endl;
    }
};
