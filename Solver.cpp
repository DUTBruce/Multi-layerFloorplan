

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
    vector<Block> blocks_cur, blocks_pre, blocks_best;  //当前操作的模块，保存的上一次的模块（作为恢复的副本），以及最好解的模块
    vector<Net> nets;
    //vector<int> blocks_tree_number;     //blocks_cur[i]所在的树，可以直接存在block中，已用blocks_cur[block_id].layer代替
    int layyer_size;    //几层的floorplanning
    unordered_map<string, int> blocks_name_id;   //块的名字到编号（0到b_num-1）的映射
    Config _cfg;
    int b_num;          //总共的block数量
    vector<BStarTree> cur_tree, pre_tree, best_tree;    //vector中layyer_size个tree的block数量总和等于b_num
    double alpha;  //代价函数的面积系数，面积所占比重
    double initial_average_area, initial_average_wire_length;   //初始化时确定的平均面积和线长，便于计算多目标Cost() = area/avg + wirelength/avg;
    double cost_pre, cost_cur, best_cost;
    COORD_TYPE total_block_area;
    double dead_sapce_rate;
    /*算法评测信息*/
    int initial_num_perturbs;
    int iter=0, best_iter=0;
    int reject_time=0;
    int accept_inferior_time=0;
    int update_best_time=0;
    time_t start_ms, end_ms;

    Solver(Config &cfg)
    {
        _cfg = cfg;
        _cfg.start_ms = start_ms = clock();
        //start_ms = _cfg.start_ms;
        layyer_size = _cfg.layyer_max;
        alpha = _cfg.alpha;
        //best_tree.resize(layyer_size);
    }
    void run()
    {
        srand(_cfg.random_seed);
        solve(_cfg.block_file, _cfg.net_file, _cfg.output_file);
    }
    void solve(string blockpath, string netpath, string output_file)
    {
        Initialization(blockpath, netpath);
        //ConvSA();
        end_ms = clock();
//        OutputBlocks(std::cout, blocks_cur);
//        OutputTrees(std::cout,cur_tree);
        OutputBlocks(std::cout, blocks_best);   //bug已修正，blocks_best未用pack后的blocks_cur赋值
        OutputTrees(std::cout,best_tree);
        Output(output_file);
    }

//    void SA()
//    {
//        /*cur_tree.Initialization();
//        cur_tree.Pack();
//        best_tree = pre_tree = cur_tree;
//        best_cost = cost_pre = Cost();*/
//        double initial_p = 0.99;
//        /*double t = ( - best_cost / log(initial_p) );
//        cout<<"t: "<<t<<endl<<endl;
//        double t_min = t / 1e3;*/
//        double t = 1e3;
//        double t_min = 1e-4;
//        double lamda = 0.85;         //退火系数，迭代128*100(num_iterations)次
//        //int num_iterations = cur_tree.b_num * cur_tree.b_num;
//        int num_iterations = b_num * b_num * 30;   //每个温度下的迭代次数
//        double deta_cost;
//        /*iter = best_iter = 0;
//        reject_time = 0;
//        accept_inferior_time = 0;
//        update_best_time=0;*/
//        while(t > t_min)
//        {
//
//            for(int i=0; i<num_iterations && (double)(clock()-start_ms)/CLOCKS_PER_SEC < _cfg.time_limit; i++)
//            {
//                Perturb();
//                cur_tree.Pack();
//                cost_cur = Cost();
//                deta_cost = cost_cur - cost_pre;
//                //cout<<"deta_cost: "<<deta_cost<<endl;
//                //cout<<"cost_pre: "<<cost_pre<<", best_cost: "<<best_cost<<endl;
//                if(deta_cost <= 0)               //为什么是<=0，和<0有何区别：因为=0的时候接受劣解概率公式结果为1，会一直接受新解
//                {
//                    //cout<<"cost_cur: "<<cost_cur<<", best_cost: "<<best_cost<<endl;
//                    if(cost_cur < best_cost)    //更新最优解
//                    {
//                        best_cost = cost_cur;
//                        best_tree = cur_tree;
//                        //cout<<"best_cost: "<<best_cost<<endl;
//                        best_iter = iter;
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
//                    /*if(iter % 100 == 0)
//                    {
//                        cout<<"accept, "<<"deta_cost: "<<deta_cost<<" t: "<<t<<" exp( - deta_cost / t): "<<exp( - deta_cost / t)<<endl;
//                    }*/
//                }
//                else
//                {
//                    cur_tree = pre_tree;            //不接受新解，从 S(i+1) 回滚到 S(i)
//                    reject_time++;
//                    //cout<<"reject in iter: "<<iter<<endl;
//                }
//                iter++;
//            }
//            t = t * lamda;
//        }
//        dead_sapce_rate = (float)best_tree.Area() / total_block_area - 1;
//    }
//    void ConvSA()
//    {
//        double conv_rate = 1;    //定义收敛率，拒绝率>=收敛率时搜索完毕
//        double initial_p = 0.999;
//        double total_deta_cost=0, average_deta_cost;
//        for(int i=0; i<cur_tree.tree_b_num * cur_tree.tree_b_num; i++)
//        {
//            cur_tree.Perturb();
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
//                cur_tree.Perturb();
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
    void Perturb() //o(h) 在解层面对所有树实行扰动，对相应影响的树进行布局Pack（实际操作实现的在模块层面进行扰动）
    {
        bool is_debug = false;
        if(is_debug)
            cout << "begin perturb" << endl;

        /*在树层面进行扰动
        int random = rand() % (2*layyer_size - 1);  //扰动的动作，从0到layyersize-1对应一棵树的扰动，其余动作对应树间扰动
        //1. 动作i在0-layyersize-1中，扰动树i
        if(random < layyer_size || layyer_size == 1)
        {
            cur_tree[layyer_size].Perturb();
        }
        //2. 剩余2*layyer_size - 1种情况，进行树间扰动：移动模块从一个树到另一个树。2层的时候为1/3概率
        else
        {

        }*/

        //模块层面进行扰动
        int perturb_move_size = 3;  //扰动操作的数量
        int random = rand() % perturb_move_size;
        switch(random)
        {
            case 0: //旋转模块 o(1)
            {
                int block_id = rand()%b_num;
                if(is_debug)
                    cout<<"rotate "<<block_id<<endl;
                RotateBlock(block_id);
                break;
            }
            case 1: //移动模块（删除并插入） o(h)
            {
                int from = rand()%b_num;
                int to = rand()%b_num;
                while(to == from)
                    to = rand()%b_num;
                if(is_debug)
                    cout<<"move "<<from<<" to "<<to<<endl;
                MoveBlocks(from, to);
                break;
            }
            case 2: //交换模块（两边同时删除和插入） o(1)
            {
                int id_1 = rand()%b_num;
                int id_2 = rand()%b_num;
                while(id_2 == id_1)
                    id_2 = rand()%b_num;
                if(is_debug)
                    cout<<"swap "<<id_1<<" and "<<id_2<<endl;
                SwapBlocks(id_1, id_2);
                break;
            }
        }

        if(is_debug)
            cout << "complete perturb" << endl;
    }
    void RotateBlock(int block_id)
    {
        cur_tree[blocks_cur[block_id].layer].RotateBlock(block_id);
        cur_tree[blocks_cur[block_id].layer].Pack();
    }
    void MoveBlocks(int from, int to)
    {
        //cout << "begin delete" << endl;
        cur_tree[blocks_cur[from].layer].DeleteBlock(from);
        //cout << "begin insert" << endl;
        cur_tree[blocks_cur[to].layer].InsertBlock(from, to);
        cur_tree[blocks_cur[from].layer].Pack();
        if(blocks_cur[from].layer != blocks_cur[to].layer)
            cur_tree[blocks_cur[to].layer].Pack();
    }
    void SwapBlocks(int id_1, int id_2)  //o(1)
    {
        bool is_debug = false;
        if(id_1 == 3 && id_2 == 2)
        {
            //is_debug = true;
        }
        Block& block_1 = blocks_cur[id_1];      //记得引用！不然是临时变量，修改完了没用！
        Block& block_2 = blocks_cur[id_2];
        int root_1 = cur_tree[block_1.layer].root;
        int root_2 = cur_tree[block_2.layer].root;
        swap(block_1.left, block_2.left);
        swap(block_1.right, block_2.right);
        swap(block_1.parent, block_2.parent);
        swap(block_1.is_from_left, block_2.is_from_left);
        swap(block_1.layer, block_2.layer);
        //特殊情况处理
        //1. 两节点相邻时，更正错误信息
        if(block_1.left == id_1)
        {
            block_1.left = id_2;
            block_2.parent = id_1;
        }
        else if(block_1.right == id_1)
        {
            block_1.right = id_2;
            block_2.parent = id_1;
        }
        else if(block_1.parent == id_1)
        {
            block_1.parent = id_2;
            if(block_1.is_from_left)
                block_2.left = id_1;
            else
                block_2.right = id_1;
        }
        //2. 更新父节点和子节点指向信息
        if(block_1.parent != -1)
        {
            if(block_1.is_from_left)
                blocks_cur[block_1.parent].left = id_1;
            else
                blocks_cur[block_1.parent].right = id_1;
        }
        if(block_1.left != -1)
            blocks_cur[block_1.left].parent = id_1;
        if(block_1.right != -1)
            blocks_cur[block_1.right].parent = id_1;
        if(block_2.parent != -1)
        {
            if(block_2.is_from_left)
                blocks_cur[block_2.parent].left = id_2;
            else
                blocks_cur[block_2.parent].right = id_2;
        }
        if(block_2.left != -1)
            blocks_cur[block_2.left].parent = id_2;
        if(block_2.right != -1)
            blocks_cur[block_2.right].parent = id_2;
        //3. 如果为根节点则更新相应tree的root
        if(is_debug) {
            cout<< "root_1: " << root_1 << ", root_2: " << root_2 << endl;
            cout << "id_1: " << id_1 << ", cur_tree[block_1.layer].root:" << cur_tree[block_1.layer].root << endl;
            cout << "id_2: " << id_2 << ", cur_tree[block_2.layer].root:" << cur_tree[block_2.layer].root << endl;
        }
        //bug在于下面这俩if判断会相互抵消，又回到了原来，所以需要提前存下root或者用else分类讨论
        if(id_1 == root_1)
            cur_tree[block_2.layer].root = id_2;
        if(id_2 == root_2)
            cur_tree[block_1.layer].root = id_1;


        if(is_debug) {
            cout << "id_1: " << id_1 << ", cur_tree[block_1.layer].root:" << cur_tree[block_1.layer].root << endl;
            cout << "id_2: " << id_2 << ", cur_tree[block_2.layer].root:" << cur_tree[block_2.layer].root << endl;
            OutputBlocks(cout, blocks_cur);
            OutputTrees(cout, cur_tree);
        }
    }
    void Pack()
    {
        for(int it = 0; it < layyer_size; it++)
            cur_tree[it].Pack();
    }
    void Initialization(string blockpath, string netpath)       //初始化树结构和平均线长平均面积等信息
    {
        bool is_debug = false;

        //1. 读取并初始化为layyersize个完全二叉树
        //blocks_cur.resize(_cfg.layyer_max, Block()); 为什么我会这么写，感觉有问题
        ReadFromFile(blockpath, netpath);
        /*cur_tree.b_num = 6;
        cur_tree.blocks_cur = {Block(1,1),Block(2,2),Block(2,1),Block(1,1),Block(1,1),Block(3,2)};
        Net::nets = {{0,1,3}, {2,4,5}, {0,1,2,3,4,5}};*/
        total_block_area = 0;
        for(auto b: blocks_cur)    //记录所有块总面积
        {
            total_block_area += b.area();
        }
        cout<<"total_block_area: "<<total_block_area<<endl;
        Initial_blocks();
        Initial_trees();
        //OutputBlocks(std::cout, blocks_cur);  //pack前，坐标还都是(0,0)
        Pack();
        blocks_best = blocks_pre = blocks_cur;
        best_tree = pre_tree = cur_tree;
        if(is_debug)
        {
            OutputBlocks(std::cout, blocks_cur);  //pack后，坐标在放置的位置上
            OutputTrees(std::cout, cur_tree);
        }


        COORD_TYPE initial_area = TotalArea(cur_tree);
        double initial_wirelength = TotalWireLength(blocks_cur);
        double initial_best_cost = 1; //随机初始化过程的最好代价 alpha * cur_area / initial_area + (1 - alpha) *  cur_wirelength/ initial_wirelength
        long long total_area = 0;
        double total_wirelength = 0;
        initial_num_perturbs = b_num * b_num;  //随机初始化的次数（单目标的话选取其中最好的解，多目标时不方便量化，不一定是最好的解）
        COORD_TYPE cur_area;
        double cur_wirelength;
        double cur_cost;
        for(int i=0; i<initial_num_perturbs; i++)
        {
            Perturb();
            Pack();
            cur_area = TotalArea(cur_tree);
            cur_wirelength = TotalWireLength(blocks_cur);
            cur_cost = alpha * cur_area / initial_area + (1 - alpha) *  cur_wirelength/ initial_wirelength;
            if(cur_cost < initial_best_cost)
            {
                initial_best_cost = cur_cost;
                blocks_best = blocks_cur;
                best_tree = cur_tree;
            }
            total_area += cur_area;
            total_wirelength += cur_wirelength;
            if(is_debug){
                OutputBlocks(std::cout, blocks_cur);
            }
        }
//        cout<<"total_area: "<<total_area<<endl;
//        cout<<"total_wirelength: "<<total_wirelength<<endl;
        initial_average_area = (double)total_area / initial_num_perturbs;
        initial_average_wire_length = (double)total_wirelength / initial_num_perturbs;
        cout<<"initial_area: " << initial_area <<endl;
        cout<<"initial_wirelength: " << initial_wirelength <<endl;
        cout<<"initial_average_area: "<<initial_average_area<<endl;
        cout << "initial_average_wire_length: " << initial_average_wire_length << endl;
        cout<<"initial best area: "<< TotalArea(best_tree)<<endl;
        cout<<"initial best wirelength: "<<TotalWireLength(blocks_best)<<endl;
        cout<<"last perturb area: " << cur_area<<endl;
        cout<<"last perturb wirelength: " << cur_wirelength<<endl;
        cout<<"initial_best_cost: "<<initial_best_cost<<endl;
        cout<<"last cost: " << cur_cost << endl;
        blocks_pre = blocks_cur = blocks_best;
        pre_tree = cur_tree = best_tree;
        best_cost = cost_pre = Cost(cur_tree, blocks_cur);
        cout<<"now best_cost: " << best_cost <<endl;
        cout<<endl;
    }
    void Initial_blocks()
    {
        for(int ib=0; ib < b_num; ib++)
        {
            blocks_cur[ib].parent = blocks_cur[ib].left = blocks_cur[ib].right = -1;
            blocks_cur[ib].is_from_left = false;
            /*旋转模块复原*/
            blocks_cur[ib].rotated_angle = 0;
        }
        blocks_best = blocks_pre = blocks_cur;
    }
    void Initial_trees()
    {
//        //每个block随机选择放入哪个树（未实现）
//        for(int it=0; it<layyer_size; it++)
//        {
//
//        }
//        for(int ib=0; ib < b_num; ib++)
//        {
//            int random_tree = rand() % layyer_size;
//            blocks_tree_number[ib] = random_tree;
//            cur_tree[random_tree].
//        }

        //0. 初始化树空间
        //cur_tree.resize(layyer_size, BStarTree(blocks_cur));
        //chatgpt告诉我对于具有引用类型成员变量的类，它们不能被默认构造或复制，因此在使用 vector 的 resize() 函数时会导致问题。得按下面的初始化
        cur_tree.reserve(layyer_size);  // 提前分配足够的空间
        best_tree.reserve(layyer_size);
        pre_tree.reserve(layyer_size);
        for (int i = 0; i < layyer_size; i++) {
            // 逐个初始化BStarTree对象
            cur_tree.emplace_back(&blocks_cur, i);
            best_tree.emplace_back(&blocks_best, i);
            pre_tree.emplace_back(&blocks_pre, i);
        }
        //1. 每个树平均放置若干block节点（以完全二叉树按层生成节点）
        int ave_tree_blocksnum = b_num / layyer_size;
        //1.x 特殊情况
        if(ave_tree_blocksnum == 0) //树比blocks多，前b_num个树每个放一个
        {
            cerr << "wait for coding." << endl;
        }
        //1.1 正常情况，前layyer_size-1个树平均放
        for(int it=0; it<layyer_size-1; it++)
        {
            cur_tree[it].initial_blocks(it*ave_tree_blocksnum, it+ave_tree_blocksnum);
        }
        //1.2 最后一个树放剩下所有blocks
        cur_tree[layyer_size-1].initial_blocks((layyer_size-1) * ave_tree_blocksnum, b_num);

        best_tree = pre_tree = cur_tree;

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
        blocks_cur.clear();
        blocks_cur.resize(b_num);
        //1.1 分layyer_size层读取blocks信息和block上的引脚信息
        for(int cur_layyer = 0; cur_layyer < layyer_size; cur_layyer++)
        {
            for(int i=0; i<b_num; i++)
            {
                //1.1.1 读取当前的block长宽信息
                Block &cur_block = blocks_cur[i];
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
                        cur_block = Block(layyer_size, block_name);
                        cur_block.add_layer_info(0, x1 - x0, y1 - y0);
                        blocks_name_id[block_name] = i;
                    }
                        //非首次读取
                    else
                    {
                        assert(blocks_name_id.count(block_name) == cur_layyer);
                        cur_block.add_layer_info(cur_layyer, x1 - x0, y1 - y0);
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
        assert(blocks_cur.size() == b_num);
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
                assert(pin_id >= 0 && pin_id < blocks_cur[block_id].pins_num);
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
            Block &b = blocks_cur[id];
            cout<<"id:"<<id<<" width:"<<b.width[b.layer]<<" height:"<<b.height[b.layer]<<endl;
            cin>>name;
        }
    }
    COORD_TYPE BlocksArea()
    {
        COORD_TYPE blocks_area = 0;
        for(Block b : blocks_cur)
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
                << b[i].left << ","
                << b[i].right << ","
                << b[i].parent << ","
                << b[i].is_from_left << ","
               <<endl;
        out << "Wirelength: " << TotalWireLength(b) << endl;
    }
    void OutputTrees(ostream &out, vector<BStarTree> &t)
    {
        out<<"layer(tree) info:"<<endl;
        for(int i=0; i<layyer_size; i++)
        {
            out<<"layer: "<<i<<" ,width: "<<t[i].width_<<" ,height: "<<t[i].height_<<" ,area: "<<t[i].Area()<<endl;
        }
    }
    double FillingRate(vector<BStarTree>& _tree)
    {

    }
    COORD_TYPE TotalArea(vector<BStarTree>& _tree)
    {
        COORD_TYPE total_area = 0;
        for(int it = 0; it < layyer_size; it++)
            total_area += _tree[it].Area();
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
        double sum_half_perimeter = 0;
        for(auto net: nets)
        {
            double x_max[2] = {INT_MIN, INT_MIN}, x_min[2] = {INT_MAX, INT_MAX};
            double y_max[2] = {INT_MIN, INT_MIN}, y_min[2] = {INT_MAX, INT_MAX};
            for(int i=0; i<net.degree; i++)     //该网表的所有block
            {
                int block_id = net.connected_blocks[i];
                int pin_id = net.connected_pins[i];
                pair<COORD_TYPE, COORD_TYPE> pin_coor = _blocks[block_id].get_pin_coor(pin_id);   //获得引脚坐标
                COORD_TYPE pin_x = pin_coor.first;
                COORD_TYPE pin_y = pin_coor.second;

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
                sum_half_perimeter += (max(x_max[0] - x_min[1], 0.0) + max(y_max[0] - y_min[1], 0.0));
                sum_half_perimeter += (max(x_max[1] - x_min[0], 0.0) + max(y_max[1] - y_min[0], 0.0));
            }
        }
        return sum_half_perimeter;
    }

    double Cost(vector<BStarTree>& _tree, vector<Block>& _blocks)
    {
        if(alpha == 1)
        {
            return TotalArea(_tree) / initial_average_area;
        }
        else
            return alpha * TotalArea(_tree) / initial_average_area + (1 - alpha) * TotalWireLength(_blocks) / initial_average_wire_length;
    }
    double Time()
    {
        return (double)(end_ms - start_ms)/CLOCKS_PER_SEC;
    }
    void Output(string output_file)
    {
        cout << endl << _cfg.instance <<"instance completed" << endl;
        cout<<"random seed: " << _cfg.random_seed << endl;
        cout <<"best wirelength: " << TotalWireLength(blocks_best) << endl;
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
        OutputBlocks(fout, blocks_best);
        OutputTrees(fout, best_tree);
        fout<<"random seed: " << _cfg.random_seed << endl;
        fout <<"best wirelength: " << TotalWireLength(blocks_best) << endl;
        //fout<<"dead_sapce_rate: "<<dead_sapce_rate<<endl;
        fout<<"initial_num_perturbs: "<<initial_num_perturbs<<endl;
        fout<<"iter: "<<iter<<endl\
            <<"best_iter: "<<best_iter<<endl\
            <<"best_cost: "<<best_cost<<endl\
            <<"reject_time: "<<reject_time<<endl\
            <<"accept_inferior_time: "<<accept_inferior_time<<endl\
            <<"update_best_time: "<<update_best_time<<endl\
            <<"time cost: "<<Time()<<endl;
        fout.close();
    }
};
