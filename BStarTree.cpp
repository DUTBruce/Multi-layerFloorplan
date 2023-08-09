/*2023/7/4
 * 重构代码，一个BStarTree改为多层,内部保存一个blocks副本代替指针,避免浅复制问题
*/
/*2023/5/30
 * 重构代码，改写为支持两层的B*tree布图规划
*/


#include "Block.cpp"
#include "Contour.hpp"

using namespace std;

class BStarTree      //一个tree即一个多层floorplan，记录了包含哪些模块以及模块的放置位置
{
public:
    vector<Block> blocks;  //对Solver中blocks的指针（引用会出错）
    int layer_size;    //几层的floorplanning,下面的数组vector都表示在第i层的情况(例如各层根节点,根节点坐标,各层模块数量)
    vector<int> root;           //根节点编号
    vector<COORD_TYPE> root_x, root_y; //根节点坐标（默认是最贴着左下角(0,0)，后续也可以调整）
    //int tree_layer;     //当前tree表示第几层
    vector<int> tree_b_num;          //当前树各层的block数量,所有层加一块==blocks.size()
    vector<int> pack_num;           //pack操作的块数量（校对和树block数量一致）
    set<int> pertub_layers;      //记录扰动过的树,没扰动过的层不需要pack操作
    vector<Contour> contour;    //到当前遍历结点的等高线，根据遍历顺序迭代更新（每次pack前需要reset）
    vector<COORD_TYPE> width_, height_;       //布图轮廓的宽和高
    vector<COORD_TYPE> blocks_area;     //当前树（布图）内所有blocks的面积和
    COORD_TYPE outline_width_, outline_height_; //固定轮廓的宽和高
    vector<COORD_TYPE> exceed_outline_blocks_area;   //超出轮廓的模块面积和
    vector<double> userate_max;      // 利用率限制，#define IfUtilizationLimit true时使用
    BStarTree(){
    }
    BStarTree(vector<Block> b, int layer_size, COORD_TYPE outline_width = COORD_TYPE_MAX, COORD_TYPE outline_height = COORD_TYPE_MAX)
    {
        blocks = b;     //如果是引用类型(这里不是)成员变量必须在构造函数初始化列表中进行初始化，而不能在构造函数体内进行赋值操作。
        this->layer_size = layer_size;
        root.resize(layer_size,-1);
        root_x.resize(layer_size, 0);
        root_y.resize(layer_size, 0);
        tree_b_num.resize(layer_size,0);
        pack_num.resize(layer_size,0);
        contour.resize(layer_size);
        width_.resize(layer_size);
        height_.resize(layer_size);
        blocks_area.resize(layer_size);
        exceed_outline_blocks_area.resize(layer_size);
        outline_width_ = outline_width;
        outline_height_ = outline_height;

    }
    void Perturb() //o(h) 在解层面对所有树实行扰动，对相应影响的树进行布局Pack（实际操作实现的在模块层面进行扰动）
    //6.15新增，Perturb后自动pack扰动的树
    //6.20新增已解决问题: 移动模块时可能将树移为空，树为空时不会再移入模块。已解决：增加把模块移到对应树根节点之前的操作，也就是新增to的选择
    //7.4新增，把Perturb和Pack移到BStarTree内，并记录扰动过的层，方便后续多次扰动一次pack的需求
    //7.27新增，如果有利用率限制则扰动不会执行（后续改为执行后填补利用率违反）
    {
        bool is_debug = false;
        if(is_debug)
            cout << "begin perturb" << endl;

        /*在树层面进行扰动
        int random = rand() % (2*layer_size - 1);  //扰动的动作，从0到layyersize-1对应一棵树的扰动，其余动作对应树间扰动
        //1. 动作i在0-layyersize-1中，扰动树i
        if(random < layer_size || layer_size == 1)
        {
            cur_tree[layer_size].PerturbAndPack();
        }
        //2. 剩余2*layer_size - 1种情况，进行树间扰动：移动模块从一个树到另一个树。2层的时候为1/3概率
        else
        {

        }*/

        //模块层面进行扰动
        pertub_layers.clear();
        int perturb_move_size = 3;  //扰动操作的数量
        int b_num = blocks.size();
        int random = rand() % perturb_move_size;
        switch(random)
        {
            case 0: //旋转模块 o(1)
            {
                int block_id = rand()%b_num;
                if(is_debug)
                    cout<<"rotateOrFlexSize "<<block_id<<endl;
                RotateBlock(block_id);
                break;
            }
            case 1: //移动模块（删除并插入） o(h)
            {
                int from = rand()%b_num;
                int to = rand() % (b_num + layer_size);      //模块移动的去向：除了1.所有模块后面，还有2.每层布局的根节点之前（成为新的根结点）
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
            default:
            {
                cerr << "wrong pertub random!" << endl;
                break;
            }
        }
        if(is_debug)
            cout << "complete perturb" << endl;
        if(is_debug && IfUtilizationLimit == true){   //看perturb之后pack之前的blocks_area面积计算的是否正确，如果正确会和pack后的一致
            OutputLayerInfo(cout);
        }
    }
    void RotateBlock(int id)
    {
        if(IfUtilizationLimit == true && blocks[id].shape_num != 1) //有利用率限制时，需要判断模块更改形状后的面积是否会超利用率
        {
            Block temp_block = blocks[id];
            int layer = blocks[id].layer;
            int random = rand() % 2;
            if(random == 0)      //旋转
            {
                blocks[id].rotateOrFlexSize(1);
            }
            else   //调整尺寸
            {
                temp_block.rotateOrFlexSize(2); //调整形状后的模块
                //int shape_id = (blocks[id].shape_id + 1) % blocks[id].shape_num;
                if(blocks_area[layer] + temp_block.area() - blocks[id].area() > use_area_limit(layer))//调整后超出利用率，则只旋转不改变形状
                {
                    blocks[id].rotateOrFlexSize(1);
                }
                else //不超出则正常调整，并更新面积
                {
                    blocks_area[layer] = blocks_area[layer] + temp_block.area() - blocks[id].area();
                    blocks[id].rotateOrFlexSize(2);
                }
            }

        }
        else
        {
            blocks[id].rotateOrFlexSize();
            pertub_layers.insert(blocks[id].layer);
        }
    }
    void MoveBlocks(int from, int to)
    {

        //cout << "begin delete" << endl;
        int layer_from = blocks[from].layer;    //预先存一下所在层，不然后续更改后blocks_cur[from].layer会变为to的layer
        int layer_to = to < blocks.size()? blocks[to].layer: to - blocks.size();
        if(IfUtilizationLimit == true && layer_to != layer_from) //有利用率限制且跨层移动时，需要判断模块移动后那层的面积是否会超利用率
        {
            if( blocks_area[layer_to] + blocks[from].area(layer_to)  > use_area_limit(layer_to) )
                return;
            else{
                blocks_area[layer_to] += blocks[from].area(layer_to);
                blocks_area[layer_from] -= blocks[from].area(layer_from);
            }
        }
        DeleteBlock(from);
        if(to < blocks.size())  //1.正常移动到模块to后面
        {
            //layer_to = blocks[to].layer;
            //cout << "begin insert" << endl;
            InsertBlock(from, to);
        }
        else    //2. 模块移动到布局 to - b_num (0到layyer_size-1) 的根节点之前
        {
            //layer_to = to - blocks.size();
            InsertRoot(from, layer_to);
        }
        pertub_layers.insert(layer_from);
        if(layer_from != layer_to)
            pertub_layers.insert(layer_to);
    }
    void SwapBlocks(int id_1, int id_2)  //o(1)
    {
        bool is_debug = false;
        if(id_1 == 3 && id_2 == 2)
        {
            //is_debug = true;
        }
        Block& block_1 = blocks[id_1];      //记得引用！不然是临时变量，修改完了没用！
        Block& block_2 = blocks[id_2];
        if(IfUtilizationLimit == true && block_1.layer != block_2.layer) //有利用率限制且跨层移动时，需要判断模块交换后两层的面积是否会超利用率
        {
            if( blocks_area[block_1.layer] - block_1.area(block_1.layer) + block_2.area(block_1.layer)  > use_area_limit(block_1.layer)   //模块1所在层
                || blocks_area[block_2.layer] - block_2.area(block_2.layer) + block_1.area(block_2.layer)  > use_area_limit(block_2.layer))   //模块2所在层
                return;
            else{
                blocks_area[block_1.layer] = blocks_area[block_1.layer]- block_1.area(block_1.layer) + block_2.area(block_1.layer);
                blocks_area[block_2.layer] = blocks_area[block_2.layer] - block_2.area(block_2.layer) + block_1.area(block_2.layer);
            }
        }
        int root_1 = root[block_1.layer];
        int root_2 = root[block_2.layer];
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
                blocks[block_1.parent].left = id_1;
            else
                blocks[block_1.parent].right = id_1;
        }
        if(block_1.left != -1)
            blocks[block_1.left].parent = id_1;
        if(block_1.right != -1)
            blocks[block_1.right].parent = id_1;
        if(block_2.parent != -1)
        {
            if(block_2.is_from_left)
                blocks[block_2.parent].left = id_2;
            else
                blocks[block_2.parent].right = id_2;
        }
        if(block_2.left != -1)
            blocks[block_2.left].parent = id_2;
        if(block_2.right != -1)
            blocks[block_2.right].parent = id_2;
        //3. 如果为根节点则更新相应tree的root
        if(is_debug) {
            cout<< "root_1: " << root_1 << ", root_2: " << root_2 << endl;
            cout << "id_1: " << id_1 << ", root[block_1.layer]:" << root[block_1.layer] << endl;
            cout << "id_2: " << id_2 << ", root[block_2.layer]:" << root[block_2.layer] << endl;
        }
        //bug在于下面这俩if判断会相互抵消，又回到了原来，所以需要提前存下root或者用else分类讨论
        if(id_1 == root_1)
            root[block_2.layer] = id_2;
        if(id_2 == root_2)
            root[block_1.layer] = id_1;
        pertub_layers.insert(block_1.layer);
        if(block_1.layer != block_2.layer)
            pertub_layers.insert(block_2.layer);

        if(is_debug) {
            cout << "id_1: " << id_1 << ", root[block_1.layer]:" << root[block_1.layer] << endl;
            cout << "id_2: " << id_2 << ", root[block_2.layer]:" << root[block_2.layer] << endl;
        }
    }
    void DeleteBlock(int from)  //替换删除，如果删除根节点会在SwapBlock中更改对应root，但如果树中只有根节点需要设置root=-1
    {
        //delete, 为了尽可能保持结点相对结构，逐层向下替换删除，\
        左右结点选一个替代父结点，递归向下直至叶子结点（左右结点均为空）
        bool is_debug = false;
        Block& del_node = blocks[from];
        //assert(del_node.layer == tree_layer);
        if(is_debug)
            cout<<"1"<<endl;
        if(tree_b_num[del_node.layer] == 1)     //如果树中只有根节点需要设置root=-1
        {
            assert(del_node.left==-1 && del_node.right==-1 && from==root[del_node.layer]); //树中此层只有from一个结点
            tree_b_num[del_node.layer] --;
            root[del_node.layer] = -1;
            return;
        }
        while(del_node.left!=-1 || del_node.right!=-1)  //！！原出错语句为while(del_node.left!=-1 || del_node.left!=-1)
        {
            if(is_debug)
                cout<<"1.1"<<endl;
            if(del_node.left!=-1 && del_node.right!=-1)
            {
                if(rand()%2)
                {
                    SwapBlocks(from, del_node.left);
                }
                else
                    SwapBlocks(from, del_node.right);
            }
            else if(del_node.left!=-1)
                SwapBlocks(from, del_node.left);
            else
                SwapBlocks(from, del_node.right);
        }
        if(is_debug)
            cout<<"2"<<endl;
        if(del_node.is_from_left)
        {
            blocks[del_node.parent].left = -1;
        }
        else
        {
            blocks[del_node.parent].right = -1;
        }
        tree_b_num[del_node.layer]--;
    }
    void InsertBlock(int from, int to)
    {
        //insert，（随机）插入到目标结点to的左结点或右节点，尽可能保持结构
        //assert(blocks[to].layer == tree_layer);
        blocks[from].layer = blocks[to].layer; //跨层插入时更正原block的layer为新所在层的layer
        if(rand()%2)    //左
        {
            blocks[from].is_from_left = true;
            blocks[from].parent = to;
            blocks[from].left = blocks[to].left;
            blocks[to].left = from;
            if(blocks[from].left != -1)
                blocks[blocks[from].left].parent = from;
        }
        else            //右
        {
            blocks[from].is_from_left = false;
            blocks[from].parent = to;
            blocks[from].right = blocks[to].right;
            blocks[to].right = from;
            if(blocks[from].right != -1)
                blocks[blocks[from].right].parent = from;
        }
        tree_b_num[blocks[to].layer]++;
    }
    void InsertRoot(int from, int layer)   //插入到第layer层根节点之前，原根节点（树）随机变为左孩子（左子树）/右孩子（右子树）
    {
        assert(layer < layer_size);
        blocks[from].layer = layer;
        blocks[from].parent = -1;
        blocks[from].is_from_left = false;
        if(rand()%2)    //左
        {
            if(root[layer] != -1)
            {
                blocks[root[layer]].parent = from;
                blocks[root[layer]].is_from_left = true;
            }
            blocks[from].left = root[layer];
            blocks[from].right = -1;
        }
        else            //右
        {
            if(root[layer] != -1)
            {
                blocks[root[layer]].parent = from;
                blocks[root[layer]].is_from_left = false;
            }
            blocks[from].left = -1;
            blocks[from].right = root[layer];
        }
        root[layer] = from;
        tree_b_num[layer]++;
    }

    void Pack()
    {
        assert(pertub_layers.size() <= layer_size);
        if(!pertub_layers.empty())
        {
            for(int layer: pertub_layers)
                Pack(layer);
        }
        else
        {
            for(int i = 0; i < layer_size; i++)
                Pack(i);
        }
        if(IfUtilizationLimit == true)
        {
            for(int layer: pertub_layers)
            {
                if(UtilizationRate(layer) > userate_max[layer])
                {
//                    OutputBlocks(cout);
//                    OutputLayerInfo(cout);
                    cerr << "IfUtilizationLimit failed in Pack()" << endl;
                    exit(-1);
                }
            }
        }
    }
    void Pack(int layer)
    {
        assert(layer < layer_size && layer >= 0);
        bool is_debug = false;
//        if(!(layer < layer_size && layer >= 0))
//            is_debug = true;
        if(is_debug)
            cout << "pack tree_layer: " << layer << ", blocks pack order: " ;
        contour[layer].reset();
        assert(blocks.size()!=0);
        pack_num[layer] = 0;
        blocks_area[layer] = 0;
        exceed_outline_blocks_area[layer] = 0;
        if(root[layer] == -1)  //树中无结点
        {
            width_[layer] = 0;
            height_[layer] = 0;
        }
        else
        {
            Pack(layer, root[layer]);
            width_[layer] = contour[layer].max_x();
            height_[layer] = contour[layer].max_y();
        }
        if(is_debug)
            cout << endl;
        assert(pack_num[layer] == tree_b_num[layer]);
    }
    void Pack(int layer, int block_id)    //根据树结构放置模块，无参数时默认从根节点开始
    {
        bool is_debug = false;
        if(is_debug)
            cout << block_id << ",";
        pack_num[layer]++;
        //1.放置模块（即更新xy坐标）
        Block& block =  blocks[block_id];
        assert(block.layer == layer);
        blocks_area[layer] += block.area();
//        if(block.layer != layer){
//            cerr << "block_id: " << block_id << ", block.layer: " << block.layer << ", tree_layer: " << layer << endl;
//            exit(3);
//        }
        if(block_id == root[layer])
        {
            block.x = root_x[layer];
            block.y = root_y[layer];
        }
        else
        {
            Block& parent = blocks[block.parent];
            block.x = block.is_from_left? parent.x+parent.get_width(): parent.x;
            block.y = contour[layer].find_max_y_between(block.x-root_x[layer], block.x-root_x[layer] + block.get_width()) + root_y[layer];   //o(n ^ 1/2)
        }
        //2.更新等高线、超出轮廓模块面积等信息
        contour[layer].Update(block.x - root_x[layer], block.x - root_x[layer] + block.get_width(), block.y - root_y[layer] + block.get_height());
        //contour.Show();//测试等高线的正确性
        exceed_outline_blocks_area[layer] += CalExceedBlockArea(block);

        //3.迭代Pack下一个模块
        if(block.left != -1)
            Pack(layer, block.left);
        if(block.right != -1)
            Pack(layer, block.right);
    }
    COORD_TYPE CalExceedBlockArea(Block &block) //7.12更正bug，原方法在模块完全超出轮廓时算出超出模块面积＞模块面积，导致assert逻辑出错
    {
        bool is_debug = false;
        COORD_TYPE exceed_width = min(block.x + block.get_width() - outline_width_, block.get_width()); //更正部分
        COORD_TYPE exceed_height = min(block.y + block.get_height() - outline_height_, block.get_height());
        if(exceed_width > 0)
        {
            if(exceed_height > 0)  //宽高都超出
            {
                block.exceed_outline_area = exceed_width * block.get_height() + exceed_height * block.get_width() - exceed_width * exceed_height;
            }

            else    //只有宽超出，超出部分*模块高
                block.exceed_outline_area = exceed_width * block.get_height();
        }
        else if(exceed_height > 0) //只有高超出，超出部分*模块宽
        {
            block.exceed_outline_area = exceed_height * block.get_width();
        }
        else    //宽高都不超出轮廓，超出模块面积和不变
        {
            block.exceed_outline_area = 0;
        }
        if(is_debug)
        {
            cout <<" exceed_width: " << exceed_width << ", exceed_height: " << exceed_height
            << ", block.exceed_outline_area: " << block.exceed_outline_area << endl;
        }
        assert(block.exceed_outline_area <= block.area());
        return block.exceed_outline_area;
    }
    /*COORD_TYPE CalExceedBlockArea(Block &block) //原bug函数，后续对比测试一下会不会比正常的效果好，原方法在模块完全超出轮廓时算出超出模块面积＞模块面积，导致assert逻辑出错
    {
        bool is_debug = true;
        COORD_TYPE exceed_width = block.x + block.get_width() - outline_width_;
        COORD_TYPE exceed_height = block.y + block.get_height() - outline_height_;
        if(exceed_width > 0)
        {
            if(exceed_height > 0)  //宽高都超出
            {
                block.exceed_outline_area = exceed_width * block.get_height() + exceed_height * block.get_width() - exceed_width * exceed_height;
            }

            else    //只有宽超出，超出部分*模块高
                block.exceed_outline_area = exceed_width * block.get_height();
        }
        else if(exceed_height > 0) //只有高超出，超出部分*模块宽
        {
            block.exceed_outline_area = exceed_height * block.get_width();
        }
        else    //宽高都不超出轮廓，超出模块面积和不变
        {
            block.exceed_outline_area = 0;
        }
        if(is_debug)
        {
            cout <<" exceed_width: " << exceed_width << ", exceed_height: " << exceed_height
                 << ", block.exceed_outline_area: " << block.exceed_outline_area << endl;
        }
        return block.exceed_outline_area;
    }*/
    /*void Perturb()      //原单层函数，o(h) 对当前树实行扰动
    {
        int perturb_move_size = 3;  //扰动操作的数量
        int random = rand() % perturb_move_size;
        switch(random)
        {
            case 0: //旋转模块 o(1)
            {
                int block_id = rand()%tree_b_num;
                //cout<<"rotateOrFlexSize "<<block_id<<endl;
                RotateBlock(block_id);
                break;
            }
            case 1: //移动模块（删除并插入） o(h)
            {
                int from = rand()%tree_b_num;
                int to = rand()%tree_b_num;
                while(to == from)
                    to = rand()%tree_b_num;
                //cout<<"move "<<from<<" to "<<to<<endl;
                MoveBlock(from, to);
                break;
            }
            case 2: //交换模块（两边同时删除和插入） o(1)
            {
                int id_1 = rand()%tree_b_num;
                int id_2 = rand()%tree_b_num;
                while(id_2 == id_1)
                    id_2 = rand()%tree_b_num;
                //cout<<"swap "<<id_1<<" and "<<id_2<<endl;
                SwapBlock(id_1, id_2);
                break;
            }
        }
    }*/
    /*void SwapBlock(int id_1, int id_2)  //o(1)
    {
        Block& block_1 = blocks[id_1];      //记得引用！不然是临时变量，修改完了没用！
        Block& block_2 = blocks[id_2];
        swap(block_1.left, block_2.left);
        swap(block_1.right, block_2.right);
        swap(block_1.parent, block_2.parent);
        swap(block_1.is_from_left, block_2.is_from_left);
        //两节点相邻时，更正错误信息
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
        //更新父节点和子节点指向信息
        if(block_1.parent != -1)
        {
            if(block_1.is_from_left)
                blocks[block_1.parent].left = id_1;
            else
                blocks[block_1.parent].right = id_1;
        }
        if(block_1.left != -1)
            blocks[block_1.left].parent = id_1;
        if(block_1.right != -1)
            blocks[block_1.right].parent = id_1;
        if(block_2.parent != -1)
        {
            if(block_2.is_from_left)
                blocks[block_2.parent].left = id_2;
            else
                blocks[block_2.parent].right = id_2;
        }
        if(block_2.left != -1)
            blocks[block_2.left].parent = id_2;
        if(block_2.right != -1)
            blocks[block_2.right].parent = id_2;
        if(id_1 == root)
            root = id_2;
        else if(id_2 == root)
            root = id_1;
    }*/
    /*void MoveBlock(int from, int to) //原单层移动模块函数，o(h)
    {
        //delete, 为了尽可能保持结点相对结构，逐层向下替换删除，\
        左右结点选一个替代父结点，递归向下直至叶子结点（左右结点均为空）
        Block& del_node = blocks[from];
        while(del_node.left!=-1 || del_node.right!=-1)  //！！原出错语句为while(del_node.left!=-1 || del_node.left!=-1)
        {
            //cout<<"1"<<endl;
            if(del_node.left!=-1 && del_node.right!=-1)
            {
                if(rand()%2)
                {
                    SwapBlock(from, del_node.left);
                }
                else
                    SwapBlock(from, del_node.right);
            }
            else if(del_node.left!=-1)
                SwapBlock(from, del_node.left);
            else
                SwapBlock(from, del_node.right);
        }
        if(del_node.is_from_left)
        {
            blocks[del_node.parent].left = -1;
        }
        else
        {
            blocks[del_node.parent].right = -1;
        }

        //insert，插入到目标结点to的左结点或右节点，尽可能保持结构
        if(rand()%2)
        {
            blocks[from].is_from_left = true;
            blocks[from].parent = to;
            blocks[from].left = blocks[to].left;
            blocks[to].left = from;
            if(blocks[from].left != -1)
                blocks[blocks[from].left].parent = from;
        }
        else
        {
            blocks[from].is_from_left = false;
            blocks[from].parent = to;
            blocks[from].right = blocks[to].right;
            blocks[to].right = from;
            if(blocks[from].right != -1)
                blocks[blocks[from].right].parent = from;
        }

    }*/

    const COORD_TYPE ExceedOutlineArea(int layer)   //超出轮廓面积代价，等于超出的轮廓面积（宏观）+超出轮廓的模块面积（细化）
    {
        bool is_debug = false;
        assert(layer < layer_size);
        COORD_TYPE exceed_outline_width = width_[layer] - outline_width_;
        COORD_TYPE exceed_outline_height = height_[layer] - outline_height_;
        COORD_TYPE exceed_outline_area = 0;
        if(exceed_outline_width > 0)
        {
            if(exceed_outline_height > 0)  //宽高都超出
            {
                exceed_outline_area = exceed_outline_width * outline_height_ +  exceed_outline_height * outline_width_ + exceed_outline_width * exceed_outline_height;
                assert(width_[layer] * height_[layer] - outline_width_ * outline_height_ == exceed_outline_area);
            }
            else    //只有宽超出，超出部分*轮廓高
                exceed_outline_area = exceed_outline_width * outline_height_;
        }
        else if(exceed_outline_height > 0) //只有高超出，超出部分*轮廓宽
        {
            exceed_outline_area = exceed_outline_height * outline_width_;
        }
        else    //都不超出
        {
            exceed_outline_area = 0;
            assert(exceed_outline_blocks_area[layer] == 0);
        }
        if(is_debug)
        {
            cout << "layer: " << layer << ", exceed_outline_area: " << exceed_outline_area
            << ", exceed_outline_blocks_area[layer]: " << exceed_outline_blocks_area[layer] <<endl;
        }
        //assert(exceed_outline_blocks_area[layer] <= exceed_outline_area);
        return exceed_outline_area + exceed_outline_blocks_area[layer];
    }
    const COORD_TYPE Area(int layer)
    {
        /*long long area = (long long)width_ * height_;
        assert(area< 2000000000);*/
        assert(layer < layer_size);
        return width_[layer] * height_[layer];
    }
    const COORD_TYPE BlocksArea(int layer)
    {
        assert(layer < layer_size);
        return blocks_area[layer];
    }
    const double FillingRate(int layer) //填充率，模块面积/围成模块轮廓的面积
    {
            return tree_b_num[layer] == 0 ? 1: (double)blocks_area[layer] / Area(layer);
    }
    const double UtilizationRate(int layer) //利用率，模块面积/固定轮廓（芯片）面积
    {
        return tree_b_num[layer] == 0 ? 0: (double)blocks_area[layer] / (outline_width_*outline_height_);
    }
    /*未使用，原计算单树的半周长线长
    double WireLength()    //半周长 max|xi - xj| + max|yi - yj|   o(net)
    {
        double sum_half_perimeter = 0;
        for(auto net: nets)
        {
            double x_max = INT_MIN, x_min = INT_MAX;
            double y_max = INT_MIN, y_min = INT_MAX;
            for(auto i: net)
            {
                double x_center = (*blocks)[i].x + blocks[i].width/2.0;
                double y_center = blocks[i].y + blocks[i].height/2.0;
                if(x_center < x_min)
                    x_min = x_center;
                if(x_center > x_max)
                    x_max = x_center;
                if(y_center < y_min)
                    y_min = y_center;
                if(y_center > y_max)
                    y_max = y_center;
            }
            //cout<<"x_max: "<<x_max<<", x_min: "<<x_min<<", y_max: "<<y_max<<", y_min: "<<y_min<<endl;
            //cout<<x_max - x_min + y_max - y_min<<endl;
            sum_half_perimeter += (x_max - x_min + y_max - y_min);
        }
        return sum_half_perimeter;
    }*/
    void initial_tree_struct()  //初始化树结构
    {
        bool is_debug = false;
        //1. 每层平均放置若干block节点（以完全二叉树生成节点）
        int ave_tree_blocksnum = blocks.size() / layer_size;
        if(is_debug)
            cout << "initial_tree_struct() ave_tree_blocksnum: " << ave_tree_blocksnum << endl;
        //1.x 特殊情况
        if(ave_tree_blocksnum == 0) //树比blocks多，前b_num个树每个放一个
        {
            cerr << "wait for coding." << endl;
        }
        if(is_debug)
            cout << "initial_tree_struct() 1 is over" << endl;
        //1.1 正常情况，前layyer_size-1个树平均放
        for(int it=0; it < layer_size - 1; it++)
        {
            initial_blocks(it*ave_tree_blocksnum, it*ave_tree_blocksnum+ave_tree_blocksnum, it);
        }
        if(is_debug)
            cout << "initial_tree_struct() 1.1 is over" << endl;
        //1.2 最后一个树放剩下所有blocks
        initial_blocks((layer_size - 1) * ave_tree_blocksnum, blocks.size(), layer_size - 1);
        if(is_debug)
            cout << "initial_tree_struct() is over" << endl;
    }
    double use_ratio(int layer) //第i层当前的利用率
    {
        assert(layer >= 0 && layer < layer_size);
        return (double) blocks_area[layer] / (outline_width_ * outline_height_);
    }
    double use_area_limit(int layer) //第i层的模块面积限制，利用率*轮廓面积
    {
        assert(layer >= 0 && layer < layer_size);
        return outline_width_ * outline_height_ * userate_max[layer];
    }
    double use_ratio_add_block(int layer, Block &b) //第i层当前的利用率
    {
        assert(layer >= 0 && layer < layer_size);
        return (double) (blocks_area[layer] + b.area(layer)) / (outline_width_ * outline_height_);
    }
    static bool cmp(Block &b1, Block &b2)
    {
        return (double)b1.area(0)  / b1.area(1)  <  (double)b2.area(0)  / b2.area(1);
    }
    bool initial_tree_struct_with_useratio(vector<double> userate_max)  //在满足利用率的前提下初始化树结构是否成功，不成功的话还需要调整
    //7.29 按照模块01两层面积比例升序进行摆放，优先摆放在0层/1层比例小的在0层，1层同理
    {
        bool is_debug = false;
        bool is_successful = true;
        if(layer_size == 2)
        {
            //todo 排序后需要更新nets的connected_blocks
            //sort(blocks.begin(), blocks.end(), cmp);
            if(is_debug)
            {
                for(int i=0; i<blocks.size(); i++)
                {
                    cout << blocks[i].name << ", area(0)/area(1): " << (double)blocks[i].area(0)  / blocks[i].area(1) << endl;
                }
            }
        }
        assert(userate_max.size() == layer_size);
        this->userate_max = userate_max;
        // 按排序放置模块，直到该层利用率满了就顺延，直到所有层放满或模块放完
        int cur_block_id = 0;   //当前放置的模块
        int cur_layer = 0;    //当前放置的层
        while(cur_block_id < blocks.size())
        {
            if(is_debug == true)
            {
                cout << "cur_block_id: " << cur_block_id << endl;
            }
            Block &cur_block = blocks[cur_block_id];
            //1. 逐层放置，如果放不进去就下一层，最多尝试 layer_size 层
            for(cur_layer = 0; cur_layer < layer_size; cur_layer++)
            {
                if(is_debug == true)
                {
                    cout << "cur_layer: " << cur_layer  <<  ", blocks_area[cur_layer]: " << blocks_area[cur_layer]
                        << ", use_ratio_add_block(cur_layer, cur_block): " << use_ratio_add_block(cur_layer, cur_block) <<endl;
                }
                //1.1 当前层，能放进去，继续放下一个块
                if(use_ratio_add_block(cur_layer, cur_block) < userate_max[cur_layer])
                {
                    InsertRoot(cur_block_id, cur_layer);
                    blocks_area[cur_layer] += cur_block.area(cur_layer);
                    break;
                }
                //1.2 放不进去，尝试下一层
                else
                {
                    //cur_layer = (cur_layer + 1) % layer_size;
                }
            }
            //1.3 尝试layer_size层都没放进去，那就放当前层（或随机选一层）放，并将算法标记为失败
            if(cur_layer == layer_size)
            {
                is_successful = false;
                InsertRoot(cur_block_id, cur_layer);
                blocks_area[cur_layer] += cur_block.area(cur_layer);
                cur_layer = (cur_layer + 1) % layer_size;
            }
            cur_block_id ++;
        }
        return is_successful;
    }
    /*7.29之前旧的方法，挨层放直到放不下，case4失败
    bool initial_tree_struct_with_useratio(vector<double> userate_max)  //在满足利用率的前提下初始化树结构是否成功，不成功的话还需要调整
    {
        bool is_debug = false;
        bool is_successful = true;
        assert(userate_max.size() == layer_size);
        this->userate_max = userate_max;
        // 按顺序将每个模块放置到每一层，直到该层利用率满了就顺延，直到所有层放满或模块放完
        int cur_block_id = 0;   //当前放置的模块
        int cur_layer = 0;    //当前放置的层
        while(cur_block_id < blocks.size())
        {
            if(is_debug == true)
            {
                cout << "cur_block_id: " << cur_block_id << endl;
            }
            Block &cur_block = blocks[cur_block_id];
            int fail_times;
            //1. 逐层放置，如果放不进去就下一层，最多尝试 layer_size 层
            for(fail_times = 0; fail_times < layer_size; fail_times++)
            {
                if(is_debug == true)
                {
                    cout << "fail_times: " << fail_times <<", cur_layer: " << cur_layer << endl;
                    cout << "blocks_area[cur_layer]: " << blocks_area[cur_layer]
                         << ", use_ratio_add_block(cur_layer, cur_block): " << use_ratio_add_block(cur_layer, cur_block) <<endl;
                }
                //1.1 当前层，能放进去，循环停止
                if(use_ratio_add_block(cur_layer, cur_block) < userate_max[cur_layer])
                {
                    InsertRoot(cur_block_id, cur_layer);
                    blocks_area[cur_layer] += cur_block.area(cur_layer);
                    cur_layer = (cur_layer + 1) % layer_size;
                    break;
                }
                    //1.2 放不进去，下一层
                else
                {
                    cur_layer = (cur_layer + 1) % layer_size;
                }
            }
            //1.3 尝试layer_size层都没放进去，那就放当前层（或随机选一层）放，并将算法标记为失败
            if(fail_times == layer_size)
            {
                is_successful = false;
                InsertRoot(cur_block_id, cur_layer);
                blocks_area[cur_layer] += cur_block.area(cur_layer);
                cur_layer = (cur_layer + 1) % layer_size;
            }
            cur_block_id ++;
        }
        return is_successful;
    }*/
    void initial_blocks(int left, int right, int layer)    //将blocks[left] - [right-1]之间的节点初始化到树layer中
    {
        root[layer] = left;
        blocks[root[layer]].parent = blocks[root[layer]].left = blocks[root[layer]].right = -1;
        blocks[root[layer]].is_from_left = false;
        blocks[root[layer]].layer = layer;
        tree_b_num[layer] = right - left;
        //1, 建立完全二叉树结构
        for(int ib=0; ib < tree_b_num[layer]; ib++)    //ib为父节点，实际遍历不到tree_b_num，为了方便这么写的
        {
            if(2*ib+1 < tree_b_num[layer]) //左孩子2*ib+1
            {
                blocks[ib+left].left = 2*ib+1 + left;
                blocks[2*ib+1+left].parent = ib + left;
                blocks[2*ib+1+left].is_from_left = true;
                blocks[2*ib+1+left].layer = layer;
            }
            if(2*ib+2 < tree_b_num[layer]) //右孩子2*ib+2
            {
                blocks[ib+left].right = 2*ib+2 + left;
                blocks[2*ib+2+left].parent = ib + left;
                blocks[2*ib+2+left].layer = layer;
            }
        }

    }
    void OutputBlocks(ostream &out)
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
           <<"exceed_outline_area,"
           <<"left,"
           <<"right,"
           <<"parent,"
           <<"is_from_left,"
           <<endl;
        for(int i=0; i<blocks.size(); i++)
            out << blocks[i].name << ","
                << blocks[i].x << ","
                << blocks[i].y << ","
                << blocks[i].layer << ","
                << blocks[i].rotated_angle << ","
                << blocks[i].get_width() << ","
                << blocks[i].get_height() << ","
                << blocks[i].area() << ","
                << blocks[i].exceed_outline_area << ","
                << blocks[i].left << ","
                << blocks[i].right << ","
                << blocks[i].parent << ","
                << blocks[i].is_from_left << ","
                <<endl;
        // "Wirelength: " << TotalWireLength(b) << endl;
    }
    void OutputLayerInfo(ostream &out)
    {
        out<<"layer(tree) info:"<<endl;
        for(int i=0; i < layer_size; i++)
        {
            out<<"layer: "<<i<<" ,width: "<<width_[i]<<" ,height: "<<height_[i]<<" ,area: "<<Area(i) <<" ,blocks_area: "<<BlocksArea(i)
               << ", filling_rate: " << FillingRate(i) << ", utilizatio_rate: " << UtilizationRate(i) << endl;
        }
    }

//    void Initialization()   //未使用，原单树时候的初始化
//    {
//        root = 0;
//        blocks[root].parent = blocks[root].left = blocks[root].right = -1;
//        blocks[root].is_from_left = false;
//        //1, 建立完全二叉树结构
//        for(int ib=0; ib < tree_b_num; ib++)
//        {
//            if(2*ib+1 < tree_b_num) //左孩子
//            {
//                blocks[ib].left = 2*ib+1;
//                blocks[2*ib+1].parent = ib;
//                blocks[2*ib+1].is_from_left = true;
//            }
//            if(2*ib+2 < tree_b_num) //右孩子
//            {
//                blocks[ib].right = 2*ib+2;
//                blocks[2*ib+2].parent = ib;
//            }
//        }
//        /*测试初始化
//        Pack();
//        OutputBlocks(std::cout);*/
//        /*2, dfs初始化坐标//和等高线contour
//        stack<int> s;
//        blocks_cur[root].x = 0;
//        blocks_cur[root].y = 0;
//        int cur = root;
//        //访问root
//        //cout<<cur<<": l"<<endl;
//        while(blocks_cur[cur].left != -1) //访问左孩子节点，右子树入栈
//        {
//            if(blocks_cur[cur].right != -1)
//                s.push(blocks_cur[cur].right);
//            cur = blocks_cur[cur].left;
//            //访问左孩子节点;
//            //cout<<cur<<": l"<<endl;
//        }
//        //访问其他结点
//        while(!s.empty())
//        {
//            cur = s.top();
//            s.pop();
//            //访问右孩子节点
//            //cout<<cur<<": r"<<endl;
//            while(blocks_cur[cur].left != -1) //访问左孩子节点，右子树入栈
//            {
//                if(blocks_cur[cur].right != -1)
//                    s.push(blocks_cur[cur].right);
//                cur = blocks_cur[cur].left;
//                //访问左孩子节点;
//                //cout<<cur<<": l"<<endl;
//            }
//        }*/
//    }

};



