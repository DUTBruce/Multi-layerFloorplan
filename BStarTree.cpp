/*2023/5/30
 * 重构代码，改写为支持两层的B*tree布图规划
*/


#include "Block.cpp"
#include "Contour.hpp"

using namespace std;

class BStarTree      //一个tree即一个floorplan，记录了包含哪些模块以及模块的放置位置
{
public:
    vector<Block> *blocks;  //对Solver中blocks的指针（引用会出错）
    int root;           //根节点编号
    int root_x=0, root_y=0; //根节点坐标（默认是最贴着左下角(0,0)，后续也可以调整）
    int tree_layer;     //当前tree表示第几层
    int offset_x, offset_y;
    int tree_b_num;          //当前树的block数量
    Contour contour;    //到当前遍历结点的等高线，根据遍历顺序迭代更新（每次pack前需要reset）
    int width_, height_;       //布图轮廓的宽和高
    BStarTree(vector<Block>* b, int layer):blocks(b)
    {
        //blocks = b;//引用类型成员变量必须在构造函数初始化列表中进行初始化，而不能在构造函数体内进行赋值操作。
        tree_layer = layer;
    }
    void Perturb()      //o(h) 对当前树实行扰动
    {
        int perturb_move_size = 3;  //扰动操作的数量
        int random = rand() % perturb_move_size;
        switch(random)
        {
            case 0: //旋转模块 o(1)
            {
                int block_id = rand()%tree_b_num;
                //cout<<"rotate "<<block_id<<endl;
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
    }
    void RotateBlock(int id)
    {
        (*blocks)[id].rotate();
//        blocks[id].is_rotated = !blocks[id].is_rotated;
//        swap(blocks[id].width, blocks[id].height);
    }
    void DeleteBlock(int from)
    {
        //delete, 为了尽可能保持结点相对结构，逐层向下替换删除，\
        左右结点选一个替代父结点，递归向下直至叶子结点（左右结点均为空）
        Block& del_node = (*blocks)[from];
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
            (*blocks)[del_node.parent].left = -1;
        }
        else
        {
            (*blocks)[del_node.parent].right = -1;
        }
    }
    void InsertBlock(int from, int to)
    {
        //insert，插入到目标结点to的左结点或右节点，尽可能保持结构
        if(rand()%2)    //左
        {
            (*blocks)[from].is_from_left = true;
            (*blocks)[from].parent = to;
            (*blocks)[from].left = (*blocks)[to].left;
            (*blocks)[to].left = from;
            if((*blocks)[from].left != -1)
                (*blocks)[(*blocks)[from].left].parent = from;
        }
        else            //右
        {
            (*blocks)[from].is_from_left = false;
            (*blocks)[from].parent = to;
            (*blocks)[from].right = (*blocks)[to].right;
            (*blocks)[to].right = from;
            if((*blocks)[from].right != -1)
                (*blocks)[(*blocks)[from].right].parent = from;
        }
    }
    void MoveBlock(int from, int to) //o(h)
    {
        //delete, 为了尽可能保持结点相对结构，逐层向下替换删除，\
        左右结点选一个替代父结点，递归向下直至叶子结点（左右结点均为空）
        Block& del_node = (*blocks)[from];
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
            (*blocks)[del_node.parent].left = -1;
        }
        else
        {
            (*blocks)[del_node.parent].right = -1;
        }

        //insert，插入到目标结点to的左结点或右节点，尽可能保持结构
        if(rand()%2)
        {
            (*blocks)[from].is_from_left = true;
            (*blocks)[from].parent = to;
            (*blocks)[from].left = (*blocks)[to].left;
            (*blocks)[to].left = from;
            if((*blocks)[from].left != -1)
                (*blocks)[(*blocks)[from].left].parent = from;
        }
        else
        {
            (*blocks)[from].is_from_left = false;
            (*blocks)[from].parent = to;
            (*blocks)[from].right = (*blocks)[to].right;
            (*blocks)[to].right = from;
            if((*blocks)[from].right != -1)
                (*blocks)[(*blocks)[from].right].parent = from;
        }

    }
    void SwapBlock(int id_1, int id_2)  //o(1)
    {
        Block& block_1 = (*blocks)[id_1];      //记得引用！不然是临时变量，修改完了没用！
        Block& block_2 = (*blocks)[id_2];
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
                (*blocks)[block_1.parent].left = id_1;
            else
                (*blocks)[block_1.parent].right = id_1;
        }
        if(block_1.left != -1)
            (*blocks)[block_1.left].parent = id_1;
        if(block_1.right != -1)
            (*blocks)[block_1.right].parent = id_1;
        if(block_2.parent != -1)
        {
            if(block_2.is_from_left)
                (*blocks)[block_2.parent].left = id_2;
            else
                (*blocks)[block_2.parent].right = id_2;
        }
        if(block_2.left != -1)
            (*blocks)[block_2.left].parent = id_2;
        if(block_2.right != -1)
            (*blocks)[block_2.right].parent = id_2;
        if(id_1 == root)
            root = id_2;
        else if(id_2 == root)
            root = id_1;
    }
    const int Area()
    {
        /*long long area = (long long)width_ * height_;
        assert(area< 2000000000);*/
        return width_ * height_;
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
                double x_center = (*blocks)[i].x + (*blocks)[i].width/2.0;
                double y_center = (*blocks)[i].y + (*blocks)[i].height/2.0;
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

    void Pack()
    {
        contour.reset();
        assert((*blocks).size()!=0);
        //pack_num = 0;
        Pack(root);
        width_ = contour.max_x();
        height_ = contour.max_y();
    }
    void Pack(int b)    //根据树结构放置模块，无参数时默认从根节点开始
    {
        //pack_num++;
        //1.放置模块（即更新xy坐标）
        Block& block =  (*blocks)[b];
        cout << "block_id: " << b << ", block.layer: " << block.layer << ", tree_layer: " << tree_layer << endl;
        assert(block.layer == tree_layer);
        if(b == root)
        {
            block.x = root_x;
            block.y = root_y;
        }
        else
        {
            Block& parent = (*blocks)[block.parent];
            block.x = block.is_from_left? parent.x+parent.get_width(): parent.x;
            block.y = contour.find_max_y_between(block.x-root_x, block.x-root_x + block.get_width()) + root_y;   //o(n ^ 1/2)
        }
        //2.更新等高线、轮廓宽高、半周长等信息
        contour.Update(block.x - root_x, block.x - root_x + block.get_width(), block.y - root_y + block.get_height());
        //contour.Show();//测试等高线的正确性

        //3.迭代Pack下一个模块
        if(block.left != -1)
            Pack(block.left);
        if(block.right != -1)
            Pack(block.right);
    }
    void initial_blocks(int left, int right)    //将blocks[left] - [right-1]之间的节点初始化到树中
    {
        root = left;
        (*blocks)[root].parent = (*blocks)[root].left = (*blocks)[root].right = -1;
        (*blocks)[root].is_from_left = false;
        (*blocks)[root].layer = tree_layer;
        tree_b_num = right - left;
        //1, 建立完全二叉树结构
        for(int ib=0; ib < tree_b_num; ib++)    //ib为父节点，实际遍历不到tree_b_num，为了方便这么写的
        {
            if(2*ib+1 < tree_b_num) //左孩子2*ib+1
            {
                (*blocks)[ib+left].left = 2*ib+1 + left;
                (*blocks)[2*ib+1+left].parent = ib + left;
                (*blocks)[2*ib+1+left].is_from_left = true;
                (*blocks)[2*ib+1+left].layer = tree_layer;
            }
            if(2*ib+2 < tree_b_num) //右孩子2*ib+2
            {
                (*blocks)[ib+left].right = 2*ib+2 + left;
                (*blocks)[2*ib+2+left].parent = ib + left;
                (*blocks)[2*ib+2+left].layer = tree_layer;
            }
        }
    }
    void Initialization()   //未使用，原单树时候的初始化
    {
        root = 0;
        (*blocks)[root].parent = (*blocks)[root].left = (*blocks)[root].right = -1;
        (*blocks)[root].is_from_left = false;
        //1, 建立完全二叉树结构
        for(int ib=0; ib < tree_b_num; ib++)
        {
            if(2*ib+1 < tree_b_num) //左孩子
            {
                (*blocks)[ib].left = 2*ib+1;
                (*blocks)[2*ib+1].parent = ib;
                (*blocks)[2*ib+1].is_from_left = true;
            }
            if(2*ib+2 < tree_b_num) //右孩子
            {
                (*blocks)[ib].right = 2*ib+2;
                (*blocks)[2*ib+2].parent = ib;
            }
        }
        /*测试初始化
        Pack();
        OutputBlocks(std::cout);*/
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

};



