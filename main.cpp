
#include "Solver.cpp"

/*void test_and_record()
{
    //批量测试
    double ave_area = 0;
    double ave_wirelength = 0;
    double ave_time = 0;
    double alpha = 0;
    string example = "xerox";
    string algo = "ConvSA";
    string blockpath = ".\\MCNC\\HARD\\" + example + ".blocks_cur";
    string netpath = ".\\MCNC\\HARD\\" + example + ".nets";
    //统计记录各次结果*
    string result_file = ".\\result_" + algo + "\\statistic\\" + example + "_alpha=" + to_string(alpha) + ".txt";
    ofstream fout(result_file, ios::app|ios::out);  //追加写
    if(!fout)
    {
        cout<<"output error in file: "<<result_file<<endl;
        exit(-1);
    }
    fout<<"seed\t"<<"area\t"<<"wire_length\t"<<"iter\t"<<"time\t"<<endl;
    //随机数种子列表
    vector<int> seed_list;   //1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20
    for(int i=101; i<=120; i++)
    {
        seed_list.push_back(i);
    }
    int best_seed;
    double best_cost = 1e10;
    for(int seed: seed_list)
    {
        srand(seed);
        string output_file = ".\\result_" + algo + "\\" + example + "_alpha=" + to_string(alpha) \
                             + "_seed=" + to_string(seed) + ".txt";   //".\\result_SA\\apte_alpha=0.500000_seed=2"
        FloorPlan floorplan;
        floorplan.solve(alpha, blockpath, netpath, output_file);
        ave_area += floorplan.best_tree.Area();
        ave_wirelength += floorplan.best_tree.WireLength();
        ave_time += floorplan.Time();
        fout<<seed<<"\t"<<floorplan.best_tree.Area()/1e6<<"\t"<<floorplan.best_tree.WireLength()/1e3<<"\t\t"\
            <<floorplan.iter<<"\t"<<floorplan.Time()<<"\t"<<endl;
        if(floorplan.best_cost < best_cost)
        {
            best_seed = seed;
            best_cost = floorplan.best_cost;
        }
    }
    fout<<"ave_area: "<<ave_area/seed_list.size()/1e6<<endl;
    fout<<"ave_wirelength: "<<ave_wirelength/seed_list.size()/1e3<<endl;
    fout<<"ave_time: "<<ave_time/seed_list.size()<<endl<<endl;
    cout<<"ave_area: "<<ave_area/seed_list.size()/1e6<<endl;
    cout<<"ave_wirelength: "<<ave_wirelength/seed_list.size()/1e3<<endl;
    cout<<"ave_time: "<<ave_time/seed_list.size()<<endl<<endl;
    cout<<"best_seed: "<<best_seed<<", best_cost: "<<best_cost<<endl;
}
*/

int main()  //(double alpha, string blockpath, string netpath, string output_file)
{
//    int x0, y0, x1, y1;
//    int dum; // ignore parameter
//    char dummy;; // ignore parameter
//    cin >> dum >> dummy >> x0 >> dummy >> y0 >> dummy >> dum >> dummy >> dum >> dummy >> x1 >> dummy >> y1 >> dummy >> dum >> dummy >> dum;
//    cout << x0 << x1 << y0 << y1 << endl;
//    return 0;

    string path = "C:\\Users\\11367\\Desktop\\algorithm\\floorplanning_iccad2023_btree\\Deploy\\Instance\\";
    string instance = "ICCAD2023_floorplanning_case2";
    unsigned int random_seed = 1;
    float time_limit = 10;
    Config cfg(path, instance, random_seed ,time_limit);
    Solver s(cfg);
    s.run();

//    /*tree结构和操作测试*/
//    BStarTree tree;
//    tree.b_num = 6;
//    tree.blocks_cur = {Block(1,1),Block(2,2),Block(2,1),Block(1,1),Block(1,1),Block(3,2)};
//    nets = {{0,1,3}, {2,4,5}, {0,1,2,3,4,5}};
//    tree.Initialization();
//    tree.Pack();
//    tree.OutputBlocks(std::cout);
//    for(int i=0; i<100; i++)
//    {
//        tree.Perturb();
//        tree.Pack();
//        tree.OutputBlocks(std::cout);
//        //assert(tree.pack_num == tree.b_num);
//    }
//    /*旋转模块测试*/
//    tree.RotateBlock(2);
//    tree.Pack();
//    tree.OutputBlocks(std::out);
//    /*交换模块测试*/
//    tree.SwapBlock(1,0);
//    tree.Pack();
//    tree.OutputBlocks(std::out);*/
//    /*移动模块测试*/
//    tree.MoveBlock(0,1);
//    tree.Pack();
//    tree.OutputBlocks(std::out);
//    cout<<tree.root<<endl;
//    /**/
//    tree.ReadFromFile(blockpath, netpath);
//    //tree.TestBlocks_name_id();
//    tree.Initialization();
//    tree.Pack();
//    tree.OutputBlocks(std::out);
//    cout<<tree.Area()<<endl<<tree.WireLength()<<endl;;


    return 0;
}
