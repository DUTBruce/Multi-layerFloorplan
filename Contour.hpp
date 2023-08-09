//
// Created by 11367 on 2023/6/4.
//

#ifndef SMARTFLOORPLAN_CONTOUR_HPP
#define SMARTFLOORPLAN_CONTOUR_HPP

class Contour   //
{
public:

    list<pair<COORD_TYPE,COORD_TYPE>> points;   //等高线的点为左闭右开区间
    //COORD_TYPE max_x = 0, max_y = 0;       //轮廓的最右和最高，即包围的最小矩形框
    void reset()
    {
        points.clear();
    }
    COORD_TYPE max_x()
    {
        if(points.size()==0)
            return 0;
        else
            return points.back().first;
    }
    COORD_TYPE max_y()
    {
        return find_max_y_between(0, max_x());
    }
    COORD_TYPE find_max_y_between(COORD_TYPE x1, COORD_TYPE x2)  // o(c) ≈ o(n ^ 1/2)
    {
        COORD_TYPE max_y = 0;
        auto iter = points.begin();
        while(iter != points.end() && iter->first <= x1) //找到最后一个<=x1的点
            iter++;
        if(iter == points.end())
            return 0;
        if(iter != points.begin())
            iter--;
        while(iter != points.end() && iter->first < x2)
        {
            if(iter->second > max_y)
                max_y = iter->second;
            iter++;
        }
        return max_y;
    }
    void Update(COORD_TYPE x1, COORD_TYPE x2, COORD_TYPE y)  //将x1--x2的等高线更新为高度y  o(c) ≈ o(n ^ 1/2)
    {
        /*if(x2 > max_x)
            max_x = x2;
        if(y > max_y)
            max_y = y;*/
        if(points.size() == 0)  //等高线为空时，直接添加到等高线中
        {
            assert(x1==0);      //此时一定是根节点，x1==0
            points.push_back({x1,y});
            points.push_back({x2,y});
        }
        //1. 删除原等高线在x1--x2之间的点
        auto iter = points.begin();
        while(iter != points.end() && iter->first < x1) //1.1 找到第一个>=x1的点
            iter++;
        COORD_TYPE  last_y0;  //最后（最右边）删除节点的y坐标，以在2.3中放置
        if(iter == points.begin())
        {
            last_y0 = iter->second;
        }
        else
        {
            iter--;
            last_y0 = iter -> second;
            iter++;
        }
        while(iter != points.end() && iter->first < x2) //1.2 逐个删除在[x1,x2)中的等高线结点
        {
            last_y0 = iter->second;
            iter = points.erase(iter);//删除后返回下一个迭代器
        }
        //2. 分删完后新等高线情况插入新的等高线结点，其中iter在删除完节点的后边位置，即待插入位置
        if(iter == points.end())    //2.1 新模块在等高线轮廓右边，直接加入
        {
            points.insert(iter, {x1,y});
            points.insert(iter, {x2,y});
        }
        else if(iter->first == x2)  //2.2 新模块右端刚好覆盖原来删除的等高线节点集
        {
            points.insert(iter, {x1,y});
        }
        else                        //2.3 新模块右端在原等高线节点之间，即有剩余未覆盖部分被截断成两段，{x1,y}和{x2,last_y0}
        {
            /*cout<<"..";
            Show();此时已经删除了结点信息，需提前保存last_y0*/
            points.insert(iter, {x1,y});
            points.insert(iter, {x2,last_y0});
        }

    }
    /*数组实现，一个想法：每个结点都维护一个contour成员，看是否更利于邻域操作
    vector<int> points_x;   //等高线的点为左闭右开区间
    vector<int> points_y;
    //static int max_x, max_y;       //轮廓的最右和最高，即包围的最小矩形框
    int find_max_y_between(int x1, int x2)
    {
        int max_y = 0;
        int iter = 0;
        while(points_x[iter] <= x1 && iter < points_x.size()) //找到最后一个<=x1的点的横坐标
            iter++;
        if(iter == points_x.size())
            return 0;
        iter--;
        while(points_x[iter] < x2 && iter < points_x.size())
        {
            if(points_y[iter] > max_y)
                max_y = points_y[iter];
            iter++;
        }
    }
    */
    void Show()
    {
        for(auto iter = points.begin(); iter != points.end(); iter++)
            cout << iter->first << "," << iter->second << "  ";
        cout<<endl;
    }

};

#endif //SMARTFLOORPLAN_CONTOUR_HPP
