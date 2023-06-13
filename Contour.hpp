//
// Created by 11367 on 2023/6/4.
//

#ifndef SMARTFLOORPLAN_CONTOUR_HPP
#define SMARTFLOORPLAN_CONTOUR_HPP

class Contour   //
{
public:

    list<pair<int,int>> points;   //�ȸ��ߵĵ�Ϊ����ҿ�����
    //int max_x = 0, max_y = 0;       //���������Һ���ߣ�����Χ����С���ο�
    void reset()
    {
        points.clear();
    }
    int max_x()
    {
        if(points.size()==0)
            return 0;
        else
            return points.back().first;
    }
    int max_y()
    {
        return find_max_y_between(0, max_x());
    }
    int find_max_y_between(int x1, int x2)  // o(c) �� o(n ^ 1/2)
    {
        int max_y = 0;
        auto iter = points.begin();
        while(iter->first <= x1 && iter != points.end()) //�ҵ����һ��<=x1�ĵ�
            iter++;
        if(iter == points.end())
            return 0;
        iter--;
        while(iter->first < x2 && iter != points.end())
        {
            if(iter->second > max_y)
                max_y = iter->second;
            iter++;
        }
        return max_y;
    }
    void Update(int x1, int x2, int y)  //��x1--x2�ĵȸ��߸���Ϊ�߶�y  o(c) �� o(n ^ 1/2)
    {
        /*if(x2 > max_x)
            max_x = x2;
        if(y > max_y)
            max_y = y;*/
        if(points.size() == 0)
        {
            assert(x1==0);
            points.push_back({x1,y});
            points.push_back({x2,y});
        }
        auto iter = points.begin();
        while(iter->first < x1 && iter != points.end()) //�ҵ���һ��>=x1�ĵ�
            iter++;
        iter--;
        int last_y0 = iter->second;
        iter++;
        while(iter->first < x2 && iter != points.end())
        {
            last_y0 = iter->second;
            iter = points.erase(iter);//ɾ���󷵻���һ��������
        }
        //����������µĵȸ��߽��
        if(iter == points.end())
        {
            points.insert(iter, {x1,y});
            points.insert(iter, {x2,y});
        }
        else if(iter->first == x2)
        {
            points.insert(iter, {x1,y});
        }
        else    //iter->first > x2
        {
            /*cout<<"..";
            Show();��ʱ�Ѿ�ɾ���˽����Ϣ������ǰ����last_y0*/
            points.insert(iter, {x1,y});
            points.insert(iter, {x2,last_y0});
        }

    }
    /*����ʵ�֣�һ���뷨��ÿ����㶼ά��һ��contour��Ա�����Ƿ�������������
    vector<int> points_x;   //�ȸ��ߵĵ�Ϊ����ҿ�����
    vector<int> points_y;
    //static int max_x, max_y;       //���������Һ���ߣ�����Χ����С���ο�
    int find_max_y_between(int x1, int x2)
    {
        int max_y = 0;
        int iter = 0;
        while(points_x[iter] <= x1 && iter < points_x.size()) //�ҵ����һ��<=x1�ĵ�ĺ�����
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
