#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include "ant.h"
using namespace std;

int num;
vector<double> view_cost;

void readFile(string name)
{
	float message;
	ifstream infile;
	infile.open(name);    //打开文件
	if (infile.is_open())
	{
		while (!infile.eof())
		{
			infile >> message;
			view_cost.push_back(message);
		}
	}
	view_cost.pop_back();
	infile.close();
	cout << "number: " << view_cost.size() << endl;
}

int main()
{
	CAnt ant;
	vector<int> shortest_route;
	string name = "view_cost.txt";
	time_t tm;
	time(&tm);
	unsigned int nSeed = (unsigned int)tm;   //用当前时间点初始化随机种子，防止每次运行的结果都相同
	srand(nSeed);


	readFile(name); 
	num = sqrt(view_cost.size());
	cout << "num:" << num << endl;;
	ant.getData(num, view_cost);     //为蚂蚁变量传递数据
	ant.computeRoute(shortest_route);

	int y = 0;
	system("pause");
	return 0;
}