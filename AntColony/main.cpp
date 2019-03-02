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
	infile.open(name);    //���ļ�
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
	unsigned int nSeed = (unsigned int)tm;   //�õ�ǰʱ����ʼ��������ӣ���ֹÿ�����еĽ������ͬ
	srand(nSeed);


	readFile(name); 
	num = sqrt(view_cost.size());
	cout << "num:" << num << endl;;
	ant.getData(num, view_cost);     //Ϊ���ϱ�����������
	ant.computeRoute(shortest_route);

	int y = 0;
	system("pause");
	return 0;
}