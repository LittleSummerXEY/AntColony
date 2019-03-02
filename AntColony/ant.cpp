#include "ant.h"

CAnt::~CAnt()
{
	//释放内存
	confidence.clear();
	city_list.clear();
	Length.clear();

	for (int i = 0; i < city_num; i++)
	{
		delete[] ant_data[i];
		delete[] ant_eta[i];
		delete[] ant_Tau[i];
		delete[] Route_best[i];
	}
	for (int i = 0; i < ant_num; i++)
	{
		delete[] ant_Tabu[i];
	}
	delete[] ant_data;
	delete[] ant_eta;
	delete[] ant_Tau;
	delete[] ant_Tabu;
	delete[] Route_best;

	delete[] Length_best;
	delete[] Length_ave;
}

void CAnt::deleteData()    //删除数组内存
{
	for (int i = 0; i < city_num; i++)
	{
		delete[] ant_data[i];
		delete[] ant_eta[i];
		delete[] ant_Tau[i];
		delete[] Route_best[i];
	}
	for (int i = 0; i < ant_num; i++)
	{
		delete[] ant_Tabu[i];
	}
	delete[] ant_data;
	delete[] ant_eta;
	delete[] ant_Tau;
	delete[] ant_Tabu;
	delete[] Route_best;

	delete[] Length_best;
	delete[] Length_ave;
}

void CAnt::getData(int num, vector<double> &data)
{
	city_num = num;     //n的数值从外部得到
	confidence = data;
}

void CAnt::initData()
{
	ant_num = 50;
	ant_alpha = 1.0;   //表示信息素重要程度的参数  α  1      
	ant_beta = 5.0;  // 表示启发式因子重要程度的参数  β  5   
	ant_rho = 0.3;   //信息素蒸发系数   ρ  0.1                 
	NC_max = 100;  //最大迭代次数
	Q = 100;      //信息素增加强度系数
	NC = 0;        //迭代计数器，记录迭代次数

	ant_data = new double *[city_num];   //对ant_data数组进行初始化   申请空间
	ant_eta = new double *[city_num];    //对ant_eta数组进行初始化
	ant_Tau = new double *[city_num];
	ant_Tabu = new int *[ant_num];   //行数为蚂蚁的数量，列数为城市的数量
	for (int i = 0; i < city_num; i++)
	{
		ant_data[i] = new double[city_num]();   //n*n
		ant_eta[i] = new double[city_num]();  //n*n
		ant_Tau[i] = new double[city_num]();  //n*n
	}
	for (int i = 0; i < ant_num; i++)
	{
		ant_Tabu[i] = new int[city_num]();  //m*n
	}

	Route_best = new int *[NC_max];
	for (int i = 0; i < NC_max; i++)
	{
		Route_best[i] = new int[city_num]();     //NC_max*n
	}

	Length_best = new double[NC_max]();   //一维数组
	Length_ave = new double[NC_max]();

	//对数组赋初值
	for (int i = 0; i < city_num; i++)
	{
		for (int j = 0; j < city_num; j++)
		{
			ant_data[i][j] = confidence[i*city_num + j];
			if (ant_data[i][j] - 0 < 0.0001)   //计算启发因子，为切换代价的倒数
			{
				ant_eta[i][j] = 0;
			}
			else
			{
				ant_eta[i][j] = 1.0 / ant_data[i][j];    //切换代价的倒数
			}
			//cout << ant_data[i][j] << "\t";
			ant_Tau[i][j] = 1;
		}
		//cout << endl;
	}

	for (int i = 0; i < ant_num; i++)
	{
		for (int j = 0; j < city_num; j++)
		{
			ant_Tabu[i][j] = 0;
		}
	}

	for (int i = 0; i < NC_max; i++)
	{
		for (int j = 0; j < city_num; j++)
		{
			Route_best[i][j] = 0;
		}

		Length_best[i] = 100000;    //无穷大，用一个较大的数代替
		Length_ave[i] = 0;     //每一代路线的平均长度
	}
}

void CAnt::randPermCpp()    //类似matlab中的randperm函数，返回从0-n-1的随机顺序的整数
{
	vector<int> rand_arr;
	int count = 0;
	for (int i = 0; i < city_num; i++)
	{
		rand_arr.push_back(0);
	}
	while (count < city_num)
	{
		int val = rand() % city_num;
		if (!rand_arr[val])
		{
			city_list.push_back(val);   //将产生的随机数保存到序列中
			//cout << val << "\t";
			rand_arr[val] = 1;
			count++;
		}
	}
	//cout << endl;
}

void CAnt::setAnt()   //2.放置蚂蚁
{
	int t = ceil(ant_num*1.0 / city_num);
	for (int i = 0; i < t; i++)
	{
		randPermCpp();
	}

	for (int i = 0; i < ant_num; i++)    //将m个蚂蚁随机，每个蚂蚁放到前面产生的城市序列中，每个蚂蚁一个城市
	{
		ant_Tabu[i][0] = city_list[i];   //使用city_list中的前ant_num个值
	}
}

void CAnt::chooseNextCity()   //3.m只蚂蚁按概率函数选择下一座城市，完成各自的周游
{
	for (int j = 1; j < city_num; j++)   //蚂蚁本身所在的城市不计算在内，因此从1开始   //在每个城市
	{
		for (int i = 0; i < ant_num; i++)   //对每个蚂蚁
		{
			vector<int> visited;   //记录已访问的城市，避免重复访问
			for (int k = 0; k < j; k++)
			{
				visited.push_back(ant_Tabu[i][k]);   //visited共包含j个元素
			}
			vector<int> wait_visit;     //记录待访问的城市
			vector<double> P_transfer;          //待选择城市的选择概率分布   转移概率   pij
			for (int k = 0; k < city_num - j; k++)
			{
				wait_visit.push_back(0);
				P_transfer.push_back(0);
			}
			int wait_counter = 0;    //待访问城市的计数器
			for (int k = 0; k < city_num; k++)
			{
				vector<int>::iterator it = find(visited.begin(), visited.end(), k);
				if (it == visited.end())
				{
					wait_visit[wait_counter] = k;
					wait_counter++;
				}
			}

			double sum_P = 0;
			//计算待选城市的概率分布
			for (int k = 0; k < wait_visit.size(); k++)
			{
				P_transfer[k] = pow(ant_Tau[visited[j - 1]][wait_visit[k]], ant_alpha)*pow(ant_eta[visited[j - 1]][wait_visit[k]], ant_beta);  //转移概率计算公式的分子
				sum_P += P_transfer[k];  //转移概率计算公式的分母
			}
			for (int k = 0; k < wait_visit.size(); k++)
			{
				P_transfer[k] = P_transfer[k] * 1.0 / sum_P;     //转移概率
			}

			//按概率原则选取下一个城市
			vector<double> Pcum;
			Pcum.push_back(P_transfer[0]);   //Pcum中的元素为P中的累加，第m个元素为P中前m个元素的和
			for (int k = 1; k < P_transfer.size(); k++)
			{
				double t = Pcum[k - 1] + P_transfer[k];
				Pcum.push_back(t);
			}
			vector<int> select_city; //Select=find(Pcum>=rand)
			double selece_rand = (rand() % 100) / 100.0;
			for (int k = 0; k < P_transfer.size(); k++)
			{
				if (Pcum[k] >= selece_rand)
				{
					select_city.push_back(k);    //寻找比随机数大的坐标
				}
			}
			int to_visit = wait_visit[select_city[0]];
			ant_Tabu[i][j] = to_visit;
		}
	}
}

double CAnt::getMinimum(vector<double> num_list, int &pos)    //求一个vector中的最小值
{
	double least_value = 100000.0;
	for (int i = 0; i < num_list.size(); i++)
	{
		if (num_list[i] < least_value)
		{
			least_value = num_list[i];        //记录最小的值
			pos = i;         //记录最小值的位置
		}
	}
	return least_value;
}

double CAnt::getMinimum(double *num_list, int &pos)    //求一个数组中的最小值
{
	double least_value = 100000.0;
	for (int i = 0; i < NC_max; i++)
	{
		if (num_list[i] < least_value)
		{
			least_value = num_list[i];
			pos = i;
		}
	}
	return least_value;
}

double CAnt::getMean(vector<double> num_list)   //获取一个序列中左右数的平均值
{
	double sum = 0.0;
	double value_mean;
	int list_size = num_list.size();
	for (int i = 0; i < list_size; i++)
	{
		sum = sum + num_list[i];
	}
	if (abs(list_size - 0) < 0.0000001)
	{
		return 0;
	}
	value_mean = sum*1.0 / list_size;
	return value_mean;
}

void CAnt::recordRoute()   //4.记录本次迭代最佳路径
{
	Length.clear();    //清空容器
	for (int i = 0; i < ant_num; i++)
	{
		Length.push_back(0);    //对L进行初始化  Length=zeros(m,1); 
	}
	for (int i = 0; i < ant_num; i++)
	{
		vector<int> Route;  // Route=Tabu(i,:);   本次的路径
		for (int j = 0; j < city_num; j++)
		{
			Route.push_back(ant_Tabu[i][j]);
		}
		for (int j = 0; j < city_num - 1; j++)
		{
			Length[i] = Length[i] + ant_data[Route[j]][Route[j + 1]];     //累加路径的代价
		}
		Length[i] = Length[i] + ant_data[Route[0]][Route[city_num - 1]];   //将开始和结束的路径的长度累加到总长度中
	}
	int pos = 0;
	Length_best[NC] = getMinimum(Length, pos);   //pos作为引用，隐式修改   获取所有蚂蚁中最小的路径
	for (int j = 0; j < city_num; j++)
	{
		Route_best[NC][j] = ant_Tabu[pos][j];
	}
	Length_ave[NC] = getMean(Length);
	//NC = NC + 1;
}

void CAnt::updatePheromone()   //5.更新信息素
{
	double **Delta_Tau;   //δ_tau  记录此次循环的信息素增量
	Delta_Tau = new double *[city_num];  //初始化数组
	for (int i = 0; i < city_num; i++)
	{
		Delta_Tau[i] = new double[city_num]();
	}

	for (int i = 0; i < city_num; i++)   //初始化，将所有元素设为0
	{
		for (int j = 0; j < city_num; j++)
		{
			Delta_Tau[i][j] = 0;
		}
	}

	for (int i = 0; i < ant_num; i++)
	{
		for (int j = 0; j < city_num - 1; j++)
		{
			Delta_Tau[ant_Tabu[i][j]][ant_Tabu[i][j + 1]] += Q*1.0 / Length[i];   //此循环在路径i,j之间的信息素增量
		}
		Delta_Tau[ant_Tabu[i][city_num - 1]][ant_Tabu[i][1]] += Q*1.0 / Length[i];
	}

	for (int i = 0; i < city_num; i++)   //信息素挥发，更新信息素
	{
		for (int j = 0; j < city_num; j++)
		{
			ant_Tau[i][j] = (1 - ant_rho)*ant_Tau[i][j] + Delta_Tau[i][j];    //(1-ρ)t[i,j]+δ_t[i,j]  信息素的相对重要程度
		}
	}

	for (int i = 0; i < ant_num; i++)
	{
		for (int j = 0; j < city_num; j++)
		{
			ant_Tabu[i][j] = 0;    //将此循环记录的路径清零
		}
	}

	for (int i = 0; i < city_num; i++)   //删除数组
	{
		delete[] Delta_Tau[i];
	}
	delete[] Delta_Tau;
}

void CAnt::computeRoute(vector<int> &route)  //计算路径
{
	srand((unsigned)time(NULL));
	initData();   //1.对数组进行初始化
	while (NC < NC_max)
	{
		cout << "NC=" << NC << "\t";
		setAnt();   //2.放置蚂蚁
		chooseNextCity();  //3.蚂蚁按照概率函数选择下一座城市
		if (NC >= 1)   //若不是第一次迭代
		{
			for (int j = 0; j < city_num; j++)
			{
				ant_Tabu[0][j] = Route_best[NC - 1][j];
			}
		}
		recordRoute();   //4.记录本次迭代最佳路径
		cout << "length: " << Length_best[NC] << endl;
		NC = NC + 1;
		updatePheromone();   //5.更新信息素

	}
	int pos = 0;
	double minimum = getMinimum(Length_best, pos);
	double shortest_length = Length_best[pos];
	cout << "shortest_length: " << shortest_length << endl;
	cout << "shortest_route:" << endl;

	for (int i = 0; i < city_num; i++)
	{
		route.push_back(Route_best[pos][i]);
		cout << Route_best[pos][i] + 1 << "\t";
	}
	cout << endl;
	deleteData();
}