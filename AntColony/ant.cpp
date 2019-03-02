#include "ant.h"

CAnt::~CAnt()
{
	//�ͷ��ڴ�
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

void CAnt::deleteData()    //ɾ�������ڴ�
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
	city_num = num;     //n����ֵ���ⲿ�õ�
	confidence = data;
}

void CAnt::initData()
{
	ant_num = 50;
	ant_alpha = 1.0;   //��ʾ��Ϣ����Ҫ�̶ȵĲ���  ��  1      
	ant_beta = 5.0;  // ��ʾ����ʽ������Ҫ�̶ȵĲ���  ��  5   
	ant_rho = 0.3;   //��Ϣ������ϵ��   ��  0.1                 
	NC_max = 100;  //����������
	Q = 100;      //��Ϣ������ǿ��ϵ��
	NC = 0;        //��������������¼��������

	ant_data = new double *[city_num];   //��ant_data������г�ʼ��   ����ռ�
	ant_eta = new double *[city_num];    //��ant_eta������г�ʼ��
	ant_Tau = new double *[city_num];
	ant_Tabu = new int *[ant_num];   //����Ϊ���ϵ�����������Ϊ���е�����
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

	Length_best = new double[NC_max]();   //һά����
	Length_ave = new double[NC_max]();

	//�����鸳��ֵ
	for (int i = 0; i < city_num; i++)
	{
		for (int j = 0; j < city_num; j++)
		{
			ant_data[i][j] = confidence[i*city_num + j];
			if (ant_data[i][j] - 0 < 0.0001)   //�����������ӣ�Ϊ�л����۵ĵ���
			{
				ant_eta[i][j] = 0;
			}
			else
			{
				ant_eta[i][j] = 1.0 / ant_data[i][j];    //�л����۵ĵ���
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

		Length_best[i] = 100000;    //�������һ���ϴ��������
		Length_ave[i] = 0;     //ÿһ��·�ߵ�ƽ������
	}
}

void CAnt::randPermCpp()    //����matlab�е�randperm���������ش�0-n-1�����˳�������
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
			city_list.push_back(val);   //����������������浽������
			//cout << val << "\t";
			rand_arr[val] = 1;
			count++;
		}
	}
	//cout << endl;
}

void CAnt::setAnt()   //2.��������
{
	int t = ceil(ant_num*1.0 / city_num);
	for (int i = 0; i < t; i++)
	{
		randPermCpp();
	}

	for (int i = 0; i < ant_num; i++)    //��m�����������ÿ�����Ϸŵ�ǰ������ĳ��������У�ÿ������һ������
	{
		ant_Tabu[i][0] = city_list[i];   //ʹ��city_list�е�ǰant_num��ֵ
	}
}

void CAnt::chooseNextCity()   //3.mֻ���ϰ����ʺ���ѡ����һ�����У���ɸ��Ե�����
{
	for (int j = 1; j < city_num; j++)   //���ϱ������ڵĳ��в��������ڣ���˴�1��ʼ   //��ÿ������
	{
		for (int i = 0; i < ant_num; i++)   //��ÿ������
		{
			vector<int> visited;   //��¼�ѷ��ʵĳ��У������ظ�����
			for (int k = 0; k < j; k++)
			{
				visited.push_back(ant_Tabu[i][k]);   //visited������j��Ԫ��
			}
			vector<int> wait_visit;     //��¼�����ʵĳ���
			vector<double> P_transfer;          //��ѡ����е�ѡ����ʷֲ�   ת�Ƹ���   pij
			for (int k = 0; k < city_num - j; k++)
			{
				wait_visit.push_back(0);
				P_transfer.push_back(0);
			}
			int wait_counter = 0;    //�����ʳ��еļ�����
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
			//�����ѡ���еĸ��ʷֲ�
			for (int k = 0; k < wait_visit.size(); k++)
			{
				P_transfer[k] = pow(ant_Tau[visited[j - 1]][wait_visit[k]], ant_alpha)*pow(ant_eta[visited[j - 1]][wait_visit[k]], ant_beta);  //ת�Ƹ��ʼ��㹫ʽ�ķ���
				sum_P += P_transfer[k];  //ת�Ƹ��ʼ��㹫ʽ�ķ�ĸ
			}
			for (int k = 0; k < wait_visit.size(); k++)
			{
				P_transfer[k] = P_transfer[k] * 1.0 / sum_P;     //ת�Ƹ���
			}

			//������ԭ��ѡȡ��һ������
			vector<double> Pcum;
			Pcum.push_back(P_transfer[0]);   //Pcum�е�Ԫ��ΪP�е��ۼӣ���m��Ԫ��ΪP��ǰm��Ԫ�صĺ�
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
					select_city.push_back(k);    //Ѱ�ұ�������������
				}
			}
			int to_visit = wait_visit[select_city[0]];
			ant_Tabu[i][j] = to_visit;
		}
	}
}

double CAnt::getMinimum(vector<double> num_list, int &pos)    //��һ��vector�е���Сֵ
{
	double least_value = 100000.0;
	for (int i = 0; i < num_list.size(); i++)
	{
		if (num_list[i] < least_value)
		{
			least_value = num_list[i];        //��¼��С��ֵ
			pos = i;         //��¼��Сֵ��λ��
		}
	}
	return least_value;
}

double CAnt::getMinimum(double *num_list, int &pos)    //��һ�������е���Сֵ
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

double CAnt::getMean(vector<double> num_list)   //��ȡһ����������������ƽ��ֵ
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

void CAnt::recordRoute()   //4.��¼���ε������·��
{
	Length.clear();    //�������
	for (int i = 0; i < ant_num; i++)
	{
		Length.push_back(0);    //��L���г�ʼ��  Length=zeros(m,1); 
	}
	for (int i = 0; i < ant_num; i++)
	{
		vector<int> Route;  // Route=Tabu(i,:);   ���ε�·��
		for (int j = 0; j < city_num; j++)
		{
			Route.push_back(ant_Tabu[i][j]);
		}
		for (int j = 0; j < city_num - 1; j++)
		{
			Length[i] = Length[i] + ant_data[Route[j]][Route[j + 1]];     //�ۼ�·���Ĵ���
		}
		Length[i] = Length[i] + ant_data[Route[0]][Route[city_num - 1]];   //����ʼ�ͽ�����·���ĳ����ۼӵ��ܳ�����
	}
	int pos = 0;
	Length_best[NC] = getMinimum(Length, pos);   //pos��Ϊ���ã���ʽ�޸�   ��ȡ������������С��·��
	for (int j = 0; j < city_num; j++)
	{
		Route_best[NC][j] = ant_Tabu[pos][j];
	}
	Length_ave[NC] = getMean(Length);
	//NC = NC + 1;
}

void CAnt::updatePheromone()   //5.������Ϣ��
{
	double **Delta_Tau;   //��_tau  ��¼�˴�ѭ������Ϣ������
	Delta_Tau = new double *[city_num];  //��ʼ������
	for (int i = 0; i < city_num; i++)
	{
		Delta_Tau[i] = new double[city_num]();
	}

	for (int i = 0; i < city_num; i++)   //��ʼ����������Ԫ����Ϊ0
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
			Delta_Tau[ant_Tabu[i][j]][ant_Tabu[i][j + 1]] += Q*1.0 / Length[i];   //��ѭ����·��i,j֮�����Ϣ������
		}
		Delta_Tau[ant_Tabu[i][city_num - 1]][ant_Tabu[i][1]] += Q*1.0 / Length[i];
	}

	for (int i = 0; i < city_num; i++)   //��Ϣ�ػӷ���������Ϣ��
	{
		for (int j = 0; j < city_num; j++)
		{
			ant_Tau[i][j] = (1 - ant_rho)*ant_Tau[i][j] + Delta_Tau[i][j];    //(1-��)t[i,j]+��_t[i,j]  ��Ϣ�ص������Ҫ�̶�
		}
	}

	for (int i = 0; i < ant_num; i++)
	{
		for (int j = 0; j < city_num; j++)
		{
			ant_Tabu[i][j] = 0;    //����ѭ����¼��·������
		}
	}

	for (int i = 0; i < city_num; i++)   //ɾ������
	{
		delete[] Delta_Tau[i];
	}
	delete[] Delta_Tau;
}

void CAnt::computeRoute(vector<int> &route)  //����·��
{
	srand((unsigned)time(NULL));
	initData();   //1.��������г�ʼ��
	while (NC < NC_max)
	{
		cout << "NC=" << NC << "\t";
		setAnt();   //2.��������
		chooseNextCity();  //3.���ϰ��ո��ʺ���ѡ����һ������
		if (NC >= 1)   //�����ǵ�һ�ε���
		{
			for (int j = 0; j < city_num; j++)
			{
				ant_Tabu[0][j] = Route_best[NC - 1][j];
			}
		}
		recordRoute();   //4.��¼���ε������·��
		cout << "length: " << Length_best[NC] << endl;
		NC = NC + 1;
		updatePheromone();   //5.������Ϣ��

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