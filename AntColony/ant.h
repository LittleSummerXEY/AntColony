#pragma once
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <iostream>
using namespace std;

//����������
class CAnt
{
public:
	CAnt(void){};
	~CAnt(void);

	void getData(int num, vector<double> &data);     //1.��������г�ʼ��  city_numΪn, ant_numΪm
	void initData();     //1.��������г�ʼ��  city_numΪn, ant_numΪm
	void deleteData();    //ɾ�������ڴ�
	void randPermCpp();    //����matlab�е�randperm���������ش�0-n-1�����˳�������
	void setAnt();   //2.��������
	void chooseNextCity();   //3.mֻ���ϰ����ʺ���ѡ����һ�����У���ɸ��Ե�����
	double getMinimum(vector<double> num_list, int &pos);    //��һ��vector�е���Сֵ,����¼��λ��
	double getMinimum(double *num_list, int &pos);   //��һ�������е���Сֵ������¼��λ��
	double getMean(vector<double> num_list);   //��ȡһ����������������ƽ��ֵ
	void recordRoute();   //4.��¼���ε������·��
	void updatePheromone();   //5.������Ϣ��

	void computeRoute(vector<int> &route);    //����·��

private:

	int ant_num = 50;   //���ϸ���   m  50
	int city_num = 64;   //��������   n  64
	double ant_alpha = 1;   //��ʾ��Ϣ����Ҫ�̶ȵĲ���  ��  1            
	double ant_beta = 5;  // ��ʾ����ʽ������Ҫ�̶ȵĲ���  ��  5   
	double ant_rho = 0.1;   //��Ϣ������ϵ��   ��  0.1          
	int Q = 100;  //��Ϣ������ǿ��ϵ��
	int NC = 0;   //��������������¼��������
	int NC_max = 200;  //����������

	vector<int> city_list;   //��������ĳ�������
	vector<double> confidence;    //������ļ��ж�ȡ��������  ��localpathplanning�л�ȡ

	double **ant_data;  //�л����۾���dij    n*n
	double **ant_eta;   //����ʽ���Ӧ� �л����۵ĵ���  n*n  1/ant_data
	double **ant_Tau;   //��Ϣ�ؾ���    n*n  ��ʼ��Ϊȫ1   ����ij֮�����Ϣ��Ũ��
	int **ant_Tabu;  //�洢����¼·��������   m*n ��ʼΪȫ0   ��¼ÿ��ÿֻ����ѡ���·��
	int **Route_best;    //�������·��  NC_max * n
	double *Length_best;     //�������·�ߵĳ��� ��ʼ����Ϊ����� NC_max 
	double *Length_ave;      //����·�ߵ�ƽ������   NC_max

	vector<double> Length;    //����k����һ��ѭ��������·���ĳ���
};