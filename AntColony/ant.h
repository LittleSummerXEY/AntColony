#pragma once
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <ctime>
#include <cmath>
#include <iostream>
using namespace std;

//定义蚂蚁类
class CAnt
{
public:
	CAnt(void){};
	~CAnt(void);

	void getData(int num, vector<double> &data);     //1.对数组进行初始化  city_num为n, ant_num为m
	void initData();     //1.对数组进行初始化  city_num为n, ant_num为m
	void deleteData();    //删除数组内存
	void randPermCpp();    //类似matlab中的randperm函数，返回从0-n-1的随机顺序的整数
	void setAnt();   //2.放置蚂蚁
	void chooseNextCity();   //3.m只蚂蚁按概率函数选择下一座城市，完成各自的周游
	double getMinimum(vector<double> num_list, int &pos);    //求一个vector中的最小值,并记录其位置
	double getMinimum(double *num_list, int &pos);   //求一个数组中的最小值，并记录其位置
	double getMean(vector<double> num_list);   //获取一个序列中所有数的平均值
	void recordRoute();   //4.记录本次迭代最佳路径
	void updatePheromone();   //5.更新信息素

	void computeRoute(vector<int> &route);    //计算路径

private:

	int ant_num = 50;   //蚂蚁个数   m  50
	int city_num = 64;   //城市数量   n  64
	double ant_alpha = 1;   //表示信息素重要程度的参数  α  1            
	double ant_beta = 5;  // 表示启发式因子重要程度的参数  β  5   
	double ant_rho = 0.1;   //信息素蒸发系数   ρ  0.1          
	int Q = 100;  //信息素增加强度系数
	int NC = 0;   //迭代计数器，记录迭代次数
	int NC_max = 200;  //最大迭代次数

	vector<int> city_list;   //保存随机的城市序列
	vector<double> confidence;    //保存从文件中读取到的数据  从localpathplanning中获取

	double **ant_data;  //切换代价矩阵dij    n*n
	double **ant_eta;   //启发式因子η 切换代价的倒数  n*n  1/ant_data
	double **ant_Tau;   //信息素矩阵    n*n  初始化为全1   城市ij之间的信息素浓度
	int **ant_Tabu;  //存储并记录路径的生成   m*n 初始为全0   记录每次每只蚂蚁选择的路径
	int **Route_best;    //各代最佳路线  NC_max * n
	double *Length_best;     //各代最佳路线的长度 初始设置为无穷大 NC_max 
	double *Length_ave;      //各代路线的平均长度   NC_max

	vector<double> Length;    //蚂蚁k经过一个循环所经过路径的长度
};