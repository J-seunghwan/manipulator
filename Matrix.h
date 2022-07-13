#pragma once
#include <memory>
#include <assert.h>

class Hmatrix
{
public:
	Hmatrix();

	Hmatrix I();
	Hmatrix trans(double x, double y, double z);	//해당 지점으로 이동
	Hmatrix rotX(double q);
	Hmatrix rotY(double q);
	Hmatrix rotZ(double q);
	Hmatrix inv();

	Hmatrix operator *(Hmatrix m);

	double e[4][4];

};

//**************************************************************************************************************************************

class Matrix
{
public:
	Matrix(int row, int col);
	Matrix(int size);
	int getrow();
	int getcol();
	Matrix copy();
	void print();
	Matrix inv();
	bool isSquare();
	double det();
	Matrix I(int size);
	Matrix operator * (Matrix &matrix);
	Matrix operator * (double s);
	double& operator () (int i, int j);
	

	std::unique_ptr<double[]> m;
	// [i*col + j] == [i][j]

private:
	int row = 0, col = 0;

};

//**************************************************************************************************************************************

double rad2deg(double rad);
double deg2rad(double deg);