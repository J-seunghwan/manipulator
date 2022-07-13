#include "Matrix.h"

/*
[0][0]	[0][1]	[0][2]	[0][3]		[0][0]	[0][1]	[0][2]	[0][3]
[1][0]	[1][1]	[1][2]	[1][3]		[1][0]	[1][1]	[1][2]	[1][3]
[2][0]	[2][1]	[2][2]	[2][3]		[2][0]	[2][1]	[2][2]	[2][3]
[3][0]	[3][1]	[3][2]	[3][3]		[3][0]	[3][1]	[3][2]	[3][3]
*/

Hmatrix::Hmatrix()
{
	memset(this->e, 0, sizeof(this->e));
	this->e[0][0] = this->e[1][1] = this->e[2][2] = this->e[3][3] = 1;
}

Hmatrix Hmatrix::I()
{
	Hmatrix tmp;
	return tmp;
}

Hmatrix Hmatrix::trans(double x, double y, double z)
{
	Hmatrix h;

	h.e[0][3] = x;
	h.e[1][3] = y;
	h.e[2][3] = z;
	return h;
}

Hmatrix Hmatrix::rotX(double q)
{
	q = deg2rad(q);
	double c = cos(q);
	double s = sin(q);

	Hmatrix h;
	h.e[1][1] = c;
	h.e[1][2] = -s;
	h.e[2][1] = s;
	h.e[2][2] = c;

	return h;
}

Hmatrix Hmatrix::rotY(double q)
{
	q = deg2rad(q);
	double c = cos(q);
	double s = sin(q);

	Hmatrix h;
	h.e[0][0] = c;
	h.e[0][2] = s;
	h.e[2][0] = -s;
	h.e[2][2] = c;
	
	return h;
}

Hmatrix Hmatrix::rotZ(double q)
{
	q = deg2rad(q);
	double c = cos(q);
	double s = sin(q);

	Hmatrix h;
	h.e[0][0] = c;
	h.e[0][1] = -s;
	h.e[1][0] = s;
	h.e[1][1] = c;

	return h;
}

Hmatrix Hmatrix::inv()
{
	Hmatrix res;

	res.e[0][0] = this->e[0][0];
	res.e[1][1] = this->e[1][1];
	res.e[2][2] = this->e[2][2];

	res.e[1][0] = this->e[0][1];
	res.e[2][0] = this->e[0][2];

	res.e[0][1] = this->e[1][0];
	res.e[0][2] = this->e[2][0];

	res.e[1][2] = this->e[2][1];
	res.e[2][1] = this->e[1][2];

	res.e[0][3] = -res.e[0][0] * this->e[0][3] - res.e[0][1] * this->e[1][3] - res.e[0][2] * this->e[2][3];
	res.e[1][3] = -res.e[1][0] * this->e[0][3] - res.e[1][1] * this->e[1][3] - res.e[1][2] * this->e[2][3];
	res.e[2][3] = -res.e[2][0] * this->e[0][3] - res.e[2][1] * this->e[1][3] - res.e[2][2] * this->e[2][3];

	return res;
}

Hmatrix Hmatrix::operator*(Hmatrix m)
{
	Hmatrix tmp;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			tmp.e[i][j] = this->e[i][0] * m.e[0][j] + this->e[i][1] * m.e[1][j]
				+ this->e[i][2] * m.e[2][j] + this->e[i][3] * m.e[3][j];
		}
	}

	return tmp;
}

//**************************************************************************************************************************************

Matrix::Matrix(int row, int col)
{
	this->m = std::make_unique<double[]>(row * col);
	this->row = row;
	this->col = col;

}

Matrix::Matrix(int size)
{
	this->m = std::make_unique<double[]>(size * size);
	this->row = size;
	this->col = size;
}

int Matrix::getrow()
{
	return this->row;
}

int Matrix::getcol()
{
	return this->col;
}

Matrix Matrix::copy()
{
	Matrix res(this->row, this->col);
	for (int i = 0; i < this->row; i++) {
		for (int j = 0; j < this->col; j++) {
			res(i, j) = this->m[i * this->col + j];
		}
	}

	return res;
}

void Matrix::print()
{
	printf("matrix([");
	bool sw = false;
	
	for (int i = 0; i < this->row; i++) {
		
		if (i > 0) printf("\t");
		
		for (int j = 0; j < this->col; j++) {
			printf("\t%f", this->m[i * this->col + j]);
		}

		if (i == this->row - 1)
		{
			sw = true;
			printf("])");
		}

		if(!sw) printf("\n");
	}
	printf("\n");
}

Matrix Matrix::inv()
{
	if (!this->isSquare()) {//정방행렬 검사
		//throw
	}
	
	//det


	//
	int size = this->row;
	Matrix tmp = this->copy(), res = this->I(size);

	const double e = 1.0e-10;
	for (int i = 0; i < size; i++) {
		if (-e < tmp(i, i) && tmp(i, i) < e)
		{
			for (int k = 0; k < size; k++) {
				if (-e < tmp(k, i) && tmp(k, i) < e)
				{
					continue;
				}
				for (int j = 0; j < size; j++)
				{
					tmp(i, j) += tmp(k, j);
					res(i, j) += res(k, j);
				}
				break;
			}
			if (-e < tmp(i, i) && tmp(i, i) < e) {
				return res;
			}
		}
	}

	for (int i = 0; i < size; i++) {
		double constant = tmp(i, i);
	
		for (int j = 0; j < size; j++) {
			tmp(i, j) /= constant;
			res(i, j) /= constant;
		}

		for (int j = 0; j < size; j++) {
			if (j == i) {
				continue;
			}
			if (tmp(j, i) == 0) {
				continue;
			}

			constant = tmp(j, i);
			for (int k = 0; k < size; k++) {
				tmp(j, k) = tmp(j, k) - tmp(i, k) * constant;
				res(j, k) = res(j, k) - res(i, k) * constant;
			}
		}
	}
	return res;
}

bool Matrix::isSquare()
{
	if (this->col == this->row) {
		return true;
	}
	else
	{
		return false;
	}
}

double Matrix::det()
{
	if (!this->isSquare()) {
		return 0;
	}


}

Matrix Matrix::I(int size)
{
	Matrix res(size);

	for (int i = 0; i < size; i++) {
		for (int j = 0; j < size; j++) {
			if (i == j) {
				res(i, j) = 1;
			}
			else {
				res(i, j) = 0;
			}
		}
	}

	return res;
}

Matrix Matrix::operator*(Matrix &matrix)
{
	if (this->col == matrix.getrow())
	{
		Matrix res(this->row, matrix.getcol());

		int res_row = res.getrow();
		int res_col = res.getcol();

		for (int i = 0; i < res_row; i++) {
			for (int j = 0; j < res_col; j++) {
				for (int k = 0; k < this->col; k++) {
					res.m[i * res_col + j] += this->m[i * this->col + k] * matrix.m[k * res_col + j];
				}
			}
		}
		return res;
	}
	else
	{
		printf("shape is different\n");
	}
}

Matrix Matrix::operator*(double s)
{
	Matrix res(this->row, this->col);

	for (int i = 0; i < this->row; i++) {
		for (int j = 0; j < this->col; j++) {
			res.m[i * this->col + j] = this->m[i * this->col + j] * s;
		}
	}
	return res;
}

double& Matrix::operator()(int i, int j)
{
	if (i >= this->row){
		printf("row index is over");
		assert(i >= this->row);
	}
	if (j >= this->col) {
		printf("colum index is over");
		assert(j >= this->col);
	}
	return this->m[i * this->col + j];
}

//**************************************************************************************************************************************

double rad2deg(double rad)
{
	return rad * 180 / 3.14159265;
}

double deg2rad(double deg)
{
	return deg * 3.14159265 / 180;
}
