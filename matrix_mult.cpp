#include <iostream>

int prod[3][3] = {{0,0,0},{0,0,0},{0,0,0}};


int matmul(int a, int b)
{
	for(int row = 0; row < 3; ++row)
	{		
		for(int col = 0; col < 3; ++col)
		{
			for(int hidden = 0; hidden < 3; ++hiden)
			{
				prod[row][col] += a[row][hidden] * b[hidden][col];
			}
		}
	}
	return prod
}

int main()
{

	int a[3][3] = {{1, 2, 3}, {4, 5, 6}, {7,8,9}};
	int b[3][3] = {{3, 2,1}, {6, 5, 4}, {9, 8, 7}};

	int prod[3][3] = matmul(a, b);

	std::cout << "Product is: \n" << prod << std::endl;
}