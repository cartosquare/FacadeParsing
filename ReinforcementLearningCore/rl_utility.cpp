#include "rl_utility.h"

#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <assert.h>

namespace RL
{
	double RLUtility::Max( double * array, int n ) {
		double max = array[0];

		for (int i = 1; i < n; ++i)
		{
			if (max < array[i])
				max = array[i];
		}
		return max;
	}

	double RLUtility::Max(double* array, std::vector<int> index) {
		double max = array[index[0]];

		for (size_t i = 1; i < index.size(); ++i)
		{
			if (max < array[index[i]])
			{
				max = array[index[i]];
			}
		}
		return max;
	}

	int RLUtility::ArgMax(double* array, int n) {
		double max = array[0];
		int IMax = 0;

		for (int i = 1; i < n; ++i)
		{
			if (max < array[i])
			{
				max = array[i];
				IMax = i;
			}
		}
		return IMax;
	}

	int RLUtility::ArgMax(double* array, std::vector<int> index) {
		double max = array[index[0]];
		int IMax = index[0];

		for (size_t i = 1; i < index.size(); ++i)
		{
			if (max < array[index[i]])
			{
				max = array[index[i]];
				IMax = index[i];
			}
		}
		return IMax;
	}

	std::vector<double> RLUtility::MaxAll(double* array, int n) {
		double max = array[0];
		std::vector<double> max_all;
		max_all.push_back(max);

		for (int i = 1; i < n; ++i)
		{
			if (max < array[i])
			{
				max = array[i];
				max_all.clear();
				max_all.push_back(array[i]);
			}
			else if (max == array[i])
			{
				max_all.push_back(array[i]);
			}
		}
		
		return max_all;
	}

	std::vector<double> RLUtility::MaxAll(double* array, std::vector<int> index) {
		double max = array[index[0]];
		std::vector<double> max_all;
		max_all.push_back(max);

		for (size_t i = 1; i < index.size(); ++i)
		{
			if (max < array[index[i]])
			{
				max = array[index[i]];

				max_all.clear();
				max_all.push_back(max);
			}
			else if (max == array[index[i]])
			{
				max_all.push_back(max);
			}
		}

		return max_all;
	}

	std::vector<int> RLUtility::ArgMaxAll(double* array, int n) {
		double max = array[0];

		std::vector<int> arg_all;
		arg_all.push_back(0);

		for (int i = 1; i < n; ++i)
		{
			if (max < array[i])
			{
				max = array[i];
				arg_all.clear();
				arg_all.push_back(i);
			}
			else if (max == array[i])
			{
				arg_all.push_back(i);
			}
		}

		return arg_all;
	}

	std::vector<int> RLUtility::ArgMaxAll(double* array, std::vector<int> index) {
		double max = array[index[0]];

		std::vector<int> max_all;
		max_all.push_back(index[0]);

		for (size_t i = 1; i < index.size(); ++i)
		{
			if (max < array[index[i]])
			{
				max = array[index[i]];

				max_all.clear();
				max_all.push_back(index[i]);
			}
			else if (max == array[index[i]])
			{
				max_all.push_back(index[i]);
			}
		}

		return max_all;
	}

	int RLUtility::Rand0A(int A) {
		return (std::rand() % A);
	}

	int RLUtility::RandAB(int A, int B) {
		assert(B > A);
		int cut = std::rand() % (B - A);
		return (A + cut);
	}

	double RLUtility::RandUnit() {
		return (static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX));
	}
}


