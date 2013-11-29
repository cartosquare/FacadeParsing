#ifndef RL_UTILITY_H
#define RL_UTILITY_H

#include <vector>

//! Reinforcement Learning Algorithm namespace.
namespace RL
{
	//! Utility functions for Reinforcement Learning.
	class RLUtility {
	public:
		RLUtility() {}
		~RLUtility() {}

		//! Get first max element in the array.
		/*!
		\param array the array.
		\param n the array size.
		\return the first max element in the array.
		\sa ArgMax(), MaxAll(), ArgMaxAll()
		*/
		static double Max(double* array, int n);

		//! Get first max element in the sub-array of an array.
		/*!
		\param array the array.
		\param index the sub-array's index in the array.
		\return the first max element in the sub-array.
		*/
		static double Max(double* array, std::vector<int> index);

		//! Get first max element's index in the array.
		/*!
		\param array the array.
		\param n the array size.
		\return the first max element in the array.
		*/
		static int ArgMax(double* array, int n);

		//! Get first max element's index in the sub-array of an array.
		/*!
		\param array the array.
		\param index the sub-array's index in the array.
		\return the first max element's index in the sub-array.
		*/
		static int ArgMax(double* array, std::vector<int> index);

		//! Get all the max elements' indexes in the array.
		/*!
		\param array the array.
		\param n array size.
		\return all the max elements' indexes in the array.
		*/
		static std::vector<double> MaxAll(double* array, int n);

		//! Get all the max elements in the sub-array of an array.
		/*!
		\param array the array.
		\param index the sub-array's index in the array.
		\return all the max elements in the sub-array of an array.
		*/
		static std::vector<double> MaxAll(double* array, std::vector<int> index);

		//! Get all the max elements' indexes in the array.
		/*!
		\param array the array.
		\param n array size.
		\return all the max elements' indexes in the array.
		*/
		static std::vector<int> ArgMaxAll(double* array, int n);

		//! Get all the max elements' indexes in the sub-array of an array.
		/*!
		\param array the array.
		\param index the sub-array's index in the array.
		\return all the max elements' indexes in the sub-array of an array.
		*/
		static std::vector<int> ArgMaxAll(double* array, std::vector<int> index);

		//! Generate an int in [0, A)
		static int Rand0A(int A);

		//! Generate an int in [A, B)
		static int RandAB(int A, int B);

		//! Generate a double in [0, 1]
		static double RandUnit();
	};
}

#endif