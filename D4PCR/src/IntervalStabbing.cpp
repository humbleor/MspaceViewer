#include "../include/IntervalStabbing.h"

IntervalStabbing::IntervalStabbing(Intervals intervals)
{
	_nStabbed = 0.0f;
	sortEndpoints(intervals);
	compute(intervals);
}

IntervalStabbing::IntervalStabbing(IntervalsW intervalsW)
{
	_nStabbed = 0.0f;
	sortEndpoints(intervalsW);
	compute(intervalsW);
}

IntervalStabbing::~IntervalStabbing()
{
}

void IntervalStabbing::sortEndpoints(Intervals& intervals)
{
	std::sort(intervals.begin(), intervals.end(), [](const Interval& a, const Interval& b) {
		return (a.value < b.value); });
}

void IntervalStabbing::sortEndpoints(IntervalsW& intervalsW)
{
	std::sort(intervalsW.begin(), intervalsW.end(), [](const IntervalW& a, const IntervalW& b) {
		return (a.value < b.value); });
}



void IntervalStabbing::compute(Intervals& intervals)
{
	unsigned int nMax = /*2 **/ intervals.size();
	//_dataSet.resize(nMax);

	int count = 0;
	int optimalIndex = 0;
	//float optimalVauleL = 0.0f;
	//float optimalVauleR = 0.0f;

	//std::ofstream dataFile;
	//dataFile.open("test.txt", std::ofstream::out);

	for (unsigned int i = 0; i < nMax; i++)
	{
		if (intervals[i].is_left)
		{
			count++;
			if (count > _nStabbed)
			{
				_nStabbed = count;
				optimalIndex = i;
				//optimalVauleL = intervals[i].value;
				//optimalVauleR = intervals[i + 1].value;
			}
		}
		else
		{
			count--;
		}

		//_dataSet[i] = std::make_pair(intervals[i].value, count);
		/*dataFile << intervals[i].value << " " << count << std::endl;*/
		//std::cout <<"the interval of parallel length is: "<< _intervals[i].first << "  ";
		//std::cout << "the count is: " << count << "." << std::endl;;
	}

	/*dataFile.close();*/
	_optimalValue[0] = intervals[optimalIndex].value;
	_optimalValue[1] = intervals[optimalIndex + 1].value;
}

void IntervalStabbing::compute(IntervalsW& intervalsW)
{
	unsigned int nMax = /*2 **/ intervalsW.size();
	//_dataSet.resize(nMax);

	float count = 0.0f;
	int optimalIndex = 0;
	//float optimalVauleL = 0.0f;
	//float optimalVauleR = 0.0f;

	for (unsigned int i = 0; i < nMax; i++)
	{
		if (intervalsW[i].is_left)
		{
			count++;
			if (count > _nStabbed)
			{
				_nStabbed = count;
				optimalIndex = i;
				//optimalVauleL = intervals[i].value;
				//optimalVauleR = intervals[i + 1].value;
			}
		}
		else
		{
			count--;
		}

		//_dataSet[i] = std::make_pair(intervals[i].value, count);
		/*dataFile << intervals[i].value << " " << count << std::endl;*/
		//std::cout <<"the interval of parallel length is: "<< _intervals[i].first << "  ";
		//std::cout << "the count is: " << count << "." << std::endl;;
	}

	/*dataFile.close();*/
	_optimalValue[0] = intervalsW[optimalIndex].value;
	_optimalValue[1] = intervalsW[optimalIndex + 1].value;
}

float* IntervalStabbing::getOptimalValue()
{
	return _optimalValue;
}

float IntervalStabbing::getNStabbed()
{
	return _nStabbed;
}

//std::vector<std::pair<float, size_t>> IntervalStabbing::getDataSet()
//{
//	return _dataSet;
//}
