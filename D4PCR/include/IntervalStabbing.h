#ifndef _INTERVALSTABBING_H_
#define _INTERVALSTABBING_H_

#include "Preparation.h"
#include "dll_export.h"
class DLL_EXPORT IntervalStabbing
{
public:
	IntervalStabbing(Intervals intervals);
	IntervalStabbing(IntervalsW intervalsW);
	~IntervalStabbing();

	float* getOptimalValue();
	float getNStabbed();

	//std::vector<std::pair<float, size_t>> getDataSet();

private:
	void sortEndpoints(Intervals& intervals);
	void sortEndpoints(IntervalsW& intervalsW);

	void compute(Intervals& intervals);
	void compute(IntervalsW& intervalsW);

private:
	//std::vector<std::pair<float, size_t>> _dataSet;
	float _optimalValue[2];
	float _nStabbed;
};



#endif // !_INTERVALSTABBING_H_

