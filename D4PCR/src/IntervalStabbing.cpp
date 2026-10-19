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
        if (a.value < b.value)
            return true;
        if (a.value > b.value)
            return false;

        // For closed intervals, process a left endpoint first at an equal value.
        if (a.is_left != b.is_left)
            return a.is_left;
        return false;
        });
}

void IntervalStabbing::sortEndpoints(IntervalsW& intervalsW)
{
    std::sort(intervalsW.begin(), intervalsW.end(), [](const IntervalW& a, const IntervalW& b) {
        if (a.value < b.value)
            return true;
        if (a.value > b.value)
            return false;

        if (a.is_left != b.is_left)
            return a.is_left;
        return false;
        });
}

void IntervalStabbing::compute(Intervals& intervals)
{
    const size_t nMax = intervals.size();

    if (nMax < 2)
    {
        _nStabbed = 0.0f;
        _optimalValue[0] = 0.0f;
        _optimalValue[1] = 0.0f;
        return;
    }

    float bestCount = _nStabbed;
    size_t optimalIndex = 0;
    float count = 0.0f;

    for (size_t i = 0; i < nMax; ++i)
    {
        if (intervals[i].is_left)
        {
            ++count;
            if (count > bestCount)
            {
                bestCount = count;
                optimalIndex = i;
            }
        }
        else
        {
            --count;
        }
    }

    _nStabbed = bestCount;
    _optimalValue[0] = intervals[optimalIndex].value;
    _optimalValue[1] = intervals[std::min(optimalIndex + 1, nMax - 1)].value;
}

void IntervalStabbing::compute(IntervalsW& intervalsW)
{
    const size_t nMax = intervalsW.size();

    if (nMax < 2)
    {
        _nStabbed = 0.0f;
        _optimalValue[0] = 0.0f;
        _optimalValue[1] = 0.0f;
        return;
    }

    float bestCount = _nStabbed;
    size_t optimalIndex = 0;
    float count = 0.0f;

    for (size_t i = 0; i < nMax; ++i)
    {
        if (intervalsW[i].is_left)
        {
            ++count;
            if (count > bestCount)
            {
                bestCount = count;
                optimalIndex = i;
            }
        }
        else
        {
            --count;
        }
    }

    _nStabbed = bestCount;
    _optimalValue[0] = intervalsW[optimalIndex].value;
    _optimalValue[1] = intervalsW[std::min(optimalIndex + 1, nMax - 1)].value;
}

float* IntervalStabbing::getOptimalValue()
{
    return _optimalValue;
}

float IntervalStabbing::getNStabbed()
{
    return _nStabbed;
}
