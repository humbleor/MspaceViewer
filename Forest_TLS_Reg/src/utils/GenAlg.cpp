#include "../../include/utils/GenAlg.h"
#include "../../include/utils/Hlp.h"
#include <cstring>

// Profile: identify contiguous groups in a 1D array based on difference threshold
bool Profile(const double* pdValIn, const long lEleNum, const double dDTh, const int nStep, int* pnGrpID)
{
    bool hasGroup = false;
    long currentGrpID = -1;

    for (long i = 0; i < lEleNum; i += nStep)
    {
        if (pdValIn[i] > INSIGNIFICANCE + 1.0)
        {
            if (currentGrpID == -1)
            {
                currentGrpID = i;
                pnGrpID[i] = (int)currentGrpID;
            }
            else
            {
                double diff = std::abs(pdValIn[i] - pdValIn[currentGrpID]);
                if (diff < dDTh)
                {
                    pnGrpID[i] = (int)currentGrpID;
                }
                else
                {
                    currentGrpID = i;
                    pnGrpID[i] = (int)currentGrpID;
                }
            }
            hasGroup = true;
        }
        else
        {
            currentGrpID = -1;
            pnGrpID[i] = -1;
        }
    }

    // Fill in skipped elements (when nStep > 1)
    if (nStep > 1)
    {
        for (long i = 0; i < lEleNum; i++)
        {
            if (pnGrpID[i] == 0)
            {
                long prev = -1, next = -1;
                for (long j = i - 1; j >= 0; j--)
                {
                    if (pnGrpID[j] != 0 && pnGrpID[j] != -1) { prev = j; break; }
                }
                for (long j = i + 1; j < lEleNum; j++)
                {
                    if (pnGrpID[j] != 0 && pnGrpID[j] != -1) { next = j; break; }
                }
                if (prev >= 0) pnGrpID[i] = pnGrpID[prev];
                else if (next >= 0) pnGrpID[i] = pnGrpID[next];
                else pnGrpID[i] = -1;
            }
        }
    }

    return hasGroup;
}

// GrpInfo: compute statistics for each group
bool GrpInfo(const int* pnGrpID, const long lEleNum, const long lGrpEleNumTh, long* plGrpNum, GrpStaic** pGrpEle)
{
    if (!pnGrpID || !plGrpNum || !pGrpEle) return false;

    std::unordered_map<int, long> grpCounts;
    std::unordered_map<int, long> grpStart;
    std::unordered_map<int, long> grpEnd;

    for (long i = 0; i < lEleNum; i++)
    {
        if (pnGrpID[i] >= 0)
        {
            int grpID = pnGrpID[i];
            grpCounts[grpID]++;
            if (grpStart.find(grpID) == grpStart.end()) grpStart[grpID] = i;
            grpEnd[grpID] = i;
        }
    }

    long validGrpCount = 0;
    for (auto& pair : grpCounts)
    {
        if (pair.second >= lGrpEleNumTh) validGrpCount++;
    }

    *plGrpNum = validGrpCount;
    if (validGrpCount == 0) return false;

    *pGrpEle = new GrpStaic[validGrpCount];
    long idx = 0;
    for (auto& pair : grpCounts)
    {
        if (pair.second >= lGrpEleNumTh)
        {
            (*pGrpEle)[idx].lGrpID = pair.first;
            (*pGrpEle)[idx].lEleNum = pair.second;
            (*pGrpEle)[idx].lStaID = grpStart[pair.first];
            (*pGrpEle)[idx].lEndID = grpEnd[pair.first];
            idx++;
        }
    }

    return true;
}

// Grp_ID_Ord: renumber group IDs to be contiguous
bool Grp_ID_Ord(int* pnGrp_ID, const long lEle_Num, const int nGrp_Ele_Min)
{
    if (!pnGrp_ID) return false;

    std::unordered_map<int, int> idMap;
    int newID = 0;
    for (long i = 0; i < lEle_Num; i++)
    {
        if (pnGrp_ID[i] >= 0 && idMap.find(pnGrp_ID[i]) == idMap.end())
            idMap[pnGrp_ID[i]] = newID++;
    }
    for (long i = 0; i < lEle_Num; i++)
    {
        if (pnGrp_ID[i] >= 0) pnGrp_ID[i] = idMap[pnGrp_ID[i]];
    }
    return true;
}

// Ele_Num_Max: find the group with maximum elements
bool Ele_Num_Max(int* pnGrp_ID, const long lEle_Num, long* plGrp_ID_Max, long* plGrp_Ele_Num_Max, long* plGrp_Ele_Sta_End)
{
    if (!pnGrp_ID || !plGrp_ID_Max || !plGrp_Ele_Num_Max) return false;

    std::unordered_map<int, long> grpCounts;
    std::unordered_map<int, long> grpStart;

    for (long i = 0; i < lEle_Num; i++)
    {
        if (pnGrp_ID[i] >= 0)
        {
            grpCounts[pnGrp_ID[i]]++;
            if (grpStart.find(pnGrp_ID[i]) == grpStart.end()) grpStart[pnGrp_ID[i]] = i;
        }
    }

    long maxCount = 0;
    long maxGrpID = -1;
    for (auto& pair : grpCounts)
    {
        if (pair.second > maxCount) { maxCount = pair.second; maxGrpID = pair.first; }
    }

    *plGrp_ID_Max = maxGrpID;
    *plGrp_Ele_Num_Max = maxCount;
    if (plGrp_Ele_Sta_End) *plGrp_Ele_Sta_End = (maxGrpID >= 0) ? grpStart[maxGrpID] : -1;

    return true;
}

// rowSeg: segment along rows (horizontal scan direction)
// Each row has N_SCAN elements; we scan Horizon_SCAN rows.
// pointMat structure: rows = Horizon_SCAN, cols = N_SCAN
bool rowSeg(double* &dataArray,
            int Horizon_SCAN, int N_SCAN,
            double dDif_Th, int nSch_Stp, int lGrp_Ele_Min,
            cv::Mat &pointTypeMat)
{
    if (!dataArray) return false;

    int* pnGrpID = new int[N_SCAN];
    bool hasSegments = false;

    for (int scan = 0; scan < Horizon_SCAN; scan++)
    {
        memset(pnGrpID, 0, N_SCAN * sizeof(int));
        double* rowPtr = dataArray + scan * N_SCAN;

        Profile(rowPtr, N_SCAN, dDif_Th, nSch_Stp, pnGrpID);

        long lGrpNum = 0;
        GrpStaic* pGrpEle = nullptr;
        GrpInfo(pnGrpID, N_SCAN, lGrp_Ele_Min, &lGrpNum, &pGrpEle);

        if (lGrpNum > 0 && pGrpEle)
        {
            hasSegments = true;
            for (long g = 0; g < lGrpNum; g++)
            {
                for (long idx = pGrpEle[g].lStaID; idx <= pGrpEle[g].lEndID; idx++)
                {
                    if (rowPtr[idx] > INSIGNIFICANCE + 1.0)
                    {
                        pointTypeMat.at<float>(scan, (int)idx) = 1.0f;
                    }
                }
            }
            delete[] pGrpEle;
        }
    }

    delete[] pnGrpID;
    return hasSegments;
}

// colSeg: segment along columns (vertical scan direction)
// Each column has Horizon_SCAN elements; we scan N_SCAN columns.
bool colSeg(double* &dataArray,
            int Horizon_SCAN, int N_SCAN,
            double dDif_Th, int nSch_Stp, int lGrp_Ele_Min,
            cv::Mat &pointTypeMat)
{
    if (!dataArray) return false;

    int* pnGrpID = new int[Horizon_SCAN];
    bool hasSegments = false;

    for (int col = 0; col < N_SCAN; col++)
    {
        memset(pnGrpID, 0, Horizon_SCAN * sizeof(int));

        // Extract column data: element at row r, column col is at index r * N_SCAN + col
        std::vector<double> colData(Horizon_SCAN);
        for (int row = 0; row < Horizon_SCAN; row++)
        {
            colData[row] = dataArray[row * N_SCAN + col];
        }

        Profile(colData.data(), Horizon_SCAN, dDif_Th, nSch_Stp, pnGrpID);

        long lGrpNum = 0;
        GrpStaic* pGrpEle = nullptr;
        GrpInfo(pnGrpID, Horizon_SCAN, lGrp_Ele_Min, &lGrpNum, &pGrpEle);

        if (lGrpNum > 0 && pGrpEle)
        {
            hasSegments = true;
            for (long g = 0; g < lGrpNum; g++)
            {
                for (long idx = pGrpEle[g].lStaID; idx <= pGrpEle[g].lEndID; idx++)
                {
                    if (colData[idx] > INSIGNIFICANCE + 1.0)
                    {
                        pointTypeMat.at<float>((int)idx, col) = 1.0f;
                    }
                }
            }
            delete[] pGrpEle;
        }
    }

    delete[] pnGrpID;
    return hasSegments;
}

// segPoints: main segmentation function
// pointMat: input range image, rows = Horizon_SCAN (imgRow), cols = N_SCAN (imgCol)
// pointTypeMat: output segmentation mask (same dimensions as pointMat)
void segPoints(cv::Mat &pointMat,
                int Horizon_SCAN, int N_SCAN,
                cv::Mat &pointTypeMat,
                ConfigSetting config_setting)
{
    // Convert pointMat to 1D double array (row-major order)
    // pointMat has Horizon_SCAN rows and N_SCAN columns
    double* dataArray = new double[Horizon_SCAN * N_SCAN];

    for (int row = 0; row < Horizon_SCAN; row++)
    {
        for (int col = 0; col < N_SCAN; col++)
        {
            dataArray[row * N_SCAN + col] = (double)pointMat.at<float>(row, col);
        }
    }

    // Initialize output mask with same dimensions as pointMat
    pointTypeMat = cv::Mat::zeros(Horizon_SCAN, N_SCAN, CV_32F);

    // Perform row segmentation (scan along each row, which has N_SCAN elements)
    double* rowArray = dataArray;
    rowSeg(rowArray, Horizon_SCAN, N_SCAN,
           config_setting.dDif_Th, config_setting.nSch_Stp, config_setting.lGrp_Ele_Min,
           pointTypeMat);

    // Perform column segmentation (scan along each column, which has Horizon_SCAN elements)
    double* colArray = dataArray;
    colSeg(colArray, Horizon_SCAN, N_SCAN,
           config_setting.dDif_Th, config_setting.nSch_Stp, config_setting.lGrp_Ele_Min,
           pointTypeMat);

    delete[] dataArray;
}
