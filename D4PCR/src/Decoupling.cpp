#include "../include/Decoupling.h"

Decoupling::Decoupling(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT, pcl::Correspondences cors, float resolution)
{
	//std::shared_ptr<AdjacencyMatrix> adjacency = std::make_shared<AdjacencyMatrix>(sampledCloudS, sampledCloudT, cors, resolution);
	//Eigen::RowVectorXf weightInfor = adjacency->getWeightSet();

	decoupling(sampledCloudS, sampledCloudT, cors, resolution);
	//computeTransformationMatrix(sampledCloudS, sampledCloudT);

}

Decoupling::~Decoupling()
{
}

Eigen::Matrix4d Decoupling::getTransformation()
{
	return transformation;
}

void Decoupling::decoupling(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT, pcl::Correspondences cors, float resolution)
{
	std::cout << "==================Decoupling .....==================" << std::endl;
	auto timeS = std::chrono::system_clock::now();

	Intervals verticalTSet;
	Interval verticalT;

	Eigen::Vector3f s_t(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f g(0.0f, 0.0f, 1.0f);
	for (unsigned int index = 0; index < cors.size(); index++)
	{
		s_t[0] = sampledCloudT->points[cors[index].index_match].x - sampledCloudS->points[cors[index].index_query].x;
		s_t[1] = sampledCloudT->points[cors[index].index_match].y - sampledCloudS->points[cors[index].index_query].y;
		s_t[2] = sampledCloudT->points[cors[index].index_match].z - sampledCloudS->points[cors[index].index_query].z;
		//s_tSet[index] = s_t;

		verticalT.value = -resolution - g.dot(s_t);
		verticalT.is_left = true;
		//verticalT.weight = weightInfor[index];
		verticalTSet.emplace_back(verticalT);

		verticalT.value = resolution - g.dot(s_t);
		verticalT.is_left = false;
		//verticalT.weight = weightInfor[index];
		verticalTSet.emplace_back(verticalT);
	}

	std::shared_ptr<IntervalStabbing> interV = std::make_shared<IntervalStabbing>(verticalTSet);

	float* optimalZSet = interV->getOptimalValue();
	//optimalZSet[0] = 1.40f;
	//optimalZSet[1] = 1.45f;
	pcl::Correspondences corsV;
	std::vector<size_t> indicesV;

	for (size_t i = 0; i < verticalTSet.size(); i += 2)
	{
		if (verticalTSet[i].value > optimalZSet[1])
			continue;
		if (verticalTSet[i + 1].value < optimalZSet[0])
			continue;

		corsV.emplace_back(cors[i / 2]);
		indicesV.emplace_back(i / 2);
	}
	//corsV = cors;
	Edge2DSet edgeSet;
	Edge2D edge;

	for (unsigned int index = 0; index < corsV.size(); index++)
	{
		edge.first[0] = sampledCloudS->points[corsV[index].index_query].x;
		edge.first[1] = sampledCloudS->points[corsV[index].index_query].y;

		edge.second[0] = sampledCloudT->points[corsV[index].index_match].x;
		edge.second[1] = sampledCloudT->points[corsV[index].index_match].y;
		edgeSet.emplace_back(edge);
	}


	IntervalW intervalR;
	IntervalsW intervalRSet, finalIntervalRSet;
	float angle1 = 0.0f, angle2 = 0.0f;
	float theta = 0.0f, error1 = 0.0f, error2 = 0.0f;
	float dx1 = 0.0f, dy1 = 0.0f, dx2 = 0.0f, dy2 = 0.0f;
	float distance1 = 0.0f, distance2 = 0.0f;
	std::vector<size_t> usedIndexSet;
	std::vector<size_t> indicesR;
	std::array<float,2> optimalRSet;
	size_t corsNum = edgeSet.size();
	size_t maxNum = 0;
	float deltaAngle = 1.0 * 2 * M_PI / 180.0f;

	std::vector<bool> usedIndex(corsNum, false);
	for (unsigned int i = 0; i < corsNum; i++)
	{
		if (usedIndex[i] == true) continue;
		for (unsigned int j = 0; j < corsNum; j++)
		{
			if (i == j)
				continue;
			if (usedIndex[j] == true)
				continue;
			dx1 = edgeSet[j].first.x() - edgeSet[i].first.x();
			dy1 = edgeSet[j].first.y() - edgeSet[i].first.y();
			distance1 = std::sqrtf(dx1 * dx1 + dy1 * dy1);
			dx2 = edgeSet[j].second.x() - edgeSet[i].second.x();
			dy2 = edgeSet[j].second.y() - edgeSet[i].second.y();
			distance2 = std::sqrtf(dx2 * dx2 + dy2 * dy2);
			if (std::abs(distance1 - distance2) < resolution)
			{
				if (2 * resolution < distance1)
					error1 = asin(2 * resolution / distance1);
				else
					error1 = M_PI;
				
				//error2 = 2 * resolution / distance1;
				angle1 = atan2f(dy1, dx1);
				angle2 = atan2f(dy2, dx2);
				theta = angle2 - angle1;
				if (theta > M_PI)
					theta = theta - 2 * M_PI;
				if (theta < -M_PI)
					theta = theta + 2 * M_PI;
				intervalR.is_left = true;
				intervalR.value = theta - error1;
				intervalR.index = j;
				intervalRSet.emplace_back(intervalR);
				intervalR.is_left = false;
				intervalR.value = theta + error1;
				intervalR.index = j;
				intervalRSet.emplace_back(intervalR);
			}
		}
		if (intervalRSet.size() < 2)
		{
			intervalRSet.clear();
			continue;
		}

		std::shared_ptr<IntervalStabbing> interR = std::make_shared<IntervalStabbing>(intervalRSet);
		float* RSet = interR->getOptimalValue();
		if (interR->getNStabbed() < 2)
		{
			intervalRSet.clear();
			continue;
		}
		usedIndexSet.clear();
		usedIndexSet.emplace_back(i);

		for (size_t k = 0; k < intervalRSet.size() - 1; k += 2)
		{
			if (intervalRSet[k].value > RSet[1])
				continue;
			if (intervalRSet[k + 1].value < RSet[0])
				continue;
			usedIndexSet.emplace_back(intervalRSet[k].index);
			//if ((intervalRSet[k + 1].value - intervalRSet[k].value)< deltaAngle)
			//	usedIndex[intervalRSet[k].index] = true;

			//relationshipTable[i][intervalRSet[k].index] = 0;
			//relationshipTable[intervalRSet[k].index][i] = 0;
		}
		intervalRSet.clear();
		if (maxNum < usedIndexSet.size())
		{
			maxNum = usedIndexSet.size();
			indicesR = usedIndexSet;
			optimalRSet[0] = RSet[0];
			optimalRSet[1] = RSet[1];
			//std::cout << optimalRSet[0] << "," << optimalRSet[1] << std::endl;
		}
	}


	//std::vector<std::vector<uint8_t>> relationshipTable(corsNum, std::vector<uint8_t>(corsNum, 1));
	//for (size_t i = 0; i < corsNum; i++)
	//{
	//	if (std::accumulate(relationshipTable[i].begin(), relationshipTable[i].end(), 1) < maxNum)
	//		continue;
	//	for (size_t j = 0; j < corsNum; j++)
	//	{
	//		if (relationshipTable[i][j] == 0)
	//			continue;

	//		dx1 = edgeSet[j].first.x() - edgeSet[i].first.x();
	//		dy1 = edgeSet[j].first.y() - edgeSet[i].first.y();
	//		distance1 = std::sqrtf(dx1 * dx1 + dy1 * dy1);
	//		dx2 = edgeSet[j].second.x() - edgeSet[i].second.x();
	//		dy2 = edgeSet[j].second.y() - edgeSet[i].second.y();
	//		distance2 = std::sqrtf(dx2 * dx2 + dy2 * dy2);
	//		if (std::abs(distance1 - distance2) < (2 * resolution))
	//		{
	//			error1 = 2 * resolution / distance1;
	//			if (error1 > M_PI)
	//				continue;
	//			//if (resolution < distance1)
	//			//	error1 = asinf(resolution / distance1);
	//			//else
	//			//	error1 = M_PI;

	//			//if (resolution < distance2)
	//			//	error2 = asinf(resolution / distance2);
	//			//else
	//			//	error2 = M_PI;

	//			//if (error2 > M_PI)
	//			//	continue;
	//			angle1 = atan2f(dy1, dx1);
	//			angle2 = atan2f(dy2, dx2);
	//			theta = angle2 - angle1;
	//			if (theta > M_PI)
	//				theta = theta - 2 * M_PI;
	//			if (theta < -M_PI)
	//				theta = theta + 2 * M_PI;
	//			intervalR.is_left = true;
	//			intervalR.value = theta - error1 /*- error2*/;
	//			intervalR.index = j;
	//			intervalRSet.emplace_back(intervalR);
	//			intervalR.is_left = false;
	//			intervalR.value = theta + error1/* + error2*/;
	//			intervalR.index = j;
	//			intervalRSet.emplace_back(intervalR);
	//		}
	//		else
	//		{
	//			relationshipTable[i][j] = 0;
	//			relationshipTable[j][i] = 0;
	//		}
	//	}
	//	if (intervalRSet.size() < 2)
	//	{
	//		intervalRSet.clear();
	//		continue;
	//	}

	//	std::shared_ptr<IntervalStabbing> interR = std::make_shared<IntervalStabbing>(intervalRSet);
	//	float* RSet = interR->getOptimalValue();
	//	if (interR->getNStabbed() < 2)
	//	{
	//		intervalRSet.clear();
	//		continue;
	//	}
	//	usedIndexSet.clear();
	//	usedIndexSet.emplace_back(i);

	//	for (size_t k = 0; k < intervalRSet.size(); k += 2)
	//	{
	//		if (intervalRSet[k].value > RSet[1])
	//			continue;
	//		if (intervalRSet[k + 1].value < RSet[0])
	//			continue;
	//		usedIndexSet.emplace_back(intervalRSet[k].index);
	//		relationshipTable[i][intervalRSet[k].index] = 0;
	//		relationshipTable[intervalRSet[k].index][i] = 0;
	//	}
	//	for (size_t k = 0; k < usedIndexSet.size() - 1; k++)
	//	{
	//		for (size_t l = k + 1; l < usedIndexSet.size(); l++)
	//		{
	//			relationshipTable[usedIndexSet[k]][usedIndexSet[l]] = 0;
	//			relationshipTable[usedIndexSet[l]][usedIndexSet[k]] = 0;
	//		}
	//	}
	//	intervalRSet.clear();
	//	if (maxNum < usedIndexSet.size())
	//	{
	//		maxNum = usedIndexSet.size();
	//		indicesR = usedIndexSet;
	//		optimalRSet[0] = RSet[0];
	//		optimalRSet[1] = RSet[1];
	//	}
	//}

	//size_t inlierMaxNum = 0, outlierMaxNum = 0;

	//std::vector<size_t> freeIndexSet(corsNum);
	//// 使用std::iota来填充vector，从0到n-1
	//size_t first = 0, second = 0;
	//std::iota(freeIndexSet.begin(), freeIndexSet.end(), 0);
	//for (size_t i = 0; i < freeIndexSet.size(); i++)
	//{
	//	first = freeIndexSet[i];
	//	for (size_t j = 0; j < freeIndexSet.size(); j++)
	//	{
	//		second = freeIndexSet[j];
	//		//if (i >= j)
	//		//	continue;
	//		dx1 = edgeSet[second].first.x() - edgeSet[first].first.x();
	//		dy1 = edgeSet[second].first.y() - edgeSet[first].first.y();
	//		distance1 = std::sqrtf(dx1 * dx1 + dy1 * dy1);
	//		dx2 = edgeSet[second].second.x() - edgeSet[first].second.x();
	//		dy2 = edgeSet[second].second.y() - edgeSet[first].second.y();
	//		distance2 = std::sqrtf(dx2 * dx2 + dy2 * dy2);
	//		if (std::abs(distance1 - distance2) < (2 * resolution))
	//		{
	//			error1 = 2 * resolution / distance2;
	//			if (error1 > M_PI)
	//				continue;
	//			error2 = 2 * resolution / distance2;
	//			if (error2 > M_PI)
	//				continue;
	//			angle1 = atan2f(dy1, dx1);
	//			angle2 = atan2f(dy2, dx2);
	//			theta = angle2 - angle1;
	//			if (theta > M_PI)
	//				theta = theta - 2 * M_PI;
	//			if (theta < -M_PI)
	//				theta = theta + 2 * M_PI;
	//			intervalR.is_left = true;
	//			intervalR.value = theta /*- error1*/ - error2;
	//			intervalR.index = j;
	//			intervalRSet.emplace_back(intervalR);
	//			intervalR.is_left = false;
	//			intervalR.value = theta /*+ error1*/ + error2;
	//			intervalR.index = j;
	//			intervalRSet.emplace_back(intervalR);
	//		}
	//	}

	//	if (intervalRSet.size() < 4)
	//		continue;
	//	std::shared_ptr<IntervalStabbing> interR = std::make_shared<IntervalStabbing>(intervalRSet);
	//	float* RSet = interR->getOptimalValue();
	//	if (interR->getNStabbed() < 3 || interR->getNStabbed() < inlierMaxNum)
	//	{
	//		if (outlierMaxNum < interR->getNStabbed())
	//		{
	//			finalIntervalRSet.swap(intervalRSet);
	//			outlierMaxNum = interR->getNStabbed();
	//		}
	//		intervalRSet.clear();
	//		continue;
	//	}

	//	usedIndexSet.clear();
	//	i--;
	//	usedIndexSet.emplace_back(first);

	//	for (size_t k = 0; k < intervalRSet.size(); k += 2)
	//	{
	//		if (intervalRSet[k].value > RSet[1])
	//			continue;
	//		if (intervalRSet[k + 1].value < RSet[0])
	//			continue;
	//		usedIndexSet.emplace_back(intervalRSet[k].index);
	//	}
	//	intervalRSet.clear();
	//	if (inlierMaxNum < usedIndexSet.size())
	//	{
	//		inlierMaxNum = usedIndexSet.size();
	//		indicesR = usedIndexSet;
	//		optimalRSet[0] = RSet[0];
	//		optimalRSet[1] = RSet[1];
	//	}
	//	std::vector<size_t> difference;
	//	std::set_difference(freeIndexSet.begin(), freeIndexSet.end(),
	//		usedIndexSet.begin(), usedIndexSet.end(),
	//		std::back_inserter(difference));
	//	if (inlierMaxNum > freeIndexSet.size())
	//		break;
	//	freeIndexSet.swap(difference);
	//}
	//if (indicesR.size() == 0)
	//{
	//	std::shared_ptr<IntervalStabbing> interR = std::make_shared<IntervalStabbing>(finalIntervalRSet);
	//	float* RSet = interR->getOptimalValue();
	//	for (size_t k = 0; k < finalIntervalRSet.size(); k += 2)
	//	{
	//		if (finalIntervalRSet[k].value > RSet[1])
	//			continue;
	//		if (finalIntervalRSet[k + 1].value < RSet[0])
	//			continue;
	//		indicesR.emplace_back(finalIntervalRSet[k].index);
	//	}
	//	optimalRSet[0] = RSet[0];
	//	optimalRSet[1] = RSet[1];
	//}



	pcl::Correspondences corsR;
	for (size_t i = 0; i < indicesR.size(); i++)
	{
		corsR.emplace_back(corsV[indicesR[i]]);
	}

	//auto timeA = std::chrono::system_clock::now();

	//std::cout << "Time consumption of Stag II: " <<
	//	double(std::chrono::duration_cast<std::chrono::microseconds>
	//		(timeA - timeS).count()) / 1000000.0 << std::endl;

	float directionT = 0.0f, directionS = 0.0f;
	//std::cout << optimalRSet[0] << "," << optimalRSet[1] << std::endl;
	float optimalR = (optimalRSet[0] + optimalRSet[1]) / 2;

	Interval intervalL;
	Intervals intervalLSet;
	Eigen::Matrix2f rotation;
	rotation << cos(optimalR), -sin(optimalR), sin(optimalR), cos(optimalR);

	std::vector<Eigen::Vector2f> pointSet;
	Eigen::Vector2f point;

	for (size_t i = 0; i < indicesR.size(); i++)
	{
		point = edgeSet[indicesR[i]].second - rotation * edgeSet[indicesR[i]].first;
		pointSet.emplace_back(point);

		intervalL.is_left = true;
		intervalL.value = point.norm() - resolution;
		intervalLSet.emplace_back(intervalL);
		intervalL.is_left = false;
		intervalL.value = point.norm() + resolution;
		intervalLSet.emplace_back(intervalL);
	}

	std::shared_ptr<IntervalStabbing> interL = std::make_shared<IntervalStabbing>(intervalLSet);
	float* optimalLSet = interL->getOptimalValue();

	pcl::Correspondences corsL;
	std::vector<size_t> indicesL;
	for (size_t i = 0; i < intervalLSet.size(); i += 2)
	{
		if (intervalLSet[i].value > optimalLSet[1])
			continue;
		if (intervalLSet[i + 1].value < optimalLSet[0])
			continue;

		corsL.emplace_back(corsR[i / 2]);
		indicesL.emplace_back(i / 2);
	}

	float optimalL = std::sqrtf((optimalLSet[0] + optimalLSet[1]) / 2);

	Interval intervalT;
	Intervals intervalTSet;
	float angleT = 0.0f;

	for (size_t i = 0; i < indicesL.size(); i++)
	{
		angleT = atan2f(pointSet[indicesL[i]].y(), pointSet[indicesL[i]].x());
		error1 = acosf((pointSet[indicesL[i]].squaredNorm() + std::powf(optimalL, 2) - std::powf(resolution, 2))
			/ (2 * pointSet[indicesL[i]].norm() * optimalL));
		
		intervalT.is_left = true;
		intervalT.value = angleT - error1 /*- error2*/;
		intervalTSet.emplace_back(intervalT);
		intervalT.is_left = false;
		intervalT.value = angleT + error1 /*+ error2*/;
		intervalTSet.emplace_back(intervalT);
	}

	std::shared_ptr<IntervalStabbing> interT = std::make_shared<IntervalStabbing>(intervalTSet);
	float* optimalTSet = interT->getOptimalValue();
	float optimalT = (optimalTSet[0] + optimalTSet[1]) / 2;
	//optimalTSet[0] = 2.830f;
	//optimalTSet[1] = 2.860f;

	//pcl::Correspondences corsT;
	//std::vector<size_t> indicesT;
	for (size_t i = 0; i < intervalTSet.size(); i += 2)
	{
		if (intervalTSet[i].value > optimalTSet[1])
			continue;
		if (intervalTSet[i + 1].value < optimalTSet[0])
			continue;

		corsRes.emplace_back(corsL[i / 2]);
		indicesL.emplace_back(indicesR[i / 2]);
	}
	auto timeE = std::chrono::system_clock::now();
	std::cout << "Time consumption of decouping: " <<
		double(std::chrono::duration_cast<std::chrono::microseconds>
			(timeE - timeS).count()) / 1000000.0 << std::endl;

	std::cout << "corsRes: " << corsRes.size() << std::endl;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ, double> te;
	te.estimateRigidTransformation(*sampledCloudS, *sampledCloudT, corsRes, transformation);

}

void Decoupling::decoupling2(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT, pcl::Correspondences cors, float resolution)
{
	std::cout << "==================Decoupling .....==================" << std::endl;
	auto timeS = std::chrono::system_clock::now();

	Intervals verticalTSet;
	Interval verticalT;

	Eigen::Vector3f s_t(0.0f, 0.0f, 0.0f);
	Eigen::Vector3f g(0.0f, 0.0f, 1.0f);
	for (unsigned int index = 0; index < cors.size(); index++)
	{
		s_t[0] = sampledCloudT->points[cors[index].index_match].x - sampledCloudS->points[cors[index].index_query].x;
		s_t[1] = sampledCloudT->points[cors[index].index_match].y - sampledCloudS->points[cors[index].index_query].y;
		s_t[2] = sampledCloudT->points[cors[index].index_match].z - sampledCloudS->points[cors[index].index_query].z;
		//s_tSet[index] = s_t;

		verticalT.value = -resolution - g.dot(s_t);
		verticalT.is_left = true;
		//verticalT.weight = weightInfor[index];
		verticalTSet.emplace_back(verticalT);

		verticalT.value = resolution - g.dot(s_t);
		verticalT.is_left = false;
		//verticalT.weight = weightInfor[index];
		verticalTSet.emplace_back(verticalT);
	}

	std::shared_ptr<IntervalStabbing> interV = std::make_shared<IntervalStabbing>(verticalTSet);

	float* optimalZSet = interV->getOptimalValue();
	//optimalZSet[0] = 1.40f;
	//optimalZSet[1] = 1.45f;
	pcl::Correspondences corsV;
	std::vector<size_t> indicesV;

	for (size_t i = 0; i < verticalTSet.size(); i += 2)
	{
		if (verticalTSet[i].value > optimalZSet[1])
			continue;
		if (verticalTSet[i + 1].value < optimalZSet[0])
			continue;

		corsV.emplace_back(cors[i / 2]);
		indicesV.emplace_back(i / 2);
	}

	Edge2DSet edgeSet;
	Edge2D edge;

	for (unsigned int index = 0; index < corsV.size(); index++)
	{
		edge.first[0] = sampledCloudS->points[corsV[index].index_query].x;
		edge.first[1] = sampledCloudS->points[corsV[index].index_query].y;

		edge.second[0] = sampledCloudT->points[corsV[index].index_match].x;
		edge.second[1] = sampledCloudT->points[corsV[index].index_match].y;
		edgeSet.emplace_back(edge);
	}

	//求方向

	Intervals horizontalLSet;
	Interval horizontalL;

	for (unsigned int index = 0; index < edgeSet.size(); index++)
	{
		horizontalL.is_left = true;
		horizontalL.value = std::abs(edgeSet[index].first.norm() - edgeSet[index].second.norm()) - resolution;
		horizontalLSet.emplace_back(horizontalL);
		horizontalL.is_left = false;
		horizontalL.value = edgeSet[index].first.norm() + edgeSet[index].second.norm() + resolution;
		horizontalLSet.emplace_back(horizontalL);
	}

	std::shared_ptr<IntervalStabbing> interL = std::make_shared<IntervalStabbing>(horizontalLSet);

	float* optimalLSet = interL->getOptimalValue();

	std::cout << optimalLSet[0] << "," << optimalLSet[1] << std::endl;
}

void Decoupling::computeTransformationMatrix(PointCloudPtr sampledCloudS, PointCloudPtr sampledCloudT)
{

	Eigen::Matrix4f transformation_matrix;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> te;
	te.estimateRigidTransformation(*sampledCloudS, *sampledCloudT, corsRes, transformation_matrix);

	std::cout << "transformation matrix before ICP: \n" << transformation_matrix << std::endl;
}


