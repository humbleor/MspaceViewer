#include "../include/Decoupling.h"

namespace
{
constexpr float anglePi = static_cast<float>(M_PI);
constexpr float twoAnglePi = 2.0f * anglePi;

void appendAngularInterval(
    float center,
    float halfWidth,
    Intervals& intervals,
    std::vector<size_t>* correspondenceIndices = nullptr,
    size_t correspondenceIndex = 0)
{
    if (!std::isfinite(center) || !std::isfinite(halfWidth) || halfWidth < 0.0f)
        return;

    const auto appendLinearInterval = [&](float left, float right)
    {
        intervals.push_back({ left, true });
        intervals.push_back({ right, false });
        if (correspondenceIndices)
        {
            correspondenceIndices->push_back(correspondenceIndex);
            correspondenceIndices->push_back(correspondenceIndex);
        }
    };

    if (halfWidth >= anglePi)
    {
        appendLinearInterval(-anglePi, anglePi);
        return;
    }

    center = std::remainder(center, twoAnglePi);
    const float left = center - halfWidth;
    const float right = center + halfWidth;

    if (left < -anglePi)
    {
        appendLinearInterval(-anglePi, right);
        appendLinearInterval(left + twoAnglePi, anglePi);
    }
    else if (right > anglePi)
    {
        appendLinearInterval(left, anglePi);
        appendLinearInterval(-anglePi, right - twoAnglePi);
    }
    else
    {
        appendLinearInterval(left, right);
    }
}
}

const pcl::Correspondences& Decoupling::getStageCorrespondences(
    std::size_t stageIndex) const noexcept
{
    switch (stageIndex)
    {
    case 0: return cors1;
    case 1: return cors2;
    case 2: return cors3;
    default: return corsRes;
    }
}

Decoupling::Decoupling(
    PointCloudPtr sampledCloudS,
    PointCloudPtr sampledCloudT,
    pcl::Correspondences cors,
    float resolution,
    const Eigen::Vector3d& gravityDirection,
    bool verbose,
    float angleResolutionDegrees,
    bool enableAngleCandidatePruning,
    bool swapVerticalAndRotationStages)
{
    if (!sampledCloudS || !sampledCloudT)
    {
        if (verbose)
            std::cerr << "Decoupling stopped: input cloud is null." << std::endl;
        return;
    }

    if (!gravityDirection.allFinite() ||
        gravityDirection.norm() <= std::numeric_limits<double>::epsilon())
    {
        if (verbose)
            std::cerr << "Decoupling stopped: gravity direction is invalid." << std::endl;
        return;
    }

    const Eigen::Vector3d normalizedGravity = gravityDirection.normalized();
    const Eigen::Matrix3d gravityAlignment =
        Eigen::Quaterniond::FromTwoVectors(
            normalizedGravity,
            Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Matrix4d alignmentTransformation = Eigen::Matrix4d::Identity();
    alignmentTransformation.block<3, 3>(0, 0) = gravityAlignment;

    PointCloudPtr alignedSource(new pcl::PointCloud<pcl::PointXYZ>);
    PointCloudPtr alignedTarget(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
        *sampledCloudS,
        *alignedSource,
        alignmentTransformation.cast<float>());
    pcl::transformPointCloud(
        *sampledCloudT,
        *alignedTarget,
        alignmentTransformation.cast<float>());

    decoupling(
        alignedSource,
        alignedTarget,
        cors,
        resolution,
        verbose,
        angleResolutionDegrees,
        enableAngleCandidatePruning,
        swapVerticalAndRotationStages);

    if (valid)
    {
        transformation =
            alignmentTransformation.inverse() *
            transformation *
            alignmentTransformation;
    }
}

void Decoupling::decoupling(
    PointCloudPtr sampledCloudS,
    PointCloudPtr sampledCloudT,
    const pcl::Correspondences& cors,
    float resolution,
    bool verbose,
    float angleResolutionDegrees,
    bool enableAngleCandidatePruning,
    bool swapVerticalAndRotationStages)
{
    stageTimesSeconds.fill(0.0);
    if (verbose)
        std::cout << "==================Decoupling .....==================" << std::endl;
    auto timeS = std::chrono::system_clock::now();

    if (!sampledCloudS || !sampledCloudT || sampledCloudS->empty() ||
        sampledCloudT->empty() || cors.empty() ||
        !std::isfinite(resolution) || resolution <= 0.0f ||
        !std::isfinite(angleResolutionDegrees) ||
        angleResolutionDegrees <= 0.0f)
    {
        if (verbose)
            std::cerr << "Decoupling stopped: invalid input or resolution." << std::endl;
        return;
    }

    auto stageStart = std::chrono::steady_clock::now();

    struct VerticalStageResult
    {
        pcl::Correspondences correspondences;
        std::vector<size_t> selectedIndices;
    };
    struct RotationStageResult
    {
        pcl::Correspondences correspondences;
        std::vector<size_t> selectedIndices;
        std::array<float, 2> optimalInterval{ 0.0f, 0.0f };
        size_t maximumConsensusSize{ 0 };
    };

    const auto buildEdges = [&](const pcl::Correspondences& input)
    {
        Edge2DSet edges;
        edges.reserve(input.size());
        for (const pcl::Correspondence& correspondence : input)
        {
            Edge2D edge;
            const auto& sourcePoint =
                sampledCloudS->points[correspondence.index_query];
            const auto& targetPoint =
                sampledCloudT->points[correspondence.index_match];
            edge.first[0] = sourcePoint.x;
            edge.first[1] = sourcePoint.y;
            edge.second[0] = targetPoint.x;
            edge.second[1] = targetPoint.y;
            edges.emplace_back(edge);
        }
        return edges;
    };

    const auto runVerticalStage = [&](const pcl::Correspondences& input)
    {
        VerticalStageResult result;
        Intervals intervals;
        intervals.reserve(input.size() * 2);
        for (const pcl::Correspondence& correspondence : input)
        {
            const float zDifference =
                sampledCloudT->points[correspondence.index_match].z -
                sampledCloudS->points[correspondence.index_query].z;
            intervals.push_back({ -resolution - zDifference, true });
            intervals.push_back({ resolution - zDifference, false });
        }
        if (intervals.empty())
            return result;
        IntervalStabbing stabbing(intervals);
        float* optimum = stabbing.getOptimalValue();
        result.correspondences.reserve(input.size());
        result.selectedIndices.reserve(input.size());
        for (size_t index = 0; index < intervals.size(); index += 2)
        {
            if (intervals[index].value > optimum[1])
                continue;
            if (intervals[index + 1].value < optimum[0])
                continue;
            result.correspondences.emplace_back(input[index / 2]);
            result.selectedIndices.emplace_back(index / 2);
        }
        return result;
    };

    const auto runRotationStage = [&](const pcl::Correspondences& input)
    {
        RotationStageResult result;
        const Edge2DSet edges = buildEdges(input);
        const float deltaAngle =
            angleResolutionDegrees * static_cast<float>(M_PI) / 180.0f;
        std::vector<bool> usedIndex(edges.size(), false);
        Intervals intervals;
        std::vector<size_t> intervalIndices;
        std::vector<size_t> supportedIndices;

        for (size_t anchor = 0; anchor < edges.size(); ++anchor)
        {
            if (usedIndex[anchor])
                continue;
            intervals.clear();
            intervalIndices.clear();
            for (size_t candidate = 0; candidate < edges.size(); ++candidate)
            {
                if (anchor == candidate || usedIndex[candidate])
                    continue;
                const float sourceDx =
                    edges[candidate].first.x() - edges[anchor].first.x();
                const float sourceDy =
                    edges[candidate].first.y() - edges[anchor].first.y();
                const float sourceDistance =
                    std::sqrtf(sourceDx * sourceDx + sourceDy * sourceDy);
                const float targetDx =
                    edges[candidate].second.x() - edges[anchor].second.x();
                const float targetDy =
                    edges[candidate].second.y() - edges[anchor].second.y();
                const float targetDistance =
                    std::sqrtf(targetDx * targetDx + targetDy * targetDy);
                if (std::fabs(sourceDistance - targetDistance) >= resolution)
                    continue;

                const float halfWidth = 2.0f * resolution < sourceDistance
                    ? asinf((2.0f * resolution) / sourceDistance)
                    : static_cast<float>(M_PI);
                const float sourceAngle = atan2f(sourceDy, sourceDx);
                const float targetAngle = atan2f(targetDy, targetDx);
                float theta = targetAngle - sourceAngle;
                if (theta > M_PI)
                    theta -= 2.0f * static_cast<float>(M_PI);
                if (theta < -M_PI)
                    theta += 2.0f * static_cast<float>(M_PI);
                appendAngularInterval(
                    theta,
                    halfWidth,
                    intervals,
                    &intervalIndices,
                    candidate);
            }
            if (intervals.size() < 2)
                continue;
            IntervalStabbing stabbing(intervals);
            float* optimum = stabbing.getOptimalValue();
            if (stabbing.getNStabbed() < 2)
                continue;

            supportedIndices.clear();
            supportedIndices.emplace_back(anchor);
            for (size_t index = 0; index + 1 < intervals.size(); index += 2)
            {
                if (intervals[index].value > optimum[1])
                    continue;
                if (intervals[index + 1].value < optimum[0])
                    continue;
                supportedIndices.emplace_back(intervalIndices[index]);
                if (enableAngleCandidatePruning &&
                    (intervals[index + 1].value - intervals[index].value) <
                        deltaAngle)
                {
                    usedIndex[intervalIndices[index]] = true;
                }
            }
            if (supportedIndices.size() > result.maximumConsensusSize)
            {
                result.maximumConsensusSize = supportedIndices.size();
                result.selectedIndices = supportedIndices;
                result.optimalInterval[0] = optimum[0];
                result.optimalInterval[1] = optimum[1];
            }
        }

        result.correspondences.reserve(result.selectedIndices.size());
        for (size_t index : result.selectedIndices)
            result.correspondences.emplace_back(input[index]);
        return result;
    };

    pcl::Correspondences corsR;
    Edge2DSet edgeSet;
    std::vector<size_t> indicesR;
    std::array<float, 2> optimalRSet{ 0.0f, 0.0f };
    size_t maxNum = 0;
    auto stageEnd = stageStart;

    if (!swapVerticalAndRotationStages)
    {
        const VerticalStageResult verticalResult = runVerticalStage(cors);
        cors1 = verticalResult.correspondences;
        stageEnd = std::chrono::steady_clock::now();
        stageTimesSeconds[0] =
            std::chrono::duration<double>(stageEnd - stageStart).count();
        stageStart = stageEnd;

        const RotationStageResult rotationResult = runRotationStage(cors1);
        cors2 = rotationResult.correspondences;
        corsR = cors2;
        edgeSet = buildEdges(cors1);
        indicesR = rotationResult.selectedIndices;
        optimalRSet = rotationResult.optimalInterval;
        maxNum = rotationResult.maximumConsensusSize;
        stageEnd = std::chrono::steady_clock::now();
        stageTimesSeconds[1] =
            std::chrono::duration<double>(stageEnd - stageStart).count();
    }
    else
    {
        const RotationStageResult rotationResult = runRotationStage(cors);
        cors1 = rotationResult.correspondences;
        optimalRSet = rotationResult.optimalInterval;
        maxNum = rotationResult.maximumConsensusSize;
        stageEnd = std::chrono::steady_clock::now();
        stageTimesSeconds[0] =
            std::chrono::duration<double>(stageEnd - stageStart).count();
        stageStart = stageEnd;

        const VerticalStageResult verticalResult = runVerticalStage(cors1);
        cors2 = verticalResult.correspondences;
        corsR = cors2;
        edgeSet = buildEdges(cors1);
        indicesR = verticalResult.selectedIndices;
        stageEnd = std::chrono::steady_clock::now();
        stageTimesSeconds[1] =
            std::chrono::duration<double>(stageEnd - stageStart).count();
    }

    if (verbose)
    {
        std::cout << "Stage I correspondences: " << cors1.size() << '\n';
        std::cout << "Stage II correspondences: " << cors2.size() << '\n';
        std::cout << "rotation interval: [" << optimalRSet[0] << ", "
            << optimalRSet[1] << "], consensus: " << maxNum << std::endl;
    }

    if (corsR.empty())
    {
        if (verbose)
            std::cerr << "Decoupling stopped: rotation stage produced no valid correspondences." << std::endl;
        return;
    }

    stageStart = std::chrono::steady_clock::now();

    // compute rotation midpoint and rotation matrix
    float optimalR = (optimalRSet[0] + optimalRSet[1]) * 0.5f;
    Eigen::Matrix2f rotation;
    const float cr = std::cos(optimalR), sr = std::sin(optimalR);
    rotation << cr, -sr, sr, cr;

    // L-stage: compute pointSet and intervals
    std::vector<Eigen::Vector2f> pointSet;
    pointSet.reserve(indicesR.size());
    Intervals intervalLSet;
    for (size_t idx = 0; idx < indicesR.size(); ++idx)
    {
        const auto& e = edgeSet[indicesR[idx]];
        Eigen::Vector2f p = e.second - rotation * e.first;
        pointSet.emplace_back(p);

        Interval il;
        il.is_left = true;
        il.value = p.norm() - resolution;
        intervalLSet.emplace_back(il);
        il.is_left = false;
        il.value = p.norm() + resolution;
        intervalLSet.emplace_back(il);
    }

    std::shared_ptr<IntervalStabbing> interL = std::make_shared<IntervalStabbing>(intervalLSet);
    float* optimalLSet = interL->getOptimalValue();

    if (interL->getNStabbed() <= 0.0f)
    {
        stageEnd = std::chrono::steady_clock::now();
        stageTimesSeconds[2] =
            std::chrono::duration<double>(stageEnd - stageStart).count();
        if (verbose)
            std::cerr << "Decoupling stopped: translation-length stage produced no valid interval." << std::endl;
        return;
    }

    pcl::Correspondences corsL;
    std::vector<size_t> indicesL;
    for (size_t i = 0; i < intervalLSet.size(); i += 2)
    {
        if (intervalLSet[i].value > optimalLSet[1]) continue;
        if (intervalLSet[i + 1].value < optimalLSet[0]) continue;
        corsL.emplace_back(corsR[i / 2]);
        indicesL.emplace_back(i / 2);
    }
	cors3 = corsL; // store scale inliers as cors3
	// intervalLSet stores p.norm() +/- resolution, so its midpoint is already a length.
	// Taking another square root changes the unit and can invalidate the next acos call.
	const float optimalL = std::max(0.0f, (optimalLSet[0] + optimalLSet[1]) * 0.5f);
    horizontalTranslationLength = optimalL;
    stageEnd = std::chrono::steady_clock::now();
    stageTimesSeconds[2] =
        std::chrono::duration<double>(stageEnd - stageStart).count();
    stageStart = stageEnd;
    if (verbose)
        std::cout << "intervalLSet size: " << intervalLSet.size() << ", optimalLSet: [" << optimalLSet[0] << ", " << optimalLSet[1] << "], nStabbed: " << interL->getNStabbed() << std::endl;
    // T-stage
    Intervals intervalTSet;
    std::vector<size_t> intervalTIndices;
    for (size_t idx = 0; idx < indicesL.size(); ++idx)
    {
        const Eigen::Vector2f& pv = pointSet[indicesL[idx]];
        const float angleT = atan2f(pv.y(), pv.x());
        const float pointNorm = pv.norm();
        const float numer = pointNorm * pointNorm + optimalL * optimalL - resolution * resolution;
        const float denom = 2.0f * pointNorm * optimalL;

        constexpr float numericTolerance = 1e-5f;
        float error1 = 0.0f;

        if (denom <= std::numeric_limits<float>::epsilon())
        {
            // A zero-length vector has no direction. Accept all angles only if
            // the remaining length constraint is still feasible.
            if (std::fabs(pointNorm - optimalL) > resolution)
                continue;
            error1 = static_cast<float>(M_PI);
        }
        else
        {
            float acosArgument = numer / denom;
            if (!std::isfinite(acosArgument) || acosArgument > 1.0f + numericTolerance)
                continue;

            if (acosArgument < -1.0f - numericTolerance)
            {
                error1 = static_cast<float>(M_PI);
            }
            else
            {
                // Clamp only small floating-point drift; infeasible constraints
                // have already been rejected above.
                acosArgument = std::clamp(acosArgument, -1.0f, 1.0f);
                error1 = std::acos(acosArgument);
            }
        }

        appendAngularInterval(
            angleT,
            error1,
            intervalTSet,
            &intervalTIndices,
            idx);
    }

    if (intervalTSet.empty())
    {
        stageEnd = std::chrono::steady_clock::now();
        stageTimesSeconds[3] =
            std::chrono::duration<double>(stageEnd - stageStart).count();
        if (verbose)
            std::cerr << "Decoupling stopped: translation-direction stage produced no valid interval." << std::endl;
        return;
    }

    std::shared_ptr<IntervalStabbing> interT = std::make_shared<IntervalStabbing>(intervalTSet);
    float* optimalTSet = interT->getOptimalValue();

    if (interT->getNStabbed() <= 0.0f)
    {
        stageEnd = std::chrono::steady_clock::now();
        stageTimesSeconds[3] =
            std::chrono::duration<double>(stageEnd - stageStart).count();
        if (verbose)
            std::cerr << "Decoupling stopped: translation-direction stage found no consensus." << std::endl;
        return;
    }

    corsRes.clear();
    corsRes.reserve(intervalTSet.size() / 2);
    for (size_t i = 0; i < intervalTSet.size(); i += 2)
    {
        if (intervalTSet[i].value > optimalTSet[1]) continue;
        if (intervalTSet[i + 1].value < optimalTSet[0]) continue;
        corsRes.emplace_back(corsL[intervalTIndices[i]]);
    }
    stageEnd = std::chrono::steady_clock::now();
    stageTimesSeconds[3] =
        std::chrono::duration<double>(stageEnd - stageStart).count();
    auto timeE = std::chrono::system_clock::now();
    if (verbose)
    {
        std::cout << "Time consumption of decouping: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(timeE - timeS).count()) / 1e6
            << " s\n";
        std::cout << "corsRes: " << corsRes.size() << std::endl;
    }

    if (corsRes.size() < 3)
    {
        if (verbose)
            std::cerr << "Decoupling stopped: fewer than three correspondences remain for 4-DOF estimation." << std::endl;
        return;
    }

    Eigen::Vector3d sourceCentroid = Eigen::Vector3d::Zero();
    Eigen::Vector3d targetCentroid = Eigen::Vector3d::Zero();
    for (const auto& correspondence : corsRes)
    {
        const auto& sourcePoint = sampledCloudS->points[correspondence.index_query];
        const auto& targetPoint = sampledCloudT->points[correspondence.index_match];
        sourceCentroid += Eigen::Vector3d(sourcePoint.x, sourcePoint.y, sourcePoint.z);
        targetCentroid += Eigen::Vector3d(targetPoint.x, targetPoint.y, targetPoint.z);
    }
    sourceCentroid /= static_cast<double>(corsRes.size());
    targetCentroid /= static_cast<double>(corsRes.size());

    double dotSum = 0.0;
    double crossSum = 0.0;
    for (const auto& correspondence : corsRes)
    {
        const auto& sourcePoint = sampledCloudS->points[correspondence.index_query];
        const auto& targetPoint = sampledCloudT->points[correspondence.index_match];
        const double sourceX = sourcePoint.x - sourceCentroid.x();
        const double sourceY = sourcePoint.y - sourceCentroid.y();
        const double targetX = targetPoint.x - targetCentroid.x();
        const double targetY = targetPoint.y - targetCentroid.y();
        dotSum += sourceX * targetX + sourceY * targetY;
        crossSum += sourceX * targetY - sourceY * targetX;
    }

    if (!std::isfinite(dotSum) || !std::isfinite(crossSum) ||
        std::hypot(dotSum, crossSum) <= std::numeric_limits<double>::epsilon())
    {
        if (verbose)
            std::cerr << "Decoupling stopped: degenerate 4-DOF rotation estimate." << std::endl;
        return;
    }

    const double refinedYaw = std::atan2(crossSum, dotSum);
    Eigen::Matrix3d refinedRotation = Eigen::Matrix3d::Identity();
    refinedRotation(0, 0) = std::cos(refinedYaw);
    refinedRotation(0, 1) = -std::sin(refinedYaw);
    refinedRotation(1, 0) = std::sin(refinedYaw);
    refinedRotation(1, 1) = std::cos(refinedYaw);

    transformation.setIdentity();
    transformation.block<3, 3>(0, 0) = refinedRotation;
    transformation.block<3, 1>(0, 3) =
        targetCentroid - refinedRotation * sourceCentroid;
    valid = transformation.allFinite();
}
