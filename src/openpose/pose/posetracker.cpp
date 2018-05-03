#include <openpose/pose/posetracker.hpp>
#include <openpose/utilities/keypoint.hpp>

namespace op
{
    PoseTracker::PoseTracker(const Array<float>& posekeyPoints , const int numberPeopleDetected , const int numberBodyParts):  
        mNumberPeople{numberPeopleDetected},
        mNumberBodyParts{numberBodyParts},
        //mMaxPosePeople{0},
        //mMaxPoseDistance{0.0},
        mNormalizeKeypoints{posekeyPoints},  //score = 0, just not necessary to initialize
        mThreShold{0.05}
    {
        mRectangle.reset({numberPeopleDetected, 4});   //
        //mResizeRectangle.reset({numberPeopleDetected, 4});
        mCoordinate.reset({numberPeopleDetected, 2});
        mMeanPose.reset({numberBodyParts, 2});
        mDistancePose.reset({numberPeopleDetected});
        mBiggestRectangle.reset({2});  //biggest width and height
    }

    PoseTracker::~PoseTracker()
    {
    }   


    Array<float> PoseTracker::resizeKeypointsRectangle(const Array<float>& posekeyPoints, const float threshold)
    {
        try
        {
            auto resizeKeypoints = posekeyPoints.clone();

            const int biggestPerson = getBiggestPerson(posekeyPoints, threshold);
            mBiggestRectangle[0] = getKeypointsRectangle(posekeyPoints, biggestPerson, threshold).width;
            mBiggestRectangle[1] = getKeypointsRectangle(posekeyPoints, biggestPerson, threshold).height;

            //const auto mBiggestWidth = getKeypointsRectangle(posekeyPoints, biggestPerson, threshold).width;
            //const auto mBiggestHeight = getKeypointsRectangle(posekeyPoints, biggestPerson, threshold).height;

            for(auto numpeople = 0; numpeople < mNumberPeople; numpeople++)
            {
                mRectangle[numpeople * 4] = getKeypointsRectangle(posekeyPoints, numpeople, threshold).x;
                mRectangle[numpeople * 4 + 1] = getKeypointsRectangle(posekeyPoints, numpeople, threshold).y;
                mRectangle[numpeople * 4 + 2] = getKeypointsRectangle(posekeyPoints, numpeople, threshold).width;
                mRectangle[numpeople * 4 + 3] = getKeypointsRectangle(posekeyPoints, numpeople, threshold).height;

                auto ratioWidth = mBiggestRectangle[0] /  mRectangle[numpeople * 4 + 2];
                auto ratioHeight = mBiggestRectangle[1] / mRectangle[numpeople * 4 + 3];

                for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                {
                    auto baseIndex = posekeyPoints.getSize(2) * (numpeople * mNumberBodyParts + numbody);

                    if(posekeyPoints[baseIndex + 2] > mThreShold)
                    {
                        resizeKeypoints[baseIndex] = (posekeyPoints[baseIndex] - mRectangle[numpeople * 4]) * ratioWidth + mRectangle[numpeople * 4];
                        resizeKeypoints[baseIndex + 1] = (posekeyPoints[baseIndex + 1] - mRectangle[numpeople * 4 + 1]) * ratioHeight + mRectangle[numpeople * 4 + 1];
                    }
                }
            }

            return resizeKeypoints;
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return Array<float>{};
        }
    }


    Array<float> PoseTracker::normalizeKeyPoints(const Array<float>& posekeyPoints)
    {
        try
        {
             mNormalizeKeypoints = resizeKeypointsRectangle(posekeyPoints, mThreShold); //resize the rectangle [0~]

             for(auto numpeople = 0; numpeople < mNumberPeople; numpeople++)
             {
                 for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                 {
                     auto baseIndex = posekeyPoints.getSize(2) * (numpeople * mNumberBodyParts + numbody);

                     if(posekeyPoints[baseIndex+2] > mThreShold)
                     {
                         mNormalizeKeypoints[baseIndex] = (posekeyPoints[baseIndex] - mRectangle[numpeople * 4]) / mBiggestRectangle[0];
                         mNormalizeKeypoints[baseIndex+1] = (posekeyPoints[baseIndex+1] - mRectangle[numpeople * 4 + 1]) / mBiggestRectangle[1];
                     }
                 }
             }

             return mNormalizeKeypoints;
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return posekeyPoints;
        }
    }

    Array<float> PoseTracker::normalizeKeyPoints(const Array<float>& posekeyPoints, const float threshold)
    {
        try
        {
            mNormalizeKeypoints = posekeyPoints.clone();
            for(auto numpeople = 0; numpeople < posekeyPoints.getSize(0); numpeople++)
            {
                Rectangle<float> rect = getKeypointsRectangle(posekeyPoints, numpeople, threshold);

                for(auto numbody = 0; numbody < posekeyPoints.getSize(1); numbody++)
                {
                    auto baseIndex = posekeyPoints.getSize(2) * (numpeople * mNumberBodyParts + numbody);

                    if(posekeyPoints[baseIndex+2] > threshold)
                    {
                        mNormalizeKeypoints[baseIndex] = (posekeyPoints[baseIndex] - rect.x) / rect.width;
                        mNormalizeKeypoints[baseIndex + 1] = (posekeyPoints[baseIndex + 1] - rect.y) / rect.height;
                    }
                }
            }

            return mNormalizeKeypoints;
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return posekeyPoints;
        }
    }


    void PoseTracker::meanPose(Array<float>& newKeypoints)
    {
        try
        {
            for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
            {

                auto peopleIndex = 0;
                float sumX = 0;
                float sumY = 0;

                for(auto numpeople = 0; numpeople < mNumberPeople; numpeople++)
                {
                    auto baseIndex = newKeypoints.getSize(2) * (numpeople * mNumberBodyParts + numbody);
                    auto score = newKeypoints[baseIndex+2];


                    if(score > mThreShold)
                    {
                        peopleIndex++;

                        sumX += newKeypoints[baseIndex];
                        sumY += newKeypoints[baseIndex+1];
                    }
                }

                if(peopleIndex != 0)
                {
                    mMeanPose[numbody * 2] = sumX / peopleIndex;
                    mMeanPose[numbody * 2 + 1] = sumY / peopleIndex;

                }

                else
                {
                    mMeanPose[numbody * 2] = 0;
                    mMeanPose[numbody * 2 + 1] = 0;    //the keypoints for everyone is 0, so compare the distance is always 0
                }
            }
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }

    }


    void PoseTracker::comparePose(Array<float>& newKeypoints, const float threshold)
    {
        try
        {
            meanPose(newKeypoints);
            mAbnormalPose.clear();

            for(auto numpeople = 0; numpeople < mNumberPeople; numpeople++)
            {
                for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                {
                    auto baseIndex = newKeypoints.getSize(2) * (numpeople * mNumberBodyParts + numbody);

                    auto valueX = newKeypoints[baseIndex];
                    auto valueY = newKeypoints[baseIndex+1];
                    auto score = newKeypoints[baseIndex+2];
                    auto distanceX = valueX - mMeanPose[numbody * 2];
                    auto distanceY = valueY - mMeanPose[numbody * 2 + 1];

                    if(score > mThreShold)
                    {
                        mDistancePose[numpeople] += (distanceX * distanceX) + (distanceY * distanceY);
                        //mMaxPoseDistance = 1;
                    }
                }
                if(mDistancePose[numpeople] > threshold)
                {
                    mAbnormalPose.emplace_back(numpeople);
                }
            }

            //for(auto numPeople = 0; numPeople < mNumberPeople; numPeople++)
            //{
                //std::cout<<"Simple Pose Distance : "<<std::endl;
                //std::cout<<numPeople<<" : "<<mDistancePose[numPeople]<<std::endl;    //output the simple distance
                //std::cout<<std::endl;

//                else
//                {
//                    this->mMaxPoseDistance = mDistancePose[numPeople];
//                    this->mMaxPosePeople = numPeople + 1;
//                }
            //}
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void PoseTracker::findSimplePose(const Array<float>& posekeyPoints, const float threshold)
    {
        try
        {
            auto normalizekeypoints = normalizeKeyPoints(posekeyPoints);
            comparePose(normalizekeypoints, threshold);
            //auto checkdata = normalizekeypoints[3];
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }

    }


    void PoseTracker:: swapArrayData(Array<float>& newposekeypoints, const int formerPeople, const int newpeople)
    {
        try
        {
            std::array<float,3> ar = {0};
            for(auto bodyparts = 0; bodyparts < newposekeypoints.getSize(1); bodyparts++)
            {
                auto formerPeopleIndex = 3 * (newposekeypoints.getSize(1) * formerPeople + bodyparts);
                auto newPeopleIndex = 3 * (newposekeypoints.getSize(1) * newpeople + bodyparts);

                for(auto i = 0; i < 3; i++)
                {
                    ar[i] = newposekeypoints[formerPeopleIndex + i];
                    newposekeypoints[formerPeopleIndex + i] = newposekeypoints[newPeopleIndex + i];
                    newposekeypoints[newPeopleIndex + i] = ar[i];
                }
            }
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }


    void PoseTracker::addZeroToArray(Array<float>& notResizeKeypoints, const int numLine)
    {
        try
        {
            Array<float> addZeroKeypoints({notResizeKeypoints.getSize(0) + 1, notResizeKeypoints.getSize(1), 3});
            if(numLine <= notResizeKeypoints.getSize(0) + 2)
            {
                auto index = 3 * notResizeKeypoints.getSize(1) * numLine;  //because numLine is start with 0;
                auto bodyIndex = 3 * notResizeKeypoints.getSize(1);
                auto finalIndex = 3 * notResizeKeypoints.getSize(0) * notResizeKeypoints.getSize(1);
                for(auto i = 0; i < index; i++)
                {
                    addZeroKeypoints[i] = notResizeKeypoints[i];
                }
                for(auto j = index; j < index + bodyIndex; j++)
                {
                    addZeroKeypoints[j] = 0.f;
                }

                if(numLine <= notResizeKeypoints.getSize(0) - 1)
                {
                    for(auto k = index; k < finalIndex; k++)
                    {
                        addZeroKeypoints[k + bodyIndex] = notResizeKeypoints[k];
                    }
                }

                notResizeKeypoints.reset({addZeroKeypoints.getSize(0), addZeroKeypoints.getSize(1), 3});
                notResizeKeypoints = addZeroKeypoints;
            }
            else
                error("the inset people number is overtop", __LINE__, __FUNCTION__, __FILE__);
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }


    bool PoseTracker::checkZeroPeople(Array<float>& checkKeypoints, const int numberPeople)
    {
        try
        {
            if(numberPeople <= checkKeypoints.getSize(0) - 1)
            {
                int nonZeroCount = 0;
                auto baseIndex = 3 * checkKeypoints.getSize(1) * numberPeople;
                for(auto i = 0; i < checkKeypoints.getSize(1); i++)
                {
                    if(checkKeypoints[baseIndex + 2 + (3 * i)] != 0)
                        nonZeroCount++;
                }
                if(nonZeroCount <= 3)
                    return true;
                else
                    return false;
            }
            else
            {
                error("the input numberPeople is wrong", __LINE__, __FUNCTION__, __FILE__);
                return false;
            }
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return false;
        }
    }


    Array<float> PoseTracker::matchRectangle(const Array<float>& formerKeypoints, const Array<float>& poseKeypoints)
    {
        try
        {
            Array<float> newPosekeypoints = poseKeypoints.clone();

            //1. for all the people of formerKeypoints, detect the corresponding in posekeyPoints;
            for(auto formerNumPeople = 0; formerNumPeople < formerKeypoints.getSize(0); formerNumPeople++)
            {
                Rectangle<float> formerRectangle = getKeypointsRectangle(formerKeypoints, formerNumPeople, mThreShold);
                std::vector<int> count(newPosekeypoints.getSize(0));
                std::vector<int> scorecount(newPosekeypoints.getSize(0));
                int currentNumpeople = 0;

                std::cout<<"former keypoints numpeople : "<<formerNumPeople<<std::endl;
                std::cout<<"pose keypoints count : "<<std::endl;

                //1.1 find in the poseKeypoints
                for(auto nowNumPeople = 0; nowNumPeople < newPosekeypoints.getSize(0); nowNumPeople++)
                {
                    for(auto bodyparts = 0; bodyparts < mNumberBodyParts; bodyparts++)
                    {
                        auto baseIndex = 3 * (nowNumPeople * mNumberBodyParts + bodyparts);
                        if((newPosekeypoints[baseIndex] >= formerRectangle.x) && (newPosekeypoints[baseIndex] <= formerRectangle.x + formerRectangle.width))
                        {
                            if((newPosekeypoints[baseIndex + 1] >= formerRectangle.y) && (newPosekeypoints[baseIndex + 1] <= formerRectangle.y + formerRectangle.height))
                                count[nowNumPeople] ++;
                        }
                        if(newPosekeypoints[baseIndex + 2] != 0)
                            scorecount[nowNumPeople] ++;
                    }
                    std::cout<<nowNumPeople<<" : "<<count[nowNumPeople]<<"  ";
                }
                std::cout<<std::endl;

                //1.2 find the biggest confidence people corresponding in poseKeypoints
                for(auto people = 1; people < newPosekeypoints.getSize(0); people++)
                {
                    if(count[currentNumpeople] < count[people])
                        currentNumpeople = people;
                }

                std::cout<<"match number : "<<currentNumpeople<<std::endl;

                //1.3 swap and add data
                if(currentNumpeople != formerNumPeople)     //if not, must not to exchange
                {
                    if(count[currentNumpeople] >= int(scorecount[currentNumpeople] / 3))
                    {
                        if(formerNumPeople <= newPosekeypoints.getSize(0) - 1)   //change the currentNumPeople to formerNumPeople, if false, it will overflow
                        {
                            swapArrayData(newPosekeypoints, formerNumPeople, currentNumpeople);    //change the numPeople and number line
                        }
                        else
                        {
                            for(auto i = newPosekeypoints.getSize(0); i < formerNumPeople + 1; i++)
                            {
                                addZeroToArray(newPosekeypoints, i);
                            }
                            swapArrayData(newPosekeypoints, formerNumPeople, currentNumpeople);
                        }
                    }
                    else
                    {
                        addZeroToArray(newPosekeypoints, formerNumPeople);
                    }
                }


                //1.4 output data after resize
                for(auto numpeople = 0; numpeople < newPosekeypoints.getSize(0); numpeople++)  //output normalizekeypoints data
                {
                    for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                    {
                        for(auto i = 0; i < 3; i++)
                        {
                            auto baseIndex = 3 * (numpeople * mNumberBodyParts + numbody) + i;
                            std::cout<<newPosekeypoints[baseIndex]<<"  ";
                        }
                        std::cout<<std::endl;
                    }
                    std::cout<<std::endl;
                    std::cout<<std::endl;
                }
            }

            mResizeRectangle.reset({newPosekeypoints.getSize(0), 4});
            //2. store the rectangle data after resize the data order
            for(auto numPeople = 0; numPeople < newPosekeypoints.getSize(0); numPeople++)
            {
                if(!checkZeroPeople(newPosekeypoints, numPeople))
                {
                    mResizeRectangle[4 * numPeople] = getKeypointsRectangle(newPosekeypoints, numPeople, mThreShold).x;
                    mResizeRectangle[4 * numPeople + 1] = getKeypointsRectangle(newPosekeypoints, numPeople, mThreShold).y;
                    mResizeRectangle[4 * numPeople + 2] = getKeypointsRectangle(newPosekeypoints, numPeople, mThreShold).width;
                    mResizeRectangle[4 * numPeople + 3] = getKeypointsRectangle(newPosekeypoints, numPeople, mThreShold).height;
                }
                else
                {
                    mResizeRectangle[4 * numPeople] = 0.f;
                    mResizeRectangle[4 * numPeople + 1] = 0.f;
                    mResizeRectangle[4 * numPeople + 2] = 0.f;
                    mResizeRectangle[4 * numPeople + 3] = 0.f;
                }
            }

            //return the final posekeypoints after resize the data order
            return newPosekeypoints;
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
            return poseKeypoints;
        }
    }

    //std::vector<int> PoseTracker::findDifferentPose(const Array<float>& formerKeypoints, const Array<float>& posekeyPoints, const float threshold)
    void PoseTracker::findDifferentPose(const Array<float>& formerKeypoints, const Array<float>& posekeyPoints, const float threshold)
    {
        try
        {
            auto resizeKeypoints = matchRectangle(formerKeypoints, posekeyPoints);
            auto normalizeFormerkeypoints = normalizeKeyPoints(formerKeypoints, mThreShold);
            auto normalizeResizeKeypoints = normalizeKeyPoints(resizeKeypoints, mThreShold);
            //std::vector<int> maxPosePeople;
            mDifferentPose.clear();
            int smallerPeople;
            if(resizeKeypoints.getSize(0) <= formerKeypoints.getSize(0))
                smallerPeople = resizeKeypoints.getSize(0);
            else
                smallerPeople = formerKeypoints.getSize(0);

            op::Array<float> distancePoseFormer(smallerPeople);

            for(auto numpeople = 0; numpeople < smallerPeople; numpeople++)  //if the people number of newPoseKeypoints is less than formerKeypoints, draw the rectangle is all 0;
            {
                if(!checkZeroPeople(resizeKeypoints, numpeople))    //ZeroPeople don't need compare
                {
                    for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                    {
                        auto baseIndex = normalizeResizeKeypoints.getSize(2) * (numpeople * mNumberBodyParts + numbody);

                        auto distanceX = std::fabs(normalizeResizeKeypoints[baseIndex] - normalizeFormerkeypoints[baseIndex]);
                        auto distanceY = std::fabs(normalizeResizeKeypoints[baseIndex + 1] - normalizeFormerkeypoints[baseIndex + 1]);

                        distancePoseFormer[numpeople] += (distanceX * distanceX) + (distanceY * distanceY);
                    }

                    //std::cout<<"distancePoseFormer : "<<std::endl;
                    //std::cout<<distancePoseFormer[numpeople]<<std::endl;
                    //std::cout<<"distancePoseFormer  display done. "<<std::endl;

                    if(distancePoseFormer[numpeople] >= threshold)
                    {

                        mDifferentPose.emplace_back(numpeople);
                        //mCoordinate[2 * numpeople] = getKeypointsRectangle(resizeKeypoints, numpeople, mThreShold).x + (getKeypointsRectangle(resizeKeypoints, numpeople, mThreShold).width) / 2;
                        //mCoordinate[2 * numpeople + 1] = getKeypointsRectangle(resizeKeypoints, numpeople, mThreShold).y + (getKeypointsRectangle(resizeKeypoints, numpeople, mThreShold).height) / 2;

                        //std::cout<<"Different Pose Distance : "<<std::endl;
                        //std::cout<<numpeople<<" : "<<distancePoseFormer[numpeople]<<std::endl;
                        //std::cout<<std::endl;
                    }
                }

            }

            //return maxPosePeople;
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

    void PoseTracker::displayData(const Array<float>& formerKeypoints, const Array<float>& posekeyPoints)
    {
        try
        {
            auto newPoseKeypoints = matchRectangle(formerKeypoints, posekeyPoints);

            std::cout<<"Keypoints after match rectangle : "<<std::endl;
            for(auto numpeople = 0; numpeople < newPoseKeypoints.getSize(0); numpeople++)  //output normalizekeypoints
            {
                for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                {
                    for(auto i = 0; i < 3; i++)
                    {
                        auto baseIndex = 3 * (numpeople * mNumberBodyParts + numbody) + i;
                        std::cout<<newPoseKeypoints[baseIndex]<<"  ";
                    }
                    std::cout<<std::endl;
                }
                std::cout<<std::endl;
                std::cout<<std::endl;
            }

            std::cout<<"former Keypoints : "<<std::endl;
            for(auto numpeople = 0; numpeople < formerKeypoints.getSize(0); numpeople++)  //output normalizekeypoints
            {
                for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                {
                    for(auto i = 0; i < 3; i++)
                    {
                        auto baseIndex = 3 * (numpeople * mNumberBodyParts + numbody) + i;
                        std::cout<<formerKeypoints[baseIndex]<<"  ";
                    }
                    std::cout<<std::endl;
                }
                std::cout<<std::endl;
                std::cout<<std::endl;
            }

            std::cout<<"current Keypoints : "<<std::endl;
            for(auto numpeople = 0; numpeople < posekeyPoints.getSize(0); numpeople++)  //output normalizekeypoints
            {
                for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
                {
                    for(auto i = 0; i < 3; i++)
                    {
                        auto baseIndex = 3 * (numpeople * mNumberBodyParts + numbody) + i;
                        std::cout<<posekeyPoints[baseIndex]<<"  ";
                    }
                    std::cout<<std::endl;
                }
                std::cout<<std::endl;
                std::cout<<std::endl;
            }
        }

        catch (const std::exception& e)
        {
            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
        }
    }

//    void PoseTracker::displayData()
//    {
//        try
//        {
//            for(auto numpeople = 0; numpeople < mNumberPeople; numpeople++)  //output normalizekeypoints
//            {
//                for(auto numbody = 0; numbody < mNumberBodyParts; numbody++)
//                {
//                    for(auto i = 0; i < 3; i++)
//                    {
//                        auto baseIndex = 3 * (numpeople * mNumberBodyParts + numbody) + i;
//                        std::cout<<mNormalizeKeypoints[baseIndex]<<"  ";
//                    }
//                    std::cout<<std::endl;
//                }
//                std::cout<<std::endl;
//                std::cout<<std::endl;
//            }

//            std::cout<<std::endl;

//            std::cout<<"abnormalPose display : "<<std::endl;

//            for(auto iter = mAbnormalPose.cbegin(); iter != mAbnormalPose.cend(); iter++)
//            {
//                std::cout<<(*iter)<<" ";
//            }

//            std::cout<<std::endl;

//            std::cout<<"differentPose display : "<<std::endl;

//            for(auto iter = mDifferentPose.cbegin(); iter != mDifferentPose.cend(); iter++)
//            {
//                std::cout<<(*iter)<<" ";
//            }

//            std::cout<<std::endl;
//            std::cout<<std::endl;
//        }

//        catch (const std::exception& e)
//        {
//            error(e.what(), __LINE__, __FUNCTION__, __FILE__);
//        }
//    }




}
