#ifndef OPENPOSE_POSE_POSE_TRACKER_HPP
#define OPENPOSE_POSE_POSE_TRACKER_HPP


#include <openpose/core/common.hpp>
#include <iostream>


namespace op
{
    class PoseTracker
    {
    public:

        std::vector<int> mAbnormalPose;
        std::vector<int> mDifferentPose;
        Array<float> mRectangle;     //4 =x, y, width, height;
        Array<float> mResizeRectangle;
        Array<float> mCoordinate;


        PoseTracker(const Array<float>& posekeyPoints , const int numberPeopleDetected , const int numberBodyParts);

        ~PoseTracker();

        Array<float> resizeKeypointsRectangle(const Array<float>& posekeyPoints, const float threshold);

        Array<float> normalizeKeyPoints(const Array<float>& posekeyPoints);

        Array<float> normalizeKeyPoints(const Array<float>& posekeyPoints, const float threshold);

        //void findMaxAndMin(const Array<float>& posekeyPoints);

        void meanPose(Array<float>& newKeypoints);  //

        void comparePose(Array<float>& newKeypoints, const float threshold);

        void findSimplePose(const Array<float>& posekeyPoints, const float threshold); // find abnormal pose per frame

        void swapArrayData(Array<float>& newposekeypoints, const int formerPeople, const int newpeople);

        void addZeroToArray(Array<float>& poseKeypoints, const int numLine);

        bool checkZeroPeople(Array<float>& checkKeypoints, const int numberPeople);

        Array<float> matchRectangle(const Array<float>& formerKeypoints, const Array<float>& poseKeypoints);

        void findDifferentPose(const Array<float>& formerKeypoints, const Array<float>& posekeyPoints, const float threshold);
        //std::vector<int> findDifferentPose(const Array<float>& formerKeypoints, const Array<float>& posekeyPoints, const float threshold);

        void displayData(const Array<float>& formerKeypoints, const Array<float>& posekeyPoints);



    private:
        const int mNumberPeople;
        const int mNumberBodyParts; 
        //int mMaxPosePeople;
        //float mMaxPoseDistance;
        Array<float> mNormalizeKeypoints;
        Array<float> mMeanPose;     //2 = x , y;
        Array<float> mDistancePose;     //store everyone's distance with meanpose
        Array<float> mBiggestRectangle; //store the width and height of the biggest rectangle
        const float mThreShold;
    };
}





#endif // POSETRACK_H

