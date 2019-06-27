#ifndef WY_KEYFRAME_H
#define WY_KEYFRAME_H

#include <vector>
#include <opencv2/opencv.hpp>
#include <ORBextractor.h>
#include <ORBVocabulary.h>
#include <Converter.h>
#include <Thirdparty/DBoW2/DBoW2/BowVector.h>
#include <Thirdparty/DBoW2/DBoW2/FeatureVector.h>

namespace wy
{

class KeyFrame
{
public:
    KeyFrame(const cv::Mat& img, ORB_SLAM2::ORBVocabulary* orb_vocabulary, ORB_SLAM2::ORBextractor* orb_extractor, int id);

    void extracORBFeaturs();

    void computeBoW();
    
    double computeScoreByBow(const KeyFrame& kf);

public:
    cv::Mat img_;
    cv::Mat desp_;
    std::vector<cv::KeyPoint> v_kpts_;

    ORB_SLAM2::ORBextractor* p_orb_extractor_;
    ORB_SLAM2::ORBVocabulary* p_orb_vocabulary_;

    //Bow
    DBoW2::FeatureVector feature_vector_;
    DBoW2::BowVector bow_vec_;

    int n_pts_;

    int mnId;
    double mTimeStamp;

    // Variables used by the keyframe database
    int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    
    int mnRelocQuery;
    int mnRelocWords;
    float mRelocScore;
    
};


} // namespace wy

#endif