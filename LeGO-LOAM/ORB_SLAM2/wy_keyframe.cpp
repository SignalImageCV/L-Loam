#include <wy_keyframe.h>

namespace wy{
    
    KeyFrame::KeyFrame(const cv::Mat& img, ORB_SLAM2::ORBVocabulary* orb_vocabulary, ORB_SLAM2::ORBextractor* orb_extractor, int id)
    {
        img_ = img.clone();
        p_orb_vocabulary_ = orb_vocabulary;
        p_orb_extractor_ = orb_extractor;
        n_pts_ = -1;

        mnId = id;
        mnLoopQuery = -1;
        mnLoopWords = -1;
        
    };

    void KeyFrame::extracORBFeaturs()
    {
        (*p_orb_extractor_)(img_, cv::Mat(), v_kpts_, desp_);
        n_pts_ = v_kpts_.size();
    }

    void KeyFrame::computeBoW()
    {
        std::vector<cv::Mat> current_desp = ORB_SLAM2::Converter::toDescriptorVector(desp_);
        // p_orb_vocabulary_->transform(current_desp, bow_vec_, feature_vector_, 4);
        p_orb_vocabulary_->transform(current_desp, bow_vec_);
    }

    double KeyFrame::computeScoreByBow(const KeyFrame& kf)
    {
        return p_orb_vocabulary_->score(this->bow_vec_, kf.bow_vec_);
    }
}