#include <wy_keyframedb.h>

using namespace std;
namespace wy
{

    KeyFrameDB::KeyFrameDB(ORB_SLAM2::ORBVocabulary& voc)
    {
        mpVoc = &voc;
        mvInvertedFile.resize(voc.size());
    }

    void KeyFrameDB::add(KeyFrame* pKF)
    {
        // std::unique_lock<mutex> lock(mMutex);

        for(DBoW2::BowVector::const_iterator vit= pKF->bow_vec_.begin(), vend=pKF->bow_vec_.end(); vit!=vend; vit++)
            mvInvertedFile[vit->first].push_back(pKF); //每个word下有的kF
    }

    // void erase(KeyFrame* pKF);

    void KeyFrameDB::clear()
    {
        mvInvertedFile.clear();
        mvInvertedFile.resize(mpVoc->size());
    };

    // Loop Detection
    std::vector<KeyFrame *> KeyFrameDB::DetectLoopCandidates(KeyFrame* pKF, float minScore)
    {
        std::list<KeyFrame*> lKFsSharingWords;

        // unique_lock<mutex> lock(mMutex);
        for(DBoW2::BowVector::const_iterator vit = pKF->bow_vec_.begin(), vend=pKF->bow_vec_.end(); vit!=vend;vit++)
        {
            std::list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

            for(std::list<KeyFrame*>::iterator lit = lKFs.begin(); lit!=lKFs.end(); lit++)
            {
                KeyFrame *pKFi = *lit;
                if(pKFi->mnLoopQuery != pKF->mnId)
                {
                    pKFi->mnLoopWords = -1;// 共同word的数量
                    // 不需要相邻帧
                    if(abs((int)pKFi->mnId - (int)pKF->mnId) > 10)
                    {
                        pKFi->mnLoopQuery = pKF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                }
                pKFi->mnLoopWords++;
            }
        }

        if(lKFsSharingWords.empty())
            return vector<KeyFrame*>();
        
        std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

        int maxCommonWords = 0;
        for(std::list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(); lit != lKFsSharingWords.end(); lit++)
        {
            if((*lit)->mnLoopWords > maxCommonWords)
            {
                maxCommonWords = (*lit)->mnLoopWords;
            }
        }

        int minCommonWords = maxCommonWords * 0.8f;

        int nscores = 0;
        float bestscore = 0;
        // Compute similarity score. Retain the matches whose score is higher than minScore
        for(list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(); lit!=lKFsSharingWords.end(); lit++)
        {
            KeyFrame* pKFi = *lit;

            if(pKFi->mnLoopWords > minCommonWords)
            {
                nscores++;

                float si = mpVoc->score(pKF->bow_vec_, pKFi->bow_vec_);
                if(si >= minScore)
                {
                    lScoreAndMatch.push_back(std::make_pair(si, pKFi));
                    if(si > bestscore)
                    {
                        bestscore = si;
                    }
                }
            }
        }

        if(lScoreAndMatch.empty())
            return vector<KeyFrame*>();

        std::vector<KeyFrame*> vpLoopCandidates;
        float minScoreToRetain = 0.75f * bestscore;

        for(auto it = lScoreAndMatch.begin(); it != lScoreAndMatch.end(); it++)
        {
            if(it->first > minScoreToRetain)
            {
                KeyFrame* pKFi = it->second;
                vpLoopCandidates.push_back(pKFi);
            }

        }
        return vpLoopCandidates;

    };

    int KeyFrameDB::DetectLoopCandidates(KeyFrame* pKF, float minScore, bool t)
    {
        // std::unique_lock<mutex> lock(mMutex);

        std::list<KeyFrame*> lKFsSharingWords;

        for(DBoW2::BowVector::const_iterator vit = pKF->bow_vec_.begin(), vend=pKF->bow_vec_.end(); vit!=vend;vit++)
        {
            std::list<KeyFrame*> &lKFs = mvInvertedFile[vit->first];

            for(std::list<KeyFrame*>::iterator lit = lKFs.begin(); lit!=lKFs.end(); lit++)
            {
                KeyFrame *pKFi = *lit;
                if(pKFi->mnLoopQuery != pKF->mnId)
                {
                    pKFi->mnLoopWords = -1;// 共同word的数量
                    // 不需要相邻帧
                    if(abs((int)pKFi->mnId - (int)pKF->mnId) > 10)
                    {
                        pKFi->mnLoopQuery = pKF->mnId;
                        // std::cout << "pKFi->mnLoopQuery = pKF->mnId;" <<std::endl;
                        lKFsSharingWords.push_back(pKFi);
                        // std::cout << "lKFsSharingWords.push_back(pKFi);" <<std::endl;

                    }
                }
                pKFi->mnLoopWords++;
            }
        }
        // std::cout << "lKFsSharingWords.empty()" << lKFsSharingWords.size() <<std::endl;
        
        if(lKFsSharingWords.empty())
            return -1;
        
        std::list<std::pair<float, KeyFrame*>> lScoreAndMatch;

        int maxCommonWords = 0;
        for(std::list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(); lit != lKFsSharingWords.end(); lit++)
        {
            if((*lit)->mnLoopWords > maxCommonWords)
            {
                maxCommonWords = (*lit)->mnLoopWords;
            }
        }

        // cout << "maxCommonWords" << maxCommonWords << std::endl;

        int minCommonWords = maxCommonWords * 0.8f;
        // cout << "minCommonWords" << minCommonWords << std::endl;

        int nscores = 0;
        float bestscore = 0;
        float second_bestscore = 0;
        int bestinx = -1;
        // Compute similarity score. Retain the matches whose score is higher than minScore
        for(list<KeyFrame*>::iterator lit = lKFsSharingWords.begin(); lit!=lKFsSharingWords.end(); lit++)
        {
            KeyFrame* pKFi = *lit;

            if(pKFi->mnLoopWords > minCommonWords)
            {
                nscores++;

                float si = mpVoc->score(pKF->bow_vec_, pKFi->bow_vec_);
                // float si = pKF->computeScoreByBow(*pKFi);
                if(si >= minScore)
                {
                    // lScoreAndMatch.push_back(std::make_pair(si, pKFi));
                    if(si > bestscore)
                    {
                        bestscore = si;
                        bestinx = pKFi->mnId;
                    }
                    else if(si > second_bestscore)
                    {
                        second_bestscore = si;
                    }
                }
            }
        }

        if(lKFsSharingWords.size() > 1 && bestscore < 1.1 * second_bestscore) return -1;

        // cout << "lScoreAndMatch" << lScoreAndMatch.size() << std::endl;

        // if(lScoreAndMatch.empty())
        //     return -1;

        // std::cout << "best inx : " << bestinx <<std::endl;
        return bestinx;

    };

} // namespace wy