#ifndef WY_KEYFRAMEDB_H
#define WY_KEYFRAMEDB_H

#include <set>
#include <vector>
#include <list>
#include <mutex>

#include <ORBVocabulary.h>
#include <wy_keyframe.h>

namespace wy
{

class KeyFrame;

class KeyFrameDB
{
public:
    KeyFrameDB(ORB_SLAM2::ORBVocabulary& voc);

    void add(KeyFrame* pKF);

    void erase(KeyFrame* pKF);

    void clear();

    // Loop Detection
    std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame* pKF, float minScore);
    int DetectLoopCandidates(KeyFrame* pKF, float minScore, bool t);

public:
    ORB_SLAM2::ORBVocabulary* mpVoc;
    std::vector<std::list<KeyFrame*>> mvInvertedFile;

    std::mutex mMutex;

}; // class KeyFrameDB


} // namespace wy


#endif