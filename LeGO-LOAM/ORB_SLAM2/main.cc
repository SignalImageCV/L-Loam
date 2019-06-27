#include <opencv2/opencv.hpp>
#include <ORBVocabulary.h>
#include <ORBextractor.h>
#include <Converter.h>

#include <wy_keyframe.h>
#include <wy_keyframedb.h>
#include <bits/stdc++.h>

using std::cerr;
using std::endl;
using std::cout;
using namespace chrono;

bool has_suffix(const std::string &str, const std::string &suffix) {
  std::size_t index = str.find(suffix, str.size() - suffix.size());
  return (index != std::string::npos);
}

int main(int argc, char** argv)
{
    ORB_SLAM2::ORBVocabulary* porbvocabulary = new ORB_SLAM2::ORBVocabulary();
    
    std::string strVocFile(argv[1]);
    std::string strSettingFile(argv[2]);

    bool bvocload = false;
    if(has_suffix(strVocFile, ".txt"))
        bvocload = porbvocabulary->loadFromTextFile(strVocFile);
    else
        bvocload = false;

    if(!bvocload)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ);
    int nFeatures = fSettings["ORBextractor.nFeatures"];
    float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
    int nLevels = fSettings["ORBextractor.nLevels"];
    int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
    int fMinThFAST = fSettings["ORBextractor.minThFAST"];


    // cv::Mat img2 = cv::imread(argv[2], 0);
    // cv::imshow("0", img1);
    // cv::imshow("1", img2);

    ORB_SLAM2::ORBextractor* porbextractor = new ORB_SLAM2::ORBextractor(nFeatures, fScaleFactor, nLevels, fIniThFAST, fMinThFAST);
    wy::KeyFrameDB* pkeyframedb = new wy::KeyFrameDB(*porbvocabulary);
    // wy::KeyFrame f2(img2, porbvocabulary, porbextractor);

    std::string dir = "/media/yingwang/DATADisk/img1/";
    std::vector<wy::KeyFrame*> vkfs;
    for(int i = 10; i < 955; i++)
    {
        std::string file_name = std::to_string(i) + ".png";
        cv::Mat img = cv::imread(dir + file_name, 0);
        wy::KeyFrame* pkf = new wy::KeyFrame(img, porbvocabulary, porbextractor, i);
        pkf->extracORBFeaturs();
        pkf->computeBoW();
        pkeyframedb->add(pkf);
        vkfs.push_back(pkf);
    }

    cv::Mat img1 = cv::imread(dir + "666.png", 0);
    // cv::imshow("query", img1);
    wy::KeyFrame f1(img1, porbvocabulary, porbextractor, 666);
    int startidx = std::max(0, (int)(f1.mnId) - 10);
    int endidx = std::min((int)vkfs.size(), (int)(f1.mnId) + 10);

    f1.extracORBFeaturs();
    f1.computeBoW();

    float minscore = 100;
    for(int i = startidx; i < endidx; i++)
    {
        float score = porbvocabulary->score(f1.bow_vec_, vkfs[i]->bow_vec_);
        if(score < minscore) minscore = score;
    }
    std::cout << "minscore : " << minscore << std::endl;
    auto start = system_clock::now();
    std::vector<wy::KeyFrame*> loops = pkeyframedb->DetectLoopCandidates(&f1, minscore*0.8);
    auto end   = system_clock::now();
    auto duration = duration_cast<microseconds>(end - start);
    cout << "花费了" << double(duration.count()) * microseconds::period::num / microseconds::period::den 
    << "秒" << endl;

    // cout << (t2-t1) * 1000000 / CLOCKS_PER_SEC << std::endl;
    std::cout << loops.size() << std::endl;

    std::vector<cv::Mat> mats;
    std::stringstream ss;
    for(int i = 0; i < loops.size(); i++)
    {
        // cv::imshow(std::to_string(loops[i]->mnId), loops[i]->img_);
        cout << loops[i]->img_.size << endl;
        mats.push_back(loops[i]->img_.clone());
        ss << loops[i]->mnId << "_";
    }

    cv::Mat res;
    std::cout << mats.size() << std::endl;
    cv::hconcat(mats, res);
    cv::imshow(ss.str(), res);

    // f2.extracORBFeaturs();
    // f2.computeBoW();

    // double score1 = f1.computeScoreByBow(f2);
    // double score2 = f2.computeScoreByBow(f1);
    // cout << score1 << " " << score2 << std::endl;

    

    cv::waitKey(0);
    return 0;

}