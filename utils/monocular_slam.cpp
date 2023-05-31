/**
* This file is part of  UCOSLAM
*
* Copyright (C) 2018 Rafael Munoz Salinas <rmsalinas at uco dot es> (University of Cordoba)
*
* UCOSLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* UCOSLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with UCOSLAM. If not, see <http://wwmap->gnu.org/licenses/>.
*/
#include "ucoslam.h"
#include "basictypes/debug.h"
#include "mapviewer.h"
#include "basictypes/timers.h"
#include "map.h"
#include "inputreader.h"
#include "basictypes/cvversioning.h"
#include <iostream>
#include <fstream>

inline float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

inline float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}

// quaternion = [x, y, z, w]'
cv::Mat mRot2Quat(const cv::Mat& m) {
    float r11 = m.at<float>(0, 0);
    float r12 = m.at<float>(0, 1);
    float r13 = m.at<float>(0, 2);
    float r21 = m.at<float>(1, 0);
    float r22 = m.at<float>(1, 1);
    float r23 = m.at<float>(1, 2);
    float r31 = m.at<float>(2, 0);
    float r32 = m.at<float>(2, 1);
    float r33 = m.at<float>(2, 2);
    float q0 = (r11 + r22 + r33 + 1.0f) / 4.0f;
    float q1 = (r11 - r22 - r33 + 1.0f) / 4.0f;
    float q2 = (-r11 + r22 - r33 + 1.0f) / 4.0f;
    float q3 = (-r11 - r22 + r33 + 1.0f) / 4.0f;
    if (q0 < 0.0f) {
        q0 = 0.0f;
    }
    if (q1 < 0.0f) {
        q1 = 0.0f;
    }
    if (q2 < 0.0f) {
        q2 = 0.0f;
    }
    if (q3 < 0.0f) {
        q3 = 0.0f;
    }
    q0 = sqrt(q0);
    q1 = sqrt(q1);
    q2 = sqrt(q2);
    q3 = sqrt(q3);
    if (q0 >= q1 && q0 >= q2 && q0 >= q3) {
        q0 *= +1.0f;
        q1 *= SIGN(r32 - r23);
        q2 *= SIGN(r13 - r31);
        q3 *= SIGN(r21 - r12);
    }
    else if (q1 >= q0 && q1 >= q2 && q1 >= q3) {
        q0 *= SIGN(r32 - r23);
        q1 *= +1.0f;
        q2 *= SIGN(r21 + r12);
        q3 *= SIGN(r13 + r31);
    }
    else if (q2 >= q0 && q2 >= q1 && q2 >= q3) {
        q0 *= SIGN(r13 - r31);
        q1 *= SIGN(r21 + r12);
        q2 *= +1.0f;
        q3 *= SIGN(r32 + r23);
    }
    else if (q3 >= q0 && q3 >= q1 && q3 >= q2) {
        q0 *= SIGN(r21 - r12);
        q1 *= SIGN(r31 + r13);
        q2 *= SIGN(r32 + r23);
        q3 *= +1.0f;
    }
    else {
        printf("coding error\n");
    }
    float r = NORM(q0, q1, q2, q3);
    q0 /= r;
    q1 /= r;
    q2 /= r;
    q3 /= r;

    cv::Mat res = (cv::Mat_<float>(4, 1) << q1, q2, q3, q0);
    return res;
}


cv::Mat getImage(cv::VideoCapture &vcap,int frameIdx){
    cv::Mat im;
    ucoslam::Frame frame;
    vcap.set(CV_CAP_PROP_POS_FRAMES,frameIdx);
    vcap.grab();
    vcap.set(CV_CAP_PROP_POS_FRAMES,frameIdx);
    vcap.retrieve(im);
    return im;
}


class CmdLineParser{int argc; char **argv;
                public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue=""){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }
                    std::vector<std::string> getAllInstances(string str){
                        std::vector<std::string> ret;
                        for(int i=0;i<argc-1;i++){
                            if (string(argv[i])==str)
                                ret.push_back(argv[i+1]);
                        }
                        return ret;
                    }
                   };

cv::Size readInpuSize(string s){
    for(auto &c:s)if(c==':')c =' ';
    stringstream sstr(s.c_str());
    cv::Size size;
    if ( sstr>>size.width>>size.height) return size;
    else return cv::Size(0,0);
}

cv::Mat resize(cv::Mat &in,cv::Size size){
    if (size.area()<=0)return in;
    cv::Mat ret,ret2;
    cv::resize(in,ret,size);  return ret;
}


int cIndexLive=0;
int getCurrentFrameIndex(cv::VideoCapture &vcap,bool isLive){

    if (isLive)return cIndexLive++;
    else return  int(vcap.get(CV_CAP_PROP_POS_FRAMES));
}


void overwriteParamsByCommandLine(CmdLineParser &cml,ucoslam::Params &params){
    if ( cml["-aruco-markerSize"])      params.aruco_markerSize = stof(cml("-aruco-markerSize", "1"));
    if ( cml["-marker_minsize"])    params.aruco_minMarkerSize= stod(cml("-marker_minsize", "0.025"));
    if (cml["-nokeypoints"])params.detectKeyPoints=false;
    if (cml["-nomarkers"])  params.detectMarkers =false;
    if (cml["-sequential"]) params.runSequential=true;
    if (cml["-maxFeatures"])    params.maxFeatures = stoi(cml("-maxFeatures","4000"));
    if (cml["-nOct"])       params.nOctaveLevels = stoi(cml("-nOct","8"));
    if (cml["-fdt"])        params.nthreads_feature_detector = stoi(cml("-fdt", "2"));
     if (cml["-desc"])       params.kpDescriptorType = ucoslam::DescriptorTypes::fromString(cml("-desc", "orb"));
    if (cml["-dict"])       params.aruco_Dictionary = cml("-dict");
    if (cml["-tfocus"])  params.targetFocus =stof(cml("-tfocus","-1"));
    if (cml["-KFMinConfidence"])  params.KFMinConfidence =stof(cml("-KFMinConfidence"));
    if(cml["-nonmax"])    params.KPNonMaximaSuppresion=true;

    if(cml["-autoAdjustKpSensitivity"])    params.autoAdjustKpSensitivity=true;
    if(cml["-extra_params"])    params.extraParams=cml("-extra_params");

    if(cml["-scale"]) params.kptImageScaleFactor=stof(cml("-scale"));

    if(cml["-nokploopclosure"]) params.reLocalizationWithKeyPoints=false;
    if(cml["-inplanemarkers"]) params.inPlaneMarkers=true;
    params.aruco_CornerRefimentMethod=cml("-aruco-cornerRefinementM","CORNER_SUBPIX");

    if (cml["-dbg_str"])
        ucoslam::debug::Debug::addString(cml("-dbg_str"),"");
}

int main(int argc,char **argv){
	try {
		CmdLineParser cml(argc, argv);
        if (argc < 3 || cml["-h"]) {
            cerr << "Usage: (video|live[:cameraIndex(0,1...)])  camera_params.yml [-params ucoslam_params.yml] [-map world]  [-out name] [-scale <float>:video resize factor]"
                    "[-loc_only do not update map, only do localization. Requires -in]"
                    "\n"
                    "[-desc descriptor orb,akaze,brisk,freak] "
                    "[-aruco-markerSize markers_size] [-dict <dictionary>:. By default ARUCO_MIP_36h12]  "
                    "[-nomarkers]  [-debug level] [-voc bow_volcabulary_file] "
                    "[-t_fe n:number of threads of the feature detector] [-st starts the processing stopped ] "
                    "[-nokeypoints] [-marker_minsize <val_[0,1]>] [-em . Uses enclosed markers] [-noX disabled windows] "
                    "[-fps X: set video sequence frames per second] [-outvideo filename]"
                    "[-featDensity <float>:features density]"
                    "[-nOct <int>:number of octave layers]"
                    "[-noMapUpdate]"
                    "[-tfocus <float>: target focus employed to create the map. Replaces the one of the camera] "
                    "[-undistort] will undistort image before processing it"
                    "[-extra_params \"param1=value1 param2=value2...\"]"
                    "[-vspeed <int:def 1> video analysis speed ]"
                 << endl; return -1;
		}

		bool liveVideo = false;
        InputReader vcap;
        cv::VideoWriter videoout;
		string TheInputVideo = string(argv[1]);
        string TheOutputVideo=cml("-outvideo");
		if (TheInputVideo.find("live") != std::string::npos)
		{
			int vIdx = 0;
			// check if the :idx is here
            char cad[100];
			if (TheInputVideo.find(":") != string::npos)
			{
				std::replace(TheInputVideo.begin(), TheInputVideo.end(), ':', ' ');
				sscanf(TheInputVideo.c_str(), "%s %d", cad, &vIdx);
			}
			cout << "Opening camera index " << vIdx << endl;
			vcap.open(vIdx);
            //vcap.set(CV_CAP_PROP_AUTOFOCUS, 0);
            liveVideo = true;

		}
        else vcap.open(argv[1],!cml["-sequential"]);

		if (!vcap.isOpened())
			throw std::runtime_error("Video not opened");

        ucoslam::UcoSlam Slam;
		int debugLevel = stoi(cml("-debug", "0"));
        Slam.setDebugLevel(debugLevel);
        Slam.showTimers(true);
        ucoslam::ImageParams image_params;
        ucoslam::Params params;
        cv::Mat in_image;

        image_params.readFromXMLFile(argv[2]);

        if( cml["-params"])        params.readFromYMLFile(cml("-params"));
        overwriteParamsByCommandLine(cml,params);

        auto TheMap=std::make_shared<ucoslam::Map>();
        //read the map from file?
        if ( cml["-map"]) TheMap->readFromFile(cml("-map"));

        Slam.setParams(TheMap, params,cml("-voc"));

        if(!cml["-voc"]  && !cml["-map"])
        {
            cerr<<"Warning!! No VOCABULARY INDICATED. RELOCALIZATION IMPOSSIBLE WITHOUT VOCABULARY FILE!!!!!"<<endl;
        }


        if (cml["-loc_only"]) Slam.setMode(ucoslam::MODE_LOCALIZATION);

        //need to skip frames?
        if (cml["-skip"]) {
            int n=stoi(cml("-skip","0"));
            vcap.set(CV_CAP_PROP_POS_FRAMES,n);
            cerr<<endl;
        }

        //read the first frame if not yet
        while (in_image.empty())
            vcap >> in_image;
        //need to resize input image?
        cv::Size vsize(0,0);
        //need undistortion

        bool undistort=cml["-undistort"];
        vector<cv::Mat > undistMap;
        if(undistort ){
            if( undistMap.size()==0){
                undistMap.resize(2);
                cv::initUndistortRectifyMap(image_params.CameraMatrix,image_params.Distorsion,cv::Mat(),cv::Mat(),image_params.CamSize,CV_32FC1,undistMap[0],undistMap[1]);
            }
            image_params.Distorsion.setTo(cv::Scalar::all(0));
        }
        //Create the viewer to see the images and the 3D
        ucoslam::MapViewer TheViewer;

        if (cml["-slam"]){
            Slam.readFromFile(cml("-slam"));
             vcap.set(CV_CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
            vcap.retrieve(in_image);
            vcap.set(CV_CAP_PROP_POS_FRAMES,Slam.getLastProcessedFrame());
            vcap.retrieve(in_image);
            TheMap=Slam.getMap();
            overwriteParamsByCommandLine(cml,params);
            Slam.updateParams(params);

        }

        if (cml["-noMapUpdate"])
            Slam.setMode(ucoslam::MODE_LOCALIZATION);

        cv::Mat auxImage;
        //Ok, lets start
        ucoslam::TimerAvrg Fps;
        bool finish = false;
        cv::Mat camPose_c2g;
        int vspeed=stoi(cml("-vspeed","1"));
        ofstream TrajectoryFile("trajectory.txt");
        while (!finish && !in_image.empty()) {
            //image resize (if required)
            in_image = resize(in_image, vsize);


            //image undistortion (if required)
            if(undistort ){               
                cv::remap(in_image,auxImage,undistMap[0],undistMap[1],cv::INTER_CUBIC);
                in_image=auxImage;
                image_params.Distorsion.setTo(cv::Scalar::all(0));
            }


            int currentFrameIndex = vcap.getCurrentFrameIndex();
            Fps.start();
            camPose_c2g=Slam.process(in_image, image_params,currentFrameIndex);
            Fps.stop();

            if(camPose_c2g.empty()){
                TrajectoryFile<<"0 0 0 0 0 0 1"<<endl;
            }
            else{
                auto quat = mRot2Quat(camPose_c2g);
                TrajectoryFile<<camPose_c2g.at<float>(0, 3)<<" "
                <<camPose_c2g.at<float>(1, 3)<<" "
                <<camPose_c2g.at<float>(2, 3)<<" "
                <<quat.at<float>(0, 0)<<" "
                <<quat.at<float>(1, 0)<<" "
                <<quat.at<float>(2, 0)<<" "
                <<quat.at<float>(3, 0)<<endl;
            }

            cout << "Image " << currentFrameIndex << " fps=" << 1./Fps.getAvrg()<< endl;
            //            Slam.drawMatches(in_image);
            //    char k = TheViewer.show(&Slam, in_image,"#" + std::to_string(currentFrameIndex) + " fps=" + to_string(1./Fps.getAvrg()) );
            char k = TheViewer.show(TheMap,   in_image, camPose_c2g,"#" + std::to_string(currentFrameIndex)/* + " fps=" + to_string(1./Fps.getAvrg())*/ ,Slam.getCurrentKeyFrameIndex());
            if (int(k) == 27)finish = true;//pressed ESC

            //save to output video?
            if (!TheOutputVideo.empty()){
                auto image=TheViewer.getImage();
                if(!videoout.isOpened())
                    videoout.open(TheOutputVideo, CV_FOURCC('X', '2', '6', '4'), stof(cml("-fps","30")),image.size()  , image.channels()!=1);
                if(videoout.isOpened())  videoout.write(image);
            }

            //draw cube



            //reset?
            if (k=='r') Slam.clear();
            //write the current map
            if (k=='e'){
                string number = std::to_string(currentFrameIndex);
                while (number.size() < 5) number = "0" + number;
                TheMap->saveToFile("world-"+number+".map");
            }
            if (k=='v'){
                Slam.saveToFile("slam.slm");
            }
            //read next
            vcap >> in_image;
            if(!camPose_c2g.empty()){
                for(int s=0;s<vspeed-1;s++)
                    vcap >> in_image;
            }
        }

        //release the video output if required
        if(videoout.isOpened()) videoout.release();

        //save the output

        TheMap->saveToFile(cml("-out","world") +".map");
        //save also the parameters finally employed
        params.saveToYMLFile("ucoslam_params_"+cml("-out","world") +".yml");
        if (debugLevel >=10){
            Slam.saveToFile("slam.slm");
        }
        TheMap->saveToMarkerMap("markermap.yml");

        ofstream MarkersFile("markers.txt");
        for(const auto &marker:TheMap->map_markers){
            auto pose = marker.second.pose_g2m;
            if(pose.isValid()){
                auto quat = mRot2Quat(pose);
                MarkersFile<<marker.second.id<<" "
                <<pose.at<float>(0, 3)<<" "
                <<pose.at<float>(1, 3)<<" "
                <<pose.at<float>(2, 3)<<" "
                <<quat.at<float>(0, 0)<<" "
                <<quat.at<float>(1, 0)<<" "
                <<quat.at<float>(2, 0)<<" "
                <<quat.at<float>(3, 0)<<endl;
            }
        }


    }
    catch (const std::exception &ex) {
        cerr << ex.what() << endl;
    }
}
