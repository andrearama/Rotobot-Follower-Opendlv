/*
 * Copyright (C) 2018 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <cstdint>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include<opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "tiny_dnn/tiny_dnn.h"

float  features(cv::Mat& frame, cv::Mat first_descriptors, double ratio) {
  cv::Mat second= frame;
  std::vector<cv::KeyPoint> second_keys;
  cv::Mat descriptors, second_descriptors;
  cv::Ptr<cv::ORB> orb = cv::ORB::create();
  orb->detectAndCompute(second, cv::noArray(), second_keys, second_descriptors);
  cv::BFMatcher matcher(cv::NORM_HAMMING);
  std::vector< std::vector<cv::DMatch> > nn_matches;
  if (second_descriptors.cols == first_descriptors.cols){
    matcher.knnMatch(second_descriptors, first_descriptors, nn_matches, 2); }
  else{
    matcher.knnMatch(second_descriptors, second_descriptors, nn_matches, 2); }
  float x_pos = 0;
  float counter = 0;
  for(int k = 0; (unsigned)k < nn_matches.size(); k++){
    if(nn_matches[k][0].distance / nn_matches[k][1].distance > ratio){
      x_pos = x_pos + second_keys[nn_matches[k][0].queryIdx].pt.x;
      counter ++;
    }
  }
  x_pos = x_pos/counter;
  return (x_pos-205.0f)/205.0f;}

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("name")) || (0 == commandlineArguments.count("cid")) || (0 == commandlineArguments.count("traincnn"))) {
    std::cerr << argv[0] << " accesses video data using shared memory provided using the command line parameter --name=." << std::endl;
    std::cerr << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session> --name=<name for the associated shared memory> [--id=<sender stamp>] [--verbose]" << std::endl;
    std::cerr << "         --name:    name of the shared memory to use" << std::endl;
    std::cerr << "         --traincnn: set 1 or 0 for training the tiny dnn example and saving a net binary" << std::endl;
    std::cerr << "         --verbose: when set, a thumbnail of the image contained in the shared memory is sent" << std::endl;
    std::cerr << "Example: " << argv[0] << " --cid=111 --name=cam0 --traincnn=1" << std::endl;
    retCode = 1;
  } else {

    uint32_t const WIDTH{1280};
    uint32_t const HEIGHT{960};
    uint32_t const BPP{24};
    uint32_t const ID{(commandlineArguments["id"].size() != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id"])) : 0};

    std::string const NAME{(commandlineArguments["name"].size() != 0) ? commandlineArguments["name"] : "/cam0"};
    cluon::OD4Session od4{static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::unique_ptr<cluon::SharedMemory> sharedMemory(new cluon::SharedMemory{NAME});
    if (sharedMemory && sharedMemory->valid()) {
      std::clog << argv[0] << ": Found shared memory '" << sharedMemory->name() << "' (" << sharedMemory->size() << " bytes)." << std::endl;

      CvSize size;
      size.width = WIDTH;
      size.height = HEIGHT;

      IplImage *image = cvCreateImageHeader(size, IPL_DEPTH_8U, BPP/8);
      sharedMemory->lock();
      image->imageData = sharedMemory->data();
      image->imageDataOrigin = image->imageData;
      sharedMemory->unlock();

      float p = 1.0f;
      float a = -3.04f;
      float b = 324.63f;
      float ratio = 0.999f;

      //Extract descriptors:
      cv::Mat first = cv::imread("/usr/bin/carcar.png", CV_LOAD_IMAGE_COLOR );
      std::clog <<first.size()<<std::endl;
      std::vector<cv::KeyPoint> first_keys;
      cv::Mat first_descriptors;
      cv::Ptr<cv::ORB> orb = cv::ORB::create();
      orb->detectAndCompute(first, cv::noArray(), first_keys, first_descriptors);

      while (od4.isRunning()) {
        sharedMemory->wait();

      // Make a scaled copy of the original image.
      int32_t const width = 410;
      int32_t const height = 308;
      cv::Mat src1;
      {
        sharedMemory->lock();
        cv::Mat sourceImage = cv::cvarrToMat(image, false);
        cv::resize(sourceImage, src1, cv::Size(width, height), 0, 0, cv::INTER_NEAREST);
        sharedMemory->unlock();
      }


       //Crop region of interest:
       cv::Mat dst = src1.rowRange(56, 196);
       cv::resize(dst, dst, dst.size() );
       //Filter red color:
       cv::Mat redOnly;
       cv::inRange(dst, cv::Scalar(10,10,80), cv::Scalar(60,60, 220), redOnly);
       //Compute baricenter:
       cv::Moments m = cv::moments(redOnly, false);
       float p1 = float(m.m10/m.m00);
       float p1_y = float(m.m01/m.m00);

       //Get angle and distance :
       float angle_car = p*(p1-205.0f)/205.0f; // 0= center, +-1 = max stir
       float bar = features(dst, first_descriptors, ratio);
       angle_car = (angle_car+0.31f*bar)/(1.31f);

       float distance_car = a*p1_y + b;
       if (m.m00 <0.001){angle_car = 0.000f;distance_car = 0.000f;}
       //Send:
       opendlv::logic::sensation::Point detection;
       detection.azimuthAngle(angle_car);
       detection.distance(distance_car);
       od4.send(detection, cluon::time::now(), ID);

      }

      cvReleaseImageHeader(&image);
    } else {
      std::cerr << argv[0] << ": Failed to access shared memory '" << NAME << "'." << std::endl;
    }
  }
  return retCode;
}
