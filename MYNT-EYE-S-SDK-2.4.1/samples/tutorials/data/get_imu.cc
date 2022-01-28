// Copyright 2018 Slightech Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <opencv2/highgui/highgui.hpp>

#include "mynteye/logger.h"
#include "mynteye/api/api.h"
#include <mutex>
#include <atomic>


#include "util/cv_painter.h"

MYNTEYE_USE_NAMESPACE

int main(int argc, char *argv[]) {

  auto &&api = API::Create(argc, argv);
  //api->EnablePlugin("/home/simonegodio/Downloads/libplugin_g_cuda9.2_opencv3.4.0.so");

// ACCELEROMETER_RANGE values: 4, 8, 16, 32
api->SetOptionValue(Option::ACCELEROMETER_RANGE, 32);
// GYROSCOPE_RANGE values: 500, 1000, 2000, 4000
api->SetOptionValue(Option::GYROSCOPE_RANGE, 4000);

LOG(INFO) << "Set FRAME_RATE to " << api->GetOptionValue(Option::FRAME_RATE);
LOG(INFO) << "Set IMU_FREQUENCY to "
          << api->GetOptionValue(Option::IMU_FREQUENCY);

LOG(INFO) << "Set ACCELEROMETER_RANGE to "
          << api->GetOptionValue(Option::ACCELEROMETER_RANGE);
LOG(INFO) << "Set GYROSCOPE_RANGE to "
          << api->GetOptionValue(Option::GYROSCOPE_RANGE);

LOG(INFO) << "ok2";
  if (!api) return 1;

  bool ok;

  auto &&request = api->SelectStreamRequest(&ok);
  if (!ok) return 1;

  api->ConfigStreamRequest(request);


  // Enable this will cache the motion datas until you get them.
  api->EnableMotionDatas();
LOG(INFO) << "ok1";

// Get motion data from callback
std::atomic_uint imu_count(0);
std::shared_ptr<mynteye::ImuData> imu;
std::mutex imu_mtx;
api->SetMotionCallback(
    [&imu_count, &imu, &imu_mtx](const api::MotionData &data) {
      CHECK_NOTNULL(data.imu);
      ++imu_count;
      {
        std::lock_guard<std::mutex> _(imu_mtx);
        imu = data.imu;
      }
    });


  api->Start(Source::ALL);

  CVPainter painter;

  cv::namedWindow("frame");
  LOG(INFO) << "ok0";

  while (true) {
    api->WaitForStreams();
    api->EnableMotionDatas();

    auto &&left_data = api->GetStreamData(Stream::LEFT);
    auto &&right_data = api->GetStreamData(Stream::RIGHT);

    if (!left_data.frame.empty() && !right_data.frame.empty()) {
      cv::Mat img;
      cv::hconcat(left_data.frame, right_data.frame, img);
      

      auto &&motion_datas = api->GetMotionDatas();

      //printf("I'm in..");
      
      for (auto &&data : motion_datas) {
        LOG(INFO) << "Imu frame_id: " << data.imu->frame_id
                  << ", timestamp: " << data.imu->timestamp
                  << ", accel_x: " << data.imu->accel[0]
                  << ", accel_y: " << data.imu->accel[1]
                  << ", accel_z: " << data.imu->accel[2]
                  << ", gyro_x: " << data.imu->gyro[0]
                  << ", gyro_y: " << data.imu->gyro[1]
                  << ", gyro_z: " << data.imu->gyro[2]
                  << ", temperature: " << data.imu->temperature;
      }
      

      painter.DrawImgData(img, *left_data.img);
      if (!motion_datas.empty() && motion_datas.size() > 0) {
        painter.DrawImuData(img, *motion_datas[0].imu);
      }
  // Draw imu data
  if (imu) {
    std::lock_guard<std::mutex> _(imu_mtx);
    painter.DrawImuData(img, *imu);
    LOG(INFO) << "Imu bool: " << imu;
  }

  // Draw counts
  //std::ostringstream ss;
  //ss <<"imu: " << imu_count;
  //LOG(INFO) << "ok";
  //painter.DrawText(img, ss.str(), CVPainter::BOTTOM_RIGHT);

      cv::imshow("frame", img);
    }

    char key = static_cast<char>(cv::waitKey(1));
    if (key == 27 || key == 'q' || key == 'Q') {  // ESC/Q
      break;
    }
  }

  api->Stop(Source::ALL);
  return 0;
}
