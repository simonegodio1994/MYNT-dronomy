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
#include "mynteye/api/processor/depth_processor_ocv.h"

#include <utility>

#include "mynteye/logger.h"

MYNTEYE_BEGIN_NAMESPACE

const char DepthProcessorOCV::NAME[] = "DepthProcessorOCV";

DepthProcessorOCV::DepthProcessorOCV(std::int32_t proc_period)
    : Processor(std::move(proc_period)) {
  VLOG(2) << __func__ << ": proc_period=" << proc_period;
}

DepthProcessorOCV::~DepthProcessorOCV() {
  VLOG(2) << __func__;
}

std::string DepthProcessorOCV::Name() {
  return NAME;
}

Object *DepthProcessorOCV::OnCreateOutput() {
  return new ObjMat();
}

bool DepthProcessorOCV::OnProcess(
    Object *const in, Object *const out,
    std::shared_ptr<Processor> const parent) {
  MYNTEYE_UNUSED(parent)
  const ObjMat *input = Object::Cast<ObjMat>(in);
  ObjMat *output = Object::Cast<ObjMat>(out);
  cv::Mat channels[3 /*input->value.channels()*/];
  cv::split(input->value, channels);
  channels[2].convertTo(output->value, CV_16UC1);
  output->id = input->id;
  output->data = input->data;
  return true;
}

MYNTEYE_END_NAMESPACE
