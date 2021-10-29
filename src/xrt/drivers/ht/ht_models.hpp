// Copyright 2021, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Code to run machine learning models for camera-based hand tracker.
 * @author Moses Turner <moses@collabora.com>
 * @author Marcus Edel <marcus.edel@collabora.com>
 * @ingroup drv_ht
 */

// Many C api things were stolen from here (MIT license):
// https://github.com/microsoft/onnxruntime-inference-examples/blob/main/c_cxx/fns_candy_style_transfer/fns_candy_style_transfer.c

#pragma once

#include "os/os_time.h"
#include "os/os_threading.h"

#include "math/m_api.h"
#include "math/m_vec2.h"
#include "math/m_vec3.h"

#include "util/u_json.h"
#include "util/u_time.h"
#include "util/u_logging.h"
#include "util/u_trace_marker.h"

#include "ht_nms.hpp"
#include "ht_driver.hpp"
#include "ht_image_math.hpp"

#include <core/session/onnxruntime_c_api.h>

#include <opencv2/core/types.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <math.h>
#include <stdio.h>
#include <string.h>

#include <cmath>
#include <cstdlib>

#include <vector>
#include <limits>
#include <numeric>
#include <sstream>
#include <fstream>
#include <iostream>
#include <exception>

#define ORT_CHECK(g_ort, expr)                                                                                         \
	do {                                                                                                           \
		OrtStatus *onnx_status = (expr);                                                                       \
		if (onnx_status != nullptr) {                                                                          \
			const char *msg = g_ort->GetErrorMessage(onnx_status);                                         \
			U_LOG_E("at %s:%d: %s\n", __FILE__, __LINE__, msg);                                            \
			g_ort->ReleaseStatus(onnx_status);                                                             \
			assert(false);                                                                                 \
		}                                                                                                      \
	} while (0);

static Hand2D
runKeypointEstimator(struct ht_view *htv, cv::Mat img)
{
	constexpr size_t lix = 224;
	constexpr size_t liy = 224;
	constexpr size_t nb_planes = 3;
	cv::Mat planes[nb_planes];

	constexpr size_t size = lix * liy * nb_planes;

	std::vector<uint8_t> combined_planes(size);
	planarize(img, combined_planes.data());

	// Normalize - supposedly, the keypoint estimator wants keypoints in [0,1]
	std::vector<float> real_thing(size);
	for (size_t i = 0; i < size; i++) {
		real_thing[i] = (float)combined_planes[i] / 255.0;
	}

	const OrtApi *g_ort = htv->htd->ort_api;
	struct ModelInfo *model = &htv->keypoint_model;

	OrtValue *input_tensor = nullptr;

	ORT_CHECK(g_ort, g_ort->CreateTensorWithDataAsOrtValue(
	                     model->memoryInfo, real_thing.data(), model->input_size_bytes, model->input_shape.data(),
	                     model->input_shape.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &input_tensor));

	// Cargo-culted
	assert(input_tensor != nullptr);
	int is_tensor;
	ORT_CHECK(g_ort, g_ort->IsTensor(input_tensor, &is_tensor));
	assert(is_tensor);

	const char *output_names[] = {"Identity", "Identity_2", "Identity_2"};

	// If any of these are non-NULL, ONNX will explode and won't tell you why! Be extremely paranoid!
	OrtValue *output_tensor[3] = {nullptr, nullptr, nullptr};
	ORT_CHECK(g_ort, g_ort->Run(model->session, nullptr, model->input_names.data(), &input_tensor, 1, output_names,
	                            3, output_tensor));

	ORT_CHECK(g_ort, g_ort->IsTensor(output_tensor[0], &is_tensor));
	assert(is_tensor);

	float *landmarks = nullptr;

	// Should give a pointer to data that is freed on g_ort->ReleaseValue(output_tensor[0]);.
	ORT_CHECK(g_ort, g_ort->GetTensorMutableData(output_tensor[0], (void **)&landmarks));

	int stride = 3;
	Hand2D dumb;
	for (size_t i = 0; i < 21; i++) {
		int rt = i * stride;
		float x = landmarks[rt];
		float y = landmarks[rt + 1];
		float z = landmarks[rt + 2];
		dumb.kps[i].x = x;
		dumb.kps[i].y = y;
		dumb.kps[i].z = z;
	}

	// We have to get to here, or else we leak a whole lot! If you need to return early, make this into a goto!
	g_ort->ReleaseValue(output_tensor[0]);
	g_ort->ReleaseValue(output_tensor[1]);
	g_ort->ReleaseValue(output_tensor[2]);
	g_ort->ReleaseValue(input_tensor);
	return dumb;
}

#undef HEAVY_SCRIBBLE

/*
 * Anchors data taken from mediapipe's hands detection, used for single-shot
 * detector model.
 *
 * See:
 * https://google.github.io/mediapipe/solutions/hands.html#palm-detection-model
 * https://github.com/google/mediapipe/blob/v0.8.8/mediapipe/calculators/tflite/ssd_anchors_calculator.cc#L101
 * https://github.com/google/mediapipe/blob/v0.8.8/mediapipe/modules/palm_detection/palm_detection_cpu.pbtxt#L60
 */
struct anchor
{
	float x, y;
};

static const struct anchor anchors[896]{
    {0.031250, 0.031250}, {0.031250, 0.031250}, {0.093750, 0.031250}, {0.093750, 0.031250}, //
    {0.156250, 0.031250}, {0.156250, 0.031250}, {0.218750, 0.031250}, {0.218750, 0.031250}, //
    {0.281250, 0.031250}, {0.281250, 0.031250}, {0.343750, 0.031250}, {0.343750, 0.031250}, //
    {0.406250, 0.031250}, {0.406250, 0.031250}, {0.468750, 0.031250}, {0.468750, 0.031250}, //
    {0.531250, 0.031250}, {0.531250, 0.031250}, {0.593750, 0.031250}, {0.593750, 0.031250}, //
    {0.656250, 0.031250}, {0.656250, 0.031250}, {0.718750, 0.031250}, {0.718750, 0.031250}, //
    {0.781250, 0.031250}, {0.781250, 0.031250}, {0.843750, 0.031250}, {0.843750, 0.031250}, //
    {0.906250, 0.031250}, {0.906250, 0.031250}, {0.968750, 0.031250}, {0.968750, 0.031250}, //
    {0.031250, 0.093750}, {0.031250, 0.093750}, {0.093750, 0.093750}, {0.093750, 0.093750}, //
    {0.156250, 0.093750}, {0.156250, 0.093750}, {0.218750, 0.093750}, {0.218750, 0.093750}, //
    {0.281250, 0.093750}, {0.281250, 0.093750}, {0.343750, 0.093750}, {0.343750, 0.093750}, //
    {0.406250, 0.093750}, {0.406250, 0.093750}, {0.468750, 0.093750}, {0.468750, 0.093750}, //
    {0.531250, 0.093750}, {0.531250, 0.093750}, {0.593750, 0.093750}, {0.593750, 0.093750}, //
    {0.656250, 0.093750}, {0.656250, 0.093750}, {0.718750, 0.093750}, {0.718750, 0.093750}, //
    {0.781250, 0.093750}, {0.781250, 0.093750}, {0.843750, 0.093750}, {0.843750, 0.093750}, //
    {0.906250, 0.093750}, {0.906250, 0.093750}, {0.968750, 0.093750}, {0.968750, 0.093750}, //
    {0.031250, 0.156250}, {0.031250, 0.156250}, {0.093750, 0.156250}, {0.093750, 0.156250}, //
    {0.156250, 0.156250}, {0.156250, 0.156250}, {0.218750, 0.156250}, {0.218750, 0.156250}, //
    {0.281250, 0.156250}, {0.281250, 0.156250}, {0.343750, 0.156250}, {0.343750, 0.156250}, //
    {0.406250, 0.156250}, {0.406250, 0.156250}, {0.468750, 0.156250}, {0.468750, 0.156250}, //
    {0.531250, 0.156250}, {0.531250, 0.156250}, {0.593750, 0.156250}, {0.593750, 0.156250}, //
    {0.656250, 0.156250}, {0.656250, 0.156250}, {0.718750, 0.156250}, {0.718750, 0.156250}, //
    {0.781250, 0.156250}, {0.781250, 0.156250}, {0.843750, 0.156250}, {0.843750, 0.156250}, //
    {0.906250, 0.156250}, {0.906250, 0.156250}, {0.968750, 0.156250}, {0.968750, 0.156250}, //
    {0.031250, 0.218750}, {0.031250, 0.218750}, {0.093750, 0.218750}, {0.093750, 0.218750}, //
    {0.156250, 0.218750}, {0.156250, 0.218750}, {0.218750, 0.218750}, {0.218750, 0.218750}, //
    {0.281250, 0.218750}, {0.281250, 0.218750}, {0.343750, 0.218750}, {0.343750, 0.218750}, //
    {0.406250, 0.218750}, {0.406250, 0.218750}, {0.468750, 0.218750}, {0.468750, 0.218750}, //
    {0.531250, 0.218750}, {0.531250, 0.218750}, {0.593750, 0.218750}, {0.593750, 0.218750}, //
    {0.656250, 0.218750}, {0.656250, 0.218750}, {0.718750, 0.218750}, {0.718750, 0.218750}, //
    {0.781250, 0.218750}, {0.781250, 0.218750}, {0.843750, 0.218750}, {0.843750, 0.218750}, //
    {0.906250, 0.218750}, {0.906250, 0.218750}, {0.968750, 0.218750}, {0.968750, 0.218750}, //
    {0.031250, 0.281250}, {0.031250, 0.281250}, {0.093750, 0.281250}, {0.093750, 0.281250}, //
    {0.156250, 0.281250}, {0.156250, 0.281250}, {0.218750, 0.281250}, {0.218750, 0.281250}, //
    {0.281250, 0.281250}, {0.281250, 0.281250}, {0.343750, 0.281250}, {0.343750, 0.281250}, //
    {0.406250, 0.281250}, {0.406250, 0.281250}, {0.468750, 0.281250}, {0.468750, 0.281250}, //
    {0.531250, 0.281250}, {0.531250, 0.281250}, {0.593750, 0.281250}, {0.593750, 0.281250}, //
    {0.656250, 0.281250}, {0.656250, 0.281250}, {0.718750, 0.281250}, {0.718750, 0.281250}, //
    {0.781250, 0.281250}, {0.781250, 0.281250}, {0.843750, 0.281250}, {0.843750, 0.281250}, //
    {0.906250, 0.281250}, {0.906250, 0.281250}, {0.968750, 0.281250}, {0.968750, 0.281250}, //
    {0.031250, 0.343750}, {0.031250, 0.343750}, {0.093750, 0.343750}, {0.093750, 0.343750}, //
    {0.156250, 0.343750}, {0.156250, 0.343750}, {0.218750, 0.343750}, {0.218750, 0.343750}, //
    {0.281250, 0.343750}, {0.281250, 0.343750}, {0.343750, 0.343750}, {0.343750, 0.343750}, //
    {0.406250, 0.343750}, {0.406250, 0.343750}, {0.468750, 0.343750}, {0.468750, 0.343750}, //
    {0.531250, 0.343750}, {0.531250, 0.343750}, {0.593750, 0.343750}, {0.593750, 0.343750}, //
    {0.656250, 0.343750}, {0.656250, 0.343750}, {0.718750, 0.343750}, {0.718750, 0.343750}, //
    {0.781250, 0.343750}, {0.781250, 0.343750}, {0.843750, 0.343750}, {0.843750, 0.343750}, //
    {0.906250, 0.343750}, {0.906250, 0.343750}, {0.968750, 0.343750}, {0.968750, 0.343750}, //
    {0.031250, 0.406250}, {0.031250, 0.406250}, {0.093750, 0.406250}, {0.093750, 0.406250}, //
    {0.156250, 0.406250}, {0.156250, 0.406250}, {0.218750, 0.406250}, {0.218750, 0.406250}, //
    {0.281250, 0.406250}, {0.281250, 0.406250}, {0.343750, 0.406250}, {0.343750, 0.406250}, //
    {0.406250, 0.406250}, {0.406250, 0.406250}, {0.468750, 0.406250}, {0.468750, 0.406250}, //
    {0.531250, 0.406250}, {0.531250, 0.406250}, {0.593750, 0.406250}, {0.593750, 0.406250}, //
    {0.656250, 0.406250}, {0.656250, 0.406250}, {0.718750, 0.406250}, {0.718750, 0.406250}, //
    {0.781250, 0.406250}, {0.781250, 0.406250}, {0.843750, 0.406250}, {0.843750, 0.406250}, //
    {0.906250, 0.406250}, {0.906250, 0.406250}, {0.968750, 0.406250}, {0.968750, 0.406250}, //
    {0.031250, 0.468750}, {0.031250, 0.468750}, {0.093750, 0.468750}, {0.093750, 0.468750}, //
    {0.156250, 0.468750}, {0.156250, 0.468750}, {0.218750, 0.468750}, {0.218750, 0.468750}, //
    {0.281250, 0.468750}, {0.281250, 0.468750}, {0.343750, 0.468750}, {0.343750, 0.468750}, //
    {0.406250, 0.468750}, {0.406250, 0.468750}, {0.468750, 0.468750}, {0.468750, 0.468750}, //
    {0.531250, 0.468750}, {0.531250, 0.468750}, {0.593750, 0.468750}, {0.593750, 0.468750}, //
    {0.656250, 0.468750}, {0.656250, 0.468750}, {0.718750, 0.468750}, {0.718750, 0.468750}, //
    {0.781250, 0.468750}, {0.781250, 0.468750}, {0.843750, 0.468750}, {0.843750, 0.468750}, //
    {0.906250, 0.468750}, {0.906250, 0.468750}, {0.968750, 0.468750}, {0.968750, 0.468750}, //
    {0.031250, 0.531250}, {0.031250, 0.531250}, {0.093750, 0.531250}, {0.093750, 0.531250}, //
    {0.156250, 0.531250}, {0.156250, 0.531250}, {0.218750, 0.531250}, {0.218750, 0.531250}, //
    {0.281250, 0.531250}, {0.281250, 0.531250}, {0.343750, 0.531250}, {0.343750, 0.531250}, //
    {0.406250, 0.531250}, {0.406250, 0.531250}, {0.468750, 0.531250}, {0.468750, 0.531250}, //
    {0.531250, 0.531250}, {0.531250, 0.531250}, {0.593750, 0.531250}, {0.593750, 0.531250}, //
    {0.656250, 0.531250}, {0.656250, 0.531250}, {0.718750, 0.531250}, {0.718750, 0.531250}, //
    {0.781250, 0.531250}, {0.781250, 0.531250}, {0.843750, 0.531250}, {0.843750, 0.531250}, //
    {0.906250, 0.531250}, {0.906250, 0.531250}, {0.968750, 0.531250}, {0.968750, 0.531250}, //
    {0.031250, 0.593750}, {0.031250, 0.593750}, {0.093750, 0.593750}, {0.093750, 0.593750}, //
    {0.156250, 0.593750}, {0.156250, 0.593750}, {0.218750, 0.593750}, {0.218750, 0.593750}, //
    {0.281250, 0.593750}, {0.281250, 0.593750}, {0.343750, 0.593750}, {0.343750, 0.593750}, //
    {0.406250, 0.593750}, {0.406250, 0.593750}, {0.468750, 0.593750}, {0.468750, 0.593750}, //
    {0.531250, 0.593750}, {0.531250, 0.593750}, {0.593750, 0.593750}, {0.593750, 0.593750}, //
    {0.656250, 0.593750}, {0.656250, 0.593750}, {0.718750, 0.593750}, {0.718750, 0.593750}, //
    {0.781250, 0.593750}, {0.781250, 0.593750}, {0.843750, 0.593750}, {0.843750, 0.593750}, //
    {0.906250, 0.593750}, {0.906250, 0.593750}, {0.968750, 0.593750}, {0.968750, 0.593750}, //
    {0.031250, 0.656250}, {0.031250, 0.656250}, {0.093750, 0.656250}, {0.093750, 0.656250}, //
    {0.156250, 0.656250}, {0.156250, 0.656250}, {0.218750, 0.656250}, {0.218750, 0.656250}, //
    {0.281250, 0.656250}, {0.281250, 0.656250}, {0.343750, 0.656250}, {0.343750, 0.656250}, //
    {0.406250, 0.656250}, {0.406250, 0.656250}, {0.468750, 0.656250}, {0.468750, 0.656250}, //
    {0.531250, 0.656250}, {0.531250, 0.656250}, {0.593750, 0.656250}, {0.593750, 0.656250}, //
    {0.656250, 0.656250}, {0.656250, 0.656250}, {0.718750, 0.656250}, {0.718750, 0.656250}, //
    {0.781250, 0.656250}, {0.781250, 0.656250}, {0.843750, 0.656250}, {0.843750, 0.656250}, //
    {0.906250, 0.656250}, {0.906250, 0.656250}, {0.968750, 0.656250}, {0.968750, 0.656250}, //
    {0.031250, 0.718750}, {0.031250, 0.718750}, {0.093750, 0.718750}, {0.093750, 0.718750}, //
    {0.156250, 0.718750}, {0.156250, 0.718750}, {0.218750, 0.718750}, {0.218750, 0.718750}, //
    {0.281250, 0.718750}, {0.281250, 0.718750}, {0.343750, 0.718750}, {0.343750, 0.718750}, //
    {0.406250, 0.718750}, {0.406250, 0.718750}, {0.468750, 0.718750}, {0.468750, 0.718750}, //
    {0.531250, 0.718750}, {0.531250, 0.718750}, {0.593750, 0.718750}, {0.593750, 0.718750}, //
    {0.656250, 0.718750}, {0.656250, 0.718750}, {0.718750, 0.718750}, {0.718750, 0.718750}, //
    {0.781250, 0.718750}, {0.781250, 0.718750}, {0.843750, 0.718750}, {0.843750, 0.718750}, //
    {0.906250, 0.718750}, {0.906250, 0.718750}, {0.968750, 0.718750}, {0.968750, 0.718750}, //
    {0.031250, 0.781250}, {0.031250, 0.781250}, {0.093750, 0.781250}, {0.093750, 0.781250}, //
    {0.156250, 0.781250}, {0.156250, 0.781250}, {0.218750, 0.781250}, {0.218750, 0.781250}, //
    {0.281250, 0.781250}, {0.281250, 0.781250}, {0.343750, 0.781250}, {0.343750, 0.781250}, //
    {0.406250, 0.781250}, {0.406250, 0.781250}, {0.468750, 0.781250}, {0.468750, 0.781250}, //
    {0.531250, 0.781250}, {0.531250, 0.781250}, {0.593750, 0.781250}, {0.593750, 0.781250}, //
    {0.656250, 0.781250}, {0.656250, 0.781250}, {0.718750, 0.781250}, {0.718750, 0.781250}, //
    {0.781250, 0.781250}, {0.781250, 0.781250}, {0.843750, 0.781250}, {0.843750, 0.781250}, //
    {0.906250, 0.781250}, {0.906250, 0.781250}, {0.968750, 0.781250}, {0.968750, 0.781250}, //
    {0.031250, 0.843750}, {0.031250, 0.843750}, {0.093750, 0.843750}, {0.093750, 0.843750}, //
    {0.156250, 0.843750}, {0.156250, 0.843750}, {0.218750, 0.843750}, {0.218750, 0.843750}, //
    {0.281250, 0.843750}, {0.281250, 0.843750}, {0.343750, 0.843750}, {0.343750, 0.843750}, //
    {0.406250, 0.843750}, {0.406250, 0.843750}, {0.468750, 0.843750}, {0.468750, 0.843750}, //
    {0.531250, 0.843750}, {0.531250, 0.843750}, {0.593750, 0.843750}, {0.593750, 0.843750}, //
    {0.656250, 0.843750}, {0.656250, 0.843750}, {0.718750, 0.843750}, {0.718750, 0.843750}, //
    {0.781250, 0.843750}, {0.781250, 0.843750}, {0.843750, 0.843750}, {0.843750, 0.843750}, //
    {0.906250, 0.843750}, {0.906250, 0.843750}, {0.968750, 0.843750}, {0.968750, 0.843750}, //
    {0.031250, 0.906250}, {0.031250, 0.906250}, {0.093750, 0.906250}, {0.093750, 0.906250}, //
    {0.156250, 0.906250}, {0.156250, 0.906250}, {0.218750, 0.906250}, {0.218750, 0.906250}, //
    {0.281250, 0.906250}, {0.281250, 0.906250}, {0.343750, 0.906250}, {0.343750, 0.906250}, //
    {0.406250, 0.906250}, {0.406250, 0.906250}, {0.468750, 0.906250}, {0.468750, 0.906250}, //
    {0.531250, 0.906250}, {0.531250, 0.906250}, {0.593750, 0.906250}, {0.593750, 0.906250}, //
    {0.656250, 0.906250}, {0.656250, 0.906250}, {0.718750, 0.906250}, {0.718750, 0.906250}, //
    {0.781250, 0.906250}, {0.781250, 0.906250}, {0.843750, 0.906250}, {0.843750, 0.906250}, //
    {0.906250, 0.906250}, {0.906250, 0.906250}, {0.968750, 0.906250}, {0.968750, 0.906250}, //
    {0.031250, 0.968750}, {0.031250, 0.968750}, {0.093750, 0.968750}, {0.093750, 0.968750}, //
    {0.156250, 0.968750}, {0.156250, 0.968750}, {0.218750, 0.968750}, {0.218750, 0.968750}, //
    {0.281250, 0.968750}, {0.281250, 0.968750}, {0.343750, 0.968750}, {0.343750, 0.968750}, //
    {0.406250, 0.968750}, {0.406250, 0.968750}, {0.468750, 0.968750}, {0.468750, 0.968750}, //
    {0.531250, 0.968750}, {0.531250, 0.968750}, {0.593750, 0.968750}, {0.593750, 0.968750}, //
    {0.656250, 0.968750}, {0.656250, 0.968750}, {0.718750, 0.968750}, {0.718750, 0.968750}, //
    {0.781250, 0.968750}, {0.781250, 0.968750}, {0.843750, 0.968750}, {0.843750, 0.968750}, //
    {0.906250, 0.968750}, {0.906250, 0.968750}, {0.968750, 0.968750}, {0.968750, 0.968750}, //
    {0.062500, 0.062500}, {0.062500, 0.062500}, {0.062500, 0.062500}, {0.062500, 0.062500}, //
    {0.062500, 0.062500}, {0.062500, 0.062500}, {0.187500, 0.062500}, {0.187500, 0.062500}, //
    {0.187500, 0.062500}, {0.187500, 0.062500}, {0.187500, 0.062500}, {0.187500, 0.062500}, //
    {0.312500, 0.062500}, {0.312500, 0.062500}, {0.312500, 0.062500}, {0.312500, 0.062500}, //
    {0.312500, 0.062500}, {0.312500, 0.062500}, {0.437500, 0.062500}, {0.437500, 0.062500}, //
    {0.437500, 0.062500}, {0.437500, 0.062500}, {0.437500, 0.062500}, {0.437500, 0.062500}, //
    {0.562500, 0.062500}, {0.562500, 0.062500}, {0.562500, 0.062500}, {0.562500, 0.062500}, //
    {0.562500, 0.062500}, {0.562500, 0.062500}, {0.687500, 0.062500}, {0.687500, 0.062500}, //
    {0.687500, 0.062500}, {0.687500, 0.062500}, {0.687500, 0.062500}, {0.687500, 0.062500}, //
    {0.812500, 0.062500}, {0.812500, 0.062500}, {0.812500, 0.062500}, {0.812500, 0.062500}, //
    {0.812500, 0.062500}, {0.812500, 0.062500}, {0.937500, 0.062500}, {0.937500, 0.062500}, //
    {0.937500, 0.062500}, {0.937500, 0.062500}, {0.937500, 0.062500}, {0.937500, 0.062500}, //
    {0.062500, 0.187500}, {0.062500, 0.187500}, {0.062500, 0.187500}, {0.062500, 0.187500}, //
    {0.062500, 0.187500}, {0.062500, 0.187500}, {0.187500, 0.187500}, {0.187500, 0.187500}, //
    {0.187500, 0.187500}, {0.187500, 0.187500}, {0.187500, 0.187500}, {0.187500, 0.187500}, //
    {0.312500, 0.187500}, {0.312500, 0.187500}, {0.312500, 0.187500}, {0.312500, 0.187500}, //
    {0.312500, 0.187500}, {0.312500, 0.187500}, {0.437500, 0.187500}, {0.437500, 0.187500}, //
    {0.437500, 0.187500}, {0.437500, 0.187500}, {0.437500, 0.187500}, {0.437500, 0.187500}, //
    {0.562500, 0.187500}, {0.562500, 0.187500}, {0.562500, 0.187500}, {0.562500, 0.187500}, //
    {0.562500, 0.187500}, {0.562500, 0.187500}, {0.687500, 0.187500}, {0.687500, 0.187500}, //
    {0.687500, 0.187500}, {0.687500, 0.187500}, {0.687500, 0.187500}, {0.687500, 0.187500}, //
    {0.812500, 0.187500}, {0.812500, 0.187500}, {0.812500, 0.187500}, {0.812500, 0.187500}, //
    {0.812500, 0.187500}, {0.812500, 0.187500}, {0.937500, 0.187500}, {0.937500, 0.187500}, //
    {0.937500, 0.187500}, {0.937500, 0.187500}, {0.937500, 0.187500}, {0.937500, 0.187500}, //
    {0.062500, 0.312500}, {0.062500, 0.312500}, {0.062500, 0.312500}, {0.062500, 0.312500}, //
    {0.062500, 0.312500}, {0.062500, 0.312500}, {0.187500, 0.312500}, {0.187500, 0.312500}, //
    {0.187500, 0.312500}, {0.187500, 0.312500}, {0.187500, 0.312500}, {0.187500, 0.312500}, //
    {0.312500, 0.312500}, {0.312500, 0.312500}, {0.312500, 0.312500}, {0.312500, 0.312500}, //
    {0.312500, 0.312500}, {0.312500, 0.312500}, {0.437500, 0.312500}, {0.437500, 0.312500}, //
    {0.437500, 0.312500}, {0.437500, 0.312500}, {0.437500, 0.312500}, {0.437500, 0.312500}, //
    {0.562500, 0.312500}, {0.562500, 0.312500}, {0.562500, 0.312500}, {0.562500, 0.312500}, //
    {0.562500, 0.312500}, {0.562500, 0.312500}, {0.687500, 0.312500}, {0.687500, 0.312500}, //
    {0.687500, 0.312500}, {0.687500, 0.312500}, {0.687500, 0.312500}, {0.687500, 0.312500}, //
    {0.812500, 0.312500}, {0.812500, 0.312500}, {0.812500, 0.312500}, {0.812500, 0.312500}, //
    {0.812500, 0.312500}, {0.812500, 0.312500}, {0.937500, 0.312500}, {0.937500, 0.312500}, //
    {0.937500, 0.312500}, {0.937500, 0.312500}, {0.937500, 0.312500}, {0.937500, 0.312500}, //
    {0.062500, 0.437500}, {0.062500, 0.437500}, {0.062500, 0.437500}, {0.062500, 0.437500}, //
    {0.062500, 0.437500}, {0.062500, 0.437500}, {0.187500, 0.437500}, {0.187500, 0.437500}, //
    {0.187500, 0.437500}, {0.187500, 0.437500}, {0.187500, 0.437500}, {0.187500, 0.437500}, //
    {0.312500, 0.437500}, {0.312500, 0.437500}, {0.312500, 0.437500}, {0.312500, 0.437500}, //
    {0.312500, 0.437500}, {0.312500, 0.437500}, {0.437500, 0.437500}, {0.437500, 0.437500}, //
    {0.437500, 0.437500}, {0.437500, 0.437500}, {0.437500, 0.437500}, {0.437500, 0.437500}, //
    {0.562500, 0.437500}, {0.562500, 0.437500}, {0.562500, 0.437500}, {0.562500, 0.437500}, //
    {0.562500, 0.437500}, {0.562500, 0.437500}, {0.687500, 0.437500}, {0.687500, 0.437500}, //
    {0.687500, 0.437500}, {0.687500, 0.437500}, {0.687500, 0.437500}, {0.687500, 0.437500}, //
    {0.812500, 0.437500}, {0.812500, 0.437500}, {0.812500, 0.437500}, {0.812500, 0.437500}, //
    {0.812500, 0.437500}, {0.812500, 0.437500}, {0.937500, 0.437500}, {0.937500, 0.437500}, //
    {0.937500, 0.437500}, {0.937500, 0.437500}, {0.937500, 0.437500}, {0.937500, 0.437500}, //
    {0.062500, 0.562500}, {0.062500, 0.562500}, {0.062500, 0.562500}, {0.062500, 0.562500}, //
    {0.062500, 0.562500}, {0.062500, 0.562500}, {0.187500, 0.562500}, {0.187500, 0.562500}, //
    {0.187500, 0.562500}, {0.187500, 0.562500}, {0.187500, 0.562500}, {0.187500, 0.562500}, //
    {0.312500, 0.562500}, {0.312500, 0.562500}, {0.312500, 0.562500}, {0.312500, 0.562500}, //
    {0.312500, 0.562500}, {0.312500, 0.562500}, {0.437500, 0.562500}, {0.437500, 0.562500}, //
    {0.437500, 0.562500}, {0.437500, 0.562500}, {0.437500, 0.562500}, {0.437500, 0.562500}, //
    {0.562500, 0.562500}, {0.562500, 0.562500}, {0.562500, 0.562500}, {0.562500, 0.562500}, //
    {0.562500, 0.562500}, {0.562500, 0.562500}, {0.687500, 0.562500}, {0.687500, 0.562500}, //
    {0.687500, 0.562500}, {0.687500, 0.562500}, {0.687500, 0.562500}, {0.687500, 0.562500}, //
    {0.812500, 0.562500}, {0.812500, 0.562500}, {0.812500, 0.562500}, {0.812500, 0.562500}, //
    {0.812500, 0.562500}, {0.812500, 0.562500}, {0.937500, 0.562500}, {0.937500, 0.562500}, //
    {0.937500, 0.562500}, {0.937500, 0.562500}, {0.937500, 0.562500}, {0.937500, 0.562500}, //
    {0.062500, 0.687500}, {0.062500, 0.687500}, {0.062500, 0.687500}, {0.062500, 0.687500}, //
    {0.062500, 0.687500}, {0.062500, 0.687500}, {0.187500, 0.687500}, {0.187500, 0.687500}, //
    {0.187500, 0.687500}, {0.187500, 0.687500}, {0.187500, 0.687500}, {0.187500, 0.687500}, //
    {0.312500, 0.687500}, {0.312500, 0.687500}, {0.312500, 0.687500}, {0.312500, 0.687500}, //
    {0.312500, 0.687500}, {0.312500, 0.687500}, {0.437500, 0.687500}, {0.437500, 0.687500}, //
    {0.437500, 0.687500}, {0.437500, 0.687500}, {0.437500, 0.687500}, {0.437500, 0.687500}, //
    {0.562500, 0.687500}, {0.562500, 0.687500}, {0.562500, 0.687500}, {0.562500, 0.687500}, //
    {0.562500, 0.687500}, {0.562500, 0.687500}, {0.687500, 0.687500}, {0.687500, 0.687500}, //
    {0.687500, 0.687500}, {0.687500, 0.687500}, {0.687500, 0.687500}, {0.687500, 0.687500}, //
    {0.812500, 0.687500}, {0.812500, 0.687500}, {0.812500, 0.687500}, {0.812500, 0.687500}, //
    {0.812500, 0.687500}, {0.812500, 0.687500}, {0.937500, 0.687500}, {0.937500, 0.687500}, //
    {0.937500, 0.687500}, {0.937500, 0.687500}, {0.937500, 0.687500}, {0.937500, 0.687500}, //
    {0.062500, 0.812500}, {0.062500, 0.812500}, {0.062500, 0.812500}, {0.062500, 0.812500}, //
    {0.062500, 0.812500}, {0.062500, 0.812500}, {0.187500, 0.812500}, {0.187500, 0.812500}, //
    {0.187500, 0.812500}, {0.187500, 0.812500}, {0.187500, 0.812500}, {0.187500, 0.812500}, //
    {0.312500, 0.812500}, {0.312500, 0.812500}, {0.312500, 0.812500}, {0.312500, 0.812500}, //
    {0.312500, 0.812500}, {0.312500, 0.812500}, {0.437500, 0.812500}, {0.437500, 0.812500}, //
    {0.437500, 0.812500}, {0.437500, 0.812500}, {0.437500, 0.812500}, {0.437500, 0.812500}, //
    {0.562500, 0.812500}, {0.562500, 0.812500}, {0.562500, 0.812500}, {0.562500, 0.812500}, //
    {0.562500, 0.812500}, {0.562500, 0.812500}, {0.687500, 0.812500}, {0.687500, 0.812500}, //
    {0.687500, 0.812500}, {0.687500, 0.812500}, {0.687500, 0.812500}, {0.687500, 0.812500}, //
    {0.812500, 0.812500}, {0.812500, 0.812500}, {0.812500, 0.812500}, {0.812500, 0.812500}, //
    {0.812500, 0.812500}, {0.812500, 0.812500}, {0.937500, 0.812500}, {0.937500, 0.812500}, //
    {0.937500, 0.812500}, {0.937500, 0.812500}, {0.937500, 0.812500}, {0.937500, 0.812500}, //
    {0.062500, 0.937500}, {0.062500, 0.937500}, {0.062500, 0.937500}, {0.062500, 0.937500}, //
    {0.062500, 0.937500}, {0.062500, 0.937500}, {0.187500, 0.937500}, {0.187500, 0.937500}, //
    {0.187500, 0.937500}, {0.187500, 0.937500}, {0.187500, 0.937500}, {0.187500, 0.937500}, //
    {0.312500, 0.937500}, {0.312500, 0.937500}, {0.312500, 0.937500}, {0.312500, 0.937500}, //
    {0.312500, 0.937500}, {0.312500, 0.937500}, {0.437500, 0.937500}, {0.437500, 0.937500}, //
    {0.437500, 0.937500}, {0.437500, 0.937500}, {0.437500, 0.937500}, {0.437500, 0.937500}, //
    {0.562500, 0.937500}, {0.562500, 0.937500}, {0.562500, 0.937500}, {0.562500, 0.937500}, //
    {0.562500, 0.937500}, {0.562500, 0.937500}, {0.687500, 0.937500}, {0.687500, 0.937500}, //
    {0.687500, 0.937500}, {0.687500, 0.937500}, {0.687500, 0.937500}, {0.687500, 0.937500}, //
    {0.812500, 0.937500}, {0.812500, 0.937500}, {0.812500, 0.937500}, {0.812500, 0.937500}, //
    {0.812500, 0.937500}, {0.812500, 0.937500}, {0.937500, 0.937500}, {0.937500, 0.937500}, //
    {0.937500, 0.937500}, {0.937500, 0.937500}, {0.937500, 0.937500}, {0.937500, 0.937500}, //
};

static std::vector<Palm7KP>
runHandDetector(struct ht_view *htv, cv::Mat &raw_input)
{
	const OrtApi *g_ort = htv->htd->ort_api;

	cv::Mat img;

	constexpr int hd_size = 128;
	constexpr size_t nb_planes = 3;
	constexpr size_t size = hd_size * hd_size * nb_planes;
	constexpr int inputTensorSize = size * sizeof(float);

	cv::Matx23f back_from_blackbar = blackbar(raw_input, img, {hd_size, hd_size});

	float scale_factor = back_from_blackbar(0, 0); // 960/128
	assert(img.isContinuous());
	constexpr float mean = 128.0f;
	constexpr float std = 128.0f;

	std::vector<float> real_thing(size);

	if (htv->htd->startup_config.palm_detection_use_mediapipe) {
		std::vector<uint8_t> combined_planes(size);
		planarize(img, combined_planes.data());
		for (size_t i = 0; i < size; i++) {
			float val = (float)combined_planes[i];
			real_thing[i] = (val - mean) / std;
		}
	} else {

		assert(img.isContinuous());

		for (size_t i = 0; i < size; i++) {
			int val = img.data[i];

			real_thing[i] = (val - mean) / std;
		}
	}

	// Convenience
	struct ModelInfo *model_hd = &htv->detection_model;

	// If this is non-NULL, ONNX will explode and won't tell you why!
	OrtValue *input_tensor = nullptr;

	ORT_CHECK(g_ort, g_ort->CreateTensorWithDataAsOrtValue(
	                     model_hd->memoryInfo, real_thing.data(), inputTensorSize, model_hd->input_shape.data(),
	                     model_hd->input_shape.size(), ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &input_tensor));

	// Cargo-culted
	assert(input_tensor != nullptr);
	int is_tensor;
	ORT_CHECK(g_ort, g_ort->IsTensor(input_tensor, &is_tensor));
	assert(is_tensor);

	// If any of these are non-NULL, ONNX will explode and won't tell you why! Be extremely paranoid!
	OrtValue *output_tensor[2] = {nullptr, nullptr};
	ORT_CHECK(g_ort, g_ort->Run(model_hd->session, nullptr, model_hd->input_names.data(),
	                            (const OrtValue *const *)&input_tensor, model_hd->input_names.size(),
	                            model_hd->output_names.data(), model_hd->output_names.size(), output_tensor));

	ORT_CHECK(g_ort, g_ort->IsTensor(output_tensor[0], &is_tensor));
	assert(is_tensor);

	float *classificators = nullptr;
	float *regressors = nullptr;

	ORT_CHECK(g_ort,
	          g_ort->GetTensorMutableData(output_tensor[0], (void **)&classificators)); // Totally won't segfault!
	ORT_CHECK(g_ort, g_ort->GetTensorMutableData(output_tensor[1], (void **)&regressors));
	// We need to free these!

	float *rg = regressors; // shorter name

	std::vector<NMSPalm> detections;
	int count = 0;

	std::vector<Palm7KP> output;
	std::vector<NMSPalm> nms_palms;

	OrtTensorTypeAndShapeInfo *classificators_info;
	ORT_CHECK(g_ort, g_ort->GetTensorTypeAndShape(output_tensor[0], &classificators_info));

	size_t classificators_size;
	ORT_CHECK(g_ort, g_ort->GetTensorShapeElementCount(classificators_info, &classificators_size));
	assert(classificators_size == 896);

	OrtTensorTypeAndShapeInfo *regressors_info;
	ORT_CHECK(g_ort, g_ort->GetTensorTypeAndShape(output_tensor[1], &regressors_info));

	size_t regressors_size;
	ORT_CHECK(g_ort, g_ort->GetTensorShapeElementCount(regressors_info, &regressors_size));
	assert(regressors_size == 896 * 18);

	g_ort->ReleaseTensorTypeAndShapeInfo(classificators_info);
	g_ort->ReleaseTensorTypeAndShapeInfo(regressors_info);

	for (size_t i = 0; i < classificators_size; ++i) {
		const float score = 1.0 / (1.0 + exp(-classificators[i]));

		// Let a lot of detections in - they'll be slowly rejected later
		if (score <= htv->htd->dynamic_config.nms_threshold.val) {
			continue;
		}

		const struct anchor *anchor = &anchors[i];

		// Boundary box.
		NMSPalm det;

		float anchx = anchor->x * 128;
		float anchy = anchor->y * 128;

		float shiftx = regressors[i * 18];
		float shifty = regressors[i * 18 + 1];

		float w = regressors[i * 18 + 2];
		float h = regressors[i * 18 + 3];

		float cx = shiftx + anchx;
		float cy = shifty + anchy;

		struct xrt_vec2 *kps = det.keypoints;

		kps[0] = {rg[i * 18 + 4], rg[i * 18 + 5]};
		kps[1] = {rg[i * 18 + 6], rg[i * 18 + 7]};
		kps[2] = {rg[i * 18 + 8], rg[i * 18 + 9]};
		kps[3] = {rg[i * 18 + 10], rg[i * 18 + 11]};
		kps[4] = {rg[i * 18 + 12], rg[i * 18 + 13]};
		kps[5] = {rg[i * 18 + 14], rg[i * 18 + 15]};
		kps[6] = {rg[i * 18 + 16], rg[i * 18 + 17]};


		for (int i = 0; i < 7; i++) {
			struct xrt_vec2 *b = &kps[i];
			b->x += anchx;
			b->y += anchy;
		}

		det.bbox.w = w;
		det.bbox.h = h;
		det.bbox.cx = cx;
		det.bbox.cy = cy;
		det.confidence = score;
		detections.push_back(det);
		count++;

		if (htv->htd->debug_scribble && (htv->htd->dynamic_config.scribble_raw_detections)) {
			xrt_vec2 center = transformVecBy2x3(xrt_vec2{cx, cy}, back_from_blackbar);

			float sz = det.bbox.w * scale_factor;

			cv::rectangle(htv->debug_out_to_this,
			              {(int)(center.x - (sz / 2)), (int)(center.y - (sz / 2)), (int)sz, (int)sz},
			              hsv2rgb(0.0f, math_map_ranges(det.confidence, 0.0f, 1.0f, 1.5f, -0.1f),
			                      math_map_ranges(det.confidence, 0.0f, 1.0f, 0.2f, 1.4f)),
			              1);

			for (int i = 0; i < 7; i++) {
				handDot(htv->debug_out_to_this, transformVecBy2x3(kps[i], back_from_blackbar),
				        det.confidence * 7, ((float)i) * (360.0f / 7.0f), det.confidence, 1);
			}
		}



		int square = fmax(w, h);

		square = square / scale_factor;
	}

	if (count == 0) {
		goto cleanup;
	}

	nms_palms = filterBoxesWeightedAvg(detections, htv->htd->dynamic_config.nms_iou.val);



	for (NMSPalm cooler : nms_palms) {

		// Display box

		struct xrt_vec2 tl = {cooler.bbox.cx - cooler.bbox.w / 2, cooler.bbox.cy - cooler.bbox.h / 2};
		struct xrt_vec2 bob = transformVecBy2x3(tl, back_from_blackbar);
		float sz = cooler.bbox.w * scale_factor;

		if (htv->htd->debug_scribble && htv->htd->dynamic_config.scribble_nms_detections) {
			cv::rectangle(htv->debug_out_to_this, {(int)bob.x, (int)bob.y, (int)sz, (int)sz},
			              hsv2rgb(180.0f, math_map_ranges(cooler.confidence, 0.0f, 1.0f, 0.8f, -0.1f),
			                      math_map_ranges(cooler.confidence, 0.0f, 1.0f, 0.2f, 1.4f)),
			              2);
			for (int i = 0; i < 7; i++) {
				handDot(htv->debug_out_to_this,
				        transformVecBy2x3(cooler.keypoints[i], back_from_blackbar),
				        cooler.confidence * 14, ((float)i) * (360.0f / 7.0f), cooler.confidence, 3);
			}
		}


		Palm7KP this_element;

		for (int i = 0; i < 7; i++) {
			struct xrt_vec2 b = cooler.keypoints[i];
			this_element.kps[i] = transformVecBy2x3(b, back_from_blackbar);
		}
		this_element.confidence = cooler.confidence;

		output.push_back(this_element);
	}

cleanup:
	g_ort->ReleaseValue(output_tensor[0]);
	g_ort->ReleaseValue(output_tensor[1]);
	g_ort->ReleaseValue(input_tensor);
	return output;
}


static void
addSlug(struct ht_device *htd, const char *suffix, char *out)
{
	strcpy(out, htd->startup_config.model_slug);
	strcat(out, suffix);
}

static void
initKeypointEstimator(struct ht_device *htd, ht_view *htv)
{
	struct ModelInfo *model_ke = &htv->keypoint_model;
	const OrtApi *g_ort = htd->ort_api;
	OrtSessionOptions *opts = nullptr;

	ORT_CHECK(g_ort, g_ort->CreateSessionOptions(&opts));

	ORT_CHECK(g_ort, g_ort->SetSessionGraphOptimizationLevel(opts, ORT_ENABLE_ALL));
	ORT_CHECK(g_ort, g_ort->SetIntraOpNumThreads(opts, 1));

	char modelLocation[1024];
	if (htd->startup_config.keypoint_estimation_use_mediapipe) {
		addSlug(htd, "hand_landmark_MEDIAPIPE.onnx", modelLocation);
	} else {
		addSlug(htd, "hand_landmark_COLLABORA.onnx", modelLocation);
	}
	ORT_CHECK(g_ort, g_ort->CreateSession(htd->ort_env, modelLocation, opts, &model_ke->session));
	g_ort->ReleaseSessionOptions(opts);

	model_ke->input_shape.push_back(1);
	model_ke->input_shape.push_back(3);
	model_ke->input_shape.push_back(224);
	model_ke->input_shape.push_back(224);

	model_ke->input_names.push_back("input_1");

	model_ke->output_names.push_back("Identity");
	model_ke->output_names.push_back("Identity_1");
	model_ke->output_names.push_back("Identity_2");

	model_ke->input_size_bytes = 224 * 224 * 3 * sizeof(float);

	ORT_CHECK(g_ort, g_ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &model_ke->memoryInfo));

	htv->run_keypoint_model = runKeypointEstimator;
}

static void
initHandDetector(struct ht_device *htd, ht_view *htv)
{
	struct ModelInfo *model_hd = &htv->detection_model;
	memset(model_hd, 0, sizeof(struct ModelInfo));

	const OrtApi *g_ort = htd->ort_api;
	OrtSessionOptions *opts = nullptr;
	ORT_CHECK(g_ort, g_ort->CreateSessionOptions(&opts));

	ORT_CHECK(g_ort, g_ort->SetSessionGraphOptimizationLevel(opts, ORT_ENABLE_ALL));
	ORT_CHECK(g_ort, g_ort->SetIntraOpNumThreads(opts, 1));

	char modelLocation[1024];

	// Hard-coded. Even though you can use the ONNX runtime's API to dynamically figure these out, that doesn't make
	// any sense because these don't change between runs, and if you are swapping models you have to do much more
	// than just change the input/output names.
	if (htd->startup_config.palm_detection_use_mediapipe) {
		addSlug(htd, "palm_detection_MEDIAPIPE.onnx", modelLocation);
		model_hd->input_shape.push_back(1);
		model_hd->input_shape.push_back(3);
		model_hd->input_shape.push_back(128);
		model_hd->input_shape.push_back(128);

		model_hd->input_names.push_back("input");
	} else {
		addSlug(htd, "palm_detection_COLLABORA.onnx", modelLocation);

		model_hd->input_shape.push_back(1);
		model_hd->input_shape.push_back(128);
		model_hd->input_shape.push_back(128);
		model_hd->input_shape.push_back(3);

		model_hd->input_names.push_back("input:0");
	}

	ORT_CHECK(g_ort, g_ort->CreateSession(htd->ort_env, modelLocation, opts, &model_hd->session));
	g_ort->ReleaseSessionOptions(opts);



	model_hd->output_names.push_back("classificators");
	model_hd->output_names.push_back("regressors");

	ORT_CHECK(g_ort, g_ort->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &model_hd->memoryInfo));

	htv->run_detection_model = runHandDetector;
}


static void
initOnnx(struct ht_device *htd)
{
	htd->ort_api = OrtGetApiBase()->GetApi(ORT_API_VERSION);
	ORT_CHECK(htd->ort_api, htd->ort_api->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "moses", &htd->ort_env));

	initHandDetector(htd, &htd->views[0]);
	initHandDetector(htd, &htd->views[1]);

	initKeypointEstimator(htd, &htd->views[0]);
	initKeypointEstimator(htd, &htd->views[1]);
}

static void
destroyModelInfo(struct ht_device *htd, ModelInfo *info)
{
	const OrtApi *g_ort = htd->ort_api;
	g_ort->ReleaseSession(info->session);
	g_ort->ReleaseMemoryInfo(info->memoryInfo);
	// Same deal as in ht_device - I'm mixing C and C++ idioms, so sometimes it's easier to just manually call their
	// destructors instead of figuring out some way to convince C++ to call them implicitly.
	info->output_names.~vector();
	info->input_names.~vector();
	info->input_shape.~vector();
}

void
destroyOnnx(struct ht_device *htd)
{
	destroyModelInfo(htd, &htd->views[0].keypoint_model);
	destroyModelInfo(htd, &htd->views[1].keypoint_model);
	destroyModelInfo(htd, &htd->views[0].detection_model);
	destroyModelInfo(htd, &htd->views[1].detection_model);
	htd->ort_api->ReleaseEnv(htd->ort_env);
}
