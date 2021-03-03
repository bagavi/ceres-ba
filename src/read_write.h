#pragma once
#include <fstream>

#include "c_api.h"
void write_data(const char* filename, BAData& ba_data, double lm_noise, double kf_noise, double bearing_noise);

unsigned int read_uint(std::ifstream& file);
double read_double(std::ifstream& file);
void read_camera(CameraFrames& camera, std::ifstream& file);
void read_lm_data(BALmData& lm_data, std::ifstream& file);
void read_kf_data(BAKfData& kf_data, std::ifstream& file);
void read_corresp_data(BACorresp& corresp_data, std::ifstream& file);