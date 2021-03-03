/*
This library is used to write&read (encode&decode) BAData. 
1. Used to create a fixed BAData set and run it across different ceres options
2. Used to save BAData obtained from rust codebase
*/
#include "read_write.h"

#include <filesystem>

#include "c_api.h"

using std::cout;
using std::endl;

void write_number(std::ofstream* file, double number) {
    *file << number << ",";
}

template <typename T>
void write_number_new_line(std::ofstream& file, T number) {
    file << number << endl;
}

void skip_lines(std::ifstream& file, int num_lines) {
    std::string line;
    while (num_lines > 0) {
        std::getline(file, line);
        --num_lines;
    }
}

unsigned int read_uint(std::ifstream& file) {
    std::string line;
    std::getline(file, line);
    return std::stoi(line);
}

double read_double(std::ifstream& file) {
    std::string line;
    std::getline(file, line);
    return std::stod(line);
}

template <typename T>
void write_array(std::ofstream* file, T array[], int size) {
    for (int i = 0; i < size; i++) {
        write_number(file, array[i]);
    }
    *file << endl;
}

template <typename T>
void read_array(std::ifstream& file, T array[], uint size) {
    // read the given line
    std::string line;
    std::getline(file, line);

    std::stringstream lineStream(line);
    std::string cell;
    uint index = 0;
    while (std::getline(lineStream, cell, ',')) {
        if (std::is_same<T, double>::value) {
            array[index] = std::stod(cell);
        } else if (std::is_same<T, unsigned int>::value) {
            array[index] = (unsigned int)std::stoi(cell);
        } else {
            throw;
        }
        index++;
    }
    if (index != size) {
        throw;
    }
}

void write_camera(CameraFrames* camera, const char* filename) {
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);  // append instead of overwrite
    write_number_new_line(file, camera->num_cams);
    write_array(&file, camera->t_fl_fx, camera->num_cams * AA_POSE_DIM);
}

void write_lm_data(BALmData* lm_data, const char* filename) {
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);  // append instead of overwrite
    write_number_new_line(file, lm_data->num_lms);
    write_array(&file, lm_data->lms_noisy, lm_data->num_lms * POINT_DIM);
    write_array(&file, lm_data->lms_gt, lm_data->num_lms * POINT_DIM);
}

void write_kf_data(BAKfData* kf_data, const char* filename) {
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);  // append instead of overwrite
    write_number_new_line(file, kf_data->num_kfs);
    write_array(&file, kf_data->kfs_noisy, kf_data->num_kfs * AA_POSE_DIM);
    write_array(&file, kf_data->kfs_gt, kf_data->num_kfs * AA_POSE_DIM);
}

void write_corresp_data(BACorresp* corresp_data, const char* filename) {
    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);  // append instead of overwrite
    write_number_new_line(file, corresp_data->num_corresp);
    write_array(&file, corresp_data->bearings, corresp_data->num_corresp * BEARING_DIM);
    write_array(&file, corresp_data->cam_index, corresp_data->num_corresp);
    write_array(&file, corresp_data->kf_index, corresp_data->num_corresp);
    write_array(&file, corresp_data->lm_index, corresp_data->num_corresp);
}

void write_data(const char* filename, BAData& ba_data, double bearing_noise, double lm_noise, double kf_noise) {
    std::filesystem::remove(filename);
    write_camera(ba_data.camera_frames, filename);
    write_lm_data(ba_data.lm_data, filename);
    write_kf_data(ba_data.var_kf_data, filename);
    write_corresp_data(ba_data.var_corresp, filename);
    write_kf_data(ba_data.fix_kf_data, filename);
    write_corresp_data(ba_data.fix_corresp, filename);

    std::ofstream file;
    file.open(filename, std::ios::out | std::ios::app);  // append instead of overwrite
    write_number_new_line(file, bearing_noise);
    write_number_new_line(file, kf_noise);
    write_number_new_line(file, lm_noise);
}

void read_camera(CameraFrames& camera, std::ifstream& file) {
    read_array(file, camera.t_fl_fx, camera.num_cams * AA_POSE_DIM);
}

void read_lm_data(BALmData& lm_data, std::ifstream& file) {
    read_array(file, lm_data.lms_noisy, lm_data.num_lms * POINT_DIM);
    read_array(file, lm_data.lms_gt, lm_data.num_lms * POINT_DIM);
}

void read_kf_data(BAKfData& kf_data, std::ifstream& file) {
    read_array(file, kf_data.kfs_noisy, kf_data.num_kfs * AA_POSE_DIM);
    read_array(file, kf_data.kfs_gt, kf_data.num_kfs * AA_POSE_DIM);
}

void read_corresp_data(BACorresp& corresp_data, std::ifstream& file) {
    read_array(file, corresp_data.bearings, corresp_data.num_corresp * BEARING_DIM);
    read_array(file, corresp_data.cam_index, corresp_data.num_corresp);
    read_array(file, corresp_data.kf_index, corresp_data.num_corresp);
    read_array(file, corresp_data.lm_index, corresp_data.num_corresp);
}
