#include "utils.h"

#include <chrono>
#include <iostream>
#include <random>

#include "../src/c_api.h"
#include "ceres/rotation.h"

using std::cout;
using std::endl;
// returns a random number between (0,1)
double rand_double() {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator.seed(seed);
    double number = distribution(generator);
    return number;
}

unsigned rand_int(unsigned int max) {
    return static_cast<unsigned int>(rand()) % max;
}

// creates a random kf pose with x,y,z in (0,1)
void create_random_kf_aa(double t_g_i[AA_POSE_DIM]) {
    double axis_angle[POINT_DIM] = {rand_double(), rand_double(), rand_double()};
    double angle = rand_double() * M_PI;

    // norm of axis_angle should be angle and setting rotation
    double scale = angle * sqrt(1.0 / ceres::DotProduct(axis_angle, axis_angle));
    for (uint i = 0; i < AA_ROT_DIM; i++) {
        t_g_i[i] = axis_angle[i] * scale;
    }
    // setting translation
    for (uint i = AA_ROT_DIM; i < AA_POSE_DIM; i++) {
        t_g_i[i] = (rand_double() - 0.5) * 10.0;  // 5 mts
    }
}

// creates a random lm pose with x,y,z in (0,1)
void create_random_lm(double lm[]) {
    for (uint i = 0; i < POINT_DIM; i++) {
        lm[i] = (rand_double() - 0.5) * 10.0;  // 5 mt
    }
}

// adds gaussian noise to `arr` of size `len`
void add_noise(double std, double arr[], uint len) {
    std::default_random_engine generator;
    auto seed = std::chrono::system_clock::now().time_since_epoch().count();
    //    cout<<"BA: seed: "<<seed<<endl;
    generator.seed(seed);
    std::normal_distribution<double> dist(0.0, std);
    for (uint i = 0; i < len; i++) {
        auto noise = dist(generator);
        //        cout <<"BA: noise " << i << ": " << noise << endl;
        arr[i] += noise;
    }
}

void add_noise_bearing(double std, double bearing[]) {
    add_noise(std, bearing, BEARING_DIM);
}

void add_noise_lm(double std, double lm[]) {
    add_noise(std, lm, POINT_DIM);
}

void add_noise_kf_aa(double std, double kf[]) {
    add_noise(std, kf, AA_POSE_DIM);
}

// deep copy from `data` to `copy`
void deep_copy(double copy[], double data[], int len) {
    for (int i = 0; i < len; i++) {
        copy[i] = data[i];
    }
}

void copy_kf(double copy[], double data[]) {
    deep_copy(copy, data, AA_POSE_DIM);
}

// transformed_lm = t_g_i*lm
void transform_lm_aa(double lm[POINT_DIM], double t_g_i[AA_POSE_DIM], double transformed_lm[POINT_DIM]) {
    // rotate
    ceres::AngleAxisRotatePoint(t_g_i, lm, transformed_lm);
    // translate
    for (uint i = 0; i < POINT_DIM; i++) {
        transformed_lm[i] += t_g_i[i + AA_ROT_DIM];
    }
}

void project_lm_to_bearing(double lm[POINT_DIM], double bearing[]) {
    for (uint i = 0; i < BEARING_DIM; i++) {
        bearing[i] = lm[i] / lm[2];
    }
}

void print_pose_aa(double t_g_i[AA_POSE_DIM]) {
    cout << "Axis_angle: (" << t_g_i[0] << ", " << t_g_i[1] << ", " << t_g_i[2] << ")" << endl;
    cout << "translation: (" << t_g_i[3] << ", " << t_g_i[4] << ", " << t_g_i[5] << ")" << endl;
}

void print_point(double point[POINT_DIM]) {
    cout << "BA: Point: (" << point[0] << ", " << point[1] << ", " << point[2] << ")" << endl;
}

void print_bearing(double bearing[BEARING_DIM]) {
    cout << "BA: Bearing: (" << bearing[0] << ", " << bearing[1] << ")" << endl;
}

double get_position_err_sq(double *t1, double *t2) {
    return get_rot_err_sq_aa(t1, t2);
}

// returns rms between the rotation angles of t1 and t2
double get_rot_err_sq_aa(double *t1, double *t2) {
    double diff = pow((t1[0] - t2[0]), 2) + pow((t1[1] - t2[1]), 2) + pow((t1[2] - t2[2]), 2);
    return diff / (AA_POSE_DIM - POINT_DIM);
}

// returns rms between the translation of t1 and t2
double get_trans_err_sq_aa(double *t1, double *t2) {
    double diff = pow((t1[3] - t2[3]), 2) + pow((t1[4] - t2[4]), 2) + pow((t1[5] - t2[5]), 2);
    return diff / POINT_DIM;
}

double get_bearing_diff(double b1[], double b2[]) {
    double diff = pow((b1[0] - b2[0]), 2) + pow((b1[1] - b2[1]), 2);
    return diff / BEARING_DIM;
}

double average(double array[], unsigned int size) {
    double sum = 0.0;
    for (uint i = 0; i < size; ++i) {
        sum += array[i];
    }
    return sum / size;
}
