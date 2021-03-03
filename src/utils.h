#pragma once

#include "c_api.h"

/*
_aa is used to deal with pose in Axis angle representation
_quat is used to deal with pose in Quaternion angle representation
*/
double rand_double();
unsigned rand_int(unsigned int max);
void create_random_kf_aa(double t_g_i[AA_POSE_DIM]);
void create_random_kf_quat(double t_g_i[QUAT_POSE_DIM]);
void create_random_lm(double lm[POINT_DIM]);

void copy_kf(double copy[], double data[]);

void add_noise_kf_aa(double noise, double kf[]);
void add_noise_kf_quat(double noise, double kf[]);
void add_noise_lm(double std, double lm[]);
void add_noise_bearing(double std, double bearing[]);

void transform_lm_aa(double lm[POINT_DIM], double t_g_i[AA_POSE_DIM], double transformed_lm[POINT_DIM]);
void transform_lm_quat(double lm[POINT_DIM], double t_g_i[QUAT_POSE_DIM], double transformed_lm[POINT_DIM]);
void project_lm_to_bearing(double lm[POINT_DIM], double bearing[BEARING_DIM]);

void deep_copy(double copy[], double data[], int len);

void print_pose_aa(double t_g_i[AA_POSE_DIM]);
void print_pose_quat(double t_g_i[QUAT_POSE_DIM]);
void print_point(double point[POINT_DIM]);
void print_bearing(double bearing[BEARING_DIM]);

double get_position_err_sq(double *t1, double *t2);
double get_rot_err_sq_aa(double *t1, double *t2);
double get_trans_err_sq_aa(double *t1, double *t2);
double get_rot_err_sq_quat(double *t1, double *t2);
double get_trans_err_sq_quat(double *t1, double *t2);
double get_bearing_diff(double b1[], double b2[]);
double average(double array[], unsigned int size);
double get_proj_error(double lm[], double t_g_ifl[], double t_ifl_ifx[], double est_bearing[]);
