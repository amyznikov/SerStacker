/*
 * cdiffs.h
 *
 *  Created on: Sep 22, 2017
 *      Author: amyznikov
 */

#pragma once

#ifndef __cdiffs_h__
#define __cdiffs_h__

#include <opencv2/opencv.hpp>

void cdiffs(cv::InputArray src,
    cv::Mat * _Gx, cv::Mat * _Gy,
    cv::Mat * _Gxx, cv::Mat * _Gyy,
    cv::Mat * _Gxy,
    cv::Mat * _Gxxx, cv::Mat * _Gyyy,
    cv::Mat * _Gxxy, cv::Mat * _Gyyx,
    cv::Mat * _Gxyx, cv::Mat * _Gxyy);

void cdiffs_s(cv::InputArray src, int s,
    cv::Mat * _Gx, cv::Mat * _Gy,
    cv::Mat * _Gxx, cv::Mat * _Gyy,
    cv::Mat * _Gxy,
    cv::Mat * _Gxxx, cv::Mat * _Gyyy,
    cv::Mat * _Gxxy, cv::Mat * _Gyyx,
    cv::Mat * _Gxyx, cv::Mat * _Gxyy);

void cdiffs(cv::InputArray src,
    cv::Mat * _Gx, cv::Mat * _Gy,
    cv::Mat * _Gxx, cv::Mat * _Gyy,
    cv::Mat * _Gxy);

void gradient(cv::InputArray src,
    cv::Mat * _Gx,
    cv::Mat * _Gy,
    cv::Mat * _G = nullptr);

void gradient(cv::InputArray src,
    double sigma,
    cv::Mat * _Gx,
    cv::Mat * _Gy,
    cv::Mat * _G = nullptr);

void gradientvec(cv::InputArray src,
    cv::Mat * _GVec);

void gradientvec(cv::InputArray src,
    double sigma,
    cv::Mat * _GVec);

void gradient_s(cv::InputArray src,
    int s,
    cv::Mat * _Gx,
    cv::Mat * _Gy,
    cv::Mat * _G = nullptr);

void gradient_s(cv::InputArray src,
    int s,
    double sigma,
    cv::Mat * _Gx,
    cv::Mat * _Gy,
    cv::Mat * _G = nullptr);


/* Per-pixel Hessian matrix elements */
void hessian(cv::InputArray src,
    cv::Mat * _Gxx, cv::Mat * _Gyy,
    cv::Mat * _Gxy);

/* Per-pixel Laplacian using hessian() */
void laplace(cv::InputArray src,
    cv::OutputArray L);

void modlaplace(cv::InputArray src,
    cv::OutputArray L);

void modlaplace2(cv::InputArray src,
    cv::OutputArray L,
    int step,
    int box);

void ridgeness(cv::InputArray Gxx,
    cv::InputArray Gyy,
    cv::InputArray Gxy,
    cv::OutputArray R );

void ridgeness(cv::InputArray src,
    cv::OutputArray R );


void eigen2d(cv::InputArray _Dxx,
    cv::InputArray _Dyy,
    cv::InputArray _Dxy,
    cv::Mat * _mu1,
    cv::Mat * _mu2,
    cv::Mat * _Nx = nullptr,
    cv::Mat * _Ny = nullptr,
    cv::InputArray _Dx = cv::noArray(),
    cv::InputArray _Dy = cv::noArray());

void eigen2d(cv::InputArray src,
    cv::Mat * _mu1,
    cv::Mat * _mu2,
    cv::Mat * _Nx = nullptr,
    cv::Mat * _Ny = nullptr,
    bool fixNormals = false);

void eigenridges(cv::InputArray _Dxx,
    cv::InputArray _Dyy,
    cv::InputArray _Dxy,
    cv::OutputArray R,
    cv::Mat * _Nx = nullptr,
    cv::Mat * _Ny = nullptr);

void eigenridges(cv::InputArray src,
    cv::OutputArray R,
    cv::Mat * _Nx = nullptr,
    cv::Mat * _Ny = nullptr);

void eigenridges2(cv::InputArray _src,
    double sigma,
    cv::Mat & R,
    cv::Mat * _Nx = nullptr,
    cv::Mat * _Ny = nullptr);

void cLvv(const cv::Mat & Lx,
    const cv::Mat & Ly,
    const cv::Mat & Lxx,
    const cv::Mat & Lyy,
    const cv::Mat & Lxy,
    cv::Mat & _Lvv);

void cLvvv(const cv::Mat & Lx,
    const cv::Mat & Ly,
    const cv::Mat & Lxxx,
    const cv::Mat & Lyyy,
    const cv::Mat & Lxyy,
    const cv::Mat & Lxxy,
    cv::Mat & _Lvvv);

void find_edges(cv::InputArray _src,
    /* out */ cv::Mat & dst,
    /* out, opt */ cv::Mat * _NMS = nullptr);

void find_edges(cv::InputArray _src,
    double sigma,
    /* out */ cv::Mat & dst,
    /* out, opt */ cv::Mat * _NMS = nullptr);

/* Non-maximum suppression */
void non_maximum_suppression(cv::InputArray _src,
    cv::InputArray _Nx,
    cv::InputArray _Ny,
    cv::Mat & _dst);

/*
 * Use of sepFilter2D with differential kernel {  0, -1, +1 };
 * for gradient computation
 * */
void ggrad(cv::InputArray src, cv::Mat * gx, cv::Mat * gy, cv::Mat * g = nullptr,
    int borderType = cv::BORDER_REPLICATE);


/*
 * Use of sepFilter2D with user specified kernels and anchors
 * for gradient computation
 * */
void ggrad(cv::InputArray src, const cv::Mat1f & Kx, const cv::Mat1f & Ky,
    cv::Point anchor_x, cv::Point anchor_y,
    cv::Mat * gx, cv::Mat * gy, cv::Mat * g = nullptr,
    int borderType = cv::BORDER_REPLICATE);

/*
 * Use of sepFilter2D with differential kernel {  0, -1, +1 };
 * for gradient computation of pre-smoothed image src
 * */
void ggrad(cv::InputArray src, double pre_smooth_sigma, cv::Mat * gx, cv::Mat * gy, cv::Mat * g = nullptr,
    int borderType = cv::BORDER_REPLICATE);

/**
 * Generalized Laplacian filter L is defined as
 *    L = Dx' W Dx   +   Dy' W Dy
 */
void glap(cv::InputArray src, cv::InputArray weight, cv::Mat & dst,
    int borderType = cv::BORDER_REPLICATE);

/*
 * Divergence of gradient of image X
 * */
void divergence(const cv::Mat & src,
    cv::Mat & D);

#endif /* __cdiffs_h__ */
