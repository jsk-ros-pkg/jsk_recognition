#ifndef FISHEYE_INTERNAL_H
#define FISHEYE_INTERNAL_H
#include <jsk_perception/precomp.hpp>

namespace cv { namespace internal {

struct CV_EXPORTS IntrinsicParams
{
    Vec2d f;
    Vec2d c;
    Vec4d k;
    double alpha;
    std::vector<int> isEstimate;

    IntrinsicParams();
    IntrinsicParams(Vec2d f, Vec2d c, Vec4d k, double alpha = 0);
    IntrinsicParams operator+(const Mat& a);
    IntrinsicParams& operator =(const Mat& a);
    void Init(const cv::Vec2d& f, const cv::Vec2d& c, const cv::Vec4d& k = Vec4d(0,0,0,0), const double& alpha = 0);
};

void projectPoints(cv::InputArray objectPoints, cv::OutputArray imagePoints,
                   cv::InputArray _rvec,cv::InputArray _tvec,
                   const IntrinsicParams& param, cv::OutputArray jacobian);

void ComputeExtrinsicRefine(const Mat& imagePoints, const Mat& objectPoints, Mat& rvec,
                            Mat&  tvec, Mat& J, const int MaxIter,
                            const IntrinsicParams& param, const double thresh_cond);
CV_EXPORTS Mat ComputeHomography(Mat m, Mat M);

CV_EXPORTS Mat NormalizePixels(const Mat& imagePoints, const IntrinsicParams& param);

void InitExtrinsics(const Mat& _imagePoints, const Mat& _objectPoints, const IntrinsicParams& param, Mat& omckk, Mat& Tckk);

void CalibrateExtrinsics(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints,
                         const IntrinsicParams& param, const int check_cond,
                         const double thresh_cond, InputOutputArray omc, InputOutputArray Tc);

void ComputeJacobians(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints,
                      const IntrinsicParams& param,  InputArray omc, InputArray Tc,
                      const int& check_cond, const double& thresh_cond, Mat& JJ2_inv, Mat& ex3);

CV_EXPORTS void  EstimateUncertainties(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints,
                           const IntrinsicParams& params, InputArray omc, InputArray Tc,
                           IntrinsicParams& errors, Vec2d& std_err, double thresh_cond, int check_cond, double& rms);

void dAB(cv::InputArray A, InputArray B, OutputArray dABdA, OutputArray dABdB);

void JRodriguesMatlab(const Mat& src, Mat& dst);

void compose_motion(InputArray _om1, InputArray _T1, InputArray _om2, InputArray _T2,
                    Mat& om3, Mat& T3, Mat& dom3dom1, Mat& dom3dT1, Mat& dom3dom2,
                    Mat& dom3dT2, Mat& dT3dom1, Mat& dT3dT1, Mat& dT3dom2, Mat& dT3dT2);

double median(const Mat& row);

Vec3d median3d(InputArray m);

  }};

namespace cv
{
  namespace fisheye
  {
    enum{
      CALIB_USE_INTRINSIC_GUESS   = 1,
      CALIB_RECOMPUTE_EXTRINSIC   = 2,
      CALIB_CHECK_COND            = 4,
      CALIB_FIX_SKEW              = 8,
      CALIB_FIX_K1                = 16,
      CALIB_FIX_K2                = 32,
      CALIB_FIX_K3                = 64,
      CALIB_FIX_K4                = 128,
      CALIB_FIX_INTRINSIC         = 256
    };
    
    //! projects 3D points using fisheye model
    CV_EXPORTS void projectPoints(InputArray objectPoints, OutputArray imagePoints, const Affine3d& affine,
				  InputArray K, InputArray D, double alpha = 0, OutputArray jacobian = noArray());
    
    //! projects points using fisheye model
    CV_EXPORTS void projectPoints(InputArray objectPoints, OutputArray imagePoints, InputArray rvec, InputArray tvec,
				  InputArray K, InputArray D, double alpha = 0, OutputArray jacobian = noArray());
    
    //! distorts 2D points using fisheye model
    CV_EXPORTS void distortPoints(InputArray undistorted, OutputArray distorted, InputArray K, InputArray D, double alpha = 0);
    
    //! undistorts 2D points using fisheye model
    CV_EXPORTS void undistortPoints(InputArray distorted, OutputArray undistorted,
        InputArray K, InputArray D, InputArray R = noArray(), InputArray P  = noArray());

    //! computing undistortion and rectification maps for image transform by cv::remap()
    //! If D is empty zero distortion is used, if R or P is empty identity matrixes are used
    CV_EXPORTS void initUndistortRectifyMap(InputArray K, InputArray D, InputArray R, InputArray P,
        const cv::Size& size, int m1type, OutputArray map1, OutputArray map2);

    //! undistorts image, optionally changes resolution and camera matrix. If Knew zero identity matrix is used
    CV_EXPORTS void undistortImage(InputArray distorted, OutputArray undistorted,
        InputArray K, InputArray D, InputArray Knew = cv::noArray(), const Size& new_size = Size());

    //! estimates new camera matrix for undistortion or rectification
    CV_EXPORTS void estimateNewCameraMatrixForUndistortRectify(InputArray K, InputArray D, const Size &image_size, InputArray R,
        OutputArray P, double balance = 0.0, const Size& new_size = Size(), double fov_scale = 1.0);

    //! performs camera calibaration
    CV_EXPORTS double calibrate(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints, const Size& image_size,
        InputOutputArray K, InputOutputArray D, OutputArrayOfArrays rvecs, OutputArrayOfArrays tvecs, int flags = 0,
            TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

    //! stereo rectification estimation
    CV_EXPORTS void stereoRectify(InputArray K1, InputArray D1, InputArray K2, InputArray D2, const Size &imageSize, InputArray R, InputArray tvec,
        OutputArray R1, OutputArray R2, OutputArray P1, OutputArray P2, OutputArray Q, int flags, const Size &newImageSize = Size(),
        double balance = 0.0, double fov_scale = 1.0);

    //! performs stereo calibaration
    CV_EXPORTS double stereoCalibrate(InputArrayOfArrays objectPoints, InputArrayOfArrays imagePoints1, InputArrayOfArrays imagePoints2,
                                  InputOutputArray K1, InputOutputArray D1, InputOutputArray K2, InputOutputArray D2, Size imageSize,
                                  OutputArray R, OutputArray T, int flags = CALIB_FIX_INTRINSIC,
                                  TermCriteria criteria = TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, DBL_EPSILON));

}
}



#endif
