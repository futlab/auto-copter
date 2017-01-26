#include "libuvc/libuvc.h"
#include <stdio.h>
#include <unistd.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "v4l2cap.h"

#include "modules/apriltags/src/TagDetector.h"
#include "modules/apriltags/src/CameraUtil.h"

V4L2Cap cap;

TagDetectorParams params;
cv::Point2d opticalCenter;

int cvPose = 0;

void onImage(const void * data, int size)
{
    TagFamily family("Tag25h7");
    family.setErrorRecoveryFraction(1.0);
    TagDetector detector(family, params);
    TagDetectionArray detections;
    if (size < cap.width() * cap.height()) return;
    cv::Mat frame(cv::Size(cap.width(), cap.height()), CV_8U, (void *)data);
    if (frame.empty()) return;

    detector.process(frame, opticalCenter, detections);

    cv::Mat show = frame;
    if (!detections.empty()) {
        double s = 0.1905;//opts.tag_size;
        double ss = 0.5*s;
        double sz = s;

        enum { npoints = 8, nedges = 12 };

        cv::Point3d src[npoints] = {
            cv::Point3d(-ss, -ss, 0),
            cv::Point3d( ss, -ss, 0),
            cv::Point3d( ss,  ss, 0),
            cv::Point3d(-ss,  ss, 0),
            cv::Point3d(-ss, -ss, sz),
            cv::Point3d( ss, -ss, sz),
            cv::Point3d( ss,  ss, sz),
            cv::Point3d(-ss,  ss, sz),
        };

        int edges[nedges][2] = {
            { 0, 1 },
            { 1, 2 },
            { 2, 3 },
            { 3, 0 },

            { 4, 5 },
            { 5, 6 },
            { 6, 7 },
            { 7, 4 },

            { 0, 4 },
            { 1, 5 },
            { 2, 6 },
            { 3, 7 }
        };

        cv::Point2d dst[npoints];

        double f = 500;//opts.focal_length;

        double K[9] = {
            f, 0, opticalCenter.x,
            0, f, opticalCenter.y,
            0, 0, 1
        };

        cv::Mat_<cv::Point3d> srcmat(npoints, 1, src);
        cv::Mat_<cv::Point2d> dstmat(npoints, 1, dst);

        cv::Mat_<double>      Kmat(3, 3, K);

        cv::Mat_<double>      distCoeffs = cv::Mat_<double>::zeros(4,1);

        for (size_t i=0; i<detections.size(); ++i) {

            if (1) {

                cv::Mat r, t;

                if (cvPose) {
                    CameraUtil::homographyToPoseCV(f, f, s, detections[i].homography, r, t);
                } else {

                    cv::Mat_<double> M =
                            CameraUtil::homographyToPose(f, f, s,
                                                         detections[i].homography,
                                                         false);

                    cv::Mat_<double> R = M.rowRange(0,3).colRange(0, 3);

                    t = M.rowRange(0,3).col(3);

                    cv::Rodrigues(R, r);

                }

                cv::projectPoints(srcmat, r, t, Kmat, distCoeffs, dstmat);

                for (int j=0; j<nedges; ++j) {
                    cv::line(show,
                             dstmat(edges[j][0],0),
                            dstmat(edges[j][1],0),
                            cvPose ? CV_RGB(0,255,0) : CV_RGB(255,0,0),
                            1, CV_AA);
                }
            }
        }
    }

    cv::imshow("win", show);
    int k = cv::waitKey(1);
    if (k % 256 == 'p') {
        cvPose = !cvPose;
    } else if (k % 256 == 27 /* ESC */) {
        return;
    }
}

void initAprilTags()
{

}

int main()
{
    if(!cap.openDevice("/dev/video0")) return -1;
    if(!cap.initDevice(&onImage)) return -1;
    opticalCenter.x = cap.width() * 0.5;
    opticalCenter.y = cap.height() * 0.5;

    if(!cap.start()) return -1;
    cap.loop();
    cap.stop();
    cap.uninitDevice();
    cap.closeDevice();
    //fprintf(stderr, "\n");
}
