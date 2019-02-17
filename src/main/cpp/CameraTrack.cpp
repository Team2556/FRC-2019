/*
 * CameraTrack.cpp
 *
 *  Created on: Feb 13, 2018
 *      Author: team2556
 */

// https://docs.opencv.org/3.4.5/index.html

#include <thread>
#include <mutex>

#include "frc/WPILib.h"
#include "frc/Preferences.h"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include "robotmap.h"
#include "CameraTrack.h"

bool bMatchPair(cv::RotatedRect Rectangle1, cv::RotatedRect Rectangle2);
void DrawTrackStatus(cv::Mat CameraImage, bool bTracked);

// ============================================================================
// CameraTrack
// ============================================================================

// ----------------------------------------------------------------------------
// Constructor / Destructor
// ----------------------------------------------------------------------------

CameraTrack::CameraTrack()
    {
//    pCamera = pTrackCamera;
    fTrackPointX = 0.0;
    fTrackPointY = 0.0;
    }

// ----------------------------------------------------------------------------
// Methods
// ----------------------------------------------------------------------------

void CameraTrack::Init()
    {
    pPrefs = frc::Preferences::GetInstance();
//  UsbCamera1 = frc::CameraServer::GetInstance()->StartAutomaticCapture();

    HueLo = pPrefs->GetInt("Target Hue Lo", 0);
    HueHi = pPrefs->GetInt("Target Hue Hi", 255);
    SatLo = pPrefs->GetInt("Target Sat Lo", 0);
    SatHi = pPrefs->GetInt("Target Sat Hi", 255);
    ValLo = pPrefs->GetInt("Target Val Lo", 250);
    ValHi = pPrefs->GetInt("Target Val Hi", 255);

    pPrefs->PutInt("Target Hue Lo", HueLo);
    pPrefs->PutInt("Target Hue Hi", HueHi);
    pPrefs->PutInt("Target Sat Lo", SatLo);
    pPrefs->PutInt("Target Sat Hi", SatHi);
    pPrefs->PutInt("Target Val Lo", ValLo);
    pPrefs->PutInt("Target Val Hi", ValHi);

    iDisplayFrame = pPrefs->GetInt("Display Frame", 0);
    pPrefs->PutInt("Display Frame", iDisplayFrame);

    // Start the camera stuff
//	UsbCamera.SetResolution(320, 240);
//	UsbCamera.SetFPS(10);

//	UsbCamera1.SetResolution(320, 240);
//	UsbCamera1.SetFPS(10);

	cvSink   = frc::CameraServer::GetInstance()->GetVideo();
	cvVidOut = frc::CameraServer::GetInstance()->PutVideo("Front Proc", 320, 240);

    }

// ----------------------------------------------------------------------------

void CameraTrack::SetTrackPoint(float fSetTrackPointX, float fSetTrackPointY)
    {
    if      (fSetTrackPointX < -1.0) fTrackPointX = -1.0;
    else if (fSetTrackPointX >  1.0) fTrackPointX =  1.0;
    else                             fTrackPointX =  fSetTrackPointX;

    if      (fSetTrackPointY < -1.0) fTrackPointY = -1.0;
    else if (fSetTrackPointY >  1.0) fTrackPointY =  1.0;
    else                             fTrackPointY =  fSetTrackPointY;
    }


// ----------------------------------------------------------------------------

bool CameraTrack::CalcTrackError(float * pfTrackErrorX, float * pfTrackErrorY, float * pfTargetSizeX, float * pfTargetSizeY)

    {
    int     iCenterX, iCenterY;
    int     iMatchesFound;
    int     bTargetTracked;

    // HueLo = pPrefs->GetInt("Target Hue Lo", 0);
    // HueHi = pPrefs->GetInt("Target Hue Hi", 255);
    // SatLo = pPrefs->GetInt("Target Sat Lo", 0);
    // SatHi = pPrefs->GetInt("Target Sat Hi", 255);
    // ValLo = pPrefs->GetInt("Target Val Lo", 250);
    // ValHi = pPrefs->GetInt("Target Val Hi", 255);

    // iDisplayFrame = pPrefs->GetInt("Display Frame", 0);

    cv::Mat                                 FrameCam;
    cv::Mat                                 FrameHsv;
    cv::Mat                                 FrameThreshold;
//  cv::Mat                                 FrameEdges;
    cv::Mat                                 FrameDraw;
    cv::Mat                                 FrameCamStatus;

    std::vector< std::vector<cv::Point> >   Contours;
    std::vector< std::vector<cv::Point> >   FiltContours;
    std::vector< cv::RotatedRect >          BoundingRects;
    cv::RNG                                 rng(12345);

    // Get a frame of data
#if 1
    while (cvSink.GrabFrame(FrameCam) == 0) {};
#else
    if (cvSink.GrabFrame(FrameCam, 0.01) == 0)
        return false;
#endif

    // Change image to HSV format
    cv::cvtColor(FrameCam, FrameHsv,  cv::COLOR_BGR2HSV);

    // Change to B/W based on HSV threshold ranges
    cv::inRange(FrameHsv, cv::Scalar(HueLo, SatLo, ValLo), cv::Scalar(HueHi, SatHi, ValHi), FrameThreshold);

    // Find contours
    cv::findContours(FrameThreshold, Contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    // Filter contours
    for (unsigned int i = 0; i < Contours.size(); i++)
    {
        // Filter based on size
        double dArea = cv::contourArea(Contours[i]);
//      printf(" Area %f ", dArea);
        if (dArea < 50.0)
            continue;

        // Find the bounding rectangle
        cv::RotatedRect BRect = cv::minAreaRect(Contours[i]);
        float fBRAspect = (float)BRect.size.height / (float)BRect.size.width;
//        printf("I%2.2d h %f w %f Asp %4.1f Ang %5.1f ", i, BRect.size.height, BRect.size.width, fBRAspect, BRect.angle);

        if (((fBRAspect > 2.0) && (fBRAspect < 5.0)) ||
            ((fBRAspect < 0.5) && (fBRAspect > 0.2)))
        {
            FiltContours.push_back(Contours[i]);
            BoundingRects.push_back(BRect);
        }
    } // end for all contours

    // Find two bounding rectangles that seem to be a good match
//    printf("%d - ", FiltContours.size());
    int    iRect1, iRect2;
    iMatchesFound = 0;

    if (FiltContours.size() >= 2)
        {
        unsigned int     iIdx1, iIdx2;
        for (iIdx1 = 0; iIdx1 < FiltContours.size() - 1; iIdx1++)
            for (iIdx2 = iIdx1+1; iIdx2 < FiltContours.size(); iIdx2++)
                {
//                printf("  %d-%d ", iIdx1, iIdx2);
                if (bMatchPair(BoundingRects[iIdx1], BoundingRects[iIdx2]))
                    {
                    iRect1 = iIdx1;
                    iRect2 = iIdx2;
                    iMatchesFound++;
                    // printf("(%3.0f %3.0f %5.1f, %3.0f %3.0f %5.1f) ", 
                    //     BoundingRects[iIdx1].center.x, BoundingRects[iIdx1].center.y, BoundingRects[iIdx1].angle, 
                    //     BoundingRects[iIdx2].center.x, BoundingRects[iIdx2].center.y, BoundingRects[iIdx2].angle);
//                    printf("TRUE ");
                    }
                else
                    {
//                    printf("FALSE");
                    }
                }
        } // end if at least two targets

    // Only calculate a target if one and only one rectangle pair was found. In the future it would be
    // nice to save all the target pairs and attempt to pick the best one.
//    if (iMatchesFound > 1) printf ("Matches %1 ", iMatchesFound);
    if (iMatchesFound == 1)
        {
        // Find the center of the target in screen coordinates
        // Left = 0  Top = 0
        iCenterX = (BoundingRects[iRect1].center.x + BoundingRects[iRect2].center.x) / 2;
        iCenterY = (BoundingRects[iRect1].center.y + BoundingRects[iRect2].center.y) / 2;

        // Now scale the position to +1.0 to -1.0
        // Left = -1.0  Right  = +1.0
        // Top  = +1.0  Bottom = -1.0
        *pfTrackErrorX =  (2.0 * (float)iCenterX / FrameThreshold.cols) - 1.0;
        *pfTrackErrorY = -(2.0 * (float)iCenterY / FrameThreshold.rows) + 1.0;

        // Calcualte the target size
        *pfTargetSizeX = fabs((BoundingRects[iRect1].center.x - 
                               BoundingRects[iRect2].center.x   ) / 
                                   (float)FrameThreshold.cols);
        *pfTargetSizeY = (BoundingRects[iRect1].size.height + 
                          BoundingRects[iRect2].size.height   ) /
                             ((float)FrameThreshold.rows * 2.0);
        bTargetTracked = true;
//        printf("TRACK %5.2f %5.2f %5.2f %5.2f\n", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);
        }
    else
        {
        // No track so zero out the error just to be safe
        *pfTrackErrorX = 0.0;
        *pfTrackErrorY = 0.0;
        bTargetTracked = false;
//        printf("NO TRACK\n", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);
        }

//    printf("%s %5.2f %5.2f %5.2f %5.2f ", bMatchFound ? "TRACK   " : "NO TRACK", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);

    // Draw contours
    FrameDraw = cv::Mat::zeros(FrameCam.size(), CV_8UC3);
    for (unsigned int i = 0; i < FiltContours.size(); i++)
    {
        cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
        cv::drawContours(FrameDraw, FiltContours, i, color, 1, 8, cv::noArray(), 0, cv::Point());
    }

//printf("\n");

    // Last but not least display a processed camera frame
    switch (iDisplayFrame)
        {
        default :
        case 0 :
            if (!FrameCam.empty()) cvVidOut.PutFrame(FrameCam);
            break;
        case 1 :
            if (!FrameHsv.empty()) cvVidOut.PutFrame(FrameHsv);
            break;
        case 2 :
            if (!FrameThreshold.empty()) cvVidOut.PutFrame(FrameThreshold);
            break;
        case 3 :
            if (!FrameDraw.empty()) cvVidOut.PutFrame(FrameDraw);
            break;
        case 4 :
            if (!FrameCam.empty()) 
                {
                // Overlay status on camera image
                FrameCamStatus = FrameCam;
                DrawTrackStatus(FrameCamStatus, bTargetTracked);
                cvVidOut.PutFrame(FrameCamStatus);
                }
            break;
        } // end switch on frame to display

    return bTargetTracked;
    } // end Periodic()


// ----------------------------------------------------------------------------

// Return the track error values. This is necessary when CalcTrackError() is
// run in a thread.
void CameraTrack::GetTrackError(bool * pbTracked, float * pfTrackErrorX, float * pfTrackErrorY, float * pfTargetSizeX, float * pfTargetSizeY)
    {
    TrackUpdateMutex.lock();
    *pbTracked       = bTargetTracked;
    *pfTrackErrorX   = fTrackErrorX;
    *pfTrackErrorY   = fTrackErrorY;
    *pfTargetSizeX   = fTargetSizeX;
    *pfTargetSizeY   = fTargetSizeY;
    TrackUpdateMutex.unlock();
    }

// ----------------------------------------------------------------------------

// This routine implements a thread routine that runs continuously in the 
// background to grab frames and update track errors.

void CameraTrack::TrackThread()
    {
    float       fLocalTrackErrorX, fLocalTrackErrorY;
    float       fLocalTargetSizeX, fLocalTargetSizeY;
    bool        bLocalTargetTracked;

    while (true)
        {
        bLocalTargetTracked = CameraTrack::CalcTrackError(
                &fLocalTrackErrorX, &fLocalTrackErrorY, 
                &fLocalTargetSizeX, &fLocalTargetSizeY);

        TrackUpdateMutex.lock();
        bTargetTracked = bLocalTargetTracked;
        fTrackErrorX   = fLocalTrackErrorX;
        fTrackErrorY   = fLocalTrackErrorY;
        fTargetSizeX   = fLocalTargetSizeX;
        fTargetSizeY   = fLocalTargetSizeY;
        TrackUpdateMutex.unlock();
        }

    } // end TrackThread()
    

// ----------------------------------------------------------------------------
// Utilities
// ----------------------------------------------------------------------------

// Compare two rectangles. Return true if they seem to be a good match pair.

bool bMatchPair(cv::RotatedRect Rectangle1, cv::RotatedRect Rectangle2)
    {
    // Check verticle alignment
//    printf(" Center Diff %d ", fYDiff);
    float   fYDiff = fabs(Rectangle1.center.y - Rectangle2.center.y);
    if (fYDiff > 10.0)
         return false;

    // Check relative areas
    float fTargetArea1 = Rectangle1.size.width * Rectangle1.size.height;
    float fTargetArea2 = Rectangle2.size.width * Rectangle2.size.height;
    float fAreaRatio   = fTargetArea1 / fTargetArea2;
//  printf(" Area %f %f %5.2f ", fTargetArea1, fTargetArea2, fAreaRatio);
    if ((fAreaRatio < 0.75) || (fAreaRatio > 1.5))
        return false;

    // Check box height to seperation distance ratio
    float fHeight   = fmax(fabs(Rectangle1.size.width), fabs(Rectangle1.size.height));
    float fDistance = fabs(Rectangle1.center.x - Rectangle2.center.x);
    if (fDistance > 0.0)
        {
        float fHtoS = fHeight / fDistance;
//        printf(" HtoS %5.2f ", fHtoS);
        if ((fHtoS < 0.3) || (fHtoS > 0.7))
        return false;
        }

    // Check relative angles. This may not be necessary. This seems to pick out
    // the targets without it.
    float fRelAngle = fabs(Rectangle1.angle - Rectangle2.angle);
    if ((fRelAngle > 70.0) || (fRelAngle < 50.0))
        return false;

//    printf("  X,Y=%d,%d XxY=%dx%d   X,Y=%d,%d XxY=%dx%d",
//            Rectangle1.x, Rectangle1.y, Rectangle1.width, Rectangle1.height,
//            Rectangle2.x, Rectangle2.y, Rectangle2.width, Rectangle2.height);
//    printf(" Center Diff %d ", (y1-y2));
//    printf(" Area %f %f %5.2f ", fTargetArea1, fTargetArea2, fAreaRatio);

    return true;
    }


// ----------------------------------------------------------------------------

// Draw a red or green circle on the camera image to show vision track status
void DrawTrackStatus(cv::Mat CameraImage, bool bTracked)
    {
    if (bTracked)
        circle( CameraImage, cv::Point(20,20), 15, cv::Scalar( 0, 255, 0), cv::FILLED, cv::LINE_8 );
    else
        circle( CameraImage, cv::Point(20,20), 15, cv::Scalar( 0, 0, 255), cv::FILLED, cv::LINE_8 );
    }