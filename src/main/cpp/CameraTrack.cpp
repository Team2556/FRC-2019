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

        // Reject a rectangle that is too low or too high
        // if ((BRect.center.y < (FrameThreshold.rows * 0.25)) ||
        //     (BRect.center.y > (FrameThreshold.rows * 0.75)))
        //     continue;

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
    float fTrackError = 1000000.0;
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
                    iMatchesFound++;

                    // Find the center of the target in screen coordinates
                    // Left = 0  Top = 0
                    iCenterX = (BoundingRects[iIdx1].center.x + BoundingRects[iIdx2].center.x) / 2;
                    iCenterY = (BoundingRects[iIdx1].center.y + BoundingRects[iIdx2].center.y) / 2;

                    // Now scale the position to +1.0 to -1.0
                    // Left = -1.0  Right  = +1.0
                    // Top  = +1.0  Bottom = -1.0
                    float fTrackErrorX =  (2.0 * (float)iCenterX / FrameThreshold.cols) - 1.0;
                    float fTrackErrorY = -(2.0 * (float)iCenterY / FrameThreshold.rows) + 1.0;
                    float fTrackErrorTemp = sqrt(fTrackErrorX*fTrackErrorX + fTrackErrorY*fTrackErrorY);

                    // Keep the track error for the target that is closest to the track point.
                    // The default track point is the center of the video frame but can be set
                    // to some other position on the screen with SetTrackPoint(). 
                    if (fTrackErrorTemp < fTrackError)
                        {
                        fTrackError    = fTrackErrorTemp;
                        *pfTrackErrorX = fTrackErrorX;
                        *pfTrackErrorY = fTrackErrorY;

                        // Calcualte the target size
                        *pfTargetSizeX = fabs((BoundingRects[iIdx1].center.x - 
                                               BoundingRects[iIdx2].center.x   ) / 
                                                (float)FrameThreshold.cols);
                        *pfTargetSizeY = (BoundingRects[iIdx1].size.height + 
                                          BoundingRects[iIdx2].size.height   ) /
                                            ((float)FrameThreshold.rows * 2.0);
                        bTargetTracked = true;
                        }

//                    printf("TRACK %5.2f %5.2f %5.2f %5.2f\n", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);
//                    printf("TRUE ");
                    }
                else
                    {
//                    printf("FALSE");
                    }
                }
//        printf("\n");
        } // end if at least two targets

    // If no target found then zero out the track errors just to be safe.
//    if (iMatchesFound > 1) printf ("Matches %1 ", iMatchesFound);
    if (iMatchesFound >= 1)
        {
#if 0
        frc::SmartDashboard::PutNumber("Average width", (BoundingRects[iRect1].size.width + BoundingRects[iRect2].size.width)/2 );
        frc::SmartDashboard::PutNumber("Left Width",     BoundingRects[iRect1].size.width);
        frc::SmartDashboard::PutNumber("Right Width",    BoundingRects[iRect2].size.width);
        frc::SmartDashboard::PutNumber("Error Angle", acos((BoundingRects[iRect2].size.width)/(BoundingRects[iRect1].size.width))*100);
#endif
        }
    else
        {
        // No track so zero out the error just to be safe
        *pfTrackErrorX = 0.0;
        *pfTrackErrorY = 0.0;
        bTargetTracked = false;
//        printf("NO TRACK\n", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);
        }

//    printf("%s %5.2f %5.2f %5.2f %5.2f ", bTargetTracked ? "TRACK   " : "NO TRACK", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);

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
            if (!FrameCam.empty()) 
                {
                // Overlay status on camera image
                FrameCamStatus = FrameDraw;
                DrawTrackStatus(FrameCamStatus, bTargetTracked);
                cvVidOut.PutFrame(FrameCamStatus);
                }
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

//   printf("%s %5.2f %5.2f %5.2f %5.2f ", bTargetTracked ? "TRACK   " : "NO TRACK", *pfTrackErrorX, *pfTrackErrorY, *pfTargetSizeX, *pfTargetSizeY);

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

// qsort() logic to sort rectangle points left to right
int SortY (const void * Pt1, const void * Pt2)
    {
    if (((cv::Point2f *)Pt1)->y <  ((cv::Point2f *)Pt2)->y) return -1;
    if (((cv::Point2f *)Pt1)->y == ((cv::Point2f *)Pt2)->y) return  0;
    if (((cv::Point2f *)Pt1)->y >  ((cv::Point2f *)Pt2)->y) return  1;
    }

// ----------------------------------------------------------------------------

// qsort() logic to sort rectangle points top to bottom
int SortX (const void * Pt1, const void * Pt2)
    {
    if (((cv::Point2f *)Pt1)->x <  ((cv::Point2f *)Pt2)->x) return -1;
    if (((cv::Point2f *)Pt1)->x == ((cv::Point2f *)Pt2)->x) return  0;
    if (((cv::Point2f *)Pt1)->x >  ((cv::Point2f *)Pt2)->x) return  1;
    }

// ----------------------------------------------------------------------------

// Sort the points into the proper order as specified on the OpenCV page.
// The order is bottomLeft, topLeft, topRight, bottomRight. 

void SortRectPoints(cv::Point2f  * aRectPts)
    {
    // Sort the points left to right
    qsort(aRectPts, 4, sizeof(cv::Point2f), SortX);

    // Figure out left top and bottom

    }
    
// ----------------------------------------------------------------------------

// Calculate the angle from vertical of the rectangle. This is necessary because
// the orientation angle that is calculated for the RotatedRect is all over
// the place.

float fRectAngle(cv::RotatedRect * pRectangle)
    {
    cv::Point2f     aRectPts[4];
    float           fAngle;
    float           fLength;
    int             iPts1, iPts2; // Point indexes for longest side

    // Get the rectangle points
    pRectangle->points(aRectPts);

    // Find the longest side
    fLength = 0.0;
//    printf("Len");
    for (int iIdx1 = 0; iIdx1 < 4; iIdx1++)
        {
        int iIdx2 = iIdx1 < 3 ? iIdx1 + 1 : 0;
        float fXDiff = aRectPts[iIdx1].x - aRectPts[iIdx2].x;
        float fYDiff = aRectPts[iIdx1].y - aRectPts[iIdx2].y;
        float fNewLength = sqrt((fXDiff * fXDiff) + (fYDiff * fYDiff));
//        printf(" %4.1f", fNewLength);
        if (fNewLength > fLength)
            {
            fLength = fNewLength;
            iPts1 = iIdx1; 
            iPts2 = iIdx2;
            }
        }
//        printf(" Max %4.1f - %d %d\n", fLength, iPts1, iPts2);

    float fXDiff =   aRectPts[iPts1].x-aRectPts[iPts2].x;
    float fYDiff = -(aRectPts[iPts1].y-aRectPts[iPts2].y);
    // Calculate the angle between the points of the longest side. 0 degrees is up.
//  fAngle = 90.0 - (atan2(aRectPts[iPts1].x-aRectPts[iPts2].x, aRectPts[iPts1].y-aRectPts[iPts2].y) * 180.0 / 3.1416);
    fAngle = atan2(fYDiff, fXDiff) * 180.0 / 3.1416;
//    printf("dX %3.0f dY %3.0d atan %5.1f ", fXDiff, fYDiff, fAngle);
    fAngle = 90.0 - fAngle;
//    printf("%5.1f ", fAngle);

    // Make it between -90 and +90
    if (fAngle > 180.0) fAngle -= 180.0;
    if (fAngle >  90.0) fAngle -= 180.0;

    return fAngle;
    }


// ----------------------------------------------------------------------------

// Compare two rectangles. Return true if they seem to be a good match pair.

bool bMatchPair(cv::RotatedRect Rectangle1, cv::RotatedRect Rectangle2)
    {
    float fHtoS;
    cv::Point2f         aPts1[4];
    cv::Point2f         aPts2[4];
    cv::RotatedRect   * pLeftRectangle;
    cv::RotatedRect   * pRightRectangle;

    // Figure out left and right side rectangles
    if (Rectangle1.center.x < Rectangle2.center.x)
        {
        pLeftRectangle  = &Rectangle1;
        pRightRectangle = &Rectangle2;
        }
    else
        {
        pLeftRectangle  = &Rectangle2;
        pRightRectangle = &Rectangle1;
        }
    // printf("Left X %4.0f Y %4.0f Right X %4.0f Y %4.0f ", 
    //     pLeftRectangle->center.x,  pLeftRectangle->center.y,
    //     pRightRectangle->center.x, pRightRectangle->center.y);

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
        fHtoS = fHeight / fDistance;
//            printf(" HtoS %5.2f ", fHtoS);
            if ((fHtoS < 0.2) || (fHtoS > 0.6))
        return false;
        }

    // Check relative angles
    float fLeftAngle  = fRectAngle(pLeftRectangle);
    float fRightAngle = fRectAngle(pRightRectangle);

    if ((fLeftAngle  <   5.0) || (fLeftAngle  >  25.0) ||
        (fRightAngle >  -5.0) || (fRightAngle < -25.0))
        return false;
//    printf(" Left Ang %6.1f Right Angle %6.1f", fLeftAngle, fRightAngle);

    // printf("Target Center  X,Y=%4.0f,%4.0f ",
    //         (pLeftRectangle->center.x + pRightRectangle->center.x) / 2.0, 
    //         (pLeftRectangle->center.y + pRightRectangle->center.y) / 2.0); 
//    printf("  X,Y=%d,%d XxY=%dx%d   X,Y=%d,%d XxY=%dx%d",
//            Rectangle1.x, Rectangle1.y, Rectangle1.width, Rectangle1.height,
//            Rectangle2.x, Rectangle2.y, Rectangle2.width, Rectangle2.height);
//    printf(" Angles %6.1f %6.1f %6.1f", fRectAngle(&Rectangle1), fRectAngle(&Rectangle2), fRelAngle);
//   printf(" Center Diff %f ", fDistance);
//   printf(" Area %f %f %5.2f", fTargetArea1, fTargetArea2, fAreaRatio);
//    printf("\n");

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