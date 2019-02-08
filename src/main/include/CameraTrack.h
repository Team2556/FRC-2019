/*
 * CameraTrack.h
 *
 *  Created on: Feb 13, 2018
 *      Author: team2556
 */

#ifndef SRC_TARGETTRACK_H_
#define SRC_TARGETTRACK_H_

#include <thread>
#include <mutex>


class CameraTrack
    {
    // Constructor / Destructor
public:
    CameraTrack();

    // Data
    frc::Preferences *  pPrefs;
//    cs::UsbCamera     * pCamera;
    cs::CvSink          cvSink;
    cs::CvSource        cvVidOut;

    int                 HueLo, HueHi;
    int		            SatLo, SatHi;
    int		            ValLo, ValHi;

    int                 iDisplayFrame;

    float               fTrackPointX;
    float               fTrackPointY;

    std::mutex          TrackUpdateMutex;

protected:
    float               fTrackErrorX, fTrackErrorY;
    float               fTargetSizeX, fTargetSizeY;
    bool                bTargetTracked;

    // Methods
public:
    void    Init();
    void    UpdateParams();
    void    SetTrackPoint(float fTrackPointX, float fTrackPointY);
    bool    CalcTrackError(float * pfTrackErrorX, float * pfTrackErrorY, float * pfTargetSizeX, float * pfTargetSizeY);
    void    GetTrackError(bool * pbTracked, float * pfTrackErrorX, float * pfTrackErrorY, float * pfTargetSizeX, float * pfTargetSizeY);
    void    TrackThread();

};

#endif /* SRC_TARGETTRACK_H_ */
