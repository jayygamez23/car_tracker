#include <iostream>
#include <cstdio>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>

bool trackCars(cv::Mat frame, cv::Mat mask, cv::Mat roi, cv::Scalar color)
{

    // get the contours from the PMOG image
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    /* 
        I tried to create a list of trackers to use for each ROI because there could be 
        multiple cars for each lane. However, I could not use it since I couldn't figure out how to deal with the segfaults. 
        The commented code remains here to be worked on later. 
    */
    /*std::vector<cv::Ptr<cv::Tracker> > trackers(10); 

    for(int i = 0; i < 10; i++)
    {
        cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create(); 
        trackers[i] = tracker;
    }

    //cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();*/

    bool tracking = false; 
    
    // if the contour is large enough, draw a bounding box 
    for(int i = 0; i < contours.size(); i++)
    {
        double area = contourArea(contours.at(i)); 

        if (area >= 11000)
        {
            cv::Rect boundRect = boundingRect(contours.at(i)); 
            //trackers[i]->init(frame, boundRect); 
            //trackers[i]->update(frame, boundRect); 
            cv::rectangle(roi, boundRect, color, 2, 1); 
            tracking = true; 
        }
    }

    return tracking; 
}

int main(int argc, char **argv)
{
    std::string filename = argv[1];
        
    // open the video file
    cv::VideoCapture capture(filename);
    if(!capture.isOpened())
    {
        std::printf("Unable to open video source, terminating program! \n");
        return 0;
    }

    // get the video source parameters
    int captureWidth = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    int captureHeight = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    int captureFPS = static_cast<int>(capture.get(cv::CAP_PROP_FPS));
    std::cout << "Video source opened successfully (width=" << captureWidth << " height=" << captureHeight << " fps=" << captureFPS << ")!" << std::endl;


    // create masks to be used for PMOG 
    cv::Mat mask1; 
    cv::Mat mask2; 
    cv::Mat mask3; 
    cv::Mat mask4;

    // set background filtering parameters and make a pointer for each lane
    const int bgHistory = 300;
    const float bgThreshold = 25;
    const bool bgShadowDetection = false;
    cv::Ptr<cv::BackgroundSubtractor> pMOG2_roi1; 
    cv::Ptr<cv::BackgroundSubtractor> pMOG2_roi2; 
    cv::Ptr<cv::BackgroundSubtractor> pMOG2_roi3; 
    cv::Ptr<cv::BackgroundSubtractor> pMOG2_roi4; 
    pMOG2_roi1 = cv::createBackgroundSubtractorMOG2(bgHistory, bgThreshold, bgShadowDetection);
    pMOG2_roi2 = cv::createBackgroundSubtractorMOG2(bgHistory, bgThreshold, bgShadowDetection);
    pMOG2_roi3 = cv::createBackgroundSubtractorMOG2(bgHistory, bgThreshold, bgShadowDetection);
    pMOG2_roi4 = cv::createBackgroundSubtractorMOG2(bgHistory, bgThreshold, bgShadowDetection);

    // process data until program termination
    bool doCapture = true;
    int frameCount1 = 0;
    int frameCount2 = 0; 
    int frameCount3 = 0; 
    int frameCount4 = 0; 
    bool isTracking1 = false;
    bool isTracking2 = false; 
    bool isTracking3 = false; 
    bool isTracking4 = false; 
    int eastbound_cnt = 0; 
    int westbound_cnt = 0; 

    while(doCapture)
    {
        // attempt to acquire and process an image frame
        // prepare to save different versions of the image frame
        cv::Mat captureFrame;
        cv::Mat grayFrame;
        cv::Mat equalized; 
        cv::Mat roi1;
        cv::Mat roi2; 
        cv::Mat roi3; 
        cv::Mat roi4;  
        cv::Mat imageEdges;
        cv::Mat frameEroded;

        bool captureSuccess = capture.read(captureFrame);
        if(captureSuccess)
        {
			// pre-process the raw image frame
            const int rangeMin = 0;
            const int rangeMax = 255;
            cv::cvtColor(captureFrame, grayFrame, cv::COLOR_BGR2GRAY);
            cv::normalize(grayFrame, grayFrame, rangeMin, rangeMax, cv::NORM_MINMAX, CV_8UC1);
            cv::equalizeHist(grayFrame, equalized);

            // erode and dilate the edges to remove noise
            int morphologySize = 1;
            cv::Mat frameDilated;
            cv::dilate(equalized, frameDilated, cv::Mat(), cv::Point(-1, -1), morphologySize);
            cv::erode(frameDilated, frameEroded, cv::Mat(), cv::Point(-1, -1), morphologySize);

            // create ROIs for each lane
            cv::Point p1(0, 0);
            cv::Point p2(frameEroded.cols, 80); 
            cv::Rect rect1(p1, p2); 

            cv::Point p3(0, 90);
            cv::Point p4(frameEroded.cols, 220); 
            cv::Rect rect2(p3, p4); 

            cv::Point p5(0, 420);
            cv::Point p6(frameEroded.cols, 600); 
            cv::Rect rect3(p5, p6); 

            cv::Point p7(0, 650);
            cv::Point p8(frameEroded.cols, 850); 
            cv::Rect rect4(p7, p8); 

            roi1 = frameEroded(rect1); 
            roi2 = frameEroded(rect2); 
            roi3 = frameEroded(rect3); 
            roi4 = frameEroded(rect4); 

			// extract the foreground mask from image for each ROI
            pMOG2_roi1->apply(roi1, mask1);
            pMOG2_roi2->apply(roi2, mask2); 
            pMOG2_roi3->apply(roi3, mask3); 
            pMOG2_roi4->apply(roi4, mask4); 
            
            // track any cars that may be present 
            isTracking1 = trackCars(captureFrame, mask1, captureFrame(rect1), cv::Scalar(0, 255, 0)); 
            isTracking2 = trackCars(captureFrame, mask2, captureFrame(rect2), cv::Scalar(0, 255, 0)); 
            isTracking3 = trackCars(captureFrame, mask3, captureFrame(rect3), cv::Scalar(0, 0, 255)); 
            isTracking4 = trackCars(captureFrame, mask4, captureFrame(rect4), cv::Scalar(0, 0, 255)); 

            // check how many frames have passed since the tracker was created for each ROI
            // if not enough frames, increase the frane count
            if(isTracking1 || isTracking2)
            {
                if(frameCount1 >= 45)
                {
                    westbound_cnt++; 
                    frameCount1 = 0; 
                    isTracking1 = false; 
                }

                else if(frameCount2 >= 45)
                {
                    westbound_cnt++; 
                    frameCount2 = 0; 
                    isTracking2 = false; 
                }

                else
                {
                    frameCount1++; 
                    frameCount2++; 
                }
            }

            else if(isTracking3 || isTracking4)
            {
                if(frameCount3 >= 45)
                {
                    eastbound_cnt++; 
                    frameCount3 = 0; 
                    isTracking3 = false; 
                }

                else if(frameCount4 >= 45)
                {
                    eastbound_cnt++; 
                    frameCount4 = 0; 
                    isTracking4 = false; 
                }

                else
                {
                    frameCount3++; 
                    frameCount4++; 
                }
            }
        }

        // update the GUI window if necessary
        if(captureSuccess)
        {
            // commented out images are for debugging
            cv::imshow("captureFrame", captureFrame);
			//cv::imshow("fgMask", fgMask);
            //cv::imshow("rect1", rect1);
            //cv::imshow("rect2", rect2);
            //cv::imshow("rect3", rect3);
            //cv::imshow("roi4", roi4);
            //cv::imshow("mask1", mask1);
            //cv::imshow("mask2", mask2);
            //cv::imshow("mask3", mask3);
            //cv::imshow("mask4", mask4);

            // display the car count in the console
            std::cout << "WESTBOUND COUNT: " << westbound_cnt << std::endl; 
            std::cout << "EASTBOUND COUNT: " << eastbound_cnt << std::endl; 

            // get the number of milliseconds per frame
            int delayMs = (1.0 / captureFPS) * 1000;

            // check for program termination
            if(((char) cv::waitKey(delayMs)) == 'q')
            {
                doCapture = false;
            }
        }
    }

    // release program resources before returning
    capture.release();
    cv::destroyAllWindows();
}