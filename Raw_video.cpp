#include <typeinfo> 
#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <time.h>
#include <fstream>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <cstdlib>
#include <math.h>
#include <ctime>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include "boost/date_time/posix_time/posix_time.hpp"


using namespace std;
using namespace boost::posix_time;
using namespace cv;

//Gloabal variables
int fps=10;
char key;
Mat image, image1;
char fname[100], fname1[100];
int USB = open( "/dev/ttyACM0", O_RDWR| O_NONBLOCK | O_NDELAY );	//Opening communication with the arduino for checking LED status
int main()
{
    //Setting up communication with arduino
char flag[1];
	int n;
  /* Error handling in case Arduino does not respond to the serial communication request*/
  if ( USB < 0 ){
    cout << "Error " << errno << " opening " << "/dev/ttyACM0" << ": " << strerror (errno) << endl;
  }

  /*Setting the baud rate and other parameters for serial communication with arduino*/
  struct termios tty;
  memset (&tty, 0, sizeof tty);
  cfsetospeed (&tty, B9600);
  cfsetispeed (&tty, B9600);
  memset (&flag, '\0', sizeof flag);

  /* Setting other Port Stuff for Arduino */
  tty.c_cflag     &=  ~PARENB;        	// Make 8n1
  tty.c_cflag     &=  ~CSTOPB;
  tty.c_cflag     &=  ~CSIZE;
  tty.c_cflag     |=  CS8;
  tty.c_cflag     &=  ~CRTSCTS;       	// no flow control
  tty.c_lflag     =   0;          	// no signaling chars, no echo, no canonical processing
  tty.c_oflag     =   0;                  // no remapping, no delays
  tty.c_cc[VMIN]      =   0;              // read doesn't block
  tty.c_cc[VTIME]     =   5;              // 0.5 seconds read timeout

  tty.c_cflag     |=  CREAD | CLOCAL;     	// turn on READ & ignore ctrl lines
  tty.c_iflag     &=  ~(IXON | IXOFF | IXANY);	// turn off s/w flow ctrl
  tty.c_lflag     &=  ~(ICANON | ECHO | ECHOE | ISIG); // make raw
  tty.c_oflag     &=  ~OPOST;              	// make raw

  /* Flush Port, then applies attributes */
  tcflush( USB, TCIFLUSH );
  if ( tcsetattr ( USB, TCSANOW, &tty ) != 0){
    cout << "Error " << errno << " from tcsetattr" << endl;
  }  
time_duration td, td1;
ptime nextFrameTimestamp, currentFrameTimestamp, initialLoopTimestamp, finalLoopTimestamp;
int delayFound = 0;
int totalDelay= 0;
int framerate = 120;
double f=120;


VideoCapture capture(0);	//Opens the camera of the device connected
VideoCapture capture1(1);
capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);

capture1.set(CV_CAP_PROP_FRAME_WIDTH,640);
capture1.set(CV_CAP_PROP_FRAME_HEIGHT,480);


capture>>image;			//Extract a frame and store in image matrix.
capture1>>image1;
int framewidth=image.cols;  //Obtain size of image matrix in the form of row7 s and columns
int frameheight=image.rows;

nextFrameTimestamp = microsec_clock::local_time();
currentFrameTimestamp = nextFrameTimestamp;
td = (currentFrameTimestamp - nextFrameTimestamp);

strcpy(fname, "../../Videos/left.avi");	//declaring path for storing video
strcpy(fname1, "../../Videos/right.avi");	//declaring path for storing video

/*Define VideoiWriter object for storing the video*/
VideoWriter video(fname,CV_FOURCC('M','J','P','G'),framerate,cvSize(framewidth, frameheight));  //CV_FOURCC('M','J','P','G') is a motion-jpeg codec
VideoWriter video1(fname1,CV_FOURCC('M','J','P','G'),framerate,cvSize(framewidth, frameheight));

 n = write( USB,"s", sizeof (char) );	//Sends a serial variable 's' while hints the arduino program to start the blink code. This was "					done to sync with the arduino and giving the control of the arduino program in the cpp file here.
cout << "Returning Value " << n << endl;
while(1)
{

// wait for X microseconds until 1second/framerate time has passed after previous frame write
	while(td.total_microseconds() < 1000000/framerate)
        {
        //determine current elapsed time
        currentFrameTimestamp = microsec_clock::local_time();
        td = (currentFrameTimestamp - nextFrameTimestamp);
		}
    //	 determine time at start of write
    initialLoopTimestamp = microsec_clock::local_time();
    capture>>image;
    video<<image;
    capture1>>image1;
    video1<<image1;
    Size sz1 = image.size();
    Size sz2 = image1.size();
    Mat im3(sz1.height, sz1.width+sz2.width, CV_8UC3);
    Mat left(im3, Rect(0, 0, sz1.width, sz1.height));
    image.copyTo(left);
    Mat right(im3, Rect(sz1.width, 0, sz2.width, sz2.height));
    image1.copyTo(right);
    namedWindow("Video",WINDOW_NORMAL);
    imshow("Video",im3);
    // video1<<im3;

    //write previous and current frame timestamp to console
    cout << nextFrameTimestamp << endl << currentFrameTimestamp << endl;
    // add 1second/framerate time for next loop pause
    nextFrameTimestamp = nextFrameTimestamp + microsec(1000000/framerate);

 	// reset time_duration so while loop engages
 	td = (currentFrameTimestamp - nextFrameTimestamp);

    cout<<(td) <<" "<<endl;
 	//determine and print out delay in ms, should be less than 1000/FPS
 	//occasionally, if delay is larger than said value, correction will occur
 	//if delay is consistently larger than said value, then CPU is not powerful
 	finalLoopTimestamp = microsec_clock::local_time();
    td1 = (finalLoopTimestamp - initialLoopTimestamp);
	delayFound = td1.total_milliseconds();
	cout << delayFound << endl;

    key = waitKey(100); 	//Capture Keyboard stroke
    if (char(key) == 27)
	{
	        break; 		//If you hit ESC key loop will break and code will terminate

	}
}
capture.release();
capture1.release();
return 0;
}

