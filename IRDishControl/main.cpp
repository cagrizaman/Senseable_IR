// 'q' for quit

//include libraries
#include <iostream>
#include <iomanip>

//opencv
#include <opencv2/opencv.hpp>

//opencv tracking
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"

//for serial
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

//unused libraries
//#include <time.h>
//#include <iostream>
//#include <errno.h>
//#include <stdio.h>
//#include <string.h>
//#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

//properties for capture
#define capWidth 640
#define capHeight 480

//current angles
int PHor=30, PVer=0;
int PHor2, PVer2;

//previousPositions
int prevX,prevY;
//mapping variables
float angleRange=(M_PI/180)*38;


//variables for serial
int fd = 0;
char serialport[256];
int baudrate = B19200;  // default
char buf[8];
int rc,n;
bool handshake=false;

// IR turn on / turn off

int bulbState = 0;
//initiate tracking
bool initTrack = false;
bool tracking=false;
int look2sec=0;
int look10sec=0;
int timer =0;
bool iSawYou=false;
//setup variables for blobs and capture
VideoCapture cap(1);
Mat frame;
string windowName;

//for init area
CvPoint iAPt1;
CvPoint iAPt2;

//functions
bool mouseMode();

//transform matrix
Mat keystone;

//detection

//paths to classifiers
//Matthias:
String upperBodyCascade = "/usr/local/share/OpenCV/haarcascades/haarcascade_fullbody.xml";
//String upperBodyCascade = "/usr/local/share/OpenCV/haarcascades/haarcascade_lowerbody.xml";
//String upperBodyCascade = "/usr/local/share/OpenCV/haarcascades/haarcascade_profileface.xml";

//Cagri:
//String upperBodyCascade = "/Users/pinhan/Documents/MIT/IR Project/IRDishControl/haarcascade_fullbody.xml";
//String upperBodyCascade = "/Users/pinhan/Documents/MIT/IR Project/IRDishControl/haarcascade_upperbody.xml";
//String upperBodyCascade = "/Users/pinhan/Documents/MIT/IR Project/IRDishControl/haarcascade_profileface.xml";

CascadeClassifier upperBody_cascade;

//for mouse events
int initStatus = 0;
bool settingArea = false;


//keystone
void antiKeystone(Mat input, Mat output) {
    
}


//serialFunctions
int serialport_writebyte( int fd, int b) {
    ssize_t n = write(fd,&b,1);
	if( (int)n!=1)
        return -1;
    return 0;
}
int serialport_write(int fd, const char* str) {
    size_t len = strlen(str);
	ssize_t n = write(fd, str, len);
    if( (int)n!=len )
        return -1;
    return 0;
}
int serialport_read_until(int fd, char* buf, char until) {
    
    char b[1];
    int i=0;
    do {
        ssize_t n = read(fd, b, 1);  // read a char at a time
        if( (int)n==-1) return -1;    // couldn't read
        if( (int)n==0 ) {
            usleep( 10 * 1000 ); // wait 10 msec try again
            continue;
        }
        buf[i] = b[0]; i++;
    } while( b[0] != until );
    
    buf[i] = 0;  // null terminate the string
    return 0;
}
int serialport_init(const char* serialport, int baud) {
    struct termios toptions;
    int fd;
    
    fprintf(stderr,"init_serialport: opening port %s @ %d bps\n",
            serialport,baud);
    
    fd = open(serialport, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1)  {
        perror("init_serialport: Unable to open port ");
        return -1;
    }
    
    if (tcgetattr(fd, &toptions) < 0) {
        perror("init_serialport: Couldn't get term attributes");
        return -1;
    }
    speed_t brate = baud; // let you override switch below if needed
    switch(baud) {
        case 4800:   brate=B4800;   break;
        case 9600:   brate=B9600;   break;
#ifdef B14400
        case 14400:  brate=B14400;  break;
#endif
        case 19200:  brate=B19200;  break;
#ifdef B28800
        case 28800:  brate=B28800;  break;
#endif
        case 38400:  brate=B38400;  break;
        case 57600:  brate=B57600;  break;
        case 115200: brate=B115200; break;
    }
    cfsetispeed(&toptions, brate);
    cfsetospeed(&toptions, brate);
    
    // 8N1
    toptions.c_cflag &= ~PARENB;
    toptions.c_cflag &= ~CSTOPB;
    toptions.c_cflag &= ~CSIZE;
    toptions.c_cflag |= CS8;
    // no flow control
    toptions.c_cflag &= ~CRTSCTS;
    
    toptions.c_cflag |= CREAD | CLOCAL;  // turn on READ & ignore ctrl lines
    toptions.c_iflag &= ~(IXON | IXOFF | IXANY); // turn off s/w flow ctrl
    
    toptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // make raw
    toptions.c_oflag &= ~OPOST; // make raw
    
    
    toptions.c_cc[VMIN]  = 0;
    toptions.c_cc[VTIME] = 20;
    
    if( tcsetattr(fd, TCSANOW, &toptions) < 0) {
        perror("init_serialport: Couldn't set term attributes");
        return -1;
    }
    
    fprintf(stderr,"init_serialport: port setup %s @ %d bps\n",
            serialport,baud);
    return fd;
}

//function called on mouseEvent where mouse callback is activated
static void onMouse( int event, int x, int y, int, void* ) {
    
    //cout << "mouse-x: " << x << " mouse-y: " << y << endl;
    
    if (initStatus == 0) {
        iAPt1.x = x;
        iAPt1.y = y;
		iAPt2.x = x;
        iAPt2.y = y;
    } //if initStatus is 1 move both points to mouse position
    
    if (initStatus == 1) {
        iAPt2.x = x;
        iAPt2.y = y;
    } //if initStatus is 2 move point 2 to mouse position
    
    if(event == CV_EVENT_LBUTTONDOWN) {
        initStatus ++;
        cout << "iAPt1: " << iAPt1.x << "/" << iAPt1.y << " iAPt2: " <<  iAPt2.x << "/" << iAPt2.y << endl;
    } //if left mouse is pressed increment initStatus
}

//draw initiation Area on video frame
void drawInitArea(){}


Mat undistFrame;


//function to set area where blobs are looked for for further tracking
bool setInitiationArea(string windowName) {
    while(1) {
        cap >> frame;
		
        warpPerspective(frame, undistFrame, keystone, Size(capWidth,capHeight));
        
        if (initStatus == 1) rectangle(undistFrame, iAPt1, iAPt2, Scalar(255,0,0)); //draw rectangle to preview initArea
        if (initStatus > 1) return true; //return after second point is set
		
        char key = waitKey(10);
        switch (key) {
            case 'q': case 'Q': return false;
            case 'm': case 'M': mouseMode(); return false;
        }
        imshow(windowName, undistFrame);
    } //input of points for rectangular area
}

//function sends new angles to arduino
void updateDish() {
    /*
     n = strtol("45", NULL,10); //convert string to number
     
     serialport_read_until(fd, buf, 'C');
     // printf("read: %s\n",buf);
     
     usleep(10*1000);
     
     
     serialport_read_until(fd, buf, 'G');
     // printf("read: %s\n",buf);
     usleep( 10 * 1000 );
     
     serialport_writebyte(fd, 200);
     usleep( 10 * 1000 );
     tcdrain(fd);
     serialport_writebyte(fd, PVer);
     usleep( 10 * 1000);
     tcdrain(fd);
     
     serialport_read_until(fd, buf, 'G');
     // printf("read: %s\n",buf);
     
     usleep( 10 * 1000);
     
     serialport_read_until(fd, buf, 'F');
     //  printf("read: %s\n",buf);
     usleep( 10 * 1000 );
     
     
     serialport_writebyte(fd, 201);
     tcdrain(fd);
     
     usleep( 10 * 1000 );
     
     //cout << "serial PHor = " << PHor << " PVer = " << PVer << endl;
     
     serialport_writebyte(fd, PHor);
     tcdrain(fd);
     
     usleep( 10 * 1000 );
     
     serialport_read_until(fd, buf, 'F');
     //printf("read: %s\n",buf);
     usleep( 10 * 1000 );
     
     serialport_read_until(fd, buf, 'H');
     //printf("read: %s\n",buf);
     usleep( 10 * 1000 );
     
     serialport_writebyte(fd, 202);
     tcdrain(fd);
     
     usleep( 10 * 1000 );
     
     //cout << "serial PHor = " << PHor << " PVer = " << PVer << endl;
     
     serialport_writebyte(fd, bulbState);
     tcdrain(fd);
     
     usleep( 10 * 1000 );
     
     serialport_read_until(fd, buf, 'H');
     //tcflush(<#int#>, <#int#>)
     //printf("read: %s\n",buf);
     
     
     // tcflush(fd, TCIOFLUSH);
     
     */
    
    serialport_writebyte(fd, 200);
    tcdrain(fd);
    serialport_writebyte(fd, PVer);
    tcdrain(fd);
    serialport_writebyte(fd, PHor);
    tcdrain(fd);
    serialport_writebyte(fd, bulbState);
    tcdrain(fd);
    serialport_writebyte(fd, 201);
    tcdrain(fd);
    
    
    
}

//function to map value from source-range to new range
float mapToRange(int input, int srcRangeMin, int srcRangeMax, int dstRangeMin, int dstRangeMax) {
    return (((float)input-(float)srcRangeMax)/((float)srcRangeMax-(float)srcRangeMin))*((float)dstRangeMax-(float)dstRangeMin)+(float)dstRangeMin;
}

//function to determine angles to rotate dish to. determined through pixel-coordinates and calibration-Values
void calcAngles(int PX, int PY) {
    
    //PVer = int((float)PY * ( (float)DVer - (float)AVer ) / (float)capHeight);
    //PHor = int( (float)AHor - ((float)PX * (float)(AHor - BHor) / (float)capWidth) );
    
    //if(PVer>13){
    //  PVer=PVer-13;
    //}
    
    
    
    
    float distance =(1/tan(angleRange))*(capWidth/2);
    
    float verticalRange= atan((capHeight/2)/distance);
    float PHorN=atan((PX-(capWidth/2))/distance);
    float PVerN=atan((PY-(capHeight/2))/distance);
    PVer= PVerN*(180/M_PI)+(verticalRange*180/M_PI);
    PHor= (angleRange*2)-PHorN*(180/M_PI)+(angleRange*180/M_PI);
    
    cout<<PVer<<"Vert\n";
    cout<<PHor<<"Horz\n";
    /*
     //experimental mapping approach
     //calculate PHor maxima
     int PHorMin = mapToRange(PY, 0, capHeight, AHor, DHor);
     int PHorMax = mapToRange(PX, 0, capWidth, BHor, CHor);
     PHor2 = mapToRange(PX, 0, capWidth, PHorMin, PHorMax);
     
     
     //calculate PVer
     int PVerMin;
     int PVerMax;
     //calculate PVer-extremes depending on what horizontal half of the picture P is in
     if (PX > capWidth/2)
     {
     PVerMin = mapToRange(PX, capWidth/2, capWidth, FVer, BVer);
     PVerMax = mapToRange(PX, 0, capWidth/2, EVer, CVer);
     }
     else
     {
     PVerMin = mapToRange(PX, 0, capWidth/2, AVer, FVer);
     PVerMax = mapToRange(PX, 0, capWidth/2, DVer, EVer);
     }
     PVer2 = mapToRange(PY, 0, capHeight, PVerMin, PVerMax);
     cout << "map 2 PHor = " << PHor2 << " PVer = " << PVer2 << endl;
     */
    
}

//function called on mouseEvent during mouseMode
static void mouseXY( int event, int x, int y, int, void* ) {
    calcAngles(x, y);
}

//function to use mouse to point dish
bool mouseMode(){
    destroyWindow(windowName);
    windowName = "mouseMode";
    namedWindow(windowName, WINDOW_AUTOSIZE);
    setMouseCallback( windowName, mouseXY, 0 );
    bulbState=1;
    
    while (1) {
        
        cap>>frame;
        warpPerspective(frame, undistFrame, keystone, Size(capWidth,capHeight));
        imshow(windowName, undistFrame);
        char key = waitKey(10);
        switch (key) {
            case 'q': case 'Q': bulbState=0; updateDish(); close(fd); return false;
        }
        updateDish();
    }
}

//detects upper bodies and draws rectangles on frame to indicate them, takes in current video frame
void cascadeDetect( Mat frame ) {
    //Compare look2sec and look10sec with global timer
    
    std::vector<cv::Rect> upperBodies;
    Mat frame_gray;
    
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    
    //-- Detect upperBodies
    upperBody_cascade.detectMultiScale( frame_gray, upperBodies, 1.1, 2, 0, cv::Size(50, 50) );
    
    
    // draw detections
    for( int i = 0; i < upperBodies.size(); i++ ){
        timer=0;
        cv::Point center( upperBodies[i].x + upperBodies[i].width*0.5, upperBodies[i].y + upperBodies[i].height*0.5 );
        ellipse( frame, center, cv::Size( upperBodies[i].width*0.5, upperBodies[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 0 ), 2, 8, 0 );
        
        
        if(center.x>iAPt1.x&&center.y>iAPt1.y&&center.x<iAPt2.x&&center.y<iAPt2.y &&!initTrack &&!tracking){
            iSawYou=true;
            look2sec++;
            // cout<<"look2sec"<<look2sec<<"\n";
            if(look2sec>50){
                bulbState=1;
                initTrack=true;
                tracking=true;
                prevX=center.x;
                prevY=center.y;
                calcAngles(prevX, prevY);
                
                
            }
        }
        //This is the change I made
        else{
            bulbState=0;
            initTrack=false;
            //look2sec=0;
            // calcAngles(0, 30);
        }
        
        if(tracking){
            if(look10sec>700){
                bulbState=0;
                tracking=false;
                initTrack=false;
                look10sec=0;
                //calcAngles(0, 30);
            }
            else{
                //cout<<"look10sec"<<look10sec<<"\n";
                bulbState=1;
                look10sec++;
                int distance = sqrt((pow((double)center.x-prevX, 2)+pow((double)center.y-prevY,2)));
                if(distance<50){
                    prevX=center.x;
                    prevY=center.y;
                    // cout<<"I am tracking you \n";
                    calcAngles(center.x, center.y);
                    ellipse( frame, center, cv::Size( upperBodies[i].width*0.5, upperBodies[i].height*0.5), 0, 0, 360, Scalar( 255, 255, 0 ), 2, 8, 0 );
                    
                }
            }
        }
    }
    
    if(iSawYou==false)
        look2sec=0;
    
    iSawYou=false;
    if(upperBodies.size()==0){
        timer++;
        // cout<<"empty frame"<<timer<<"\n";
        if(timer>100){
            bulbState=0;
            tracking=false;
            initTrack=false;
            look2sec=0;
            timer=0;
            //calcAngles(0, 30);
        }
        
        
    }
}

int main(){
    
    //settings for capture size
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,capHeight);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,capWidth);
    cap.set(CV_CAP_PROP_CONTRAST,50);
    
    
    //set keystone transform matrix
    Point2f src[4], dst[4];
    src[0].x = 0;
    src[0].y = 0;
    dst[0].x = capWidth*0.175;
    dst[0].y = capHeight*0;
    src[1].x = capWidth;
    src[1].y = 0;
    dst[1].x = capWidth*0.80;
    dst[1].y = 0;
    src[2].x = capWidth;
    src[2].y = capHeight;
    dst[2].x = capWidth;
    dst[2].y = capHeight;
    src[3].x = 0;
    src[3].y = capHeight;
    dst[3].x = capWidth*0;
    dst[3].y = capHeight;
    
    keystone = getPerspectiveTransform(src, dst);
    cap >> frame;
    undistFrame.create(capWidth*0.85, capHeight, frame.type());
    
    if( !upperBody_cascade.load( upperBodyCascade ) ){ printf("--(!)Error loading\n"); return -1; }
    
    //set serial port
    fd = serialport_init("/dev/tty.usbmodem241441", baudrate);
    usleep(100000);
    //set initiation area
    windowName = "initArea";
    namedWindow(windowName, WINDOW_AUTOSIZE);
    setMouseCallback( windowName, onMouse, 0 );
    if (!setInitiationArea(windowName)) {
        cout << "Program canceled during initiatinAreaSet" << endl;
        close(fd);
        return -1;
    }
    destroyWindow(windowName);
    
    
    //detect
    windowName = "detection";
    
    
    cout<<fd;
    
    while (1) {
        cap >> frame;
        
        warpPerspective(frame, undistFrame, keystone, Size(capWidth,capHeight));
        
        cascadeDetect(undistFrame);
        
        imshow(windowName, undistFrame);
        
        char key = cvWaitKey(10);
        
        switch (key) {
            case 'q': case 'Q' :close(fd);  return -1;
        }
        updateDish();
    }
}



