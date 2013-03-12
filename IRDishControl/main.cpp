// 'q' for quit
// 'p' for print config
// 'n' for next calibration point
// 'l' for load calibration

//include libraries
#include <iostream>
#include <iomanip>
//opencv
#include <opencv2/opencv.hpp>
//opencv tracking
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//for serial
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <time.h>
#include <iostream>
//for time
#include <CoreServices/CoreServices.h>
#include <sys/time.h>

using namespace cv;
using namespace std;

//properties for capture
#define capWidth 480
#define capHeight 270

//calibration-values
int AHor, AVer;
int BHor, BVer;
int CHor, CVer;
int DHor, DVer;
int EHor, EVer;
int FHor, FVer;

//current angles
int PHor=45, PVer=45;
int PHor2, PVer2;

//variables for serial
int fd = 0;
char serialport[256];
int baudrate = B19200;  // default
char buf[256];
int rc,n;
bool handshake=false;

//initiate tracking
bool initTrack = false;
bool tracking=false;
int look2sec=0;

//setup variables for blobs and capture
VideoCapture cap(1);
Mat frame;

//for init area
CvPoint iAPt1;
CvPoint iAPt2;

//detection
String upperBodyCascade = "/Users/pinhan/Documents/MIT/IR Project/IRDishControl/haarcascade_upperbody.xml";
CascadeClassifier upperBody_cascade;


//for mouse events
int initStatus = 0;
bool settingArea = false;

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

//function to convert string to char
char* stringToChar(string s) {
	
	char *a=new char[s.size()+1];
	a[s.size()]=0;
	memcpy(a,s.c_str(),s.size());
	
	return a;
	
}

//draw initiation Area
void drawInitArea(){}

//function to set area where blobs are looked for for further tracking
bool setInitiationArea(string windowName) {
    while(1) {
        cap >> frame;
		
        if (initStatus == 1) rectangle(frame, iAPt1, iAPt2, Scalar(255,0,0)); //draw rectangle to preview initArea
        if (initStatus > 1) return true; //return after second point is set
		
        char key = waitKey(10);
        switch (key) {
            case 'q': case 'Q': return false;
        }
        imshow(windowName, frame);
    } //input of points for rectangular area
}

//function sends new angles to arduino
void updateDish() {
    
    n = strtol("45", NULL,10); //convert string to number
	if(!handshake){
    cout<<"reading";
    serialport_read_until(fd, buf, 'R');
    //printf("read: %s\n",buf);
    cout<<"writing";
    serialport_writebyte(fd, 'R');
    usleep( 10 * 1000 );
        handshake=true;
    }
    cout<<"reading";
    serialport_read_until(fd, buf, 'G');
    //printf("read: %s\n",buf);
    cout<<"writing";
    serialport_writebyte(fd, 200);
    usleep( 10 * 1000 );
    
    serialport_writebyte(fd, PVer);
    usleep( 10 * 1000 );
    
    serialport_read_until(fd, buf, 'G');
    //printf("read: %s\n",buf);
    
    
    serialport_read_until(fd, buf, 'F');
    //printf("read: %s\n",buf);
    
    
    serialport_writebyte(fd, 201);
    usleep( 10 * 1000 );
    
    //cout << "serial PHor = " << PHor << " PVer = " << PVer << endl;
    
    serialport_writebyte(fd, PHor);
    usleep( 10 * 1000 );
    
    serialport_read_until(fd, buf, 'F');
    //printf("read: %s\n",buf);
    
}

//function to map value from source-range to new range
float mapToRange(int input, int srcRangeMin, int srcRangeMax, int dstRangeMin, int dstRangeMax) {
    return (((float)input-(float)srcRangeMax)/((float)srcRangeMax-(float)srcRangeMin))*((float)dstRangeMax-(float)dstRangeMin)+(float)dstRangeMin;
}

//function to determine angles to rotate dish to. determined through pixel-coordinates and calibration-Values
void calcAngles(int PX, int PY) {
    
    PVer = int((float)PY * ( (float)DVer - (float)AVer ) / (float)capHeight);
    PHor = int( (float)AHor - ((float)PX * (float)(AHor - BHor) / (float)capWidth) );
    
    
    cout<<PVer<<"\n";
    cout<<PHor<<"\n";
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

//function to load a preset configuration
void loadSavedConfig() {
    AHor = 90; AVer = 0;
    BHor = 0; BVer = 0;
    CHor = 0; CVer = 60;
    DHor = 90; DVer = 60;
    EHor = 45; EVer = 0;
    FHor = 45; FVer = 60;
}

//function to calibrate dish
bool calibrateDish(string windowName) {
    
    char key;
    int stage = 1;
    int radius = 30;
    
    while (1) {
        
        cap >> frame;
		
		// draw circle according to calibration Stage
        switch (stage) {
            case 1:
                circle(frame, Point2d(0,0), radius, Scalar(255,255,255), 3); break;
            case 2:
                circle(frame, Point2d(capWidth,0), radius, Scalar(255,255,255), 3); break;
            case 3:
                circle(frame, Point2d(capWidth,capHeight), radius, Scalar(255,255,255), 3); break;
            case 4:
                circle(frame, Point2d(0,capHeight), radius, Scalar(255,255,255), 3); break;
            case 5:
                circle(frame, Point2d(capWidth/2,0), radius, Scalar(255,255,255), 3); break;
            case 6:
                circle(frame, Point2d(capWidth/2,capHeight), radius, Scalar(255,255,255), 3); break;
        }
        
        imshow(windowName, frame);
        key = '0';
        key = cvWaitKey(5);
        
        switch (key) {
            case 'w': case 'W': PVer --; cout << 'W'<<PVer; break;
                
            case 's': case 'S': PVer ++; cout << 'S'<<PVer; break;
                
            case 'D': case 'd': PHor --; cout << 'D'<<PHor; break;
                
            case 'A': case 'a': PHor ++; cout << 'A'<<PHor; break;
                
            case 'N': case 'n': cout << 'N' << endl;
                
				switch (stage) {
                    case 1: AHor = PHor; AVer = PVer; break;
                    case 2: BHor = PHor; BVer = PVer; break;
                    case 3: CHor = PHor; CVer = PVer; break;
                    case 4: DHor = PHor; DVer = PVer; break;
                    case 5: EHor = PHor; EVer = PVer; break;
                    case 6: FHor = PHor; FVer = PVer; return true;} //save point according to stage
                //usleep(1000*10);
                stage++; cout << "Stage: " << stage << endl; break;
                
            case 'L': case 'l':
                loadSavedConfig(); stage = 7;
                cout << "load config";
                return true;
                
            case 'Q': case 'q': return false;
        }
        
        //qupdateDish();
    }
    
    
    
}

//sorts the spanning points of the initArea-rectangle so that iAPt1 is the top left one and iAPt2 is the bottom right one
void sortInitAreaPoints() {
	int x1, x2, y1, y2;
	
	x1 = iAPt1.x;
	y1 = iAPt1.y;
	x2 = iAPt2.x;
	y2 = iAPt2.y;
	
	if (!(x1<x2)) {iAPt1.x = x2; iAPt2.x = x1;}
	if (!(y1<y2)) {iAPt1.y = y2; iAPt2.y = y1;}
}

double GetTimeSinceBootInMilliseconds() {
	struct timeval tm;
	gettimeofday( &tm, NULL );
	return (double)tm.tv_sec + (double)tm.tv_usec / 1000000.0;
}

//detects upper bodies and draws rectangles on frame to indicate them, takes in current video frame
void cascadeDetect( Mat frame ) {
    
    std::vector<cv::Rect> upperBodies;
    Mat frame_gray;
    
    cvtColor( frame, frame_gray, CV_BGR2GRAY );
    equalizeHist( frame_gray, frame_gray );
    
    //-- Detect upperBodies
    upperBody_cascade.detectMultiScale( frame_gray, upperBodies, 1.1, 2, 0, cv::Size(80, 80) );

    // draw detections
    for( int i = 0; i < upperBodies.size(); i++ ){
        
        cv::Point center( upperBodies[i].x + upperBodies[i].width*0.5, upperBodies[i].y + upperBodies[i].height*0.5 );
        ellipse( frame, center, cv::Size( upperBodies[i].width*0.5, upperBodies[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 0 ), 2, 8, 0 );
        
        
            if(center.x>iAPt1.x&&center.y>iAPt1.y&&center.x<iAPt2.x&&center.y<iAPt2.y &&!initTrack &&!tracking){
        
                look2sec++;
                cout<<"stage"<<look2sec;
                if(look2sec>50){
                    initTrack=true;
                    tracking=true;
                    calcAngles(center.x, center.y);
                   
                }
            }
            else{
                initTrack=false;
                look2sec=0;
            }
        
        if(tracking){
            calcAngles(center.x, center.y);
        }
        
    }
}




int main(){
    //settings for capture size
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,capHeight);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,capWidth);
    if( !upperBody_cascade.load( upperBodyCascade ) ){ printf("--(!)Error loading\n"); return -1; }
    string windowName;
    
    //set initiation area
    windowName = "initArea";
    namedWindow(windowName, WINDOW_AUTOSIZE);
    setMouseCallback( windowName, onMouse, 0 );
    if (!setInitiationArea(windowName)) {
        cout << "Program canceled during initiatinAreaSet" << endl;
        return -1;
    }
    destroyWindow(windowName);
    
    //calibrate dish
    windowName = "calibration";
    namedWindow(windowName, WINDOW_AUTOSIZE);
    if (!calibrateDish(windowName)) {
        cout << "Program canceled during dishCalibration" << endl;
        return -1;
    }
    destroyWindow(windowName);
    
    //detect
    windowName = "detection";
    
    //set serial port
    fd = serialport_init("/dev/tty.usbmodem264421", baudrate);
    
   

    cout<<fd;
    
    while (1) {
        cap >> frame;
        
        cascadeDetect(frame);
        
        imshow(windowName, frame);
        
        char key = cvWaitKey(10);
        
        switch (key) {
            case 'q': case 'Q' : return -1;
        }
        updateDish();
    }
}



