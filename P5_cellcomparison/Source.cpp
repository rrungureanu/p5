#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <fstream>
#include <queue>
#include <vector>
#include <chrono>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string.h>
#include "SerialPort.h"

using namespace std;
using namespace cv;
#pragma comment (lib, "Ws2_32.lib")
#define WELLSNR 96
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "30002" 


#define	pos_CALIBRATION 0
#define	pos_STANDBY 1
#define	pos_VIS_APP 2 
#define	pos_VIS_IN 3
#define	pos_RACK_LOW_APP 4
#define	pos_RACK_HIGH_APP 5
#define	pos_RACK_LOW_IN 6
#define	pos_RACK_HIGH_IN 7 
#define	pos_IO_LOW_APP 8 
#define	pos_IO_HIGH_APP 9 
#define	pos_IO_LOW_IN 10 
#define	pos_IO_HIGH_IN 11

#define	task_INPUT 0
#define	task_EVALUATE 1 
#define	task_OUTPUT 2


class sample;
//Structure that holds the data for each task that is to be performed
struct task {
	int action;
	sample *sampleTarget;
};

#define ADJ_RACK_X 0.201
#define ADJ_RACK_Z 0.070
#define ADJ_IO_Z 0.016

//Global TCP variables
WSADATA wsaData;
int iResult;

SOCKET ListenSocket = INVALID_SOCKET;
SOCKET ClientSocket = INVALID_SOCKET;

struct addrinfo *result = NULL;
struct addrinfo hints;

int iSendResult;
char recvbuf[DEFAULT_BUFLEN];
int recvbuflen = DEFAULT_BUFLEN;

//Global Serial Port
const char *port = "\\\\.\\COM8";
SerialPort arduinoSP(port);

//Global coordinates
Point wellCoordinates[WELLSNR][2];
float positionCoordinates[12][6];
queue<task> taskQueue;

//Global variables
int ioCapacity;
Mat currFrame;
VideoCapture cap;

//Structure the holds the data for each evaluation on a sample
struct evaluation{
	int time;
	bool wellResults[2][WELLSNR];
	string picFilenameWhite, picFilenameUV;
};

//Class that holds the data for each individual well along with methods for assessing and comparing wells
class well
{
private:
	int position, intensity[4][9], intensityAvg; //Position on the sample, also used to identify if well is small or large
	Point topLeft, bottomRight; //Coordinates of the well in the image
public:
	well() {}
	well(int pos, Point tl, Point br) { topLeft = tl; bottomRight = br; position = pos; }
	void readWellColi(Mat image);
	void readWellEColi(Mat image);
	int evalWellColi(well comparator);
	int evalWellEColi(well comparator);
};

//Method that records the intensity values of a well for the 3 different channels in multiple points for a coli image
void well::readWellColi(Mat image)
{
	/*
	int xDiff = bottomRight.x - topLeft.x;
	int yDiff = bottomRight.y - topLeft.y;
		//Point 1
		//rectangle(image, topLeft, topLeft, Scalar(255, 0, 0), 2);
		intensity[0][0] = image.at<Vec3b>(topLeft)[0];
		intensity[1][0] = image.at<Vec3b>(topLeft)[1];
		intensity[2][0] = image.at<Vec3b>(topLeft)[2];
		//Point 2
		//rectangle(image, Point( topLeft.x, topLeft.y + yDiff / 2 ), Point(topLeft.x, topLeft.y + yDiff / 2), Scalar(255, 0, 0), 2);
		intensity[0][1] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x)[0];
		intensity[1][1] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x)[1];
		intensity[2][1] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x)[2];
		//Point 3
		//rectangle(image, Point(topLeft.x, topLeft.y + yDiff), Point(topLeft.x, topLeft.y + yDiff), Scalar(255, 0, 0), 2);
		intensity[0][2] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x)[0];
		intensity[1][2] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x)[1];
		intensity[2][2] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x)[2];
		//Point 4
		//rectangle(image, Point(topLeft.x + xDiff / 2, topLeft.y), Point(topLeft.x + xDiff / 2, topLeft.y), Scalar(255, 0, 0), 2);
		intensity[0][3] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff / 2)[0];
		intensity[1][3] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff / 2)[1];
		intensity[2][3] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff / 2)[2];
		//Point 5
		//rectangle(image, Point(topLeft.x + xDiff / 2,topLeft.y + yDiff / 2), Point(topLeft.x + xDiff / 2, topLeft.y + yDiff / 2), Scalar(255, 0, 0), 2);
		intensity[0][4] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff / 2)[0];
		intensity[1][4] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff / 2)[1];
		intensity[2][4] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff / 2)[2];
		//Point 6
		//rectangle(image, Point(topLeft.x + xDiff / 2,topLeft.y + yDiff), Point(topLeft.x + xDiff / 2, topLeft.y + yDiff), Scalar(255, 0, 0), 2);
		intensity[0][5] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x + xDiff / 2)[0];
		intensity[1][5] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x + xDiff / 2)[1];
		intensity[2][5] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x + xDiff / 2)[2];
		//Point 7
		//rectangle(image, Point(topLeft.x + xDiff, topLeft.y), Point(topLeft.x + xDiff, topLeft.y), Scalar(255, 0, 0), 2);
		intensity[0][6] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff)[0];
		intensity[1][6] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff)[1];
		intensity[2][6] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff)[2];
		//Point 8
		//rectangle(image, Point(topLeft.x + xDiff, topLeft.y + yDiff / 2), Point(topLeft.x + xDiff, topLeft.y + yDiff / 2), Scalar(255, 0, 0), 2);
		intensity[0][7] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff)[0];
		intensity[1][7] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff)[1];
		intensity[2][7] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff)[2];
		//Point 9
		//rectangle(image, bottomRight, bottomRight, Scalar(255, 0, 0), 2);
		intensity[0][8] = image.at<Vec3b>(bottomRight)[0];
		intensity[1][8] = image.at<Vec3b>(bottomRight)[1];
		intensity[2][8] = image.at<Vec3b>(bottomRight)[2];
		*/
	int count = 0;
	int sum = 0;
	int val;
	for(int i= topLeft.x;i<=bottomRight.x;i++)
		for (int j = topLeft.y; j <= bottomRight.y; j++)
		{
			val = image.at<Vec3b>(j,i)[1];
			count++;
			sum += val;
		}
	intensityAvg = sum / count;
}

//Method that records the intensity values of a well for the 3 different channels in multiple points for a Ecoli image
void well::readWellEColi(Mat image)
{
	int xDiff = bottomRight.x - topLeft.x;
	int yDiff = bottomRight.y - topLeft.y;
	//Point 1
	//rectangle(image, topLeft, topLeft, Scalar(255, 0, 0), 2);
	intensity[3][0] = image.at<Vec3b>(topLeft)[1];
	//Point 2
	//rectangle(image, Point( topLeft.x, topLeft.y + yDiff / 2 ), Point(topLeft.x, topLeft.y + yDiff / 2), Scalar(255, 0, 0), 2);
	intensity[3][1] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x)[1];
	//Point 3
	//rectangle(image, Point(topLeft.x, topLeft.y + yDiff), Point(topLeft.x, topLeft.y + yDiff), Scalar(255, 0, 0), 2);
	intensity[3][2] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x)[1];
	//Point 4
	//rectangle(image, Point(topLeft.x + xDiff / 2, topLeft.y), Point(topLeft.x + xDiff / 2, topLeft.y), Scalar(255, 0, 0), 2);
	intensity[3][3] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff / 2)[1];
	//Point 5
	//rectangle(image, Point(topLeft.x + xDiff / 2,topLeft.y + yDiff / 2), Point(topLeft.x + xDiff / 2, topLeft.y + yDiff / 2), Scalar(255, 0, 0), 2);
	intensity[3][4] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff / 2)[1];
	//Point 6
	//rectangle(image, Point(topLeft.x + xDiff / 2,topLeft.y + yDiff), Point(topLeft.x + xDiff / 2, topLeft.y + yDiff), Scalar(255, 0, 0), 2);
	intensity[3][5] = image.at<Vec3b>(topLeft.y + yDiff, topLeft.x + xDiff / 2)[1];
	//Point 7
	//rectangle(image, Point(topLeft.x + xDiff, topLeft.y), Point(topLeft.x + xDiff, topLeft.y), Scalar(255, 0, 0), 2);
	intensity[3][6] = image.at<Vec3b>(topLeft.y, topLeft.x + xDiff)[1];
	//Point 8
	//rectangle(image, Point(topLeft.x + xDiff, topLeft.y + yDiff / 2), Point(topLeft.x + xDiff, topLeft.y + yDiff / 2), Scalar(255, 0, 0), 2);
	intensity[3][7] = image.at<Vec3b>(topLeft.y + yDiff / 2, topLeft.x + xDiff)[1];
	//Point 9
	//rectangle(image, bottomRight, bottomRight, Scalar(255, 0, 0), 2);
	intensity[3][8] = image.at<Vec3b>(bottomRight)[1];
}

//Method that compares the intensity values of the first well using the blue channel values of the second well as a threshold
//returns 1 if the well contains coli, -1 otherwise
int well::evalWellColi(well comparator)
{
	/*
	int coliPositives = 0;
	if (position < 48)
	{
		for (int i = 0; i < 9; i++)
		{
			//cout << intensity[0][i] << " vs comp: " << comparator.intensity[0][i] << endl;
			if (intensity[0][i] <= comparator.intensity[0][i])
			{
				coliPositives++;
			}
		}
		if (coliPositives >= 7)
		{
			return 1;
		}
		else
			return -1;
	}
	else
	{
		for (int i = 0; i < 9; i++)
			if (intensity[0][i] <= comparator.intensity[0][i])
				coliPositives++;
		if (coliPositives >= 7)
		{
			return 1;
		}
		else
			return -1;
	}
	*/
	std::cout << intensityAvg << " vs " << comparator.intensityAvg - 2<<endl;
	if (intensityAvg >= comparator.intensityAvg-2)
		return 1;
	else
		return -1;
}

//Method that compares the intensity values of the first well using the 
int well::evalWellEColi(well comparator)
{
	return -1;
}

//Class that holds the data for each individual sample along with methods for testing wells
class sample
{
private:
	well wells[WELLSNR]; //Vector containing all 96 well objects which the sample contains
	String qrCode; //Position of the sample on the storage rack
	Point rackPos; //Position of the sample on the storage rack
	vector<int> evalTimes; //Vector containing the times at which the sample is to be evaluated relative to the time it first entered the system
	vector<evaluation> evalInstances; //Vector containing the results of each individual evaluation
public:
	chrono::high_resolution_clock::time_point inputTime;
	sample(){}
	sample(Point rackPos, String code, vector<int> &times);
	well returnWell(int i) { return wells[i];}
	Point returnRackPos() { return rackPos; }
	String returnQrCode() { return qrCode; }
	void readSample(Mat imageWhite, Mat imageUV);
	void evalSample(sample comparator, Mat imageWhite, Mat imageUV);
	int computeMPN();
};
vector<sample> sampleVector;
sample comparator;

//Constructor for the sample class
sample::sample(Point position, String code, vector<int> &times)
{
	Point tl, br;
	rackPos = position; qrCode = code; evalTimes = times;
	for (int i = 0; i < WELLSNR; i++)
	{
		if (i < 46)
		{
			tl= Point(wellCoordinates[i][0].x + 5, wellCoordinates[i][0].y + 5); br = Point(wellCoordinates[i][1].x - 5, wellCoordinates[i][1].y - 5);
		}
		else
		{
			tl = Point(wellCoordinates[i][0].x + 5, wellCoordinates[i][0].y + 5); br = Point(wellCoordinates[i][1].x - 5, wellCoordinates[i][1].y - 5);
		}
		wells[i] = well(i, tl, br);
	}
}

//Method that reads the values for each individual well object from an image with white light and an image with UV light
void sample::readSample(Mat imageWhite, Mat imageUV)
{
	for (int i = 0; i < WELLSNR; i++)
	{
		wells[i].readWellColi(imageWhite);
		wells[i].readWellEColi(imageUV);
	}
}

//Method that evaluates a sample against the values of another sample
void sample::evalSample(sample comparator, Mat imageWhite, Mat imageUV)
{
	Mat whitecopy,onechannel;
	cvtColor(imageWhite, whitecopy, COLOR_BGR2HSV);
	this->readSample(whitecopy, imageUV);
	evaluation currEval;
	auto current_time = chrono::high_resolution_clock::now();
	currEval.time = chrono::duration_cast<chrono::minutes>(current_time - this->inputTime).count();
	for (int i = 0; i < WELLSNR; i++)
	{
		well comparatorWell = comparator.returnWell(i);
		if (wells[i].evalWellColi(comparatorWell) == 1)
			currEval.wellResults[0][i] = true;
		else
			currEval.wellResults[0][i] = false;
		if (wells[i].evalWellEColi(comparatorWell) == 1)
			currEval.wellResults[1][i] = true;	
		else
			currEval.wellResults[1][i] = false;
	}
	for (int i = 0; i < WELLSNR; i++)
	{
		if(currEval.wellResults[0][i])
			rectangle(imageWhite, Point(wellCoordinates[i][0].x - 5, wellCoordinates[i][0].y - 5), Point(wellCoordinates[i][1].x + 5, wellCoordinates[i][1].y + 5), Scalar(0, 0, 255),2);
		else
			rectangle(imageWhite, Point(wellCoordinates[i][0].x - 5, wellCoordinates[i][0].y - 5), Point(wellCoordinates[i][1].x + 5, wellCoordinates[i][1].y + 5), Scalar(0, 255, 0),2);
		if (currEval.wellResults[1][i])
			rectangle(imageUV, Point(wellCoordinates[i][0].x - 5, wellCoordinates[i][0].y - 5), Point(wellCoordinates[i][1].x + 5, wellCoordinates[i][1].y + 5), Scalar(0, 0, 255),2);
		else
			rectangle(imageUV, Point(wellCoordinates[i][0].x - 5, wellCoordinates[i][0].y - 5), Point(wellCoordinates[i][1].x + 5, wellCoordinates[i][1].y + 5), Scalar(0, 255, 0),2);
	}
	currEval.picFilenameWhite ="C:\\Users\\rrung\\source\\repos\\P5_cellcomparison\\P5_cellcomparison\\" + this->returnQrCode() + "_" + to_string(currEval.time) + "min_White.jpg";
	currEval.picFilenameUV ="C:\\Users\\rrung\\source\\repos\\P5_cellcomparison\\P5_cellcomparison\\" + this->returnQrCode() + "_" + to_string(currEval.time) + "min_UV.jpg";
	//imshow(currEval.picFilenameWhite, imageWhite);
	//imshow(currEval.picFilenameUV, imageUV);
	//waitKey(0);
	imwrite(currEval.picFilenameWhite, imageWhite);
	imwrite(currEval.picFilenameUV, imageUV);
	evalInstances.push_back(currEval);
}

//Method that computes and returns the most probable number
int sample::computeMPN()
{
	return 0;
}

//Function that initializes the well coordinates and stores it in a global array
void initGlobalPosCoord()
{
	float x, y, z, rx, ry, rz;
	ifstream GPCfile;
	GPCfile.open("GPCfile.txt");
	if (GPCfile.is_open())
	{
		for (int i = 0; i < 12; i++)
		{
			GPCfile >> positionCoordinates[i][0];
			GPCfile >> positionCoordinates[i][1];
			GPCfile >> positionCoordinates[i][2];
			GPCfile >> positionCoordinates[i][3];
			GPCfile >> positionCoordinates[i][4];
			GPCfile >> positionCoordinates[i][5];
		}
	}
	GPCfile.close();
}

//Function that initializes the position coordinates and stores them in a global array
void initGlobalWellCoord()
{
	int y_TL, x_TL, y_BR, x_BR;
	ifstream GWCfile;
	GWCfile.open("GWCfile.txt");
	if (GWCfile.is_open())
	{
		for (int i = 0; i < WELLSNR; i++)
		{
			GWCfile >> y_TL;
			GWCfile >> x_TL;
			GWCfile >> y_BR;
			GWCfile >> x_BR;
			Point TL(y_TL-5, x_TL-5);
			Point BR(y_BR-5, x_BR-5);
			wellCoordinates[i][0] = TL;
			wellCoordinates[i][1] = BR;
		}
	}
	GWCfile.close();
}

void initPositionCoord()
{
	ifstream URPOSfile;
	URPOSfile.open("URPOSfile.txt");
	if (URPOSfile.is_open())
	{
		for (int i = 0; i < 12; i++)
		{
			URPOSfile >> positionCoordinates[i][0];
			URPOSfile >> positionCoordinates[i][1];
			URPOSfile >> positionCoordinates[i][2];
			URPOSfile >> positionCoordinates[i][3];
			URPOSfile >> positionCoordinates[i][4];
			URPOSfile >> positionCoordinates[i][5];
		}
	}
	URPOSfile.close();
}

//Function that initializes and returns a sample object based on the comparator image
void initComparator()
{
	Mat comparatorImgWhite = imread("newComparator.jpg");
	GaussianBlur(comparatorImgWhite, comparatorImgWhite, Size(7, 7), 0, 0);
	cvtColor(comparatorImgWhite, comparatorImgWhite, COLOR_BGR2HSV);
	Mat comparatorImgUV = imread("comparatorPicUV.jpg");
	GaussianBlur(comparatorImgUV, comparatorImgUV, Size(7, 7), 0, 0);
	vector<int> times;
	sample comparatorLocal(Point(-1,-1),"-1",times);
	comparatorLocal.readSample(comparatorImgWhite, comparatorImgUV);
	comparator = comparatorLocal;
	return;
}

//Function that initializes the TCP connection with the UR5 robot
void initTCP()
{
	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		return;
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;
	hints.ai_flags = AI_PASSIVE;

	// Resolve the server address and port
	iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return;
	}

	// Create a SOCKET for connecting to server
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket == INVALID_SOCKET) {
		printf("socket failed with error: %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		return;
	}

	// Setup the TCP listening socket
	iResult = ::bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		printf("bind failed with error: %d\n", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
		return;
	}

	freeaddrinfo(result);

	iResult = ::listen(ListenSocket, SOMAXCONN);
	if (iResult == SOCKET_ERROR) {
		printf("listen failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return;
	}

	cout << "Waiting to connect...  ";
	// Accept a client socket
	ClientSocket = ::accept(ListenSocket, NULL, NULL);
	cout << "Connection successful!";
	if (ClientSocket == INVALID_SOCKET) {
		printf("accept failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return;
	}


	// No longer need server socket
	closesocket(ListenSocket);
}

//Function that initializes the camera, store video capture object in global variable
void initCamera()
{
	cap.open(0, CV_CAP_DSHOW);
	cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));
	cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
	cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);

}

//Function that sends coordinates to the robot and evaluates the robot's response
int sendCoordinatesTCP(float x, float y, float z, float rx, float ry, float rz)
{
	string URmessage = "(" + to_string(x/1000) + ", " + to_string(y / 1000) + ", " + to_string(z / 1000) + ", " + to_string(rx) + ", " + to_string(ry) + ", " + to_string(rz) + ")";
	iSendResult = send(ClientSocket, URmessage.c_str(), URmessage.length(), 0); //Send the message
	if (iSendResult == SOCKET_ERROR) {
		printf("send failed with error: %d\n", WSAGetLastError());
		closesocket(ClientSocket);
		WSACleanup();
		return -1;
	}
	else
	{
		while (1)
		{
			iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
			if (iResult > 0) { //If a message has been received
				if (strcmp(recvbuf, "True") == 0) //If the message is True
				{
					cout << "Message received was True";
					return 1;
				}
				else
				{
					cout << "Message received was not True";
					return -1;
				}
			}
			else if(result<0){
				printf("recv failed with error: %d\n", WSAGetLastError());
				closesocket(ClientSocket);
				WSACleanup();
				return -1;
			}
		}
	}
}

//Function that sends commands to the UR5 robot via the TCP connection in order to move to a preset position
int moveRobot(int position)
{
	float x, y, z, rx, ry, rz;
	x = positionCoordinates[position][0];
	y = positionCoordinates[position][1];
	z = positionCoordinates[position][2];
	rx = positionCoordinates[position][3];
	ry = positionCoordinates[position][4];
	rz = positionCoordinates[position][5];
	if (sendCoordinatesTCP(x, y, z, rx, ry, rz))
	{
		cout << "SUCCESS: Robot moved to preset position " << position << endl;
		return 1;
	}
	else
	{
		cout << "FAILED: Robot moved to preset position " << position << endl;
		return -1;
	}
}

//Function that sends commands to the UR5 robot via the TCP connection in order to move to a position on the rack
int moveRobot(int position, sample &sampleTarget)
{
	Point rackPos = sampleTarget.returnRackPos();
	float x, y, z, rx, ry, rz;
	x = positionCoordinates[position][0]+(ADJ_RACK_X*1000*rackPos.y);
	y = positionCoordinates[position][1]+(0.8*rackPos.y);
	z = positionCoordinates[position][2]+(ADJ_RACK_Z*1000*rackPos.x)+(1*rackPos.y);
	rx = positionCoordinates[position][3];
	ry = positionCoordinates[position][4];
	rz = positionCoordinates[position][5];
	if (sendCoordinatesTCP(x, y, z, rx, ry, rz))
	{
		cout << "SUCCESS: Robot moved to rack point " << position << "at rack position " << rackPos << endl;
		return 1;
	}
	else
	{
		cout << "FAILED: Robot moved to rack point " << position << "at rack position " << rackPos << endl;
		return -1;
	}
}

//Function that sends commands to the UR5 robot via the TCP connection in order to move to a position on the I/O rack
int moveRobot(int position, int ioCapacity)
{
	float x, y, z, rx, ry, rz;
	x = positionCoordinates[position][0];
	y = positionCoordinates[position][1];
	z = positionCoordinates[position][2] + ADJ_IO_Z*1000*(ioCapacity-1);
	rx = positionCoordinates[position][3];
	ry = positionCoordinates[position][4];
	rz = positionCoordinates[position][5];
	if (sendCoordinatesTCP(x, y, z, rx, ry, rz))
	{
		cout << "SUCCESS: Robot moved to I/O rack position " << ioCapacity << endl;
		return 1;
	}
	else
	{
		cout << "FAILED: Robot moved to I/O rack position " << ioCapacity << endl;
		return -1;
	}
}

//Function that reads the curr frame from the video and stores it into a global variable
int runCamera(VideoCapture cap)
{
	Mat frame;
	cap >> frame;
	if (frame.empty())
	{
		cout << "Frame capture did not work!";
		return -1;
	}
	frame.copyTo(currFrame);
	return 1;
}

//Function that takes the current frame and attempts to read the QR code, returns pointer to sample
sample* readQR()
{
	if (arduinoSP.isConnected())
	{
		arduinoSP.writeSerialPort("W", strlen("W"));
		auto input_time = chrono::high_resolution_clock::now();
		sample* samplePointer;
		QRCodeDetector qrDecoder = QRCodeDetector::QRCodeDetector();
		Mat bbox, rectifiedImage;
		for (;;)
		{
			runCamera(cap);
			
			Rect cropRectangle(Point(332, 496), Point(480, 640));
			Mat croppedFrame = currFrame(cropRectangle);
			resize(croppedFrame, croppedFrame, Size(), 2, 2);
			std::string data = qrDecoder.detectAndDecode(croppedFrame, bbox, rectifiedImage);
			if (data.length() > 0)
			{
				std::cout << "Decoded Data : " << data << endl;
				for (int i = 0; i < sampleVector.size(); i++)
				{
					if (sampleVector[i].returnQrCode().compare(data) == 0)
					{
						samplePointer = &sampleVector[i];
						samplePointer->inputTime = input_time;
						arduinoSP.writeSerialPort("S", strlen("S"));
						return samplePointer;
					}
				}
			}
			auto current_time = chrono::high_resolution_clock::now();
			int runTime = chrono::duration_cast<chrono::seconds>(current_time - input_time).count();
			if (runTime >= 15)
			{
				moveRobot(pos_VIS_APP);
				moveRobot(pos_VIS_IN);
				auto input_time = chrono::high_resolution_clock::now();
				runTime = 0;
			}
		}
		
	}
}

//Function that takes pictures of sample and runs evaluating methods against comparator
void visInspection(sample &sampleTarget) 
{
	if (arduinoSP.isConnected())
	{
		int commCase = 0;
		Mat imageWhite, imageUV;
		auto start_time = chrono::high_resolution_clock::now();
		while (1)
		{
			auto current_time = chrono::high_resolution_clock::now();
			int secondsPassed = chrono::duration_cast<chrono::seconds>(current_time - start_time).count();
			//std::cout << "Seconds passed: " << secondsPassed<<endl;
			switch (commCase)
			{
			case 0:
				
					arduinoSP.writeSerialPort("W", strlen("W")); //Turn on white light
					commCase = 1;
					cout << "Case 0";
					   break;
			case 1:
				if (secondsPassed >= 12)
				{
					runCamera(cap);
					currFrame.copyTo(imageWhite);
					arduinoSP.writeSerialPort("S", strlen("S")); //Turn off white light
					commCase = 2;
					cout << "Case 1";
					break;
				}
			case 2:
				if (secondsPassed >= 15)
				{
					arduinoSP.writeSerialPort("U", strlen("U")); //Turn on UV light
					commCase = 3;
					cout << "Case 2";
					break;
				}
			case 3:
				if (secondsPassed >= 22)
				{
					runCamera(cap);
					currFrame.copyTo(imageUV);
					arduinoSP.writeSerialPort("S", strlen("S")); //Turn off UV light
					commCase = 4;
					cout << "Case 3";
					break;
				}
			}
			if (commCase == 4)
				break;
		}
		sampleTarget.evalSample(comparator, imageWhite, imageUV);
		return;
	}
}

//Function that performs the task that it is given
int performTask(task taskCurr)
{
	sample *sampleCurr = taskCurr.sampleTarget;
	switch (taskCurr.action) {
		//Input task
		case task_INPUT: 
			cout << "Starting input task..." << endl;
			moveRobot(pos_IO_LOW_APP,ioCapacity);
			moveRobot(pos_IO_LOW_IN, ioCapacity);
			moveRobot(pos_IO_HIGH_IN, ioCapacity);
			moveRobot(pos_IO_HIGH_APP, ioCapacity);
			ioCapacity--;
			arduinoSP.writeSerialPort("W", strlen("W"));
			moveRobot(pos_VIS_APP);
			moveRobot(pos_VIS_IN);
			sampleCurr = readQR();
			visInspection(*sampleCurr);
			moveRobot(pos_VIS_APP);
			moveRobot(pos_RACK_HIGH_APP, *sampleCurr);
			moveRobot(pos_RACK_HIGH_IN, *sampleCurr);
			moveRobot(pos_RACK_LOW_IN, *sampleCurr);
			moveRobot(pos_RACK_LOW_APP, *sampleCurr);
			cout << "Finished input task..." << endl;
			return 1;
		//Visual inspection
		case task_EVALUATE:
			cout << "Starting evaluating task..." << endl;
			moveRobot(pos_RACK_LOW_APP, *sampleCurr);
			moveRobot(pos_RACK_LOW_IN, *sampleCurr);
			moveRobot(pos_RACK_HIGH_IN, *sampleCurr);
			moveRobot(pos_RACK_HIGH_APP, *sampleCurr);
			arduinoSP.writeSerialPort("W", strlen("W"));
			moveRobot(pos_VIS_APP);
			moveRobot(pos_VIS_IN);
			visInspection(*sampleCurr);
			moveRobot(pos_VIS_APP);
			moveRobot(pos_RACK_HIGH_APP, *sampleCurr);
			moveRobot(pos_RACK_HIGH_IN, *sampleCurr);
			moveRobot(pos_RACK_LOW_IN, *sampleCurr);
			moveRobot(pos_RACK_LOW_APP, *sampleCurr);
			cout << "Finished evaluating task..." << endl;
			return 1;
		//Output task
		case task_OUTPUT:
			cout << "Starting output task..." << endl;
			moveRobot(pos_RACK_LOW_APP, *sampleCurr);
			moveRobot(pos_RACK_LOW_IN, *sampleCurr);
			moveRobot(pos_RACK_HIGH_IN, *sampleCurr);
			moveRobot(pos_RACK_HIGH_APP, *sampleCurr);
			ioCapacity++;
			moveRobot(pos_IO_HIGH_APP, ioCapacity); 
			moveRobot(pos_IO_HIGH_IN, ioCapacity);
			moveRobot(pos_IO_LOW_IN, ioCapacity);
			moveRobot(pos_IO_LOW_APP, ioCapacity);
			cout << "Finished output task..." << endl;
			return 1;
	}
	return -1;
}
 
int main(int argc, char** argv)
{
	initCamera();
	initGlobalWellCoord();
	initGlobalPosCoord();
	initPositionCoord();
	initComparator();
	initTCP();
	vector<int> testTimes;
	testTimes.push_back(2);
	testTimes.push_back(5);
	sample testSample1(Point(0,1),"#0002",testTimes);
	sampleVector.push_back(testSample1);
	sample testSample2(Point(1, 1), "#00001", testTimes);
	sampleVector.push_back(testSample2);
	task testTask1;
	testTask1.action = task_INPUT;
	testTask1.sampleTarget = &testSample2;
	task testTask2; 
	testTask2.action = task_OUTPUT;
	testTask2.sampleTarget = &testSample1;
	task testTask3;
	testTask3.action = task_EVALUATE;
	testTask3.sampleTarget = &testSample1;
	ioCapacity = 1;
	//readQR(); 
	//visInspection(testSample);
		performTask(testTask1);
}
 