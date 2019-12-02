#undef UNICODE
#define WIN32_LEAN_AND_MEAN
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>

using namespace std;
using namespace cv;
#pragma comment (lib, "Ws2_32.lib")
#define DEFAULT_BUFLEN 512
#define DEFAULT_PORT "30002" 

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

//Global coordinates
Point wellCoordinates[46][2];

//Class that holds the data for each individual well along with methods for assessing and comparing wells
class well
{
private:
	int position, intensity[3][9]; //Position on the sample, also used to identify if well is small or large
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
	if (position < 48) //If the well is a large well
	{
		//Point 1
		intensity[0][0] = image.at<Vec3b>(topLeft)[0];
		intensity[1][0] = image.at<Vec3b>(topLeft)[1];
		intensity[2][0] = image.at<Vec3b>(topLeft)[2];
		//Point 2
		intensity[0][1] = image.at<Vec3b>(topLeft.x + 11, topLeft.y)[0];
		intensity[1][1] = image.at<Vec3b>(topLeft.x + 11, topLeft.y)[1];
		intensity[2][1] = image.at<Vec3b>(topLeft.x + 11, topLeft.y)[2];
		//Point 3
		intensity[0][2] = image.at<Vec3b>(topLeft.x + 22, topLeft.y)[0];
		intensity[1][2] = image.at<Vec3b>(topLeft.x + 22, topLeft.y)[1];
		intensity[2][2] = image.at<Vec3b>(topLeft.x + 22, topLeft.y)[2];
		//Point 4
		intensity[0][3] = image.at<Vec3b>(topLeft.x, topLeft.y + 11)[0];
		intensity[1][3] = image.at<Vec3b>(topLeft.x, topLeft.y + 11)[1];
		intensity[2][3] = image.at<Vec3b>(topLeft.x, topLeft.y + 11)[2];
		//Point 5
		intensity[0][4] = image.at<Vec3b>(topLeft.x + 11, topLeft.y + 11)[0];
		intensity[1][4] = image.at<Vec3b>(topLeft.x + 11, topLeft.y + 11)[1];
		intensity[2][4] = image.at<Vec3b>(topLeft.x + 11, topLeft.y + 11)[2];
		//Point 6
		intensity[0][5] = image.at<Vec3b>(topLeft.x + 22, topLeft.y + 11)[0];
		intensity[1][5] = image.at<Vec3b>(topLeft.x + 22, topLeft.y + 11)[1];
		intensity[2][5] = image.at<Vec3b>(topLeft.x + 22, topLeft.y + 11)[2];
		//Point 7
		intensity[0][6] = image.at<Vec3b>(topLeft.x, topLeft.y + 22)[0];
		intensity[1][6] = image.at<Vec3b>(topLeft.x, topLeft.y + 22)[1];
		intensity[2][6] = image.at<Vec3b>(topLeft.x, topLeft.y + 22)[2];
		//Point 8
		intensity[0][7] = image.at<Vec3b>(topLeft.x + 11, topLeft.y + 22)[0];
		intensity[1][7] = image.at<Vec3b>(topLeft.x + 11, topLeft.y + 22)[1];
		intensity[2][7] = image.at<Vec3b>(topLeft.x + 11, topLeft.y + 22)[2];
		//Point 9
		intensity[0][8] = image.at<Vec3b>(bottomRight)[0];
		intensity[1][8] = image.at<Vec3b>(bottomRight)[1];
		intensity[2][8] = image.at<Vec3b>(bottomRight)[2];
	} 
	else //If the well is a small well
	{

	}
}

//Method that records the intensity values of a well for the 3 different channels in multiple points for a Ecoli image
void well::readWellEColi(Mat image)
{
	if (position < 48) //If the well is a large well
	{
		
	}
	else //If the well is a small well
	{

	}
}

//Method that compares the intensity values of the first well using the blue channel values of the second well as a threshold
//returns 1 if the well contains coli, -1 otherwise
int well::evalWellColi(well comparator)
{
	int coliPositives = 0;
	if (position < 48)
	{
		for (int i = 0; i < 9; i++)
			if (intensity[0][i] < comparator.intensity[0][i])
				coliPositives++;
		if (coliPositives >= 5)
		{
			return 1;
		}
		else
			return -1;
	}
	else
	{

	}
}

//Method that compares the intensity values of the first well using the 
int well::evalWellEColi(well comparator)
{
	int EcoliPositives = 0;
	if (position < 48)
	{
		for (int i = 0; i < 9; i++)
			
		if (EcoliPositives >= 5)
		{

			return 1;
		}
		else
			return -1;
	}
	else
	{

	}
}

//Class that holds the data for each individual sample along with methods for testing wells
class sample
{
private:
	well wells[96]; //Vector containing all 96 well objects which the sample contains
	int rackPos, idCode; //Position of the sample on the storage rack
	vector<int> evalTimes; //Vector containing the times at which the sample is to be evaluated relative to the time it first entered the system
public:
	sample(){}
	sample(int position, int id, vector<int> times);
	well returnWell(int i) { return wells[i];}
	void readSample(Mat imageWhite, Mat imageUV);
	void evalSample(sample comparator);
	int computeMPN();
};

//Constructor for the sample class
sample::sample(int position, int id, vector<int> times)
{
	rackPos = position; idCode = id; evalTimes = times;
	for (int i = 0; i < 96; i++)
	{
		Point tl = wellCoordinates[i][0]; Point br = wellCoordinates[i][1];
		wells[i] = well(i, tl, br);
	}
}

//Method that reads the values for each individual well object from an image with white light and an image with UV light
void sample::readSample(Mat imageWhite, Mat imageUV)
{
	for (int i = 0; i < 49; i++)
	{
		wells[i].readWellColi(imageWhite);
		wells[i].readWellEColi(imageUV);
	}
}

//Method that evaluates a sample against the values of another sample (DOES NOT DO ANYTHING AFTER EVALUATING ATM!!!)
void sample::evalSample(sample comparator)
{
	int coliCounter = 0;
	int eColiCounter = 0;
	for (int i = 0; i < 49; i++)
	{
		well comparatorWell = comparator.returnWell(i);
		if (wells[i].evalWellColi(comparatorWell) == 1)
			coliCounter++;
		if (wells[i].evalWellEColi(comparatorWell) == 1)
			eColiCounter++;
	}
}

//Method that computes and returns the most probable number
int computeMPN()
{
	return 0;
}

void initializeTCP()
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

	cout << "test1";
	// Resolve the server address and port
	iResult = getaddrinfo(NULL, DEFAULT_PORT, &hints, &result);
	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		return;
	}

	cout << "test2";
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

	cout << "test3";
	// Accept a client socket
	ClientSocket = ::accept(ListenSocket, NULL, NULL);
	cout << "test4";
	if (ClientSocket == INVALID_SOCKET) {
		printf("accept failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		return;
	}


	// No longer need server socket
	closesocket(ListenSocket);
}

void listenSendTCP()
{
	while (1)
	{
		iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
		cout <<endl <<recvbuf<<endl;


		if (iResult > 0) {
			printf("Bytes received: %d\n", iResult);

			string URmessage = "(" + to_string(6) + ", " + to_string(5) + ", " + to_string(4) + ", " + to_string(3) + ", " + to_string(2) + ", " + to_string(1) + ")";
			cout << URmessage << endl;
			iSendResult = send(ClientSocket, URmessage.c_str(), URmessage.length(), 0);
			if (iSendResult == SOCKET_ERROR) {
				printf("send failed with error: %d\n", WSAGetLastError());
				closesocket(ClientSocket);
				WSACleanup();
				return;
			}
			printf("Bytes sent: %d\n", iSendResult);
			bool movementFinished = false;
			while (!movementFinished)
			{
				iResult = recv(ClientSocket, recvbuf, recvbuflen, 0);
				if (recvbuf[0] == 'F')
				{
					cout << "Movement is finished, program can now resume";
					movementFinished = true;
				}
				else {
					Sleep(1000);
				}
			}
		}
		else if (iResult == 0)
			printf("Connection closing...\n");
		else {
			printf("recv failed with error: %d\n", WSAGetLastError());
			closesocket(ClientSocket);
			WSACleanup();
			return;
		}
	}

}

int main(int argc, char** argv)
{
	initializeTCP();
	listenSendTCP();
	return 0;
}
 