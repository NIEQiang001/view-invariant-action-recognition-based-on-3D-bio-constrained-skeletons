#pragma once

#include <stdafx.h>
#include "myKinect.h"



using namespace std;
using namespace cv;

CBodyRepair::CBodyRepair(CBodyBasics* BodyBasics_) :LocalTstamp(0), OSkeleton_(NULL), size(0)
{
	
	Boneslimit[0] = { -135, -180, 135, 180, -180, 180, 0, 0, -135, -45, 0, 0 }; //angle 2 3 4,Boneslimit[0]
	Boneslimit[1] = { -180, -120, 150, 180, -180, 180, 0, 0, -120, -60, 0, 0 };// angle 3 4 0, Boneslimit[1]
	Boneslimit[2] = { -45, 140, 0, 0, -80, 130, 0, 0, -180, 40, 0, 0 };//angle 6 5 2, Boneslimit[2]
	Boneslimit[3] = { -180, -135, 40, 180, -180, -90, 50, 180, -180, 40, 0, 0 };//angle 12 11 2, Boneslimit[3]
	Boneslimit[4] = { -180, -170, 30, 180, -60, 0, 0, 70, - 100, 60, 0, 0 };//angle7 6 5, Boneslimit[4]
	Boneslimit[5] = { -180, -30, 170, 180, -180, -120, 110, 180, -60, 100, 0, 0 };//angle 13 12 11, Boneslimit[5]
	Boneslimit[6] = { -180, -140, 150, 180, -130, -90, 90, 130, -150, 0, 0, 0 };//angle 8 7 6, Boneslimit[6]
	Boneslimit[7] = { -180, -150, 140, 180, 50, 90, -90, -50, -150, 0, 0, 0 };//angle 14 13 12, Boneslimit[7]
	Boneslimit[8] = { -180, -170, 170, 180, -90, -90, 90, 90, -120, 10, 0, 0 };//angle 9 8 7, Boneslimit[8]
	Boneslimit[9] = { -180, -170, 170, 180, -90, -90, 90, 90, -120, 10, 0, 0 };//angle 15 14 13, Boneslimit[9]
	Boneslimit[10] = { -100, -70, 0, 0, -180, -160, 160, 180, -180, 180, 0, 0 };//angle 5 3 4, Boneslimit[10]
	Boneslimit[11] = { -180, -90, 90, 180, -130, -50, 50, 130, -100, 0, 0, 0 };//angle 10 7 6, Boneslimit[11]
	Boneslimit[12] = { -180, -90, 90, 180, -130, -50, 50, 130, -100, 0, 0, 0 };//angle 16 13 12, Boneslimit[12]
	Boneslimit[13] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//angle 17 0, Boneslimit[13]
	Boneslimit[14] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };//angle 21 0, Boneslimit[14]
	Boneslimit[15] = { -135, -60, 0, 0, -180, -75, 60, 180, -180, -135, 120, 180 };//angle 18 17 0, Boneslimit[15]
	Boneslimit[16] = { 60, 135, 0, 0, -105, 0, 0, 60, -180, -135, 120, 180 };//angle 22 21 0, Boneslimit[16]
	Boneslimit[17] = { -180, -30, 170, 180, 140, 180, -180, -130, 0, 0, 0, 0 };//angle 19 18 17, Boneslimit[17]
	Boneslimit[18] = { -180, -170, 30, 180, -50, 40, 0, 0, 0, 0, 0, 0 };//angle 23 22 21, Boneslimit[18]
	Boneslimit[19] = { 145, 180, -180, -150, -125, -60, 0, 0, -45, 50, 0, 0 };//angle 20 19 18, Boneslimit[19]
	Boneslimit[20] = { -180, -145, 150, 180, -120, -55, 0, 0, -45, 50, 0, 0 };//angle 24 23 22, Boneslimit[20]
	Boneslimit[21] = { -120, -180, 120, 180, -180, 180, 0, 0, -120, -30, 0, 0 };//angle 1 2 3, Boneslimit[21]
	Boneslimit[22] = { 70, 100, 0, 0, -20, 20, 0, 0, -180, 180, 0, 0 };//angle 11 3 4, Boneslimit[22]
	BodyBasics = BodyBasics_;
	

}

CBodyRepair::~CBodyRepair()
{
	//delete Boneslimit;
}


/// Initializes the default Kinect sensor
HRESULT CBodyBasics::InitializeDefaultSensor()
{
	//用于判断每次读取操作的成功与否
	HRESULT hr;

	//搜索kinect
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr)){
		return hr;
	}

	//找到kinect设备
	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the body reader
		IBodyFrameSource* pBodyFrameSource = NULL;//读取骨架
		IDepthFrameSource* pDepthFrameSource = NULL;//读取深度信息
		IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;//读取背景二值图
		IColorFrameSource* pColorFrameSource = NULL;//read RGB image
		
		//打开kinect
		hr = m_pKinectSensor->Open();

		//coordinatemapper
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		//bodyframe
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		//depth frame
		if (SUCCEEDED(hr)){
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr)){
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}

		//body index frame
		if (SUCCEEDED(hr)){
			hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
		}

		if (SUCCEEDED(hr)){
			hr = pBodyIndexFrameSource->OpenReader(&m_pBodyIndexFrameReader);
		}
		//// color frame
		if (SUCCEEDED(hr)){
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}

		if (SUCCEEDED(hr)){
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}

		SafeRelease(pBodyFrameSource);
		SafeRelease(pDepthFrameSource);
		SafeRelease(pBodyIndexFrameSource);
		SafeRelease(pColorFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		std::cout << "Kinect initialization failed!" << std::endl;
		return E_FAIL;
	}

	//skeletonImg,用于画骨架、背景二值图的MAT
	skeletonImg.create(cDepthHeight, cDepthWidth, CV_8UC3);
	skeletonImg.setTo(0);
	RskeletonImg.create(cDepthHeight, cDepthWidth, CV_8UC3);
	RskeletonImg.setTo(0);

	//color image
	colorframe.create(cColorHeight, cColorWidth, CV_8UC4);
	colorframe.setTo(0);

	//depthImg,用于画深度信息的MAT
	depthImg.create(cDepthHeight, cDepthWidth, CV_8UC1);
	depthImg.setTo(0);
	filted_depthImg.create(cDepthHeight, cDepthWidth, CV_8UC1);
	filted_depthImg.setTo(0);
	pBodyPoints = new CameraSpacePoint[cDepthWidth*cDepthHeight];
	return hr;
}


/// Main processing function
void CBodyBasics::Update()
{
	//每次先清空skeletonImg
	skeletonImg.setTo(0);
	//colorframe.setTo(0);
	//如果丢失了kinect，则不继续操作
	if (!m_pBodyFrameReader)
	{
		return;
	}

	IBodyFrame* pBodyFrame = NULL;//骨架信息
	IDepthFrame* pDepthFrame = NULL;//深度信息
	IBodyIndexFrame* pBodyIndexFrame = NULL;//背景二值图
	IColorFrame* pColorFrame = NULL;//color frame pointer

	//记录每次操作的成功与否
	HRESULT hr = S_OK;

	//---------------------------------------获取背景二值图并显示---------------------------------
	if (SUCCEEDED(hr)){
		hr = m_pBodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);//获得背景二值图信息
			
	}
	if (SUCCEEDED(hr)){
		bodyIndexArray = new BYTE[cDepthHeight * cDepthWidth];//背景二值图是8 bit uchar，有人是黑色，没人是白色
		pBodyIndexFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, bodyIndexArray);
		
		//把背景二值图画到MAT里
		uchar* skeletonData = (uchar*)skeletonImg.data;
		RowMin = 0; RowMax = 0; ColumMin = 0; ColumMax = 0;
		for (int j = 0; j < cDepthHeight * cDepthWidth; ++j){
			*skeletonData = bodyIndexArray[j] > 6 ? 0 : 255; ++skeletonData;
			*skeletonData = bodyIndexArray[j] > 6 ? 0 : 255; ++skeletonData;
			*skeletonData = bodyIndexArray[j] > 6 ? 0 : 255; ++skeletonData;
			//cout << int(bodyIndexArray[j]);
			
			if (bodyIndexArray[j] < 6)
			{
				int row = 0, column = 0;
				row = int(j / cDepthWidth);
				column = j % cDepthWidth;
				RowMin = RowMin == 0 ? row : RowMin>row ? row : RowMin;
				RowMax = RowMax < row ? row : RowMax;
				ColumMin = ColumMin == 0 ? column : ColumMin > column ? column : ColumMin;
				ColumMax = ColumMax < column ? column : ColumMax;
			}
		}
		//cout << RowMin << " " << ColumMin << " " << RowMax << " " << ColumMax << endl;
		cv::rectangle(skeletonImg, cvPoint(ColumMin, RowMin), cvPoint(ColumMax, RowMax ), (0, 128, 128), 2, 8, 0);
		//delete[] bodyIndexArray;
		skeletonImg.copyTo(RskeletonImg);
		cv::imshow("skeleton image", skeletonImg);
		waitKey(5);
	}
	SafeRelease(pBodyIndexFrame);//必须要释放，否则之后无法获得新的frame数据


	////-----------------------获取深度数据并显示--------------------------
	if (SUCCEEDED(hr)){
		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//获得深度数据
		if (SUCCEEDED(hr)){
			
			IFrameDescription* pFrameDescription = NULL;
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
			}
			if (SUCCEEDED(hr))
			{
				hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			}
			if (SUCCEEDED(hr)){
			//nDepthMaxDistance = USHRT_MAX;
				hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
			}
			depthArray = new UINT16[cDepthHeight * cDepthWidth];//深度数据是16位unsigned int
			pDepthFrame->CopyFrameDataToArray(cDepthHeight * cDepthWidth, depthArray);

			//store depth data into MAT
			uchar* depthData = (uchar*)depthImg.data;
			for (int j = 0; j < cDepthHeight * cDepthWidth; ++j){
				//*depthData = ((depthArray[j] >= nDepthMinReliableDistance) && (depthArray[j] <= nDepthMaxDistance) ? (depthArray[j] % 256) : 0);
				*depthData = ((depthArray[j] >= nDepthMinReliableDistance) && (depthArray[j] <= nDepthMaxDistance) ? (256*(depthArray[j]-nDepthMinReliableDistance)/(nDepthMaxDistance-nDepthMinReliableDistance)) : 0);
				++depthData;
			}
			cv::bilateralFilter(depthImg, filted_depthImg, 5, 50, 50);
			m_pCoordinateMapper->MapDepthFrameToCameraSpace(cDepthHeight * cDepthWidth, depthArray, cDepthHeight * cDepthWidth, pBodyPoints);
			//delete[] depthArray;
		}
	}
	SafeRelease(pDepthFrame);//必须要释放，否则之后无法获得新的frame数据
	//imshow("depthImg", depthImg);
	
	//cv::waitKey(5);

	//-------------------------obtain color frame and show it--------------
	if (SUCCEEDED(hr)){
		hr = m_pColorFrameReader->AcquireLatestFrame(&pColorFrame);
		if (SUCCEEDED(hr)){
			UINT cbuffersize = cColorHeight*cColorWidth * 4 * sizeof(BYTE);
			pColorFrame->CopyConvertedFrameDataToArray(cbuffersize, colorframe.data, ColorImageFormat_Bgra);
		}
		DepthSpacePoint lefttop, rightdown;
		lefttop.X = ColumMin; lefttop.Y = RowMin;
		rightdown.X = ColumMax; rightdown.Y = RowMax;
		//UINT16 LT_value=(RowMin*512+ColumMin)
		int LT_index = RowMin * 512 + ColumMin, RD_index = RowMax * 512 + ColumMax;
		ColorSpacePoint clefttop, crightdown;
		m_pCoordinateMapper->MapDepthPointToColorSpace(lefttop,depthArray[LT_index], &clefttop);
		m_pCoordinateMapper->MapDepthPointToColorSpace(rightdown, depthArray[RD_index], &crightdown);
		cout << clefttop.X << " " << clefttop.Y << endl;
		cv::rectangle(colorframe, cvPoint(clefttop.X, clefttop.Y), cvPoint(crightdown.X, crightdown.Y ), (0, 128, 128), 2, 8, 0);
		imshow("colorImg", colorframe);
		waitKey(5);
	}
	SafeRelease(pColorFrame);
	//-----------------------------obtain skeleton and show it----------------------------
	if (SUCCEEDED(hr)){
		hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);//获取骨架信息
	}
	if (SUCCEEDED(hr))
	{
		IBody* ppBodies[BODY_COUNT] = { 0 };//每一个IBody可以追踪一个人，总共可以追踪六个人
		IBody* skeleton;
		if (SUCCEEDED(hr))
		{
			//把kinect追踪到的人的信息，分别存到每一个IBody中
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			TIMESPAN LTstamp;
			hr = pBodyFrame->get_RelativeTime(&LTstamp);
			if (SUCCEEDED(hr)) hr = pBodyFrame->get_FloorClipPlane(&floorPlane);
			DeltT = float(LTstamp - Tstamp) / 10000000;
			FPS = float((int)(100 / DeltT)/100);//frames per second
			//cout << FPS << endl;
			Tstamp = LTstamp;
			
			for (int i = 0; i < 6; ++i)
			{
				BOOLEAN isTracked = false;
				ppBodies[i]->get_IsTracked(&isTracked);
				if (isTracked)
				{
					skeleton = ppBodies[i];					
					break;
				}
				else skeleton = NULL;
			}
			
		}
		
	//----------------------------------------------------------------------------------	
			if (Initialized_Skeleton_flag < 3)
			{
				//对每一个IBody，initialize their bones' length
				InitializeSkeleton(skeleton);

			}
			if (Initialized_Skeleton_flag == 3)
			{
				cout << "This body is initialized!" << endl;
				cout << "Length of Head_Neck is: " << Length_Head_Neck << endl;
				cout << "Length of Length_Neck_SpineShoulder is: " << Length_Neck_SpineShoulder << endl;
				cout << "Length of Length_SpineMid_SpineBase is: " << Length_SpineMid_SpineBase << endl;
				cout << "Length of Length_SpineShoulder_SpineMid is: " << Length_SpineShoulder_SpineMid << endl;
				cout << "Length of Ankle_Foot is: " << Length_Ankle_Foot << endl;
				cout << "Length of Elbow_Wrist is: " << Length_Elbow_Wrist << endl;
				cout << "Length of Hand_Handtip is: " << Length_Hand_Handtip << endl;
				cout << "Length of Hip_Knee is: " << Length_Hip_Knee << endl;
				cout << "Length of Knee_Ankle is: " << Length_Knee_Ankle << endl;
				cout << "Length of Shoulder_Elbow is: " << Length_Shoulder_Elbow << endl;
				cout << "Length of SpineBase_Hip is: " << Length_SpineBase_Hip << endl;
				cout << "Length of Wrist_Hand is: " << Length_Wrist_Hand << endl;
				cout << "Length of Wrist_Thumb is: " << Length_Wrist_Thumb << endl;
				Initialized_Skeleton_flag++;
				height = float((int)(100 * (Length_Head_Neck + Length_Neck_SpineShoulder + Length_SpineShoulder_SpineMid + Length_SpineMid_SpineBase + Length_Hip_Knee + Length_Knee_Ankle))) / 100;
				
				
			}
			if ( skeleton && Initialized_Skeleton_flag>=3)
			{
				HRESULT hrr;
				Joint joints[JointType_Count];//存储关节点类
				hrr = skeleton->GetJoints(_countof(joints), joints);
				DepthSpacePoint depthSpacePosition ;
				for (int i = 0; i < JointType_Count; i++)
				{
						OSkeleton.push_back(joints[i]);
						m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[i].Position, &depthSpacePosition);
						OSkeletonD.push_back(depthSpacePosition);
				}
								
				//
			}
			if (OSkeleton.size() > OSize)
			{
				OSize = OSkeleton.size();
				OSnum++;
			}
				
	//---------------------------------------------------------------------------------------	
		if (SUCCEEDED(hr))
		{
						
			if (Initialized_Skeleton_flag >= 3)
			{
 				if (height < 1.3) {
					//CvFont font ;
					//cvInitFont(&font, CV_FONT_HERSHEY_TRIPLEX, 1, 1, 0, 3, 8);
					stringstream s,ss;
					s << height;
					string num = s.str();
					string str2 = "Height: " + num + "m";
					ss << FPS;
					string str3 = ss.str() + "FPS";
					putText(skeletonImg, "Child !", Point(10, 20), 5, 1, Scalar(0, 255, 255),2);
					putText(skeletonImg, str2, Point(10, 50), 5, 1, Scalar(0, 0xff, 0xff), 1.5);
					putText(skeletonImg, str3, Point(10, 80), 5, 1, Scalar(0, 0xff, 0xff), 1.5);
				}
				else
				{
					stringstream s;
					s << height;
					string num = s.str();
					string str2 = "Height: " + num + "m";
					putText(skeletonImg, "Adult ", Point(20, 20), CV_FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 255, 0), 2);
					putText(skeletonImg, str2, Point(50, 50), CV_FONT_HERSHEY_TRIPLEX, 1, Scalar(0, 0xff, 0xff), 2);
				}
			}

			//对每一个IBody，我们找到他的骨架信息，并且画出来
			DrawBody(BODY_COUNT, ppBodies);
			
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);//释放所有
		}
		//SafeRelease(skeleton);
		//Tstamp++;
	}
	SafeRelease(pBodyFrame);//必须要释放，否则之后无法获得新的frame数据
	
}

/// Handle new body data
void CBodyBasics::DrawBody(int nBodyCount, IBody** ppBodies)
{
	//记录操作结果是否成功
	HRESULT hr;

	//对于每一个IBody
	for (int i = 0; i < nBodyCount; ++i)
	{
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			BOOLEAN isrestricted = true;
			hr = pBody->get_IsTracked(&bTracked);

			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count];//存储关节点类
				HandState leftHandState = HandState_Unknown;//左手状态
				HandState rightHandState = HandState_Unknown;//右手状态
				/*pBody->get_IsRestricted(&isrestricted);
				if (isrestricted) cout << "true" << endl;
				else cout << "false" << endl;*/

				//获取左右手状态
				pBody->get_HandLeftState(&leftHandState);
				pBody->get_HandRightState(&rightHandState);

				//存储深度坐标系中的关节点位置
				DepthSpacePoint *depthSpacePosition = new DepthSpacePoint[_countof(joints)];

				//获得关节点类
				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					for (int j = 0; j < _countof(joints); ++j)
					{
						//将关节点坐标从摄像机坐标系（-1~1）转到深度坐标系（424*512）
						m_pCoordinateMapper->MapCameraPointToDepthSpace(joints[j].Position, &depthSpacePosition[j]);
					}
					
					//------------------------hand state left-------------------------------
					/*DrawHandState(depthSpacePosition[JointType_HandLeft], leftHandState);
					DrawHandState(depthSpacePosition[JointType_HandRight], rightHandState);*/

					//---------------------------body-------------------------------
					DrawBone(joints, depthSpacePosition, JointType_Head, JointType_Neck, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_Neck, JointType_SpineShoulder, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_SpineMid, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_SpineMid, JointType_SpineBase, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_SpineShoulder, JointType_ShoulderLeft, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_SpineBase, JointType_HipLeft, skeletonImg);

					// -----------------------Right Arm ------------------------------------ 
					DrawBone(joints, depthSpacePosition, JointType_ShoulderRight, JointType_ElbowRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_ElbowRight, JointType_WristRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_HandRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_HandRight, JointType_HandTipRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_WristRight, JointType_ThumbRight, skeletonImg);

					//----------------------------------- Left Arm--------------------------
					DrawBone(joints, depthSpacePosition, JointType_ShoulderLeft, JointType_ElbowLeft, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_ElbowLeft, JointType_WristLeft, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_HandLeft, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_HandLeft, JointType_HandTipLeft, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_WristLeft, JointType_ThumbLeft, skeletonImg);

					// ----------------------------------Right Leg--------------------------------
					DrawBone(joints, depthSpacePosition, JointType_HipRight, JointType_KneeRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_KneeRight, JointType_AnkleRight, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_AnkleRight, JointType_FootRight, skeletonImg);

					// -----------------------------------Left Leg---------------------------------
					DrawBone(joints, depthSpacePosition, JointType_HipLeft, JointType_KneeLeft, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_KneeLeft, JointType_AnkleLeft, skeletonImg);
					DrawBone(joints, depthSpacePosition, JointType_AnkleLeft, JointType_FootLeft, skeletonImg);
				//---------------------------------draw joints---------------------------------------
					for (i = 0; i < JointType_Count; ++i)
					{
						if (joints[i].TrackingState == TrackingState_Inferred)
						{
							circle(skeletonImg, cvPoint(depthSpacePosition[i].X, depthSpacePosition[i].Y), 4, (255, 0, 0), 0.8);
						}
						else if (joints[i].TrackingState == TrackingState_Tracked)
						{
							circle(skeletonImg, cvPoint(depthSpacePosition[i].X, depthSpacePosition[i].Y), 4, (0, 255, 0), 1.5);
						}
					}
					circle(skeletonImg, cvPoint(depthSpacePosition[12].X, depthSpacePosition[12].Y), 10, (0, 255, 255), 2);
					//rectangle(filted_depthImg, cvPoint(R1.X, R1.Y), cvPoint(R2.X, R2.Y), (0, 128, 128), 1, 8, 0);
				}
				delete[] depthSpacePosition;
				
			}
		}
	}
	//cv::imshow("filted depthImg", filted_depthImg);
	//cv::imshow("skeletonImg", skeletonImg);
	cv::waitKey(5);
}




//画手的状态
void CBodyBasics::DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState)
{
	//给不同的手势分配不同颜色
	CvScalar color;
	switch (handState){
	case HandState_Open:
		color = cvScalar(255, 0, 0);
		break;
	case HandState_Closed:
		color = cvScalar(0, 255, 0);
		break;
	case HandState_Lasso:
		color = cvScalar(0, 0, 255);
		break;
	default://如果没有确定的手势，就不要画
		return;
	}

	circle(skeletonImg,
		cvPoint(depthSpacePosition.X, depthSpacePosition.Y),
		20, color, -1);
}


/// Draws one bone of a body (joint to joint)
void CBodyBasics::DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1,cv::Mat img)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If we can't find either of these joints, exit
	if ((joint0State == TrackingState_NotTracked) || (joint1State == TrackingState_NotTracked))
	{
		return;
	}

	// Don't draw if both points are inferred
	if ((joint0State == TrackingState_Inferred) && (joint1State == TrackingState_Inferred))
	{
		return;
	}

	CvPoint p1 = cvPoint(depthSpacePosition[joint0].X, depthSpacePosition[joint0].Y),
		p2 = cvPoint(depthSpacePosition[joint1].X, depthSpacePosition[joint1].Y);

	// We assume all drawn bones are inferred unless BOTH joints are tracked
	if ((joint0State == TrackingState_Tracked) && (joint1State == TrackingState_Tracked))
	{
		//tracked skeleton，use green line
	    line(img, p1, p2, cvScalar(0, 255, 0),4);
	}
	else
	{
		//inferred skeleton，use red line
		line(img, p1, p2, cvScalar(0, 0, 255),1);
	}
}

//overload DrawBone
void CBodyBasics::DrawBone(const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1, cv::Mat img)
{
		
	// We assume all drawn bones are inferred unless BOTH joints are tracked
	CvPoint p1 = cvPoint(depthSpacePosition[joint0].X, depthSpacePosition[joint0].Y),
		p2 = cvPoint(depthSpacePosition[joint1].X, depthSpacePosition[joint1].Y);
		//tracked skeleton，use green line
		line(img, p1, p2, cvScalar(0, 0, 255), 4);
	
}

/// Constructor
CBodyBasics::CBodyBasics() :
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pBodyFrameReader(NULL),
Initialized_Skeleton_flag(0),
Tstamp(0), OSize(0), OSnum(0), nDepthMinReliableDistance(0), nDepthMaxDistance(0),
floorPlane({ 0, 0, 0, 0 }), depthArray(NULL)
{
	Length_Head_Neck = 0;
	Length_Neck_SpineShoulder = 0;
	Length_SpineShoulder_SpineMid = 0;
	Length_SpineMid_SpineBase = 0;
	Length_SpineShoulder_Shoulder = 0;
	Length_SpineBase_Hip = 0;
	Length_Shoulder_Elbow = 0;
	Length_Elbow_Wrist = 0;
	Length_Wrist_Hand = 0;
	Length_Hand_Handtip = 0;
	Length_Wrist_Thumb = 0;
	Length_Hip_Knee = 0;
	Length_Knee_Ankle = 0;
	Length_Ankle_Foot = 0;
}

/// Destructor
CBodyBasics::~CBodyBasics()
{
	SafeRelease(m_pBodyFrameReader);
	SafeRelease(m_pCoordinateMapper);

	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}
	SafeRelease(m_pKinectSensor);
}

//initialize skeleton Data
void CBodyBasics::InitializeSkeleton( IBody* skeleton)
{
	HRESULT hrr;
	
	//for every body tracked, using the update data to initialize their bones 
	if (skeleton)
		{
			
			Joint joints[JointType_Count];//存储关节点类
			hrr = skeleton->GetJoints(_countof(joints), joints);
			int TrackingCount = 0;
			for (int i = 0; i < JointType_Count; i++)
			{
				if (joints[i].TrackingState == TrackingState_Tracked) TrackingCount++;
			}
			if (TrackingCount < 20) return;
			if (SUCCEEDED(hrr)  && Initialized_Skeleton_flag<3)
			{   
				int l = 0;//count the number of tracked body bones
				int j = 0;//count the number of tracked arm bones
				int k = 0;//count the number of tracked leg bones
				
				
				if (SUCCEEDED(hrr))
				{
					// -----------------initialize length of Body bones--------------------------------
					float a[8] = { 0, 0, 0, 0, 0, 0, 0, 0};
					a[0] = TrackingJointState(joints, JointType_Head, JointType_Neck);
					a[1] = TrackingJointState(joints, JointType_Neck, JointType_SpineShoulder);
					a[2] = TrackingJointState(joints, JointType_SpineShoulder, JointType_SpineMid);
					a[3] = TrackingJointState(joints, JointType_SpineMid, JointType_SpineBase);
					a[4] = TrackingJointState(joints, JointType_SpineShoulder, JointType_ShoulderRight);
					a[5] = TrackingJointState(joints, JointType_SpineShoulder, JointType_ShoulderLeft);
					a[6] = TrackingJointState(joints, JointType_SpineBase, JointType_HipLeft);
					a[7] = TrackingJointState(joints, JointType_SpineBase, JointType_HipRight);
					if (a[0])
					{ 
						l++;
						Length_Head_Neck = Length_Head_Neck == 0 ? a[0] : (Length_Head_Neck + a[0]) / 2;
						//cout << "the length of bone between head and neck is:" << Length_Head_Neck<<endl;
					}
					if (a[1])
					{
						l++;
						Length_Neck_SpineShoulder = Length_Neck_SpineShoulder == 0 ? a[1] : (Length_Neck_SpineShoulder + a[1]) / 2;
						
					};
					if (a[2])
					{
						l++;
						Length_SpineShoulder_SpineMid = Length_SpineShoulder_SpineMid == 0 ? a[2] : (Length_SpineShoulder_SpineMid + a[2]) / 2;
						
					}
					if (a[3])
					{
						l++;
						Length_SpineMid_SpineBase = Length_SpineMid_SpineBase == 0 ? a[3] : (Length_SpineMid_SpineBase + a[3]) / 2;
						
					}
					
					if (a[4] || a[5])
					{
						l++;
						float len;
						len = a[4] == 0 ? a[5] : a[5] == 0 ? a[4] : (a[4] + a[5]) / 2;
						// make sure SpineShoulder_Shoulder only be tracked once, and if it has been tracked in left then use the averange length as bone length
						Length_SpineShoulder_Shoulder = Length_SpineShoulder_Shoulder == 0 ? len : (Length_SpineShoulder_Shoulder + len) / 2;
					}
					
					if (a[6] || a[7])
					{
						l++;
						float len;						
						len = a[6] == 0 ? a[7] : a[7] == 0 ? a[6] : (a[6] + a[7]) / 2;
						Length_SpineBase_Hip = Length_SpineBase_Hip == 0 ? len : (Length_SpineBase_Hip + len) / 2;
					}
					

					//---------------------Right and left Arm------------------------------------------------
					if (TrackingJointState(joints, JointType_ShoulderRight, JointType_ElbowRight) || TrackingJointState(joints, JointType_ShoulderLeft, JointType_ElbowLeft)){

						j++;
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_ShoulderRight, JointType_ElbowRight);
						len2 = TrackingJointState(joints, JointType_ShoulderLeft, JointType_ElbowLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						if (Length_Shoulder_Elbow == 0)		Length_Shoulder_Elbow = len;
						
						else Length_Shoulder_Elbow =(Length_Shoulder_Elbow+len)/2 ;
						//cout << "The length of Shoulder_Elbow is: " << Length_Shoulder_Elbow<<endl;
					}
					if (TrackingJointState(joints, JointType_ElbowRight, JointType_WristRight) || TrackingJointState(joints, JointType_ElbowLeft, JointType_WristLeft)){
						
						j++;
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_ElbowRight, JointType_WristRight);
						len2 = TrackingJointState(joints, JointType_ElbowLeft, JointType_WristLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						if (Length_Elbow_Wrist == 0){
							
							Length_Elbow_Wrist = len;
						}
						else Length_Elbow_Wrist = (Length_Elbow_Wrist + len) / 2;
						//cout << "The length of Elbow_Wrist is: " << Length_Elbow_Wrist << endl;
					}
					if (TrackingJointState(joints, JointType_WristRight, JointType_HandRight) || TrackingJointState(joints, JointType_WristLeft, JointType_HandLeft)){
						j++;
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_WristRight, JointType_HandRight);
						len2 = TrackingJointState(joints,JointType_WristLeft, JointType_HandLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						
						if (Length_Wrist_Hand == 0){
							
							Length_Wrist_Hand = len;
						}
						else Length_Wrist_Hand = (Length_Wrist_Hand + len) / 2;
						//cout << "The length of Wrist_Hand is: " << Length_Wrist_Hand << endl;
					}
					if (TrackingJointState(joints, JointType_HandRight, JointType_HandTipRight) || TrackingJointState(joints, JointType_HandLeft, JointType_HandTipLeft)){
						
						j++;
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_HandRight, JointType_HandTipRight);
						len2 = TrackingJointState(joints, JointType_HandLeft, JointType_HandTipLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						if (Length_Hand_Handtip == 0){
							
							Length_Hand_Handtip = len;
						}
						else Length_Hand_Handtip = (Length_Hand_Handtip + len) / 2;
						//cout << "The length of Hand_Handtip is: " << Length_Hand_Handtip << endl;
					}
					if (TrackingJointState(joints, JointType_WristRight, JointType_ThumbRight) || TrackingJointState(joints, JointType_WristLeft, JointType_ThumbLeft)){
						
						
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_WristRight, JointType_ThumbRight);
						len2 = TrackingJointState(joints, JointType_WristLeft, JointType_ThumbLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						if (Length_Wrist_Thumb == 0){
							
							Length_Wrist_Thumb = len;
						}
						else Length_Wrist_Thumb = (Length_Wrist_Thumb + len) / 2;
						//cout << "The length of Wrist_Thumb is: " << Length_Wrist_Thumb << endl;
					}
					
					

					//---------------------Right & Left Leg-------------------------------------------------
					if (TrackingJointState(joints, JointType_HipRight, JointType_KneeRight) || TrackingJointState(joints, JointType_HipLeft, JointType_KneeLeft)){
						
						k++;
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_HipRight, JointType_KneeRight);
						len2 = TrackingJointState(joints, JointType_HipLeft, JointType_KneeLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						if (Length_Hip_Knee == 0){
							
							Length_Hip_Knee = len;
						}
						else Length_Hip_Knee = (Length_Hip_Knee + len) / 2;
					}
					if (TrackingJointState(joints, JointType_KneeRight, JointType_AnkleRight) || TrackingJointState(joints, JointType_KneeLeft, JointType_AnkleLeft)){
						
						k++;
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_KneeRight, JointType_AnkleRight);
						len2 = TrackingJointState(joints, JointType_KneeLeft, JointType_AnkleLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						if (Length_Knee_Ankle == 0){
									Length_Knee_Ankle = len;
						}
						else Length_Knee_Ankle = (Length_Knee_Ankle + len) / 2;
					}
					if (TrackingJointState(joints, JointType_AnkleRight, JointType_FootRight) || TrackingJointState(joints, JointType_AnkleLeft, JointType_FootLeft)){
						float len1, len2, len;
						len1 = TrackingJointState(joints, JointType_AnkleRight, JointType_FootRight);
						len2 = TrackingJointState(joints, JointType_AnkleLeft, JointType_FootLeft);
						len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
						if (Length_Ankle_Foot == 0){
							
							Length_Ankle_Foot = len;
						}
						else Length_Ankle_Foot = (Length_Ankle_Foot + len) / 2;
					}

					
				}
				cout << l << " " << k << " " << j<<endl;
				if (l >=6 && j>=4 && k>=2)
				{
					//cout << "The bones's length of body "<<0<<" is initialized"<<endl;
					
					Initialized_Skeleton_flag++;
				}
				
				return;
			}//start to calculate length
		}
		
		return;
}

float CBodyBasics::TrackingJointState(const Joint* pJoints, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;
	
	// If both joints are NotTracked, exit
	if ((joint0State != TrackingState_Tracked) || (joint1State != TrackingState_Tracked))
	{
		return 0;
	}

		
	//record joint0's position
	cv::Point3f p0 = Point3f(pJoints[joint0].Position.X, pJoints[joint0].Position.Y, pJoints[joint0].Position.Z);
	//cout << p0.x << "Y" << p0.y << "Z" << p0.z << "joint"<<joint0<< endl;
	//cout << pJoints[joint0].Position.X<<"joint"<<endl;
	//record joint1's position
	cv::Point3f p1 = Point3f(pJoints[joint1].Position.X, pJoints[joint1].Position.Y, pJoints[joint1].Position.Z);
	//cout << p1.x << "Y" << p1.y << "Z" << p1.z << "joint" << joint1 << endl;
	// We assume all drawn bones are inferred unless BOTH joints are tracked
	
		float Jointlength = 0;
		Jointlength = sqrt((p0.x - p1.x)*(p0.x - p1.x) + (p0.y - p1.y)* (p0.y - p1.y) + (p0.z - p1.z)*(p0.z - p1.z));
		return Jointlength;
	
	
}



//Judging if the bones move out of its normal range
//only judge the inferred joints to see if it is out of the limit range
bool CBodyRepair::OrientationJudging(const Joint joint, Joint* Jpointer)
{   
	if (joint.JointType == JointType_SpineBase || joint.JointType == JointType_SpineMid || joint.JointType == JointType_HipLeft || joint.JointType == JointType_HipRight)
	{
		return true;
	}
	
	//according to the joint type, judging if its position locate in the right range
	double JointOrientation[3] = { 0, 0, 0 };
	
	CBodyMotionLimits *limits=nullptr ;
	double errTolerance = 5;
	errRecord[0] = 0;
	errRecord[1] = 0;
	errRecord[2] = 0;
	JointOrientationCal(joint, Jpointer, JointOrientation);
	limits=LookupLimits(joint.JointType);
	
	//check the angle a
	if ((((limits->angle_a_inf1 - errTolerance) < JointOrientation[0]) && (JointOrientation[0] < (limits->angle_a_sup1 + errTolerance))) || (((limits->angle_a_inf2 - errTolerance) < JointOrientation[0]) && (JointOrientation[0] < (limits->angle_a_sup2 + errTolerance))))
		errRecord[0] = 1;
	if ((((limits->angle_b_inf1 - errTolerance) < JointOrientation[1]) && (JointOrientation[1] < (limits->angle_b_sup1 + errTolerance))) || (((limits->angle_b_inf2 - errTolerance) < JointOrientation[1]) && (JointOrientation[1] < (limits->angle_b_sup2 + errTolerance))))
		errRecord[1] = 1;
	if ((((limits->angle_r_inf1 - errTolerance) < JointOrientation[2]) && (JointOrientation[2] < (limits->angle_r_sup1 + errTolerance))) || (((limits->angle_r_inf2 - errTolerance) < JointOrientation[2]) && (JointOrientation[2] < (limits->angle_r_sup2 + errTolerance))))
		errRecord[2] = 1;
	if ((errRecord[0] + errRecord[1] + errRecord[2]) != 3)
	{
		
		//cout << "a:" << JointOrientation[0] << "b:" << JointOrientation[1] << "r:" << JointOrientation[2] << endl << "JointType:" << joint.JointType << " is wrong" << endl;
		return true;
	}
	
	else return false;
}
//Calculate the kinematic information of each joint
bool CBodyRepair::KinematicCal()
{
	
	OSkeleton_ = &BodyBasics->OSkeleton;
	//if (!globalTRec) globalTRec = BodyBasics->Tstamp;
	//cout << LocalTstamp <<endl;
	if (BodyBasics->OSnum>LocalTstamp)
	{
		LocalTstamp = BodyBasics->OSnum;
		int Jstep = 25;
		if (LocalTstamp == 1)
		{
			for (int i = 0; i < JointType_Count; i++)
			{
				cv::Point3f p = Point3f(OSkeleton_->at(0 + i).Position.X, OSkeleton_->at(0 + i).Position.Y, OSkeleton_->at(0 + i).Position.Z);
				nextSkeleton.push_back(p);
			}
			//globalTRec = BodyBasics->Tstamp;
			return true;
		}
		if (LocalTstamp == 2)
		{
			currSkeleton.swap(nextSkeleton);
			nextSkeleton.clear();
			//float deltaT = float(BodyBasics->Tstamp - globalTRec)/10000000;
			deltaTs.push_back(BodyBasics->DeltT);
			//cout << "deltaT: " << deltaT;
			for (int i = 0; i < JointType_Count; i++)
			{
				cv::Point3f p = Point3f(OSkeleton_->at(Jstep + i).Position.X, OSkeleton_->at(Jstep + i).Position.Y, OSkeleton_->at(Jstep + i).Position.Z);
				nextSkeleton.push_back(p);
				cv::Point3f v = Point3f((p - currSkeleton[i])*(1 / BodyBasics->DeltT));
				Speed.push_back(v);
				
			}
			//globalTRec = BodyBasics->Tstamp;
			
			return true;
		}
		lastSkeleton.swap(currSkeleton);
		currSkeleton.swap(nextSkeleton);
		nextSkeleton.clear();
		//float deltaT = float(BodyBasics->Tstamp - globalTRec) / 10000000;
		deltaTs.push_back(BodyBasics->DeltT);
		//cout << "deltaT: " << deltaT;
		for (int i = 0; i < JointType_Count; i++)
		{
			int id = (LocalTstamp-1)*Jstep+i;
			int ids = (LocalTstamp - 3)*Jstep + i;
			cv::Point3f p = Point3f(OSkeleton_->at(id).Position.X, OSkeleton_->at(id).Position.Y, OSkeleton_->at(id).Position.Z);
			nextSkeleton.push_back(p);
			cv::Point3f v = Point3f((p - currSkeleton[i])*(1 / BodyBasics->DeltT));
			cv::Point3f a = Point3f((v - Speed[ids])*(1 / BodyBasics->DeltT));
			Speed.push_back(v);
			accel.push_back(a);
			
		}
				
		//globalTRec = BodyBasics->Tstamp;
		return true;
	}
	//cout << "illegal skeleton!!" << endl;
	return false;
}


void CBodyRepair::JointOrientationCal(const Joint joint, Joint* Jpointer, double *angle){
	
	//Jpointer point to the root joint of current skeleton
	int sign_a;
	int sign_b;
	int sign_r;
	double angle_a;
	double angle_b;
	double angle_r;
	
	JointType joint0,joint1, joint2, joint3;
	joint0 = joint.JointType;
	if (joint0 == JointType_SpineBase || joint0 == JointType_SpineMid || joint0 == JointType_HipLeft || joint0 == JointType_HipRight)
	{
		cout << "Illeagle input joint type!!!" << endl;
		return;
	}
	
	joint1 = FindFatherJoint(joint0);
	joint2 = FindFatherJoint(joint1);
	
	if (joint0 == JointType_KneeLeft || joint0 == JointType_KneeRight) joint3 = JointType_SpineMid;
	else if (joint0 == JointType_SpineShoulder) joint3 = JointType_HipRight;
	else joint3 = FindFatherJoint(joint2);
	
	Joint joint1_, joint2_, joint3_;
	joint1_ = Jpointer[joint1];
	joint2_ = Jpointer[joint2];
	joint3_ = Jpointer[joint3];
	//joint0 is the one to be judged, joint1=jointj, which is father joint of joint0;joint2= jointk,which is father joint of jointj; joint3 is father joint of k
	cv::Point3f p0 = Point3f(-joint.Position.X, joint.Position.Y, joint.Position.Z);
	cv::Point3f p1 = Point3f(-joint1_.Position.X, joint1_.Position.Y, joint1_.Position.Z);
	cv::Point3f p2 = Point3f(-joint2_.Position.X, joint2_.Position.Y, joint2_.Position.Z);
	cv::Point3f p3 = Point3f(-joint3_.Position.X, joint3_.Position.Y, joint3_.Position.Z);
	cv::Vec3f p1p0 = Vec3f(p0-p1);
	cv::Vec3f p1p2 = Vec3f(p2.x-p1.x, p2.y-p1.y,p2.z-p1.z);
	cv::Vec3f p2p3 = Vec3f(p3 - p2);
	// calculate the local x,y,z direction vector
	nyw = p1p2 / norm(p1p2);
	nzw = p2p3.cross(p1p2) / norm(p2p3.cross(p1p2));
	nxw = nyw.cross(nzw) / norm(nyw.cross(nzw));
	//calculate the project vector of p0p1 in x,y,z directions
	cv::Vec3f prjx_p0p1 = (p1p0 - (p1p0.dot(nxw))*(nxw));
	cv::Vec3f prjy_p0p1 = (p1p0 - (p1p0.dot(nyw))*(nyw));
	cv::Vec3f prjz_p0p1 = (p1p0 - (p1p0.dot(nzw))*(nzw));
	//calculate the direction of angle a,b,r
	if ((nyw.cross(prjx_p0p1)).dot(nxw)>0)	sign_a = 1;
	else sign_a = -1;
	if ((nzw.cross(prjy_p0p1)).dot(nyw)>0) sign_b = 1;
	else sign_b = -1;
	if ((nxw.cross(prjz_p0p1)).dot(nzw)>0) sign_r = 1;
	else sign_r = -1;
	//calculate the angle of a,b,r
	angle_a = sign_a*acos(nyw.dot(prjx_p0p1) / norm(prjx_p0p1))*180/PI;
	angle_b = sign_b*acos(nzw.dot(prjy_p0p1) / norm(prjy_p0p1))*180/PI;
	angle_r = sign_r*acos(nxw.dot(prjz_p0p1) / norm(prjz_p0p1))*180/PI;
	//return the orientation angle
	*angle = angle_a; angle++;
	*angle = angle_b; angle++;
    *angle = angle_r;
	//cout << "a:" << angle_a << "b:" << angle_b << "r:" << angle_r << endl;
	return;
}

//Lable the wrong joint's position
void CBodyRepair::LabelWrJoint(DepthSpacePoint depthSpacePosition,cv::Mat img,int i)
{
	cv::Scalar color=(0,0,0);
	switch (i)
	{
	case 1: color = Scalar(0x80, 0x0, 0x80); break;  //purple
	case 2: color = Scalar(0x0, 0x80, 0x0); break;   //green
	case 3: color = Scalar(255, 0, 0); break;        //blue
	case 4: color = Scalar(0, 0xff, 0xff); break;    //golden
	case 5: color = Scalar(0, 0x0, 0xff); break;     //red
	default:color = Scalar(0x0, 0x0, 0xff); break;
	}
	
	circle(img, cvPoint(depthSpacePosition.X, depthSpacePosition.Y), 8, color, 4);
	//cout << "JointDepthPosition:" << "X: " << depthSpacePosition1.X << "Y:" << depthSpacePosition1.Y << endl;
	return;
}


//To find a joint's father joint
//parameter:
JointType CBodyRepair::FindFatherJoint(JointType joint0)
{
	switch (joint0)
	{
	case 0: //cout << "no father joint can be find for joint 0" << endl; 
		return(JointType_SpineBase); break;
	case 4: return(JointType_SpineShoulder); break;
	case 8: return(JointType_SpineShoulder); break;
	case 2: return(JointType_SpineShoulder); break;
	case 12: return(JointType_SpineBase); break;
	case 16: return(JointType_SpineBase); break;
	case 20: return(JointType_SpineMid); break;
	case 21: return(JointType_HandLeft); break;
	case 22:return(JointType_WristLeft); break;
	case 23: return(JointType_HandRight); break;
	case 24:return(JointType_WristRight); break;
	default: return((JointType)(joint0-1)); break;
	}
}

//look up each joint's motion limits
CBodyMotionLimits* CBodyRepair::LookupLimits(JointType joint )
{
	CBodyMotionLimits *limits_=nullptr;
	switch (joint)
	{
	case JointType_SpineBase: cout << "no limits exist, check the joint type!" << endl; break;
	case JointType_SpineMid:cout << "no limits exist, check the joint type!" << endl; break;
	case JointType_SpineShoulder:limits_ = &Angle20_1_0; break;
	case JointType_Neck:limits_ = &Angle2_20_1; break;
	case JointType_Head:limits_ = &Angle3_2_20; break;
	case JointType_ShoulderLeft:limits_ = &Boneslimit[10]; break;
	case JointType_ElbowLeft:limits_ = &Angle5_4_20; break;
	case JointType_WristLeft:limits_ = &Angle6_5_4; break;
	case JointType_HandLeft:limits_ = &Angle7_6_5; break;
	case JointType_HandTipLeft:limits_ = &Angle21_7_6; break;
	case JointType_ThumbLeft:limits_ = &Angle22_6_5; break;
	case JointType_ShoulderRight:limits_ = &Boneslimit[22]; break;
	case JointType_ElbowRight:limits_ = &Angle9_8_20; break;
	case JointType_WristRight:limits_ = &Angle10_9_8; break;
	case JointType_HandRight:limits_ = &Angle11_10_9; break;
	case JointType_HandTipRight:limits_ = &Angle23_11_10; break;
	case JointType_ThumbRight:limits_ = &Angle24_10_9; break;
	case JointType_HipLeft:limits_ = &Angle_12_0; break;
	case JointType_KneeLeft:limits_ = &Angle13_12_0; break;
	case JointType_AnkleLeft:limits_ = &Angle14_13_12; break;
	case JointType_FootLeft:limits_ = &Angle15_14_13; break;
	case JointType_HipRight:limits_ = &Angle_16_0; break;
	case JointType_KneeRight:limits_ = &Angle17_16_0; break;
	case JointType_AnkleRight:limits_ = &Angle18_17_16; break;
	case JointType_FootRight:limits_ = &Angle19_18_17; break;
	default: break;
	}
	return limits_;
}
//calculate bone's lenght between two joints
float CBodyRepair::LengthCal(const Joint Joint1, const Joint Joint2)
{
	cv::Vec3f J12 = Vec3f(Joint1.Position.X-Joint2.Position.X,Joint1.Position.Y-Joint2.Position.Y,Joint1.Position.Z-Joint2.Position.Z);
	float length = norm(J12);
	return length;
}

//Judging if a bone's length is resonable
bool CBodyRepair::CheckLength(Joint joint1, Joint joint2)
{
	float len = LengthCal(joint1, joint2);
	float lenstd=0;
	if (joint1.JointType != FindFatherJoint(joint2.JointType) && joint2.JointType != FindFatherJoint(joint1.JointType))
	{
		cout << "illeagle pair of joints!!" << endl;
		return false;
	}
	lenstd = LookupStdLength(joint1.JointType, joint2.JointType);
	
	
	if (lenstd>0.15 && (len<0.7*lenstd || len>1.3*lenstd))
	{
		//cout << "length:" << len << "  " << lenstd << endl;
		//cout << "It's wrong!" << joint1.JointType<<joint2.JointType<<endl;
		return true;
		//LabelWrJoint(&joint1, joint1.JointType, BodyBasics->skeletonImg);
	}
	if (lenstd<=0.15 && fabs(len-lenstd)>0.05)
	{
		//cout << "length:" << len << "  " << lenstd << endl;
		//cout << "It's wrong!" << joint1.JointType << joint2.JointType << endl;
		return true;
		//LabelWrJoint(&joint1, joint1.JointType, BodyBasics->skeletonImg);
	}
	return false;
}

//judging the calculated kinematic information is resonable or not
int CBodyRepair::CheckKinematics(Joint joint,DepthSpacePoint depthposition)
{
	//judge if the joint is out of human body
	int x0 = int(depthposition.X);
	int y0 = int(depthposition.Y);
	int pixel=0;
	if (x0 > 3 && x0 < 510 && y0 < 421 && y0>3)
	{
		for (int i = x0 - 2; i < x0 + 3; i++)
			for (int j = y0 - 2; j < y0 + 3; j++)
			{
				int idx = (i + j * 512);
				int a = int(BodyBasics->bodyIndexArray[idx])<8 ? 1 : 0;
				pixel +=a;
			}
		if (pixel < 6)
		{
			//cout << "The joint is located out of body!!" << endl;
			//label with red color
			LabelWrJoint(depthposition, BodyBasics->skeletonImg,1);
			return 1;
		}
	}
	
	//judge if the joint moves too fast
	if (LocalTstamp >= 2)
	{
		int idv = (LocalTstamp - 2) * 25 + int(joint.JointType);
		float Jspeed = norm(Speed[idv]);
		if (Jspeed > control_eff*max_v)
		{
			//cout << "This joint moves too fast and maybe a wrong joint!!" << endl;
			//label with green color
			LabelWrJoint(depthposition, BodyBasics->skeletonImg,2);
			return 1;
		}
	}
	return 0;
}

//judging every joint's state and find those joints that are not resonable
void CBodyRepair::CheckJoints(unsigned int localTstamp)
{
	if (localTstamp == 0) return;
	Joint joint,jointf;
	DepthSpacePoint Dposition,Dpositionf;
	//FrameState LState;
	int id = (localTstamp-1) * 25;
	int Tracked = 0;
	int UnTracked = 0;
	for (int i = 0; i < JointType_Count; i++)
	{
		joint = BodyBasics->OSkeleton[id+i];
		Dposition = BodyBasics->OSkeletonD[id + i];
		if (joint.TrackingState == TrackingState_Tracked)
		{
			Tracked++;
			if (joint.JointType == JointType_SpineBase)
			{
				wrJ[i] = CheckKinematics(joint, Dposition);
				continue;
			}
			else 
			{
				jointf = BodyBasics->OSkeleton[id + int(FindFatherJoint(joint.JointType))];
				wrJ[i] = CheckKinematics(joint, Dposition);
				if (wrJ[i])
				{
					continue;
				}
				//label with blue color
				if (CheckLength(joint, jointf))
				{
					wrJ[i] = 1;
					LabelWrJoint(Dposition, BodyBasics->skeletonImg, 3);
				}
				continue;
			}
		}
		
		else if (joint.TrackingState == TrackingState_Inferred)
		{
			wrJ[i] = CheckKinematics(joint, Dposition);
			if (wrJ[i])
			{
				continue;
			}
			jointf = BodyBasics->OSkeleton[id + int(FindFatherJoint(joint.JointType))];
			Dpositionf = BodyBasics->OSkeletonD[id + int(FindFatherJoint(joint.JointType))];
			if (jointf.JointType != joint.JointType)
			{
				if (CheckLength(joint, jointf))
				{
					wrJ[i] = 1;
					LabelWrJoint(Dposition, BodyBasics->skeletonImg, 3);
					continue;
				}
				//label with yellow color
				if (OrientationJudging(joint, &BodyBasics->OSkeleton[id]))
				{
					wrJ[i] = 1;
					LabelWrJoint(Dpositionf, BodyBasics->skeletonImg, 4);
				}
				continue;
			}
						
		}
		else
		{
			//label with orange color
			UnTracked++;
			wrJ[i] = 1;
			LabelWrJoint(Dposition, BodyBasics->skeletonImg,5);
			continue;
		}
	}
	
	if (Tracked > 21) Frame_States.push_back(Frame_Tracked);
	else if (Tracked > 14 && UnTracked == 0 && Tracked<22) Frame_States.push_back(Frame_Inferred);
	else  Frame_States.push_back(Frame_NotTracked);
	//cout << "Frame state: " << Frame_States[localTstamp-1]<<endl;
	
	//imshow("skeleton_Labelwr", BodyBasics->skeletonImg);
}



//Look for the initialized joint length
float CBodyRepair::LookupStdLength(JointType joint1, JointType joint2)
{
	float lenstd_=0;
	int a = int(joint1 + joint2);
	switch (a)
	{
	case 5: lenstd_ = BodyBasics->Length_Head_Neck; break;
	case 22: lenstd_ = BodyBasics->Length_Neck_SpineShoulder; break;
	case 21:if (joint1 == JointType_SpineShoulder || joint1 == JointType_SpineMid) lenstd_ = BodyBasics->Length_SpineShoulder_SpineMid;
			else lenstd_ = BodyBasics->Length_Wrist_Hand; break;
	case 1: lenstd_ = BodyBasics->Length_SpineMid_SpineBase; break;
	case 24: lenstd_ = BodyBasics->Length_SpineShoulder_Shoulder; break;
	case 28: if (joint1 == JointType_SpineShoulder || joint1 == JointType_ShoulderRight) lenstd_ = BodyBasics->Length_SpineShoulder_Shoulder;
			 else if (joint1 == JointType_HandLeft || joint1 == JointType_HandTipLeft) lenstd_ = BodyBasics->Length_Hand_Handtip;
			 else lenstd_ = BodyBasics->Length_Wrist_Thumb; break;
	case 34: if (joint1 == JointType_HandRight || joint1 == JointType_HandTipRight) lenstd_ = BodyBasics->Length_Hand_Handtip;
			 else lenstd_ = BodyBasics->Length_Wrist_Thumb; break;
	case 12:case 16: lenstd_ = BodyBasics->Length_SpineBase_Hip; break;
	case 9:case 17: lenstd_ = BodyBasics->Length_Shoulder_Elbow; break;
	case 11:case 19: lenstd_ = BodyBasics->Length_Elbow_Wrist; break;
	case 13: lenstd_ = BodyBasics->Length_Wrist_Hand; break;
	case 25:case 33: lenstd_ = BodyBasics->Length_Hip_Knee; break;
	case 27:case 35: lenstd_ = BodyBasics->Length_Knee_Ankle; break;
	case 29:case 37: lenstd_ = BodyBasics->Length_Ankle_Foot; break;
	default: break;

	}
	return lenstd_;
}



