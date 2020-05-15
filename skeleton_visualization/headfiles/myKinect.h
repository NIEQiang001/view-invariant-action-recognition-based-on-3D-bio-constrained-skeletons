#pragma once
#include <Kinect.h>
#include <opencv2\opencv.hpp>
#include <opencv2\imgproc\imgproc_c.h>


#define PI 3.141593

#define Angle2_20_1 Boneslimit[0]
#define Angle20_1_0 Boneslimit[1]
#define Angle5_4_20 Boneslimit[2]
#define Angle9_8_20 Boneslimit[3]
#define Angle6_5_4 Boneslimit[4]
#define Angle10_9_8 Boneslimit[5]
#define Angle7_6_5 Boneslimit[6]
#define Angle11_10_9 Boneslimit[7]
#define Angle21_7_6 Boneslimit[8]
#define Angle23_11_10 Boneslimit[9]
#define Angle4_20_1 Boneslimit[10]
#define Angle22_6_5 Boneslimit[11]
#define Angle24_10_9 Boneslimit[12]
#define Angle_12_0 Boneslimit[13]
#define Angle_16_0 Boneslimit[14]
#define Angle13_12_0 Boneslimit[15]
#define Angle17_16_0 Boneslimit[16]
#define Angle14_13_12 Boneslimit[17]
#define Angle18_17_16 Boneslimit[18]
#define Angle15_14_13 Boneslimit[19]
#define Angle19_18_17 Boneslimit[20]
#define Angle3_2_20 Boneslimit[21]
#define Angle8_20_1 Boneslimit[22]

using namespace cv;
// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}



class CBodyBasics
{
	//friend class CBodyRepair;
	//kinect 2.0 的深度空间的高*宽是 424 * 512，在官网上有说明
	static const int        cDepthWidth = 512;
	static const int        cDepthHeight = 424;
	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;


public:
	CBodyBasics();
	~CBodyBasics();
	
	void                    Update();//获得骨架、背景二值图和深度信息
	HRESULT                 InitializeDefaultSensor();//用于初始化kinect
	//画骨架函数
	void DrawBone(const Joint* pJoints, const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1, cv::Mat img);
	void DrawBone(const DepthSpacePoint* depthSpacePosition, JointType joint0, JointType joint1, cv::Mat img);
	//显示图像的Mat
	cv::Mat skeletonImg;
	cv::Mat RskeletonImg;
	cv::Mat depthImg;
	cv::Mat filted_depthImg;
	cv::Mat colorframe;
	CameraSpacePoint* pBodyPoints;
	USHORT nDepthMinReliableDistance = 0;
	USHORT nDepthMaxDistance = 0;
	UINT16 *depthArray;
	int RowMin = 0, RowMax = 0, ColumMin = 0, ColumMax = 0;
	// global time stamp
	TIMESPAN Tstamp ;
	float DeltT,FPS;//unit s, time interval between two continunous frames
	int OSnum;
	
	//vector to contain the joint information
	std::vector<Joint> OSkeleton;
	std::vector<DepthSpacePoint> OSkeletonD;
	//store the current bodyIndex map
	BYTE *bodyIndexArray;
	ICoordinateMapper*      m_pCoordinateMapper;//用于坐标变换
	Vector4 floorPlane;
	
	// store the lenth of each bone
	float  Length_Head_Neck ;
	float  Length_Neck_SpineShoulder;
	float  Length_SpineShoulder_SpineMid ;
	float  Length_SpineMid_SpineBase ;
	float  Length_SpineShoulder_Shoulder;
	float  Length_SpineBase_Hip ;
	float  Length_Shoulder_Elbow ;
	float  Length_Elbow_Wrist;
	float  Length_Wrist_Hand ;
	float  Length_Hand_Handtip ;
	float  Length_Wrist_Thumb ;
	float  Length_Hip_Knee ;
	float  Length_Knee_Ankle ;
	float  Length_Ankle_Foot ;
	int Initialized_Skeleton_flag;
	float height; //height of the skeleton
	

private:
	IKinectSensor*          m_pKinectSensor;//kinect源
	IBodyFrameReader*       m_pBodyFrameReader;//用于骨架数据读取
	IDepthFrameReader*      m_pDepthFrameReader;//用于深度数据读取
	IBodyIndexFrameReader*  m_pBodyIndexFrameReader;//用于背景二值图读取
	IColorFrameReader*      m_pColorFrameReader;// use to read color map
	
	unsigned int OSize;//record the size of observed skeleton
	
	
	//通过获得到的信息，把骨架和背景二值图画出来
	void DrawBody(int nBodyCount, IBody** ppBodies);
	
	//画手的状态函数
	void DrawHandState(const DepthSpacePoint depthSpacePosition, HandState handState);
	//tracking the state of two neighbour joints, if the two joints are both tracked, then calculate the lenght of bone consited by the two joints
	float TrackingJointState(const Joint* pJoints, JointType joint0, JointType joint1);
	// Initialize the length of each bone
	void InitializeSkeleton(IBody* skeleton);
	
};

struct CBodyMotionLimits
{
	int angle_a_inf1;
	int angle_a_sup1;
	int angle_a_inf2;
	int angle_a_sup2;
	int angle_b_inf1;
	int angle_b_sup1;
	int angle_b_inf2;
	int angle_b_sup2;
	int angle_r_inf1;
	int angle_r_sup1;
	int angle_r_inf2;
	int angle_r_sup2;
};


//declare enum values for a frame's tracking state
enum FrameState{ Frame_Tracked=0, Frame_Inferred=1, Frame_NotTracked=2 };


class CBodyRepair 
{
public:
	
	// repair the wrong inferred joints and untracked joints
	void CBodyRepair::SkeletonRepair(unsigned int LocalTstamp);
	//constructor
	CBodyRepair(CBodyBasics* BodyBasics_);
	//deconstructor
	~CBodyRepair();	
	
	//Judging if the bones move out of its normal range
	bool OrientationJudging(const Joint joint, Joint* Jpointer);
	//judging every joint's state and find those joints that are not resonable
	void CheckJoints(unsigned int localTstamp);
	//Calculate the kinematci information of each joint
	bool KinematicCal();
	int CheckKinematics(Joint joint, DepthSpacePoint depthposition);
	//look up each joint's motion limits
	CBodyMotionLimits* LookupLimits(JointType joint);
	//ICoordinateMapper*      m_pCoordinateMapper;//用于坐标变换
	int errRecord[3];      //record the orientation err of joint
	//define the limits of every joint
	CBodyMotionLimits Boneslimit[23];
	unsigned int LocalTstamp;
	//record the wrong joint num
	int wrJ[25];
	
private:
	
	
	//Label out the position of wrong joints
	void LabelWrJoint(DepthSpacePoint depthSpacePosition, cv::Mat img, int i);
	//To find a joint's father joint
	JointType FindFatherJoint(JointType joint0);
	//Find last tracked Frame
	int FindLastTrackedFrame(unsigned int LocalTstamp);
	//Calculate the joints' orientation reference to their father joint
	void JointOrientationCal(const Joint joint, Joint* Jpointer, double *angle);	
	//calculate bone's lenght between two joints
	float LengthCal(const Joint Joint1, const Joint Joint2);	
	//Judging if a bone's length is resonable
	bool CheckLength(Joint joint1, Joint joint2);
		
	//Repair Length error
	cv::Point3f RepairLengthErr(cv::Point3f joint1, cv::Point3f joint2, JointType joint1T, JointType joint2T);
	//Look for the initialized joint length
	float LookupStdLength(JointType joint1, JointType joint2);
	

	//vector to contain the joint information
	std::vector<Joint>* OSkeleton_;
	std::vector<Joint> RSkeleton;
	std::vector<cv::Point3f> lastSkeleton;
	std::vector<cv::Point3f> currSkeleton;
	std::vector<cv::Point3f> nextSkeleton;
	//vector to contain the joint information in depth Space
	std::vector<DepthSpacePoint> lastSkeletonD;
	std::vector<DepthSpacePoint> currSkeletonD;
	std::vector<DepthSpacePoint> nextSkeletonD;
	//vector to contain the joint information in camera Space
	std::vector<CameraSpacePoint> lastSkeletonC;
	std::vector<CameraSpacePoint> currSkeletonC;
	std::vector<CameraSpacePoint> nextSkeletonC;
	//container of speed and accelerate
	std::vector<cv::Point3f> Speed;
	std::vector<cv::Point3f> RSpeed;
	std::vector<cv::Point3f> accel;
	//creat an object of bodybasics
	CBodyBasics* BodyBasics;
	//local time stamp	
	TIMESPAN globalTRec=0;
	std::vector<float> deltaTs;
	unsigned int size;
	//control effector
	float control_eff = 1;
	float max_v = 3.5;//unit is m/s
	//record the frame state 
	std::vector<FrameState> Frame_States;	
	//local x,y,z direction vector
	cv::Vec3f nyw;
	cv::Vec3f nzw;
	cv::Vec3f nxw;
	
};

