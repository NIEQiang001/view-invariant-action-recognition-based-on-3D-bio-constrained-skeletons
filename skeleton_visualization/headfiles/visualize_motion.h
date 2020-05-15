#include "stdafx.h"
#include "myKinect.h"
#include "jointopt.h"

struct body
{
	char bodyID[17];
	int clipedEdges;
	int handLeftConfidence;
	int handLeftState;
	int handRightConfidence;
	int handRightState;
	int isResticted;
	struct lean{ float X; float Y; } lean;
	int trackingState;
	int jointNum;
	Joint joint[25];
	FrameState frameState;
	int wJ[25];
};

struct userSkeleton
{
	int uSkeltonID;
	bool iniUSFlag;
	user_Joint ujoint[25];
	dlib::matrix<float, 3, 1> trans;
	dlib::matrix<float, 3, 3> RB;
};

struct frame
{
	int bodynum;	
	std::vector<body> bodys;
	std::vector<userSkeleton> uSks;

};


struct person
{
	int personID;
	int iniSkeletonFlag;
	// store the lenth of each bone
	float  Head_Neck;
	float  Neck_SpineShoulder;
	float  SpineShoulder_SpineMid;
	float  SpineMid_SpineBase;
	float  SpineShoulder_Shoulder;
	float  SpineBase_Hip;
	float  Shoulder_Elbow;
	float  Elbow_Wrist;
	float  Wrist_Hand;
	float  Hand_Handtip;
	float  Wrist_Thumb;
	float  Hip_Knee;
	float  Knee_Ankle;
	float  Ankle_Foot;
	float  Height;
};

struct motion
{
	string MotionID;
	int frameCount;
	int maximumbodies;
	person persons[6];//therefor the maximum persons in one frame is 6
	userSkeleton uSkeletons[6];
	std::vector<frame> MFrames;
	cv::Mat motionImg[6];
	
};

class common_function
{
public:
	//rotation matrix
	dlib::matrix<float, 3, 3> RotationMat(float angle, int axis);
	dlib::matrix<float, 3, 3> RotationMat(dlib::matrix<float, 3, 1> ori);
	int FindFatherJnum(int Jnum);
	float SpeedCal(Joint joint, Joint Ljoint);
	//calculate bone's lenght between two joints
	float LengthCal(const Joint Joint1, const Joint Joint2);
	//go through a folder to find all the txt files
	HRESULT goThroughfolder(string* foldpath, string* fullpath);
	//split a string by delimiter
	std::vector<std::string> split(const std::string& s, char delimiter);

private:

};

class visualizeMotion
{
public:
	
	void myReadfile(string* fullpath);//Read data from txt file and save them in a motion.frames
	void myReadfile_MSRC_12(string* fullpath);//Read data from txt file of MSRC-12 DATASET	
	void myReadfile_northwestern(string* fullpath);//Read data from txt file of northwestern uc datasets
	void IniPerson(motion* motionid, frame* frameAddr);
	void IniUSkeleton(frame* frameAddr);
	void errDetection(motion* motionid, unsigned int frameStamp, int bodyID);
	void JointOpt_genAngle(motion* motionid);
	void visualize_Motion(motion* motionid);
	void genEDMs(motion* motionid);
	void genhandsimg(motion* motionid);
	//constructor and destructor
	visualizeMotion(CBodyRepair* cbodyrepair_, CBodyOptimizer* cbodyopt_, common_function* comfun_);
	~visualizeMotion();

	//data	
	motion motions;
	cv::Mat skeletonImgVis;
	int Allinitialized = 0;

private:
	//data
	
	//std::vector<person> persons;	
	column_vector Lparams[5], Body_Lparams;//last params

	CBodyBasics* cbodybasic_vis;
	CBodyRepair* cbodyrepair_vis;
	CBodyOptimizer* cbodyopt_vis;
	common_function* comfun_vis;

	//function
	void IniskeletonFromFile(body* bodyAddr, person* person);
	//if two neighbour joints both are tracked, then return the bone length
	float visualizeMotion::CalJotState_vis(const Joint* pJoints, JointType joint0, JointType joint1);
	//enquery the saved person information	
	float LookupPersonInf(person* personAddr, int person, JointType joint1, JointType joint2);
	//Judging if a bone's length is resonable
	bool CheckLength(person* personAddr, int person, Joint joint1, Joint joint2);
	//optimize the tracked joints and generate their Eular angles
	
	void DeterminBody_vis(motion* motionid, unsigned int frameStamp, int bodyID, userSkeleton* tempuSk);
	void FineTuningLimbs_vis(motion* motionid, unsigned int frameStamp, int bodyID, userSkeleton* tempuSk);
		
	//nomalize the angel to RGB value between [0,255] according to their joints limits
	dlib::matrix<int, 3, 1> nomalize_ori(JointType jointType, userSkeleton* usk);
	Scalar handcolor(HandState handstate);
};


class Obj_function4
{
public:
	Obj_function4(const dlib::matrix<float, 3, 1> OCP[], const dlib::matrix<double, 13, 1> &Lparams, const dlib::matrix<float, 3, 1>  v[], const int* wr, common_function* com_fun)
	{
		target = OCP;
		com_fun_ = com_fun;
		wrJ_ = wr;
		v_ = v;
		Lparams_ = Lparams;
	}

	double operator() (const column_vector& params) const;

private:
	const dlib::matrix<float, 3, 1> *target;//Postion of observed joints
	dlib::matrix<double, 13, 1> Lparams_;
	const int* wrJ_;
	const dlib::matrix<float, 3, 1> *v_;
	common_function* com_fun_;

};

class Obj_function5
{

public:

	Obj_function5(const dlib::matrix<float, 3, 1> OCP[], const dlib::matrix<float, 3, 1> UP[], const dlib::matrix<float, 3, 3>& libRB, const int Jnum, const float* wr,  common_function* com_fun)
	{
		target = OCP;
		UP_ = UP;
		I = identity_matrix<float>(3);
		RB = libRB;
		Jnum_ = Jnum;
		weight = wr;
		com_fun_ = com_fun;
	}

	double operator() (const column_vector& params) const;


private:
	const dlib::matrix<float, 3, 1> *target;//Postion of observed joints
	const dlib::matrix<float, 3, 1> *UP_;//initial postion of user skeleton jonits
	int Jnum_;
	const float* weight;
	dlib::matrix<float, 3, 3> RB, I;//basic rotation matrix
	common_function* com_fun_;
	
};