#include <stdafx.h>
#include "myKinect.h"
#include <dlib\optimization.h>
#include <dlib\matrix.h>
#include <dlib/optimization/find_optimal_parameters.h>
#include <dlib/matrix/matrix_la_abstract.h>
#include <complex>
#include <dlib/threads.h>




using namespace cv;
using namespace std;
using namespace dlib;

struct user_Joint
{
	cv::Point3f Position;
	cv::Point3f Orientation;
	cv::Vec3f nxw;
	cv::Vec3f nyw;
	cv::Vec3f nzw;
	JointType   JT;
};

typedef matrix<double, 0, 1> column_vector;

struct stdLimits
{
	int a_inf;
	int a_sup;
	
	int b_inf;
	int b_sup;	
	
	int r_inf;
	int r_sup;
	
};

class CBodyOptimizer
{
public:
	//define a standard skeleton with unit length of bones
	user_Joint stdSkeleton[25];
	stdLimits stdlim[25];
	//User skeleton initilized flag
	bool USinitialized;
	dlib::matrix<float, 3, 1> Rsk[23];
	//joints' position after repairing
	std::vector<CameraSpacePoint> RSkeleton;
	//constructor and deconstructor
	CBodyOptimizer(CBodyBasics* bodybasic_, CBodyRepair* bodyrepair_);
	~CBodyOptimizer();
	//draw std skeleton
	void drawstdskeleton();
	//initialize user skeleton
	void InitializeUSkeleton();
	//convert cameraspace point t point3f
	Point3f ConvertCamToP3f(CameraSpacePoint point);
	//convert point3f to dlib column vector
	dlib::matrix<float, 3, 1> ConvertP3fToLibvector(Point3f point);
	cv::Point3f ConvertLibVToP3f(dlib::matrix<float, 3, 1> LibV);
	dlib::matrix<float> ConvertMatxToLibM(cv::Matx33f R);
	//convert column vector in Dlib to CameraspacePoint
	CameraSpacePoint ConvertLibMToCamP(dlib::matrix<float, 3, 1> LibM);
	//convert CameraspacePoint to Dlib vector
	dlib::matrix<float, 3, 1> ConvertCamPToLibV(CameraSpacePoint);	
	//rotation matrix
	dlib::matrix<float,3,3> RotationMat(float angle, int axis);
	dlib::matrix<float,3,3> RotationMat(dlib::matrix<float, 3, 1> ori);
	//estimate direction of foot by using the principal eigenvector of point cloud around foot
	void foot_Direction();
	//top down view of human body
	dlib::matrix<float,3,1> Foot_topDownview(const int jointnum);
	
	
private:
	CBodyBasics* cbodybasic;
	CBodyRepair* cbodyrepair;
	int iteratornum;
	double stopcon = 1e-7;
	cv::Matx33f RO ;//initial rotation matrix from user-skeleton space to camera space
	Matx33f RB;//Rotation matrix of base frame (pelvis frame)
	float Lsita;//Rotation angle of pelvis about z axis in last frame
	column_vector Lparams[5];//last params
	dlib::matrix<float, 3, 1> v[5][6];//
	dlib::matrix<float, 3, 3> K[5][15];
	CameraSpacePoint CRsk[21];//optimized position in camera space
	DepthSpacePoint DRsk[21];//optimized position in depth space
	dlib::matrix<double, 15, 15> C;
	dlib::matrix<double, 15, 1> B;
	dlib::matrix<float, 3, 1> principal_eigenvector;
	cv::Mat TopDownView;

	//determine the pelvis frame position and orientation, which is also the basic postion and orientation of human body
	void DerterminePelvis();
	//fine tuning the rest Euler angles
	void FineTuningAngles();
	
	//approximate closed-loop solution
	void Closed_Loop_Solution(dlib::matrix<float, 3, 3> dlibRB, const dlib::matrix<float, 3, 1> OCP[], const int m);
	
	
};

class Obj_function
{
	
public:

	Obj_function(const dlib::matrix<float, 3, 1> OCP[], const dlib::matrix<float, 3, 1> UP[], const dlib::matrix<float, 3, 3>& libRB, const int Jnum, const float* wr)
	{
		target = OCP;
		UP_ = UP;
		I = identity_matrix<float>(3);
		RB = libRB;
		Jnum_ = Jnum;
		weight = wr;
	}

	double operator() (const column_vector& params) const;
	

private:
	const dlib::matrix<float, 3, 1> *target;//Postion of observed joints
	const dlib::matrix<float, 3, 1> *UP_;//initial postion of user skeleton jonits
	int Jnum_;
	const float* weight;
	dlib::matrix<float, 3, 3> RB,I;//basic rotation matrix
	CBodyOptimizer* jointopt_;
	
	
	
};


class Obj_function2
{

public:

	Obj_function2(const dlib::matrix<float, 3, 1> OCP[], const dlib::matrix<float, 3, 3>& libRB, const dlib::matrix<float, 3, 3> K[], const dlib::matrix<float, 3, 1> v[], const column_vector  Lparams,const int Jnum, const float* wr, CBodyBasics* cbodybasic)
	{
		target = OCP;
		Lparams_ = Lparams;
		I = identity_matrix<float>(3);
		RB = libRB;
		v_ = v;
		K_ = K;
		Jnum_ = Jnum;
		weight = wr;
		cbodybasic_ = cbodybasic;
	}

	double operator() (const column_vector& params) const;


private:
	const dlib::matrix<float, 3, 1> *target;//Postion of observed joints
	const dlib::matrix<float, 3, 1> *UP_;//initial postion of user skeleton jonits
	const dlib::matrix<float, 3, 3> *K_;
	const dlib::matrix<float, 3, 1> *v_;
	int Jnum_;
	const float* weight;
	dlib::matrix<float, 3, 3> RB, I;//basic rotation matrix
	column_vector Lparams_;
	CBodyBasics* cbodybasic_;

};

class Obj_function3
{

public:

	Obj_function3(const dlib::matrix<double, 15, 1>  &B, const dlib::matrix<double, 15, 15> &C, const dlib::matrix<double, 15, 1>  &Lparams)
	{
		C_=C;
		//B_.set_size(15);
		B_ = B;
		Lparams_ = Lparams;
	}

	double operator() (const column_vector& params) const;


private:
	dlib::matrix<double, 15, 1> B_;//Postion of observed joints
	dlib::matrix<double, 15, 15> C_;
	dlib::matrix<double, 15, 1> Lparams_;
	
};

