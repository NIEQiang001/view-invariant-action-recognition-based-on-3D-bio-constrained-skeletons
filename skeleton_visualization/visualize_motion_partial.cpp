//This program is used to encode a motion into a picture
#include "stdafx.h"
#include "visualize_motion.h"
using namespace std;

//constructor and destructor
visualizeMotion::visualizeMotion(CBodyRepair* cbodyrepair_, CBodyOptimizer* cbodyopt_,  common_function* comfun_) :
cbodyrepair_vis(cbodyrepair_), comfun_vis(comfun_), cbodyopt_vis(cbodyopt_)
{
	//all the joints divide into 5 pipeline
	int Jnum[5] = { 4, 4, 3, 4, 4 };
	for (int i = 0; i < 5; i++)
	{
		Lparams[i].set_size(3 * (Jnum[i] - 1));
		Lparams[i] = 0;
	}
	
	Body_Lparams.set_size(13);
	Body_Lparams = 0;

	skeletonImgVis.create(424, 512, CV_8UC3);
	skeletonImgVis.setTo(0);

	motions.maximumbodies = 0;
	Allinitialized = 0;
}

visualizeMotion::~visualizeMotion()
{
	
}

// read ntu data
void visualizeMotion::myReadfile(string* fullpath)
{
	//full system path of the file 
	/*string filepath = "D:\\data\\NTURGB-D dataset\\nturgbd_skeletons\\nturgb+d_skeletons\\";
	string filename = "S001C001P003R001A055.skeleton";
	std::cout << filepath + filename << endl;*/
	//FILE *reader;
	//reader=fopen(filepath + filename,"r");
	ifstream reader(*fullpath);

	if (!reader) {
		std::cout << "Error opening input file" << endl;
		//exit(1);
	}

	motions.frameCount = 0;
	//int bodynum ;
	reader >> motions.frameCount;
	//std::cout << motions.frameCount << endl;	
	
	for (int i = 0; i < motions.frameCount && !reader.eof(); i++)
	{
		frame frame;
		reader >> frame.bodynum;
				
		for (int j = 0; j < frame.bodynum; j++)
		{				
			body tembody;
			reader.ignore(1, '\n');
			for (int n = 0; n < 17; n++)
			{
				reader.get(tembody.bodyID[n]);
			}
			
			//reader >> tembody.bodyID;
			reader >> tembody.clipedEdges;
			reader >> tembody.handLeftConfidence;
			reader >> tembody.handLeftState;
			reader >> tembody.handRightConfidence;
			reader >> tembody.handRightState;
			reader >> tembody.isResticted;
			reader >> tembody.lean.X;
			reader >> tembody.lean.Y;
			reader >> tembody.trackingState;
			reader >> tembody.jointNum;
			for (int m = 0; m < 25; m++)
			{
				tembody.joint[m].JointType = JointType(m);
				reader >> tembody.joint[m].Position.X;
				reader >> tembody.joint[m].Position.Y;
				reader >> tembody.joint[m].Position.Z;
				for (int n = 0; n < 8; n++)
				{
					float temp;
					reader >> temp;
				}
				int k = 0;
				reader >> k;
				tembody.joint[m].TrackingState=TrackingState(k);
			}
			frame.bodys.push_back(tembody);
		}
		motions.MFrames.push_back(frame);
		if (frame.bodynum > motions.maximumbodies) motions.maximumbodies = frame.bodynum;
		//print the read joints coordinates
		/*cout << frames[i].bodynum << endl;
		for (int n = 0; n<25; n++)
		{

			cout << frames[i].bodys[0].joint[n].Position.X << "  " << frames[i].bodys[0].joint[n].Position.Y << "  " << frames[i].bodys[0].joint[n].Position.Z << endl;

		}
		cout << i << endl;*/
	}

	

	reader.close();
}

void visualizeMotion::myReadfile_MSRC_12(string* fullpath)
{
	ifstream reader(*fullpath);

	if (!reader) {
		std::cout << "Error opening input file" << endl;
		//exit(1);
	}

	motions.frameCount = 0;	
	//int bodynum ;
	reader >> motions.MotionID;
	reader >> motions.frameCount;
	std::cout << motions.MotionID<<"  "<<motions.frameCount << endl;

	for (int i = 0; i < motions.frameCount && !reader.eof(); i++)
	{
		frame frame;
		int frameNum = 0;
		reader >> frameNum;
		for (int j = 0; j < 1; j++)
		{
			body tembody;
			reader.ignore(1, '\n');			
			for (int m = 0; m < 20; m++)
			{
				int am = 0;
				if (m == 2) am = 20;
				else am = m;
				tembody.joint[am].JointType = JointType(am);
				reader >> tembody.joint[am].Position.X;
				reader >> tembody.joint[am].Position.Y;
				reader >> tembody.joint[am].Position.Z;
				
				float k = 0;
				reader >> k;
				tembody.joint[am].TrackingState = TrackingState(int(k)+1);
			}// for m
			tembody.joint[2].JointType = JointType(2);
			tembody.joint[2].Position.X = 0.75*tembody.joint[20].Position.X + 0.25*tembody.joint[3].Position.X;
			tembody.joint[2].Position.Y = 0.75*tembody.joint[20].Position.Y + 0.25*tembody.joint[3].Position.Y;
			tembody.joint[2].Position.Z = 0.75*tembody.joint[20].Position.Z + 0.25*tembody.joint[3].Position.Z;
			tembody.joint[2].TrackingState = TrackingState(1);
			frame.bodys.push_back(tembody);
		}//for j, body number
		frame.bodynum = 1;
		motions.MFrames.push_back(frame);		
		if (frame.bodynum > motions.maximumbodies) motions.maximumbodies = frame.bodynum;
		//print the read joints coordinates
		/*cout << motions.MFrames[i].bodynum << endl;
		for (int n = 0; n<21; n++)
		{

			cout << motions.MFrames[i].bodys[0].joint[n].Position.X << "  " << motions.MFrames[i].bodys[0].joint[n].Position.Y << "  " << motions.MFrames[i].bodys[0].joint[n].Position.Z << endl;

		}
		cout << i << endl;*/
	}



	reader.close();
}

void visualizeMotion::myReadfile_northwestern(string* fullpath)
{
	//full system path of the file 
	string filepath = *fullpath+ "*.txt";
	string lastfilename;
	CString tmpstr(filepath.c_str());
	//std::cout << filepath + filename << endl;	
	motions.frameCount = 0;
	
	LPCTSTR Targetpath = tmpstr;
	HANDLE subhfind;
	WIN32_FIND_DATA findsubFileData;
	char  txtpath[200];
	if ((subhfind = FindFirstFile(Targetpath, &findsubFileData)) != INVALID_HANDLE_VALUE)
	{
		do{
			// put the paht into FilePath  
			sprintf(txtpath, "%ws", findsubFileData.cFileName);
			string filename(txtpath);
			//cout << filename << endl;

			if (filename == "fileList.txt") continue;
			if (!lastfilename.empty() && filename.substr(6, 3) < lastfilename)
			{
				std::cout << "file reading error" << endl;
				system("PAUSE");
			}
			lastfilename = filename.substr(6, 3);
			filename = *fullpath + filename;
			string subtmpstr;				
			ifstream subreader(filename);
			if (!subreader) {
				std::cout << "Error opening subinput file" << endl;
				//exit(-1);
			}

			subreader >> motions.MotionID;
			//int bodynum ;		
			for (int i = 0; i<1 && !subreader.eof(); i++)
			{
				frame frame;
				frame.bodynum = 1;

				for (int j = 0; j < frame.bodynum; j++)
				{
					body tembody;
					subreader.ignore(1, '\n');
					tembody.jointNum = 20;
					//reader >> tembody.bodyID;				
					for (int m = 0; m < 20; m++)
					{
						subreader >> subtmpstr;
						char delimiter = ',';
						std::vector<std::string> results;
						results = comfun_vis->split(subtmpstr, delimiter);
						//cout << results[0] << "  " << results[1] << endl;
						int am = 0;
						if (m == 2) am = 20;
						else am = m;
						tembody.joint[am].JointType = JointType(am);
						tembody.joint[am].Position.X = stof(results[0]);
						tembody.joint[am].Position.Y = stof(results[1]);
						tembody.joint[am].Position.Z = stof(results[2]);
						tembody.joint[am].TrackingState = TrackingState(stoi(results[3]));
					}
					tembody.joint[2].JointType = JointType(2);
					tembody.joint[2].Position.X = 0.75*tembody.joint[20].Position.X + 0.25*tembody.joint[3].Position.X;
					tembody.joint[2].Position.Y = 0.75*tembody.joint[20].Position.Y + 0.25*tembody.joint[3].Position.Y;
					tembody.joint[2].Position.Z = 0.75*tembody.joint[20].Position.Z + 0.25*tembody.joint[3].Position.Z;
					tembody.joint[2].TrackingState = TrackingState(1);

					frame.bodys.push_back(tembody);
				}
				motions.MFrames.push_back(frame);
				motions.frameCount++;
				if (frame.bodynum > motions.maximumbodies) motions.maximumbodies = frame.bodynum;					
				//cout << i << endl;*/
			}//for subreader
			subreader.close();
		} while (FindNextFile(subhfind, &findsubFileData));
	}//if find txt file		
		
	return;
}

//calculating bones length from a skeleton frame
void visualizeMotion::IniskeletonFromFile(body* bodyAddr, person* person)
{
		
	int TrackingCount = 0;
	for (int i = 0; i < JointType_Count; i++)
	{
		if (bodyAddr->joint[i].TrackingState == TrackingState_Tracked) TrackingCount++;
	}
	if (TrackingCount < 18) return;
	//if tracked joints larger than 20
	
		int l = 0;//count the number of tracked body bones
		int j = 0;//count the number of tracked arm bones
		int k = 0;//count the number of tracked leg bones
		Joint* joints = bodyAddr->joint;
				
	// -----------------initialize length of Body bones--------------------------------
			float a[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
			a[0] = CalJotState_vis(joints, JointType_Head, JointType_Neck);
			a[1] = CalJotState_vis(joints, JointType_Neck, JointType_SpineShoulder);
			a[2] = CalJotState_vis(joints, JointType_SpineShoulder, JointType_SpineMid);
			a[3] = CalJotState_vis(joints, JointType_SpineMid, JointType_SpineBase);
			a[4] = CalJotState_vis(joints, JointType_SpineShoulder, JointType_ShoulderRight);
			a[5] = CalJotState_vis(joints, JointType_SpineShoulder, JointType_ShoulderLeft);
			a[6] = CalJotState_vis(joints, JointType_SpineBase, JointType_HipLeft);
			a[7] = CalJotState_vis(joints, JointType_SpineBase, JointType_HipRight);
			if (a[0])
			{
				l++;
				person->Head_Neck = person->Head_Neck == 0 ? a[0] : (person->Head_Neck + a[0]) / 2;
				//cout << "the length of bone between head and neck is:" << Length_Head_Neck<<endl;
			}
			if (a[1])
			{
				l++;
				person->Neck_SpineShoulder = person->Neck_SpineShoulder == 0 ? a[1] : (person->Neck_SpineShoulder + a[1]) / 2;

			};
			if (a[2])
			{
				l++;
				person->SpineShoulder_SpineMid = person->SpineShoulder_SpineMid == 0 ? a[2] : (person->SpineShoulder_SpineMid + a[2]) / 2;

			}
			if (a[3])
			{
				l++;
				person->SpineMid_SpineBase = person->SpineMid_SpineBase == 0 ? a[3] : (person->SpineMid_SpineBase + a[3]) / 2;

			}

			if (a[4] || a[5])
			{
				l++;
				float len;
				len = a[4] == 0 ? a[5] : a[5] == 0 ? a[4] : (a[4] + a[5]) / 2;
				// make sure SpineShoulder_Shoulder only be tracked once, and if it has been tracked in left then use the averange length as bone length
				person->SpineShoulder_Shoulder = person->SpineShoulder_Shoulder == 0 ? len : (person->SpineShoulder_Shoulder + len) / 2;
			}

			if (a[6] || a[7])
			{
				l++;
				float len;
				len = a[6] == 0 ? a[7] : a[7] == 0 ? a[6] : (a[6] + a[7]) / 2;
				person->SpineBase_Hip = person->SpineShoulder_Shoulder == 0 ? len : (person->SpineBase_Hip + len) / 2;
			}


		//---------------Right and left Arm------------------------------------------------
			if (CalJotState_vis(joints, JointType_ShoulderRight, JointType_ElbowRight) || CalJotState_vis(joints, JointType_ShoulderLeft, JointType_ElbowLeft)){

				j++;
				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_ShoulderRight, JointType_ElbowRight);
				len2 = CalJotState_vis(joints, JointType_ShoulderLeft, JointType_ElbowLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
				if (person->Shoulder_Elbow == 0)		person->Shoulder_Elbow = len;
				else person->Shoulder_Elbow = (person->Shoulder_Elbow + len) / 2;
				//cout << "The length of Shoulder_Elbow is: " << Length_Shoulder_Elbow<<endl;
			}
			if (CalJotState_vis(joints, JointType_ElbowRight, JointType_WristRight) || CalJotState_vis(joints, JointType_ElbowLeft, JointType_WristLeft)){

				j++;
				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_ElbowRight, JointType_WristRight);
				len2 = CalJotState_vis(joints, JointType_ElbowLeft, JointType_WristLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
				if (person->Elbow_Wrist == 0)
				{
					person->Elbow_Wrist = len;
				}
				else person->Elbow_Wrist = (person->Elbow_Wrist + len) / 2;
				//cout << "The length of Elbow_Wrist is: " << Length_Elbow_Wrist << endl;
			}
			if (CalJotState_vis(joints, JointType_WristRight, JointType_HandRight) || CalJotState_vis(joints, JointType_WristLeft, JointType_HandLeft)){
				j++;
				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_WristRight, JointType_HandRight);
				len2 = CalJotState_vis(joints, JointType_WristLeft, JointType_HandLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;

				if (person->Wrist_Hand == 0)
				{
					person->Wrist_Hand = len;
				}
				else person->Wrist_Hand = (person->Wrist_Hand + len) / 2;
				//cout << "The length of Wrist_Hand is: " << Length_Wrist_Hand << endl;
			}
			if (CalJotState_vis(joints, JointType_HandRight, JointType_HandTipRight) || CalJotState_vis(joints, JointType_HandLeft, JointType_HandTipLeft)){

				j++;
				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_HandRight, JointType_HandTipRight);
				len2 = CalJotState_vis(joints, JointType_HandLeft, JointType_HandTipLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
				if (person->Hand_Handtip == 0)
				{
					person->Hand_Handtip = len;
				}
				else person->Hand_Handtip = (person->Hand_Handtip + len) / 2;
				//cout << "The length of Hand_Handtip is: " << Length_Hand_Handtip << endl;
			}
			if (CalJotState_vis(joints, JointType_WristRight, JointType_ThumbRight) || CalJotState_vis(joints, JointType_WristLeft, JointType_ThumbLeft)){


				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_WristRight, JointType_ThumbRight);
				len2 = CalJotState_vis(joints, JointType_WristLeft, JointType_ThumbLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
				if (person->Wrist_Thumb == 0)
				{
					person->Wrist_Thumb = len;
				}
				else person->Wrist_Thumb = (person->Wrist_Thumb + len) / 2;
				//cout << "The length of Wrist_Thumb is: " << Length_Wrist_Thumb << endl;
			}



		//------------------Right & Left Leg------------------------------------------------
			if (CalJotState_vis(joints, JointType_HipRight, JointType_KneeRight) || CalJotState_vis(joints, JointType_HipLeft, JointType_KneeLeft)){

				k++;
				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_HipRight, JointType_KneeRight);
				len2 = CalJotState_vis(joints, JointType_HipLeft, JointType_KneeLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
				if (person->Hip_Knee == 0)
				{
					person->Hip_Knee = len;
				}
				else person->Hip_Knee = (person->Hip_Knee + len) / 2;
			}
			if (CalJotState_vis(joints, JointType_KneeRight, JointType_AnkleRight) || CalJotState_vis(joints, JointType_KneeLeft, JointType_AnkleLeft)){

				k++;
				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_KneeRight, JointType_AnkleRight);
				len2 = CalJotState_vis(joints, JointType_KneeLeft, JointType_AnkleLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
				if (person->Knee_Ankle == 0)
				{
					person->Knee_Ankle = len;
				}
				else person->Knee_Ankle = (person->Knee_Ankle + len) / 2;
			}
			if (CalJotState_vis(joints, JointType_AnkleRight, JointType_FootRight) || CalJotState_vis(joints, JointType_AnkleLeft, JointType_FootLeft)){
				float len1, len2, len;
				len1 = CalJotState_vis(joints, JointType_AnkleRight, JointType_FootRight);
				len2 = CalJotState_vis(joints, JointType_AnkleLeft, JointType_FootLeft);
				len = len1 == 0 ? len2 : len2 == 0 ? len1 : (len1 + len2) / 2;
				if (person->Ankle_Foot == 0)
				{
					person->Ankle_Foot = len;
				}
				else person->Ankle_Foot = (person->Ankle_Foot + len) / 2;
			}
			
		
		//std::cout << l << " " << k << " " << j << endl;
		if (l >= 6 && j >= 4 && k >= 2)
		{
			person->iniSkeletonFlag++;
		}

		return;
}

//calculating tracked bones length
float visualizeMotion::CalJotState_vis(const Joint* pJoints, JointType joint0, JointType joint1)
{
	TrackingState joint0State = pJoints[joint0].TrackingState;
	TrackingState joint1State = pJoints[joint1].TrackingState;

	// If both joints are NotTracked, exit
	if ((joint0State != TrackingState_Tracked) && (joint1State != TrackingState_Tracked))
	{
		return 0;
	}

	//record joint0's position
	cv::Point3f p0 = Point3f(pJoints[joint0].Position.X, pJoints[joint0].Position.Y, pJoints[joint0].Position.Z);
	
	cv::Point3f p1 = Point3f(pJoints[joint1].Position.X, pJoints[joint1].Position.Y, pJoints[joint1].Position.Z);
	// We assume all drawn bones are inferred unless BOTH joints are tracked

	float Jointlength = 0;
	Jointlength = sqrt((p0.x - p1.x)*(p0.x - p1.x) + (p0.y - p1.y)* (p0.y - p1.y) + (p0.z - p1.z)*(p0.z - p1.z));
	return Jointlength;
	
}

//initializing a person
void visualizeMotion::IniPerson(motion* motionid, frame* frameAddr)
{
	for (int i = 0; i < 6; i++)
	{
		motions.persons[i] = { i, 0, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
	}
	for (int i = 0; Allinitialized < frameAddr->bodynum && i<motionid->frameCount; i++)
	{
		//for every person£¬initialize their bones' length
		for (int j = 0; j < frameAddr[i].bodynum; j++)
		{
			if (motions.persons[j].iniSkeletonFlag>3) continue;
			IniskeletonFromFile(&frameAddr[i].bodys[j], &motions.persons[j]);
			if (motions.persons[j].iniSkeletonFlag == 3)
			{
				std::cout << "Person "<<j<<" is initialized!" << endl;
				std::cout << "Length of Head_Neck is: " << motions.persons[j].Head_Neck << endl;
				std::cout << "Length of Length_Neck_SpineShoulder is: " << motions.persons[j].Neck_SpineShoulder << endl;
				std::cout << "Length of Length_SpineMid_SpineBase is: " << motions.persons[j].SpineMid_SpineBase << endl;
				std::cout << "Length of Length_SpineShoulder_SpineMid is: " << motions.persons[j].SpineShoulder_SpineMid << endl;
				std::cout << "Length of Ankle_Foot is: " << motions.persons[j].Ankle_Foot << endl;
				std::cout << "Length of Elbow_Wrist is: " << motions.persons[j].Elbow_Wrist << endl;
				std::cout << "Length of Hand_Handtip is: " << motions.persons[j].Hand_Handtip << endl;
				std::cout << "Length of Hip_Knee is: " << motions.persons[j].Hip_Knee << endl;
				std::cout << "Length of Knee_Ankle is: " << motions.persons[j].Knee_Ankle << endl;
				std::cout << "Length of Shoulder_Elbow is: " << motions.persons[j].Shoulder_Elbow << endl;
				std::cout << "Length of SpineBase_Hip is: " << motions.persons[j].SpineBase_Hip << endl;
				std::cout << "Length of Wrist_Hand is: " << motions.persons[j].Wrist_Hand << endl;
				std::cout << "Length of Wrist_Thumb is: " << motions.persons[j].Wrist_Thumb << endl;
				motions.persons[j].iniSkeletonFlag++;
				motions.persons[j].Height = float((int)(100 * (motions.persons[j].Head_Neck + motions.persons[j].Neck_SpineShoulder + motions.persons[j].SpineShoulder_SpineMid + motions.persons[j].SpineMid_SpineBase + motions.persons[j].Hip_Knee + motions.persons[j].Knee_Ankle))) / 100;
				Allinitialized++;

			}
		}// for every person
		
	}// for all frame	
	return;
}

//initializing user skeleton for every person
void visualizeMotion::IniUSkeleton(frame* frameAddr)
{
	for (int i = 0; i < frameAddr->bodynum; i++)
	{
		if (motions.persons[i].iniSkeletonFlag >= 3)//the best value is 3
		{
			float l01 = motions.persons[i].SpineMid_SpineBase,
				l120 = motions.persons[i].SpineShoulder_SpineMid,
				l012 = motions.persons[i].SpineBase_Hip,
				l204 = motions.persons[i].SpineShoulder_Shoulder,
				l45 = motions.persons[i].Shoulder_Elbow,
				l56 = motions.persons[i].Elbow_Wrist,
				l202 = motions.persons[i].Neck_SpineShoulder,
				l23 = motions.persons[i].Head_Neck,
				l1213 = 1.15*motions.persons[i].Hip_Knee,
				l1314 = 1.3*motions.persons[i].Knee_Ankle,
				l1415 = motions.persons[i].Ankle_Foot,
				l67 = motions.persons[i].Wrist_Hand;
			if (l1314 / l1213 < 0.5)//calibrate the length ratio between thigh and shank
			{
				float leg = l1213 + l1314;
				l1213 = leg*0.6;
				l1314 = leg - l1213;
			}
			motions.uSkeletons[i].ujoint[0] = { Point3f(0.0, 0.0, 0.0), cv::Point3f(0.0, 0.0, 0.0), (1, 0, 0), (0, 0, 1), (1, 0, 0), JointType_SpineBase };
			motions.uSkeletons[i].ujoint[1] = { Point3f(0, 0, l01), Point3f(0, 0, 0), (1, 0, 0), (0, 0, 1), (1, 0, 0), JointType_SpineMid };
			motions.uSkeletons[i].ujoint[2] = { Point3f(0, 0, l01 + l120 + l202), Point3f(0, 0, 0), (1, 0, 0), (0, 0, 1), (1, 0, 0), JointType_Neck };
			motions.uSkeletons[i].ujoint[3] = { Point3f(0, 0, l01 + l120 + l202 + l23), Point3f(0, 0, 0), (1, 0, 0), (0, 0, 1), (1, 0, 0), JointType_Head };
			motions.uSkeletons[i].ujoint[4] = { Point3f(0, -l204, l01 + l120), Point3f(0, 0, 0), (0, 0, -1), (0, 0, 1), (1, 0, 0), JointType_ShoulderLeft };
			motions.uSkeletons[i].ujoint[5] = { Point3f(0, -l204, l01 + l120 - l45), Point3f(0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), JointType_ElbowLeft };
			motions.uSkeletons[i].ujoint[6] = { Point3f(0, -l204, motions.uSkeletons[i].ujoint[5].Position.z - l56), Point3f(0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), JointType_WristLeft };
			motions.uSkeletons[i].ujoint[7] = { Point3f(0, -l204, motions.uSkeletons[i].ujoint[6].Position.z - l67), Point3f(0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), JointType_HandLeft };
			motions.uSkeletons[i].ujoint[8] = { Point3f(0, l204, l01 + l120), Point3f(0, 0, 0), (-1, 0, 0), (0, -1, 0), (0, 0, -1), JointType_ShoulderRight };
			motions.uSkeletons[i].ujoint[9] = { Point3f(0, motions.uSkeletons[i].ujoint[8].Position.y, motions.uSkeletons[i].ujoint[8].Position.z - l45), Point3f(0, 0, 0), (0, -1, 0), (0, 0, 1), (-1, 0, 0), JointType_ElbowRight };
			motions.uSkeletons[i].ujoint[10] = { Point3f(0, motions.uSkeletons[i].ujoint[8].Position.y, motions.uSkeletons[i].ujoint[9].Position.z - l56), Point3f(0, 0, 0), (0, -1, 0), (0, 0, 1), (-1, 0, 0), JointType_WristRight };
			motions.uSkeletons[i].ujoint[11] = { Point3f(0, motions.uSkeletons[i].ujoint[8].Position.y, motions.uSkeletons[i].ujoint[10].Position.z - l67), Point3f(0, 0, 0), JointType_HandRight };
			motions.uSkeletons[i].ujoint[12] = { Point3f(0.2588*l012, -0.9659*l012, 0), Point3f(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), JointType_HipLeft };
			motions.uSkeletons[i].ujoint[13] = { Point3f(motions.uSkeletons[i].ujoint[12].Position.x, motions.uSkeletons[i].ujoint[12].Position.y, -l1213), Point3f(0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), JointType_KneeLeft };
			motions.uSkeletons[i].ujoint[14] = { Point3f(motions.uSkeletons[i].ujoint[12].Position.x, motions.uSkeletons[i].ujoint[12].Position.y, motions.uSkeletons[i].ujoint[13].Position.z - l1314), Point3f(0, 0, 0), (0, 1, 0), (0, 0, 1), (1, 0, 0), JointType_AnkleLeft };
			motions.uSkeletons[i].ujoint[15] = { Point3f(motions.uSkeletons[i].ujoint[14].Position.x + l1415, motions.uSkeletons[i].ujoint[14].Position.y, motions.uSkeletons[i].ujoint[14].Position.z), Point3f(0, 0, 0), JointType_FootLeft };
			motions.uSkeletons[i].ujoint[16] = { Point3f(0.2588*l012, 0.9659*l012, 0), Point3f(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1), JointType_HipRight };
			motions.uSkeletons[i].ujoint[17] = { Point3f(motions.uSkeletons[i].ujoint[16].Position.x, motions.uSkeletons[i].ujoint[16].Position.y, -l1213), Point3f(0, 0, 0), (0, -1, 0), (0, 0, 1), (-1, 0, 0), JointType_KneeRight };
			motions.uSkeletons[i].ujoint[18] = { Point3f(motions.uSkeletons[i].ujoint[17].Position.x, motions.uSkeletons[i].ujoint[17].Position.y, motions.uSkeletons[i].ujoint[17].Position.z - l1314), Point3f(0, 0, 0), (0, -1, 0), (0, 0, 1), (-1, 0, 0), JointType_AnkleRight };
			motions.uSkeletons[i].ujoint[19] = { Point3f(motions.uSkeletons[i].ujoint[18].Position.x + l1415, motions.uSkeletons[i].ujoint[18].Position.y, motions.uSkeletons[i].ujoint[18].Position.z), Point3f(0, 0, 0), JointType_FootRight };
			//joint spine_shoulder is a triple joint
			motions.uSkeletons[i].ujoint[20] = { Point3f(0, 0, l01 + l120), Point3f(0, 0, 0), (1, 0, 0), (0, 0, -1), (0, 1, 0), JointType_SpineShoulder };
			motions.uSkeletons[i].ujoint[21] = { Point3f(0, 0, l01 + l120), Point3f(0, 0, 0), (1, 0, 0), (0, 0, -1), (0, 1, 0), JointType_SpineShoulder };
			motions.uSkeletons[i].ujoint[22] = { Point3f(0, 0, l01 + l120), Point3f(0, 0, 0), (1, 0, 0), (0, 0, -1), (0, 1, 0), JointType_SpineShoulder };
			motions.uSkeletons[i].iniUSFlag = true;
			
		}//if person
		
		else
		{
			motions.uSkeletons[i].iniUSFlag = false;
			std::cout << "Can't find body information for person " << i << endl;
		}
	}//for bodynum
	
	return;
}

//Looking for the initialized joint length
float visualizeMotion::LookupPersonInf(person* personAddr, int personid, JointType joint1, JointType joint2)
{
	float lenstd_ = 0;
	int a = int(joint1 + joint2);
	switch (a)
	{
	case 5: lenstd_ = personAddr[personid].Head_Neck; break;
	case 22: lenstd_ = personAddr[personid].Neck_SpineShoulder; break;
	case 21:if (joint1 == JointType_SpineShoulder || joint1 == JointType_SpineMid) lenstd_ = personAddr[personid].SpineShoulder_SpineMid;
			else lenstd_ = personAddr[personid].Wrist_Hand; break;
	case 1: lenstd_ = personAddr[personid].SpineMid_SpineBase; break;
	case 24: lenstd_ = personAddr[personid].SpineShoulder_Shoulder; break;
	case 28: if (joint1 == JointType_SpineShoulder || joint1 == JointType_ShoulderRight) lenstd_ = personAddr[personid].SpineShoulder_Shoulder;
			 else if (joint1 == JointType_HandLeft || joint1 == JointType_HandTipLeft) lenstd_ = personAddr[personid].Hand_Handtip;
			 else lenstd_ = personAddr[personid].Wrist_Thumb; break;
	case 34: if (joint1 == JointType_HandRight || joint1 == JointType_HandTipRight) lenstd_ = personAddr[personid].Hand_Handtip;
			 else lenstd_ = personAddr[personid].Wrist_Thumb; break;
	case 12:case 16: lenstd_ = personAddr[personid].SpineBase_Hip; break;
	case 9:case 17: lenstd_ = personAddr[personid].Shoulder_Elbow; break;
	case 11:case 19: lenstd_ = personAddr[personid].Elbow_Wrist; break;
	case 13: lenstd_ = personAddr[personid].Wrist_Hand; break;
	case 25:case 33: lenstd_ = personAddr[personid].Hip_Knee; break;
	case 27:case 35: lenstd_ = personAddr[personid].Knee_Ankle; break;
	case 29:case 37: lenstd_ = personAddr[personid].Ankle_Foot; break;
	default: break;

	}
	return lenstd_;
}

//Judging if a bone's length is resonable
bool visualizeMotion::CheckLength(person* personAddr, int personid, Joint joint1, Joint joint2)
{
	float len = comfun_vis->LengthCal(joint1, joint2);
	float lenstd = 0;
	if (joint1.JointType != comfun_vis->FindFatherJnum(joint2.JointType) && joint2.JointType != comfun_vis->FindFatherJnum(joint1.JointType))
	{
		std::cout << "illeagle pair of joints!!" << endl;
		return false;
	}
	lenstd = LookupPersonInf(personAddr, personid, joint1.JointType, joint2.JointType);


	if (lenstd>0.15 && (len<0.7*lenstd || len>1.3*lenstd))
	{
		//cout << "length:" << len << "  " << lenstd << endl;
		//cout << "It's wrong!" << joint1.JointType<<joint2.JointType<<endl;
		return true;
		//LabelWrJoint(&joint1, joint1.JointType, BodyBasics->skeletonImg);
	}
	if (lenstd <= 0.15 && fabs(len - lenstd)>0.05)
	{
		//cout << "length:" << len << "  " << lenstd << endl;
		//cout << "It's wrong!" << joint1.JointType << joint2.JointType << endl;
		return true;
		//LabelWrJoint(&joint1, joint1.JointType, BodyBasics->skeletonImg);
	}
	return false;
}

//detecting error joints in current frame
void visualizeMotion::errDetection(motion* motionid, unsigned int frameStamp, int bodyID)
{
	if (motionid->persons[bodyID].iniSkeletonFlag >= 3)//3
	{
		Joint joint, jointf;
		//only check for length and orientation error
		//FrameState LState;
		frame* frameAddr = &motionid->MFrames[0];
		int id = frameStamp;
		int bid = bodyID;
		int Tracked = 0;
		int UnTracked = 0;
		int* wrJ = frameAddr[id].bodys[bid].wJ;
		Joint* skeleton_pointer = &frameAddr[id].bodys[bid].joint[0];
		for (int i = 0; i < JointType_Count; i++)
		{
			wrJ[i] = 0;
			joint = frameAddr[id].bodys[bid].joint[i];

			if (joint.TrackingState == TrackingState_Tracked)
			{
				Tracked++;
				if (joint.JointType == JointType_SpineBase)
				{
					if (id > 0 && frameAddr[id - 1].bodynum>id)
					{
						Joint Ljoint = frameAddr[id - 1].bodys[bid].joint[i];
						float detaPos = comfun_vis->SpeedCal(joint, Ljoint);
						if (detaPos > 0.3) { wrJ[i] = 1; continue; }
					}
				}
				else
				{
					jointf = frameAddr[id].bodys[bid].joint[int(comfun_vis->FindFatherJnum(joint.JointType))];
					if (id > 0 && frameAddr[id - 1].bodynum > id)
					{
						Joint Ljoint = frameAddr[id - 1].bodys[bid].joint[i];
						float detaPos = comfun_vis->SpeedCal(joint, Ljoint);
						if (detaPos>0.3)
						{
							wrJ[i] = 1;
							continue;
						}
					}

					//label with blue color
					if (CheckLength(&motionid->persons[0], bid, joint, jointf))
					{
						wrJ[i] = 1;
					}
					continue;
				}
			}

			else if (joint.TrackingState == TrackingState_Inferred)
			{
				if (id > 0 && frameAddr[id - 1].bodynum > id)
				{
					Joint Ljoint = frameAddr[id - 1].bodys[bid].joint[i];
					float detaPos = comfun_vis->SpeedCal(joint, Ljoint);
					if (detaPos > 0.3) { wrJ[i] = 1; continue; }
				}


				jointf = frameAddr[id].bodys[bid].joint[int(comfun_vis->FindFatherJnum(joint.JointType))];

				if (jointf.JointType != joint.JointType)
				{
					if (CheckLength(&motionid->persons[0], bid, joint, jointf))
					{
						wrJ[i] = 1;
						continue;
					}
					//label with yellow color
					if (cbodyrepair_vis->OrientationJudging(joint, skeleton_pointer))
					{
						wrJ[i] = 1;
					}
					continue;
				}

			}
			else
			{
				//label with orange color
				UnTracked++;
				wrJ[i] = 1;
				continue;
			}
		}
		//cout << "wrJ: ";
		//for (int i = 0; i < 25; i++) cout << wrJ[i] << "  "; cout << endl;

		if (Tracked > 21) frameAddr[id].bodys[bid].frameState = Frame_Tracked;
		else if (Tracked > 14 && UnTracked == 0 && Tracked<22) frameAddr[id].bodys[bid].frameState = (Frame_Inferred);
		else  frameAddr[id].bodys[bid].frameState = Frame_NotTracked;
	}

	else
	{
		std::cout << "person " << bodyID << "haven't initialized" << endl;
	}
	
	return;	
}


//--------------optimizing the tracked joints and generate their Eular angles---------------
void visualizeMotion::JointOpt_genAngle(motion* motionid)
{
	for (int i = 0; i < motions.frameCount; i++)
		for (int j = 0; j < motionid->MFrames[i].bodynum; j++)
		{
			userSkeleton tempuSk;
			if (motionid->persons[j].iniSkeletonFlag < 3)
			{
				motionid->MFrames[i].uSks.push_back(tempuSk);//put into an empty usk
				continue;
			}			
			DeterminBody_vis(motionid, i, j, &tempuSk);
			FineTuningLimbs_vis(motionid, i, j, &tempuSk);
			motionid->MFrames[i].uSks.push_back(tempuSk);
			/*cout << frames[1].uSks[0].trans << endl;
			cout << frames[1].uSks[0].RB << endl;
			cout << frames[1].uSks[0].ujoint[20].Orientation << endl;*/
			//cbodyopt_vis->drawstdskeleton();			
		}	
	return;
}

void visualizeMotion::DeterminBody_vis(motion* motionid, unsigned int frameStamp, int bodyID, userSkeleton* tempuSk)
{
	int a[7] = { 0, 1, 12, 16, 20, 4, 8 };
	body* mybody = &motionid->MFrames[frameStamp].bodys[bodyID];
	user_Joint* myUSkeletonJ = motionid->uSkeletons[bodyID].ujoint;
	dlib::matrix<float, 3, 1> v[7], OCP[7];
	dlib::matrix<float, 3, 3> R[7];
	dlib::matrix<double, 13, 1> BodyLower_bound, BodyUpper_bound;
	column_vector Body_params;
	Body_params.set_size(13);
	Body_params = 0;
	int* wr = mybody->wJ;//wrJ=1 if wrong

	{
		Body_Lparams(0) = mybody->joint[0].Position.X;
		Body_Lparams(1) = mybody->joint[0].Position.Y;
		Body_Lparams(2) = mybody->joint[0].Position.Z;
		//cout << "Lparames: " << Body_Lparams << endl;
	}
	//pelvis_params = pelvis_Lparams*100;
	BodyLower_bound = -100, -100, -100, -180, -180, -180, -30, -30, -30, -15, -15, -15, -15;
	BodyUpper_bound = 100, 100, 100, 180, 180, 180, 30, 90, 30, 15, 15, 15, 15;

	//if the skeleton frame is the first frame after initializing

	for (int i = 0; i < 7; i++)
	{
		v[i] = cbodyopt_vis->ConvertP3fToLibvector(Point3f(myUSkeletonJ[a[i]].Position - myUSkeletonJ[0].Position));
		OCP[i] = cbodyopt_vis->ConvertCamPToLibV(mybody->joint[a[i]].Position);		
	}


	//------ optimize the skelton and achieve the general body orientation---------//	
	//------ As this part related to a work that has been applied for patent, readers are suggested to reproduce this part according to our paper----------//

	//cout << "final: " << body_trans << ": " << ori << endl;
	Body_Lparams = Body_params;
	
	return;
}

//fine tuning the rest Euler angles
void visualizeMotion::FineTuningLimbs_vis(motion* motionid, unsigned int frameStamp, int bodyID, userSkeleton* tempuSk)
{
	//divide joints into 5 group according to their relationships
	const int a[5][4] = { { 4, 5, 6, 7 }, { 8, 9, 10, 11 }, { 20, 2, 3 }, { 12, 13, 14, 15 }, { 16, 17, 18, 19 } };
	const int Jsize[5] = { 4, 4, 3, 4, 4 };
	body* mybody = &motionid->MFrames[frameStamp].bodys[bodyID];
	user_Joint* myUSkeletonJ = motionid->uSkeletons[bodyID].ujoint;
	float wr[4];
	dlib::matrix<float, 3, 1> OCP[4], UP[4];
	dlib::matrix<float, 3, 3> dlibRB;	
	column_vector lower_bound[5], upper_bound[5];
	dlibRB = tempuSk->RB;
	double err = 0;
	
	//judging the left and right side
	float d1, d2;
	dlib::matrix<float, 3, 1> coc12, coc16;
	coc12 = cbodyopt_vis->Rsk[12] - (cbodyopt_vis->ConvertCamPToLibV(mybody->joint[12].Position));
	coc16 = cbodyopt_vis->Rsk[12] - (cbodyopt_vis->ConvertCamPToLibV(mybody->joint[16].Position));
	d1 = dot(coc12, coc12);
	d2 = dot(coc16, coc16);
	//turn the data into wanted form of dlib

	for (int m = 0; m<5; m++)
	{
		int m_mask = 0;
		//if joint 12 is nearer to 16,the left and right has to exchange themselves
		if (d1 > d2 )
		{			
			switch (m)
			{
			case 0: m_mask = 1; break;
			case 1: m_mask = 0; break;
			case 2: m_mask = 2; break;
			case 3: m_mask = 4; break;
			case 4: m_mask = 3; break;
			default: m_mask = m; break;
			}
		}
		else m_mask = m;
		lower_bound[m].set_size(3 * (Jsize[m] - 1));
		upper_bound[m].set_size(3 * (Jsize[m] - 1));
		for (int i = 0; i < Jsize[m]; i++)
		{
						
			wr[i] = (1 - mybody->wJ[a[m_mask][i]]);
			//cout << w[i] << endl;
			UP[i] = cbodyopt_vis->ConvertP3fToLibvector(myUSkeletonJ[a[m][i]].Position);
			if (a[m][i] >= 20) OCP[i] = cbodyopt_vis->ConvertP3fToLibvector(cbodyopt_vis->ConvertCamToP3f(mybody->joint[20].Position));
			else if (i == 0) OCP[i] = cbodyopt_vis->ConvertP3fToLibvector(tempuSk->ujoint[a[m][0]].Position);

			else OCP[i] = cbodyopt_vis->ConvertP3fToLibvector(cbodyopt_vis->ConvertCamToP3f(mybody->joint[a[m_mask][i]].Position));
			

			for (int j = 0; i < Jsize[m] - 1 && j < 3; j++)
			{
				int bound_id = j + i * 3;
				switch (j)
				{
				case 0: lower_bound[m](bound_id) = cbodyopt_vis->stdlim[a[m][i]].a_inf;
					upper_bound[m](bound_id) = cbodyopt_vis->stdlim[a[m][i]].a_sup;
					break;
				case 1: lower_bound[m](bound_id) = cbodyopt_vis->stdlim[a[m][i]].b_inf;
					upper_bound[m](bound_id) = cbodyopt_vis->stdlim[a[m][i]].b_sup;
					break;
				case 2: lower_bound[m](bound_id) = cbodyopt_vis->stdlim[a[m][i]].r_inf;
					upper_bound[m](bound_id) = cbodyopt_vis->stdlim[a[m][i]].r_sup;
					break;
				default: break;
				}
				if (lower_bound[m](bound_id) == 0) lower_bound[m](bound_id) -= 6;
				if (upper_bound[m](bound_id) == 0) upper_bound[m](bound_id) += 6;

			}
			//cout << "lower bound: "<<lower_bound[m] << endl;
		}

		try
		{
			column_vector params;
			params.set_size(3 * (Jsize[m] - 1), 1);
			params = Lparams[m];


			//------ optimize the skelton and achieve the general body orientation---------//	
			//------ As this part related to a work that has been applied for patent, readers are suggested to reproduce this part according to our paper----------//
			
			//std::cout << " best result of BOBYQA: " << m << " " << best_score << endl;
			//cout << " the Euler angles are: " << m << " " << params << endl;
			
			Lparams[m] = params;
			//cout << " the Euler angles are: " << Lparams << endl;
			//cout << Lparams[0] << endl;
			
		}//try

		catch (std::exception& e)
		{
			std::cout << e.what() << endl;
		}


		//cout << err << endl;
	};//m for

}

//---------------------------------------------------------------------------------------

//-----------------------generate the encoded JEAs image for every motion frame----------------
// inputs are body orientation, joint Euler angles, and hand states which are stored in the motionid->MFrames->uSks

void visualizeMotion::visualize_Motion(motion* motionid )
{
	int width = motionid->frameCount;
	int height = 56;
	for (int i = 0; i < 6; i++)
	{
		motionid->motionImg[i].create(height, width, CV_8UC3);
		motionid->motionImg[i].setTo(0);
	}	
	
	for (int frame = 0; frame < width; frame++)
	{
		int personCount = motionid->MFrames[frame].bodynum;
		int missingbody = 0;
		for (int bodyid = 0; bodyid < personCount ; bodyid++)
		{
			if (motionid->persons[bodyid].iniSkeletonFlag < 3)//3
			{
				missingbody++;
				continue;
			}			
			uchar* motionImgdata = (uchar*)motionid->motionImg[bodyid].data;
			
			
			//calculate the ground_human relationship
			dlib::matrix<float, 3, 1> RGround_body, groundvec, groundBodyColor;
			groundvec = 0, 1, 0;
			RGround_body = motionid->MFrames[frame].uSks[bodyid].RB*groundvec;//
			groundBodyColor = (RGround_body - (-1, -1, -1))*(255 / 2);
			for (int i = 0; i < 3; i++)
			{
				switch (i)
				{
				case 0:
					motionImgdata[frame * 3] = int(groundBodyColor(i));
					break;
				case 1:
					motionImgdata[3 * width + 1 + frame * 3] = int(groundBodyColor(i));
					//cout << motionImgdata[3 * width + 1] << endl;
					break;
				case 2:
					motionImgdata[2 * 3 * width + 2] = int(groundBodyColor(i));
					//cout << motionImgdata[2*3 * width + 2+3*frame] << endl;
					break;
				default: break;
				}
			}
			int imgdatapointer;
			int a[17] = { 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14, 16, 17, 18, 20, 21, 22 };
			userSkeleton* usk = &motionid->MFrames[frame].uSks[bodyid];//
			//calculate joint ori color
			for (int i = 0; i < 17; i++)
			{
				dlib::matrix<int, 3, 1> jointcolor;
				imgdatapointer = (i * 3 + 3) * 3 * width + frame * 3;
				jointcolor = nomalize_ori(JointType(a[i]), usk);

				for (int j = 0; j < 3; j++)
				{
					switch (j)
					{
					case 0:
						motionImgdata[imgdatapointer] = jointcolor(j);
						break;
					case 1:
						motionImgdata[imgdatapointer + 3 * width + 1] = jointcolor(j);
						break;
					case 2:
						motionImgdata[imgdatapointer + 2 * 3 * width + 2] = jointcolor(j);
						break;
					default: break;
					}
				}

			}
			// put the handstate color in to code img
			HandState LeftState = HandState(motionid->MFrames[frame].bodys[bodyid].handLeftState);//
			HandState RightState = HandState(motionid->MFrames[frame].bodys[bodyid].handRightState);//
			Scalar color;
			color = handcolor(LeftState);
			motionImgdata[54 * 3 * width + frame * 3] = color(0);
			motionImgdata[54 * 3 * width + 1 + frame * 3] = color(1);
			motionImgdata[54 * 3 * width + 2 + frame * 3] = color(2);
			color = handcolor(RightState);
			motionImgdata[55 * 3 * width + frame * 3] = color(0);
			motionImgdata[55 * 3 * width + 1 + frame * 3] = color(1);
			motionImgdata[55 * 3 * width + 2 + frame * 3] = color(2);

			stringstream s;
			s << bodyid;
			string num = s.str();
			string winname = "MotionImg" + num;
			cv::imshow(winname, motionid->motionImg[bodyid]);
			cv::waitKey(5);

		}//bodyid for
		
		if (missingbody >= personCount) break;		
		
	}//frame for
	return;
}

dlib::matrix<int, 3, 1> visualizeMotion::nomalize_ori(JointType jointType, userSkeleton* usk)
{
	// enquiry the joint limits for current jointtype
	dlib::matrix<float,3,1> jointLowerLim, jointUpLim;
	
	jointLowerLim(0) = cbodyopt_vis->stdlim[jointType].a_inf;
	jointLowerLim(1) = cbodyopt_vis->stdlim[jointType].b_inf;
	jointLowerLim(2) = cbodyopt_vis->stdlim[jointType].r_inf;
	jointUpLim(0) = cbodyopt_vis->stdlim[jointType].a_sup;
	jointUpLim(1) = cbodyopt_vis->stdlim[jointType].b_sup;
	jointUpLim(2) = cbodyopt_vis->stdlim[jointType].r_sup;

	for (int i = 0; i < 3 && jointType<20 ; i++)
	{
		if (jointLowerLim(i) == 0) jointLowerLim(i) = -6;
		if (jointUpLim(i) == 0) jointUpLim(i) = 6;
	}

	dlib::matrix<int, 3, 1> jointcolor;
	jointcolor(0) = int(255*((usk->ujoint[jointType].Orientation.x - jointLowerLim(0)) / (jointUpLim(0) - jointLowerLim(0))));
	jointcolor(1) = int(255*(usk->ujoint[jointType].Orientation.y - jointLowerLim(1)) / (jointUpLim(1) - jointLowerLim(1)));
	jointcolor(2) = int(255*(usk->ujoint[jointType].Orientation.z - jointLowerLim(2)) / (jointUpLim(2) - jointLowerLim(2)));

	return jointcolor;
}


Scalar visualizeMotion::handcolor(HandState handstate)
{
	cv::Scalar color = (0, 0, 0);
	switch (handstate)
	{
	case 0: color = Scalar(0, 0x00, 0x00); break;    //black
	case 1: color = Scalar(0x80, 0x0, 0x80); break;  //purple
	case 2: color = Scalar(0x0, 0x80, 0x0); break;   //green
	case 3: color = Scalar(0, 0x0, 0xff); break;     //red
	case 4: color = Scalar(255, 0, 0); break;        //blue	
	default:color = Scalar(0x0, 0x0, 0x00); break;
	}

	
	return color;
}

void visualizeMotion::genhandsimg(motion* motionid)
{
	int width = motionid->frameCount;
	int height = 6;
	int lefthandjoints[6][2] = { { 21, 6 }, { 21, 22 }, { 7, 22 }, {7,6}, {21,7}, {22,6} };
	int righthandjoints[6][2] = { { 23, 10 }, { 23, 24 }, { 11, 24 }, { 11, 10 }, { 23, 11 }, { 24, 10 } };
	dlib::matrix<float, 3, 1> lefthandpose[6], righthandpose[6], handcolor[6];
	for (int i = 0; i < 6; i++)
	{		
		motionid->motionImg[i].create(height, width, CV_8UC3);
		motionid->motionImg[i].setTo(0);
	}
	//encoding the color frame by frame
	
	for (int frame = 0; frame < width; frame++)
	{
		int personCount = motionid->MFrames[frame].bodynum;
		int missingbody = 0;
		for (int bodyid = 0; bodyid < personCount; bodyid++)
		{
			//calculate the hand pose reference to wrist joint
			for (int k = 0; k < 6; k++)
			{
				lefthandpose[k] = 0;
				righthandpose[k ] = 0;
				lefthandpose[k](0) = motionid->MFrames[frame].bodys[bodyid].joint[lefthandjoints[k][0]].Position.X - motionid->MFrames[frame].bodys[bodyid].joint[lefthandjoints[k][1]].Position.X;
				lefthandpose[k](1) = motionid->MFrames[frame].bodys[bodyid].joint[lefthandjoints[k][0]].Position.Y - motionid->MFrames[frame].bodys[bodyid].joint[lefthandjoints[k][1]].Position.Y;
				lefthandpose[k](2) = motionid->MFrames[frame].bodys[bodyid].joint[lefthandjoints[k][0]].Position.Z - motionid->MFrames[frame].bodys[bodyid].joint[lefthandjoints[k][1]].Position.Z;
				righthandpose[k ](0) = motionid->MFrames[frame].bodys[bodyid].joint[righthandjoints[k][0]].Position.X - motionid->MFrames[frame].bodys[bodyid].joint[righthandjoints[k][1]].Position.X;
				righthandpose[k ](1) = motionid->MFrames[frame].bodys[bodyid].joint[righthandjoints[k][0]].Position.Y - motionid->MFrames[frame].bodys[bodyid].joint[righthandjoints[k][1]].Position.Y;
				righthandpose[k ](2) = motionid->MFrames[frame].bodys[bodyid].joint[righthandjoints[k][0]].Position.Z - motionid->MFrames[frame].bodys[bodyid].joint[righthandjoints[k][1]].Position.Z;
			}
			//encode hand pose to hand color
			uchar* motionImgdata = (uchar*)motionid->motionImg[bodyid].data;
			float dl = length(lefthandpose[3]) + length(lefthandpose[4]) + length(lefthandpose[5]);
			float dr = length(righthandpose[3]) + length(righthandpose[4]) + length(righthandpose[5]);
			for (int j = 0; j < 3; j++)
			{
				/*float norl = length(handpose[j]);
				handcolor[j](0) = 255*(handpose[j](0) + length(handpose[j])) / (2 * length(handpose[j]));
				handcolor[j](1) = 255*(handpose[j](1) + length(handpose[j])) / (2 * length(handpose[j]));
				handcolor[j](2) = 255*(handpose[j](2) + length(handpose[j])) / (2 * length(handpose[j]));
				motionImgdata[j*width * 3 + frame * 3] = handcolor[j](0) > 255 ? 255 : handcolor[j](0) < 0 ? 0 : handcolor[j](0);
				motionImgdata[j*width * 3 + frame * 3 + 1] = handcolor[j](1) > 255 ? 255 : handcolor[j](1)<0 ? 0 : handcolor[j](1);
				motionImgdata[j*width * 3 + frame * 3 + 2] = handcolor[j](2) > 255 ? 255 : handcolor[j](2)<0 ? 0 : handcolor[j](2);*/
				
				handcolor[0](j) = 255 * 1.2*length(lefthandpose[j]) / dl;
				handcolor[1](j) = handcolor[0](j);
				handcolor[2](j) = handcolor[0](j);
				handcolor[3](j) = 255 * 1.2*length(righthandpose[j]) / dr;
				handcolor[4](j) = handcolor[3](j);
				handcolor[5](j) = handcolor[3](j);				
			}
			for (int m = 0; m < 6; m++)
			{
				motionImgdata[m*width * 3 + frame * 3] = handcolor[m](0) > 255 ? 255 : handcolor[m](0) < 0 ? 0 : handcolor[m](0);
				motionImgdata[m*width * 3 + frame * 3 + 1] = handcolor[m](1) > 255 ? 255 : handcolor[m](1)<0 ? 0 : handcolor[m](1);
				motionImgdata[m*width * 3 + frame * 3 + 2] = handcolor[m](2) > 255 ? 255 : handcolor[m](2)<0 ? 0 : handcolor[m](2); 
			}
			/*stringstream s;
			s << bodyid;
			string num = s.str();
			string winname = "handMotionImg" + num;
			cv::imshow(winname, motionid->motionImg[bodyid]);
			cv::waitKey(5);*/
			
		}//for bodyid
	}//for frame
	
	return;
}

//----------------------------------------------------------------------------------------
// generate the encoded EDM image
void visualizeMotion::genEDMs(motion* motionid)
{
	cv::Mat iniEDMimg, lastEDMimg, avgEDMimg, syntheticEDMimg;
	dlib::matrix<float, 20, 20> iniEDM, lastEDM, avgEDM;
	iniEDMimg.create(20, 20, CV_8UC1);
	iniEDMimg.setTo(0);
	lastEDMimg.create(20, 20, CV_8UC1);
	lastEDMimg.setTo(0);
	avgEDMimg.create(20, 20, CV_8UC1);
	avgEDMimg.setTo(0);
	syntheticEDMimg.create(20, 20, CV_8UC3);
	syntheticEDMimg.setTo(0);
	iniEDM = 0;
	lastEDM = 0;
	avgEDM = 0;
	//calculate the initial EDM, last EDM and average EDM
	for (int s = 0; s < motionid->frameCount; s++)
	{
	    dlib::matrix<float, 20, 20> temEDM;
		temEDM = 0;
		for (int i = 0; i < 20; i++)
			for (int j = 0; j < 20; j++)
			{
			    dlib::matrix<float, 3, 1> tempveci,tempvecj;
				tempveci = cbodyopt_vis->ConvertCamPToLibV(motionid->MFrames[s].bodys[0].joint[i].Position);
				tempvecj = cbodyopt_vis->ConvertCamPToLibV(motionid->MFrames[s].bodys[0].joint[j].Position);
				temEDM(i, j) = length(tempveci-tempvecj);
			}
		if (s == 0) iniEDM = temEDM;
		if (s == motionid->frameCount - 1) lastEDM = temEDM;
		avgEDM += temEDM;
	}
	/*cout << iniEDM << endl;
	cout << lastEDM << endl;
	cout << avgEDM << endl;*/
	avgEDM = avgEDM*float(1.0 / motionid->frameCount);
	std::cout << avgEDM << endl;
	//encode the three EDM into color
	uchar* iniEDMdata = (uchar*)iniEDMimg.data;
	uchar*	lastEDMdata = (uchar*)lastEDMimg.data;
	uchar*	avgEDMdata = (uchar*)avgEDMimg.data;
	uchar*  syntheticEDMdata = (uchar*)syntheticEDMimg.data;

	for (int i = 0; i < 20; i++)
		for (int j = 0; j < 20; j++)//normalize EDM
		{
			iniEDM(i, j) = iniEDM(i, j) / 2;
			lastEDM(i, j) = lastEDM(i, j) / 2;
			avgEDM(i, j) = avgEDM(i, j) / 1.5;

			*iniEDMdata = iniEDM(i, j) * 255; iniEDMdata++;
			*lastEDMdata = lastEDM(i, j) * 255; lastEDMdata++;
			*avgEDMdata = avgEDM(i, j) * 255; avgEDMdata++;

			*syntheticEDMdata = iniEDM(i, j) * 255; syntheticEDMdata++;
			*syntheticEDMdata = lastEDM(i, j) * 255; syntheticEDMdata++;
			*syntheticEDMdata = avgEDM(i, j) * 255; syntheticEDMdata++;
		}
	
	cv::imshow("iniEDM", iniEDMimg);
    cv:: waitKey(5);
	cv::imshow("lastEDM", lastEDMimg);
	cv::waitKey(5);
	cv::imshow("avgEDM", avgEDMimg);
	cv::waitKey(5);
	cv::imshow("syntheticEDM", syntheticEDMimg);
	cv::waitKey(5);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\iniEDM1.5c.jpg", iniEDMimg);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\lastEDM1.5c.jpg", lastEDMimg);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\avgEDM1.5c.jpg", avgEDMimg);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\syntheticEDM1.5c.jpg", syntheticEDMimg);
	
	//resize image
	cv::Mat dstimg, syndstimg;
	dstimg.create(240, 240, CV_8UC1);
	dstimg.setTo(0);
	syndstimg.create(240, 240, CV_8UC3);
	syndstimg.setTo(0);
	cv::resize(iniEDMimg, dstimg, dstimg.size(), 0, 0);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\resized_iniEDM1.5c.jpg", dstimg);
	cv::resize(lastEDMimg, dstimg, dstimg.size(), 0, 0);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\resized_lastEDM1.5c.jpg", dstimg);
	cv::resize(avgEDMimg, dstimg, dstimg.size(), 0, 0);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\resized_avgEDM1.5c.jpg", dstimg);
	cv::resize(syntheticEDMimg, syndstimg, syndstimg.size(), 0, 0);
	cv::imwrite("D:\\data\\skeleton_image_sequence\\ntu_edm\\resized_synEDM1.5c.jpg", syndstimg);
	return;	
}



dlib::matrix<float, 3, 3> common_function::RotationMat(float angle, int axis)
{
	float angle_r = angle*pi / 180;
	dlib::matrix<float, 3, 3> R;
	if (axis == 1)
		R = 1, 0, 0,
		0, cos(angle_r), -sin(angle_r),
		0, sin(angle_r), cos(angle_r);
	else if (axis == 2)
		R = cos(angle_r), 0, sin(angle_r),
		0, 1, 0,
		-sin(angle_r), 0, cos(angle_r);
	else if (axis == 3)
		R = cos(angle_r), -sin(angle_r), 0,
		sin(angle_r), cos(angle_r), 0,
		0, 0, 1;
	else std::cout << "pls. input a legal axis!" << endl;
	return R;
}

dlib::matrix<float, 3, 3> common_function::RotationMat(dlib::matrix<float, 3, 1> ori)
{
	float angle_r1 = ori(0);
	float angle_r2 = ori(1);
	float angle_r3 = ori(2);
	dlib::matrix<float, 3, 3> R;
	return R = RotationMat(angle_r1, 1)*RotationMat(angle_r2, 2)*RotationMat(angle_r3, 3);
}

int common_function::FindFatherJnum(int Jnum)
{
	switch (Jnum)
	{
	case 0: //cout << "no father joint can be find for joint 0" << endl; 
		return(0); break;
	case 4: return(20); break;
	case 8: return(20); break;
	case 2: return(20); break;
	case 12: return(0); break;
	case 16: return(0); break;
	case 20: return(1); break;
	case 21: return(7); break;
	case 22:return(6); break;
	case 23: return(11); break;
	case 24:return(10); break;
	default: return(Jnum - 1); break;
	}
}

float common_function::SpeedCal(Joint joint, Joint Ljoint)
{
	cv::Point3f point1, point2, point;
	point1 = Point3f(joint.Position.X, joint.Position.Y, joint.Position.Z);
	point2 = Point3f(Ljoint.Position.X, Ljoint.Position.Y, Ljoint.Position.Z);
	point = point1 - point2;
	float speed = norm(point);
	return speed;
}

float common_function::LengthCal(const Joint Joint1, const Joint Joint2)
{
	cv::Vec3f J12 = Vec3f(Joint1.Position.X - Joint2.Position.X, Joint1.Position.Y - Joint2.Position.Y, Joint1.Position.Z - Joint2.Position.Z);
	float length = norm(J12);
	return length;
}

double Obj_function4::operator() (const column_vector& params) const
{
	dlib::matrix<float, 3, 3> RB, R1, R[7];
	//0, 1, 12, 16, order in CP array
	dlib::matrix<float, 3, 1> Ori, CP[7], trans, us[7];
	CameraSpacePoint cam_CP[7];
	dlib::matrix<float, 3, 1> err_dpos[7];
	dlib::matrix<double, 13, 1> x;
	//dlib::matrix<double, 6, 6> M;
	int a[7] = { 0, 1, 12, 16, 20, 4, 8 };
	int c[7] = { 0, 1, 3, 2, 4, 6, 5 };
	R1 = 0, 1, 0,
		0, 0, 1,
		-1, 0, 0;

	x = Lparams_;
	//x = params;
	trans = 100 * x(0) + params(0), 100 * x(1) + params(1), 100 * x(2) + params(2);
	Ori = params(3), params(4), params(5);
	trans = 0.01*trans;
	RB = R1*com_fun_->RotationMat(Ori);

	//cout << "trans:" << trans <<"Ori:" <<Ori<< endl;
	//rotation matrix for joint 1 and 20
	R[0] = identity_matrix<float>(3);
	R[1] = RB;
	R[2] = RB;
	R[3] = RB;
	R[4] = RB*com_fun_->RotationMat(params(6), 1)*com_fun_->RotationMat(params(7), 2)*com_fun_->RotationMat(params(8), 3);
	R[5] = R[4] * com_fun_->RotationMat(params(9), 1)*com_fun_->RotationMat(params(10), 3);
	R[6] = R[4] * com_fun_->RotationMat(params(11), 1)*com_fun_->RotationMat(params(12), 3);
	us[0] = v_[0]; us[1] = v_[1] - v_[0]; us[2] = v_[2] - v_[0];
	us[3] = v_[3] - v_[0]; us[4] = v_[4] - v_[1]; us[5] = v_[5] - v_[4];
	us[6] = v_[6] - v_[4];
	CP[0] = trans;
	CP[1] = RB*us[1] + CP[0];
	CP[2] = RB*us[2] + CP[0];
	CP[3] = RB*us[3] + CP[0];
	CP[4] = R[4] * us[4] + CP[1];
	CP[5] = R[5] * us[5] + CP[4];
	CP[6] = R[6] * us[6] + CP[4];
	for (int i = 0; i < 7; i++)
	{
		//cout << CP[i] << endl;
		cam_CP[i].X = CP[i](0);
		cam_CP[i].Y = CP[i](1);
		cam_CP[i].Z = CP[i](2);
		err_dpos[i] = (CP[i] - target[i]) * (1-wrJ_[a[i]]) * 100;

	}

		
	double result = 0, err_d = 0;
	
	//int a = 10, b = 10;
	//error in depth space
	for (int i = 0; i < 4; i++)
	{
		err_d += dot(err_dpos[i], err_dpos[i]);		
	}

	//cout << "the errors at each space are:" << err_c<<" "<<err_d << endl;
	result = err_d;
	//return the mean squared error between the target vector and the input vector
	return result;

}

double Obj_function5::operator() (const column_vector& params) const
{
	dlib::matrix<float, 3, 1> CP[4], Ori[4], e[4]; // position of user skeleton joints in camera space
	dlib::matrix<float, 3, 3> R[4];//basic rotation matrix
	//float delta_sita[6];
	for (int i = 0; i < Jnum_; i++)
	{
		CP[i] = 0;
		Ori[i] = 0;
		e[i] = 0;
		R[i] = I;

	}
	CP[0] = target[0];

	for (int i = 0; i < Jnum_ - 1; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			Ori[i](j) = params(i * 3 + j);
		}
		for (int j = 0; j < i + 1; j++)
		{
			R[i] = R[i] * com_fun_->RotationMat(Ori[j]);
		}

		CP[i + 1] = RB*R[i] * (UP_[i + 1] - UP_[i]) + CP[i];

	}
	
	dlib::matrix<double, 2, 1> err_cpos[4];

	double result = 0;

	for (int i = 1; i < Jnum_; i++)
	{
				
		//calculate error in depth space
		//e[i] = (CP[i] - target[i]);
		e[i] = 10 * (CP[i] - target[i]) / target[i](2);

		result += 10 * weight[i] * trans(e[i])*e[i];

	}
	
	// return the mean squared error between the target vector and the input vector
	return result;
}

HRESULT common_function::goThroughfolder(string* foldpath, string* fullpath)
{
	
	HANDLE hfind;
	WIN32_FIND_DATA findFileData;
	char  filepath[200];
	int i = 0;

	//sprintf(Targetpath, "%s%s", foldpath, fileType);
	LPCTSTR Targetpath = L"D:\\data\\NTURGB-D dataset\\nturgbd_skeletons\\nturgb+d_skeletons\\*.skeleton";
	if ((hfind = FindFirstFile(Targetpath, &findFileData)) != INVALID_HANDLE_VALUE) 
	{
		do { 
			if (!(findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)) { // not floder dir 
				//wcout << findFileData.cFileName << endl;
				// put the paht into FilePath  
				sprintf(filepath, "%ws", findFileData.cFileName);

				string filename(filepath);
				*fullpath = *foldpath + filename;
				//cout << *foldpath << endl;
				std::cout << *fullpath << endl;
				i++;
			}
		} while (FindNextFile(hfind, &findFileData) && i<10); // find next file 

		FindClose(hfind);
	}

	return 0;


}


//split a string by delimiter
std::vector<std::string> common_function::split(const std::string& s, char delimiter)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream(s);
	while (std::getline(tokenStream, token, delimiter))
	{
		tokens.push_back(token);
	}
	return tokens;
}