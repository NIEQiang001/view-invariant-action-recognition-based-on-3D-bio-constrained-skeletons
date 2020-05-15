
#include <stdafx.h>
#include "myKinect.h"
//#include "jointopt.h"
#include "visualize_motion.h"

using namespace std;

void ImageSave(cv::Mat img, int a, string* filename);
void WriteDown(string* filename,int type);

int main()
{
	common_function myComfun;
	CBodyBasics myKinect;
	CBodyRepair myRepair(&myKinect);
	CBodyOptimizer myOpt(&myKinect, &myRepair);	
	HRESULT hr = myKinect.InitializeDefaultSensor();
	if (SUCCEEDED(hr))
	{
		string foldpath = "I:\\ntu_challenge\\resized\\synthesized\\";
		//string fullpath;
		string missingfile = "D:\\data\\NTURGB-D dataset\\nturgbd_skeletons\\nturgb+d_skeletons\\S009C003P007R002A050.skeleton";
		LPCTSTR Targetpath = L"I:\\ntu_challenge\\resized\\synthesized\\*.jpg";
		string foldpath2 = "I:\\ntu_challenge\\ntu_AllSkeletonFiles_remove_nan_nolabel\\";
		HANDLE hfind;
		WIN32_FIND_DATA findFileData;
		char  filepath[200];
		int fileCount = 0, errorflag = 0;
		/*ifstream finder(missingfile);
		if (!finder) { cerr << "Can't open missing file list!" << endl; return -1; }*/	

		if ((hfind = FindFirstFile(Targetpath, &findFileData)) != INVALID_HANDLE_VALUE)
		{
					
			do {			
				//errorflag = 0; finder.seekg(0);
				if (!(findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
				{ // not floder dir 				
					// put the paht into FilePath  
					sprintf(filepath, "%ws", findFileData.cFileName);
					string filename(filepath);
					string fullpath = foldpath + filename;
					/*string s = filename.substr(18, 2);
					if (stoi(s) >= 50)
					{
						if (filename.substr(30, 1) != "s")
						{
							continue;
						}
										
					}*/
					//cv::Mat tmpimage[4],tmpimagef,tmpimageO;
					//tmpimagef.create(60, 128, CV_8UC3);
					//tmpimage[3].create(6, 128, CV_8UC3);
					//tmpimage[3].setTo(0);
					//int subimgcount = 0;
					//for (int i = 0; i < 3;i++)
					//{
					//	stringstream ss;
					//	ss << i;
					//	string tmpstr = ss.str();						
					//	tmpimage[i].create(6, 128, CV_8UC3);
					//	string imgname = foldpath2 + filename.substr(0, 30) + tmpstr + ".jpg";
					//	tmpimage[i] = imread(imgname);
					//	if (tmpimage[i].data == NULL)
					//	{
					//		continue;
					//	}
					//	tmpimage[3] += tmpimage[i];
					//	subimgcount++;
					//}
					//tmpimage[3] = tmpimage[3] / subimgcount;
					//tmpimageO = cv::imread(fullpath);
					
					if (filename != "." && filename != "..")
					{
						//cout << filename << "  ";
						string txtfilename = filename.substr(0, 15);
						string txtfullpath = foldpath2 + txtfilename;
						//-------------------------------------------------------
						visualizeMotion *visM = new visualizeMotion(&myRepair, &myOpt, &myComfun);
						//read the file and generate coding picture

						visM->myReadfile(&txtfullpath);//Read data from txt file and save them in a motion.frames
						visM->genhandsimg(&visM->motions);
						/*if (visM->motions.frameCount < 10) goto WriteDownErrFile;
						visM->IniPerson(&visM->motions, &(visM->motions.MFrames[0]));

						if (visM->Allinitialized < visM->motions.maximumbodies) goto WriteDownErrFile;


						visM->IniUSkeleton(&(visM->motions.MFrames[0]));
						if (visM->motions.uSkeletons[0].iniUSFlag == false) goto WriteDownErrFile;

						for (int i = 0; i < visM->motions.frameCount; i++)
						{
						for (int j = 0; j < visM->motions.MFrames[i].bodynum; j++)
						{
						visM->errDetection(&visM->motions, i, j);
						}
						}
						visM->JointOpt_genAngle(&visM->motions);
						visM->visualize_Motion(&visM->motions);*/
						cv::Mat tmpimageS,tmpimageO,tmpimagef;
						tmpimageS.create(6, 128, CV_8UC3);
						tmpimageS.setTo(0);
						tmpimagef.create(60, 128, CV_8UC3);
						for (int bodyid = 0; bodyid < visM->motions.maximumbodies; bodyid++)
						{
							cv::Mat tmpimage;
							tmpimage.create(6, 128, CV_8UC3);
							int Width = visM->motions.motionImg[bodyid].size().width;

							if (Width > 128)
							{
								std::vector<int> cols;
								cols.push_back(0);
								cols.push_back(floor(Width / 2));
								cols.push_back(Width - 1);
								while (cols.size() < 128)
								{
									int size = cols.size();
									for (int i = 0; i < size - 1; i++)
									{
										if (size + i == 128) break;
										int k = floor((cols[i] + cols[i + 1]) / 2);
										cols.push_back(k);
									}
									sort(cols.begin(), cols.end(), less<int>());

								}
								for (int j = 0; j < cols.size(); j++)
								{
									visM->motions.motionImg[bodyid].col(cols[j]).copyTo(tmpimage.col(j));									
								}
								//ImageSave(tmpimage, bodyid, &filename);
								tmpimageS += tmpimage;
							}//image width larger than 128

							else
							{
								cv::resize(visM->motions.motionImg[bodyid], tmpimage, tmpimage.size(), 0, 0);
								//ImageSave(tmpimage, bodyid, &filename);
								tmpimageS += tmpimage;
							}
						}
						tmpimageS = tmpimageS / visM->motions.maximumbodies;
						tmpimageO = cv::imread(fullpath);
						tmpimageO.copyTo(tmpimagef(Rect(0, 0, tmpimageO.cols, tmpimageO.rows)));
						//tmpimageO.copyTo(tmpimagef.rowRange(0,55));
						tmpimageS.copyTo(tmpimagef(Rect(0, 54, tmpimageS.cols, tmpimageS.rows)));
						cv::imshow("synthesized image",tmpimagef);
						waitKey(5);
						ImageSave(tmpimagef, 0, &filename);
						/*for (int bodies = 0; bodies < visM->motions.maximumbodies; bodies++)
						{
						if (visM->motions.persons[bodies].iniSkeletonFlag>3)
						ImageSave(visM->motions.motionImg[bodies], bodies, &filename);
						}*/

						//-------------------------------------------------------------
						fileCount++;
						cout << fileCount << "  " << fullpath << endl;
						//WriteDown(&filename, 1);
						delete visM;
						continue;
					/*WriteDownErrFile:
						WriteDown(&filename, 0);*/
					}//if succeed reading a file
				 }
				} while (FindNextFile(hfind, &findFileData)); // find next file 

				FindClose(hfind);					
		}//if find first file

		
		//delete visM;
		//--------------------------------------------------------------


	}// if succeed hr
		
	
	Sleep(200000);
		
}


//image saving
//Parameter:
//Result:
void ImageSave(cv::Mat img, int a,string* filename)
{
	stringstream s;
	s << a;
	string num = s.str();
	string originname = (*filename).substr(0, 29);
	//static const string SaveFold = "D://data//skeleton_image_sequence//ntu_motion//";
	static const string SaveFold = "I:\\ntu_challenge\\resized\\final2\\";
	//const string filename = "S001C001P003R001A055.skeleton_";
	//const string SavePath = SaveFold + originname +"_" + num + ".jpg";
	const string SavePath = SaveFold + *filename;
	cv::imwrite(SavePath, img);
	waitKey(30);
}

void WriteDown(string* filename, int type)
{
	/*static const string errorfile = "D://data//skeleton_image_sequence//ntu_motion//errfiles.txt";*/
	static const string errorfile = "D:\\data\\skeleton_image_sequence\\ntu_motion\\2nd\\errfiles.txt";
	static const string rightfile = "D:\\data\\skeleton_image_sequence\\ntu_motion\\2nd\\filesrun.txt";
	string txtname;
	if (type == 0)
		txtname = errorfile;
	else txtname = rightfile;
	ofstream recorder(rightfile,ios::app);
	if (!recorder) { cerr << "Can't open error file list!" << endl; return; }
	recorder << *filename << endl;
	recorder.close();
	return;

}


// backup main function for northwestern dataset
/*
{
   if ((hfind = FindFirstFile(Targetpath, &findFileData)) != INVALID_HANDLE_VALUE)
		{
					
			do {
					//errorflag = 0; finder.seekg(0);
					if ((findFileData.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY))
					{ // not floder dir 				
						// put the paht into FilePath  
						sprintf(filepath, "%ws", findFileData.cFileName);
						string filename(filepath);
						
						if (filename != "." && filename != "..")
						{
							cout << filename << "  ";
							fullpath = foldpath + filename + "\\";
							//-------------------------------------------------------
							visualizeMotion *visM = new visualizeMotion(&myRepair, &myOpt, &myComfun);
							//read the file and generate coding picture

							visM->myReadfile_northwestern(&fullpath);//Read data from txt file and save them in a motion.frames
							if (visM->motions.frameCount < 10) goto WriteDownErrFile;
							visM->IniPerson(&visM->motions, &(visM->motions.MFrames[0]));
							
							if (visM->Allinitialized < visM->motions.maximumbodies) goto WriteDownErrFile;


							visM->IniUSkeleton(&(visM->motions.MFrames[0]));
							if (visM->motions.uSkeletons[0].iniUSFlag == false) goto WriteDownErrFile;

							for (int i = 0; i < visM->motions.frameCount; i++)
							{
								for (int j = 0; j < visM->motions.MFrames[i].bodynum; j++)
								{
									visM->errDetection(&visM->motions, i, j);
								}
							}
							visM->JointOpt_genAngle(&visM->motions);
							visM->visualize_Motion(&visM->motions);
							for (int bodies = 0; bodies < visM->motions.maximumbodies; bodies++)
							{
								if (visM->motions.persons[bodies].iniSkeletonFlag>3)
									ImageSave(visM->motions.motionImg[bodies], bodies, &filename);
							}

							//-------------------------------------------------------------
							cout << fileCount << "  " << fullpath << endl;
							fileCount++;
							WriteDown(&filename,1);
							delete visM;
							continue;
						WriteDownErrFile:
							WriteDown(&filename,0);
						}//if succeed reading a file
					}
						
				
				} while (FindNextFile(hfind, &findFileData)); // find next file 

				FindClose(hfind);					
		}//if find first file
}
*/