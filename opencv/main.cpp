#include "highgui.h"
#include "cxcore.h"
#include "cv.h"
#include "cvut.h"
#include <iostream>
#include <fstream>
#include <string>

#include <windows.h> 
#include<WinUser.h>
#include <NuiApi.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

using namespace cvut;
using namespace std;
using namespace cv;

CvMat *intrinsics;
CvMat *distortion_coeff;
CvMat *rotation_vectors;
CvMat *translation_vectors;
CvMat *object_points;
CvMat *point_counts;
CvMat *image_points;

CvSize board_size = cvSize(9,6);    /* 定标板上每行、列的角点数 标定板尺寸*/
CvPoint2D32f* image_points_buf = new CvPoint2D32f[board_size.width*board_size.height];   /* 缓存每幅图像上检测到的角点 */
cvut::Seq<CvPoint2D32f> image_points_seq;  /* 保存检测到的所有角点 */
USHORT* bits=new USHORT[480*640];
USHORT* bitsfirst=new USHORT[480*640];


class point
{
public:
	USHORT x;
	USHORT y;

	point(USHORT i=0,USHORT j=0)
	{
		x=i;
		y=j;
	}
};

point* depthtoimage=new point[480*640];

vector<point*> borderunsort;
vector<point*> fingertips;
vector<point*> inhand;

IplImage     *img_gray;
CvSize image_size;
int image_count=0;
ifstream fin("image_pic.txt"); /* 定标所用图像文件的路径 */
CvMat *single=cvCreateMat(3,3,CV_64FC1);

CvPoint2D32f transfer(const CvMat* M,CvPoint2D32f src);//图像坐标映射到屏幕坐标
CvPoint2D32f transferRev(const CvMat* M,CvPoint2D32f src);//屏幕坐标映射到图像坐标
void drawBoder(const CvMat* M,string filename);
CvMat** calcuInMatrix();
CvMat* getRotAndTrans(string filename,const CvMat* M,const CvMat* W);
int getFinger(CvPoint2D32f* finger,CvPoint2D32f pCenter,const CvMat* M);
bool findTwoHand(uchar *ptr,point &pf);//是否是双手触摸，第一只手赋值为2，第二只手赋值为0.
void setFild(uchar *ptr,int i,int j,int org,int num);//将(i,j)位置的值设为2,并递归调用
point* findFirstHand(uchar *ptr);//找到第一个手的点

/*void use()
{
	Kinect* kinect=new Kinect();
	kinect->init(640,480);
	while(1)
		kinect->queryFrame();
}*/

void main(int argc,char** argv)
{
	/*初始化内参矩阵*/
	CvMat** inner=calcuInMatrix();
	CvMat* instrain=inner[0];
	CvMat* destro=inner[1];
	/*计算投影旋转和平移矩阵和单应性矩阵*/
	CvMat* rotandtrans=getRotAndTrans("a.bmp",instrain,destro);
	
	cvMatMul(instrain, rotandtrans, single);
	CvPoint2D32f center;
	center.x=1366/2;
	center.y=768/2;
	CvPoint2D32f pCenter=transferRev(single,center);
	CvPoint2D32f* finger=new CvPoint2D32f();
	/*画出投影区域边框*/
	drawBoder(single,"a.bmp");
	/*坐标映射*/
	CvPoint2D32f dest;
	CvPoint2D32f src;
	src.x=640-214;
	src.y=199;
	dest=transfer(single,src);//图像坐标到电脑桌面坐标映射
//	SetCursorPos(dest.x,dest.y);
	getFinger(finger,pCenter,single);
//	use();
//	cout<<dest.x<<" "<<dest.y;
//	system("pause");
}

CvPoint2D32f transfer(const CvMat* M,CvPoint2D32f src)
{
	CvMat *X=cvCreateMat(3,1,CV_64FC1);
	CvMat *invert=cvCreateMat(3,3,CV_64FC1);
	CvMat *xdest=cvCreateMat(3,1,CV_64FC1);
	double source[3]={src.x,src.y,1};
	cvInitMatHeader(X,3,1,CV_64FC1,source);
	cvInvert(M, invert);
	double s;
	cvMatMul(invert, X, xdest);
	s=cvmGet(xdest,2,0);
	CvPoint2D32f a;
	a.x=cvmGet(xdest,0,0)/s;
	a.y=cvmGet(xdest,1,0)/s;
	cvReleaseMat(&X);
	cvReleaseMat(&xdest);
	cvReleaseMat(&invert);
	return a;
}

CvPoint2D32f transferRev(const CvMat* M,CvPoint2D32f src)
{
	CvMat *X=cvCreateMat(3,1,CV_64FC1);
	CvMat *xdest=cvCreateMat(3,1,CV_64FC1);
	double source[3]={src.x,src.y,1};
	cvInitMatHeader(X,3,1,CV_64FC1,source);
	double s;
	cvMatMul(M, X, xdest);
	s=cvmGet(xdest,2,0);
	CvPoint2D32f a;
	a.x=cvmGet(xdest,0,0)/s;
	a.y=cvmGet(xdest,1,0)/s;
	cvReleaseMat(&X);
	cvReleaseMat(&xdest);
	return a;
}

void drawBoder(const CvMat* M,string filename)
{
	CvPoint2D32f temp;
	temp.x=0;
	temp.y=0;
	CvPoint2D32f a1=transferRev(M,temp);
	temp.x=1366;
	temp.y=0;
	CvPoint2D32f a2=transferRev(M,temp);
	temp.x=1366;
	temp.y=768;
	CvPoint2D32f a3=transferRev(M,temp);
	temp.x=0;
	temp.y=768;
	CvPoint2D32f a4=transferRev(M,temp);
	CvSize imagesize;
	imagesize.width=640;
	imagesize.height=480;
	IplImage* img=cvLoadImage(filename.c_str());
	CvPoint curve1[]= {a1.x,a1.y, a2.x,a2.y, a3.x,a3.y, a4.x,a4.y};
    CvPoint* curvearr[1]={curve1};
    int ncurvepts[1]={4};
    int ncurves= 1;
    int iscurveclosed= 1;
    int linewidth=2;
    cvPolyLine(img,//图像
        curvearr,//折线的顶点指针数组
        ncurvepts,//折线的定点个数数组大小,pts指针数组大小
        ncurves,//折线的线段数量
        iscurveclosed,//是否封闭，如果封闭，开始点和结束点连线
        cvScalar(0,0,255),//折线的颜色
        linewidth//线条的粗细程度
        );//绘制简单线段或折线
	cvNamedWindow("show",CV_WINDOW_AUTOSIZE);
	cvShowImage("show",img);
	cvWaitKey(0);
	cvReleaseImage(&img);
	cvDestroyWindow("show");
}

CvMat** calcuInMatrix()
{
	string filename;
	while (getline(fin,filename))
	{
		image_count++;
		Image<uchar> view(filename);
		if (image_count == 1) 
		{
			image_size.width = view.size().width;
			image_size.height = view.size().height;
		}

		int count;
 		 
	 	img_gray = cvCreateImage(cvSize(image_size.width, image_size.height), IPL_DEPTH_8U, 1);
 		cvCvtColor(view.cvimage, img_gray, CV_BGR2GRAY);

		cvFindChessboardCorners( view.cvimage, board_size,
            image_points_buf, &count, CV_CALIB_CB_ADAPTIVE_THRESH );
		cvFindCornerSubPix( img_gray, image_points_buf, count, cvSize(11,11),
					cvSize(-1,-1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
				image_points_seq.push_back(image_points_buf,count);
		cvDrawChessboardCorners( view.cvimage, board_size, image_points_buf, count, 1);
//		cvNamedWindow("show",CV_WINDOW_AUTOSIZE);
//		cvShowImage("show",view.cvimage);
//		cvWaitKey(0);
	}	
	delete []image_points_buf;
 	CvSize square_size = cvSize(91,91);  /* 实际测量得到的定标板上每个棋盘格的大小 */
	Matrix<double> object_points(1,board_size.width*board_size.height*image_count,3); /* 保存定标板上角点的三维坐标 */
 	Matrix<double> image_points(1,image_points_seq.cvseq->total,2); /* 保存提取的所有角点 */
 	Matrix<int> point_counts(1,image_count,1); /* 每幅图像中角点的数量 */
 	Matrix<double> intrinsic_matrix(3,3,1); /* 摄像机内参数矩阵 */
 	Matrix<double> distortion_coeffs(1,4,1); /* 摄像机的4个畸变系数：k1,k2,p1,p2 */
 	Matrix<double> rotation_vectors(1,image_count,3); /* 每幅图像的旋转向量 */
 	Matrix<double> translation_vectors(1,image_count,3); /* 每幅图像的平移向量 */	
 
 	/* 初始化定标板上角点的三维坐标 */
 	int i,j,t;
 	for (t=0;t<image_count;t++) 
	{
 		for (i=0;i<board_size.height;i++) 
		{
 			for (j=0;j<board_size.width;j++) 
			{
 				/* 假设定标板放在世界坐标系中z=0的平面上 */
 				object_points(0,t*board_size.height*board_size.width+i*board_size.width+j,0) = j*square_size.width+square_size.width+9;
				object_points(0,t*board_size.height*board_size.width+i*board_size.width+j,1) = i*square_size.height+square_size.height+9;
 				object_points(0,t*board_size.height*board_size.width+i*board_size.width+j,2) = 0;
 			}
 		}
 	}
 	
 	/* 将角点的存储结构转换成矩阵形式 */
 	for (i=0;i<image_points_seq.cvseq->total;i++) {
 		image_points(0,i,0) = image_points_seq[i].x;
 		image_points(0,i,1) = image_points_seq[i].y;
 	}
	
	 	/* 初始化每幅图像中的角点数量，这里我们假设每幅图像中都可以看到完整的定标板 */
 	for (i=0;i<image_count;i++)
 		point_counts(0,i) = board_size.width*board_size.height;
 	
 	/* 开始定标 */
 	cvCalibrateCamera2(object_points.cvmat,
 					   image_points.cvmat,
                       point_counts.cvmat,
					   image_size,
                       intrinsic_matrix.cvmat,
 					   distortion_coeffs.cvmat,
                       rotation_vectors.cvmat,
 					   translation_vectors.cvmat,
 					   0);
	CvMat** inner=new CvMat*[2];
	inner[0]=cvCloneMat(intrinsic_matrix.cvmat);
	inner[1]=cvCloneMat(distortion_coeffs.cvmat);
	image_points_seq.clear();
	return inner;
}

CvMat* getRotAndTrans(string filename,const CvMat* M,const CvMat* W)
{
	image_points_buf = new CvPoint2D32f[board_size.width*board_size.height];
	image_points_seq.clear();

	Image<uchar> view(filename);
	image_size.width = view.size().width;
	image_size.height = view.size().height;

	int count;
 		 
	img_gray = cvCreateImage(cvSize(image_size.width, image_size.height), IPL_DEPTH_8U, 1);
 	cvCvtColor(view.cvimage, img_gray, CV_BGR2GRAY);

	cvFindChessboardCorners( view.cvimage, board_size,
        image_points_buf, &count, CV_CALIB_CB_ADAPTIVE_THRESH );
	cvFindCornerSubPix( img_gray, image_points_buf, count, cvSize(11,11),
				cvSize(-1,-1), cvTermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
			image_points_seq.push_back(image_points_buf,count);
	cvDrawChessboardCorners( view.cvimage, board_size, image_points_buf, count, 1);

	delete []image_points_buf;

 	CvSize square_size = cvSize(91,91);  /* 实际测量得到的定标板上每个棋盘格的大小 */
	Matrix<double> object_points(1,board_size.width*board_size.height,3); /* 保存定标板上角点的三维坐标 */
 	Matrix<double> image_points(1,board_size.width*board_size.height,2); /* 保存提取的所有角点 */
 	Matrix<double> rotation_vectors(1,1,3); /* 每幅图像的旋转向量 */
 	Matrix<double> translation_vectors(1,1,3); /* 每幅图像的平移向量 */	
 
 	/* 初始化定标板上角点的三维坐标 */
 	int i,j;
 	for (i=0;i<board_size.height;i++) 
	{
 		for (j=0;j<board_size.width;j++) 
		{
 			/* 假设定标板放在世界坐标系中z=0的平面上 */
 			object_points(0,i*board_size.width+j,0) = j*square_size.width+square_size.width+9;
			object_points(0,i*board_size.width+j,1) = i*square_size.height+square_size.height+9;
 			object_points(0,i*board_size.width+j,2) = 0;
 		}
 	}
 	
 	/* 将角点的存储结构转换成矩阵形式 */
 	for (i=0;i<board_size.width*board_size.height;i++) {
 		image_points(0,i,0) = image_points_seq[i].x;
 		image_points(0,i,1) = image_points_seq[i].y;
 	}

 	cvFindExtrinsicCameraParams2(object_points.cvmat,image_points.cvmat,M,W,rotation_vectors.cvmat,translation_vectors.cvmat);


	Matrix<double> rotation_vector(3,1);
	Matrix<double> rotation_matrix(3,3);

	for (j=0;j<3;j++) {
		rotation_vector(j,0,0) = rotation_vectors(0,0,j);
	}
	/* 将旋转向量转换为相对应的旋转矩阵 */
	cvRodrigues2(rotation_vector.cvmat,rotation_matrix.cvmat);
	cvmSet(rotation_matrix.cvmat,0,2,translation_vectors(0,0,0));
	cvmSet(rotation_matrix.cvmat,1,2,translation_vectors(0,0,1));
	cvmSet(rotation_matrix.cvmat,2,2,translation_vectors(0,0,2));
	image_points_seq.clear();
	CvMat* rotandtrans=cvCloneMat(rotation_matrix.cvmat);
	return rotandtrans;
}



int getFinger(CvPoint2D32f* finger,CvPoint2D32f pCenter,const CvMat* M)
{
	INuiSensor * pNuiSensor = NULL;
    HRESULT hr;

    int iSensorCount = 0;
    hr = NuiGetSensorCount(&iSensorCount);
    if (FAILED(hr) ) { return hr; }

    // Look at each Kinect sensor
    for (int i = 0; i < iSensorCount; ++i)
    {
        // Create the sensor so we can check status, if we can't create it, move on to the next
        hr = NuiCreateSensorByIndex(i, &pNuiSensor);
        if (FAILED(hr))
        {
            continue;
        }

        // Get the status of the sensor, and if connected, then we can initialize it
        hr = pNuiSensor->NuiStatus();
        if (S_OK == hr)
        {
            break;
        }

        // This sensor wasn't OK, so release it since we're not using it
        pNuiSensor->Release();
    }
	Mat image;
	image.create(480, 640, CV_8UC1); 
 
    //1、初始化NUI，注意这里是DEPTH_AND_PLAYER_INDEX
    hr = pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX); 
    if (FAILED(hr)) 
    { 
        cout<<"NuiInitialize failed"<<endl; 
        return hr; 
    } 

    //2、定义事件句柄 
	//创建读取下一帧的信号事件句柄，控制KINECT是否可以开始读取下一帧数据
    HANDLE nextColorFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
    HANDLE depthStreamHandle = NULL; //保存图像数据流的句柄，用以提取数据 
 
    //3、打开KINECT设备的彩色图信息通道，并用depthStreamHandle保存该流的句柄，以便于以后读取
	hr = pNuiSensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX, NUI_IMAGE_RESOLUTION_640x480, 
							0, 2, nextColorFrameEvent, &depthStreamHandle); 
    if( FAILED( hr ) )//判断是否提取正确 
    { 
        cout<<"Could not open color image stream video"<<endl; 
        NuiShutdown(); 
        return hr; 
    }
	namedWindow("depthImage", CV_WINDOW_AUTOSIZE);
	point* center=new point();
	point* lastCenter=new point();
	lastCenter->x=pCenter.x;
	lastCenter->y=pCenter.y;
	CvPoint2D32f dest[4];
				CvPoint2D32f src[4];
				src[0].x=0;
				src[0].y=0;
				dest[0]=transferRev(single,src[0]);
				src[1].x=1366;
				src[1].y=0;
				dest[1]=transferRev(single,src[1]);
				src[2].x=0;
				src[2].y=768;
				dest[2]=transferRev(single,src[2]);
				src[3].x=1366;
				src[3].y=768;
				dest[3]=transferRev(single,src[3]);
				int max_x=0;
				int min_x=10000;
				int max_y=0;
				int min_y=10000;
				for(int i=0;i<4;++i)
				{
					if(dest[i].x>max_x)
						max_x=dest[i].x;
					if(dest[i].y>max_y)
						max_y=dest[i].y;
				}
				for(int i=0;i<4;++i)
				{
					if(dest[i].x<min_x)
						min_x=dest[i].x;
					if(dest[i].y<min_y)
						min_y=dest[i].y;
				}
				int middle;
				int flag=0;
				bool lastkey=false;
				bool getfirst=false;
				bool twofirst=true;
				int count=0;
				double deta=0;
/*				Mat mDepth;
				Mat mRgb;
				kinect->init(640,480);*/
				long* m_colorCoordinates=new long[640*480*2];
				USHORT pBufferArg[640*480];
				USHORT *Erroe=new USHORT[640*480];
				USHORT *Last=new USHORT[640*480];
				for(int i=0;i<640*480;++i)
				{
					pBufferArg[i]=0;
					Erroe[i]=0;
					Last[i]=0;
				}
				bool firstErroe=false;
	while(count<30) 
    { 
		count++;
        const NUI_IMAGE_FRAME * pImageFrame = NULL; 
		//4.1、无限等待新的数据，等到后返回

		if (WaitForSingleObject(nextColorFrameEvent, INFINITE)==0) 
        { 
			//4.2、从刚才打开数据流的流句柄中得到该帧数据，读取到的数据地址存于pImageFrame
            hr = NuiImageStreamGetNextFrame(depthStreamHandle, 0, &pImageFrame); 
			if (FAILED(hr))
			{
				cout<<"Could not get depth image"<<endl; 
				NuiShutdown();
				return -1;
			}

			INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
			NUI_LOCKED_RECT LockedRect;

			//4.3、提取数据帧到LockedRect，它包括两个数据对象：pitch每行字节数，pBits第一个字节地址
			//并锁定数据，这样当我们读数据的时候，kinect就不会去修改它
            pTexture->LockRect(0, &LockedRect, NULL, 0); 
			
			//4.4、确认获得的数据是否有效
            if( LockedRect.Pitch != 0 ) 
            { 
				uchar *pBufferRun = (uchar*)(LockedRect.pBits);
				USHORT * pBuffer = (USHORT*) pBufferRun;
				for(int i=0;i<640*480;++i)
				{
					pBufferArg[i]+=pBuffer[i]>>3;
					if(firstErroe==false)
					{
						Last[i]=pBuffer[i]>>3;
					}
					else
					{
						Erroe[i]+=abs((pBuffer[i]>>3)-Last[i]);
						Last[i]=pBuffer[i]>>3;
					}
				}
				firstErroe=true;
			} 
            else 
            { 
                cout<<"Buffer length of received texture is bogus\r\n"<<endl; 
            }

			//5、这帧已经处理完了，所以将其解锁
			pTexture->UnlockRect(0);
            //6、释放本帧数据，准备迎接下一帧 
            NuiImageStreamReleaseFrame(depthStreamHandle, pImageFrame ); 
        }      
				
    } 
	for(int i=0;i<640*480;++i)
	{
		pBufferArg[i]/=30;
		pBufferArg[i]=pBufferArg[i]<<3;
		Erroe[i]/=29;
	}
    //4、开始读取深度数据 
    while(1) 
    { 
		center->x=lastCenter->x;
		center->y=lastCenter->y;
        const NUI_IMAGE_FRAME * pImageFrame = NULL; 
		//4.1、无限等待新的数据，等到后返回

		if (WaitForSingleObject(nextColorFrameEvent, INFINITE)==0) 
        { 
			//4.2、从刚才打开数据流的流句柄中得到该帧数据，读取到的数据地址存于pImageFrame
            hr = NuiImageStreamGetNextFrame(depthStreamHandle, 0, &pImageFrame); 
			if (FAILED(hr))
			{
				cout<<"Could not get depth image"<<endl; 
				NuiShutdown();
				return -1;
			}

			INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
			NUI_LOCKED_RECT LockedRect;

			//4.3、提取数据帧到LockedRect，它包括两个数据对象：pitch每行字节数，pBits第一个字节地址
			//并锁定数据，这样当我们读数据的时候，kinect就不会去修改它
            pTexture->LockRect(0, &LockedRect, NULL, 0); 
			
			//4.4、确认获得的数据是否有效
            if( LockedRect.Pitch != 0 ) 
            { 

				uchar *pBufferRun = (uchar*)(LockedRect.pBits);
				USHORT * pBuffer = (USHORT*) pBufferRun;
//				if(getfirst==false)
				pNuiSensor->NuiImageGetColorPixelCoordinateFrameFromDepthPixelFrameAtResolution(
								NUI_IMAGE_RESOLUTION_640x480,
                                NUI_IMAGE_RESOLUTION_640x480,
								640*480,
								pBuffer,
								640*480*2,
								m_colorCoordinates
								);
				//4.5、将数据转换为OpenCV的Mat格式
				for (int i=0; i<image.rows; i++) 
                {				
					//其二是既表示深度值又含有人物序号，则像素值的高13位保存了深度值，低三位保存用户序号，
					//注意这里需要转换，因为每个数据是2个字节，存储的同上面的颜色信息不一样，
                    uchar *pBufferRun = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;
					USHORT * pBuffer = (USHORT*) pBufferRun;
					for (int j=0; j<image.cols; j++)   
                    {   
/*						long lX,lY;
						if(getfirst==false)
							NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(
								NUI_IMAGE_RESOLUTION_640x480,
                                NUI_IMAGE_RESOLUTION_640x480,
                                NULL,
                                i,
                                j,
                                pBuffer[j],
                                &lX,
                                &lY
                                );//矫正深度图像
						else
						{
							lX=depthtoimage[i*image.cols+j].x;
							lY=depthtoimage[i*image.cols+j].y;
						}*/
						if(m_colorCoordinates[(i*image.cols+j)*2+1]<480&&m_colorCoordinates[(i*image.cols+j)*2+1]>=0&&m_colorCoordinates[(i*image.cols+j)*2]<640&&m_colorCoordinates[(i*image.cols+j)*2]>=0)
							bits[m_colorCoordinates[(i*image.cols+j)*2+1]*image.cols+m_colorCoordinates[(i*image.cols+j)*2]]=(pBuffer[j] & 0xfff8) >> 3;
						if(getfirst==false)
						{
//							depthtoimage[i*image.cols+j].x=lX;
//							depthtoimage[i*image.cols+j].y=lY;
							if(m_colorCoordinates[(i*image.cols+j)*2+1]<480&&m_colorCoordinates[(i*image.cols+j)*2+1]>=0&&m_colorCoordinates[(i*image.cols+j)*2]<640&&m_colorCoordinates[(i*image.cols+j)*2]>=0)
								bitsfirst[m_colorCoordinates[(i*image.cols+j)*2+1]*image.cols+m_colorCoordinates[(i*image.cols+j)*2]]=(pBufferArg[i*640+j] & 0xfff8) >> 3;
						}
                    }  
				}
				getfirst=true;
				bool keyd=false;
				bool handin=false;
				for(int i=0;i<image.rows; i++)
				{
					uchar *ptr = image.ptr<uchar>(i);  //第i行的指针
					for(int j=0;j<image.cols; j++)
					{
						if(i>=min_y&&i<=max_y&&j<=640-min_x&&j>=640-max_x&&bits[i*image.cols+j]>(bitsfirst[i*image.cols+j]-(0.0265*bitsfirst[i*image.cols+j]+Erroe[i*image.cols+j]/100))&&bits[i*image.cols+j]<(bitsfirst[i*image.cols+j]-(0.0112*bitsfirst[i*image.cols+j]+Erroe[i*image.cols+j]/100)))
						{
							ptr[j]=0;
						}
						else
							ptr[j]=1;
					}
				}
/*				for(int i=0;i<image.rows; i++)
				{
					uchar *ptr = image.ptr<uchar>(i);  //第i行的指针
					for(int j=0;j<image.cols; j++)
					{
						bits[i*image.cols+j]*=(1-ptr[j]);
					}
				}*/
				int sum=0;
				bool flag[480][640];
				for(int k=0;k<3;++k)
				{
					uchar *ptr = image.ptr<uchar>(0);  //第i行的指针
					for(int i=0;i<image.rows; i++)
					{
						for(int j=0;j<image.cols; j++)
						{
							if(ptr[i*image.cols+(j+1)%640]+ptr[i*image.cols+j]+ptr[i*image.cols+(j-1+640)%640]+ptr[((i-1+480)%480)*image.cols+j]+ptr[((i+1)%480)*image.cols+j]==0)
								flag[i][j]=true;
							else
								flag[i][j]=false;
						}
					}
					for(int i=0;i<image.rows; i++)
					{
						for(int j=0;j<image.cols; j++)
						{
							if(flag[i][j])
								ptr[i*image.cols+j]=0;
							else
								ptr[i*image.cols+j]=1;
						}
					}
				}
				for(int k=0;k<3;++k)
				{
					uchar *ptr = image.ptr<uchar>(0);  //第i行的指针
					for(int i=0;i<image.rows; i++)
					{
						for(int j=0;j<image.cols; j++)
						{
							if(ptr[i*image.cols+(j+1)%640]+ptr[i*image.cols+j]+ptr[i*image.cols+(j-1+640)%640]+ptr[((i-1+480)%480)*image.cols+j]+ptr[((i+1)%480)*image.cols+j]<5)
								flag[i][j]=true;
							else
								flag[i][j]=false;
						}
					}
					for(int i=0;i<image.rows; i++)
					{
						for(int j=0;j<image.cols; j++)
						{
							if(flag[i][j])
								ptr[i*image.cols+j]=0;
							else
								ptr[i*image.cols+j]=1;
						}
					}
				}
				for(int i=1;i<image.rows-1; i++)
				{
					uchar *ptr = image.ptr<uchar>(i);  //第i行的指针
					for(int j=1;j<image.cols-1; j++)
					{
						sum+=(1-ptr[j]);
					}
				}
				if(sum>=2)
				{
					handin=true;
				}
				if(sum>=3)
				{
					keyd=true;
				}
				
				for(int i=0;i<image.rows; i++)
				{
					uchar *ptr = image.ptr<uchar>(i);  //第i行的指针
					for(int j=0;j<image.cols; j++)
					{
						bits[i*image.cols+j]*=(1-ptr[j]);
					}
				}
				
				bool find=false;
				int count=0;
				point* first=new point();
				point* last=new point();
				uchar *ptr = image.ptr<uchar>(0);
				point pf;
				bool twohands=findTwoHand(ptr,pf);


				for(int i=1;i<image.rows-1; i++)
				{
					uchar *ptr = image.ptr<uchar>(0);  //第i行的指针
					for(int j=1;j<image.cols-1; j++)
					{
						if(ptr[i*image.cols+j]==0)
						{
							if(ptr[i*image.cols+j+1]!=1&&ptr[i*image.cols+j-1]!=1&&ptr[(i+1)*image.cols+j]!=1&&ptr[(i-1)*image.cols+j]!=1)
							{
								point* p=new point(i,j);
								inhand.push_back(p);
							}
							else
							{
								count++;
								point* p1=new point(i,j);
								borderunsort.push_back(p1);
								ptr[i*image.cols+j]=255;
							}
						}
					}
				}
				ptr = image.ptr<uchar>(0);  //第i行的指针
				USHORT min=10000;
				USHORT max=0;
				point* center=new point();
				int m=borderunsort.size();
				int d=1;
				for(int i=0;i<inhand.size();++i)
				{
					min=10000;
					
					for(int j=0;j<m;j+=d)
					{
						if(((inhand[i]->x-borderunsort[j]->x)*(inhand[i]->x-borderunsort[j]->x)+(inhand[i]->y-borderunsort[j]->y)*(inhand[i]->y-borderunsort[j]->y))<min)
						{
							min=(inhand[i]->x-borderunsort[j]->x)*(inhand[i]->x-borderunsort[j]->x)+(inhand[i]->y-borderunsort[j]->y)*(inhand[i]->y-borderunsort[j]->y);		
						}
						if(min<max)
							break;
					}
					if(min>max)
					{
						max=min;
						center->x=inhand[i]->x;
						center->y=inhand[i]->y;
					}
				}
/*				if(max<1||abs(center->x-lastCenter->x)<5&&abs(center->y-lastCenter->y)<5)
				{
					center->x=lastCenter->x;
					center->y=lastCenter->y;
				}
				else
				{
					lastCenter->x=center->x;
					lastCenter->y=center->y;
				}*/

				max=0;
				CvPoint2D32f temp_center;
				temp_center.x=lastCenter->x;
				temp_center.y=lastCenter->y;
				for(int i=0;i<image.rows; i++)
				{
					for(int j=0;j<image.cols; j++)
					{
						if(bits[i*image.cols+j]>max&&abs(center->x-i)<20&&abs(center->y-j)<20)
						{
							max=bits[i*image.cols+j];
							temp_center.x=i;
							temp_center.y=j;
						}
					}
				}
				if(abs(temp_center.x-lastCenter->x)<10&&abs(temp_center.y-lastCenter->y)<10)
				{
					temp_center.x=lastCenter->x;
					temp_center.y=lastCenter->y;
				}
				else
				{
					lastCenter->x=temp_center.x;
					lastCenter->y=temp_center.y;
				}
				int a=temp_center.x;
				int b=temp_center.y;
//				if(max>(bitsfirst[a*image.cols+b]-50*bitsfirst[a*image.cols+b]/bitsfirst[240*image.cols+320]))
//					keyd=true;
				CvPoint2D32f temp;
				temp.x=640-temp_center.y;
				temp.y=temp_center.x;
				temp=transfer(M,temp);
				if(twohands)
				{
					CvPoint2D32f temp_pf;
					temp_pf.x=640-pf.y;
					temp_pf.y=pf.x;
					temp_pf=transfer(M,temp_pf);
					pf.x=temp_pf.x;
					pf.y=temp_pf.y;
				}
				
				if(twohands)
				{
					if(twofirst)
					{
						twofirst=false;
						deta=(temp.x-pf.x)*(temp.x-pf.x)+(temp.y-pf.y)*(temp.y-pf.y);
					}
					if((temp.x-pf.x)*(temp.x-pf.x)+(temp.y-pf.y)*(temp.y-pf.y)-deta>12)
					{
						SetCursorPos((temp.x+pf.x)/2,(temp.y+pf.y)/2);
						//keybd_event(VK_CONTROL,0,0,0); 
						mouse_event(MOUSEEVENTF_WHEEL,0,0,120,0);
						deta=(temp.x-pf.x)*(temp.x-pf.x)+(temp.y-pf.y)*(temp.y-pf.y);
					}
					else if(deta-(temp.x-pf.x)*(temp.x-pf.x)+(temp.y-pf.y)*(temp.y-pf.y)>17)
					{
						SetCursorPos((temp.x+pf.x)/2,(temp.y+pf.y)/2);
						//keybd_event(VK_CONTROL,0,0,0); 
						mouse_event(MOUSEEVENTF_WHEEL,0,0,-120,0);
						deta=(temp.x-pf.x)*(temp.x-pf.x)+(temp.y-pf.y)*(temp.y-pf.y);
					}
				}
				else
				{
					if(handin)
					{
					SetCursorPos(temp.x,temp.y);
					if(keyd)
					{
						mouse_event(0x0002,0,0,0,0);
					}
					else
					{
						mouse_event(0x0004,0,0,0,0);
					}
					}
				else
				{
					if(lastkey)
						mouse_event(0x0004,0,0,0,0);
				}
				}
				if(twohands==false)
				{
					deta=0;
					//keybd_event(VK_CONTROL,0,KEYEVENTF_KEYUP,0);
					twofirst=true;
				}
				lastkey=keyd;
				int det=10;

				
//				for(int i=0;i<fingertips.size();++i)
//				{
					
//				}
				for(int i=0;i<inhand.size();++i)
				{
					ptr[inhand[i]->x*640+inhand[i]->y]=100;
				}
				ptr[center->x*640+center->y]=255;
				for(int i=0;i<10;++i)
				{
					if(center->x<=10||center->y<=10)
						break;
					ptr[(center->x-i)*640+center->y]=255;
					ptr[center->x*640+center->y-i]=255;
					ptr[(center->x+i)*640+center->y]=255;
					ptr[center->x*640+center->y+i]=255;
				}

				fingertips.clear();
				inhand.clear();
				borderunsort.clear();
				finger->x=center->x;
				finger->y=center->y;
				delete center;
				delete first;
				delete last;
//				delete []m_colorCoordinates;
                imshow("depthImage", image); //显示图像 
//				system("pause");
			} 
            else 
            { 
                cout<<"Buffer length of received texture is bogus\r\n"<<endl; 
            }

			//5、这帧已经处理完了，所以将其解锁
			pTexture->UnlockRect(0);
            //6、释放本帧数据，准备迎接下一帧 
            NuiImageStreamReleaseFrame(depthStreamHandle, pImageFrame ); 
        } 
        if (cvWaitKey(20) == 27) 
            break;     
				
    } 

	return 0;
}

bool findTwoHand(uchar *ptr,point &pf)
{
	point* first=findFirstHand(ptr);
	if(first==NULL)
	{
		return false;
	}
	pf.x=first->x;
	pf.y=first->y;
	setFild(ptr,first->x,first->y,0,2);
	
	point* secon=findFirstHand(ptr);
	if(secon==NULL)
	{
		setFild(ptr,first->x,first->y,2,0);
		delete first;
		return false;
	}
	delete first;
	delete secon;
	return true;
}

void setFild(uchar *ptr,int i,int j,int org,int num)
{
	for(int m=i-1;m<=i+1;++m)
		for(int n=j-1;n<=j+1;++n)
		{
			if(ptr[m*640+n]==org)
			{
				ptr[m*640+n]=num;
				setFild(ptr,m,n,org,num);
			}
		}
}

point* findFirstHand(uchar *ptr)
{
	for(int i=1;i<480-1;++i)
		for(int j=1;j<640-1;++j)
		{
			if(ptr[i*640+j]==0)
			{
				point* temp=new point(i,j);
				return temp;
			}
		}
	return NULL;
}