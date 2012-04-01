/**
	author @kassy708
	OpenGLとKinect SDK v1.0を使ったポイントクラウド
*/

#include<Windows.h>
#include<NuiApi.h>	//KinectSDK利用時にinclude
#pragma comment(lib,"Kinect10.lib")

#include <GL/glut.h>
#include <opencv2/opencv.hpp>
#ifdef _DEBUG
    //Debugモードの場合
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_core220d.lib")            // opencv_core
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_imgproc220d.lib")        // opencv_imgproc
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_highgui220d.lib")        // opencv_highgui
#else
    //Releaseモードの場合
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_core220.lib")            // opencv_core
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_imgproc220.lib")        // opencv_imgproc
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_highgui220.lib")        // opencv_highgui
#endif

using namespace cv;

//openNIのための宣言・定義
//マクロ定義
#define KINECT_IMAGE_WIDTH 640
#define KINECT_IMAGE_HEGIHT 480
#define KINECT_DEPTH_WIDTH 640
#define KINECT_DEPTH_HEGIHT 480
//#define KINECT_DEPTH_WIDTH 320
//#define KINECT_DEPTH_HEGIHT 240

Mat image(KINECT_IMAGE_HEGIHT,KINECT_IMAGE_WIDTH,CV_8UC4);
Mat depth(KINECT_DEPTH_HEGIHT,KINECT_DEPTH_WIDTH,CV_16UC1);
//ポイントクラウドの座標
Mat pointCloud_XYZ(KINECT_DEPTH_HEGIHT,KINECT_DEPTH_WIDTH,CV_32FC3,cv::Scalar::all(0));

void retrievePointCloudMap(Mat &depth,Mat &pointCloud_XYZ);		//3次元ポイントクラウドのための座標変換
void drawPointCloud(Mat &rgbImage,Mat &pointCloud_XYZ);		//ポイントクラウド描画

//RGBデータを取得する関数
int GetRGBImage(cv::Mat &image);
int GetDepthImage(cv::Mat &depth);

//openGLのための宣言・定義
//---変数宣言---
int FormWidth = 640;
int FormHeight = 480;
int mButton;
float twist, elevation, azimuth;
float cameraDistance = 0,cameraX = 0,cameraY = 0;
int xBegin, yBegin;
//---マクロ定義---
#define glFovy 45		//視角度
#define glZNear 1.0		//near面の距離
#define glZFar 150.0	//far面の距離
void polarview();		//視点変更

//ハンドル
HANDLE m_hNextImageFrameEvent;
HANDLE m_hNextDepthFrameEvent;
HANDLE m_pImageStreamHandle;
HANDLE m_pDepthStreamHandle;

//描画
void display(){
    // clear screen and depth buffer
    glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    // Reset the coordinate system before modifying
    glLoadIdentity(); 
    glEnable(GL_DEPTH_TEST); //「Zバッファ」を有効
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);   //視点の向き設定

	//取得をする
	if(GetRGBImage(image)==-1)
		return;
	if(GetDepthImage(depth)==-1)
		return;

    //3次元ポイントクラウドのための座標変換
    retrievePointCloudMap(depth,pointCloud_XYZ);

    //視点の変更
    polarview();

    imshow("image",image);
    imshow("depth",depth);
	
	//RGBAからBGRAに変換
    cvtColor(image,image,CV_RGBA2BGRA);  

    //ポイントクラウド
    drawPointCloud(image,pointCloud_XYZ);
   
 
    glFlush();
    glutSwapBuffers();
}
//初期化
int init(){	
	//Kinectの初期化関数
	NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON );

	//各ハンドルの設定
	m_hNextImageFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_pImageStreamHandle   = NULL;
	m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_pDepthStreamHandle   = NULL;

	//深度センサストリームの設定
	HRESULT hr;
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR , NUI_IMAGE_RESOLUTION_640x480 , 0 , 2 , m_hNextImageFrameEvent , &m_pImageStreamHandle );
	if( FAILED( hr ) ) 
		return -1;//取得失敗
#if KINECT_DEPTH_WIDTH == 320
	//奥行き画像の解像度が320x240の場合
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX , NUI_IMAGE_RESOLUTION_320x240 , 0 , 2 , m_hNextDepthFrameEvent , &m_pDepthStreamHandle );
#else if KINECT_DEPTH_WIDTH == 640
	//奥行き画像の解像度が640x480の場合
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX , NUI_IMAGE_RESOLUTION_640x480 , 0 , 2 , m_hNextDepthFrameEvent , &m_pDepthStreamHandle );
#endif
	if( FAILED( hr ) ) 
		return -1;//取得失敗

	return 1;
}
// アイドル時のコールバック
void idle(){
    //再描画要求
    glutPostRedisplay();
}
//ウィンドウのサイズ変更
void reshape (int width, int height){
    FormWidth = width;
    FormHeight = height;
    glViewport (0, 0, (GLsizei)width, (GLsizei)height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    //射影変換行列の指定
    gluPerspective (glFovy, (GLfloat)width / (GLfloat)height,glZNear,glZFar);
    glMatrixMode (GL_MODELVIEW);
}
//マウスの動き
void motion(int x, int y){
    int xDisp, yDisp;
    xDisp = x - xBegin;
    yDisp = y - yBegin;
    switch (mButton) {
    case GLUT_LEFT_BUTTON:
        azimuth += (float) xDisp/2.0;
        elevation -= (float) yDisp/2.0;
        break;
    case GLUT_MIDDLE_BUTTON:
        cameraX -= (float) xDisp/40.0;
        cameraY += (float) yDisp/40.0;
        break;
    case GLUT_RIGHT_BUTTON:
		cameraDistance += xDisp/40.0;
        break;
    }
    xBegin = x;
    yBegin = y;
}
//マウスの操作
void mouse(int button, int state, int x, int y){
    if (state == GLUT_DOWN) {
        switch(button) {
        case GLUT_RIGHT_BUTTON:
        case GLUT_MIDDLE_BUTTON:
        case GLUT_LEFT_BUTTON:
            mButton = button;
            break;
        }
        xBegin = x;
        yBegin = y;
    }
}
//視点変更
void polarview(){
    glTranslatef( cameraX, cameraY, cameraDistance);
    glRotatef( -twist, 0.0, 0.0, 1.0);
    glRotatef( -elevation, 1.0, 0.0, 0.0);
    glRotatef( -azimuth, 0.0, 1.0, 0.0);
}
//メイン
int main(int argc, char *argv[]){
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(FormWidth, FormHeight);
    glutCreateWindow(argv[0]);
    //コールバック
    glutReshapeFunc (reshape);
    glutDisplayFunc(display);
    glutIdleFunc(idle);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    init();
    glutMainLoop();
	
	NuiShutdown();
    return 0;
}


int GetRGBImage(cv::Mat &image){
//フレームを入れるクラス
	const NUI_IMAGE_FRAME *pImageFrame = NULL;
	
	//次のRGBフレームが来るまで待機
	WaitForSingleObject(m_hNextImageFrameEvent,INFINITE);
	HRESULT hr = NuiImageStreamGetNextFrame(m_pImageStreamHandle, 30 , &pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//取得失敗

	//フレームから画像データの取得
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    
	if( LockedRect.Pitch != 0 ){
		//pBitsに画像データが入っている
		BYTE *pBuffer = (BYTE*) LockedRect.pBits;
		memcpy(image.data,pBuffer,image.step * image.rows);
	}

	hr = NuiImageStreamReleaseFrame( m_pImageStreamHandle, pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//取得失敗

	return 0;
}
int GetDepthImage(cv::Mat &depth){
	//フレームを入れるクラス
	const NUI_IMAGE_FRAME *pImageFrame = NULL;
	
	WaitForSingleObject(m_hNextDepthFrameEvent,INFINITE);
	HRESULT hr = NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 30 , &pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//取得失敗

	//フレームから画像データの取得
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );

	if( LockedRect.Pitch != 0 ){
		//pBitsに画像データが入っている
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		memcpy( depth.data, (BYTE*)pBuffer, depth.step * depth.rows );
	 }

    hr = NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//取得失敗

	return 0;
}


//ポイントクラウド描画
void drawPointCloud(Mat &rgbImage,Mat &pointCloud_XYZ){
    static int x,y;
    glPointSize(2);
    glBegin(GL_POINTS);
    uchar *p;
    Point3f *point = (Point3f*)pointCloud_XYZ.data;
	LONG colorX,colorY;
    for(y = 0;y < pointCloud_XYZ.rows;y++){
		for(x = 0;x < pointCloud_XYZ.cols;x++,point++){
            if(point->z == 0)
                continue;
			//ズレを直す
#if KINECT_DEPTH_WIDTH == 320
			//奥行き画像の解像度が320x240の場合
			NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480,NULL,x,y,0,&colorX,&colorY);
#else if KINECT_DEPTH_WIDTH == 640
			//奥行き画像の解像度が640x480の場合
			NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_640x480,NULL,x,y,0,&colorX,&colorY);
#endif
			//画像内の場合
			if(0 <= colorX && colorX <= rgbImage.cols && 0 <= colorY && colorY <= rgbImage.rows){
				p = &rgbImage.data[colorY * rgbImage.step + colorX * rgbImage.channels()];
				glColor3ubv(p);
				glVertex3f(point->x,point->y,point->z);
			}
        }
    }
    glEnd();
}
//3次元ポイントクラウドのための座標変換
void retrievePointCloudMap(Mat &depth,Mat &pointCloud_XYZ){
    unsigned short* dp = (unsigned short*)depth.data;
	Point3f *point = (Point3f *)pointCloud_XYZ.data;
	for(int y = 0;y < depth.rows;y++){
		for(int x = 0;x < depth.cols;x++, dp++,point++){
#if KINECT_DEPTH_WIDTH == 320
			//奥行き画像の解像度が320x240の場合
			Vector4 RealPoints = NuiTransformDepthImageToSkeleton(x,y,*dp);
#else if KINECT_DEPTH_WIDTH == 640
			//奥行き画像の解像度が640x480の場合
			Vector4 RealPoints = NuiTransformDepthImageToSkeleton(x,y,*dp, NUI_IMAGE_RESOLUTION_640x480);
#endif
			point->x = RealPoints.x;
			point->y = RealPoints.y;
			point->z = RealPoints.z;
		}
	}
}   
