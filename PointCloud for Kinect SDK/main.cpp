/**
	author @kassy708
	OpenGL��Kinect SDK v1.0���g�����|�C���g�N���E�h
*/

#include<Windows.h>
#include<NuiApi.h>	//KinectSDK���p����include
#pragma comment(lib,"Kinect10.lib")

#include <GL/glut.h>
#include <opencv2/opencv.hpp>
#ifdef _DEBUG
    //Debug���[�h�̏ꍇ
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_core220d.lib")            // opencv_core
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_imgproc220d.lib")        // opencv_imgproc
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_highgui220d.lib")        // opencv_highgui
#else
    //Release���[�h�̏ꍇ
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_core220.lib")            // opencv_core
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_imgproc220.lib")        // opencv_imgproc
    #pragma comment(lib,"C:\\OpenCV2.2\\lib\\opencv_highgui220.lib")        // opencv_highgui
#endif

using namespace cv;

//openNI�̂��߂̐錾�E��`
//�}�N����`
#define KINECT_IMAGE_WIDTH 640
#define KINECT_IMAGE_HEGIHT 480
#define KINECT_DEPTH_WIDTH 640
#define KINECT_DEPTH_HEGIHT 480
//#define KINECT_DEPTH_WIDTH 320
//#define KINECT_DEPTH_HEGIHT 240

Mat image(KINECT_IMAGE_HEGIHT,KINECT_IMAGE_WIDTH,CV_8UC4);
Mat depth(KINECT_DEPTH_HEGIHT,KINECT_DEPTH_WIDTH,CV_16UC1);
//�|�C���g�N���E�h�̍��W
Mat pointCloud_XYZ(KINECT_DEPTH_HEGIHT,KINECT_DEPTH_WIDTH,CV_32FC3,cv::Scalar::all(0));

void retrievePointCloudMap(Mat &depth,Mat &pointCloud_XYZ);		//3�����|�C���g�N���E�h�̂��߂̍��W�ϊ�
void drawPointCloud(Mat &rgbImage,Mat &pointCloud_XYZ);		//�|�C���g�N���E�h�`��

//RGB�f�[�^���擾����֐�
int GetRGBImage(cv::Mat &image);
int GetDepthImage(cv::Mat &depth);

//openGL�̂��߂̐錾�E��`
//---�ϐ��錾---
int FormWidth = 640;
int FormHeight = 480;
int mButton;
float twist, elevation, azimuth;
float cameraDistance = 0,cameraX = 0,cameraY = 0;
int xBegin, yBegin;
//---�}�N����`---
#define glFovy 45		//���p�x
#define glZNear 1.0		//near�ʂ̋���
#define glZFar 150.0	//far�ʂ̋���
void polarview();		//���_�ύX

//�n���h��
HANDLE m_hNextImageFrameEvent;
HANDLE m_hNextDepthFrameEvent;
HANDLE m_pImageStreamHandle;
HANDLE m_pDepthStreamHandle;

//�`��
void display(){
    // clear screen and depth buffer
    glClear ( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    // Reset the coordinate system before modifying
    glLoadIdentity(); 
    glEnable(GL_DEPTH_TEST); //�uZ�o�b�t�@�v��L��
    gluLookAt(0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0);   //���_�̌����ݒ�

	//�擾������
	if(GetRGBImage(image)==-1)
		return;
	if(GetDepthImage(depth)==-1)
		return;

    //3�����|�C���g�N���E�h�̂��߂̍��W�ϊ�
    retrievePointCloudMap(depth,pointCloud_XYZ);

    //���_�̕ύX
    polarview();

    imshow("image",image);
    imshow("depth",depth);
	
	//RGBA����BGRA�ɕϊ�
    cvtColor(image,image,CV_RGBA2BGRA);  

    //�|�C���g�N���E�h
    drawPointCloud(image,pointCloud_XYZ);
   
 
    glFlush();
    glutSwapBuffers();
}
//������
int init(){	
	//Kinect�̏������֐�
	NuiInitialize(NUI_INITIALIZE_FLAG_USES_COLOR | NUI_INITIALIZE_FLAG_USES_DEPTH_AND_PLAYER_INDEX | NUI_INITIALIZE_FLAG_USES_SKELETON );

	//�e�n���h���̐ݒ�
	m_hNextImageFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_pImageStreamHandle   = NULL;
	m_hNextDepthFrameEvent = CreateEvent( NULL, TRUE, FALSE, NULL );
	m_pDepthStreamHandle   = NULL;

	//�[�x�Z���T�X�g���[���̐ݒ�
	HRESULT hr;
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR , NUI_IMAGE_RESOLUTION_640x480 , 0 , 2 , m_hNextImageFrameEvent , &m_pImageStreamHandle );
	if( FAILED( hr ) ) 
		return -1;//�擾���s
#if KINECT_DEPTH_WIDTH == 320
	//���s���摜�̉𑜓x��320x240�̏ꍇ
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX , NUI_IMAGE_RESOLUTION_320x240 , 0 , 2 , m_hNextDepthFrameEvent , &m_pDepthStreamHandle );
#else if KINECT_DEPTH_WIDTH == 640
	//���s���摜�̉𑜓x��640x480�̏ꍇ
	hr = NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH_AND_PLAYER_INDEX , NUI_IMAGE_RESOLUTION_640x480 , 0 , 2 , m_hNextDepthFrameEvent , &m_pDepthStreamHandle );
#endif
	if( FAILED( hr ) ) 
		return -1;//�擾���s

	return 1;
}
// �A�C�h�����̃R�[���o�b�N
void idle(){
    //�ĕ`��v��
    glutPostRedisplay();
}
//�E�B���h�E�̃T�C�Y�ύX
void reshape (int width, int height){
    FormWidth = width;
    FormHeight = height;
    glViewport (0, 0, (GLsizei)width, (GLsizei)height);
    glMatrixMode (GL_PROJECTION);
    glLoadIdentity ();
    //�ˉe�ϊ��s��̎w��
    gluPerspective (glFovy, (GLfloat)width / (GLfloat)height,glZNear,glZFar);
    glMatrixMode (GL_MODELVIEW);
}
//�}�E�X�̓���
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
//�}�E�X�̑���
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
//���_�ύX
void polarview(){
    glTranslatef( cameraX, cameraY, cameraDistance);
    glRotatef( -twist, 0.0, 0.0, 1.0);
    glRotatef( -elevation, 1.0, 0.0, 0.0);
    glRotatef( -azimuth, 0.0, 1.0, 0.0);
}
//���C��
int main(int argc, char *argv[]){
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(FormWidth, FormHeight);
    glutCreateWindow(argv[0]);
    //�R�[���o�b�N
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
//�t���[��������N���X
	const NUI_IMAGE_FRAME *pImageFrame = NULL;
	
	//����RGB�t���[��������܂őҋ@
	WaitForSingleObject(m_hNextImageFrameEvent,INFINITE);
	HRESULT hr = NuiImageStreamGetNextFrame(m_pImageStreamHandle, 30 , &pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//�擾���s

	//�t���[������摜�f�[�^�̎擾
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );
    
	if( LockedRect.Pitch != 0 ){
		//pBits�ɉ摜�f�[�^�������Ă���
		BYTE *pBuffer = (BYTE*) LockedRect.pBits;
		memcpy(image.data,pBuffer,image.step * image.rows);
	}

	hr = NuiImageStreamReleaseFrame( m_pImageStreamHandle, pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//�擾���s

	return 0;
}
int GetDepthImage(cv::Mat &depth){
	//�t���[��������N���X
	const NUI_IMAGE_FRAME *pImageFrame = NULL;
	
	WaitForSingleObject(m_hNextDepthFrameEvent,INFINITE);
	HRESULT hr = NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 30 , &pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//�擾���s

	//�t���[������摜�f�[�^�̎擾
    INuiFrameTexture * pTexture = pImageFrame->pFrameTexture;
    NUI_LOCKED_RECT LockedRect;
    pTexture->LockRect( 0, &LockedRect, NULL, 0 );

	if( LockedRect.Pitch != 0 ){
		//pBits�ɉ摜�f�[�^�������Ă���
		BYTE * pBuffer = (BYTE*) LockedRect.pBits;
		memcpy( depth.data, (BYTE*)pBuffer, depth.step * depth.rows );
	 }

    hr = NuiImageStreamReleaseFrame( m_pDepthStreamHandle, pImageFrame );
	if( FAILED( hr ) ) 
		return -1;//�擾���s

	return 0;
}


//�|�C���g�N���E�h�`��
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
			//�Y���𒼂�
#if KINECT_DEPTH_WIDTH == 320
			//���s���摜�̉𑜓x��320x240�̏ꍇ
			NuiImageGetColorPixelCoordinatesFromDepthPixel(NUI_IMAGE_RESOLUTION_640x480,NULL,x,y,0,&colorX,&colorY);
#else if KINECT_DEPTH_WIDTH == 640
			//���s���摜�̉𑜓x��640x480�̏ꍇ
			NuiImageGetColorPixelCoordinatesFromDepthPixelAtResolution(NUI_IMAGE_RESOLUTION_640x480,NUI_IMAGE_RESOLUTION_640x480,NULL,x,y,0,&colorX,&colorY);
#endif
			//�摜���̏ꍇ
			if(0 <= colorX && colorX <= rgbImage.cols && 0 <= colorY && colorY <= rgbImage.rows){
				p = &rgbImage.data[colorY * rgbImage.step + colorX * rgbImage.channels()];
				glColor3ubv(p);
				glVertex3f(point->x,point->y,point->z);
			}
        }
    }
    glEnd();
}
//3�����|�C���g�N���E�h�̂��߂̍��W�ϊ�
void retrievePointCloudMap(Mat &depth,Mat &pointCloud_XYZ){
    unsigned short* dp = (unsigned short*)depth.data;
	Point3f *point = (Point3f *)pointCloud_XYZ.data;
	for(int y = 0;y < depth.rows;y++){
		for(int x = 0;x < depth.cols;x++, dp++,point++){
#if KINECT_DEPTH_WIDTH == 320
			//���s���摜�̉𑜓x��320x240�̏ꍇ
			Vector4 RealPoints = NuiTransformDepthImageToSkeleton(x,y,*dp);
#else if KINECT_DEPTH_WIDTH == 640
			//���s���摜�̉𑜓x��640x480�̏ꍇ
			Vector4 RealPoints = NuiTransformDepthImageToSkeleton(x,y,*dp, NUI_IMAGE_RESOLUTION_640x480);
#endif
			point->x = RealPoints.x;
			point->y = RealPoints.y;
			point->z = RealPoints.z;
		}
	}
}   
