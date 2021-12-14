/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with 
  or without modification, are permitted provided that the 
  following conditions are met:

* Redistributions of source code must retain the above 
  copyright notice, this list of conditions and the 
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the 
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/


#ifndef GLWIDGET_H
#define GLWIDGET_H

#define _USE_MATH_DEFINES

#include <cmath>

#include <QtOpenGL/QGLWidget>
#include <QtCore/QTimer>

namespace vlr {

#define      DEFAULT_ZOOM_SENSITIVITY             0.2
#define      DEFAULT_ROTATE_SENSITIVITY           0.50
#define      DEFAULT_MOVE_SENSITIVITY             0.001
#define      DEFAULT_MIN_ZOOM_RANGE               0.5
#define      DEFAULT_CAMERA_FOV                   30.0
#define      DEFAULT_MIN_CLIP_RANGE               1.0
#define      DEFAULT_MAX_CLIP_RANGE               400.0
#define		 DEFAULT_FPS						  30.0

#define		 DISPLAY_REFRESH_DELAY_MS			  66.6666666667 // ~(15 Hz)

#define KEY_ROTATE_AMOUNT 5.0
#define KEY_MOVE_AMOUNT   10.0
#define KEY_ZOOM_AMOUNT   5.0

#ifndef CAMERA_STATE_DEFINED
  #define CAMERA_STATE_DEFINED
  typedef enum {IDLE, ROTATING, MOVING, ZOOMING} CameraState;
#else
  typedef camera_state_t CameraState;
#endif

typedef void (*DisplayFunction)(void);
typedef void (*KeyboardFunction)(unsigned char, int, int);
typedef void (*MouseFunction)(int, int, int, int);
typedef void (*MotionFunction)(int, int);

template<class T> inline T rad(T x) {return T(x*M_PI/180.0);}

typedef struct
	{
	CameraState state;
	double pan, tilt, distance;
	double xOffset, yOffset, zOffset;
	double zoom, warp_x, warp_y;
	} cameraPose_t, *cameraPose_p;

class GLWidget : public QGLWidget
 {
     Q_OBJECT

protected:
	QTimer timer;
	int windowID;
	int windowWidth, windowHeight;
	double fps;
	cameraPose_t cameraPose;
	int lastMouseX, lastMouseY;

	DisplayFunction userDisplayFunction;
	KeyboardFunction userKeyboardFunction;
	MouseFunction userMouseFunction;
	MotionFunction userMotionFunction;

	double zoomSensitivity;
	double rotateSensitivity;
	double moveSensitivity;
	double minZoomRange;
	double cameraFov;
	double minClipRange;
	double maxClipRange;

public slots:
	void redraw();

public:
     GLWidget(QGLFormat glFormat=QGLFormat(QGL::DoubleBuffer), QWidget *parent = 0);
     ~GLWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

     void setInitialCameraPos(float pan, float tilt, float range, float xOffset, float yOffset, float zOffset);

     void setCameraParams(double zoomSensitivity_, double rotateSensitivity_, double moveSensitivity_, double minZoomRange_,
     		double cameraFov_, double minClipRange_, double maxClipRange_);

     void setDisplayFunction(DisplayFunction func);
     void setMouseFunction(MouseFunction func);
     void setMotionFunction(MotionFunction func);

     void requestRedraw(void);

     void recenter(void);
     void pickPoint(int mouse_x, int mouse_y, double *scene_x, double *scene_y);

     double getCamDistance() {return cameraPose.distance;}
     double getCamPan() {return cameraPose.tilt;}
     double getCamTilt() {return cameraPose.tilt;}

     virtual void mousePressEvent(QMouseEvent *event);
     virtual void mouseReleaseEvent(QMouseEvent *event);
     virtual void mouseMoveEvent(QMouseEvent *event);

protected:
	 void rotateCamera(double dx, double dy);
	 void zoomCamera(double dy);
	 void moveCamera(double dx, double dy);
	 void init3DMode(int w, int h, double& fovY, double& zNear, double& zFar);

 protected:
   virtual void initializeGL();
   virtual void paintGL();
	 virtual void resizeGL(int width, int height);

	 void activate3DMode();
	 void activate2DMode();

 protected:
	bool refresh_required;
	float ambientLight[4];
	float diffuseLight[4];
	float specularLight[4];
	float lightPosition[4];
  bool gl_initialized_;    // this flag is set after initializeGL was called..otherwise an early resizeGL might crash the app

private:
	static const float ambientLightDefault[4];
	static const float diffuseLightDefault[4];
	static const float specularLightDefault[4];
	static const float lightPositionDefault[4];
};

}	// namespace vlr

#endif /*GLWIDGET_H*/


