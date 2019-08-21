/////////////////////////////////////////////////////////////
/* Template OpengGL sengaja dibuat untuk kuliah robotik 
*  di Departemen Teknik Elektro
*  Bagi yang ingin memodifikasi untuk keperluan yang lain,
*  dipersilahkan dengan menuliskan acknowledgement pada
*    Dr. Abdul Muis, MEng.
*    Autonomous Control Electronics (ACONICS) Research Group
*    http://www.ee.ui.ac.id/aconics
*////////////////////////////////////////////////////////////

#include <stdio.h> 
#include <stdlib.h> 
#include <GL/glut.h> // Header File For The GLUT Library2
#include <GL/gl.h> // Header File For The OpenGL32 Library
#include <GL/glu.h> // Header File For The GLu32 Library
#include <unistd.h> // Header file for sleeping.
#include <math.h> 
#include <fcntl.h>			/* File control definitions */
#include <errno.h>			/* Error number definitions */
#include <termios.h>		/* POSIX terminal control definitions */
#include <sys/time.h>
#include "planar.c"
//#include "serial.h"
#include <complex.h>
#include<String.h>

#define DTR   PI/180.0           // Conversi degree to radian
#define RTD   180.0/PI           // Conversi degree to radian
/* ascii code for the escape key */
#define ESCkey	27

/* The number/handle of our GLUT window */
int window, wcam;  
 
/* To draw a quadric model */
GLUquadricObj *obj;

// ROBOT MODEL PARAMATER
#define Xoffset	0.0	
#define Yoffset	0.0
#define Zoffset	0.05

#define Link1 L1
#define Link2 L2
#define Link3 L3
#define Link4 L4
#define Link5 L5
#define Link6 L6
#define Link0 L0

#define PERIOD 0.001

//deklarasi variabel
int gerak = 0;

float k = 0;
int N = 1000;
float r;
float x_init,y_init, z_init;
float x_final = 0, y_final = 0, z_final = 0;
float x = 16, x_cmd = 0, x_cmd_old = 0, dx_cmd = 0;
float y = 0, y_cmd = 0, y_cmd_old = 0, dy_cmd = 0;
float z = -5, z_cmd = 0, z_cmd_old = 0, dz_cmd = 0;
float x_old = 0, y_old = 0, z_old = 0,dx = 0, ddx = 0, dy = 0, ddy = 0, dz = 0, ddz = 0;
float minPWM3,maxPWM3,temp3;

float dq1 = 0, dq2 = 0, dq3 = 0, dq4 = 0, dq5 = 0, dq6 = 0;
float dq1_ref = 0, dq2_ref = 0, dq3_ref = 0, dq4_ref = 0, dq5_ref = 0, dq6_ref = 0;
float dq1_refold = 0, dq2_refold = 0, dq3_refold = 0, dq4_refold = 0, dq5_refold = 0, dq6_refold = 0;
float ddq1 = 0, ddq2 = 0, ddq3 = 0, ddq4 = 0, ddq5 = 0, ddq6 = 0;
float torque1 = 0, torque2 = 0, torque3 = 0, torque4 = 0, torque5 = 0, torque6 = 0;
float ddq1_ref = 0, ddq2_ref = 0, ddq3_ref = 0, ddq4_ref = 0, ddq5_ref = 0, ddq6_ref = 0;
float ddq1_refold = 0, ddq2_refold = 0, ddq3_refold = 0, ddq4_refold = 0, ddq5_refold = 0, ddq6_refold = 0;
float dx_ref=0,dy_ref=0,dz_ref=0;
float dx_refold=0,dy_refold=0,dz_refold=0;
float ddx_ref=0,ddy_ref=0,ddz_ref=0;

int counter = 0;
int divider = 10;
int pertama = 0;

unsigned char header = 0xF5;
unsigned char kirimsudut1_a = 0, kirimsudut2_a = 0, kirimsudut3_a = 0, kirimsudut1_b = 0, kirimsudut2_b = 0, kirimsudut3_b = 0;
unsigned char kirimsudut4_a = 0, kirimsudut5_a = 0, kirimsudut6_a = 0, kirimsudut4_b = 0, kirimsudut5_b = 0, kirimsudut6_b = 0;
float pwm1,pwm2,pwm3,pwm4,pwm5,pwm6;
float Kp = 0.000001, Kv = 0.03, Ki=0.000001;

//Perintah ke TROBOT
//char perintah[40];
char spwm1[4];
char spwm2[30];
char spwm3[30];
char spwm4[30];
char spwm5[30];
char spwm6[30];
char servo1[40]="#1P";
char servo2[4]="#2P";
char servo3[4]="#4P";
char servo4[4]="#5P";
char servo5[4]="#7P";
char servo6[4]="#8P";
char tservo[6]="T200";

//float Kp = 0.7, Kv = 0.45;
//float Kp = 1, Kv = 0.1; //time sampling 0.02
//float Kp = 0.05, Kv = 0.45, Ki=0.1;

float qa,qb;

float *tetha1=&q1;
float *tetha2=&q2;
float *tetha3=&q3;
float *theta4=&q4;
float *theta5=&q5;
float *theta6=&q6;

float *tes1=&dq1;
float *tes2=&dq2;
float *tes3=&dq3;
float *tes4=&dq4;
float *tes5=&dq5;
float *tes6=&dq6;

float a = 5;
float *count=&k;
float det;

//deklarasi variabel Jinv
float J11,J12,J13,J14,J15,J16;
float J21,J22,J23,J24,J25,J26;
float J31,J32,J33,J34,J35,J36;

float T11,T12,T13;
float T21,T22,T23;
float T31,T32,T33;

float X11,X12,X13;
float X21,X22,X23;
float X31,X32,X33;

float IJ11,IJ12,IJ13;
float IJ21,IJ22,IJ23;
float IJ31,IJ32,IJ33;
float IJ41,IJ42,IJ43;
float IJ51,IJ52,IJ53;
float IJ61,IJ62,IJ63;

void Sim_main(void); // Deklarasi lebih awal agar bisa diakses oleh fungsi sebelumnya
void display(void); // fungsi untuk menampilkan gambar robot / tampilan camera awal

/* define color */  
GLfloat green1[4]  ={0.8, 1.0, 0.8, 1.0};
GLfloat blue1[4]  ={0.1, 0.1, 1.0, 1.0};
GLfloat blue2[4]  ={0.2, 0.2, 1.0, 1.0};
GLfloat blue3[4]  ={0.3, 0.3, 1.0, 1.0};
GLfloat yellow1[4]={0.1, 0.1, 0.0, 1.0};
GLfloat yellow2[4]={0.2, 0.2, 0.0, 1.0};
GLfloat pink6[4] ={0.8, 0.55, 0.6, 1.0};
GLfloat yellow5[4]={0.8, 0.8, 0.0, 1.0};
GLfloat abu2[4]={0.5,0.5,0.5,1.0};
GLfloat gray1[4]  ={0.1, 0.1, 0.1, 1.0};
GLfloat gray2[4]  ={0.2, 0.2, 0.2, 1.0};
GLfloat gray3[4]  ={0.3, 0.3, 0.3, 1.0};
GLfloat gray4[4]  ={0.4, 0.4, 0.4, 1.0};
GLfloat gray5[4]  ={0.5, 0.5, 0.5, 1.0};
GLfloat gray6[4]  ={0.6, 0.6, 0.6, 1.0};
GLfloat gray7[4]  ={0.7, 0.7, 0.7, 1.0};
GLfloat gray8[4]  ={0.8, 0.8, 0.7, 1.0};
GLfloat gray9[4]  ={0.9, 0.9, 0.7, 1.0};


void  drawOneLine(double x1, double y1, double x2, double y2) 
   {glBegin(GL_LINES); glVertex3f((x1),(y1),0.0); glVertex3f((x2),(y2),0.0); glEnd();}
   
void  model_cylinder(GLUquadricObj * object, GLdouble lowerRadius,
  GLdouble upperRadius, GLdouble length, GLint res, GLfloat *color1, GLfloat *color2)
{
  glPushMatrix();
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glTranslatef(0,0,-length/2);
	  gluCylinder(object, lowerRadius, upperRadius, length, 20, res);
    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    gluDisk(object, 0.01, lowerRadius, 20, res); 
    glTranslatef(0, 0, length);
    gluDisk(object, 0.01, upperRadius, 20, res); 
  glPopMatrix();
}

void  model_box(GLfloat width, GLfloat depth, GLfloat height, GLfloat *color1, GLfloat *color2, GLfloat *color3, int color)
{
   width=width/2.0;depth=depth/2.0;height=height/2.0;
   glBegin(GL_QUADS);
// top
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth, height);
    glVertex3f( width,-depth, height);
    glVertex3f( width, depth, height);
    glVertex3f(-width, depth, height);
   glEnd();
   glBegin(GL_QUADS);
// bottom
    if (color==1) 
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color1);
    glVertex3f(-width,-depth,-height);
    glVertex3f( width,-depth,-height);
    glVertex3f( width, depth,-height);
    glVertex3f(-width, depth,-height);
   glEnd();
   glBegin(GL_QUAD_STRIP);
// sides
    if (color==1) 
	    glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, color2);
    glVertex3f(-width,-depth,height);
    glVertex3f(-width,-depth,-height);
    glVertex3f(width,-depth,height);
    glVertex3f(width,-depth,-height);
    glVertex3f(width,depth,height);
    glVertex3f(width,depth,-height);
    glVertex3f(-width,depth,height);
    glVertex3f(-width,depth,-height);
    glVertex3f(-width,-depth,height);
   glEnd();
}



void disp_floor(void)
{
  int i,j,flagc=1;

  glPushMatrix();
  
  GLfloat dx=4.5,dy=4.5;
  GLint amount=15;
  GLfloat x_min=-dx/2.0, x_max=dx/2.0, x_sp=(GLfloat) dx/amount, y_min=-dy/2.0, y_max=dy/2.0, y_sp=(GLfloat) dy/amount;

  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
  for(i = 0; i<=48; i++){
     drawOneLine(-2.4+0.1*i, -2.4,       -2.4+0.1*i,  2.4);
     drawOneLine(-2.4,       -2.4+0.1*i,  2.4,       -2.4+0.1*i);
  }

  glPopMatrix();
}

void  lighting(void)
{

	GLfloat light_ambient[] =  {0.2, 0.2, 0.2, 1.0};
	GLfloat light_diffuse[] =  {0.4, 0.4, 0.4, 1.0};
	GLfloat light_specular[] = {0.3, 0.3, 0.3, 1.0};
	GLfloat light_position[] = {2, 0.1, 7,1.0};
	GLfloat spot_direction[] = {0.0, -0.1, -1.0, 1.0};

	glClearColor(0.0, 0.0, 0.0, 0.0);     
  
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	glLightf(GL_LIGHT0, GL_SPOT_CUTOFF, 40.0);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, spot_direction);
	glLightf(GL_LIGHT0, GL_SPOT_EXPONENT, 4);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
}

void disp_robot(void)
{
  glPushMatrix();
    model_box(0.3, 0.5, L1, gray8, gray7, gray6,1);
    glTranslatef(Xoffset, Yoffset, Zoffset/2);
    // Draw base
    model_cylinder(obj, 0.03, 0.03, Zoffset, 2, blue1, yellow2);
    // Menuju joint-1
	glTranslatef(0, 0, Zoffset/2);
    glRotatef((*tetha1)*RTD,0,0,1);
    glPushMatrix();
      // Gambar link1-1
      glRotatef(0,0,0,1);
      glTranslatef(0,0,Link2/2); //translasi terhadap sumbu z dengan setengan panjang link
      model_cylinder(obj, 0.005, 0.005, Link2, 2, blue3, yellow2);
    glPopMatrix();
    // Menuju joint-2
    glTranslatef(0,0,Link2);
    glRotatef((*tetha2)*RTD-90,1,0,0);
    glPushMatrix();
      // Gambar link1-2
      glRotatef(0,1,0,0);
      glTranslatef(0,0,Link3/2);
      model_cylinder(obj, 0.005, 0.005, Link3, 2, pink6, yellow2);
    glPopMatrix();
    //Menuju Joint-3
    glTranslatef(0,0,Link3);
    glRotatef((*tetha3*RTD),1,0,0);
    glPushMatrix();
      // Gambar link1-3
      glRotatef(0,0,1,0);
      glTranslatef(0,0,Link4/2);
      model_cylinder(obj, 0.005, 0.005, Link4, 2, blue3, yellow2);
    glPopMatrix();
    //Menuju Joint-4
    glTranslatef(0,0,Link4);
    glRotatef(*theta4*RTD,0,1,0);
    glPushMatrix();
      // Gambar link1-4
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link5/2);
      model_cylinder(obj, 0.005, 0.005, Link5, 2, pink6, yellow2);
    glPopMatrix();
    //Menuju Joint-5
    glTranslatef(0,Link5,0);
    glRotatef(*theta5*RTD-90,0,0,1);
    glPushMatrix();
      // Gambar link1-5
      //glRotatef(-90,1,0,0);
      //glTranslatef(0,0,Link6/2);
      //model_cylinder(obj, 0.03, 0.03, Link6, 2, blue3, yellow2);
    glPopMatrix();
    //Menuju Joint-6
    glTranslatef(0,0,0);
    glRotatef(*theta6*RTD,0,1,0);
    glPushMatrix();
      // Gambar link1-6
      glRotatef(-90,1,0,0);
      glTranslatef(0,0,Link6/2);
      model_cylinder(obj, 0.005, 0.005, Link6, 2, green1, yellow2);
    glPopMatrix();
    
    glPopMatrix();
  glPopMatrix();
  glPushMatrix();
    glTranslatef(Xoffset-0.16-0.15, Yoffset+0.23-0.09, Zoffset);
    double x=0;
    double y=0;
    double radius=0.1;
    double y1=0;
		double x1=0;
		glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, green1);
		glBegin(GL_LINES);
		for(double angle=0.0f;angle<=(2.0f*3.14159);angle+=0.01f)
		{
			double x2=x-(radius*(float)sin((double)angle));
			double y2=y-(radius*(float)cos((double)angle));
			glVertex3f(x1,y1,0);
			y1=y2;
			x1=x2;
		}
    glEnd();
  glPopMatrix();


}

// Draw Object
void display(void)
{
//   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT) ; // Clear The Screen and The Depth Buffer 
   //glLoadIdentity();  // Reset View
   disp_floor();
   
   disp_robot();

   /* since window is double buffered, 
      Swap the front and back buffers (used in double buffering). */
   glutSwapBuffers() ; 
   

}
void forward_kinematics() {
	x = 2*cos(q1) + 14*cos(q1)*cos(q2) - 3*sin(q6)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - 3*cos(q6)*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + 3*cos(q1)*cos(q2)*cos(q3) + 15*cos(q1)*cos(q2)*sin(q3) + 15*cos(q1)*cos(q3)*sin(q2) - 3*cos(q1)*sin(q2)*sin(q3);
	y = 2*sin(q1) + 14*cos(q2)*sin(q1) + 3*sin(q6)*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + 3*cos(q6)*(cos(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - 3*sin(q1)*sin(q2)*sin(q3) + 3*cos(q2)*cos(q3)*sin(q1) + 15*cos(q2)*sin(q1)*sin(q3) + 15*cos(q3)*sin(q1)*sin(q2);
	z = 14*sin(q2) - 15*cos(q2)*cos(q3) + 3*cos(q2)*sin(q3) + 3*cos(q3)*sin(q2) + 15*sin(q2)*sin(q3) - 3*cos(q6)*(sin(q5)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)) + cos(q4)*cos(q5)*(cos(q2)*sin(q3) + cos(q3)*sin(q2))) + 3*sin(q4)*sin(q6)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + 10;
  //printf("x = %.2f y = %.2f z = %.2f \n",x,y,z);
}

void inverse_jacobian() {
//  dq4_refold = dq4_ref;
//  dq5_refold = dq5_ref;
//  dq6_refold = dq6_ref;
  /**
  dq1_refold = dq1_ref;
  dq2_refold = dq2_ref;
  dq3_refold = dq3_ref;
  dq4_refold = dq4_ref;
  dq5_refold = dq5_ref;
  dq6_refold = dq6_ref;
  **/
  
//Inverse Jacob	
  J11 =3*sin(q1)*sin(q2)*sin(q3) - 14*cos(q2)*sin(q1) - 3*sin(q6)*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - 3*cos(q6)*(cos(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - 2*sin(q1) - 3*cos(q2)*cos(q3)*sin(q1) - 15*cos(q2)*sin(q1)*sin(q3) - 15*cos(q3)*sin(q1)*sin(q2);
  J12 =3*cos(q6)*(sin(q5)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) + cos(q4)*cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - 14*cos(q1)*sin(q2) - 3*sin(q4)*sin(q6)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + 15*cos(q1)*cos(q2)*cos(q3) - 3*cos(q1)*cos(q2)*sin(q3) - 3*cos(q1)*cos(q3)*sin(q2) - 15*cos(q1)*sin(q2)*sin(q3);
  J13 =-3*cos(q1)*(cos(q2)*sin(q3) - 5*cos(q2)*cos(q3) + cos(q3)*sin(q2) + 5*sin(q2)*sin(q3) - cos(q2 + q3)*cos(q6)*sin(q5) + sin(q2 + q3)*sin(q4)*sin(q6) - sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6));
  J14 =3*sin(q6)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - 3*cos(q5)*cos(q6)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)));
  J15 =3*cos(q6)*(sin(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) + cos(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
  J16 =3*sin(q6)*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) - 3*cos(q6)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)));
  J21 =2*cos(q1) + 14*cos(q1)*cos(q2) - 3*sin(q6)*(cos(q4)*sin(q1) - sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - 3*cos(q6)*(cos(q5)*(sin(q1)*sin(q4) + cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3))) - sin(q5)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2))) + 3*cos(q1)*cos(q2)*cos(q3) + 15*cos(q1)*cos(q2)*sin(q3) + 15*cos(q1)*cos(q3)*sin(q2) - 3*cos(q1)*sin(q2)*sin(q3);
  J22 =15*cos(q2)*cos(q3)*sin(q1) - 3*cos(q6)*(sin(q5)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) - cos(q4)*cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - 15*sin(q1)*sin(q2)*sin(q3) - 3*sin(q4)*sin(q6)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - 14*sin(q1)*sin(q2) - 3*cos(q2)*sin(q1)*sin(q3) - 3*cos(q3)*sin(q1)*sin(q2);
  J23 =-3*sin(q1)*(cos(q2)*sin(q3) - 5*cos(q2)*cos(q3) + cos(q3)*sin(q2) + 5*sin(q2)*sin(q3) - cos(q2 + q3)*cos(q6)*sin(q5) + sin(q2 + q3)*sin(q4)*sin(q6) - sin(q2 + q3)*cos(q4)*cos(q5)*cos(q6));
  J24 =3*cos(q5)*cos(q6)*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - 3*sin(q6)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)));
  J25 =-3*cos(q6)*(sin(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - cos(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)));
  J26 =3*cos(q6)*(cos(q1)*cos(q4) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) - 3*sin(q6)*(cos(q5)*(cos(q1)*sin(q4) + cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1))) + sin(q5)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)));
  J31 =0;
  J32 =3*cos(q2 + q3) + 15*sin(q2 + q3) + 14*cos(q2) + (3*cos(q4 - q6)*cos(q2 + q3))/2 - (3*cos(q2 + q3)*cos(q4 + q6))/2 - (3*cos(q2 + q3)*cos(q4 + q5)*cos(q6))/2 + 3*sin(q2 + q3)*cos(q6)*sin(q5) - (3*cos(q4 - q5)*cos(q2 + q3)*cos(q6))/2;
  J33 =3*cos(q2 + q3) + 15*sin(q2 + q3) + 3*cos(q2 + q3)*sin(q4)*sin(q6) + 3*sin(q2 + q3)*cos(q6)*sin(q5) - 3*cos(q2 + q3)*cos(q4)*cos(q5)*cos(q6);
  J34 =3*sin(q2 + q3)*(cos(q4)*sin(q6) + cos(q5)*cos(q6)*sin(q4));
  J35 =-3*cos(q6)*(cos(q2 + q3)*cos(q5) - sin(q2 + q3)*cos(q4)*sin(q5));
  J36 =3*sin(q6)*(cos(q2 + q3)*sin(q5) + sin(q2 + q3)*cos(q4)*cos(q5)) + 3*sin(q2 + q3)*cos(q6)*sin(q4);
  T11 = J11*J11 + J12*J12 + J13*J13 + J14*J14 + J15*J15 + J16*J16;
  T12 = J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26;
  T13 = J11*J31 + J12*J32 + J13*J33 + J14*J34 + J15*J35 + J16*J36;
  T21 = J11*J21 + J12*J22 + J13*J23 + J14*J24 + J15*J25 + J16*J26;
  T22 = J21*J21 + J22*J22 + J23*J23 + J24*J24 + J25*J25 + J26*J26;
  T23 = J21*J31 + J22*J32 + J23*J33 + J24*J34 + J25*J35 + J26*J36;
  T31 = J11*J31 + J12*J32 + J13*J33 + J14*J34 + J15*J35 + J16*J36;
  T32 = J21*J31 + J22*J32 + J23*J33 + J24*J34 + J25*J35 + J26*J36;
  T33 = J31*J31 + J32*J32 + J33*J33 + J34*J34 + J35*J35 + J36*J36;
  det = T11*T22*T33-T11*T23*T32-T12*T21*T33+T12*T23*T31+T13*T21*T32-T13*T22*T31;
 
  X11 = (T22*T33 - T32*T23)/det;
  X12 = (T13*T32 - T12*T33)/det;
  X13 = (T12*T23 - T13*T22)/det;
  X21 = (T23*T31 - T21*T33)/det;
  X22 = (T11*T33 - T13*T31)/det;
  X23 = (T13*T21 - T11*T23)/det;
  X31 = (T21*T32 - T22*T31)/det;
  X32 = (T12*T31 - T11*T32)/det;
  X33 = (T11*T22 - T12*T21)/det;
  
  IJ11 =J11*X11 + J21*X21 + J31*X31;
  IJ12 =J11*X12 + J21*X22 + J31*X32;
  IJ13 =J11*X13 + J21*X23 + J31*X33;
  IJ21 =J12*X11 + J22*X21 + J32*X31;
  IJ22 =J12*X12 + J22*X22 + J32*X32;
  IJ23 =J12*X13 + J22*X23 + J32*X33;
  IJ31 =J13*X11 + J23*X21 + J33*X31;
  IJ32 =J13*X12 + J23*X22 + J33*X32;
  IJ33 =J13*X13 + J23*X23 + J33*X33;
  IJ41 =J14*X11 + J24*X21 + J34*X31;
  IJ42 =J14*X12 + J24*X22 + J34*X32;
  IJ43 =J14*X13 + J24*X23 + J34*X33;
  IJ51 =J15*X11 + J25*X21 + J35*X31;
  IJ52 =J15*X12 + J25*X22 + J35*X32;
  IJ53 =J15*X13 + J25*X23 + J35*X33;
  IJ61 =J16*X11 + J26*X21 + J36*X31;
  IJ62 =J16*X12 + J26*X22 + J36*X32;
  IJ63 =J16*X13 + J26*X23 + J36*X33;
  
  
  ddq1 = IJ11*ddx+IJ12*ddy+IJ13*ddz;
  ddq2 = IJ21*ddx+IJ22*ddy+IJ23*ddz;
  ddq3 = IJ31*ddx+IJ32*ddy+IJ33*ddz;
  ddq4 = IJ41*ddx+IJ42*ddy+IJ43*ddz;
  ddq5 = IJ51*ddx+IJ52*ddy+IJ53*ddz;
  ddq6 = IJ61*ddx+IJ62*ddy+IJ63*ddz;
	
  
//  dq1_ref = IJ11*dx+IJ12*dy+IJ13*dz;
//  dq2_ref = IJ21*dx+IJ22*dy+IJ23*dz;
//  dq3_ref = IJ31*dx+IJ32*dy+IJ33*dz;
//  dq4_ref = IJ41*dx+IJ42*dy+IJ43*dz;
//  dq5_ref = IJ51*dx+IJ52*dy+IJ53*dz;
//  dq6_ref = IJ61*dx+IJ62*dy+IJ63*dz;

//pake trigono
//	r = sqrt(dx*dx+dz*dz);
//	dq1 = acos(dy/dx);
//	dq2 = atan(dz/dx)+acos((L0*L0-L3*L3-r*r)/2*L3*r);
//	dq3 = acos((L3*L3-L0*L0-r*r)/2*L3*L0);
}

void controller() {
	
  ddx_ref=(dx-dx_refold)/PERIOD;
  ddy_ref=(dy-dy_refold)/PERIOD;
  ddz_ref=(dz-dz_refold)/PERIOD;
  
//  ddq1_ref = (dq1_ref - dq1_refold)/PERIOD;
//  ddq2_ref = (dq2_ref - dq2_refold)/PERIOD;
//  ddq3_ref = (dq3_ref - dq3_refold)/PERIOD;
//  ddq4_ref = (dq4_ref - dq4_refold)/PERIOD;
//  ddq5_ref = (dq5_ref - dq5_refold)/PERIOD;
//  ddq6_ref = (dq6_ref - dq6_refold)/PERIOD;
  
  ddx = Kp*dx + Kv*ddx_ref+Ki*dx_ref*PERIOD;
  ddy = Kp*dy + Kv*ddy_ref+Ki*dy_ref*PERIOD;
  ddz = Kp*dz + Kv*ddz_ref+Ki*dz_ref*PERIOD;
  
//  torque1 = Kp*dq1_ref + Kv*ddq1_ref+Ki*dq1_ref*PERIOD;
//  torque2 = Kp*dq2_ref + Kv*ddq2_ref+Ki*dq2_ref*PERIOD;
//  torque3 = Kp*dq3_ref + Kv*ddq3_ref+Ki*dq3_ref*PERIOD;
//  torque4 = Kp*dq4_ref + Kv*ddq4_ref+Ki*dq4_ref*PERIOD;
//  torque5 = Kp*dq5_ref + Kv*ddq5_ref+Ki*dq5_ref*PERIOD;
//  torque6 = Kp*dq6_ref + Kv*ddq6_ref+Ki*dq6_ref*PERIOD;
}

void double_integrator() {
//    ddq1 = torque1;
//    ddq2 = torque2;
//    ddq3 = torque3;
//    ddq4 = torque4;
//    ddq5 = torque5;
//    ddq6 = torque6;
    
  dq1 = dq1 + ddq1*PERIOD;
  dq2 = dq2 + ddq2*PERIOD;
  dq3 = dq3 + ddq3*PERIOD;
  dq4 = dq4 + ddq4*PERIOD;
  dq5 = dq5 + ddq5*PERIOD;
  dq6 = dq6 + ddq6*PERIOD;
//  dq4 = dq4 + ddq4*PERIOD;
//  dq5 = dq5 + ddq5*PERIOD;
//  dq6 = dq6 + ddq6*PERIOD;

  q1 = q1 + dq1*PERIOD;
  q2 = q2 + dq2*PERIOD;
  q3 = q3 + dq3*PERIOD;
  q4 = q4 + dq4*PERIOD;
  q5 = q5 + dq5*PERIOD;
  q6 = q6 + dq6*PERIOD;
  //q3_servo = q3-(q2-90*DTR);
//  q4 = q4 + dq4*PERIOD;
//  q5 = q5 + dq5*PERIOD;
//  q6 = q6 + dq6*PERIOD;	
//	printf("q1 = %.2f q2 = %.2f q3 = %.2f \n",q1*RTD,q2*RTD,q3*RTD);
}

void trajectory_init() {
  k = 0;
  counter = 0;
  forward_kinematics();
  x_init = x;
  y_init = y;
  z_init = z;
  ddq1 = 0;
  ddq2 = 0;
  ddq3 = 0;
  ddq4 = 0;
  ddq5 = 0;
  ddq6 = 0;
}

void trajectory_planning() {
  x_cmd = x_init + (x_final - x_init)*k/(N*PERIOD);
  y_cmd = y_init + (y_final - y_init)*k/(N*PERIOD);
  z_cmd = z_init + (z_final - z_init)*k/(N*PERIOD);
  //x_cmd = x_init + a*sin(2*PI*k/(N*PERIOD));
  //y_cmd = y_init + a*cos(2*PI*k/(N*PERIOD));
  //z_cmd = z_init;
  //printf("x_cmd = %.2f y_cmd = %.2f z_cmd = %.2f \n",x_cmd,y_cmd,z_cmd);
}

void hitung_dydz() {
  dx_refold=dx;
  dy_refold=dy;
  dz_refold=dz;
  dx = (x_cmd - x)/PERIOD;
  dy = (y_cmd - y)/PERIOD;
  dz = (z_cmd - z)/PERIOD;
  
}
void Sim_main(void)
{
  if (pertama==0){
  	q1=0;
  	q2=0;
  	q3=0;
  	q4=0;
  	q5=0;
  	q6=0;
	 forward_kinematics();
	 printf("x = %.2f y = %.2f z = %.2f\n",x,y,z);
	 printf("Program Mulai");
	pertama=1;
  }
  glutSetWindow(window);
  display();
  if (gerak == 1) {
    trajectory_init();
    //printf("init y=%.2f z=%.2f\n", y_init, z_init);
    while (k <= N*PERIOD) {
      k += PERIOD;
      counter++;
      trajectory_planning();
      hitung_dydz();
      controller();
      inverse_jacobian();
//      if(q1 + dq1*PERIOD < -90*DTR){
//        	q1 = -90*DTR;
//		}
//		if(q1 + dq1*PERIOD > 90*DTR){
//        	q1 = 90*DTR;
//		}
//		if(q2 + dq2*PERIOD < 0){
//        	q2 = 0;
//		}
//		if(q2 + dq2*PERIOD > 140*DTR){
//        	q2 = 140*DTR;
//		}
//		if(q3 + dq3*PERIOD < 60*DTR){
//        	q3 = 60*DTR;
//		}
//		if(q3 + dq3*PERIOD > 130*DTR){
//        	q3 = 130*DTR;
//		}
//		if(q4 + dq4*PERIOD < 0){
//        	q4 = 0;
//		}
//		if(q4 + dq4*PERIOD > 180*DTR){
//        	q4 = 180*DTR;
//		}
//		if(q5 + dq5*PERIOD < 0){
//        	q5 = 0;
//		}
//		if(q5 + dq5*PERIOD > 160*DTR){
//        	q5 = 160*DTR;
//		}
      double_integrator();
      forward_kinematics();
      if (counter%(N/divider) == 0) {	    		
		float sudut1 = fmod ((q1*RTD), 360);
        //printf("sudut1 : %d \n", sudut1);
        float sudut2 = fmod ((q2*RTD), 360);
        float sudut3 = fmod ((q3*RTD), 360);
        float sudut4 = fmod ((q4*RTD), 360);
        float sudut5 = fmod ((q5*RTD), 360);
        float sudut6 = fmod ((q6*RTD), 360);
		printf("\nq1 = %.2f q2 = %.2f q3 = %.2f q4 = %.2f q5 = %.2f q6 = %.2f\n",q1*RTD,q2*RTD,q3*RTD,q4*RTD,q5*RTD,q6*RTD);
        printf("x = %.2f y = %.2f z = %.2f\n",x,y,z);
        printf("x_final = %.2f y_final = %.2f z_final = %.2f\n",x_final,y_final,z_final);
        //printf("sudut1 after mod : %d\n", sudut1);
        //hitung PWM
        sudut3 = sudut3 + sudut2;
        printf("Sebelum Batas sudut1 = %.2f sudut2 = %.2f sudut3 = %.2f sudut4 = %.2f sudut5 = %.2f sudut6 = %.2f\n",sudut1,sudut2,sudut3,sudut4,sudut5,sudut6);
        if(sudut1 < -90){
    		sudut1 = -90;
		}
		if(sudut1 > 90){
	    	sudut1 = 90;
		}
		if(sudut2 < 0){
	    	sudut2 = 0;
		}
		if(sudut2 > 180){
	    	sudut2 = 180;
		}
		if(sudut3 < -40){
	    	sudut3 = -40;
		}
		if(sudut3 > 90){
	    	sudut3 = 90;
		}
		if(sudut4 < 0){
	    	sudut4 = 0;
		}
		if(sudut4 > 180){
	    	sudut4 = 180;
		}
		if(sudut5 < 0){
	    	sudut5 = 0;
		}
		if(sudut5 > 160){
	    	sudut5 = 160;
		}
		printf("sudut1 = %.2f sudut2 = %.2f sudut3 = %.2f sudut4 = %.2f sudut5 = %.2f sudut6 = %.2f\n",sudut1,sudut2,sudut3,sudut4,sudut5,sudut6);
        pwm1=(sudut1*9.944) + 1630;
  		pwm2=(sudut2*10.4) + 536.7;
  		temp3 = ((sudut3)*2000/180) + 1700;
  		minPWM3 = 600 + (1522-600)*sudut2/180;
  		maxPWM3 = 1856 + (2500-1856)*sudut2/180;
  		if(temp3 < minPWM3){
  			pwm3 = minPWM3;	
		}
		else if(temp3 >maxPWM3){
			pwm3 = maxPWM3;
		}
		else{
			pwm3 = temp3;
		}
  		//pwm3=((sudut3-sudut2)*RTD*2000/180) + 1700;
  		pwm4=(sudut4*2000/180)+1700;
  		pwm5=(sudut5*1500/160)+1660;
  		pwm6=(sudut6*2000/180)+500;
  		printf("pwm1 = %.2f pwm2 = %.2f pwm3 = %.2f pwm4 = %.2f pwm5 = %.2f pwm6 = %.2f\n",pwm1,pwm2,pwm3,pwm4,pwm5,pwm6);
  		//printf("%f %f \n", pwm1, pwm2);
  		
  		
  		int pwm1int = pwm1;
  		int pwm2int = pwm2;
  		int pwm3int = pwm3;
  		int pwm4int = pwm4;
  		int pwm5int = pwm5;
  		int pwm6int = pwm6;
  		
		sprintf(spwm1, "%d", pwm1int);  //convert int ke string
        sprintf(spwm2, "%d", pwm2int);
        sprintf(spwm3, "%d", pwm3int);
        sprintf(spwm4, "%d", pwm4int);
        sprintf(spwm5, "%d", pwm5int);
        sprintf(spwm6, "%d", pwm6int);
        
        char perintah[40] = "#1P";
         	
//		strcat(perintah, servo1);
		strcat(perintah, spwm1);
		strcat(perintah, servo2);
		strcat(perintah, spwm2);
		strcat(perintah, servo3);
		strcat(perintah, spwm3);
		strcat(perintah, servo4);
		strcat(perintah, spwm4);
		strcat(perintah, servo5);
		strcat(perintah, spwm5);
		strcat(perintah, servo6);
		strcat(perintah, spwm6);
		strcat(perintah, tservo);
//		strcat(perintah, "\r\n");
		        
        char test[2]={0x0D,0x0A};			// simbol untuk /r /n

		 write(fd,&perintah,sizeof(perintah));//header // 
         write(fd,&test,sizeof(test));
        
        printf("%s \n",perintah);
		
      }
      display();
    }
    gerak = 0;
  }
}
//void Sim_main(void)
//{
//  glutSetWindow(window);
//  display();
//  if (gerak == 1) {
//    trajectory_init();
//    printf("init y=%.2f z=%.2f\n", y_init, z_init);
//    while (k <= N*PERIOD) {
//      k += PERIOD;
//      counter++;
//      trajectory_planning();
//      hitung_dydz();
//      inverse_jacobian();
//      controller();
//      double_integrator();
//      forward_kinematics();
//      if (counter%(N/divider) == 0) {
//        printf("%d %.2f %.2f %.4f %.4f\n", counter, y_cmd, z_cmd, q1*RTD, q2*RTD);
//        int sudut1 = fmod ((q1*RTD), 360);
//        int sudut2 = fmod ((q2*RTD), 360);
//        if (sudut1 < 0) sudut1 += 360;
//        if (sudut2 < 0) sudut2 += 360;
//        if (sudut1 > 255) {
//          kirimsudut1_a = 255;
//          kirimsudut1_b = sudut1 - 255;
//        } else if (sudut1 <= 255) {
//          kirimsudut1_a = sudut1;
//          kirimsudut1_b = 0;
//        }
//        if (sudut2 > 255) {
//          kirimsudut2_a = 255;
//          kirimsudut2_b = sudut2 - 255;
//        } else if (sudut2 <= 255) {
//          kirimsudut2_a = sudut2;
//          kirimsudut2_b = 0;
//        }
//        write(fd,&header,sizeof(header));//header
//        write(fd,&kirimsudut1_a,sizeof(kirimsudut1_a));//data sudut 1
//        write(fd,&kirimsudut1_b,sizeof(kirimsudut1_b));//data sudut 1
//        write(fd,&kirimsudut2_a,sizeof(kirimsudut2_a));//data sudut 2
//        write(fd,&kirimsudut2_b,sizeof(kirimsudut2_b));//data sudut 2
//      }
//      display();
//    }
//    gerak = 0;
//  }
//}

void keyboard(unsigned char key, int i, int j)
{
	 switch(key){
	 /* Joint Control */
	 case ESCkey: exit(1);
	 case '0' : x_final = 5;y_final = 5;z_final = 5;gerak = 1;break;
     case '1' : x_final = x + 2; gerak = 1; break;
     case '2' : x_final = x - 2; gerak = 1; break;
     case '3' : y_final = y + 2; gerak = 1; break;
     case '4' : y_final = y - 2; gerak = 1; break;
     case '5' : z_final = z + 2; gerak = 1; break;
     case '6' : z_final = z - 2; gerak = 1; break;
	 //case '6' : x_final = 0; y_final = 0; z_final = 0.2; gerak = 1; break;
	 case 'q' : q1 += 5*DTR;trajectory_init();break;
	 case 'w' : q2 += 5*DTR;trajectory_init();break;
	 case 'e' : q3 += 5*DTR;trajectory_init();break;
	 case 'r' : q4 += 5*DTR;trajectory_init();break;
	 case 't' : q5 += 5*DTR;trajectory_init();break;
	 case 'y' : q6 += 5*DTR;trajectory_init();break;
	//arah gerak
	 //case '0' : break;
	 case '9' : break;
	 case '8' : break;
	 case '7' : break;
	 //case '6' : break;
	//Kamera	
	 	case 'a' : glRotatef(10,0,0,0);break;
     	case 'f' : glRotatef(1,0,0,-1);break;
     	case 's' : glRotatef(1,0,0,1);break;
     	case 'i' : glRotatef(1,0,1,0);break;
     	case 'k' : glRotatef(1,0,-1,0);break;
     	case 'j' : glRotatef(1,-1,0,0);break;
     	case 'l' : glRotatef(1,1,0,0);break;
      
   }
}



void init(void) 
{ 
   obj = gluNewQuadric(); 
   /* Clear background to (Red, Green, Blue, Alpha) */
   glClearColor(0.0f, 0.0f, 0.0f, 0.0f) ;
   glEnable(GL_DEPTH_TEST); // Enables Depth Testing
   glMatrixMode(GL_PROJECTION);
   glLoadIdentity();
   gluPerspective(30.0, 1, 0.2, 8);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0.3, 0.0, 1.5,  -0.1, 0.0, 0.4,  0.0, 0.0, 1.0); 
	 lighting();
	 
   /* When the shading model is GL_FLAT only one colour per polygon is used, 
      whereas when the shading model is set to GL_SMOOTH the colour of 
      a polygon is interpolated among the colours of its vertices.  */
   glShadeModel(GL_SMOOTH) ; 

   glutDisplayFunc (&display) ;
   glutKeyboardFunc(&keyboard);
   glRotatef(-50,0,1,0);

}

// Main Program
int main(int argc, char** argv)
{
 // Initialize GLUT
   /* Initialize GLUT state - glut will take any command line arguments 
      see summary on OpenGL Summary */  
   glutInit (&argc, argv);
   
   // Berikut jika ingin menggunakan serial port
   fd = open_port();
   init_port(fd);

   /* Select type of Display mode:   
      Double buffer 
      RGBA color
      Alpha components supported 
      Depth buffer */  
   //glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
   glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB );
   /* set a 400 (width) x 400 (height) window and its position */
   glutInitWindowSize(400,400);	
   glutInitWindowPosition (40, 100);

   /* Open a window */  
   window = glutCreateWindow ("Simple Window");

   /* Initialize our window. */
   init() ;
   init_robot();
   trajectory_init();
   /* Register the function to do all our OpenGL drawing. */
   glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

   /* Start Event Processing Engine */ 
   glutMainLoop () ;
   return 0 ;
}           
