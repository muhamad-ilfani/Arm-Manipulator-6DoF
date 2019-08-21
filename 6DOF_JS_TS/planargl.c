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
int i = 0;


float k = 0, T = 0.02;
int N = 1000;
float r;
float x_init,y_init, z_init,x_final, y_final, z_final;
float x = 0, x_cmd = 0, x_cmd_old = 0, dx_cmd = 0;
float y = 0, y_cmd = 0, y_cmd_old = 0, dy_cmd = 0;
float z = 0, z_cmd = 0, z_cmd_old = 0, dz_cmd = 0;
float x_old = 0, y_old = 0, z_old = 0,dx = 0, ddx = 0, dy = 0, ddy = 0, dz = 0, ddz = 0;

float dq1 = 0, dq2 = 0, dq3 = 0, dq4 = 0, dq5 = 0, dq6 = 0;
float dq1_ref = 0, dq2_ref = 0, dq3_ref = 0, dq4_ref = 0, dq5_ref = 0, dq6_ref = 0;
float q1_awal = 0, q2_awal = 0, q3_awal = 0, q4_awal = 0, q5_awal = 0, q6_awal = 0;
float q1_ref = 0, q2_ref = 0, q3_ref = 0, q4_ref = 0, q5_ref = 0, q6_ref = 0;
float err_q1 = 0, err_q2 = 0, err_q3 = 0, err_q4 = 0, err_q5 = 0, err_q6 = 0;
float dq1_refold = 0, dq2_refold = 0, dq3_refold = 0, dq4_refold = 0, dq5_refold = 0, dq6_refold = 0;
float ddq1 = 0, ddq2 = 0, ddq3 = 0, ddq4 = 0, ddq5 = 0, ddq6 = 0;
float torque1 = 0, torque2 = 0, torque3 = 0, torque4 = 0, torque5 = 0, torque6 = 0;
float ddq1_ref = 0, ddq2_ref = 0, ddq3_ref = 0, ddq4_ref = 0, ddq5_ref = 0, ddq6_ref = 0;
float ddq1_refold = 0, ddq2_refold = 0, ddq3_refold = 0, ddq4_refold = 0, ddq5_refold = 0, ddq6_refold = 0;
float dx_ref=0,dy_ref=0,dz_ref=0;
float dx_refold=0,dy_refold=0,dz_refold=0;
float ddx_ref=0,ddy_ref=0,ddz_ref=0; 
float q1_cmd = 0, q2_cmd = 0, q3_cmd = 0, q4_cmd = 0, q5_cmd = 0, q6_cmd = 0;
float q3_servo = 0;

int counter = 0;
int divider = 1;
float samplingtime = 0.001;

unsigned char header = 0xF5;
unsigned char kirimsudut1_a = 0, kirimsudut2_a = 0, kirimsudut3_a = 0, kirimsudut1_b = 0, kirimsudut2_b = 0, kirimsudut3_b = 0;
unsigned char kirimsudut4_a = 0, kirimsudut5_a = 0, kirimsudut6_a = 0, kirimsudut4_b = 0, kirimsudut5_b = 0, kirimsudut6_b = 0;
float pwm1,pwm2,pwm3,pwm4,pwm5,pwm6;
float Kp = 0.009, Kv = 0.6, Ki=0.0001;

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
char tservo[6]="T1000";

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
    model_cylinder(obj, 0.005, 0.005, Zoffset, 2, blue1, yellow2);
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

void TSJSUpdate()
{
    q1_awal=q1; q2_awal=q2;
    q3_awal=q3; q4_awal=q4;
    q5_awal=q5; q6_awal=q6;
    dq1 = 0; dq2 = 0; dq3 = 0; 
    dq4 = 0; dq5 = 0; dq6 = 0;
    q1_ref = q1*RTD; q2_ref = q2*RTD; 
    q3_ref = q3*RTD; q4_ref = q4*RTD; 
    q5_ref = q5*RTD; q6_ref = q6*RTD; 
    gerak = 0;
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

void controlrobot(){
  
  q1_cmd =q1_awal + dq1 * k/(N*PERIOD);  //k itu counter, nanti nambah tiap
  err_q1 = q1_cmd - q1;
  dq1_ref = 2010*(err_q1);
  q1_ref = q1_ref + dq1_ref * samplingtime;
  q1 = q1_ref * DTR;
  // printf("q1 = %f   q1_awal = %f   kq1_ref= %f\n",q1*RTD,q1_awal*RTD,q1_ref*RTD);
 
  q2_cmd =q2_awal + dq2 * k/(N*PERIOD);
  err_q2 = q2_cmd - q2;
  dq2_ref = 2010*(err_q2);
  q2_ref = q2_ref + dq2_ref * samplingtime;
  q2 = q2_ref * DTR;
  // printf("q2 = %f   q2_awal = %f   q2_ref = %f\n",q2*RTD,q2_awal*RTD,q2_ref*RTD);
  
  q3_cmd =q3_awal + dq3 * k/(N*PERIOD);
  err_q3 = q3_cmd - q3;
  dq3_ref = 2010*(err_q3);
  q3_ref = q3_ref + dq3_ref * samplingtime;
  q3 = q3_ref * DTR;
  // printf("q3 = %f   q3_awal = %f   q1_ref = %f\n",q3*RTD,q3_awal*RTD,q3_ref*RTD);
  
  q4_cmd =q4_awal + dq4 * k/(N*PERIOD);
  err_q4 = q4_cmd - q4;
  dq4_ref = 2010*(err_q4);
  q4_ref = q4_ref + dq4_ref * samplingtime;
  q4 = q4_ref * DTR;
  // printf("q4 = %f   q4_awal = %f   k = %f\n",q4*RTD,q4_awal*RTD,k);
  
  q5_cmd =q5_awal + dq5 * k/(N*PERIOD);
  err_q5 = q5_cmd - q5;
  dq5_ref = 2010*(err_q5);
  q5_ref = q5_ref + dq5_ref * samplingtime;
  q5 = q5_ref * DTR;
  // printf("q5 = %f   q5_awal = %f   k = %f\n",q5*RTD,q5_awal*RTD,k);
  
  q6_cmd =q6_awal + dq6 * k/(N*PERIOD);
  err_q6 = q6_cmd - q6;
  dq6_ref = 2010*(err_q6);
  q6_ref = q6_ref + dq6_ref * samplingtime;
  q6 = q6_ref * DTR;
  // printf("q6 = %f   q6_awal = %f   k = %f\n",q6*RTD,q6_awal*RTD,k);
}

void forward_kinematics() 
{
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
  

  
  ddx = Kp*dx + Kv*ddx_ref+Ki*dx_ref*PERIOD;
  ddy = Kp*dy + Kv*ddy_ref+Ki*dy_ref*PERIOD;
  ddz = Kp*dz + Kv*ddz_ref+Ki*dz_ref*PERIOD;
  

}

void converts()
{
  q3_servo = q3+q2;
  int sudut1 = fmod ((q1*RTD), 360);
  int sudut2 = fmod ((q2*RTD), 360);
  int sudut3 = fmod ((q3_servo*RTD), 360);
  int sudut4 = fmod ((q4*RTD), 360);
  int sudut5 = fmod ((q5*RTD), 360);
  int sudut6 = fmod ((q6*RTD), 360);

  // if (sudut1 < 0) sudut1 += 360;
  // if (sudut2 < 0) sudut2 += 360;
  // if (sudut3 < 0) sudut3 += 360;
  // if (sudut4 < 0) sudut4 += 360;
  // if (sudut5 < 0) sudut5 += 360;
  // if (sudut6 < 0) sudut6 += 360;
  //printf("sudut1 after mod : %d\n", sudut1);
  //hitung PWM
  pwm1=(sudut1*9.944)+1630; //1/360
  pwm2=(sudut2*10.4)+536.7;
  pwm3=-8.226*sudut3+2182;
  pwm4=11.7*sudut4+655.1;
  pwm5=9.404*sudut5+816; // 1/160
  pwm6=(sudut6*0.0055555)*2000+500;

  //PWM Limiter
  if(pwm1<800){
    pwm1=800;
  }
  if(pwm1>2490){
    pwm1=2490;
  }
  if(pwm2<1000){
    pwm2=1000;
  }
  if(pwm2>2400){
    pwm2=2400;
  }
  if(pwm2>1200&&pwm2<2000){
    if(pwm3>1856){
      pwm3=1856;
    }
    else if(pwm3<600){
      pwm3=600;
    }
  }
  else{
    if(pwm3<1000){
      pwm3=1000;
    }
    if(pwm3>2490){
      pwm3=2490;
    }
  }
  if(pwm4<657){
    pwm4=657;
  }
  if(pwm4>2500){
    pwm4=2500;
  }
  if(pwm5<800){
    pwm5=800;
  }
  if(pwm5>2300){
    pwm5=2300;
  }
}


void double_integrator() {
    
  dq1 = dq1 + ddq1*PERIOD;
  dq2 = dq2 + ddq2*PERIOD;
  dq3 = dq3 + ddq3*PERIOD;
  dq4 = dq4 + ddq4*PERIOD;
  dq5 = dq5 + ddq5*PERIOD;
  dq6 = dq6 + ddq6*PERIOD;

  q1 = q1 + dq1*PERIOD;
  q2 = q2 + dq2*PERIOD;
  q3 = q3 + dq3*PERIOD;
  q4 = q4 + dq4*PERIOD;
  q5 = q5 + dq5*PERIOD;
  q6 = q6 + dq6*PERIOD;


  if(q1*RTD > 360){
  	q1 = q1 - 360*DTR;
  }
  if(q2*RTD > 360){
  	q2 = q2 - 360*DTR;
  }
  if(q3*RTD > 360){
  	q3 = q3 - 360*DTR;
  }
  if(q4*RTD > 360){
  	q4 = q4 - 360*DTR;
  }
  if(q5*RTD > 360){
  	q5 = q5 - 360*DTR;
  }
  if(q6*RTD > 360){
  	q6 = q6 - 360*DTR;
  }
  if(q1*RTD < -360){
  	q1 = q1 + 360*DTR;
  }
  if(q2*RTD < -360){
  	q2 = q2 + 360*DTR;
  }
  if(q3*RTD < -360){
  	q3 = q3 + 360*DTR;
  }
  if(q4*RTD < -360){
  	q4 = q4 + 360*DTR;
  }
  if(q5*RTD < -360){
  	q5 = q5 + 360*DTR;
  }
  if(q6*RTD < -360){
  	q6 = q6 + 360*DTR;
  }

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
  glutSetWindow(window);
  display();
  forward_kinematics();
  if (gerak == 1) 
  {
    trajectory_init();
    //printf("init y=%.2f z=%.2f\n", y_init, z_init);
    while (k <= N*PERIOD) {
      k += PERIOD;
      counter++;
      trajectory_planning();
      hitung_dydz();
      controller();
      inverse_jacobian();
      double_integrator();  
      forward_kinematics();
      if (counter%(N/divider) == 0) {	
        printf("============TASK SPACE MOVEMENT============\n");
        printf("q1 = %.2f q2 = %.2f q3 = %.2f q4 = %.2f q5 = %.2f q6 = %.2f\n",q1*RTD,q2*RTD,q3*RTD,q4*RTD,q5*RTD,q6*RTD);
        printf("x = %.2f y = %.2f z = %.2f\n",x_final,y_final,z_final);

        converts();

  		  int pwm1int = pwm1;
  		  int pwm2int = pwm2;
  		  int pwm3int = pwm3;
  		  int pwm4int = pwm4;
  		  int pwm5int = pwm5;
  		  int pwm6int = pwm6;
  		
		    sprintf(spwm1, "%d", pwm1int);
        sprintf(spwm2, "%d", pwm2int);
        sprintf(spwm3, "%d", pwm3int);
        sprintf(spwm4, "%d", pwm4int);
        sprintf(spwm5, "%d", pwm5int);
        sprintf(spwm6, "%d", pwm6int);
        
        char perintah[40] = "#1P";
         	
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
//		    strcat(perintah, "\r\n");
        char test[2]={0x0D,0x0A};
        printf("%s",perintah);
        write(fd,&perintah,sizeof(perintah));
        write(fd,&test,sizeof(test));
		
      }
      display();
      forward_kinematics();
    }

    //update data from task space movement to joint space movement
    TSJSUpdate();

  }
  else if (gerak == 2)
  {
    while(k <= (N*PERIOD))
    {
      k += PERIOD;
      counter++;      
      controlrobot();
      forward_kinematics();
      if((counter%(N/divider))==0)
      {
        forward_kinematics();
        printf("============JOINT SPACE MOVEMENT============\n");
        printf("q1 = %.2f q2 = %.2f q3 = %.2f q4 = %.2f q5 = %.2f q6 = %.2f\n",q1*RTD,q2*RTD,q3*RTD,q4*RTD,q5*RTD,q6*RTD);
        printf("x = %.2f y = %.2f z = %.2f\n",x_final,y_final,z_final);

        converts();
      
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
          
//    strcat(perintah, servo1);
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
//    strcat(perintah, "\r\n");
            
        char test[2]={0x0D,0x0A};     // simbol untuk /r /n

        write(fd,&perintah,sizeof(perintah));//header // 
        write(fd,&test,sizeof(test));  
        printf("%s \n",perintah);

        dq1=0; q1_awal=q1;dq2=0; q2_awal=q2;
        dq3=0; q3_awal=q3;dq4=0; q4_awal=q4;
        dq5=0; q5_awal=q5;dq6=0; q6_awal=q6;
      }
      display();
    }
    k = 0;
    forward_kinematics();
    x_final = x; y_final = y; z_final = z;
  }
  gerak = 0;
}


void keyboard(unsigned char key, int i, int j)
{
	switch(key){
	 /* Joint Control */
	case ESCkey: exit(1);
  case '1' : x_final = (L2+L5)*100; y_final = L6*100; z_final = (L1+L3+L4)*100; gerak = 1; break;
  case '2' : x_final = x_final+2; gerak = 1; break;
  case '3' : x_final = x_final-2; gerak = 1; break;
  case '4' : y_final = y_final+2; gerak = 1; break;
  case '5' : y_final = y_final-2; gerak = 1; break;
  case '6' : z_final = z_final+2; gerak = 1; break;
  case '7' : z_final = z_final-2; gerak = 1; break;
  case '8' : x_final = L0; y_final = 0; z_final = L1+L2+L3; gerak = 1; break;
	 //case '6' : x_final = 0; y_final = 0; z_final = 0.2; gerak = 1; break;
    case 'q': dq1 += 10*DTR, gerak=2; break;
    case 'Q': dq1 += -10*DTR, gerak=2; break;
    case 'w': dq2 += 10*DTR, gerak=2; break;
    case 'W': dq2 += -10*DTR, gerak=2; break;
    case 'e': dq3 += 10*DTR, gerak=2; break;
    case 'E': dq3 += -10*DTR, gerak=2; break;
    case 'r': dq4 += 10*DTR, gerak=2; break;
    case 'R': dq4 += -10*DTR, gerak=2; break;
    case 't': dq5 += 10*DTR, gerak=2; break;
    case 'T': dq5 += -10*DTR, gerak=2; break;
    case 'y': dq6 += 10*DTR, gerak=2; break;
    case 'Y': dq6 += -10*DTR, gerak=2; break;
	//arah gerak
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
   //fd = open_port();
   //init_port(fd);

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

   /* Register the function to do all our OpenGL drawing. */
   glutIdleFunc(&Sim_main); // fungsi untuk simulasi utama

   /* Start Event Processing Engine */ 
   glutMainLoop () ;
   return 0 ;
}           
