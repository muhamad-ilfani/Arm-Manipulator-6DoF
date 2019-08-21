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
#include <GL/glut.h> // Header File For The GLUT Library
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

#define PERIOD 0.01

float *tetha1=&q1;
float *tetha2=&q2;
float *tetha3=&q3;
float *theta4=&q4;
float *theta5=&q5;
float *theta6=&q6;

int gerak = 0;

float k = 0;
int N = 1000;
float y_init, x_init, y_final, x_final;
float y = 0, y_cmd = 0, y_cmd_old = 0, dy_cmd = 0;
float x = 0, x_cmd = 0, x_cmd_old = 0, dx_cmd = 0;
float z;
float y_old = 0, x_old = 0, dy = 0, ddy = 0, dx = 0, ddx = 0;

float dq1 = 0, dq2 = 0;
float dq1_ref = 0, dq2_ref = 0;
float dq1_refold = 0, dq2_refold = 0;
float ddq1 = 0, ddq2 = 0;
float torque1 = 0, torque2 = 0;
float ddq1_ref = 0, ddq2_ref = 0;

int counter = 0;
int divider = 10;

int m=1;

unsigned char header = '~';

float Kp = 0.05, Kv = 0.45;


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
  	printf("x = %.2f y = %.2f z = %.2f ",x,y,z);
}
void Sim_main(void)
{
	glutSetWindow(window);
  display();

  if (gerak == 1) {
	forward_kinematics();
	printf("q1 = %.2f q2 = %.2f q3 = %.2f \n",q1*RTD,q2*RTD,q3*RTD);
    }
    gerak = 0;
}


void keyboard(unsigned char key, int i, int j)
{
	 switch(key){
	 /* Joint Control */
		case 'q': q1 += 5*DTR, gerak=1; break;
		case 'Q': q1 += -5*DTR, gerak=1; break;
		case 'w': q2 += 5*DTR, gerak=1; break;
		case 'W': q2 += -5*DTR, gerak=1; break;
		case 'e': q3 += 5*DTR, gerak=1; break;
		case 'E': q3 += -5*DTR, gerak=1; break;
		case 'r': q4 += 5*DTR, gerak=1; break;
		case 'R': q4 += -5*DTR, gerak=1; break;
		case 't': q5 += 5*DTR, gerak=1; break;
		case 'T': q5 += -5*DTR, gerak=1; break;
		case 'y': q6 += 5*DTR, gerak=1; break;
		case 'Y': q6 += -5*DTR, gerak=1; break;
	//Kamera	
		case ESCkey: exit(1); break; 
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
   gluPerspective(40.0, 1, 0.2, 8);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
   gluLookAt(0.3, 0.0, 1.0,  0.1, 0.0, 0.4,  0.0, 0.0, 1.0); 
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
