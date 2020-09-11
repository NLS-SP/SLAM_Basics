//
// Created by Robotics_qi on 2020/8/24.
//

#include <GLUT/glut.h>

GLsizei winWidth = 500, winHeight = 500;

void init(void){
    glClearColor(1.0, 1.0, 1.0, 1.0);
}

void wireQuadSurfs(void){
    glClear(GL_COLOR_BUFFER_BIT);

    glColor3f(0.0, 0.0, 1.0);

    gluLookAt(2.0, 2.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);

    glPushMatrix();
    glTranslatef(1.0, 1.0, 0.0);
    glutWireSphere(0.75, 8, 6);
    glPopMatrix();

    // Position and display GLUT wire-frame cone.
    glPushMatrix();
    glTranslatef(1.0, -0.5, -0.5);
    glutWireCone(0.7, 2.0, 7, 6);
    glPopMatrix();

    // Position and display GLU wire-frame cylinder.
    GLUquadricObj* cylinder;
    glPushMatrix();
    glTranslatef(0.0, 1.2, 0.8);
    cylinder = gluNewQuadric();
    gluQuadricDrawStyle(cylinder, GLU_LINE);
    gluCylinder(cylinder, 0.6, 0.6, 1.5, 6, 4);
    glPopMatrix();

    glFlush();
}

void winReshapeFcn(GLint newWidth, GLint newHeight){
    glViewport(0, 0, newWidth, newHeight);

    glMatrixMode(GL_PROJECTION);
    glOrtho(-2.0, 2.0, -2.0, 2.0, 0.0, 5.0);
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT);
}

int main(int argc, char**argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
   glutInitWindowPosition(100, 100);
   glutInitWindowSize(winWidth, winHeight);
   glutCreateWindow("Wire-Frame Quadric Surfaces");

   init();
   glutDisplayFunc(wireQuadSurfs);
   glutReshapeFunc(winReshapeFcn);
   glutMainLoop();
}