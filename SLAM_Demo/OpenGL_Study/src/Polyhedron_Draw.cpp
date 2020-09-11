//
// Created by Robotics_qi on 2020/8/24.
//

#include <GLUT/glut.h>
GLsizei winWidth = 500, winHeight = 500;

void init(void){
    glClearColor(1.0, 1.0, 1.0, 1.0);   // White Display Window.
}

void displayWirePolyHedra(void){
    glClear(GL_COLOR_BUFFER_BIT);   // Create Display Window.
    glColor3f(0.0, 0.0, 1.0);   // set line color to blue.
    // Set viewing transformation.
    gluLookAt(5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    // Scale Cube and display as wire-frame parallelepiped.
    glScalef(1.5, 2.0, 1.0);
    glutWireCube(1.0);

    // Scale Translate and display wire-frame dodecahedron.
    glScalef(0.8, 0.5, 0.8);
    glTranslatef(-6.0, -5.0, 0.0);
    glutWireDodecahedron();

    // Translate and display wire-frame tetrahedron.
    glTranslatef(8.6, 8.6, 2.0);
    glutWireTetrahedron();

    // Translate and display wire-frame octahedron.
    glTranslatef(-3.0, -1.0, 0.0);
    glutWireOctahedron();

    // Scale, translate and display  wire-frame icosahedron.
    glScalef(0.8, 0.8, 1.0);
    glTranslatef(4.3, -2.0, 0.5);
    glutWireIcosahedron();
    glFlush();
}

void winReshapeFcn(GLint newWidth, GLint newHeight){
    glViewport(0, 0, newWidth, newHeight);
    glMatrixMode(GL_PROJECTION);
    glFrustum(-1.0, 1.0, -1.0, 1.0, 2.0, 20.0);
    glMatrixMode(GL_MODELVIEW);
    glClear(GL_COLOR_BUFFER_BIT);
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowPosition(100, 100);
    glutInitWindowSize(winWidth, winHeight);
    glutCreateWindow("Wire-Frame Polyhedra");

    init();
    glutDisplayFunc(displayWirePolyHedra);
    glutReshapeFunc(winReshapeFcn);

    glutMainLoop();

    return 0;
}
