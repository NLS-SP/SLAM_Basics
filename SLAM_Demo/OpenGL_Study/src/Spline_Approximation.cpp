//
// Created by Robotics_qi on 2020/8/24.
//

#include <GLUT/glut.h>
#include <stdlib.h>
#include <math.h>

// Set initial size of the display window.
GLsizei winWidth = 600, winHeight = 600;

// Set size of world-coordinate clipping window.
GLfloat xwcMin = -50.0, xwcMax = 50.0;
GLfloat ywcMin = -50.0, ywcMax = 50.0;
class wcPt3D{
public:
    GLfloat x, y, z;
};

void init(void){
    // Set color of display window to white.
    glClearColor(1.0, 1.0, 1.0, 1.0);
}

void plotPoint(wcPt3D bezCurvePt){
    glBegin(GL_POINTS);
        glVertex2f(bezCurvePt.x, bezCurvePt.y);
    glEnd();
}

// Compute binomial coefficients C for given value of n.
void binomialCoeffs(GLint n, GLint* C){
    GLint k, j;
    for(k = 0; k <= n; ++k){
        // Compute n!/(k!(n-k!))
        C[k] = 1;
        for(j = n; j >= k+1; --j)
            C[k] *= j;
        for(j = n - k; j >= 2; --j)
            C[k] /= j;
    }
}

void computeBezPt(GLfloat u, wcPt3D* bezPt, GLint nCtrlPts, wcPt3D* ctrlPts, GLint* C){
    GLint k, n = nCtrlPts - 1;
    GLfloat bezBlendFcn;
    bezPt->x = bezPt->y = bezPt->z = 0.0;
    // Compute blending functions and blend control points.
    for(k = 0; k < nCtrlPts; k++){
        bezBlendFcn = C[k] * pow(u, k) * pow(1-u, n-k);
        bezPt->x += ctrlPts[k].x * bezBlendFcn;
        bezPt->y += ctrlPts[k].y * bezBlendFcn;
        bezPt->z += ctrlPts[k].z * bezBlendFcn;
    }
}

void bezier(wcPt3D* ctrlPts, GLint nCtrlPts, GLint nBezCurvePts){
    wcPt3D bezCurvePt;
    GLfloat u;
    GLint *C, k;
    // Allocate space for binomial coefficients.
    C = new GLint[nCtrlPts];

    binomialCoeffs(nCtrlPts - 1, C);
    for(k = 0; k < nBezCurvePts; ++k){
        u = GLfloat(k) / GLfloat(nBezCurvePts);
        computeBezPt(u, &bezCurvePt, nCtrlPts, ctrlPts, C);
        plotPoint(bezCurvePt);
    }
    delete[] C;
}

void displayFcn(void){
    // Set example number of control points and number of curve positions to be plotted along the Bezier curve.
    GLint nCtrlPts = 4, nBezCurvePts = 1000;
    wcPt3D ctrlPts[4] = {{-40.0, -40.0, 0.0}, {-10.0, 200.0, 0.0}, {10.0, -200.0, 0.0}, {40.0, 40.0, 0.0}};
    glClear(GL_COLOR_BUFFER_BIT);

    glPointSize(4);
    glColor3f(1.0, 0.0, 0.0);
    bezier(ctrlPts, nCtrlPts, nBezCurvePts);
    glFlush();
}

void winReshapeFcn(GLint newWidth, GLint newHeight){
    // Maintain an aspect ratio of 1.0;
    glViewport(0, 0, newWidth, newHeight);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(xwcMin, xwcMax, ywcMin, ywcMax);
    glClear(GL_COLOR_BUFFER_BIT);
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
    glutInitWindowPosition(50, 50);
    glutInitWindowPosition(50, 50);
    glutInitWindowSize(winWidth, winHeight);
    glutCreateWindow("Bezier Curve");

    init();
    glutDisplayFunc(displayFcn);
    glutReshapeFunc(winReshapeFcn);
    glutMainLoop();
}

