//
// Created by gatsby on 2019-02-28.
//

#include <pangolin/pangolin.h>

int main()
{
	pangolin::CreateWindowAndBind("Main", 640, 480);
	glEnable(GL_DEPTH_TEST);

	// Define projection and initial ModelView matrix
	pangolin::OpenGlRenderState s_cam(pangolin::ProjectionMatrix(640, 480, 420, 420, 320, 240, 0.2, 100),
			pangolin::ModelViewLookAt(-2, 2, -2, 0, 0, 0, pangolin::AxisY));

	// Create Interactive View in window
	pangolin::Handler3D handler(s_cam);
	pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0, 1.0, 0.0, 1.0, -640.f/480.f).SetHandler(&handler);

	while(!pangolin::ShouldQuit()){
		// Clear screen and activate view to render into
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		d_cam.Activate(s_cam);

		// Render OpenGL Cube.
		pangolin::glDrawColouredCube();

		// Swap frames and process Events.
		pangolin::FinishFrame();

	}
	return 0;
}