
#include <opencv2/line_descriptor.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace cv::line_descriptor;
using namespace std;

int main(int argc, char** argv)
{
    std::string img_address = "/Users/robotics_qi/SLAM_Basics/SLAM_Demo/Frontend/Features/000000.png";
    Mat image = imread(img_address, IMREAD_GRAYSCALE);
    if( image.empty() )
    {
        return -1;
    }

    /* create a random binary mask */
    cv::Mat mask = Mat::ones( image.size(), CV_8UC1 );

    /* create a pointer to a BinaryDescriptor object with deafult parameters */
    Ptr<LSDDetector> bd = LSDDetector::createLSDDetector(); /* create a structure to store extracted lines */
    vector<KeyLine> lines;

    /* extract lines */
    cv::Mat output = image.clone();
    bd->detect( image, lines, 2, 1, mask );

    /* draw lines extracted from octave 0 */
    if( output.channels() == 1 )
        cvtColor( output, output, COLOR_GRAY2BGR );
    for ( size_t i = 0; i < lines.size(); i++ )
    {
        KeyLine kl = lines[i];
        if( kl.octave == 0)
        {
            /* get a random color */
            int R = ( rand() % (int) ( 255 + 1 ) );
            int G = ( rand() % (int) ( 255 + 1 ) );
            int B = ( rand() % (int) ( 255 + 1 ) );

            /* get extremes of line */
            Point pt1 = Point2f( kl.startPointX, kl.startPointY );
            Point pt2 = Point2f( kl.endPointX, kl.endPointY );

            /* draw line */
            line( output, pt1, pt2, Scalar( B, G, R ), 3 );
        }

    }

    /* show lines on image */
    imshow( "LSD lines", output );
    waitKey();

    return 0;
}