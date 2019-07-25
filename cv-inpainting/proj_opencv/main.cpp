#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/photo/photo.hpp"

#include <iostream>

using namespace cv;
using namespace std;

static void help()
{
    cout << "\nOpenCV Project \n"
         << "Image Inpainting \n"
         << "Made by: Pedro Silva 72645; Nuno Silva 72708;\n" << endl;

    cout << "\nRunning program: \n"
         "\tmy_inpaint -h --> print help\n"
         "\tmy_inpaint -d --> run inpainting module with default image lena.jpg\n"
         "\tmy_inpaint -s source_img masked_img --> Given a masked_img run inpainting algorithm on the source_img and in the end show all the images of the inpaiting\n"
         "\tmy_inpaint source_img --> run the inpainting module with the mouse aplication\n" << endl;

}



static void helpmodule()
{
    cout << "\nOpenCV Project \n"
         << "Image Inpainting \n"
         << "Made by: Pedro Silva 72645; Nuno Silva 72708;\n" << endl;

    cout << "\nCool inpainting module. Inpainting repairs damage to images by floodfilling the damage "
            << "with surrounding image areas.\n"
            "Using OpenCV version " << CV_VERSION << "\n"
    "Usage:\n"
        "\t my_inpaint [image_name -- Default lena.jpg]\n" << endl;

    cout << "Hot keys: \n"
        "\tESC - quit the program\n"
        "\tr - restore the original image\n"
        "\ti or SPACE - run inpainting algorithm\n"
        "\t\t(before running it, paint something on the image)\n" << endl;
}

Mat img, inpaintMask;
Point prevPt(-1,-1);

static void onMouse( int event, int x, int y, int flags, void* )
{
    if( event == CV_EVENT_LBUTTONUP || !(flags & CV_EVENT_FLAG_LBUTTON) )
        prevPt = Point(-1,-1);
    else if( event == CV_EVENT_LBUTTONDOWN )
        prevPt = Point(x,y);
    else if( event == CV_EVENT_MOUSEMOVE && (flags & CV_EVENT_FLAG_LBUTTON) )
    {
        Point pt(x,y);
        if( prevPt.x < 0 )
            prevPt = pt;
        line( inpaintMask, prevPt, pt, Scalar::all(255), 5, 8, 0 );
        line( img, prevPt, pt, Scalar::all(255), 5, 8, 0 );
        prevPt = pt;
        imshow("image", img);
    }
}

int showImgInpainting(int argc, char** argv)
{
     if(argc != 4){
        cout << "Wrong number of input arguments!\n"
        << "Use: my_inpaint -s [source_image mask_image]\n" << endl;
        return -1;
     }
     char* source_filename = argv[2];
     char* mask_filename = argv[3];
     Mat source_img = imread(source_filename, CV_LOAD_IMAGE_UNCHANGED);
     Mat mask_img = imread(mask_filename, CV_LOAD_IMAGE_UNCHANGED);

     if(source_img.empty()){
        cout << "Could not open the source image!\n" << source_filename << endl;
        return -1;
     }
     if(mask_img.empty()){
        cout << "Could not open the mask image!\n" << mask_filename << endl;
     }

     namedWindow("Source Image", CV_WINDOW_AUTOSIZE);
     namedWindow("Mask Image", CV_WINDOW_AUTOSIZE);
     namedWindow("Inpainted Image");

     //calculate the diference;
     Mat diff_img = mask_img - source_img;
     Mat grey_diff, th_diff;
     cvtColor(diff_img, grey_diff, CV_BGR2GRAY);

     //threshold(src_gray, dst, threshold_value, max_BINARY_value, threshold_type);
     // Use basic binary threshold. Any pixel above 10 is assigned a 255 value;
     threshold(grey_diff, th_diff, 10, 255, 0);

     //Ready to call the inpainting function
     Mat inpainted_img = source_img.clone();
     // Using Alexandru Telea Method
     inpaint(source_img, th_diff, inpainted_img, 3, CV_INPAINT_TELEA);
     // Using Navier-Stokes based method;
     //inpaint(source_img, th_diff, inpainted_img, 3, CV_INPAINT_NS);

     imshow("Source Image", source_img);
     imshow("Mask Image", mask_img);
     imshow("Inpainted Image", inpainted_img);

     waitKey(0);
     destroyWindow( "Source Image" );
     destroyWindow( "Mask Image" );
     destroyWindow( "Inpainted Image" );

     return 0;
}

int defaultInpaiting(char* default_filename){
     cout << default_filename << endl;
     Mat img0 = imread(default_filename, -1);
     if(img0.empty())
     {
        cout << "Couldn't open the image " << default_filename << ". Usage: my_inpaint <image_name>\n" << endl;
        return 0;
     }

     helpmodule();

     namedWindow( "image", 1 );

     img = img0.clone();
     inpaintMask = Mat::zeros(img.size(), CV_8U);

     imshow("image", img);
     setMouseCallback( "image", onMouse, 0 );

     for(;;)
     {
        char c = (char)waitKey();

        if( c == 27 )
            break;

        if( c == 'r' )
        {
            inpaintMask = Scalar::all(0);
            img0.copyTo(img);
            imshow("image", img);
        }

        if( c == 'i' || c == ' ' )
        {
            //call the inpainting function;
            Mat inpainted;
            // Using the Alexandru Telea method;
            inpaint(img, inpaintMask, inpainted, 3, CV_INPAINT_TELEA);
            // Using the Navier-Strokes based method;
            // inpaint(img, inpaintMask, inpainted, 3, CV_INPAINT_NS);
            imshow("inpainted image", inpainted);
        }
     }

     return 0;
}


int main( int argc, char** argv )
{
    if(strcmp(argv[1], "-h") == 0){
        cout << argv[1] << "calling help" << endl;
        help();
        return -1;
    }
    else if(strcmp(argv[1], "-s") == 0){
        cout << argv[1] << endl;
        return showImgInpainting(argc, argv);
    }
    else if(strcmp(argv[1], "-d") == 0){
        cout << argv[1] << endl;
        return defaultInpaiting((char*)"lena.jpg");
    }
    else{
        cout << argc << endl;
        char* filename;
        if(argc >= 2){
            filename = argv[1];
        }
        else{
            cout << "Can't read image!\n" << endl;
            return 0;
        }

        //char* filename = argc >= 2 ? argv[1] : (char*)"fruits.jpg";
        cout << filename << endl;
        Mat img0 = imread(filename, -1);
        if(img0.empty())
        {
            cout << "Couldn't open the image " << filename << ". Usage: my_inpaint <image_name>\n" << endl;
            return 0;
        }

        helpmodule();

        namedWindow( "image", 1 );

        img = img0.clone();
        inpaintMask = Mat::zeros(img.size(), CV_8U);

        imshow("image", img);
        setMouseCallback( "image", onMouse, 0 );

        for(;;)
        {
            char c = (char)waitKey();

            if( c == 27 )
                break;

            if( c == 'r' )
            {
                inpaintMask = Scalar::all(0);
                img0.copyTo(img);
                imshow("image", img);
            }

            if( c == 'i' || c == ' ' )
            {
                //call the inpainting function;
                Mat inpainted;
                // Using the Alexandru Telea method;
                inpaint(img, inpaintMask, inpainted, 3, CV_INPAINT_TELEA);
                // Using the Navier-Strokes based method;
                // inpaint(img, inpaintMask, inpainted, 3, CV_INPAINT_NS);
                imshow("inpainted image", inpainted);
            }
        }

        return 0;
    }
}


