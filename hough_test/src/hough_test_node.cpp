#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Pose2D.h>
#include <math.h>


using namespace cv;
using namespace std;

int hough_slider_max=250;
int v_slider_max = 255;
int center_slider_max = 100;
int canny1_slider_max=200;
int canny2_slider_max=500;
int flag = 0;

int Hough_slider=150;
int v_slider = 220;
int center_slider = 5;
int CannyThres1=115;
int CannyThres2=210;
int first_image_frame=1;

unsigned long int loop=0;

float Xout,Yout;
//Low pass filter parameters
float sampling_time=1.0;
float cutoff_freq=1/0.04;
float alpha;
geometry_msgs::Pose2D KC;

static const std::string OPENCV_WINDOW = "Image window";

class codebridge
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;
	geometry_msgs::Pose2D vanish_point;
	ros::Publisher vanish_pub;
	cv::VideoWriter writer_vanish;
	ros::Subscriber center_kalman;


public:
	codebridge():it_(nh_)
            {
		       // Subscrive to input video feed and publish output video feed
		       image_sub_ = it_.subscribe("/ardrone/bottom/image_raw", 1,&codebridge::imageCb, this);
		       vanish_pub = nh_.advertise<geometry_msgs::Pose2D>("/vanishing_point", 1);
		       center_kalman = nh_.subscribe("/corrected_centers",1,&codebridge::kalmanread,this);
		      //image_pub_ = it_.advertise("/codebridge/output_video", 1);
		       cv::namedWindow(OPENCV_WINDOW);
			   cv::namedWindow("HSLImage");
		    }

	~codebridge()
	     {
	       cv::destroyWindow(OPENCV_WINDOW);
		   cv::destroyWindow("HSLImage");
	     }

	void kalmanread(const geometry_msgs::Pose2D& msg)
	{
		KC.x = msg.x;
		KC.y = msg.y;

	}


	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
//		if (first_image_frame)
//		{
//			string filename_apr = "/home/ashishkb/Desktop/vanish_vid.avi";
//			int fcc = CV_FOURCC('D','I','V','3');
//			int fps = 10;
//			cv::Size frameSize(640,360);
//			writer_vanish = VideoWriter(filename_apr,fcc,fps,frameSize);
//
//			while (!writer_vanish.isOpened()) {
//			cout<<"ERROR OPENING FILE FOR WRITE"<<endl; }
//
//
//			first_image_frame = 0;
//		}
		cv_bridge::CvImagePtr cv_ptr;
		 try
         {
	         cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
         }
		 catch (cv_bridge::Exception& e)
		 {
			 ROS_ERROR("cv_bridge exception: %s", e.what());
			 return;
		 }


		Mat gra,hsv, The_Vid;

		The_Vid = cv_ptr->image.clone();

/*
		    double ticks = 0;
		    bool found = false;

		    int notFoundCount = 0;

	        double precTick = ticks;
	        ticks = (double) cv::getTickCount();

	        double dT = (ticks - precTick) / cv::getTickFrequency(); //seconds

	        cout<<"***************************dT"<<dt<<'\n';
	        */

		//IMAGE SHARPENING
		//GaussianBlur( cv_ptr->image,cv_ptr-> image, Size( 3, 3 ), 0, 0 );
		GaussianBlur( cv_ptr->image,cv_ptr-> image, Size( 0,0 ), 3 , 3);
		addWeighted( The_Vid, 1.5,  cv_ptr->image, -0.5, 0, cv_ptr->image);

		cvtColor( cv_ptr->image, gra, CV_BGR2GRAY );
		//equalizeHist(gra,gra);
		// cvNormalize function call to apply linear stretch
	//	cv::normalize(gra, gra, 0, 255, NORM_MINMAX);
		cvtColor( cv_ptr->image, hsv, CV_BGR2HSV );
		////trackbar
		namedWindow("Manual Tuning", 1);
		createTrackbar( "HSV", "Manual Tuning", &v_slider, v_slider_max);

		waitKey(3);
		cv::inRange(hsv, cv::Scalar(0, 0, v_slider, 0), cv::Scalar(180, 255, 255, 0), hsv);
		// Create a structuring element (SE)
		//int morph_size = 10;
		//Mat element = getStructuringElement( MORPH_RECT, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
		//morphologyEx( hsv, hsv, MORPH_OPEN, element );

		Mat conImg;
		conImg = hsv.clone();
		vector<vector<Point> > contours,biggestContour;
		vector<Vec4i> hierarchy;
		vector<float> areas;
		int largest_area=0;
		int largest_contour_index=0;
		Rect bounding_rect;
		findContours( conImg, contours, hierarchy,CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));

        /*/cout<<"numContours = "<<contours.size()<<'\n';
        /for(int i5 = 0; i5 < contours.size(); i5++ )
        {
        	areas.push_back(contourArea(contours[i5]));
        	vector<Point> approx;
        	//approxPolyDP(contours[i5], approx, 5, true);
        	//double area1 = contourArea(approx);

        	//"area1 =" << area1 << endl <<
        	cout<<"area: "<<contourArea(contours[i5]);
         }*/
		 for( int i = 0; i< contours.size(); i++ ) // iterate through each contour.
		 {
			 double a=contourArea( contours[i],false);  //  Find the area of contour
			 if(a>largest_area)
			 {
		       largest_area=a;
		       largest_contour_index=i;                //Store the index of largest contour
		       bounding_rect=boundingRect(contours[i]); // Find the bounding rectangle for biggest contour
			 }

		 }

        if(largest_area > 8000)
        {
        	drawContours( cv_ptr->image, contours,largest_contour_index , Scalar(0,0,255), 3, 8, hierarchy,0 );
        	cout<<"largestArea: "<<largest_area<<'\n';
        	Moments mu;
        	mu = moments( contours[largest_contour_index], false );
        	///  Get the mass centers:
        	Point2f mc;
        	mc = Point2f( mu.m10/mu.m00 , mu.m01/mu.m00 );
        	circle( cv_ptr->image,mc, 3, Scalar(255,0,0), -1, 8, 0 );
        	//cout<<"cntr points"<<contours[largest_contour_index]<<'\n';
        	vanish_point.x  = mc.x;
        	vanish_point.y  = mc.y;
    		vanish_pub.publish(vanish_point);
        }
		// grab contours
		//biggestContour = contours[contours.size()-1];
        /// Get the moments

		cv::imshow("HSLImage", hsv);
		vector<Vec3f> circles;
		createTrackbar( "Canny edge", "Manual Tuning", &Hough_slider, hough_slider_max);
		createTrackbar( "Center Detection", "Manual Tuning", &center_slider, center_slider_max);

		/// Apply the Hough Transform to find the circles
		HoughCircles( gra, circles, CV_HOUGH_GRADIENT, 1, 50, Hough_slider,center_slider, 0, 0);
//void HoughCircles(InputArray image, OutputArray circles, int method, double dp, double minDist, double param1=100, double param2=100, int minRadius=0, int maxRadius=0 )
		/*
		 Parameters:
image – 8-bit, single-channel, grayscale input image.
circles – Output vector of found circles. Each vector is encoded as a 3-element floating-point vector (x, y, radius) .
circle_storage – In C function this is a memory storage that will contain the output sequence of found circles.
method – Detection method to use. Currently, the only implemented method is CV_HOUGH_GRADIENT , which is basically 21HT , described in [Yuen90].
dp – Inverse ratio of the accumulator resolution to the image resolution. For example, if dp=1 , the accumulator has the same resolution as the input image. If dp=2 , the accumulator has half as big width and height.
minDist – Minimum distance between the centers of the detected circles. If the parameter is too small, multiple neighbor circles may be falsely detected in addition to a true one. If it is too large, some circles may be missed.
param1 – First method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the higher threshold of the two passed to the Canny() edge detector (the lower one is twice smaller).
param2 – Second method-specific parameter. In case of CV_HOUGH_GRADIENT , it is the accumulator threshold for the circle centers at the detection stage. The smaller it is, the more false circles may be detected. Circles, corresponding to the larger accumulator values, will be returned first.
minRadius – Minimum circle radius.
maxRadius – Maximum circle radius.
		 */
		if(circles.size()== 1)
		{
			//cout << "Number of circles: " << circles.size() << '\n';
			/// Draw the circles detected
			Point center(cvRound(circles[0][0]),cvRound(circles[0][1]));
			int radius = cvRound(circles[0][2]);

			//vanish_point.x  = center.x;
			//vanish_point.y  = center.y;
			//vanish_pub.publish(vanish_point);

			//cout<<"loop iterator"<<i<<'\n';
			cout<<"center:" <<center<<'\n';
			cout<<"radius"<<radius<<'\n';
			Point kCx = (Point)KC.x;
			Point kCy = (Point)KC.y;
			cout<<"kCx"<<kCx<<'\n';
			cout<<"kCy"<<kCy<<'\n';

			// circle center
			//circle( cv_ptr->image, center, 3, Scalar(0,255,0), -1, 8, 0 );
			// circle outline
			circle( cv_ptr->image, center, radius, Scalar(0,255,0), 3, 8, 0 );

			// Kalman circle center
			//circle( cv_ptr->image,Point(KC.y,KC.x), 3, Scalar(255,0,0), -1, 8, 0 );
			// circle outline
			//circle( cv_ptr->image,Point(KC.y,KC.x), 5, Scalar(0,255,0), -1, 8, 0 );

			Mat dst, dst_norm, dst_norm_scaled;
			dst = Mat::zeros( gra.size(), CV_32FC1 );

			/// Detector parameters
			//int blockSize = 2;
			//int apertureSize = 3;
			//double k = 0.04;
			int thresh = 200;
			/// Parameters for Shi-Tomasi algorithm
			vector<Point2f> corners;
			double qualityLevel = 0.01;
			double minDistance = 5;
			int blockSize = 10;
			bool useHarrisDetector = false;
			double k = 0.4;
			int maxCorners = 10;
			/// Apply corner detection
			goodFeaturesToTrack( gra,corners,maxCorners,qualityLevel,minDistance,Mat(),blockSize,useHarrisDetector,k );
			//cout<<"Number of corners detected: "<<corners.size()<<endl;

			/// Detecting corners
			//cornerHarris( gra, dst, blockSize, apertureSize, k, BORDER_DEFAULT );
			//cout<<"number of corners: "<<dst.size()<<'\n';
			/// Normalizing
			normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
			convertScaleAbs( dst_norm, dst_norm_scaled );
		  	/// Drawing a circle around corners
			//for( int j = 0; j < dst_norm.rows ; j++ )
			  // { for( int i = 0; i < dst_norm.cols; i++ )
			    //   {
			      //     if( (int) dst_norm.at<float>(j,i) > thresh )
			        //     {
			          //     circle( cv_ptr->image, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
			            // }
			       //}
			    //}
/*			int r = 4,xdiff,ydiff;
			float distance, ratio;
			vector<Point2f> goodCorners;
			vector<float> distances,ratios;

			for( int i = 0; i < corners.size(); i++ )
			{
				  //circle( cv_ptr->image, corners[i], r, Scalar(0), -1, 8, 0 );

				  //cout<<"corner x: "<<corners[i].x<<'\n';
				  //cout<<"corner y: "<<corners[i].y<<'\n';
				  xdiff = pow((corners[i].x - center.x),2);
				  ydiff = pow((corners[i].y - center.y),2);
				  //cout<<"xdiff"<<xdiff<<'\n';
				  //cout<<"ydiff"<<ydiff<<'\n';
				  distance = sqrt(xdiff + ydiff);
				  if (distance <= (radius+5))
				  {
					  //goodCorners[i].x = corners[i].x;
					  //goodCorners[i].y = corners[i].y;
					  distances.push_back(distance);
					  //goodCorners.push_back(corners[i]);
					  //show corners
					  circle( cv_ptr->image, corners[i], r, Scalar(0), -1, 8, 0 );
					  cout<<"Corners: "<<corners[i]<<'\n';
				  }
				  //distance[i] = sqrt(pow((corners[i].x - center.x),2) + pow((corners[i].y - center.y),2));
				  //cout<<"distance from center : "<<distance[i]<<'\n';
			}*/
			//cout<<"Good corners size"<<goodCorners.size()<<'\n';

			// print out content:
			/*cout << "distances:";
			for (vector<float>::iterator it1=distances.begin(); it1!=distances.end(); ++it1)
			{
				cout << ' ' << *it1;
			}
			cout << '\n';
			//for (vector<float>::iterator it2=distances.begin(); it2!=distances.end(); ++it2)
			for(int j= 0;j<=distances.size();j++)
			{
				ratio = radius/ distances[j];
				if (ratio<1.8)
				{
					ratios.push_back(ratio);
					goodCorners.push_back(corners[j]);
				}



			}
			cout<<"****goodCorners"<<goodCorners.size()<<'\n';
			cout<<"ratios";
			for (vector<float>::iterator it3=ratios.begin(); it3!=ratios.end(); ++it3)
			{
				cout<<' '<<*it3;
			}
			cout<<'\n';
			if(goodCorners.size()==3)
			{
				flag = flag + 1;
			}

*/


			//if(goodCorners.size()>3)
						//{
			/*
			float cameraMatrix[3][3]=
				{{700.490828918144, 0, 319.508832099787},
				{0, 701.654116650887, 218.740253550967},
				{ 0, 0, 1}};
			vector<Point3f>objectPoints;
			vector<Point2f>imagePoints;
			objectPoints.push_back(cv::Point3f(0., 0.,0.));
			//objectPoints.push_back(cv::Point3f(-0.5,0.5,0.));
			//objectPoints.push_back(cv::Point3f(0.5,0.5,0.));
			//objectPoints.push_back(cv::Point3f(-0.5,-0.5,0.));
			imagePoints.push_back(center);
			//imagePoints.push_back(first);
			//imagePoints.push_back(second);
			//imagePoints.push_back(third);
			cv::Mat rvec(1,3,cv::DataType<double>::type);
			cv::Mat tvec(1,3,cv::DataType<double>::type);
			vector<float> distCoeffs = {0.0182389759532889, 0.0520276742502367, 0.00651075732801101, 0.000183496184521575, 0};
			solvePnP(objectPoints,imagePoints,cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )
			//}
			  */

		}


		 cv::imshow(OPENCV_WINDOW,cv_ptr->image);
		 cv::waitKey(3);

//////////////////////////////////////MY CODE///////////////////////////////////////////////////

		/* clock_t begin = clock();

		 Mat gra,The_Vid;

		The_Vid=cv_ptr->image.clone();

		//IMAGE SHARPENING
		//GaussianBlur( cv_ptr->image,cv_ptr-> image, Size( 3, 3 ), 0, 0 );
		GaussianBlur( cv_ptr->image,cv_ptr-> image, Size( 0,0 ), 3 , 3);
		addWeighted( The_Vid, 1.5,  cv_ptr->image, -0.5, 0, cv_ptr->image);
		 cvtColor( cv_ptr->image, gra, CV_BGR2GRAY );

		////trackbar

			  namedWindow("Manual Tuning", 1);

			  createTrackbar( "Hough line Threshold", "Manual Tuning", &Hough_slider, hough_slider_max);
			  createTrackbar( "Canny Threshold 1", "Manual Tuning", &CannyThres1, canny1_slider_max);
			  createTrackbar( "Canny Threshold 2", "Manual Tuning", &CannyThres2, canny2_slider_max);

			  waitKey(3);

		////edge detection through canny and display the binary image
		     Mat dst,cdst, edgevideo;
			 Canny(gra, dst, CannyThres1,CannyThres2, 3);
			 cdst=dst.clone();

		////Detect lines using hough transform function
			  vector<Vec2f> lines, vidlines;
				   HoughLines(cdst, lines, 1, CV_PI/180, Hough_slider, 0, 0 );
				   for( size_t i = 0; i < lines.size(); i++ )
				   {
				      float rho = lines[i][0], theta = lines[i][1];
				      Point pt1, pt2;
				      double a = cos(theta), b = sin(theta);
				      double x0 = a*rho, y0 = b*rho;
				       //Draw the lines on the color image
				      if((theta>(10+90)*(CV_PI/180) && theta<(90+80)*(CV_PI/180))||(theta>10*(CV_PI/180) && theta<80*(CV_PI/180)))
				      {

				      pt1.x = cvRound(x0 + 6000*(-b));

				      pt1.y = cvRound(y0 + 6000*(a));

				      pt2.x = cvRound(x0 - 6000*(-b));

				      pt2.y = cvRound(y0 - 6000*(a));

				      line( cv_ptr->image, pt1, pt2, Scalar(255,255,255), 1, CV_AA);

				      }
				   }

		///TEST//////////////////////////////////////////////////DON'T SEE//////////////////////////


		///TEST////////////////////////////////////////////////////////////DON'T SEE ABOVE//////////////////////



		////initialize an matrix for storing b-m values
		      int ct=0;
		      Mat B(10000, 2, DataType<float>::type);
		      for(int i=0;i<10000;i++)
		           {
		         	  for(int j=0;j<2;j++)
		         	  {
		         		 B.at<float>(i,j)=-1;
		         	  }
		           }




		////Storing values of b and m for every value of rho and theta
			   for( size_t i = 0; i < lines.size(); i++ )
			   {
				   float rho = lines[i][0], theta=lines[i][1];
				   Point pt3, pt4;
				   float b,m;
				   if((theta>(10+90)*(CV_PI/180) && theta<(90+80)*(CV_PI/180))||(theta>10*(CV_PI/180) && theta<80*(CV_PI/180)))
				  {
				   b=rho/sin(theta);
		           B.at<float>(ct,0)=b;
		     	   m=cos(theta)/sin(theta);
				   B.at<float>(ct,1)=m;
				//   cout<<b<<"             "<<m<<"_____________________________"<<B.at<float>(ct,0)<<"          "<<B.at<float>(ct,1)<<"\n";
				   ct=ct+1;

				   }
			   }

		////Reading the size of the b-m matrix
		       int sz=0;
			   int numofpt;
			   for(int i=0;i<10000;i++)
		       {
		     	  for(int j=0;j<2;j++)
		     	  {
		     		  if(B.at<float>(i,j)!=-1)
			     	  {
			     		   sz++;
			     	  }

				  }
			   }

			   numofpt=sz/2;


		////RANSAC implementation

			   int ct2;                              //count variable
			   int numinlier;                        //number of inliers
			   int itr=500;                          //required iterations
		       int maxInliers=0;                     //Max number of inliers initialization
		       int maxInliers_num;

		       float x1,y1,x2,y2,xa,ya,dp;           //(x1,y1) & (x2,y2) points connecting a line. (xa,ya) chosen point. dp=length of the line connecting 2 points

		       float slopeBest=0;                    //initialization of slope of best fit line
		       float slope,LineIntercept;            //slope and intercepts of lines
		       float LineInterceptbest=0;            //intercept of best line
		       float inlierThresh=0.3;               //percentage of rejected inliers
		       float inlierRatio=0.4;            //percentage of points chosen as inliers
		       int k;                                //iteration variable upto value of 'itr'
		       int ex=1;                             //a parameter to ensure generation of different random variables
		       int r1,r2;                            //two random variables denotiong index of chosen points
		       Mat chosenpt(2, 2, DataType<float>::type),bestpt(2, 2, DataType<float>::type);
		       Mat inlierind(numofpt, 1, DataType<int>::type);
		       Mat bestinlierind(numofpt, 1, DataType<int>::type);
		       Mat distmat(numofpt, 1, DataType<float>::type),distmatab(numofpt, 1, DataType<float>::type);

		       //Initializing values to -1 in the matrix inlier indexes



if(ct>2)
{
		      //Part of code to find the best line fit
		      srand(time(0));
		      for(k=0;k<=itr;k++)
		       {
		    	//re-initialize inlier indexes
		    	  for(int i=0;i<numofpt;i++)
		    	   {
		    	      inlierind.at<int>(i,0)= (int)-1; //the value of -1 appears for unused positions

		    	   }
		    	  for(int i=0;i<numofpt;i++)
		    	  {
		    	  		distmat.at<float>(i)=-1; //the value of -1 appears for unused positions
		    	  }
		      //Generate two random numbers between 0 to numofpt
		            while(ex==1)
		            {
		    	       r1=rand() %(numofpt) ;
		               r2=rand() %(numofpt) ;
		               if(r1!=r2)
		   		       {
		   	        	  ex=0;
		   		       }
		            }
		        //       cout<<"\n"<<r1<<"\n"<<r2<<"\n";
		            ex=1;
		      //Store the values of the chosen points
		           for(int i=0;i<numofpt;i++)
		           {
		        	   for(int j=0;j<2;j++)
		        	   {
		        		   if(i==r1)
		        		   {
		        			   chosenpt.at<float>(0,0)=B.at<float>(i,0);
		        			   x1=chosenpt.at<float>(0,0);
		        			   chosenpt.at<float>(0,1)=B.at<float>(i,1);
		        			   y1=chosenpt.at<float>(0,1);
		        		   }
		        		   if(i==r2)
		        		   {
		        			   chosenpt.at<float>(1,0)=B.at<float>(i,0);
		        			   x2=chosenpt.at<float>(1,0);
		        			   chosenpt.at<float>(1,1)=B.at<float>(i,1);
		        			   y2=chosenpt.at<float>(1,1);
		        		   }
		        	   }

		           }

		      //Calculate the distance of every point from the lines formed by connecting two points

		           for(int i=0;i<numofpt;i++)
		           {
		            	if(i==r1) //(i!=r1)||(i!=r2))
		            	{
		            		distmat.at<float>(i,0)=(float)0.0;

		            	}
		            	else if(i==r2)
		            	{
		            		distmat.at<float>(i,0)=(float)0.0;
		            	}
		            	else
		            	{
							xa=B.at<float>(i,0);
							ya=B.at<float>(i,1);
							dp=sqrt(pow((y2-y1),2)+pow((x2-x1),2));
							distmat.at<float>(i,0)=abs(((y2-y1)*xa)-((x2-x1)*ya)+(x2*y1)-(y2*x1))/dp;
		            	}

		           }


		      //store the values of inliers in an array that are close the selected line. The closeness is defined by a threshold
		           ct2=0;
		           for(int i=0;i<numofpt;i++)
		           {
		        	   if(distmat.at<float>(i,0)<=inlierThresh)
		        	   {
		        		   inlierind.at<int>(ct2,0)= (int)i;
		        		   ct2=ct2+1;
		        	   }
		           }
		          // cout<<inlierind<<endl;

		      //Find the number of inliers

		           numinlier=0;
		           for(int i=0; i<numofpt;i++)
		           {
		        	   if(inlierind.at<int>(i,0)!=(int)-1)
		        		   numinlier=numinlier+1;
		           }

		      //Error showing the need to increase the threshold. This is because of
		          if((numinlier<round(inlierRatio*numofpt)))
		          {
		        	//  cout<<"\nplease reduce the inlier ratio or increase the number of points by increasing the threshold\n";
		              //break;
		          }
		      //properties of the chosen line
		          if((numinlier>=round(inlierRatio*numofpt))&&(numinlier>maxInliers))
		          {
		        	  maxInliers=numinlier;
		        	  slope=(y2-y1)/(x2-x1);
		        	  LineIntercept = y1-(slope*x1);
		        	  slopeBest=slope;
		        	  LineInterceptbest = LineIntercept;
		        	  maxInliers_num = maxInliers;
		        	  bestpt=chosenpt.clone();
		        	  bestinlierind=inlierind.clone();

		          }
		         // cout << "maxInliers " << maxInliers << endl;


		       }
		      int ct4=0;

		       for(int i=0;i<numofpt;i++)
		       {
		    	   if(bestinlierind.at<int>(i)!=-1)
		         {

		    		   ct4=ct4+1;

		         }
		       }

		////Least Square Fitting implementation
		      Mat chosenInlier(ct4, 2, DataType<float>::type);
		      Mat reqLine(1, 4, DataType<float>::type);
		      Mat mpt(2, 2, DataType<float>::type);
		      Mat bpt(2, 1, DataType<float>::type);
		      Mat XYpoints(2, 1, DataType<double>::type);
		      //store the values of the chosen inliers in an array

		          int ix,ct3=0;
		          for(int i=0;i<numofpt;i++)
		          {
		        		  if(bestinlierind.at<int>(i)!=-1)
		        		  {
		        			      ix=bestinlierind.at<int>(i);
		        			  	  chosenInlier.at<float>(ct3,0)=B.at<float>(ix,0);
		        			  	  chosenInlier.at<float>(ct3,1)=B.at<float>(ix,1);
		        			  	  ct3=ct3+1;

		        		  }
		          }

		      //declare variables for least square fit output
		          float xp,yp,vx,vy; //(xp,yp)- value of b and m in bm space.
		                             //the vector along the chosen line is resolved into vx and vy


		      //Finding the parameters of the best fit line.
		          cv::fitLine(chosenInlier,reqLine,CV_DIST_L2,0,0.01,0.01);

		      //Storing the parameters in their respective variables
		         vx=reqLine.at<float>(0,0);
				 vy=reqLine.at<float>(0,1);
				 xp=reqLine.at<float>(0,2);
				 yp=reqLine.at<float>(0,3);

		      //forming a 2x2 matrix made from two equations -b1=m1 x - y & -b2=m2 x - y
				 mpt.at<float>(0,0)=1;
				 mpt.at<float>(0,1)=-1;
				 mpt.at<float>(1,0)=vy+yp;  //m2
				 mpt.at<float>(1,1)=-yp;    //m1

		      //forming a 2x1 matrix with values of -b1 and -b2
				 bpt.at<float>(0,0)=-xp;
				 bpt.at<float>(1,0)=-(xp+vx);

		      //Finding the values of (x,y) from the given pair of (b1,m1) and (b2,m2)
				 XYpoints= abs((1/(vy))*mpt*bpt);//this array stores the (x,y) value of vanishing point

				 //LOW PASS FILTER
				 loop=loop+1;


										 if(loop>1)
										 {
										 alpha = sampling_time/(sampling_time+(1/cutoff_freq*2*3.14));

										 Xout = abs(alpha*Xout) + (1-alpha)*XYpoints.at<float>(0,0);
										 Yout = abs(alpha*Yout) + (1-alpha)*XYpoints.at<float>(1,0);
										 }
										 else if(loop<=1)
										 {
										 Xout=XYpoints.at<float>(0,0);
										 Yout=XYpoints.at<float>(1,0);

										 }



		      //displaying the vanishing point
				 int rows,cols;

				 cv::Size s = (cv_ptr->image).size();
				 rows = s.height;
				 cols = s.width;


				 Point pt3,pt4,pt5,pt6,pt7;

				 pt3.x = 0;
				 pt3.y = 0;
				 //pt4.x = cvRound(XYpoints.at<float>(0,0));
				 pt4.x = cvRound(Xout);
				 pt4.y = 0;
				 //pt5.x = cvRound(XYpoints.at<float>(0,0));
				 pt5.x = cvRound(Xout);
				 pt5.y = rows;
		         pt6.x = 0;
		         pt6.y = cvRound(Yout);
		         //pt6.y = cvRound(XYpoints.at<float>(1,0));
				 pt7.x = cols;
				 pt7.y = cvRound(Yout);
		         //pt7.y = cvRound(XYpoints.at<float>(1,0));


				 line( cv_ptr->image, pt4, pt5, Scalar(0,0,255), 2, CV_AA);

				 line( cv_ptr->image, pt6, pt7, Scalar(0,0,255), 2, CV_AA);

		    	// imshow("vanish point Image",cv_ptr->image);

				 //vanish_point.x  = cvRound(XYpoints.at<float>(0,0));
				 //vanish_point.y  = cvRound(XYpoints.at<float>(1,0));
				 vanish_point.x  = cvRound(Xout);
				 vanish_point.y  = cvRound(Yout);
				 if((Xout<cols)&&(Yout<rows))
				 {


				 vanish_pub.publish(vanish_point);
				 }
				 cv::imshow(OPENCV_WINDOW, cv_ptr->image);
				// writer_vanish.write(cv_ptr->image);

				 clock_t end = clock();
				 double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

				// cout<<"\n"<<elapsed_secs<<"\n";




				 cv::waitKey(3);
           }
		   else
		   {
			   cv::imshow(OPENCV_WINDOW, cv_ptr->image);
			  // writer_vanish.write(cv_ptr->image);
			   cv::waitKey(3);
		   }

//////////////////////////////////////MY CODE//////////////////////////////////////////////////
*/

	}
};

int main(int argc, char** argv)
{

	 ros::init(argc, argv, "hough_test");
	 codebridge ic;
	 ros::spin();
	 cout<<"number of goodC 3"<<flag<<'\n';
	 return 0;
}


























/*
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui_c.h"
#include "cvplot.h"
//#include <koolplot.h>

#include <iostream>
//#include
//#define rowPtr(imagePtr, dataType, lineIndex) \
	//    (dataType *)(imagePtr->imageData + (lineIndex) * imagePtr->widthStep)

using namespace cv;
using namespace std;

void help()
{
 cout << "\nThis program demonstrates line finding with the Hough transform.\n"
         "Usage:\n"
         "./houghlines <image_name>, Default is pic1.jpg\n" << endl;
}

int main(int argc, char** argv)
{
 const char* filename = argc >= 2 ? argv[1] : "/home/ashishkb/catkin_ws/src/hough_test/src/corridor.jpg";

 Mat src = imread(filename, 0);
 if(src.empty())
 {
     help();
     cout << "can not open " << filename << endl;
     return -1;
 }

 Mat dst, cdst;
 Canny(src, dst, 50, 200, 3);
 cvtColor(dst, cdst, CV_GRAY2BGR);

 #if 0
  vector<Vec2f> lines;
  HoughLines(dst, lines, 1, CV_PI/180, 100, 0, 0 );

  for( size_t i = 0; i < lines.size(); i++ )
  {
     float rho = lines[i][0], theta = lines[i][1];
     Point pt1, pt2;
     double a = cos(theta), b = sin(theta);
     double x0 = a*rho, y0 = b*rho;
     pt1.x = cvRound(x0 + 1000*(-b));
     pt1.y = cvRound(y0 + 1000*(a));
     pt2.x = cvRound(x0 - 1000*(-b));
     pt2.y = cvRound(y0 - 1000*(a));
     line( cdst, pt1, pt2, Scalar(0,0,255), 3, CV_AA);
  }
 #else
  vector<Vec4i> lines;
  HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10 );
  for( size_t i = 0; i < lines.size(); i++ )
  {
    Vec4i l = lines[i];
    line( cdst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
  }
 #endif
 cv::namedWindow("source", CV_WINDOW_NORMAL);
 cv::resizeWindow("source", 500,800);

 cv::namedWindow("detectline test", CV_WINDOW_NORMAL);
 cv::resizeWindow("detectline test", 500,800);
 imshow("source", src);
 imshow("detectline test", cdst);

 //int the_line = 100;
 unsigned char  pb[10]= {1,2,3,4,5,6,7,8,9,10};
 int width = 0;

// *width = 0;
 //template<typename T>
// const * p;

// CvPlot::plot("RGB", pb, 10, 1, 255, 0, 0);
 //CvPlot::label("B");


 waitKey();

 //while(1);

 return 0;
}
*/
