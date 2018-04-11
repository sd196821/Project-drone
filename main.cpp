#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>  
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <string>
using namespace std;
using namespace cv;

int main()
{
	int n = 1;
	string ImgName;
	string OutName1;
	string OutName2;
	Mat src1 = imread("cam (1).jpg");
	imshow("src", src1);
	for ( n = 1; n <= 20;n++ )
	{
		Mat src;
		Mat dst1, dst2;
		ImgName = "cam (";
		OutName1 = "out1(";
		OutName2 = "out2(";
		stringstream ss;
		string str;
		ss << n;
		ss >> str;

		ImgName = ImgName + str + ")" + ".jpg";
		OutName1 = OutName1 + str + ")" + ".jpg";
		OutName2 = OutName2 + str + ")" + ".jpg";
		//Mat src = imread(ImgName);
		src = imread("cam (1).jpg");
		imshow("src", src);

		Point point1(320, 0);//middle top
		Point point2(320, 480);//middle bottom

		for ( int x = 0; x < 640; x++ )
		{
			for ( int y = 0; y < 480; y++ )
			{
				if ( x <= point1.x )
				{
					dst1.at<Vec3b>(Point(x, y))[0] = src.at<Vec3b>(Point(x, y))[0];
					dst1.at<Vec3b>(Point(x, y))[1] = src.at<Vec3b>(Point(x, y))[1];
					dst1.at<Vec3b>(Point(x, y))[2] = src.at<Vec3b>(Point(x, y))[2];
				}
				else
				{
					dst2.at<Vec3b>(Point(x, y))[0] = src.at<Vec3b>(Point(x, y))[0];
					dst2.at<Vec3b>(Point(x, y))[1] = src.at<Vec3b>(Point(x, y))[1];
					dst2.at<Vec3b>(Point(x, y))[2] = src.at<Vec3b>(Point(x, y))[2];
				}
			}
		}
		imwrite(OutName1, dst1);
		imwrite(OutName2, dst2);
		imshow("OUT1", dst1);
		imshow("OUT2", dst2);
	}
	waitKey(30);
	return 0;
}