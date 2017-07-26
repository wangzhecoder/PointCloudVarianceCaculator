/*
 * VarianceCalculator.cpp
 *
 *  Created on: 2017年7月24日
 *      Author: wz
 */
#include "ros/ros.h"
#include "ros/console.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <string>
#include <numeric>
#include <fstream>
#include <algorithm>
#include <pcl/visualization/cloud_viewer.h>

class VarianceCalculator{
	public:
		VarianceCalculator();
		void callBack(const sensor_msgs::PointCloud2ConstPtr & cloud_msg);
		std::vector<double> cut(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
		double getVariance(std::vector<double> values);
		void saveAsPLY(int frameNum,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
		void saveVectorToFile(const char* fileName,std::vector<double> vector);
		void saveValueToFile(const char* fileName,double value);
	private:
		ros::NodeHandle _nh;
		ros::Subscriber pointCloud2_sub;
		long frameNum;
		std::vector<double> variance;
		pcl::visualization::CloudViewer viewer;
};

VarianceCalculator::VarianceCalculator()
	:frameNum(0),
	 viewer("Cloud Viewer")
{
	pointCloud2_sub = _nh.subscribe("rslidar_points",1,&VarianceCalculator::callBack,this);
	variance.clear();
}

std::vector<double> VarianceCalculator::cut(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	std::vector<double> values;
	pcl::PointCloud<pcl::PointXYZI>::Ptr seg(new pcl::PointCloud<pcl::PointXYZI>);
	for(long int i = 0;i < cloud->points.size();i++)
	{
		if(cloud->points[i].x > -2.025 && cloud->points[i].x <4.276 && cloud->points[i].y < 1.76 && cloud->points[i].y > 0.476)
		{
			seg->points.push_back(cloud->points[i]);

			values.push_back(cloud->points[i].y);
		}
	}
	viewer.showCloud(seg);
	return values;
}

double VarianceCalculator::getVariance(std::vector<double> values)
{
	double sum = std::accumulate(values.begin(), values.end(), 0.0);
	double mean =  sum / values.size();

	saveValueToFile("mean.txt",mean);

	double accum  = 0.0;
//	std::for_each (values.begin(), values.end(), [&](const double d) {
//	    accum  += (d-mean)*(d-mean);
//	});
	std::vector<double>::iterator i;
	for(i = values.begin();i!=values.end();i++)
	{
		accum  += (*i-mean)*(*i-mean);
	}

	double stdev = sqrt(accum/(values.size()-1));
	return stdev;
}

void VarianceCalculator::saveVectorToFile(const char* fileName,std::vector<double> vector)
{
	std::ofstream output(fileName,std::ios::out);
	if(! output)
	{
		std::cerr << "Open output file error!" << std::endl;
		std::exit(-1);
	}
	std::vector<double>::iterator i;
	for(i = vector.begin();i != vector.end();i++)
	{
		output << *i << std::endl;
	}
	output.close();
}

void VarianceCalculator::saveValueToFile(const char* fileName,double value)
{
	std::ofstream output(fileName,std::ios::app);
	if(! output)
	{
		std::cerr << "Open output file error!" << std::endl;
		std::exit(-1);
	}
		output << value << std::endl;
	output.close();
}

void VarianceCalculator::saveAsPLY(int frameNum,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	std::string fileNames = "rslidar-";
	std::stringstream ss;
	std::string num;
	ss<<frameNum;ss>>num;
	fileNames = fileNames+num+".ply";
	pcl::io::savePLYFile(fileNames,*cloud);
	ROS_INFO("rslidar saved");
}

void VarianceCalculator::callBack(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
{
	if(frameNum%100 == 0)
	{
		pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
		pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl_conversions::toPCL(*cloud_msg,*cloud);
		pcl::fromPCLPointCloud2(*cloud,*inputCloud);

//		viewer.showCloud(inputCloud);

		std::vector<double> y_values = cut(inputCloud);
		double stdev = getVariance(y_values);
//		saveValueToFile("variance.txt",stdev);
	}
	frameNum++;
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"p2f");
	VarianceCalculator c;
	ros::spin();
	return 0;
}




