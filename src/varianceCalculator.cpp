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
		std::vector<std::vector<double> > cut(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
		double getVariance(std::vector<double> values);
		double getMean(std::vector<double> values);
		void saveAsPLY(int frameNum,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
		void saveVectorToFile(const char* fileName,std::vector<double> vector);
		void saveValueToFile(const char* fileName,double value);
		char * getASeriesFileName(std::string fileName,int seriesNum);
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

std::vector<std::vector<double> > VarianceCalculator::cut(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
	std::vector<std::vector<double> > values;
	std::vector<double> x_values;
	std::vector<double> y_values;
	pcl::PointCloud<pcl::PointXYZI>::Ptr seg(new pcl::PointCloud<pcl::PointXYZI>);
	for(long int i = 0;i < cloud->points.size();i++)
	{
        if(cloud->points[i].x > -2.025 && cloud->points[i].x <2.060 && cloud->points[i].y < 1.76 && cloud->points[i].y > 0.876)
		{
			seg->points.push_back(cloud->points[i]);
			x_values.push_back(cloud->points[i].x);
			y_values.push_back(cloud->points[i].y);
		}
	}
	values.push_back(x_values);
	values.push_back(y_values);
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

double VarianceCalculator::getMean(std::vector<double> values)
{
	double sum = std::accumulate(values.begin(), values.end(), 0.0);
	double mean =  sum / values.size();
	return mean;
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

char * VarianceCalculator::getASeriesFileName(std::string fileName,int seriesNum)
{
	std::stringstream ss;
	std::string num;
	ss<<seriesNum;ss>>num;
	fileName = fileName+num+".txt";

	return (char*)fileName.c_str();
}

void VarianceCalculator::callBack(const sensor_msgs::PointCloud2ConstPtr & cloud_msg)
{
    if(frameNum%10 == 0)
	{
		pcl::PCLPointCloud2 * cloud = new pcl::PCLPointCloud2;
		pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZI>);
		pcl_conversions::toPCL(*cloud_msg,*cloud);
		pcl::fromPCLPointCloud2(*cloud,*inputCloud);
		if(!inputCloud->empty())
		{
			std::vector<std::vector<double> > values = cut(inputCloud);
			for(int i = 0;i < 2;i++)
			{
				double mean = getMean(values[i]);
				char * fileName = getASeriesFileName("mean",i);
				saveValueToFile(fileName,mean);
			}
		}
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




