#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>
#include <ctime>



#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <iostream>
#include <vector>
#include <ctime>

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>





using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;
//VARIABILI GLOBALI
	//our visualizer
pcl::visualization::PCLVisualizer *p;
    //its left and right viewports
int vp_1, vp_2;
    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Display source and target on the first viewport of the visualizer
     *
     */
    void showCloudsLeft(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source)
    {
      p->removePointCloud ("vp1_target");
      p->removePointCloud ("vp1_source");

      PointCloudColorHandlerCustom<pcl::PointXYZRGB> tgt_h (cloud_target, 0, 255, 0);
      PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_h (cloud_source, 255, 0, 0);
      p->addPointCloud (cloud_target, tgt_h, "vp1_target", vp_1);
      p->addPointCloud (cloud_source, src_h, "vp1_source", vp_1);

      p-> spin();
    }


    ////////////////////////////////////////////////////////////////////////////////
    /** \brief Display source and target on the second viewport of the visualizer
     *
     */


    void showCloudsRight(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_target, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_source)
    {
      p->removePointCloud ("source");
      p->removePointCloud ("target");



      PointCloudColorHandlerCustom<pcl::PointXYZRGB> tgt_color_handler (cloud_target, 0, 255, 0);
      PointCloudColorHandlerCustom<pcl::PointXYZRGB> src_color_handler (cloud_source, 255, 0, 0);


      //p->addPointCloud (cloud_target, tgt_color_handler, "target", vp_2);
      p->addPointCloud (cloud_source, src_color_handler, "source", vp_2);

      p->spin();
    }

    std::vector<int> confrontaImmagini(pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB, std::vector<int> newPointIdxVector, int temp, int i, char** argv){

    	std::cout << "valore base" << temp << std::endl;
    	std::cout << "valore immagine confronto" << i << std::endl;
    	newPointIdxVector.erase (newPointIdxVector.begin(),newPointIdxVector.end());
    	std::cout << newPointIdxVector.size () << std::endl;
    	pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[temp], *cloudA);//* load the file
    	octree.setInputCloud (cloudA);
    	octree.addPointsFromInputCloud ();

    	// Switch octree buffers: This resets octree but keeps previous tree structure in memory.
    	octree.switchBuffers ();



    	// Generate pointcloud data for cloudB
    	pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[i], *cloudB); //* load the file
    	// Add points from cloudB to octree
    	octree.setInputCloud (cloudB);
    	octree.addPointsFromInputCloud ();
    	// Get vector of point indices from octree voxels which did not exist in previous buffer
    	octree.getPointIndicesFromNewVoxels (newPointIdxVector);

    	return newPointIdxVector;

    }



int
main (int argc, char** argv)
{
  srand ((unsigned int) time (NULL));

  // Octree resolution - side length of octree voxels
  float resolution = 32.0f;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZRGB> octree (resolution);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudA (new pcl::PointCloud<pcl::PointXYZRGB> );
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudB (new pcl::PointCloud<pcl::PointXYZRGB> );

  std::vector<int> newPointIdxVector;
  std::cout << newPointIdxVector.size () << std::endl;




  // Generate pointcloud data for cloudA CARICARE NUVOLA

  int temp=1; //temp tiene traccia dell ultimo movimento

  for(int i=1; i<argc; i++)
  {
	  if(newPointIdxVector.size ()<800){


		  newPointIdxVector=confrontaImmagini(octree, cloudA, cloudB, newPointIdxVector, temp, i, argv);

		  /*
		 std::cout << "valore base" << temp << std::endl;
		 std::cout << "valore immagine confronto" << i << std::endl;
		 newPointIdxVector.erase (newPointIdxVector.begin(),newPointIdxVector.end());
		 std::cout << newPointIdxVector.size () << std::endl;
		 pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[temp], *cloudA);//* load the file
		 octree.setInputCloud (cloudA);
		 octree.addPointsFromInputCloud ();

		   // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
		 octree.switchBuffers ();



		   // Generate pointcloud data for cloudB
		 pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[i], *cloudB); //* load the file
		   // Add points from cloudB to octree
		 octree.setInputCloud (cloudB);
		 octree.addPointsFromInputCloud ();
		   // Get vector of point indices from octree voxels which did not exist in previous buffer
		 octree.getPointIndicesFromNewVoxels (newPointIdxVector);
		*/

	  }else{


		  std::cout << newPointIdxVector.size () << std::endl;
		  newPointIdxVector.erase (newPointIdxVector.begin(),newPointIdxVector.end());

		  std::cout << temp  << std::endl;
		  std::cout << "movimento"  << std::endl;
		  std::cout <<  i-1  << std::endl;

		  temp=i-1;



		  newPointIdxVector=confrontaImmagini(octree, cloudA, cloudB, newPointIdxVector, temp, i, argv);


		  /*

		  std::cout << newPointIdxVector.size () << std::endl;
		  pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[temp], *cloudA);//* load the file
		  octree.setInputCloud (cloudA);
		  octree.addPointsFromInputCloud ();

		  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
		  octree.switchBuffers ();



		  // Generate pointcloud data for cloudB
		  pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[i], *cloudB); //* load the file
		  // Add points from cloudB to octree
		  octree.setInputCloud (cloudB);
		  octree.addPointsFromInputCloud ();
		  // Get vector of point indices from octree voxels which did not exist in previous buffer
		  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

		 */

	  }

  }

/*

  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[1], *cloudA) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
    return (-1);
  }

*/

  //remove NAN points from the cloud
  /*std::vector<int> indices;
  pcl::removeNaNFromPointCloud(cloudA,cloudA, indices);*/

/*

// Add points from cloudA to octree
  octree.setInputCloud (cloudA);
  octree.addPointsFromInputCloud ();

  // Switch octree buffers: This resets octree but keeps previous tree structure in memory.
  octree.switchBuffers ();



  // Generate pointcloud data for cloudB
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (argv[2], *cloudB) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

  // Add points from cloudB to octree
  octree.setInputCloud (cloudB);
  octree.addPointsFromInputCloud ();



  // Get vector of point indices from octree voxels which did not exist in previous buffer
  octree.getPointIndicesFromNewVoxels (newPointIdxVector);

  // Output points

*/

  /*
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudC (new pcl::PointCloud<pcl::PointXYZRGB> );






     cloudC->width = newPointIdxVector.size () ;
     cloudC->height = 1;
     cloudC->points.resize (cloudC->width * cloudC->height);



  for (size_t i = 0; i < newPointIdxVector.size (); ++i){

	  cloudC->points[i].x = cloudB->points[newPointIdxVector[i]].x;
	  cloudC->points[i].y = cloudB->points[newPointIdxVector[i]].y;
	  cloudC->points[i].z = cloudB->points[newPointIdxVector[i]].z;
  }



//std::cout << newPointIdxVector.size () << std::endl;




  p = new pcl::visualization::PCLVisualizer (argc, argv, "Pairwise Incremental Registration example");
  p->createViewPort (0.0, 0, 0.5, 1.0, vp_1);
  p->createViewPort (0.5, 0, 1.0, 1.0, vp_2);


  // Add visualization data
  showCloudsLeft(cloudA, cloudC);

  showCloudsRight(cloudA, cloudC);

  pcl::io::savePCDFileASCII<pcl::PointXYZRGB> ("provaDiff.pcd", *cloudC);


*/



}
