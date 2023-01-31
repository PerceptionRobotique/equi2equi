/*!
 \file equi2equi.cpp
 \brief Spherical transformations of equirectangular to equirectangular mapping (RGB image format), exploiting PeR core and io modules
 * example of command line :

  ./equi2equi ../media/ 0 9 1 ../media/poses.txt 1
or
  ./equi2equi ../media/ 0 9 1 ../media/poses.txt 0 ../media/depthmaps/

 \param imagesDir directory where equirectangular images to read (with 6 digits before the extension) are and where the output equirectangular images will be written (with characters 'e_' before the 6 digits)
 \param iFirst the number of the first image to transform
 \param iLast the number of the last image to transform
 \param iStep the increment to the next image to transform
 \param posesFic a text file of one 3D pose per image (one row - one image) stored as the 3 elements of the translation vector followed by the 3 elements of the axis-angle vector representation of the rotation
 \param iInvertPose a flag to let the program knows if poses of posesFic must be applied to images as they are or inversed
  \param depthMapsDir directory where the depth maps of the equirectangular images are to read (with 6 digits before the extension, numbering matching the one of images); in this case, forward mapping will be applied (probably holes in the resulting image)

 *
 \author Guillaume CARON
 \version 0.1
 \date January 2023
*/

#include <iostream>
#include <iomanip>

#include <per/prcommon.h>
#include <per/prEquirectangular.h>

#include <boost/regex.hpp>
#include <boost/filesystem.hpp>

// VISP includes
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImageTools.h>

#include <visp/vpTime.h>

#include <visp/vpDisplayX.h>

#define INTERPTYPE prInterpType::IMAGEPLANE_BILINEAR

#define VERBOSE

/*!
 * \fn main()
 * \brief Main function of the dual fisheye to equirangular spherical image transformation
 *
 * 1. Loading a divergent stereovision system made of two fisheye cameras considering the Barreto's model from an XML file got from the MV calibration software
 *
 *
 */
int main(int argc, char **argv)
{  
    //Loading the reference image with respect to which the cost function will be computed
    if(argc < 2)
    {
#ifdef VERBOSE
        std::cout << "no image files directory path given" << std::endl;
#endif
        return -4;
    }

    //Get filename thanks to boost
    char myFilter[1024];
    char *chemin = (char *)argv[1];
    char ext[] = "png";
    
    if(argc < 3)
    {
#ifdef VERBOSE
        std::cout << "no initial image file number given" << std::endl;
#endif
        return -6;
    }
    unsigned int i0 = atoi(argv[2]);//1;//0;
    
    if(argc < 4)
    {
#ifdef VERBOSE
        std::cout << "no image files count given" << std::endl;
#endif
        return -7;
    }
    unsigned int i360 = atoi(argv[3]);
    
    if(argc < 5)
    {
#ifdef VERBOSE
        std::cout << "no image sequence step given" << std::endl;
#endif
        return -8;
    }
    unsigned int iStep = atoi(argv[4]);
    
    
    //fichier avec les poses initiales r_0
    bool ficInit = false;
    std::vector<vpPoseVector> v_pv_init;
    if(argc < 6)
    {
#ifdef VERBOSE
        std::cout << "no initial poses file given" << std::endl;
#endif
        //return -9;
    }
    else
    {
        ficInit = true;
        
        std::ifstream ficPosesInit(argv[5]);
        if(!ficPosesInit.good())
        {
#ifdef VERBOSE
            std::cout << "poses file " << argv[5] << " not existing" << std::endl;
#endif
            return -9;
        }
        vpPoseVector r;
        while(!ficPosesInit.eof())
        {
            ficPosesInit >> r[0] >> r[1] >> r[2] >> r[3] >> r[4] >> r[5];
						//r[0] = r[1] = r[2] = 0;
						//r[3] -= M_PI;
						//r[5] = r[5];
						//std::cout << r.t() << std::endl;
            v_pv_init.push_back(r);
        }
        ficPosesInit.close();
    }
    
    //direct or inverse pose
    unsigned int inversePose = 0;
    if(argc < 7)
    {
#ifdef VERBOSE
        std::cout << "no stabilisation parameter given" << std::endl;
#endif
        //return -9;
    }
    else
        inversePose = atoi(argv[6]);
        
        
    //Add depthmap and only in this case the translations of v_pv_init will not be ignored
    bool depthSoForwardTransf = false;
		//Loading the depthmap of the reference image with respect to which the cost function will be computed
    vpImage<float> depthmap_se, depthmap_te;
    char *depthmapPath = NULL;
    if(argc < 8)
    {
#ifdef VERBOSE
        std::cout << "no depthmap files directory path given" << std::endl;
#endif
        
    }
    else
    {
    	depthmapPath = (char *)argv[7];
    	depthSoForwardTransf = true;
    }

    //Get filename thanks to boost
    char myFilter_depth[1024];
		char ext_depthmap[] = "pfm";
    

    vpDisplayX disp;
    vpDisplayX disp2;
    vpDisplayX disp_depthmap;
    
    vpImage<unsigned char> depthmap_se_disp;
    
    //Pour chaque image du dataset
    int nbPass = 0;
    bool clickOut = false;
    unsigned int imNum = i0;
    double temps;
    std::vector<double> v_temps;
    v_temps.reserve((i360-i0)/iStep);
    
    vpHomogeneousMatrix cMc0, erMdf;
    erMdf.buildFrom(0, 0, 0, 0, 0, 0);//.5*M_PI);
    
    
    std::ostringstream s;
    std::string filename;
    
    //save the actual poses used to transform equirectangular images
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << argv[5] << "_updt.txt";
    filename = s.str();
    std::ofstream ficPosesInit_updt(filename);
    
    vpImage<vpRGBa> I_se; //Source Equirectangular image
    vpImage<vpRGBa> I_te; //Transformed Equirectangular image
    
    boost::filesystem::path dir(chemin);
    boost::regex my_filter;
		boost::filesystem::path dir_depthmap;
		if(depthSoForwardTransf)
			dir_depthmap=boost::filesystem::path(depthmapPath);
			
		boost::regex my_filter_depthmap;
    std::string name;
    
    prEquirectangular equirectCam;
    prPointFeature P;
    double Xs, Ys, Zs, u, v, dv, du, unmdv, unmdu;
    unsigned int icam, imWidth, imHeight;
    unsigned int i, j;
    vpRGBa curPix, *pt_bitmap_te, *pt_bitmap_se;
    float *pt_depthmap_se;
    while(!clickOut && (imNum <= i360))
    {
        temps = vpTime::measureTimeMs();
        std::cout << "num request image : " << nbPass << std::endl;
        
        //load source image
        sprintf(myFilter, "e_%06d.*\\.%s", imNum, ext);
        
        my_filter.set_expression(myFilter);
        
        for (boost::filesystem::directory_iterator iter(dir),end; iter!=end; ++iter)
        {
            name = iter->path().leaf().string();
            if (boost::regex_match(name, my_filter))
            {
                std::cout << iter->path().string() << " loaded" << std::endl;
                vpImageIo::read(I_se, iter->path().string());
                break;
            }
        }
        if(nbPass == 0)
        {
            imWidth = I_se.getWidth();
            imHeight = I_se.getHeight();
            I_te.resize(imHeight, imWidth);
            equirectCam.init(imWidth*0.5/M_PI, imHeight*0.5/(M_PI*0.5), imWidth*0.5, imHeight*0.5);
            
            disp.init(I_se, 50, 50, "Source Equirectangular");
            disp2.init(I_te, 500, 50, "Transformed Equirectangular");
        }

        vpDisplay::display(I_se);
        vpDisplay::flush(I_se);
        
        if(depthSoForwardTransf)
        {
					//Load and display depthmap_req
					sprintf(myFilter_depth, "%06d.*\\.%s", imNum, ext_depthmap); //filter on extension not working

					my_filter_depthmap.set_expression(myFilter_depth);

					for (boost::filesystem::directory_iterator iter(dir_depthmap),end; iter!=end; ++iter)
					{
						name = iter->path().filename().string();
						std::cout << iter->path().string() << std::endl;
						if (boost::regex_match(name, my_filter_depthmap))
						{
							vpImageIo::readPFM(depthmap_se, iter->path().string());
							break;
						}
					}

					//std::cout << depthmap_se.getWidth() << " " << depthmap_se.getHeight() << std::endl;

					if( (depthmap_se.getWidth() != I_se.getWidth()) || (depthmap_se.getHeight() != I_se.getHeight()) )
					{
						vpImage<float> depthmap_se_resize;
						vpImageTools::resize(depthmap_se, depthmap_se_resize, I_se.getWidth(), I_se.getHeight(), vpImageTools::INTERPOLATION_LINEAR);//vpImageTools::INTERPOLATION_AREA);
						depthmap_se = depthmap_se_resize;
					}

					//std::cout << depthmap_se.getWidth() << " " << depthmap_se.getHeight() << std::endl;

					vpImageConvert::convert(depthmap_se, depthmap_se_disp); 	
					
        	if(nbPass == 0)
        	{
						disp_depthmap.init(depthmap_se_disp, 250, 25, "depthmap_se");
					}
					
					vpDisplay::display(depthmap_se_disp);
					vpDisplay::flush(depthmap_se_disp);
        }
        
        if(v_pv_init.size() > imNum)
        {
        		if(depthSoForwardTransf)
            	cMc0.buildFrom(v_pv_init[imNum]);
            else
            {
            	//set translations to zero
            	v_pv_init[imNum][0] = v_pv_init[imNum][1] = v_pv_init[imNum][2] = 0;
            	cMc0.buildFrom(v_pv_init[imNum]);
            }
            	
            if(inversePose)
                cMc0 = cMc0.inverse();
        }
        else
            cMc0.eye();
        
        cMc0 = erMdf.inverse()*cMc0*erMdf;
        
        vpPoseVector p_updt(cMc0);
        ficPosesInit_updt << p_updt.t() << std::endl;
        
        if(!depthSoForwardTransf)
        {
		      //equirectangular to equirectangular (no depth, inverse mapping)
		      pt_bitmap_te = I_te.bitmap;
		      for(unsigned int v_te = 0 ; v_te < imHeight ; v_te++)
		      {
		          for(unsigned int u_te = 0 ; u_te < imWidth ; u_te++, pt_bitmap_te++)
		          {
		              //Equirectangular to sphere
		              P.set_u(u_te);
		              P.set_v(v_te);
		              equirectCam.pixelMeterConversion(P);
		              equirectCam.projectImageSphere(P, Xs, Ys, Zs);
		              //sphere rotation
		              P.set_oX(Xs);
		              P.set_oY(Ys);
		              P.set_oZ(Zs);
		              
		              //rotation "compensation"
		              P.changeFrame(cMc0);
		              
		              //sphere to equirectangular
		              equirectCam.project3DImage(P);
		              equirectCam.meterPixelConversion(P);
		                               
		              u = P.get_u();
		              v = P.get_v();
		              
		              if( (u >= 0) && (v >= 0) && (u < (imWidth-1)) && (v < (imHeight-1))  )
		              {
		                  switch(INTERPTYPE)
		                  {
		                      case IMAGEPLANE_BILINEAR:
		                          curPix.R = curPix.G = curPix.B = 0;
		                          
		                          i = (int)v; dv = v-i; unmdv = 1.0-dv;
		                          j = (int)u; du = u-j; unmdu = 1.0-du;
		                          
		                          
		                              curPix = curPix + I_se[i][j]*unmdv*unmdu;

		                              curPix = curPix + I_se[i+1][j]*dv*unmdu;
		                          
		                              curPix = curPix + I_se[i][j+1]*unmdv*du;
		                          
		                              curPix = curPix + I_se[i+1][j+1]*dv*du;
		                          
		                          
		                          *pt_bitmap_te = curPix;
		                          
		                          break;
		                      case IMAGEPLANE_NEARESTNEIGH:
		                      default:
		                          
		                          i = vpMath::round(v);
		                          j = vpMath::round(u);
		                          
		                          
		                              *pt_bitmap_te = I_se[i][j];
		                          
		                          
		                          break;
		                  }
		              }
		          }
		          //std::cout << P.get_u() << "\t" << P.get_v() << "\t" << P.get_x() << "\t" << P.get_y() << std::endl;
		      }
        }
        else
        {
        	//equirectangular to equirectangular (depth so forward mapping)
		      pt_bitmap_se = I_se.bitmap;
		      pt_depthmap_se = depthmap_se.bitmap;
		      for(unsigned int v_se = 0 ; v_se < imHeight ; v_se++)
		      {
		          for(unsigned int u_se = 0 ; u_se < imWidth ; u_se++, pt_bitmap_se++, pt_depthmap_se++)
		          {
		          	if(*pt_depthmap_se > 0)
		          	{
		              //Equirectangular to sphere
		              P.set_u(u_se);
		              P.set_v(v_se);
		              equirectCam.pixelMeterConversion(P);
		              equirectCam.projectImageSphere(P, Xs, Ys, Zs);
		              //sphere rotation
		              P.set_oX(Xs*(*pt_depthmap_se));
		              P.set_oY(Ys*(*pt_depthmap_se));
		              P.set_oZ(Zs*(*pt_depthmap_se));
		              
		              //rotation "compensation"
		              P.changeFrame(cMc0.inverse());
		              
		              //sphere to equirectangular
		              equirectCam.project3DImage(P);
		              equirectCam.meterPixelConversion(P);
		                               
		              u = P.get_u();
		              v = P.get_v();
		              
		              if( (u >= 0) && (v >= 0) && (u < (imWidth-1)) && (v < (imHeight-1))  )
		              {                       
                      i = vpMath::round(v);
                      j = vpMath::round(u);
                      
                      I_te[i][j] = *pt_bitmap_se;           
		              }
		            }
		          }
		          //std::cout << P.get_u() << "\t" << P.get_v() << "\t" << P.get_x() << "\t" << P.get_y() << std::endl;
		      }
        }
        
        v_temps.push_back(vpTime::measureTimeMs()-temps);
        std::cout << "Pass " << nbPass << " time : " << v_temps[nbPass] << " ms" << std::endl;
        
        vpDisplay::display(I_te);
        vpDisplay::flush(I_te);
        
        clickOut=vpDisplay::getClick(I_te,false);
        

        //save the equirectangular image
        s.str("");
        s.setf(std::ios::right, std::ios::adjustfield);
        s << chemin << "/te_" << std::setfill('0') << std::setw(6) << imNum << "." << ext;
        filename = s.str();
        vpImageIo::write(I_te, filename);
        
        imNum+=iStep;
        nbPass++;
        //angle += 2.5*M_PI/180.;
    }
    
    ficPosesInit_updt.close();
    
    //save times list to file
    s.str("");
    s.setf(std::ios::right, std::ios::adjustfield);
    s << chemin << "/e_time_" << i0 << "_" << i360 << ".txt";
    filename = s.str();
    std::ofstream ficTime(filename.c_str());
    std::vector<double>::iterator it_time = v_temps.begin();
    for(;it_time != v_temps.end() ; it_time++)
    {
        ficTime << *it_time << std::endl;
    }
    ficTime.close();
    
	return 0;
}
