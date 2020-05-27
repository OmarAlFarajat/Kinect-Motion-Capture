#ifndef SKELETON_H
#define SKELETON_H
#include <vector>
#include <string>
#include <fstream>
#include <cstdlib>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifdef _WIN32
#include ".\GL\glut.h"
#else
#include <GL/freeglut.h>
#endif
#endif

#include "simpleMath.h"
#include ".\Eigen\Eigen"
#include <string>
#include <iostream>



struct Joint
{
    int parentID;
    Vec3 position;
    Vec2 screenCoord;
    bool isHovered;
    bool isPicked;
    
    float local_t[16];
    float global_t[16];
    
    Joint()
    {
        parentID = -1;
        isHovered = false;
        isPicked = false;
        
        for(int i=0; i<16; ++i)
            local_t[i]=0.0;
        local_t[0]=local_t[5]=local_t[10]=local_t[15]=1.0;
        
        for(int i=0; i<16; ++i)
            global_t[i]=0.0;
        global_t[0]=global_t[5]=global_t[10]=global_t[15]=1.0;
    }

	void resetJointPosition()
	{
		for (int i = 0; i<16; ++i)
			local_t[i] = 0.0;
		local_t[0] = local_t[5] = local_t[10] = local_t[15] = 1.0;

		for (int i = 0; i<16; ++i)
			global_t[i] = 0.0;
		global_t[0] = global_t[5] = global_t[10] = global_t[15] = 1.0;
	}
};

class Skeleton
{
private:
    /*Update screen coordinates of joints*/
    void updateScreenCoord();
    
public:
    std::vector<Joint> joints;
	//key frames
	std::vector<std::vector<Eigen::Matrix4f>> key_frame;
	std::vector<std::vector<Eigen::Quaternionf>> key_frame_quaternion;
    /*True if the skeleton has a joint selected*/
    bool hasJointSelected;   
	Skeleton() {
		hasJointSelected = false;
	};
    /*
     * Load Skeleton file
     */
    void loadSkeleton(std::string skelFileName);

	void convert_eigen_to_float_matrix(float *local_t);

	void save_key_frame();

	void delete_key_frame();

	void clear_key_frame();

	std::vector<std::vector<Eigen::Quaternionf>> matrix_to_quaternion(std::vector<std::vector<Eigen::Matrix4f>> m_vector);

	std::vector<std::vector<Eigen::Matrix4f>> quaternion_to_matrix(std::vector<std::vector<Eigen::Quaternionf>> q_vector);

	void save_animation(std::string animationFileName);

    /*
     * Load animation file
     */
    void loadAnimation(std::string skelFileName);

    /*
     * Draw skeleton with OpenGL
     */
    void glDrawSkeleton();

    /*
     * Check if any joint is hovered by given mouse coordinate
     */
    void checkHoveringStatus(int x, int y);

    void release();
    
    void updateGlobal();
    
    void addRotation(float* q);
    
    void selectOrReleaseJoint();
};

#endif
