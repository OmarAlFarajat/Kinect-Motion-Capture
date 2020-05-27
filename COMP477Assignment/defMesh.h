/*
 * Mesh with skeleton attached
 * You could put attachment here, or create an attachment class to manage weights
 */
#ifndef MESH_H
#define MESH_H

#include "./glm.h"
#include "./skeleton.h"
class DefMesh
{
public:
    std::vector<float> weights;
    Skeleton mySkeleton;
    GLMmodel * pmodel;
	float interval = 0.01;
    
    std::vector<float*> remember;
    
    float * pcopy;
    float * ncopy;
    GLuint mode;
    DefMesh(); 
    void glDraw(int type);
    void loadWeights();
    void updateVertices();
};
#endif
