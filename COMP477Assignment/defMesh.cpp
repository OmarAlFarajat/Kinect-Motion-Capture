#include "defMesh.h"
#include <fstream>
#include <iostream>

DefMesh::DefMesh()
{
    pmodel = NULL;
    if (!pmodel) {	/* load up the model */

    char meshFile[] = "model/cheb.obj";
    pmodel = glmReadOBJ(meshFile);
    if (!pmodel) {
        return;
    }
        //glmUnitize(pmodel);
        glmFacetNormals(pmodel);
        glmVertexNormals(pmodel, 0);
        glmFacetNormals(pmodel);
        pcopy=new float[3*(pmodel->numvertices+1)];
        ncopy=new float[3*(pmodel->numnormals+1)];
        memcpy(pcopy, pmodel->vertices, sizeof(float)*3*(pmodel->numvertices+1));
        memcpy(ncopy, pmodel->normals, sizeof(float)*3*(pmodel->numnormals+1));
        
        for(int i=0; i<pmodel->numvertices; ++i)
            remember.push_back(new float[16]);
    }
    mySkeleton.loadSkeleton("./skeleton.out");
    loadWeights();
}

void DefMesh::updateVertices()
{
    for(int i=1; i<=pmodel->numvertices; ++i)
    {
        float m[]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        for(int j=1; j<mySkeleton.joints.size(); ++j)
        {
            float t[16];
            scalar(weights[(i-1)*17+j-1], mySkeleton.joints[j].global_t, t);
            add(m, t, m);
        }
        float v[]={pcopy[i*3], pcopy[i*3+1], pcopy[i*3+2]};
        multv(m, v, v);
        
        pmodel->vertices[i*3]=v[0];pmodel->vertices[i*3+1]=v[1];pmodel->vertices[i*3+2]=v[2];
        
        memcpy(remember[i-1], m, sizeof(float)*16);
        
        //pmodel->normals[i*3]=n[0];pmodel->normals[i*3+1]=n[1];pmodel->normals[i*3+2]=n[2];
    }
    
    for(int i=0; i<pmodel->numtriangles; ++i)
    {
        GLMtriangle* triangle = &(pmodel->triangles[i]);
        
        for(int j=0; j<3; ++j)
        {
            int nin=triangle->nindices[j];
            int vin=triangle->vindices[j]-1;
            
            float n[]={ncopy[nin*3], ncopy[nin*3+1], ncopy[nin*3+2]};
            multv(remember[vin],n ,n, 0.0);
            
            pmodel->normals[nin*3]=n[0];pmodel->normals[nin*3+1]=n[1];pmodel->normals[nin*3+2]=n[2];
        }
    }
    
}

void DefMesh::loadWeights()
{
    std::string file("model/weights.out");
    
    std::ifstream reader(file);
    
    float f;
    reader >> f;
    
    while(!reader.eof())
    {
        weights.push_back(f);
        reader >> f;
    }
    reader.close();
    
    for(int i=0; i<pmodel->numvertices; ++i)
    {
        float sum=0;
        for(int j=0; j<17; ++j)
            sum+=weights[i*17+j];
        for(int j=0; j<17; ++j)
            weights[i*17+j]/=sum;
    }
}

void DefMesh::glDraw(int type)
{
    
    switch(type){
    case 0:
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); break;
    case 1:
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); break;
    case 2:
        mySkeleton.glDrawSkeleton(); return;
    
    }
    glColor3f(0.5, 0.5, 0.5);
    mode = GLM_NONE;
    mode = mode | GLM_SMOOTH;
    
    glPushMatrix();
    glScalef(2,2,2);
    glTranslatef(-0.5, -0.5, -0.5);
    glmDraw(pmodel, mode);
    glPopMatrix();
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    
    mySkeleton.glDrawSkeleton();
}