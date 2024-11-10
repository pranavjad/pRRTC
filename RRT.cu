/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

/*
NOTE:
1. Can sample and store SE3 space
2. If using SE3 space, need to modify distance function
3. If using SE3 space, need to modify goal state copy to pass in the quaternion to GPU goalCoord
4. If using SE3 space, need to modify copying path motion states back to CPU
5. If using SE3 space, need to modify start motion initialization 
6. If using SE3 space, need to modify pruning threshold metric and distance calculation
7. Will need to modify pruning distThreshold once I think about this more
8. Horizontal and Vertical Pruning both seems to be working, CUDA error memory access is caused by some threads exiting early when a path is found. Should definitely fix but not urgent.
*/

/*
PENDING:
1. Determine the frequency of pruning (currently set to a low number of sampling rounds for testing)
2. fix interpolation on GPU
3. iterate all threads for pruning to make sure they cover all the nodes, in case when number of nodes > number of threads
4. Determine the best threshold for vertical and horizontal pruning
5. Rewrite sampling and dist calculation so they can handle any dimension

Numerical Limits:
1. total number of nodes <= 1e6
2. each node can have <= 100 children
*/


#include "ompl/geometric/planners/rrt/RRT.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

#include "ompl/base/Cost.h"
#include "ompl/base/spaces/SE3StateSpace.h"
#include "ompl/base/goals/GoalState.h"
#include <cstdio>
#include <cuda_runtime.h>
#include <curand_kernel.h>
#include <cstdlib> 


__global__ void ompl::geometric::initCurandStates(curandState *states) {
    int id = threadIdx.x + blockIdx.x * blockDim.x;
    curand_init(id, 0, 0, &states[id]);
}


__device__ bool ompl::geometric::validState(double x, double y, double z){

    int numberOfObstacles=1;
    double obstacles[100]={0, 0, 0, 149.99}; // structured as [x1, y1, z1, r1, x2, y2, z2, r2 ...]
    double robotRadius=0;

    for (int i=0;i<numberOfObstacles*4;i+=4){
        if ((x-obstacles[i])*(x-obstacles[i])+(y-obstacles[i+1])*(y-obstacles[i+1])+(z-obstacles[i+2])*(z-obstacles[i+2])<=(robotRadius+obstacles[i+3])*(robotRadius+obstacles[i+3])){
            return false;
        }
    }

    return true;

}


__device__ bool ompl::geometric::checkMotion(double x1, double y1, double z1, double x2, double y2, double z2){

    int numberOfObstacles=1;
    double obstacles[100]={0, 0, 0, 149.99}; // structured as [x1, y1, z1, r1, x2, y2, z2, r2...]
    double robotRadius=0;
    

    for (int i=0;i<numberOfObstacles*4;i+=4){

        double r=obstacles[i+3];
        double h=obstacles[i];
        double k=obstacles[i+1];
        float A = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
        float B = 2 * ((x2 - x1) * (x1 - h) + (y2 - y1) * (y1 - k));
        float C = (x1 - h) * (x1 - h) + (y1 - k) * (y1 - k) - r * r;
        float discriminant = B * B - 4 * A * C;

        if (discriminant < 0) {
            ; // No intersection
        } else {
            float t1 = (-B + sqrt(discriminant)) / (2 * A);
            float t2 = (-B - sqrt(discriminant)) / (2 * A);
            if ((t1 >= 0 && t1 <= 1) || (t2 >= 0 && t2 <= 1)) {
                return false; // Intersection
            } else {
                ; // No intersection
            }
        }

    }
    //printf("x %f y %f x2 %f y2%f \n", x1, y1, x2, y2);
    return true;

}






__device__ void ompl::geometric::pruneHMotion(double * nodeCoord, int * parent, int * children, int * nextAvailPos, int dimension, int nodeCoordSize, int motionArrSize){


    // thread is pointing at the grandparent in the grandparent/parent/children subtree

    int tId = threadIdx.x + (blockIdx.x * blockDim.x);
    
    int totalThreads = gridDim.x * gridDim.y * gridDim.z * blockDim.x * blockDim.y * blockDim.z;
    double distHThreshold=2;

    while (true){

        if (tId*dimension>nodeCoordSize*dimension-2*dimension || nodeCoord[tId*dimension]==-1){
            break;
        }

        int motionId=tId;
        for (int i=motionId*100; i<motionId*100+100; i++){

            if (children[i]==0) continue;

            for (int j=i+1; j<motionId*100+100; j++){

                if (children[j]==0) continue;
                // mId1, mId2 are the left and right children
                int mId1=children[i];
                int mId2=children[j];

                double dist = sqrt((nodeCoord[mId1*dimension]-nodeCoord[mId2*dimension])*(nodeCoord[mId1*dimension]-nodeCoord[mId2*dimension]) + (nodeCoord[mId1*dimension+1]-nodeCoord[mId2*dimension+1])*(nodeCoord[mId1*dimension+1]-nodeCoord[mId2*dimension+1])+ (nodeCoord[mId1*dimension+2]-nodeCoord[mId2*dimension+2])*(nodeCoord[mId1*dimension+2]-nodeCoord[mId2*dimension+2]));
                if (dist>distHThreshold){
                    continue;
                }

                // subTreeCoord1, subTreeCoord2 are the left and right grandchildren
                double subTreeCoord1[200];
                double subTreeCoord2[200];
                int subTreeSize1=0;
                int subTreeSize2=0;
                int ind1=0;
                int ind2=0;

                for (int r=mId1*100; r<mId1*100+100; r++){
                    if (children[r]==0) continue;
                    for (int e=0;e<dimension; e++){
                        subTreeCoord1[ind1+e]=nodeCoord[children[r]*dimension+e];
                    }
                    ind1+=dimension;
                    subTreeSize1++;
                    if (ind1>=290){
                        break;
                    }
                }
                for (int r=mId2*100; r<mId2*100+100; r++){
                    if (children[r]==0) continue;
                    for (int e=0;e<dimension;e++){
                        subTreeCoord2[ind2+e]=nodeCoord[children[r]*dimension+e];
                    }
                    ind2+=dimension;
                    subTreeSize2++;
                    if (ind2>=290){
                        break;
                    }
                }

                bool allNodesMoved1=true;
                bool allNodesMoved2=true;
                
                // iterate through the left subtree, see if all children can be moved to the right parent, and vice versa
                for (int w=0; w<subTreeSize1; w++){
                    if (!checkMotion(subTreeCoord1[w*dimension], subTreeCoord1[w*dimension+1], subTreeCoord1[w*dimension+2], nodeCoord[mId2*dimension], nodeCoord[mId2*dimension+1], nodeCoord[mId2*dimension+2])){
                        allNodesMoved1=false;
                        break;
                    }
                }

                for (int w=0; w<subTreeSize2; w++){
                    if (!checkMotion(subTreeCoord2[w*dimension], subTreeCoord2[w*dimension+1], subTreeCoord2[w*dimension+2], nodeCoord[mId1*dimension], nodeCoord[mId1*dimension+1], nodeCoord[mId1*dimension+2])){
                        allNodesMoved2=false;
                        break;
                    }
                }

                
                if (allNodesMoved1){

                    // can move the left subtree to under the right parent
                    for (int r=mId1*100; r<mId1*100+100; r++){
                        if (children[r]==0) continue;
                        parent[children[r]]=mId2;
                    }

                    // setting children of the right parent to include the left subtree
                    for (int t=mId1*100; t<mId1*100+100; t++){

                        int childId=children[t];
                        if (childId==0) continue;
                        children[t]=0;
                        for (int childInsertId=mId2*100; childInsertId<mId2*100+100; childInsertId++){
                            if (children[childInsertId]==0){
                                children[childInsertId] = childId;
                                break;
                            }
                        }

                    }
                    
                    // removing the left parent
                    for (int r=0; r<dimension; r++){
                        nodeCoord[mId1*dimension+r]=-1;
                    }

                    for (int t=parent[mId1]*100; t<parent[mId1]*100+100; t++){
                        if (t<0) continue;
                        if (children[t]==mId1){
                            children[t]=0;
                            break;
                        }
                    }
                    parent[mId1]=-1;
                    
                    // shift the last coordinate to the new space
                    int shiftOldId = atomicAdd(nextAvailPos, -1);
                    shiftOldId-=1;
                    
                    for (int r=0; r<dimension; r++){
                        nodeCoord[mId1*dimension+r]=nodeCoord[shiftOldId*dimension+r];
                        nodeCoord[shiftOldId*dimension+r]=-1;
                    }

                    //printf("I got pruned H\n");

                    
                    for (int t=parent[shiftOldId]*100; t<parent[shiftOldId]*100+100; t++){
                        if (t<0) continue;
                        if (children[t]==shiftOldId){
                            children[t]=mId1;
                            break;
                        }
                    }
                    parent[mId1]=parent[shiftOldId];
                    parent[shiftOldId]=-1;
                    
                    
                }
                else if (allNodesMoved2){

                    // can move the right subtree to under the left parent
                    for (int r=mId2*100; r<mId2*100+100; r++){
                        if (children[r]==0) continue;
                        parent[children[r]]=mId1;
                    }

                    // setting children of the left parent to include the right subtree
                    for (int t=mId2*100; t<mId2*100+100; t++){

                        int childId=children[t];
                        if (childId==0) continue;
                        children[t]=0;
                        for (int childInsertId=mId1*100; childInsertId<mId1*100+100; childInsertId++){
                            if (children[childInsertId]==0){
                                children[childInsertId] = childId;
                                break;
                            }
                        }

                    }
                    // removing the right parent
                    for (int r=0; r<dimension; r++){
                        nodeCoord[mId2*dimension+r]=-1;
                    }

                    for (int t=parent[mId2]*100; t<parent[mId2]*100+100; t++){
                        if (t<0) continue;
                        if (children[t]==mId2){
                            children[t]=0;
                            break;
                        }
                    }
                    parent[mId2]=-1;

                    // shift the last coordinate to the new space
                    int shiftOldId = atomicAdd(nextAvailPos, -1);
                    
                    shiftOldId-=1;
                    
                    for (int r=0; r<dimension; r++){
                        nodeCoord[mId2*dimension+r]=nodeCoord[shiftOldId*dimension+r];
                        nodeCoord[shiftOldId*dimension+r]=-1;
                    }

                    //printf("I got pruned H2\n");

                    for (int t=parent[shiftOldId]*100; t<parent[shiftOldId]*100+100; t++){
                        if (t<0) continue;
                        if (children[t]==shiftOldId){
                            children[t]=mId2;
                            break;
                        }
                    }
                    parent[mId2]=parent[shiftOldId];
                    parent[shiftOldId]=-1;

                }
                

            }

        }
        tId+=totalThreads;
    }

    __syncthreads();
    return ;

}

__device__ void ompl::geometric::pruneVMotion(double * nodeCoord, int * parent, int * children, int * nextAvailPos, int dimension, int nodeCoordSize, int motionArrSize){

    int tId = threadIdx.x + (blockIdx.x * blockDim.x);
    int totalThreads = gridDim.x * gridDim.y * gridDim.z * blockDim.x * blockDim.y * blockDim.z;
    double distVThreshold=2;
    
    

    while (true){

        if (tId>nodeCoordSize-2 || nodeCoord[tId*dimension]==-1){
            break;
        }

        double subTreeCoord[300];
        int subTreeCoordInd=0;
        int subTreeSize=0;

        if (tId%10==0){

            // this is the thread that will perform pruning, it is the parent node of the subtree (among the grandparent, parent, children pair)
            
            // check that the grandparent node exists and cannot get pruned
            if (parent[tId]>-1 && parent[tId]%10!=0){

                //adding the current (parent) motion to the subtree
                int motionId=tId;
                for (int i=0;i<dimension;i++){
                    subTreeCoord[subTreeCoordInd+i] = nodeCoord[motionId*dimension+i];
                }
                subTreeCoordInd+=dimension;
                subTreeSize++;


                // adding the grandparent motion to the subtree
                motionId=parent[tId];
                for (int i=0;i<dimension;i++){
                    subTreeCoord[subTreeCoordInd+i] = nodeCoord[motionId*dimension+i];
                }
                subTreeCoordInd+=dimension;
                subTreeSize++;


                // adding the children motions to the subtree
                motionId=tId;
                for (int t=motionId*100; t<motionId*100+100; t++){
                    motionId = children[t];
                    if (motionId==0){
                        continue;
                    }
                    for (int i=0;i<dimension;i++){
                        subTreeCoord[subTreeCoordInd+i] = nodeCoord[motionId*dimension+i];
                    }
                    subTreeCoordInd+=dimension;
                    subTreeSize++;
                    if (subTreeCoordInd>=290){
                        break;
                    }
                }


                bool nodePruned=false;
                bool allChildrenMoved=true;

                // here, subTreeCoord contains all the motions in this subtree, structured as parent | grabdparent | children
                for (int i=2; i<subTreeSize; i++){
                    
                    // iterate through the children

                    // comute distance of each child motion with grandparent
                    double dist = sqrt((subTreeCoord[i*dimension]-subTreeCoord[0*dimension])*(subTreeCoord[i*dimension]-subTreeCoord[0*dimension])+(subTreeCoord[i*dimension+1]-subTreeCoord[0*dimension+1])*(subTreeCoord[i*dimension+1]-subTreeCoord[0*dimension+1])+(subTreeCoord[i*dimension+2]-subTreeCoord[0*dimension+2])*(subTreeCoord[i*dimension+2]-subTreeCoord[0*dimension+2]));
                    
                    // vertical pruning
                    if (dist<distVThreshold && checkMotion(subTreeCoord[i*dimension], subTreeCoord[i*dimension+1], subTreeCoord[i*dimension+2], subTreeCoord[0*dimension], subTreeCoord[0*dimension+1], subTreeCoord[0*dimension+2])){
                        ;
                    }
                    else{
                        allChildrenMoved=false;
                        break;
                    } 

                }

                
                if (allChildrenMoved){

                    // the children can be moved to under grandparent, and the current node can be pruned
                    nodePruned=true;

                    // motion with motionId can be pruned
                    motionId = tId;

                    // setting parent of the children node to grandparent
                    for (int t=motionId*100; t<motionId*100+100; t++){
                        if (children[t]==0){
                            continue;
                        }
                        parent[children[t]] = parent[motionId];
                    }

                    
                    // setting children of the parent node to children of grandparent
                    for (int t=motionId*100; t<motionId*100+100; t++){

                        if (children[t]==0){
                            continue;
                        }
                        int childId=children[t];
                        children[t]=0;
                        for (int childInsertId=parent[motionId]*100; childInsertId<parent[motionId]*100+100; childInsertId++){
                            if (children[childInsertId]==0){
                                children[childInsertId] = childId;
                                break;
                            }
                        }

                    }

                    // removing the current (parent) node
                    for (int r=0;r<dimension;r++){
                        nodeCoord[motionId*dimension+r]=-1;
                    }
                    

                    for (int t=parent[motionId]*100; t<parent[motionId]*100+100; t++){
                        if (children[t]==motionId){
                            children[t]=0;
                            break;
                        }
                    }
                    parent[motionId]=-1;

                    for (int t=motionId*100; t<motionId*100+100; t++){
                        children[t]=0;
                    }
                    

                    // shift the last coordinate to the new space
                    int shiftOldId = atomicAdd(nextAvailPos, -1);
                    shiftOldId-=1;

                    for (int r=0;r<dimension;r++){
                        nodeCoord[motionId*dimension+r]=nodeCoord[shiftOldId*dimension+r];
                        nodeCoord[shiftOldId*dimension+r]=-1;
                    }

                    //printf("I got pruned V\n");
                    
                    
                    for (int t=parent[shiftOldId]*100; t<parent[shiftOldId]*100+100; t++){

                        //printf("pInd %d\n", t);
                        // why would some t be negative?
                        if (t<0){
                            continue;
                        }
                        
                        if (children[t]==shiftOldId){
                            children[t]=motionId;
                            break;
                        }
                        
                        
                    }
                    
                    parent[motionId]=parent[shiftOldId];
                    parent[shiftOldId]=-1;

                    int childInsertCounter=0;
                    for (int t=shiftOldId*100; t<shiftOldId*100+100; t++){

                        if (children[t]==0){
                            continue;
                        }
                        children[motionId*100+childInsertCounter]=children[t];
                        parent[children[t]] = motionId;
                        children[t]=0;
                        childInsertCounter++;

                    }
                    

                }
                

            }

        }
        tId+=totalThreads;
    }

    __syncthreads();

    return;

}


__global__ void ompl::geometric::parallelRRT(curandState* states, double * startCoord, double * goalCoord, double lowBoundx, double lowBoundy, double lowBoundz, double highBoundx, double highBoundy, double highBoundz, double * nodeCoord, double maxDistance, int * parent, int * children, volatile int * terminate, double * path, int * nextAvailPos, int dimension, double *obstacles, int numberOfObstacles, double robotRadius, volatile double *configs, volatile bool *cc_result){

    double goal_bias=0.05;
    double distThreshold=3;
    int tId = threadIdx.x + (blockIdx.x * blockDim.x);
    bool goal_can_sample;
    double x, y, z;
    int totalThreads = gridDim.x * gridDim.y * gridDim.z * blockDim.x * blockDim.y * blockDim.z;
    int nodeCoordSize=1e5;
    int motionArrSize=3e4;
    int nodeCoordInd=tId;
    int pruneCycle=0;
    int rounds=0;


    // fill nodeCoord with -1
    for (int i=tId; i<nodeCoordSize;i+=totalThreads){
        nodeCoord[i]=-1;
    }

    // fill parent and path with -1
    for (int i=tId; i<motionArrSize;i+=totalThreads){
        parent[i]=-1;
        path[i]=-1;
    }

    __syncthreads();

    // initializing the start states
    if (tId==0){
        for (int i=0;i<dimension;i++){
            nodeCoord[i]=startCoord[i];
        }
        *nextAvailPos=1;
    }

    
    __syncthreads();

    // main RRT loop
    while (true){

        rounds++;

        // sample a new configuration
        if (curand_uniform(&states[tId])<goal_bias){

            // goal sampling
            x=goalCoord[0];
            y=goalCoord[1];
            z=goalCoord[2];
        }
        else {
            // sampling x,y,z for 3-D vector space
            x=(highBoundx-lowBoundx)*curand_uniform_double(&states[tId])+lowBoundx;
            y=(highBoundy-lowBoundy)*curand_uniform_double(&states[tId])+lowBoundy;
            z=(highBoundz-lowBoundz)*curand_uniform_double(&states[tId])+lowBoundz;
        }

        double nearestDist=1e10;
        int nearNodeInd=-1;
        double nearx;
        double neary;
        double nearz;

        if (validState(x, y, z)){
            // find nearest neighbor
            for (int i=0;i<(*nextAvailPos)*dimension; i+=dimension){
                if (nodeCoord[i]==-1){
                    continue;
                }
                double nodex=nodeCoord[i];
                double nodey=nodeCoord[i+1];
                double nodez=nodeCoord[i+2];
                double dist=sqrt((nodex-x)*(nodex-x)+(nodey-y)*(nodey-y)+(nodez-z)*(nodez-z));
                if (dist<nearestDist){
                    nearestDist=dist;
                    nearNodeInd=i/dimension;
                    nearx=nodex;
                    neary=nodey;
                    nearz=nodez;
                }
                if ( (i/dimension)%100==0 && *terminate==1){
                    return;
                }
            }
        }

        // At this point we have an edge to check between (x,y,z) <=> (nearx, neary, nearz)


        // check and add motion
        if (validState(x, y, z) && checkMotion(x, y, z, nearx, neary, nearz) && nearestDist > distThreshold){

            nodeCoordInd = atomicAdd(nextAvailPos, 1);
            nodeCoord[nodeCoordInd*dimension]=x;
            nodeCoord[nodeCoordInd*dimension+1]=y;
            nodeCoord[nodeCoordInd*dimension+2]=z;
            
            parent[nodeCoordInd] = nearNodeInd;

            for (int t=0;t<100;t++){
                if (children[nearNodeInd*100+t]==0){
                    children[nearNodeInd*100+t] = nodeCoordInd;
                    break;
                }
            }
            

            // check if goal has been satisfied and a path found

            if (x==goalCoord[0] && y==goalCoord[1] && z==goalCoord[2] && atomicCAS((int *)terminate, 0, 1)==0){
                

                // the robot has reached the goal, path found

                // this threads performed the "terminate" variable swap
                // this thread now constructs the found path backwards, goal ... start motion
                int pathMotionInd = (nodeCoordInd);
                int pathArrInd=0;

                while (true){
                    
                    for (int i=0;i<dimension;i++){
                        path[pathArrInd+i] = nodeCoord[pathMotionInd*dimension+i];
                    }
                    
                    printf("path x y z\n %f %f %f %d\n", path[pathArrInd], path[pathArrInd+1], path[pathArrInd+2], pathMotionInd);
                    pathMotionInd = parent[pathMotionInd];
                    
                    pathArrInd+=dimension;
                    if (pathMotionInd==-1){
                        break;
                    }
                    
                    
                }

            }
            
        }

        __syncthreads();

        // if goal has been satsified then terminate all threads
        if (*terminate==1){
            return;
        }

        __syncthreads();

        if (rounds%10==0){

            pruneVMotion(nodeCoord, parent, children, nextAvailPos, dimension, nodeCoordSize, motionArrSize);
            __syncthreads();
            //pruneHMotion(nodeCoord, parent, children, nextAvailPos, dimension, nodeCoordSize, motionArrSize);
            __syncthreads();
            

        }

        __syncthreads();

    }

    

    
}


ompl::geometric::RRT::RRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
  : base::Planner(si, addIntermediateStates ? "RRTintermediate" : "RRT")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RRT::setRange, &RRT::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RRT::setGoalBias, &RRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RRT::setIntermediateStates, &RRT::getIntermediateStates,
                                "0,1");

    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::RRT::~RRT()
{
    freeMemory();
    
}

void ompl::geometric::RRT::clear()
{
    
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
    
}

void ompl::geometric::RRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::RRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

ompl::base::PlannerStatus ompl::geometric::RRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);
    auto *goal_state = dynamic_cast<base::GoalState *>(goal);


    if (goal_s == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }


    if (!goal_s->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }


    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        nn_->add(motion);
    }


    /*
    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    */


    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    

    // USER INPUT BEGIN *****************************************************************************************

    double lowBoundx=-150;
    double lowBoundy=-150;
    double lowBoundz=0;
    double highBoundx=150;
    double highBoundy=150;
    double highBoundz=0;
    int threadPerBlock=128;
    int numberOfBlock=20;
    double startx=-145;
    double starty=-145;
    double startz=0;
    double goalx=145;
    double goaly=145;
    double goalz=0;
    int dimension=3;
    int nodeCoordSize=1e5;
    int motionArrSize=3e4;

    // USER INPUT END   *****************************************************************************************

    if (nn_->size() == 0)
    {   
        auto * start_motion = new Motion;
        start_motion->state = si_->getStateSpace()->allocState();
        auto *tempState = static_cast<base::SE3StateSpace::StateType *>(start_motion->state);
        tempState->setX(startx);
        tempState->setY(starty);
        tempState->setZ(startz);
        tempState->rotation().setIdentity();
        nn_->add(start_motion);
    }


    curandState *d_states;
    cudaMalloc(&d_states, 3e4 * sizeof(curandState));
    std::vector<double> goalCoord(dimension, -1);
    std::vector<double> startCoord(dimension, -1);
    double * d_goalCoord;
    cudaMalloc(&d_goalCoord, dimension * sizeof(double));
    double * d_startCoord;
    cudaMalloc(&d_startCoord, dimension * sizeof(double));
    double * d_node_coord;
    cudaMalloc(&d_node_coord, nodeCoordSize * dimension * sizeof(double));
    int * d_parent;
    cudaMalloc(&d_parent, nodeCoordSize * sizeof(int));
    int * d_terminate;
    cudaMalloc(&d_terminate, 1 * sizeof(int));
    std::vector<double> path(1000, -1);
    double * d_path;
    cudaMalloc(&d_path, nodeCoordSize * sizeof(double));
    int * d_children;
    cudaMalloc(&d_children, nodeCoordSize * 100 * sizeof(int));
    int * d_nextAvailPos;
    cudaMalloc(&d_nextAvailPos, 1 * sizeof(int));


    goalCoord[0]=goalx;
    goalCoord[1]=goaly;
    goalCoord[2]=goalz;
    startCoord[0]=startx;
    startCoord[1]=starty;
    startCoord[2]=startz;
    cudaMemcpy(d_goalCoord, goalCoord.data(), sizeof(double) * dimension, cudaMemcpyHostToDevice);
    cudaMemcpy(d_startCoord, startCoord.data(), sizeof(double) * dimension, cudaMemcpyHostToDevice);
    
    // RRT loop in CPU


    
    // initCurandStates<<<numberOfBlock, threadPerBlock>>>(d_states);
    // cudaDeviceSynchronize();
    
    parallelRRT<<<numberOfBlock, threadPerBlock>>>(d_states, d_startCoord, d_goalCoord, lowBoundx, lowBoundy, lowBoundz, highBoundx, highBoundy, highBoundz, d_node_coord, maxDistance_, d_parent, d_children, d_terminate, d_path, d_nextAvailPos, dimension, device_obstacles, numberOfObstacles, robotRadius, device_configs, cc_result);

    cudaDeviceSynchronize();

    bool solved = true;
    bool approximate = false;

    cudaError_t err = cudaGetLastError();  // Retrieve the last error
    if (err != cudaSuccess) {
        // If there was an error, print it
        printf("CUDA error: %s\n", cudaGetErrorString(err));
        solved=false;
    }

    

    // construct found path
    cudaMemcpy(path.data(), d_path, 1000 * sizeof(double), cudaMemcpyDeviceToHost);

    /*
    double errorToGoal=0.0;
    
    std::vector<Motion*> solMotions;

    
    for (int i=0;i<2e5; i+=dimension){

        if (path[i]==-1){
            break;
        }
        auto * motion = new Motion;
        motion->state = si_->getStateSpace()->allocState();
        auto *tempState = static_cast<base::SE3StateSpace::StateType *>(motion->state);
        tempState->setX(path[i]);
        tempState->setY(path[i+1]);
        tempState->setZ(path[i+2]);
        tempState->rotation().setIdentity();
        nn_->add(motion);

        printf("sol: x y z %f %f %f CPU\n", path[i], path[i+1], path[i+2]);

        bool sat = goal->isSatisfied(motion->state, &errorToGoal);
        if (sat)
        {
            approxdif = errorToGoal;
            solution = motion;
            
        }
        solMotions.push_back(motion);

    }
    */

    /*    
    // set parent
    for (int i=0;i<nn_->size()-1;i++){
        solMotions[i]->parent = solMotions[i+1];
    }
    */

    
    
    
    

    //OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    cudaFree(d_states);
    cudaFree(d_node_coord);
    cudaFree(d_parent);
    cudaFree(d_terminate);
    cudaFree(d_goalCoord);
    cudaFree(d_startCoord);
    cudaFree(d_path);
    cudaFree(d_children);
    cudaFree(d_nextAvailPos);

    return {solved, approximate};
}

void ompl::geometric::RRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
