#include <iostream>
#include "My_Math_Lib.h"
#include "Marker.h"
#include "RealTimeIKui.h"
#include "PhowardData.h"
#include "Model.h"
#include "C3dFileInfo.h"
#include "Transform.h"
#include "TransformNode.h"
#include "Dof.h"

using namespace std;

extern RealTimeIKUI *UI;

float My_Math_Lib::test() {
    cout << "Got a custom class working kinda sorta" << endl;
    return 1.f;
}

Vec3d My_Math_Lib::get_c_value(int handleIndex) {
    Marker* mark = UI->mData->mSelectedModel->mHandleList[handleIndex];
    Vec3d p_line = UI->mData->mSelectedModel->mOpenedC3dFile->GetMarkerPos(0,handleIndex);
    return mark->mGlobalPos - p_line;
}


Matd My_Math_Lib::computeJacobian(int handleIndex) {
    Model* selectedModel =  UI->mData->mSelectedModel;
    Matd J = Matd(3, selectedModel->GetDofCount());
    Marker* mark = selectedModel->mHandleList[handleIndex];
    TransformNode* node = selectedModel->mLimbs[mark->mNodeIndex];
    while (node != NULL) {
        Mat4d parent = node->mParentTransform;
        //Go through each rotational transform and set the J[column]
        int derivIndex = 0;
        while (derivIndex < node->GetSize()) {
            Mat4d localMatrices = vl_I; //Represents all the rotation matrices (including derivative one) multiplied together
            for (int transformIndex = 0; transformIndex < node->GetSize(); transformIndex++) {
                if (transformIndex == derivIndex) {
                    if (node->mTransforms[transformIndex]->IsDof()) {
                        localMatrices *= node->mTransforms[transformIndex]->GetDeriv(transformIndex);
                    } else {
                        localMatrices *= node->mTransforms[transformIndex]->GetTransform();
                        derivIndex += 1;
                    }
                } else {
                    localMatrices *= node->mTransforms[transformIndex]->GetTransform();
                }
            }
            Vec4d offset = Vec4d(mark->mOffset, 1);
            TransformNode* offsetNode = selectedModel->mLimbs[mark->mNodeIndex];
            while (offsetNode != node) {
                offset = offsetNode->mLocalTransform * offset;
                offsetNode = offsetNode->mParentNode; 
            }
            //cout << "offset: " << offset << endl; 
            Vec4d J_i = parent*localMatrices*offset;
            Dof* dof = node->mTransforms[derivIndex]->GetDof(0);
            int column = dof->mId;
            
            cout << "new column: " << column << endl;
            
            J[0][column] = J_i[0];
            J[1][column] = J_i[1];
            J[2][column] = J_i[2];
            
            derivIndex++;   
        }
        node = node->mParentNode;
    }
    
    cout << "My Jacobian fresh from calculating: " << J << endl; 
    return J;
}

Matd My_Math_Lib::getJacobianPseudoInverse(Matd jacobianMatrix) {
    Matd Jt_times_J = trans(jacobianMatrix) * jacobianMatrix;
    cout << "My Jacobian Transpose times Jacobian: " << Jt_times_J << endl;
    Matd temp = inv(Jt_times_J);
    return temp * trans(jacobianMatrix);
}