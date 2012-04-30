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

Vec3d My_Math_Lib::get_c_value(int handleIndex, int frameIndex) {
    Marker* mark = UI->mData->mSelectedModel->mHandleList[handleIndex];
    Vec3d p_line = UI->mData->mSelectedModel->mOpenedC3dFile->GetMarkerPos(frameIndex,handleIndex);
    return mark->mGlobalPos - p_line;
}


Matd My_Math_Lib::computeJacobian(int handleIndex, int frameIndex) {
    Model* selectedModel =  UI->mData->mSelectedModel;
    Matd J = Matd(3, selectedModel->GetDofCount(), vl_0);
    //cout << "My Jacobian before calculating" << J << endl;
    Marker* mark = selectedModel->mHandleList[handleIndex];
    TransformNode* node = selectedModel->mLimbs[mark->mNodeIndex];
    //cout << "My Jacobian for " << node->GetName() << endl; 
    while (node != NULL) {
        Mat4d parent = node->mParentTransform;
        //Go through each rotational transform and set the J[column]
        for (int derivIndex = 0; derivIndex < node->GetSize(); derivIndex++) {
            Transform* currentDerivTransform = node->mTransforms[derivIndex];
            if (!currentDerivTransform->IsDof()) {
                continue;
            }
            for (int dofIndex = 0; dofIndex < currentDerivTransform->GetDofCount(); dofIndex++) {
                Mat4d rotateMatrices = Mat4d(vl_I);
                for (int tIndex = 0; tIndex < node->GetSize(); tIndex++) {
                    if (tIndex==derivIndex) {
                        rotateMatrices *= node->mTransforms[tIndex]->GetDeriv(dofIndex);
                    } else {
                        rotateMatrices *= node->mTransforms[tIndex]->GetTransform();
                    }
                }
                
                
                Vec4d offset = Vec4d(mark->mOffset, 1);
                TransformNode* tempNode = selectedModel->mLimbs[mark->mNodeIndex];
                while (tempNode != node) {
                    offset = tempNode->mLocalTransform * offset;
                    tempNode = tempNode->mParentNode;
                }
                Vec4d J_i = parent * rotateMatrices * offset;
                Dof* dof = currentDerivTransform->GetDof(dofIndex);
                int column = dof->mId;
                if (column == 0 || column == 1 || column ==2) {
                    
                }
                J[0][column] = J_i[0];
                J[1][column] = J_i[1];
                J[2][column] = J_i[2];
            }
            
        }
        node = node->mParentNode;
    }
     
    return J;
}

Matd My_Math_Lib::getJacobianPseudoInverse(Matd jacobianMatrix) {
    Matd Jt_times_J = trans(jacobianMatrix) * jacobianMatrix;
   // cout << "My Jacobian Transpose times Jacobian: " << Jt_times_J << endl;
    Matd temp = inv(Jt_times_J);
    return temp * trans(jacobianMatrix);
}