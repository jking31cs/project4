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

Vec3d My_Math_Lib::get_c_value() {
    Marker* mark = UI->mData->mSelectedModel->mHandleList[0];
    Vec3d p_line = UI->mData->mSelectedModel->mOpenedC3dFile->GetMarkerPos(0,0);
    return mark->mGlobalPos - p_line;
}

//TODO Fix this so that it is correct.
Matd My_Math_Lib::computeJacobian() {
    Model* selectedModel =  UI->mData->mSelectedModel;
    Matd J = Matd(selectedModel->GetDofCount(),3);
    Marker* mark = selectedModel->mHandleList[0];
    for (int nodeIndex = selectedModel->GetNodeCount() -1; nodeIndex >= 0; nodeIndex--) {
        TransformNode* node = selectedModel->mLimbs[nodeIndex];
        Mat4d parent = node->mParentTransform;
        Mat4d t = node->mTransforms[0]->GetTransform();
        //Go through each rotational transform and set the J[column]
        int derivIndex = node->GetSize() - 1;
        while (derivIndex > 0) {
            Mat4d rotationMatrices = vl_I; //Represents all the rotation matrices (including derivative one) multiplied together
            for (int transformIndex = 0; transformIndex < node->GetSize(); transformIndex++) {
                if (transformIndex == derivIndex-1) {
                    rotationMatrices *= node->mTransforms[transformIndex]->GetDeriv(transformIndex);
                } else {
                    rotationMatrices *= node->mTransforms[transformIndex]->GetTransform();
                }
            }
            Vec4d offset = Vec4d(mark->mOffset, 1);
            for (int otherNodeIndex = selectedModel->GetNodeCount() -1; otherNodeIndex > nodeIndex; otherNodeIndex--) {
                offset = selectedModel->mLimbs[otherNodeIndex]->mCurrentTransform * offset;
            }
            Vec4d J_i = parent*t*rotationMatrices*offset;
            Dof* dof = node->mTransforms[derivIndex]->GetDof(derivIndex-1);
            int column = dof->mId;
            J[column] = Vec3d(J_i[0], J_i[1], J_i[2]);
            
            derivIndex--;   
        }
    }
    return trans(J);
}

Matd My_Math_Lib::getJacobianPseudoInverse(Matd jacobianMatrix) {
    Matd Jt_times_J = trans(jacobianMatrix) * jacobianMatrix;
    cout << "My supposedly square matrix: " << Jt_times_J.Rows() << " rows and " << Jt_times_J.Cols() << " Columns" << endl;
    cout << "My Jacobian is " << jacobianMatrix << endl;
    Matd temp = inv(Jt_times_J);
    return temp * trans(jacobianMatrix);
}