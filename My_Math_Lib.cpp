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
    Matd J = Matd(selectedModel->GetDofCount(), 3 * selectedModel->GetHandleCount());
    for (int handleIndex = 0; handleIndex < selectedModel->GetHandleCount(); handleIndex++) {
        Marker* mark = selectedModel->mHandleList[handleIndex];
        TransformNode* node = selectedModel->mLimbs[mark->mNodeIndex];
        while (node != NULL) {
            Mat4d parent = node->mParentTransform;
            Mat4d t = node->mTransforms[0]->GetTransform();
            //Go through each rotational transform and set the J[column]
            int derivIndex = 1;
            Vec4d offset = My_Math_Lib::computeOffset(Vec4d(mark->mOffset, 1), node);
            while (derivIndex < node->GetSize()) {
                Mat4d rotationMatrices = vl_I; //Represents all the rotation matrices (including derivative one) multiplied together
                for (int transformIndex = 1; transformIndex < node->GetSize(); transformIndex++) {
                    if (transformIndex == derivIndex) {
                        rotationMatrices *= node->mTransforms[transformIndex]->GetDeriv(0);
                    } else {
                        rotationMatrices *= node->mTransforms[transformIndex]->GetTransform();
                    }
                }
                
                //cout << "offset: " << offset << endl; 
                Vec4d J_i = parent*t*rotationMatrices*offset;
                Dof* dof = node->mTransforms[derivIndex]->GetDof(0);
                int column = dof->mId;
                
                int startingRowIndex = 3*handleIndex;
                
                J[column][startingRowIndex] = J_i[0];
                J[column][startingRowIndex+1] = J_i[1];
                J[column][startingRowIndex+2] = J_i[2];
                
                derivIndex++;   
            }
            node = node->mParentNode;
        }
    }
    
    cout << "My Jacobian fresh from calculating: " << J << endl; 
    return trans(J);
}

Vec4d My_Math_Lib::computeOffset(Vec4d offset, TransformNode* node) {
    if (node->GetChildrenCount() == 0) {
        return offset;
    }
    //Assuming only one child at the moment.
    return node->mLocalTransform * computeOffset(offset, node->mChildren[0]);
}

Matd My_Math_Lib::getJacobianPseudoInverse(Matd jacobianMatrix) {
    Matd Jt_times_J = trans(jacobianMatrix) * jacobianMatrix;
    cout <<"Info on Jt*t: Rows: " << Jt_times_J.Rows() << " Cols: " << Jt_times_J.Cols() << endl;
    cout << "My Jacobian Transpose times Jacobian: " << Jt_times_J << endl;
    Matd temp = inv(Jt_times_J);
    return temp * trans(jacobianMatrix);
}