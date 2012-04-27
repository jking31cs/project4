#include <iostream>
#include "My_Math_Lib.h"
#include "Marker.h"
#include "RealTimeIKui.h"
#include "PhowardData.h"
#include "Model.h"
#include "C3dFileInfo.h"
#include "Transform.h"
#include "TransformNode.h"

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

Matd My_Math_Lib::computeJacobian() {
    Matd J = Matd(UI->mData->mSelectedModel->GetDofCount(),3);
    for (int i = 0; i < UI->mData->mSelectedModel->GetDofCount() - 1; i++) {
        Marker* mark = UI->mData->mSelectedModel->mHandleList[0];
        TransformNode* node = UI->mData->mSelectedModel->mLimbs[mark->mNodeIndex];
        Mat4d parent = node->mParentTransform;
        Mat4d t = node->mTransforms[0]->GetTransform();
        Mat4d dr_dq = node->mTransforms[1]->GetDeriv(0);
        Mat4d r = node->mTransforms[2]->GetTransform();
        Vec4d offset = Vec4d(mark->mOffset, 1);
        Vec4d J_i = parent * t * dr_dq * r * offset;
        int column = node->mTransforms[1]->GetDof(0)->mId;
        cout << "What is the size of J[i]? " << J[i] << endl;
        J[column] = Vec3d(J_i[0], J_i[1], J_i[2]);
        cout << "Got column: " << i << endl;
    }
    return J;
    
}