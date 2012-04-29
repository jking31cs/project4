#include "vl/vld.h"
#include "TransformNode.h"

class My_Math_Lib {
public:
    static float test();
    static Vec3d get_c_value();
    static Matd computeJacobian();
    static Matd getJacobianPseudoInverse(Matd jacobianMatrix);
private:
    static Vec4d computeOffset(Vec4d offset, TransformNode* node);
    
};
