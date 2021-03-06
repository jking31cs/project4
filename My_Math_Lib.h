#include "vl/vld.h"

class My_Math_Lib {
    public:
        static float test();
        static Vec3d get_c_value(int handleIndex, int frameIndex);
        static Matd computeJacobian(int handleIndex, int frameIndex);
    static Matd getJacobianPseudoInverse(Matd jacobianMatrix);
};
