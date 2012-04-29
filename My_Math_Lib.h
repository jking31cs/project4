#include "vl/vld.h"

class My_Math_Lib {
    public:
        static float test();
        static Vec3d get_c_value(int handleIndex);
        static Matd computeJacobian(int handleIndex);
    static Matd getJacobianPseudoInverse(Matd jacobianMatrix);
};
