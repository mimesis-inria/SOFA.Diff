#include <SofaDiff/utils.h>

namespace sofadiff
{
// core::MultiVecCoordId newCoordId(core::objectmodel::BaseContext* ctx, const char * name, const char * group)
// {
//     simulation::common::VectorOperations vop(core::mechanicalparams::defaultInstance(), ctx);
//     const auto vecId = core::TMultiVecId<core::VecType::V_COORD, core::VecAccess::V_WRITE>();
//     core::behavior::MultiVecCoord vec(&vop, vecId);
//     vec.realloc(&vop, false, true, core::VecIdProperties{name, group});
//     return vec.id();
// }
//
// core::MultiVecDerivId newDerivId(core::objectmodel::BaseContext* ctx, const char * name, const char * group)
// {
//     simulation::common::VectorOperations vop(core::mechanicalparams::defaultInstance(), ctx);
//     const auto vecId = core::TMultiVecId<core::VecType::V_DERIV, core::VecAccess::V_WRITE>();
//     core::behavior::MultiVecDeriv vec(&vop, vecId);
//     vec.realloc(&vop, false, true, core::VecIdProperties{name, group});
//     return vec.id();
// }

}