#pragma once

#include <sofa/core/behavior/MultiVec.h>
#include <sofa/simulation/VectorOperations.h>


namespace sofadiff
{
using namespace sofa;

// core::MultiVecCoordId newCoordId(core::objectmodel::BaseContext* ctx, const char * name, const char * group);
// core::MultiVecDerivId newDerivId(core::objectmodel::BaseContext* ctx, const char * name, const char * group);

template<core::VecType vtype>
core::TMultiVecId<vtype, core::V_WRITE> newVecId(core::objectmodel::BaseContext* ctx, const std::string& name, const std::string& group)
{
    simulation::common::VectorOperations vop(core::mechanicalparams::defaultInstance(), ctx);
    const auto vecId = core::TMultiVecId<vtype, core::VecAccess::V_WRITE>();
    core::behavior::TMultiVec<vtype> vec(&vop, vecId);
    vec.realloc(&vop, false, true, core::VecIdProperties{name, group});
    return vec.id();
}

}