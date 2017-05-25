#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/operators.h>
#include <pybind11/pytypes.h>
#include <pybind11/stl.h>
#include <PxPhysicsAPI.h>
#include <vector>
#include "simulator/gownSimulator.h"
#include "simulator/shortsSimulator.h"
#include "simulator/baseSimulator.h"
#include "simulator/RigPart.h"
#include "simulator/Cloth.h"
#include "simulator/Mesh.h"
#include "renderer/Renderer.cpp"
#include "CMAES/HapticObjective.h"

namespace py = pybind11;

PYBIND11_PLUGIN(pysim) {
    py::module m("pysim", "Python bindings for c++ simulation code using pybind11");

    using namespace pybind11::literals;

    py::class_<baseSimulator>(m, "baseSimulator");

    py::class_<gownSimulator>(m, "gownSimulator", py::base<baseSimulator>())
        .def(py::init())
        .def("initialize", &gownSimulator::initialize, "maxSteps"_a=1200, "enableWind"_a=false)
        .def("reset", &gownSimulator::reset)
        .def("destroy", &gownSimulator::destroy)
        .def("positionArm", &gownSimulator::positionArm, "horizontalOffset"_a, "verticalOffset"_a, "sidewaysOffset"_a, "rotateFist"_a=(vector<double>){0, 0, 0, 0}, "rotateArm"_a=(vector<double>){0, 0, 0, 0}, "rotateArmCenter"_a=(vector<double>){0, 0, 0})
        .def("initSpline", &gownSimulator::initSpline, "splinePoints"_a)
        .def("setVelocityFactor", &gownSimulator::setVelocityFactor, "velocityFactor"_a=1.0)
        .def("rotateSphereAround", &gownSimulator::rotateSphereAround, "index"_a, "angle"_a, "axis"_a, "center"_a)
        .def("getForcemap", &gownSimulator::getForcemap)
        .def("getFullForcemap", &gownSimulator::getFullForcemap)
        .def("getArmSpheres", &gownSimulator::getArmSpheres)
        .def("getGripperPos", &gownSimulator::getGripperPos)
        .def("simulate", &gownSimulator::simulate, "steps"_a)
        .def("setParameter", &gownSimulator::setParameter, "param"_a)
        .def("getParameterSize", &gownSimulator::getParameterSize)
        .def("getParameterBound", &gownSimulator::getParameterBound, "min"_a, "max"_a)
        .def("updateForces", &gownSimulator::updateForces, "steps"_a)
        .def("identifyGrippedParticles", &gownSimulator::identifyGrippedParticles)
        .def_readwrite("cloth", &gownSimulator::cloth)
        .def_readwrite("cloth_solve_iteration_aftergrip", &gownSimulator::cloth_solve_iteration_aftergrip)
        .def_readwrite("random_wind", &gownSimulator::random_wind)
        .def_readwrite("rig_parts", &gownSimulator::rig_parts)
        .def_readwrite("arm_horizontal_perturb", &gownSimulator::arm_horizontal_perturb)
        .def_readwrite("fist_radius", &gownSimulator::fist_radius)
        .def_readwrite("wrist_radius", &gownSimulator::wrist_radius)
        .def_readwrite("elbow_radius", &gownSimulator::elbow_radius)
        .def_readwrite("shoulder_radius", &gownSimulator::shoulder_radius)
        .def_readwrite("forearm_length", &gownSimulator::forearm_length)
        .def_readwrite("gTimeStep", &gownSimulator::gTimeStep)
        .def_readwrite("move_step", &gownSimulator::move_step)
        .def_readwrite("simulated_step", &gownSimulator::simulated_step)
        .def_readwrite("moving_speed", &gownSimulator::moving_speed)
        .def_readwrite("recorded_positiosn", &gownSimulator::recorded_positiosn)
        .def_readwrite("recorded_time", &gownSimulator::recorded_time)
        .def_readwrite("testbit", &gownSimulator::testbit);

    py::class_<shortsSimulator>(m, "shortsSimulator", py::base<baseSimulator>())
        .def(py::init())
        .def("initialize", &shortsSimulator::initialize, "maxSteps"_a=1200, "enableWind"_a=false)
        .def("reset", &shortsSimulator::reset)
        .def("destroy", &shortsSimulator::destroy)
        .def("positionArm", &shortsSimulator::positionArm, "horizontalOffset"_a, "verticalOffset"_a, "sidewaysOffset"_a, "rotateFist"_a=(vector<double>){0, 0, 0, 0}, "rotateArm"_a=(vector<double>){0, 0, 0, 0}, "rotateArmCenter"_a=(vector<double>){0, 0, 0})
        .def("initSpline", &shortsSimulator::initSpline, "splinePoints"_a)
        .def("setVelocityFactor", &shortsSimulator::setVelocityFactor, "velocityFactor"_a=1.0)
        .def("rotateSphereAround", &shortsSimulator::rotateSphereAround, "index"_a, "angle"_a, "axis"_a, "center"_a)
        .def("getForcemap", &shortsSimulator::getForcemap)
        .def("getFullForcemap", &shortsSimulator::getFullForcemap)
        .def("getArmSpheres", &shortsSimulator::getArmSpheres)
        .def("getGripperPos", &shortsSimulator::getGripperPos)
        .def("simulate", &shortsSimulator::simulate, "steps"_a)
        .def("setParameter", &shortsSimulator::setParameter, "param"_a)
        .def("getParameterSize", &shortsSimulator::getParameterSize)
        .def("getParameterBound", &shortsSimulator::getParameterBound, "min"_a, "max"_a)
        .def("updateForces", &shortsSimulator::updateForces, "steps"_a)
        .def("identifyGrippedParticles", &shortsSimulator::identifyGrippedParticles)
        .def_readwrite("cloth", &shortsSimulator::cloth)
        .def_readwrite("cloth_solve_iteration_aftergrip", &shortsSimulator::cloth_solve_iteration_aftergrip)
        .def_readwrite("random_wind", &shortsSimulator::random_wind)
        .def_readwrite("rig_parts", &shortsSimulator::rig_parts)
        .def_readwrite("arm_horizontal_perturb", &shortsSimulator::arm_horizontal_perturb)
        .def_readwrite("fist_radius", &shortsSimulator::fist_radius)
        .def_readwrite("wrist_radius", &shortsSimulator::wrist_radius)
        .def_readwrite("elbow_radius", &shortsSimulator::elbow_radius)
        .def_readwrite("shoulder_radius", &shortsSimulator::shoulder_radius)
        .def_readwrite("forearm_length", &shortsSimulator::forearm_length)
        .def_readwrite("gTimeStep", &shortsSimulator::gTimeStep)
        .def_readwrite("move_step", &shortsSimulator::move_step)
        .def_readwrite("simulated_step", &shortsSimulator::simulated_step)
        .def_readwrite("moving_speed", &shortsSimulator::moving_speed)
        .def_readwrite("recorded_positiosn", &shortsSimulator::recorded_positiosn)
        .def_readwrite("recorded_time", &shortsSimulator::recorded_time)
        .def_readwrite("testbit", &shortsSimulator::testbit);

    py::class_<RigPart>(m, "RigPart")
        .def(py::init())
        .def("updateContactForce", &RigPart::updateContactForce, "orig"_a, "force"_a)
        .def("clearForceTorque", &RigPart::clearForceTorque)
        .def("addToScene", &RigPart::addToScene, "scene"_a)
        .def("translate", &RigPart::translate, "ind"_a, "vec"_a, "mod_ft"_a=false)
        .def("rotate", &RigPart::rotate, "ind"_a, "angle"_a, "axis"_a, "mod_ft"_a=false)
        .def("rotateAround", &RigPart::rotateAround, "ind"_a, "angle"_a, "axis"_a, "center"_a, "mod_ft"_a=false)
        .def("translateTo", &RigPart::translateTo, "ind"_a, "vec"_a, "mod_ft"_a=false)
        .def("rotateTo", &RigPart::rotateTo, "ind"_a, "angle"_a, "axis"_a, "mod_ft"_a=false)
        .def("recordInitTransform", &RigPart::recordInitTransform)
        .def("reset", &RigPart::reset)
        .def("recordForceTorque", &RigPart::recordForceTorque)
        .def_readwrite("recorded_forces", &RigPart::recorded_forces)
        .def_readwrite("recorded_torques", &RigPart::recorded_torques);

    py::class_<Renderer>(m, "Renderer")
        .def(py::init())
        .def("renderMesh", &Renderer::renderMesh, "mesh"_a, "color"_a, "wire"_a=true, "seqinv"_a=false)
        .def("renderRigPart", &Renderer::renderRigPart, "part"_a, "color"_a)
        .def("renderShape", &Renderer::renderShape, "desc"_a, "color"_a)
        .def("renderForceMap", &Renderer::renderForceMap, "alpha"_a=0.25f)
        .def("renderSphereManager", &Renderer::renderSphereManager, "color"_a, "drawSpheres"_a=true)
        .def("renderRigMotion", &Renderer::renderRigMotion, "part"_a, "color"_a);

    py::class_<physx::PxTransform>(m, "PxTransform")
        .def(py::init());

    py::class_<physx::PxMeshScale>(m, "PxMeshScale")
        .def(py::init());

    py::class_<Cloth>(m, "Cloth")
        .def(py::init())
        .def("createCloth", &Cloth::createCloth, "physx"_a, "transform"_a=physx::PxTransform(), "scale"_a=physx::PxMeshScale())
        .def("loadMesh", &Cloth::loadMesh, "filename"_a, "translation"_a, "rotation"_a, "scale"_a)
        .def("updateMesh", &Cloth::updateMesh)
        .def("resetCloth", &Cloth::resetCloth)
        .def("setParameters", &Cloth::setParameters)
        .def("setParameter", &Cloth::setParameter, "parameters"_a)
        .def("getParameterBound", &Cloth::getParameterBound, "min"_a, "max"_a)
        .def_readwrite("cloth_mesh", &Cloth::cloth_mesh)
        .def_readwrite("hstretch_stiff", &Cloth::hstretch_stiff)
        .def_readwrite("vstretch_stiff", &Cloth::vstretch_stiff)
        .def_readwrite("shear_stiff", &Cloth::shear_stiff)
        .def_readwrite("bend_stiff", &Cloth::bend_stiff)
        .def_readwrite("friction", &Cloth::friction)
        .def_readwrite("self_collision_distance", &Cloth::self_collision_distance)
        .def_readwrite("self_friction", &Cloth::self_friction)
        .def_readwrite("stiffpower", &Cloth::stiffpower);

    py::class_<HapticObjective>(m, "HapticObjective")
        .def(py::init())
        .def("evalObjective", &HapticObjective::evalObjective, "sim"_a)
        .def("ReadData", &HapticObjective::ReadData, "filename"_a)
        .def("Clear", &HapticObjective::Clear)
        .def("sanityCheck", &HapticObjective::sanityCheck)
        .def_readwrite("exp_time", &HapticObjective::exp_time)
        .def_readwrite("exp_position", &HapticObjective::exp_position)
        .def_readwrite("exp_fmove", &HapticObjective::exp_fmove)
        .def_readwrite("exp_flateral", &HapticObjective::exp_flateral)
        .def_readwrite("exp_fgravity", &HapticObjective::exp_fgravity)
        .def("getInterpolationFrac", [](HapticObjective &hap, double val) { int ind1, ind2; double frac1, frac2; hap.getInterpolationFrac(hap.exp_time, val, ind1, frac1, ind2, frac2); vector<double> ret {ind1, frac1, ind2, frac2}; return ret; });

    py::class_<Mesh>(m, "Mesh")
        .def(py::init());

    py::class_<physx::PxVec3>(m, "PxVec3")
        .def(py::init())
        .def_readwrite("x", &physx::PxVec3::x)
        .def_readwrite("y", &physx::PxVec3::y)
        .def_readwrite("z", &physx::PxVec3::z);

    py::class_<physx::PxVec4>(m, "PxVec4")
        .def(py::init())
        .def_readwrite("x", &physx::PxVec4::x)
        .def_readwrite("y", &physx::PxVec4::y)
        .def_readwrite("z", &physx::PxVec4::z)
        .def_readwrite("w", &physx::PxVec4::w);

    py::class_<physx::PxReal>(m, "PxReal")
        .def(py::init());

    return m.ptr();
}

