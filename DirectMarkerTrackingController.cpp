#include "DirectMarkerTrackingController.h"
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/Model/Marker.h>
#include <OpenSim/Simulation/Model/MarkerSet.h>
#include <OpenSim/Simulation/Model/BodySet.h>
#include <OpenSim/Simulation/Model/Model.h>

using SimTK::Array_;
using SimTK::Matrix;
using SimTK::Vec3;
using SimTK::Vector;

using namespace OpenSim;

// TODO must be 3.
const int DirectMarkerTrackingController::_numSpaceDims = 3;

DirectMarkerTrackingController::DirectMarkerTrackingController()
{
    constructProperties();
}

DirectMarkerTrackingController::DirectMarkerTrackingController(
        MarkersReference * markersRef) : _markersRef(markersRef)
{

    // TODO don't pass in model; we only need it temporarily.

    // Standard method calls.
    // ------------------------------------------------------------------------
    constructProperties();

    // TODO may not need this member variable?
    _numMarkers = _markersRef->getNumRefs();
    _numTasks = _numSpaceDims * _numMarkers;
    _count = 0;// TODO remove
}

void DirectMarkerTrackingController::constructProperties()
{
    constructProperty_position_gain(100);
    constructProperty_speed_gain(20);
}

void DirectMarkerTrackingController::computeControls(
        const SimTK::State & s, SimTK::Vector & controls) const
{
    // task: task-space quantity (_numTasks)
    // joint: joint-space quantity (getModel()->getNumSpeeds())
    // Notation is Khatib's (operational space framework).

    // Assemble the  necessary quantities.
    // ========================================================================
    // TODO can we realize the stage just like this?
    //getModel().getMultibodySystem().realize(s, SimTK::Stage::Position); not necessary here
    const SimTK::SimbodyMatterSubsystem & smss =
            getModel().getMatterSubsystem();

    // Jacobian, J; and JacobianTranspose, J^T
    // ------------------------------------------------------------------------
    Matrix Jacobian(_numTasks, getModel().getNumSpeeds());
    smss.calcStationJacobian(s, _mobilizedBodyIndices,
            _stationPositionsInBodies, Jacobian);

    Matrix JacobianTranspose = Jacobian.transpose();

    // Task-space mass matrix, Lambda
    // -----------------------------------------------------------------------
    // Lambda = (J * A^-1 * J^T)^-1, but in a few steps.

    // A^-1 * J^T
    Matrix jointMassMatrixInverseTimesJacobianTranspose(
                getModel().getNumSpeeds(), _numTasks);
    for (int iTask = 0; iTask < _numTasks; iTask++)
    {
        smss.multiplyByMInv(s, JacobianTranspose.col(iTask),
                jointMassMatrixInverseTimesJacobianTranspose.updCol(iTask));
    }

    // Lambda^-1 = J * A^-1 * J^T
//    std::cout << "DEBUG Jacobian: " << Jacobian << std::endl;
 //   std::cout << "DEBUG jointMassMatrixInverseTimesJacobianTranspose: " << jointMassMatrixInverseTimesJacobianTranspose << std::endl;
    Matrix taskMassMatrixInverse =
            Jacobian * jointMassMatrixInverseTimesJacobianTranspose;
//    std::cout << "DEBUG task mass matrix inverse: " << taskMassMatrixInverse << std::endl;

    // Lambda = (Lambda^-1)^-1
    SimTK::FactorLU taskMassMatrixInverseLU(taskMassMatrixInverse);
    Matrix taskMassMatrix(_numTasks, _numTasks);
    taskMassMatrixInverseLU.inverse(taskMassMatrix);
    // TODO is there an easier way to get this inverse?

    // TODO various inverses? null space?

    // Actual (model) task vector.
    // ------------------------------------------------------------------------
    getModel().getMultibodySystem().realize(s, SimTK::Stage::Position);
    Vector task(_numTasks);
    for (int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        // Update current marker position, expressed in ground.
        /* TODO efficiency?
        getModel().getSimbodyEngine().transformPosition(s,
                getModel().getBodySet().get(_markersBodyNames[iMarker]),
                _stationPositionsInBodies[iMarker],
                currentMarkerPosInGround);
        */
        Vec3 thisMarkerPosInGround = smss.getMobilizedBody(
                    _mobilizedBodyIndices[iMarker]).findStationLocationInGround(
                    s, _stationPositionsInBodies[iMarker]);

        // Place this Vec3 in its proper place in the task vector.
        int baseIndex = _numSpaceDims * iMarker;
        for (int iDim = 0; iDim < _numSpaceDims; iDim++)
        {
            task[baseIndex + iDim] = thisMarkerPosInGround[iDim];
        }
    }

    // Derivative of actual (model) task.
    // ------------------------------------------------------------------------
    // xd = J * u
    Vector taskDot = Jacobian * s.getU();

    // Desired task vector (from 'experimental' marker positions).
    // -----------------------------------------------------------------------
    // We'll fill these with data from the _markersRef.
    Vector taskDesired(_numTasks); // TODO is resize necessary?
    // TODO Vector taskDotDesired(_numTasks);
    // TODO Vector taskDotDotDesired(_numTasks);

    // Marker positions expressed in the ground frame, and derivatives.
    Array_<Vec3> desMarkerPos(_numMarkers);
    // TODO Array_<Vec3> desMarkerVel(_numMarkers);
    // TODO Array_<Vec3> desMarkerAcc(_numMarkers);

    // Fill in the Array_'s with current state data.
    _markersRef->getValues(s, desMarkerPos);
    // TODO not implemented_markersRef->getSpeedValues(s, desMarkerVel);
    // TODO not implemented_markersRef->getAccelerationValues(s, desMarkerAcc);

    // Convert Array_'s of Vec3's into Vector's.
    for (int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        int baseIndex = _numSpaceDims * iMarker;

        for (int iDim = 0; iDim < _numSpaceDims; iDim++)
        {
            taskDesired[baseIndex + iDim] = desMarkerPos[iMarker][iDim];
            // TODO taskDotDesired[baseIndex + iDim] = desMarkerVel[iMarker][iDim];
            // TODO taskDotDotDesired[baseIndex + iDim] = desMarkerAcc[iMarker][iDim];
        }
    }


    // Use the quantities above to compute the resulting actuation.
    // ========================================================================

    // Compute the control law.
    // ------------------------------------------------------------------------
    // This is called F* (Fstar) by Khatib.
    // Alternatively, view this as the desired task-space accelerations,
    // altered by 2 feedback terms (the 2nd 2 terms) to minimize error from
    // disturbances, etc.
    // Fstar = xdd_des + kv * (xd_des - xd) + kp * (x_des - x)
    Vector desiredTaskAcceleration = //taskDotDotDesired
                // TODO        + get_speed_gain() * (taskDotDesired - taskDot) when we have taskDotDesired.
                        + get_speed_gain() * (- taskDot)
                        + get_position_gain() * (taskDesired - task);

    // Compute task-space actuation necessary to achieve above accelerations.
    // ------------------------------------------------------------------------
    // F = Lambda * Fstar + mu + p
  //  std::cout << "DEBUG desiredTaskAccel: " << desiredTaskAcceleration << std::endl;
  //  std::cout << "DEBUG taskMassMatrix: " << taskMassMatrix << std::endl;
    Vector taskActuation = taskMassMatrix * desiredTaskAcceleration;
        // TODO                       + taskCoriolis // centrifugal
        // TODO                       + taskGravity; // Gravity::getBodyForces()
        // gravity using computeresiduals after setting qdd an qd to zero.
        // then coriolis + gravity by setting qdd to zero only, get residuals,
        // subtract off gravity.

    // Compute joint actuation to achieve task space actuation.
    // ------------------------------------------------------------------------
    // Gamma = J^T * F
    // TODO does not account for optimal force setting.
    // TODO use multiplyByStationJacobianTranspose. requires taskActuation
    // to be a Vector<Vec3>
   // std::cout << "DEBUG taskActuation: " << taskActuation << std::endl;
    Vector jointActuation = JacobianTranspose * taskActuation;

    // TODO I think I do want qdots, etc, because we are controlling coordinates.

    // Set the control signals of the actuators.
    // ========================================================================
    // TODO modify when we are more general about actuators.
    for (int iActuator = 0; iActuator < getActuatorSet().getSize(); iActuator++)
    {
        Vector thisActuatorsControls(1, jointActuation[iActuator]);
        const Actuator & thisActuator = getActuatorSet().get(iActuator);
        // TODO make sure we're adding to the correct actuator.
        thisActuator.addInControls(thisActuatorsControls, controls);
    }
    /* TODO alternative to the above loop.
    for (int iCoord = 0; iCoord < getModel().getNumCoordinates(); iCoord++)
    {
        string coordName = getModel().getCoordinateSet().get(iCoord).getName();
        updActuatorSet().get(coordName).addInControls(
                    jointActuation[iCoord], controls);
    }
    */
    _count++;
    std::cout << "DEBUG COUNT: " << _count << std::endl;
    std::cout << "DirectMarkerTrackingController.computeControls:  t = " << s.getTime() << std::endl;
}

void DirectMarkerTrackingController::connectToModel(Model & model)
{
    // Time-invariant quantities required for computing the Jacobian.
    // ========================================================================
    _mobilizedBodyIndices.resize(_numMarkers);
    _stationPositionsInBodies.resize(_numMarkers);
    for (int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        // Get these items two for convenience:
        std::string markerName = _markersRef->getNames()[iMarker];
        const Marker & marker = model.getMarkerSet().get(markerName);

        _mobilizedBodyIndices[iMarker] =
            model.getBodySet().get(marker.getBodyName()).getIndex();

        _stationPositionsInBodies[iMarker] = marker.getOffset();
    }


    // Add CoordinateActuator for each (enabled? TODO) coordinate.
    // ========================================================================
    /*
    // TODO use CoordinateActuator static method.
    const CoordinateSet & cset = model.getCoordinateSet();
    for (int iCoord = 0; iCoord < cset.getSize(); iCoord++)
    {
        // TODO if coordinate is not locked.
        std::string coordinateName = cset.get(iCoord).getName();
        std::string actName = coordinateName;
        CoordinateActuator * act = new CoordinateActuator(coordinateName);
        // TODO other code may depend on this being the name of the actuator.
        // TODO exception if actuator with this name already exists.
        act->setName(actName);

        // TODO cannot use this? b/c it calls connect to model?
        model.addForce(act);
        // model.updForceSet().append(actuator).

        updProperty_actuator_list().appendValue(actName);
    }*/
    /* TODO look at usage in InverseDynamics
    // TODO overrides replaceForceSet behavior; deletes all forces.
    CoordinateActuator::CreateForceSetOfCoordinateActuatorsForModel(
                initState, *_model, 1, false);
    // CreateForceSet... invalidates the system.
    // TODO is there an easier way to validate the system?
    initState = _model->initSystem();
    */


    Super::connectToModel(model);

    setNumControls(getActuatorSet().getSize());
}
