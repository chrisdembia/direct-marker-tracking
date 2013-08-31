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

const int DirectMarkerTrackingController::_numSpaceDims = 3;

DirectMarkerTrackingController::DirectMarkerTrackingController()
{
    constructProperties();
}

DirectMarkerTrackingController::DirectMarkerTrackingController(Model * model,
        MarkersReference * markersRef) : _markersRef(markersRef)
{

    // TODO don't pass in model; we only need it temporarily.

    // Standard method calls.
    // ------------------------------------------------------------------------
    constructProperties();

    // TODO may not need this member variable?
    _numMarkers = _markersRef->getNumRefs();
    _numTasks = _numSpaceDims * _numMarkers;

    // Time-invariant quantities required for computing the Jacobian.
    // ------------------------------------------------------------------------
    _mobilizedBodyIndices.resize(_numMarkers);
    _stationPositionsInBodies.resize(_numMarkers);
    for (int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        std::string markerName = _markersRef->getNames()[iMarker];
        const Marker & marker = model->getMarkerSet().get(markerName);
        _markersBodyNames[iMarker] = marker.getBodyName();
        _mobilizedBodyIndices[iMarker] =
            model->getBodySet().get(marker.getBodyName()).getIndex();

        // Position in body.
        _stationPositionsInBodies[iMarker] = marker.getOffset();
    }
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

    // Assemble the  necessary quantities.
    // ========================================================================
    const SimTK::SimbodyMatterSubsystem & smss =
            getModel().getMatterSubsystem();

    // Jacobian, J; and JacobianTranspose, J^T
    // ------------------------------------------------------------------------
    Matrix Jacobian(_numTasks, getModel().getNumSpeeds());
    Matrix JacobianTranspose = ~Jacobian;

    smss.calcStationJacobian(s, _mobilizedBodyIndices,
            _stationPositionsInBodies, Jacobian);

    // Task-space mass matrix, Lambda
    // -----------------------------------------------------------------------
    // A^-1 * J^T
    Matrix jointMassMatrixInverseTimesJacobianTranspose(
                getModel().getNumSpeeds(), _numTasks);
    for (int iTask = 0; iTask < _numTasks; iTask++)
    {
        smss.multiplyByMInv(s, JacobianTranspose.col(iTask),
                jointMassMatrixInverseTimesJacobianTranspose.col(iTask));
    }
    // Lambda^-1 = J * A^-1 * J^T
    SimTK::FactorLU taskMassMatrixInverse(
            Jacobian * jointMassMatrixInverseTimesJacobianTranspose);
    Matrix taskMassMatrix(_numTasks, _numTasks);
    taskMassMatrixInverse.inverse(taskMassMatrix);
    // TODO is there an easier way to get this inverse?

    // TODO various inverses? null space?

    // Actual (model) task vector.
    // ------------------------------------------------------------------------
    Vector task(_numTasks);
    Vec3 currentMarkerPosInGround;
    for (int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        // Update current marker position, expressed in ground.
        getModel().getSimbodyEngine().transformPosition(s,
                getModel().getBodySet().get(_markersBodyNames[iMarker]),
                _stationPositionsInBodies[iMarker],
                currentMarkerPosInGround);

        // Place this Vec3 in its proper place in the task vector.
        int baseIndex = _numSpaceDims * iMarker;
        for (int iDim = 0; iDim < _numSpaceDims; iDim++)
        {
            task[baseIndex + iDim] = currentMarkerPosInGround[iDim];
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
    Vector taskDotDesired(_numTasks);
    Vector taskDotDotDesired(_numTasks);

    // Marker positions expressed in the ground frame, and derivatives.
    Array_<Vec3> desMarkerPos(_numMarkers);
    Array_<Vec3> desMarkerVel(_numMarkers);
    Array_<Vec3> desMarkerAcc(_numMarkers);

    // Fill in the Array_'s with current state data.
    _markersRef->getValues(s, desMarkerPos);
    _markersRef->getSpeedValues(s, desMarkerVel);
    _markersRef->getAccelerationValues(s, desMarkerAcc);

    // Convert Array_'s of Vec3's into Vector's.
    for (int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        int baseIndex = _numSpaceDims * iMarker;

        for (int iDim = 0; iDim < _numSpaceDims; iDim++)
        {
            taskDesired[baseIndex + iDim] = desMarkerPos[iMarker][iDim];
            taskDotDesired[baseIndex + iDim] = desMarkerVel[iMarker][iDim];
            taskDotDotDesired[baseIndex + iDim] = desMarkerAcc[iMarker][iDim];
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
    Vector desiredTaskAcceleration = taskDotDotDesired
                        + get_speed_gain() * (taskDotDesired - taskDot)
                        + get_position_gain() * (taskDesired - task);

    // Compute task-space actuation necessary to achieve above accelerations.
    // ------------------------------------------------------------------------
    // F = Lambda * Fstar + mu + p
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
    Vector jointActuation = JacobianTranspose * taskActuation;

    // TODO I think I do want qdots, etc, because we are controlling coordinates.

    // Set the control signals of the actuators.
    // ========================================================================
    // TODO modify when we are more general about actuators.
    for (int iActuator = 0; iActuator < getActuatorSet().getSize(); iActuator++)
    {
        Vector thisActuatorsControls(1, jointActuation[iActuator]);
        // TODO make sure we're adding to the correct actuator.
        getActuatorSet().get(iActuator).addInControls(
                thisActuatorsControls, controls);
    }
    /* TODO
    for (int iCoord = 0; iCoord < getModel().getNumCoordinates(); iCoord++)
    {
        string coordName = getModel().getCoordinateSet().get(iCoord).getName();
        updActuatorSet().get(coordName).addInControls(
                    jointActuation[iCoord], controls);
    }
    */
}
