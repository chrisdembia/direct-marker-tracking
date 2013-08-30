#include "DirectMarkerTrackingController.h"

using namespace OpenSim;

int DirectMarkerTrackingController::_numSpatialDims = 3;

DirectMarkerTrackingController::DirectMarkerTrackingController()
{
    // TODO should we have this default constructor?
    setNull();
    constructProperties();
}


DirectMarkerTrackingController::DirectMarkerTrackingController(
        const MarkersReference & markersRef) : _markersRef(markersRef)
{
    // Standard method calls.
    // ------------------------------------------------------------------------
    setNull();
    constructProperties();

    // TODO may not need this member variable?
    _numMarkers = _markersRef.getNumRefs();
    _numTasks = _numSpatialDims * _numMarkers;

    // Time-invariant quantities required for computing the Jacobian.
    // ------------------------------------------------------------------------
    _mobilizedBodyIndices.resize(_numMarkers);
    _stationPositionsInBodies.resize(_numMarkers);
    for (int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        // Index.
        string markerName = _markersRef.getNames()[i];
        const Marker & marker = getModel().getMarkerSet().get(markerName);
        string markersBodyName = marker.getBodyName();
        _mobilizedBodyIndices[iMarker] =
            getModel().getBodySet().get(markersBodyName).getIndex();

        // Position in body.
        _stationPositionsInBodies[i] = marker.getOffset();
    }
}

void DirectMarkerTrackingController::computeControls(
        const SimTK::State & s, SimTK::Vector & controls) const
{
    // ts: task-space quantity (_numTasks)
    // js: joint-space quantity (getModel()->getNumSpeeds())

    // Assemble the  necessary quantities.
    // ========================================================================
    const SimbodyMatterSubsystem & smss = getModel()->getMatterSubsystem();

    // Joint-space mass matrix, A
    // ------------------------------------------------------------------------
    // TODO

    // Jacobian, J; and JacobianTranspose, J^T
    // ------------------------------------------------------------------------
    Matrix Jacobian(_numTasks, getModel()->getNumSpeeds());
    Matrix JacobianTranspose = ~Jacobian;

    smss.calcStationJacobian(s, _mobilizedBodyIndices,
            _stationPositionsInBodies, Jacobian);

    // Task-space mass matrix, Lambda
    // -----------------------------------------------------------------------
    // A^-1 * J^T
    Matrix jsMassMatrixInverseTimesJacobianTranspose(
                getModel()->getNumSpeeds(), _numTasks);
    for (int iTask = 0; iTask < _numTasks; iTask++)
    {
        smss.multiplyByMInv(s, JacobianTranspose.col(iTask),
                jsMassMatrixInverseTimesJacobianTranspose.col(iTask));
    }
    // Lambda^-1 = J * A^-1 * J^T
    FactorLU tsMassMatrixInverse(
            Jacobian * jsMassMatrixInverseTimesJacobianTranspose);
    Matrix tsMassMatrix(_numTasks, _numTasks);
    tsMassMatrixInverse.inverse(tsMassMatrix);
    // TODO is there an easier way to get this inverse?

    // Jd * u
    // ------------------------------------------------------------------------
    Vector JacobianDotTimesSpeeds;
    smss.calcBiasForStationJacobian(s, _mobilizedBodyIndices,
            _stationPositionsInBodies, JacobianDotTimesSpeeds);

    // J * ud
    // ------------------------------------------------------------------------
    Vector JacobianTimesSpeedsDot;
    smss.multiplyByStationJacobian(s, _mobilizedBodyIndices,
            _stationPositionsInBodies, JacobianTimesSpeedsDot;

    // TODO various inverses? null space?

    // Actual (model) task vector.
    // ------------------------------------------------------------------------
    // xd = J * u
    Vector taskDot = Jacobian * s.getU();

    // xdd = Jd * u + J * ud
    // TODO calcBiasForStationJacobian
    // TODO does calling getU() incur a cost for multiple calls?
    // Vector taskDotDot = JacobianDotTimesSpeeds + Jacobian * s.getUDot();

    // Desired task vector (from 'experimental' marker positions).
    // -----------------------------------------------------------------------
    Vector taskDesired(_numTasks); // TODO is resize necessary?
    Vector taskDotDesired(_numTasks);

    // TODO expressed in what frame(s)? TODO make these member variables.
    // Expressed in the ground frame.
    SimTK::Array_<SimTK::Vec3> desMarkerPos(_numMarkers);

    // Vector derivative taken in the ground frame.
    SimTK::Array_<SimTK::Vec3> desMarkerVel(_numMarkers);
    // TODO do I need to size these arrays?

    for (unsigned int iMarker = 0; iMarker < _numMarkers; iMarker++)
    {
        // Position.
        _markersRef.getValues(s, desMarkerPos);
        taskDesired[numSpaceDims * iMarker] = desMarkerPos[iMarker][0];
        taskDesired[numSpaceDims * iMarker + 1] = desMarkerPos[iMarker][1];
        taskDesired[numSpaceDims * iMarker + 2] = desMarkerPos[iMarker][2];

        // Velocity.
        _markersRef.getSpeedValues(s, desMarkerVel);
        taskDesired[numSpaceDims * iMarker] = desMarkerVel[iMarker][0];
        taskDesired[numSpaceDims * iMarker + 1] = desMarkerVel[iMarker][1];
        taskDesired[numSpaceDims * iMarker + 2] = desMarkerVel[iMarker][2];
    }


    // Use the quantities above to compute the resulting actuation.
    // ========================================================================

    // Compute the control law.
    // ------------------------------------------------------------------------
    // This is called F* (Fstar) by Khatib.
    // Alternatively, view this as the desired task-space accelerations,
    // altered by 2 feedback terms (the 2nd 2 terms) to minimize error from
    // disturbances, etc.
    // Fstar = xdd - kv * (xd - xd_des) - kp * (x - x_des)
    Vector desiredTaskAcceleration = taskDotDotDesired
                        - get_speed_gain() * (taskDot - taskDotDesired)
                        - get_position_gain() * (task - taskDesired);

    // Compute task-space actuation necessary to achieve above accelerations.
    // ------------------------------------------------------------------------
    // F = Lambda * Fstar + mu + p
    Vector tsActuation = tsMassMatrix * desiredTaskAcceleration
        ;
        // TODO                       + tsCoriolis // centrifugal
        // TODO                       + tsGravity; // Gravity::getBodyForces()

    // Compute joint actuation to achieve task space actuation.
    // ------------------------------------------------------------------------
    // Gamma = J^T * F
    // TODO does not account for optimal force setting.
    Vector jointActuation = (~Jacobian) * tsActuation;


    // Set the control signals of the actuators.
    // ========================================================================
    for (int iActuator = 0; iActuator < getActuatorSet().getSize(); iActuator++)
    {
        // TODO make sure we're adding to the correct actuator.
        updActuatorSet().get(iActuator).addInControls(
                jointActuation[iActuator], controls);
    }
}
