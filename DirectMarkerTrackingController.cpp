#include "DirectMarkerTrackingController.h"

using namespace OpenSim;

DirectMarkerTrackingController::DirectMarkerTrackingController()
{
    setNull();
    constructProperties();
}

void DirectMarkerTrackingController::computeControls(
        const SimTK::State & s, SimTK::Vector & controls) const
{
    // TODO assuming only one level tasks for now.

    // Build Jacobian.
    // ========================================================================
    Matrix Jacobian;
    Jacobian.resize(numMarkers * numSpatialDimensions, numDegreesOfFreedom);

    // For each marker, compute a Jacobian.
    // ------------------------------------------------------------------------
    // Each of these Jacobians is numSpatialDimensions x numDegreesOfFreedom.
    for (int iMarker = 0; iMarker < numMarkers; iMarker++)
    {
        // TODO
    }

    // Concatenate individual-marker Jacobians together.
    // ------------------------------------------------------------------------
    // TODO

    // Assemble desired task vector from marker positions.
    // =======================================================================
    Vector taskDesired; // TODO is resize necessary?
    taskDesired.resize(numMarkers * numSpatialDimensions);
    taskDotDesired.resize(numMarkers * numSpatialDimensions);

    // TODO expressed in what frame(s)?
    // Expressed in the ground frame.
    SimTK::Array_<SimTK::Vec3> markerPositions;

    // Vector derivative taken in the ground frame.
    SimTK::Array_<SimTK::Vec3> markerVelocities;
    // TODO do I need to size these arrays?

    for (unsigned int iMarker = 0; iMarker < numMarkers; iMarker++)
    {
        // Position.
        markers.getValues(s, markerPositions);
        taskDesired[numSpaceDims * iMarker] = markerPositions[iMarker][0];
        taskDesired[numSpaceDims * iMarker + 1] = markerPositions[iMarker][1];
        taskDesired[numSpaceDims * iMarker + 2] = markerPositions[iMarker][2];

        // Velocity.
        markers.getSpeedValues(s, markerVelocities);
        taskDesired[numSpaceDims * iMarker] = markerVelocities[iMarker][0];
        taskDesired[numSpaceDims * iMarker + 1] = markerVelocities[iMarker][1];
        taskDesired[numSpaceDims * iMarker + 2] = markerVelocities[iMarker][2];
    }

    // Compute the control law.
    // ========================================================================

    // xd = J * qd
    Vecotr taskDot = Jacobian * s.getQDot();

    // xdd = Jd * qd + J * qdd
    Vector taskDotDot = JacobianDot * s.getQDot() + Jacobian * s.getQDotDot();

    // Here's the control law. `controlValue` is called F* (Fstar) by Khatib.
    // Alternatively, view this as the desired task-space accelerations,
    // altered by 2 feedback terms (the 2nd 2 terms) to minimize error from
    // disturbances, etc.
    // Fstar = xdd - kv * (xd - xd_des) - kp * (x - x_des)
    Vector desiredTaskAcceleration = taskDotDotDesired
                        - _speedGain * (taskDot - taskDotDesired)
                        - _positionGain * (task - taskDesired);

    // Compute task-space actuation necessary to achieve above accelerations.
    // ========================================================================
    // F = Lambda * Fstar + mu + p
    Vector taskSpaceActuation = taskSpaceMassMatrix * desiredTaskAcceleration
                              + taskSpaceCoriolis
                              + taskSpaceGravity;

    // Compute joint actuation to achieve task space actuation.
    // ========================================================================
    // Gamma = J^T * F
    Vector jointActuation = (~Jacobian) * taskSpaceActuation;

    // TODO does not account for optimal force setting.

    // Set the control signals of the actuators.
    // ========================================================================
    for (int iActuator = 0; iActuator < getActuatorSet().getSize(); iActuator++)
    {
        updActuatorSet().get(iActuator).addInControls(
                jointActuation[iActuator], controls);
    }
}

Matrix taskSpaceMassMatrix(const SimTK::State & s) const
{
    Matrix 
}









