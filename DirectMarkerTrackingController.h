#ifndef OPENSIM_DIRECTMARKERTRACKINGCONTROLLER_H_
#define OPENSIM_DIRECTMARKERTRACKINGCONTROLLER_H_

#include "osimPlugin.h"
#include <Simbody.h>
#include <OpenSim/Simulation/Control/Controller.h>

namespace OpenSim {

class MarkersReference;

    // TODO assuming only one level of tasks for now.
// TODO assumes trc markers and markerset are aligned properly (same ordering,
// same # of markers, etc.) same issue that InverseKinematicsTool must have.

/**
 * A controller that finds the torques necessary for a model to track
 * experimental marker trajectories. Based off of Emel Demircan's direct marker
 * control, and Gerald Brantner's operational-space controller for BIOE 485.
 * */
    // TODO control which actuators are given to this controller.
class OSIMPLUGIN_API DirectMarkerTrackingController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(DirectMarkerTrackingController, Controller)
public:

    /// @name Property declarations
    /**@{**/
    OpenSim_DECLARE_PROPERTY(position_gain, double,
            "Control gain (1/s^2) on error in marker positions.")
    OpenSim_DECLARE_PROPERTY(speed_gain, double,
            "Control gain (1/s) on error in marker speeds.")
    /**@}**/

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    DirectMarkerTrackingController();
    DirectMarkerTrackingController(MarkersReference *markersRef);

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setPositionGain(double value)
    {
        set_position_gain(value);
    }
    double getPositionGain()
    {
        return get_position_gain();
    }

    void setSpeedGain(double value)
    {
        set_speed_gain(value);
    }
    double getSpeedGain() const
    {
        return get_speed_gain();
    }

    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    void computeControls(const SimTK::State & s,
                         SimTK::Vector & controls) const;

protected:

    void connectToModel(Model & model) OVERRIDE_11;

private:

    void constructProperties();

    static const int _numSpaceDims;

    int _numTasks;

    // TODO make this const reference.
    MarkersReference * _markersRef;
    int _numMarkers;
    SimTK::Array_<SimTK::MobilizedBodyIndex> _mobilizedBodyIndices;
    SimTK::Array_<SimTK::Vec3> _stationPositionsInBodies;

};

} // namespace

#endif
