#ifndef OPENSIM_DIRECTMARKERTRACKINGCONTROLLER_H_
#define OPENSIM_DIRECTMARKERTRACKINGCONTROLLER_H_

#include <OpenSim/OpenSim.h>
#include "osimPlugin.h"

using SimTK::Array_;
using SimTK::MobilizedBodyIndex;
using SimTK::Vec3;

namespace OpenSim {

    // TODO assuming only one level of tasks for now.


/**
 * A controller that finds the torques necessary for a model to track
 * experimental marker trajectories. Based off of Emel Demircan's direct marker
 * control, and Gerald Brantner's operational-space controller for BIOE 485.
 * */
    // TODO control which actuators are given to this controller.
class OSIMPLUGIN_API DirectMarkerTrackingController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(DirectMarkerTrackingController, Controller);

public:

    /// @name Property declarations
    /**@{**/
    OpenSim_DECLARE_PROPERTY(speed_gain, double,
            "Control gain (1/s) on error in marker speeds.");
    OpenSim_DECLARE_PROPERTY(position_gain, double,
            "Control gain (1/s^2) on error in marker positions.");
    /**@}**/

    // Constructors
    // ------------------------------------------------------------------------
    DirectMarkerTrackingController();
    DirectMarkerTrackingController(const MarkersReference & markersRef);

    // Pure virtual method implementations
    // ------------------------------------------------------------------------
    void computeControls(const SimTK::State & s, SimTK::Vector & controls) const;

private:

    static const int _numSpatialDims;

    int _numTasks;

    const MarkersReference & _markersRef;
    const int _numMarkers;
    Array_<MobilizedBodyIndex> _mobilizedBodyIndices;
    Array_<Vec3> _stationPositionsInBodies;

};

} // namespace

#endif
