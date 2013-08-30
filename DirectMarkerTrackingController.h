#ifndef OPENSIM_DIRECTMARKERTRACKINGCONTROLLER_H_
#define OPENSIM_DIRECTMARKERTRACKINGCONTROLLER_H_
#include <osimPlugin.h>

namespace OpenSim {

/**
 * A controller that finds the torques necessary for a model to track
 * experimental marker trajectories. Based off of Emel Demircan's direct marker
 * control, and Gerald Brantner's operational-space controller for BIOE 485.
 * */
    // TODO control which actuators are given to this controller.
class OSIMPLUGIN_API DirectMarkerTrackingController : public Controller {
OpenSim_DECLARE_CONCRETE_OBJECT(DirectMarkerTrackingController, Controller);

public:

    /** @name Property declarations
     * */
    /**@{**/
    // TODO OpenSim_DECLARE_PROPERTY();
    /**@}**/


    // Constructors
    // ------------------------------------------------------------------------
    DirectMarkerTrackingController();

    // Pure virtual method implementations
    // ------------------------------------------------------------------------
    void computeControls(const SimTK::State & s, SimTK::Vector & controls) const;

private:
    static const numSpatialDims;


};

} // namespace

#endif
