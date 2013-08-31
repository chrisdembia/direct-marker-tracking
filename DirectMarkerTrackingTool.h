#ifndef OPENSIM_DIRECTMARKERTRACKINGTOOL_H_
#define OPENSIM_DIRECTMARKERTRACKINGTOOL_H_

#include "osimPlugin.h"
#include <OpenSim/Simulation/Model/AbstractTool.h>
#include <OpenSim/Common/Function.h>

// TODO decide who adds the coordinateactuators.

namespace OpenSim {

class MarkersReference;

class OSIMPLUGIN_API DirectMarkerTrackingTool : public AbstractTool {
OpenSim_DECLARE_CONCRETE_OBJECT(DirectMarkerTrackingTool, AbstractTool)
public:
    /** @name Property declarations
     * These are the class's serializable properties.
     * */
    /**@{**/
    OpenSim_DECLARE_PROPERTY(marker_file, std::string,
            "Path to marker file (.trc) containing marker locations to track.")
    /**@}**/

    //--------------------------------------------------------------------------
    // CONSTRUCTION
    //--------------------------------------------------------------------------
    DirectMarkerTrackingTool();
    DirectMarkerTrackingTool(const std::string & aFileName,
            bool aLoadModel=true) SWIG_DECLARE_EXCEPTION;

    //--------------------------------------------------------------------------
    // GET AND SET
    //--------------------------------------------------------------------------
    void setMarkerDataFileName(const std::string & markerDataFileName)
    {
        set_marker_file(markerDataFileName);
    }
    const std::string & getMarkerDataFileName() const
    {
        return get_marker_file();
    }

    //--------------------------------------------------------------------------
    // INTERFACE
    //--------------------------------------------------------------------------
    virtual bool run() SWIG_DECLARE_EXCEPTION;

private:

    void constructProperties();
    void boundInitialAndFinalTimesByMarkerData(MarkersReference & markersRef);
    void addNecessaryAnalyses();

};

} // namespace

#endif
