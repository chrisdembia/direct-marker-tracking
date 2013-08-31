#include "DirectMarkerTrackingTool.h"

#include <OpenSim/Actuators/CoordinateActuator.h>
#include <OpenSim/Analyses/Actuation.h>
#include <OpenSim/Analyses/Kinematics.h>
#include <OpenSim/Common/IO.h>
#include <OpenSim/Simulation/MarkersReference.h>
#include <OpenSim/Simulation/Manager/Manager.h>
#include <OpenSim/Simulation/Model/Model.h>

#include "DirectMarkerTrackingController.h"

using std::cout;
using std::endl;
using std::string;

using namespace OpenSim;

DirectMarkerTrackingTool::DirectMarkerTrackingTool()
{
    constructProperties();
}

DirectMarkerTrackingTool::DirectMarkerTrackingTool(
        const std::string & aFileName, bool aLoadModel) :
    AbstractTool(aFileName, false)
{
    constructProperties();
    updateFromXMLDocument();

    if (aLoadModel)
    { // TODO don't understand this code.
        loadModel(aFileName); // TODO optional originalForceSet argument.
        updateModelForces(*_model, aFileName);
        setModel(*_model);
        setToolOwnsModel(true);
    }
}

void DirectMarkerTrackingTool::constructProperties()
{
    constructProperty_marker_file("Unassigned");
}

bool DirectMarkerTrackingTool::run()
{
    // General structure of this method is taken from InverseKinematicsTool and
    // CMCTool.

    cout << "Running tool " << getName() << "." << endl;

    // Check for a model.
    // ------------------------------------------------------------------------
    if (_model == NULL)
    {
        string msg = "ERROR- A model has not been set.";
        cout << msg << endl;
        throw Exception(msg, __FILE__, __LINE__);
    }

    // Change current working directory.
    // ------------------------------------------------------------------------
    string cwdBeforeExecution = IO::getCwd();
    string directoryOfSetupFile = IO::getParentDirectory(getDocumentFileName());
    IO::chDir(directoryOfSetupFile);

    try
    {
        IO::SetPrecision(_outputPrecision);

        // Read in marker trajectories.
        // ====================================================================
        MarkersReference markersRef;
        markersRef.loadMarkersFile(get_marker_file());

        // Check initial and final times for the simulation.
        // -------------------------------------------------------------------
        boundInitialAndFinalTimesByMarkerData(markersRef);


        // Modify ForceSet.
        // --------------------------------------------------------------------
        // Remove all existing forces. TODO remove this.
        _model->updForceSet().setSize(0);

        /* TODO look at usage in InverseDynamics
        // TODO overrides replaceForceSet behavior; deletes all forces.
        CoordinateActuator::CreateForceSetOfCoordinateActuatorsForModel(
                    initState, *_model, 1, false);
        // CreateForceSet... invalidates the system.
        // TODO is there an easier way to validate the system?
        initState = _model->initSystem();
        */

        createExternalLoads(_externalLoadsFileName, *_model);

        // Add CoordinateActuator for each Coordinate.
        // --------------------------------------------------------------------
        // TODO remove this comment above.

        // Add CoordinateActuator for each (enabled? TODO) coordinate.
        // ========================================================================
        // TODO use CoordinateActuator's static method.
        const CoordinateSet & cset = _model->getCoordinateSet();
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
            _model->addForce(act);
            // model.updForceSet().append(actuator).


        }
        /* TODO look at usage in InverseDynamics
        // TODO overrides replaceForceSet behavior; deletes all forces.
        CoordinateActuator::CreateForceSetOfCoordinateActuatorsForModel(
                    initState, *_model, 1, false);
        // CreateForceSet... invalidates the system.
        // TODO is there an easier way to validate the system?
        initState = _model->initSystem();
        */

        // Create controller.
        // ====================================================================
        DirectMarkerTrackingController * controller =
            new DirectMarkerTrackingController(&markersRef);
        controller->setName("DirectMarkerTrackingToolController");
        controller->setDisabled(false);

        // Tell model and controller about each other.
        // --------------------------------------------------------------------
        // Give the controller control of all actuators, which is necessarily
        // just the CoordinateActuators added just above.
        // TODO will not be so when selecting actuators more intelligently.
        // TODO must call this after initSystem, otherwise connectToModel is called twice.
        controller->setActuators(_model->getActuators());
        // TODO do not invoke connect. _model->addController(controller);
        //_model->updControllerSet().adoptAndAppend(controller);

        // Filter
        // ====================================================================
        // TODO only relevant for coordinate data?


        // Initialize model.
        // ====================================================================
        // Must be done before controller is constructed, so that its
        // constructor can get MobilizedBodyIndex's from the model.
        SimTK::State & initState = _model->initSystem();
        _model->getMultibodySystem().realize(initState, SimTK::Stage::Position);
        // TODO unnecessary while we're setting the model's actuators.
        _model->equilibrateMuscles(initState);

        // TODO must happen after initSystem so that we can get
        // mobilized body indices in the controller.
        _model->addController(controller);

        // Print way too much information to cout; before we add OUR analyses.
        _model->printDetailedInfo(initState, cout);

        // Add Analyses
        // ====================================================================
        addNecessaryAnalyses();

        // Control constraints
        // ====================================================================
        // TODO

        // Set initial configuration of model using IK.
        // ====================================================================
        // TODO do an IK to set the initial configuration of the model.
        // Probably have to init state again, then.
        // TODO may have to go before initSystem.


        // Prepare for forward simulation.
        // ====================================================================
        SimTK::RungeKuttaMersonIntegrator integrator(_model->getMultibodySystem());
        integrator.setMaximumStepSize(_maxDT);
        integrator.setMinimumStepSize(_minDT);
        integrator.setAccuracy(_errorTolerance);

        Manager manager(*_model, integrator);
        manager.setSessionName(getName());
        manager.setInitialTime(_ti);
        manager.setFinalTime(_tf);

        // TODO don't need this? _model->setAllControllersEnabled(true);
        // TODO especially because we enable our controller above.

        // Initialize auxiliary states
        // --------------------------------------------------------------------
        // TODO can this go above preparation for forward simulation?


        // And we're off!
        // ====================================================================
        // TODO do we need this?:
        initState.updTime() = _ti;
        // TODO do we need this realize?
        //_model->getMultibodySystem().realize(initState,
          //                                   SimTK::Stage::Acceleration);

        // Prepare paths for output files.
        IO::makeDir(getResultsDir());
        string outputPathPrefix = getResultsDir() + "/" + getName() + "_";
        string statesOutputPath = outputPathPrefix + "states.sto";

        // TODO
        std::cout << "DEBUG FILES" << std::endl;
        print("DEBUG_SETUP.xml");
        _model->print("DEBUG_MODEL.osim");

        try
        {
            manager.integrate(initState);
        }
        catch (const Exception & ex)
        {
            ex.print(cout);
            IO::chDir(cwdBeforeExecution);
            // Print partial results.
            manager.getStateStorage().print(statesOutputPath);
            throw ex;
        }
        catch (...)
        {
            IO::chDir(cwdBeforeExecution);
            // Print partial results.
            manager.getStateStorage().print(statesOutputPath);
            throw;
        }

        // Print results.
        // ====================================================================
        printResults(getName(), getResultsDir());
        _model->printControlStorage(outputPathPrefix + "controls.sto");
        manager.getStateStorage().print(statesOutputPath);
        // TODO unavailable controller->getPositionErrorStorage()->print(outputPathPrefix + "_pErr.sto");
        // TODO print marker errors, IK-style.
        
    }
    catch (const Exception & ex)
    {
        ex.print(cout);
        IO::chDir(cwdBeforeExecution);
        throw ex;
    }
    catch (...)
    {
        IO::chDir(cwdBeforeExecution);
        throw;
    }

    IO::chDir(cwdBeforeExecution);

    // Execution only reaches this point if successful.
    return true;
}


void DirectMarkerTrackingTool::boundInitialAndFinalTimesByMarkerData(
        MarkersReference & markersRef)
{
    SimTK::Vec2 markersValidTimeRange = markersRef.getValidTimeRange();
    if (markersValidTimeRange[0] > _ti)
    {
        cout << "The initial time for the run precedes the first time in "
            "the marker file " << get_marker_file() << ". Resetting the "
            "initial time from " << _ti << " seconds to " <<
            markersValidTimeRange[0] << " seconds." << endl;
        _ti = markersValidTimeRange[0];
    }

    if (markersValidTimeRange[1] < _tf)
    {
        cout << "The final time for the run precedes the last time in "
            "the marker file " << get_marker_file() << ". Resetting the "
            "final time from " << _tf << " seconds to " <<
            markersValidTimeRange[1] << " seconds." << endl;
        _tf = markersValidTimeRange[1];
    }
}

void DirectMarkerTrackingTool::addNecessaryAnalyses()
{
    int stepInterval = 1;
    AnalysisSet & as = _model->updAnalysisSet();
    // Add Actuation if necessary
    Actuation * act = NULL;
    for (int i=0; i < as.getSize(); i++)
        if (as.get(i).getConcreteClassName() == "Actuation")
        {
            act = (Actuation*)&as.get(i);
            break;
        }
    if (!act) {
        std::cout << "No Actuation analysis found in analysis set -- adding one" << std::endl;
        act = new Actuation(_model);
        act->setModel(*_model );
        act->setStepInterval(stepInterval);
        _model->addAnalysis(act);
    }

    // Add Kinematics if necessary
    // NOTE: also checks getPrintResultFiles() so that the Kinematics analysis added from the GUI does not count
    Kinematics *kin = NULL;
    for (int i=0; i < as.getSize(); i++)
        if (as.get(i).getConcreteClassName() == "Kinematics" && as.get(i).getPrintResultFiles())
        {
            kin = (Kinematics*)&as.get(i);
            break;
        }
    if (!kin) {
        std::cout << "No Kinematics analysis found in analysis set -- adding one" << std::endl;
        kin = new Kinematics(_model);
        kin->setModel(*_model );
        kin->setStepInterval(stepInterval);
        kin->setInDegrees(true);
        _model->addAnalysis(kin);
    } else {
        kin->setInDegrees(true);
    }
}
