#ifndef OPENSIM_TASKSPACE_CONTROLLER_H_
#define OPENSIM_TASKSPACE_CONTROLLER_H_

namespace OpenSim {

namespace TaskSpace {

class Controller : public OpenSim::Controller
{
public:
    Controller();
private:
    TaskSpace::TaskSet _taskSet;
};

} // namespace TaskSpace

} // namespace OpenSim

#endif // TASKSPACECONTROLLER_H
