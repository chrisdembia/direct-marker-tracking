#include <iostream>
#include "DirectMarkerTrackingTool.h"

int main(int argc, char * argv[])
{
    if (argc == 1)
    {
        std::string fname = "default_tracker_setup.xml";
        std::cout << "No setup file provided. Printing default as " <<
                     fname << "." << std::endl;
        OpenSim::DirectMarkerTrackingTool tool;
        tool.setMarkerDataFileName("Unassigned");
        tool.print(fname);
    }
    else
    {
        OpenSim::DirectMarkerTrackingTool tool(argv[1]);
        tool.run();
    }

    return EXIT_SUCCESS;
}
