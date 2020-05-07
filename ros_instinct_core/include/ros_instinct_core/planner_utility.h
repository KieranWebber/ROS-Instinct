#pragma once

#include <instinct/Instinct.h>
#include <iosfwd>
#include <sstream>

/**
 * Static utility functions to aid with the operation of the Instinct Planner
 */
class PlannerUtility
{
  private:
    /**
     * Basic alias for string startsWith
     *
     * @param line String to check against
     * @param start Sequence string is required to start with
     * @return True if line starts with start
     */
    static inline bool startsWith(const std::string& line, const char* start)
    {
        return line.rfind(start, 0) == 0;
    }

  public:
    /**
     * Loads an Instinct plan from text into the provided planner.
     *
     * Ignores all comments.
     * Adapted from https://github.com/rwortham/Instinct-RobotWorld/blob/master/InstinctRobotWorld.cpp
     *
     * @param pPlan Planner to load the plan into
     * @param pNames Names buffer array - Must be precreated large enough to avoid an overflow
     * @param planText Plan file as a text string separated with newlines
     * @param bufferSize Size of the buffer to use when receiving planner loading messages
     */
    static void loadPlan(Instinct::CmdPlanner* pPlan, Instinct::Names* pNames, const char* planText,
                         int bufferSize = 100)
    {
        char msgBuffer[bufferSize];
        std::stringstream planStream(planText);
        std::string planLine;
        while (std::getline(planStream, planLine))
        {
            // Ignore comments or empty lines
            if (planLine.length() == 0 || startsWith(planLine, "//"))
            {
                continue;
            }

            // If the line still has a carriage return remove it
            if (planLine.back() == '\r')
            {
                planLine.pop_back();
            }

            if (startsWith(planLine, "PLAN"))
            {
                std::string commandContents = planLine.substr(5);
                // c_str is null terminated by default (C++11)
                if (!(pPlan->executeCommand(commandContents.c_str(), msgBuffer, sizeof(msgBuffer))))
                {
                    throw "Error executing command!";
                }
            }
            else if (startsWith(planLine, "PELEM"))
            {
                char nameBuffer[20];
                unsigned int uiID;
                if (sscanf(planLine.c_str(), "PELEM %[^=]=%u", nameBuffer, &uiID) == 2)
                {
                    pNames->addElementName(uiID, nameBuffer);
                }
                else
                {
                    throw "Invalid named element format!";
                }
            }
        }
    }
};