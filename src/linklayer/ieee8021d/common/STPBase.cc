//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 
// Author: Zsolt Prontvai
//

#include "STPBase.h"
#include "InterfaceEntry.h"
#include "ModuleAccess.h"
#include "NodeOperations.h"
#include "NodeStatus.h"

static const char *ENABLED_LINK_COLOR = "#000000";
static const char *DISABLED_LINK_COLOR = "#bbbbbb";
static const char *ROOT_SWITCH_COLOR = "#a5ffff";

void STPBase::initialize(int stage)
{
    if (stage == 0)
    {
        visualize = par("visualize");
        bridgePriority = par("bridgePriority");

        maxAge = par("maxAge");
        helloTime = par("helloTime");
        forwardDelay = par("forwardDelay");

        macTable = check_and_cast<MACAddressTable *>(getModuleByPath(par("macTableName")));
        ifTable = check_and_cast<IInterfaceTable*>(getModuleByPath(par("interfaceTableName")));
        numPorts = this->getParentModule()->gate("ethg$o", 0)->getVectorSize();
    }

    if (stage == 1) // "auto" MAC addresses assignment takes place in stage 0
    {
        NodeStatus *nodeStatus = dynamic_cast<NodeStatus *>(findContainingNode(this)->getSubmodule("status"));
        isOperational = (!nodeStatus) || nodeStatus->getState() == NodeStatus::UP;

        // gets the bridge's mac address
        InterfaceEntry * ifEntry = ifTable->getInterface(0);
        if (ifEntry != NULL)
            bridgeAddress = ifEntry->getMacAddress();
        else
        {
            EV_INFO<<"interface not found. Is not this module connected to another BEB?"<<endl;
            EV_INFO<<"Setting AAAAAA000001 as backbone mac address."<<endl;
            bridgeAddress.setAddress("AAAAAA000001");
        }
    }
}

bool STPBase::handleOperationStage(LifecycleOperation *operation, int stage, IDoneCallback *doneCallback)
{
    Enter_Method_Silent();

    if (dynamic_cast<NodeStartOperation *>(operation))
    {
        if (stage == NodeStartOperation::STAGE_LINK_LAYER)
            start();
    }
    else if (dynamic_cast<NodeShutdownOperation *>(operation))
    {
        if (stage == NodeShutdownOperation::STAGE_LINK_LAYER)
            stop();
    }
    else if (dynamic_cast<NodeCrashOperation *>(operation))
    {
        if (stage == NodeCrashOperation::STAGE_CRASH)
            stop();
    }
    else
        throw cRuntimeError("Unsupported operation '%s'", operation->getClassName());

    return true;
}

void STPBase::start()
{
    isOperational = true;
}

void STPBase::stop()
{
    isOperational = false;
    // colors all connected link gray
    for (unsigned int i = 0; i < numPorts; i++)
        colorLink(i, false);
    this->getParentModule()->getDisplayString().setTagArg("i", 1, "");
}

void STPBase::colorLink(unsigned int i, bool forwarding)
{
    if (ev.isGUI() && visualize)
    {
        cGate * outGate = getParentModule()->gate("ethg$o", i);
        cGate * inGate = getParentModule()->gate("ethg$i", i);
        cGate * outGateNext = outGate->getNextGate();
        cGate * inGatePrev = inGate->getPreviousGate();
        cGate * outGatePrev = outGate->getPreviousGate();
        cGate * inGatePrev2 = inGatePrev->getPreviousGate();

        if (outGate && inGate && inGatePrev && outGateNext && outGatePrev && inGatePrev2)
        {
            if(forwarding)
            {
                outGatePrev->getDisplayString().setTagArg("ls", 0, ENABLED_LINK_COLOR);
                inGate->getDisplayString().setTagArg("ls", 0, ENABLED_LINK_COLOR);
            }
            else
            {
                outGatePrev->getDisplayString().setTagArg("ls", 0, DISABLED_LINK_COLOR);
                inGate->getDisplayString().setTagArg("ls", 0, DISABLED_LINK_COLOR);
            }

            if((!inGatePrev2->getDisplayString().containsTag("ls") || strcmp(inGatePrev2->getDisplayString().getTagArg("ls", 0),ENABLED_LINK_COLOR) == 0) && forwarding)
            {
                outGate->getDisplayString().setTagArg("ls", 0, ENABLED_LINK_COLOR);
                inGatePrev->getDisplayString().setTagArg("ls", 0, ENABLED_LINK_COLOR);
            }
            else
            {
                outGate->getDisplayString().setTagArg("ls", 0, DISABLED_LINK_COLOR);
                inGatePrev->getDisplayString().setTagArg("ls", 0, DISABLED_LINK_COLOR);
            }
        }
    }
}

void STPBase::visualizer()
{
    if (ev.isGUI() && visualize)
    {
        Ieee8021DInterfaceData * port;
        for (unsigned int i = 0; i < numPorts; i++)
        {
            port = getPortInterfaceData(i);

            // color link
            colorLink(i, port->getState() == Ieee8021DInterfaceData::FORWARDING);

            // label ethernet interface with port status and role
            cModule * puerta = this->getParentModule()->getSubmodule("eth", i);
            if (puerta != NULL)
            {
                char buf[25];
                sprintf(buf, "%s\n%s\n", port->getRoleName(), port->getStateName());
                puerta->getDisplayString().setTagArg("t", 0, buf);
            }
        }

        // mark root switch
        if (getRootIndex() == -1)
            this->getParentModule()->getDisplayString().setTagArg("i", 1, ROOT_SWITCH_COLOR);
        else
            this->getParentModule()->getDisplayString().setTagArg("i", 1, "");
    }
}

Ieee8021DInterfaceData * STPBase::getPortInterfaceData(unsigned int portNum)
{
    cGate * gate = this->getParentModule()->gate("ethg$o", portNum);
    if (!gate)
        error("gate is NULL");
    InterfaceEntry * gateIfEntry = ifTable->getInterfaceByNodeOutputGateId(gate->getId());
    if (!gateIfEntry)
        error("gateIfEntry is NULL");
    Ieee8021DInterfaceData * portData = gateIfEntry->ieee8021DData();
    if (!portData)
        error("IEEE8021DInterfaceData not found!");

    return portData;
}

int STPBase::getRootIndex()
{
    int result = -1;
    for (unsigned int i = 0; i < numPorts; i++)
    {
        Ieee8021DInterfaceData * iPort = getPortInterfaceData(i);
        if (iPort->getRole() == Ieee8021DInterfaceData::ROOT)
        {
            result = i;
            break;
        }
    }
    return result;
}
