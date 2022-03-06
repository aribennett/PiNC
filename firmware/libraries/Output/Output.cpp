#include <Output.h>
#include <SerialClient.h>
#include <IntervalTimer.h>

OutputList outputList;
IntervalTimer outputTimer;

void OutputList::addOutput(Output* output)
{
    _outputList[_outputCount] = output;
    output->setId(_outputCount);
    ++_outputCount;
}

void OutputList::runOutputs()
{
    for(uint8_t i = 0; i < _outputCount; ++i)
    {
        _outputList[i]->run();
    }
}

void runOutputsAlias()
{
    outputList.runOutputs();
}

void startOutputTimer()
{
    outputTimer.begin(runOutputsAlias, PMW_INTERVAL);
}
