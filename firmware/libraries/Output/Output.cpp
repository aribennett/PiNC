#include <Output.h>
#include <SerialClient.h>

OutputList outputList;

void OutputList::addOutput(Output* output)
{
    _outputList[_outputCount] = output;
    output->setId(_outputCount);
    ++_outputCount;
}