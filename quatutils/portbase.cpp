#include "portbase.h"
#include "portrs232.h"

PortBase::PortBase(const PortProperty& portpty):
    QThread()
{
    this->portproperty = portpty;
	analyzer = NULL;
	canbedestroy = false;
}


PortBase* createPort(const PortProperty& portpty){
   // if(0 == config->get("_setup_","port").compare("hid",Qt::CaseInsensitive))
   //     return new PortHid(config);
    //default
   return new PortRs232(portpty, true);
};


PortBase::~PortBase(){
    //if (analyzer)
	//	delete analyzer;
}
