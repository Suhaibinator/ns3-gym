#include "ns3/object.h"
#include "ns3/uinteger.h"
#include <vector>

#include <iostream>

using namespace ns3;

class MyReceived : public Object{

private:
	std::vector<uint64_t>* ref;


public:
	static TypeId GetTypeId (void);

MyReceived(){
	ref = 0;
}
MyReceived(std::vector<uint64_t>* v);


uint64_t GetPackets (int id);
int GetNumLeaf();

};
