#include "packetsStatus.h"

#include <iostream>

TypeId MyReceived::GetTypeId (void) {
	static TypeId tid = TypeId ("MyReceived")
	.SetParent(Object::GetTypeId ())
	.AddConstructor<MyReceived>();
	return tid;
}

MyReceived::MyReceived(std::vector<uint64_t>* v){
	ref = v;
}


uint64_t MyReceived::GetPackets (int id){
	return ref->at(id);
}

int MyReceived::GetNumLeaf(){
	return 3;
}