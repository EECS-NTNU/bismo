#include <iostream>
using namespace std;

#include "BISMOInstruction.hpp"

int main() {
	BISMOSyncInstruction sync;
	BISMOFetchRunInstruction fetch;
	BISMOExecRunInstruction exec;
	BISMOResultRunInstruction res;
	cout << "sync: " << sizeof(sync) << endl;
	cout << "fetch: " << sizeof(fetch) << endl;
	cout << "exec: " << sizeof(exec) << endl;
	cout << "res: " << sizeof(res) << endl;
	return 0;
}
