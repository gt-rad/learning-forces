#include <iostream>
#include <iomanip>
#include "utils/CppCommon.h"

#include "myUtils/ConfigManager.h"
#include "myUtils/mathlib.h"

#include "CMAES/CMASearcher.h"
#include "CMAES/gownData.h"

#include <boost/interprocess/ipc/message_queue.hpp>
using namespace boost::interprocess;

using namespace std;


CMASearcher* gPolicySearch = NULL;

static void buildPolicy()
{
    CMAData* cmaData = NULL;
    cmaData = new gownData();
    
    int numParams = cmaData->GetNumParameters();
    double* params = new double[numParams];
    gPolicySearch = new CMASearcher;
    gPolicySearch->SetDimension(numParams);
    gPolicySearch->Search(cmaData, params, 100);
    LOG(INFO) << setprecision(16) << "params " << params[0] << endl;
     
    delete[] params;
}

int main(int argc, char* argv[])
{
    // google::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging((const char*)argv[0]);
    // Define logging flag
    FLAGS_alsologtostderr = true;
    FLAGS_minloglevel = google::INFO;
#ifndef _DEBUG
    FLAGS_log_dir = "./glog/";
#endif
    LOG(INFO) << "BioloidGP program begins...";
    
    srand( (unsigned int) time (NULL) );
    
    buildPolicy();
    
    return 0;
}
