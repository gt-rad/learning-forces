#include "CMASearcher.h"
#include "cmaes_interface.h"
#include "../myUtils/timer.h"
#include <stdio.h>
#include "../myUtils/ConfigManager.h"
#include "CMAData.h"

#include <boost/interprocess/ipc/message_queue.hpp>

using namespace boost::interprocess;


CMASearcher::CMASearcher() : mPrevSol(NULL), mStandardDeviation(NULL), mInfeasibleTime(NULL)
{
    mDim = 1;
    mEvaluator = NULL;
    
}
CMASearcher::CMASearcher(int dim) : mPrevSol(NULL), mStandardDeviation(NULL), mInfeasibleTime(NULL)
{
    SetDimension(dim);
    mEvaluator = NULL;
}
CMASearcher::~CMASearcher()
{
    clear();
}

void CMASearcher::SetDimension(int dim)
{
    clear();
    mDim = dim;
    mPrevSol = new double[mDim];
    mStandardDeviation = new double[mDim];
    mInfeasibleTime = new int[mDim];
    string cmaPath;
    DecoConfig::GetSingleton()->GetString("CMA", "CMAFilePath", cmaPath);
    setCMALogFilePath(cmaPath);
    
}
void CMASearcher::CreateMessageQueue()
{
    try{
        //Erase previous message queue
        char queueName[512];
        const int maxWorkLoadPerProcess = 64;
        for (int i = 0; i < NUM_IPC_PROCESS; ++i)
        {
            sprintf(queueName, "mqToSlave%d", i);
            message_queue::remove(queueName);
            message_queue mqOut(create_only, queueName, mDim * maxWorkLoadPerProcess, sizeof(double));
            
            sprintf(queueName, "mqFromSlave%d", i);
            message_queue::remove(queueName);
            message_queue mqIn(create_only, queueName, maxWorkLoadPerProcess, sizeof(double));
        }
        
        
    }
    catch(interprocess_exception &ex){
        LOG(INFO) << ex.what() << std::endl;
        
    }
}
void CMASearcher::clear()
{
    if (mPrevSol)
        delete[] mPrevSol;
    if (mStandardDeviation)
        delete[] mStandardDeviation;
    if (mInfeasibleTime)
        delete[] mInfeasibleTime;
}

void CMASearcher::setInitialGuess(double* lower_bound, double* upper_bound)
{
    vector<double> userSpecifiedMean;
    DecoConfig::GetSingleton()->GetDoubleVector("CycleTraining", "CMAMean", userSpecifiedMean);
    if (userSpecifiedMean.size() == mDim)
    {
        LOG(INFO) << "Using user specified mean.";
        
        for (int i = 0; i < mDim; ++i)
        {
            mPrevSol[i] = userSpecifiedMean[i];
        }
    }
    else
    {
        for (int i = 0; i < mDim; ++i)
        {
            //mPrevSol[i] = 0.5 * (lower_bound[i] + upper_bound[i]);
            mPrevSol[i] = 0.5 * (lower_bound[i] + upper_bound[i]);
        }
    }
    calculateSearchStandardDeviation(lower_bound, upper_bound);
    
}


void CMASearcher::recoverFromScaling(int n, double* lower_bound, double* upper_bound, double* scaledValue, double* trueValue)
{
    for (int i = 0; i < n; ++i)
    {
        trueValue[i] = scaledValue[i] * (upper_bound[i] - lower_bound[i]) + lower_bound[i];
    }
}

int CMASearcher::Search(CMAData* cData, double* argMin, int maxIterations)
{
    int nParams = cData->GetNumParameters();
    double* lb = new double[nParams];
    double* ub = new double[nParams];
    
    cData->GetParameterLowerBounds(lb);
    cData->GetParameterUpperBounds(ub);
    int ret = Search(cData, lb, ub, argMin, maxIterations);
    delete[] lb;
    delete[] ub;
    return ret;
}

int CMASearcher::Search(CMAData* cData, double* lower_bound, double* upper_bound, double* argMin, int maxIterations)
{
    DecoConfig::GetSingleton()->GetInt("CMA", "SearchProcessNum", NUM_IPC_PROCESS);
    CreateMessageQueue();
    
    const string cmaInitPath = "initials.par";
    const string cmaSignalPath = "signals.par";
    vector<double> normalizedLb(mDim, 0);
    vector<double> normalizedUb(mDim, 1);
    double* realPop = new double[mDim];
    setInitialGuess(&normalizedLb[0], &normalizedUb[0]);
    
    int iterationCount = 0;
    
    cmaes_t evo; /* an CMA-ES type struct or "object" */
    double *arFunvals, *const*pop, *xfinal;
    int i;
    
    /* Initialize everything into the struct evo, 0 means default */
    arFunvals = cmaes_init(&evo, mDim, mPrevSol, mStandardDeviation, 0, 0, cmaInitPath.c_str());
    cmaes_WriteLogFile(cmaes_SayHello(&evo));
    cmaes_WriteLogFile("\n");
    cmaes_ReadSignals(&evo, cmaSignalPath.c_str());  /* write header and initial values */
    /* Iterate until stop criterion holds */
    
    int nSamples = static_cast<int>(cmaes_Get(&evo, "lambda"));
    cout << "nSamples" << nSamples << endl;
    vector<Eigen::VectorXd> timingInfoPerProcess;
    timingInfoPerProcess.resize(nSamples);
    for (int i = 0; i < nSamples; ++i)
    {
        timingInfoPerProcess[i] = Eigen::VectorXd::Zero(maxIterations);
    }
    
    while(!cmaes_TestForTermination(&evo))
    {
        DecoTimer timer;
        timer.Start("CMA iteration");
        /* generate lambda new search points, sample population */
        pop = cmaes_SamplePopulation(&evo); /* do not change content of pop */
        
        for (i = 0; i < cmaes_Get(&evo, "popsize"); ++i)
        {
            while (!isFeasible(pop[i], &normalizedLb[0], &normalizedUb[0]))
                cmaes_ReSampleSingle(&evo, i);
        }
        int numSamples = static_cast<int>(cmaes_Get(&evo, "lambda"));
        
        // IPC
        
        LOG(INFO) << "sending parameters.";
        //CHECK(numSamples % NUM_IPC_PROCESS == 0);
        int workLoadPerProcess = numSamples / NUM_IPC_PROCESS;
        if (numSamples % NUM_IPC_PROCESS)
            workLoadPerProcess += 1;
        
        for (int i = 0; i < numSamples; ++i)
        {
            char queueName[512];
            int processId = i / workLoadPerProcess;
            sprintf(queueName, "mqToSlave%d", processId);
            message_queue mqOut(open_only, queueName);
            
            if (i % workLoadPerProcess == 0)
            {
                double numEvaluations = workLoadPerProcess;
                if (processId == NUM_IPC_PROCESS - 1)
                    numEvaluations = numSamples - (workLoadPerProcess * (NUM_IPC_PROCESS - 1));
                LOG(INFO) << "eval num: " << numEvaluations << " to process " << processId << endl;
                mqOut.send(&numEvaluations, sizeof(double), 0);
            }
            
            LOG(INFO) << "sending " << i << " to process " << processId;
            for (int j = 0; j < mDim; ++j)
            {
                mqOut.send(&(pop[i][j]), sizeof(double), 0);
            }
        }
        LOG(INFO) <<  "receiving rewards.";
        for (int i = 0; i < numSamples; ++i)
        {
            char logStr[1024];
            char queueName[512];
            int processId = i / workLoadPerProcess;
            
            sprintf(queueName, "mqFromSlave%d", processId);
            message_queue mqOut(open_only, queueName);
            double value;
            size_t recvd_size;
            unsigned int priority;
            mqOut.receive(&value, sizeof(double), recvd_size, priority);
            arFunvals[i] = (value);
            recoverFromScaling(mDim, lower_bound, upper_bound, pop[i], realPop);
            
            cmaes_WriteLogFile("(");
            
            for (int j = 0; j < mDim; ++j)
            {
                memset(logStr, 0, 512 * sizeof(char));
                sprintf(logStr, " %0.16f ", realPop[j]);
                
                cmaes_WriteLogFile(logStr);
                
            }
            memset(logStr, 0, 512 * sizeof(char));
            sprintf(logStr, "):   %f\n", arFunvals[i]);
            
            //timingInfoPerProcess[i][iterationCount] = values[3];
            LOG(INFO) <<  "Value for process " << processId << ": " << arFunvals[i];
            cmaes_WriteLogFile(logStr);
        }
        
        // end IPC
        
        /* update the search distribution used for cmaes_SampleDistribution() */
        cmaes_UpdateDistribution(&evo, arFunvals);
        
        double* covar = cmaes_GetNew(&evo, "diag(C)");
        double* mean = cmaes_GetNew(&evo, "xmean");
        double* best = cmaes_GetNew(&evo, "xbestever");
        double bestValue = *(cmaes_GetNew(&evo, "fbestever"));
        
        
        if (covar && mean && best)
        {
            const int strLen = 65536;
            char covarChar[strLen], meanChar[strLen], bestChar[strLen], bestValueChar[strLen];
            memset(covarChar, 0, strLen * sizeof(char));
            memset(meanChar, 0, strLen * sizeof(char));
            memset(bestChar, 0, strLen * sizeof(char));
            memset(bestValueChar, 0, strLen * sizeof(char));
            for (int ithDim = 0; ithDim < mDim; ++ithDim)
            {
                sprintf(covarChar, "%s%f  ", covarChar, covar[ithDim]);
                sprintf(meanChar, "%s%f  ", meanChar, mean[ithDim]);
                sprintf(bestChar, "%s%f  ", bestChar, best[ithDim]);
                
            }
            cmaes_WriteLogFile("Diag Covariance:   ");
            cmaes_WriteLogFile(covarChar);
            cmaes_WriteLogFile("\n");
            cmaes_WriteLogFile("Mean:             ");
            cmaes_WriteLogFile(meanChar);
            cmaes_WriteLogFile("\n");
            cmaes_WriteLogFile("Best x so far:    ");
            cmaes_WriteLogFile(bestChar);
            cmaes_WriteLogFile("\n");
            sprintf(bestValueChar, "%f ", bestValue);
            cmaes_WriteLogFile("Best f so far:    ");
            cmaes_WriteLogFile(bestValueChar);
            cmaes_WriteLogFile("\n");
            free(covar);
            free(mean);
            free(best);
        }
        char logStr[512];
        double currentTime = timer.Stop("CMA iteration");
        iterationCount++;
        memset(logStr, 0, 512 * sizeof(char));
        sprintf(logStr, "-------------------%d CMA iteration finished in %f seconds---------------\n", iterationCount, currentTime);
        cmaes_WriteLogFile(logStr);
        cmaes_WriteLogFile("\n\n");
        
        if (iterationCount >= maxIterations)
        {
            break;
        }
        //	/* read instructions for printing output or changing termination conditions */
        cmaes_ReadSignals(&evo, cmaSignalPath.c_str());
        fflush(stdout); /* useful in MinGW */
    }
    if (iterationCount < maxIterations)
    {
        const char* terminationText = cmaes_TestForTermination(&evo);
        cmaes_WriteLogFile("Stop:\n");
        cmaes_WriteLogFile(terminationText); /* print termination reason */
    }
    
    /* get best estimator for the optimum, xmean */
    xfinal = cmaes_GetNew(&evo, "xbestever"); /* "xbestever" might be used as well */
    cmaes_exit(&evo); /* release memory */
    memcpy(mPrevSol, xfinal, mDim * sizeof(double));
    memcpy(argMin, xfinal, mDim * sizeof(double));
    /* do something with final solution and finally release memory */
    free(xfinal); 
#ifdef _IPC
    for (int i = 0; i < NUM_IPC_PROCESS; ++i)
    {
        char queueName[512];
        
        sprintf(queueName, "mqToSlave%d", i);
        message_queue mqOut(open_only, queueName);
        
        double numEvaluations = -1;
        mqOut.send(&numEvaluations, sizeof(double), 0);
    }
#endif
    delete[] realPop;
    return 1;
}

void CMASearcher::calculateSearchStandardDeviation(double* lower_bound, double* upper_bound)
{
    vector<double> userSpecifiedSD;
    DecoConfig::GetSingleton()->GetDoubleVector("CycleTraining", "CMAStandarDeviation", userSpecifiedSD);
    if (userSpecifiedSD.size() == mDim)
    {
        LOG(INFO) << "Using user specified standard deviation.";
        for (int i = 0; i < mDim; ++i)
        {
            mStandardDeviation[i] = userSpecifiedSD[i];
        }
    }
    else
    {
        for (int i = 0; i < mDim; ++i)
        {
            double minBoundary = abs(mPrevSol[i] - lower_bound[i]);
            double maxBoundary = abs(mPrevSol[i] - upper_bound[i]);
            mStandardDeviation[i] = 5 * max(minBoundary, maxBoundary) / 2.0;
        }
    }
}

bool CMASearcher::isFeasible(double* value, double* lower_bound, double* upper_bound)
{
    for (int i = 0; i < mDim; ++i)
        if (value[i] < lower_bound[i] || value[i] > upper_bound[i])
        {
            
            //printf("Infeasible sample hit!\n");
            return false;
        }
    
    return true;
}







