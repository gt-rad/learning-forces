#ifndef TIME_H
#define TIME_H

#include "stdafx.h"
#include <ctime>
#include <memory>
#ifdef WIN32
#include <WinSock.h>
//#include "boost/date_time/posix_time/posix_time.hpp" 


//#include <windows.h> //I've ommited this line.
#if defined(_MSC_VER) || defined(_MSC_EXTENSIONS)
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000Ui64
#else
#define DELTA_EPOCH_IN_MICROSECS  11644473600000000ULL
#endif
#else
#include <sys/time.h>
#endif



#ifdef _WIN32
int gettimeofday(struct timeval *tv, struct DecoTimeZone *tz);

struct DecoTimeZone 
{
    int  tz_minuteswest; /* minutes W of Greenwich */
    int  tz_dsttime;     /* type of dst correction */
};

#endif

void printCurrentTime(const string& str);
char* getCurrentTimeString();

class DecoTimer
{
public:
  DecoTimer(void);
	~DecoTimer(void) {};
	void Start(const string& reason);
	double Stop(const string& reason);
	bool isStarted(void)
	{
		return (m_isStarted);
	}
	void startTimer(void);
	void resetTimer(void);
	void pauseTimer(void);
	void resumeTimer(void);
	double getCurrentTime(void);

private:
	bool   m_isPaused;
	bool m_isStarted;

	double timeDiffInSecond(timeval time1, timeval time2);
	double timeDiffInMilliSecond(timeval time1, timeval time2);
	timeval m_startTime;
	timeval m_pauseTime;
	timeval m_nowTime;

	string m_reason;
	clock_t m_startClock;
	time_t m_startT;

#ifdef _WIN32
	LARGE_INTEGER m_frequency;
	LARGE_INTEGER m_start;
#endif

};

#endif
