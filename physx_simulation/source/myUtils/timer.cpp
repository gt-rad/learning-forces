#include "stdafx.h"
#include "timer.h"
#include <stdio.h>
#include <time.h>
#include <cassert>
#include <ctime>

 #ifdef _WIN32
int gettimeofday(struct timeval *tv, struct DecoTimeZone *tz)
{
  FILETIME ft;
  unsigned __int64 tmpres = 0;
  static int tzflag;
 
  if (NULL != tv)
  {
    GetSystemTimeAsFileTime(&ft);
 
    tmpres |= ft.dwHighDateTime;
    tmpres <<= 32;
    tmpres |= ft.dwLowDateTime;
 
    /*converting file time to unix epoch*/
    tmpres -= DELTA_EPOCH_IN_MICROSECS; 
    tmpres /= 10;  /*convert into microseconds*/
    tv->tv_sec = (long)(tmpres / 1000000UL);
    tv->tv_usec = (long)(tmpres % 1000000UL);
  }
 
  if (NULL != tz)
  {
    if (!tzflag)
    {
      _tzset();
      tzflag++;
    }
    tz->tz_minuteswest = _timezone / 60;
    tz->tz_dsttime = _daylight;
  }
 
  return 0;
}
#endif

void printCurrentTime(const string& str)
{
#ifdef _WIN32

#else
  time_t now = time(0);
  
  tm* localtm = localtime(&now);
  cout << str << asctime(localtm) << endl;
  //DebugOutput("%s %s.\n", str.c_str(), asctime(localtm));
#endif
}

char* getCurrentTimeString()
{
#ifdef _WIN32
  return NULL;
#else
  time_t now = time(0);
  
  tm* localtm = localtime(&now);
  return asctime(localtm);
#endif

}


DecoTimer::DecoTimer(void) : m_isPaused(false), m_isStarted(false)
{

}
void DecoTimer::Start(const string& reason)
{
	m_reason = reason;
	
#ifdef _WIN32
	QueryPerformanceFrequency(&m_frequency);
	QueryPerformanceCounter(&m_start);
	//m_startClock = clock();

#else
	startTimer();
	//m_startT = time(0);
#endif
}
double DecoTimer::Stop(const string& reason)
{
	CHECK(reason == m_reason) << "Timer Start() Stop() reason mismatch.";
	double diff = 0;
#ifdef _WIN32
	LARGE_INTEGER end;
	QueryPerformanceCounter(&end);
	diff = (double)(end.QuadPart - m_start.QuadPart) / m_frequency.QuadPart;
	//clock_t currentClock = std::clock();
	//diff = (currentClock - m_startClock ) / (double)CLOCKS_PER_SEC;
#else
	//time_t currentTime = time(0);
	//diff = difftime(currentTime, m_startT); 
	diff = this->getCurrentTime();
#endif
	//LOG(INFO) << m_reason << " spent " << diff << " seconds";
	return diff;
}

void DecoTimer::startTimer(void)
{
	resetTimer();
}

void DecoTimer::resetTimer(void)
{
  gettimeofday(&m_startTime, NULL);

  m_isStarted = true;
}
void DecoTimer::pauseTimer(void)
{
	if (!m_isPaused)
	{
	  gettimeofday(&m_pauseTime, NULL);

		m_isPaused = true;

	}
}
void DecoTimer::resumeTimer(void)
{
	if (m_isPaused)
	{
		timeval resumeTime;
		gettimeofday(&resumeTime, NULL);
		m_startTime.tv_sec += resumeTime.tv_sec - m_pauseTime.tv_sec;
		m_isPaused = false;
		
	}
}
double DecoTimer::getCurrentTime(void)
{
	if (m_isPaused)
	{
	  return timeDiffInSecond(m_pauseTime, m_startTime);
	} else {

	  gettimeofday(&m_nowTime, NULL);
	  return timeDiffInSecond(m_nowTime, m_startTime);
	}

}

double DecoTimer::timeDiffInSecond(timeval time1, timeval time2)
{
  double elapsedTime = 0;

  //  printf("timeDiffInSecond\n");
  //  printf("time1: %d, %d\n", time1.tv_sec, time1.tv_usec);
  //  printf("time2: %d, %d\n", time2.tv_sec, time2.tv_usec);
  elapsedTime = (time1.tv_sec - time2.tv_sec);
  elapsedTime += (time1.tv_usec - time2.tv_usec) / 1000000.0;

  return elapsedTime;
}

double DecoTimer::timeDiffInMilliSecond(timeval time1, timeval time2)
{
	double elapsedTime = (time1.tv_sec - time2.tv_sec);
	elapsedTime += (time1.tv_usec - time2.tv_usec) / 1000.0;
	return elapsedTime;
}

