#ifndef _CONFIG_MANAGER
#define _CONFIG_MANAGER

#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

#undef ERROR
#include <glog/logging.h>
#include "RenderType.h"

using namespace google;
using namespace std;


class DecoConfigKeyPair
{
public:
	string Key;
	string Value;
};
class DecoConfigSection
{
private:
	string m_secName;
	vector<DecoConfigKeyPair> m_pairs;
public:
	DecoConfigSection(string name) : m_secName(name)
	{}
	const string& GetName()
	{
		return m_secName;
	}
	const string* Find(const char* Key) const;
	void AddKeyPair(const DecoConfigKeyPair& pair);
};



class DecoConfig
{
private:
	DecoConfig()
	{}
	~DecoConfig();
	static DecoConfig* msConfigManager;

	const static string m_configFile;
	vector<DecoConfigSection*> m_sections;
	void parseIntoSections (const string& tmpTextBuffer, vector<DecoConfigSection*>& sections);
	
public:
	void Init(const char* configPath = NULL);
	static DecoConfig* GetSingleton();
	static void DestroySingleton();
	BOOL GetString( const char* Section, const char* Key, char* Value, INT Size) const;
	BOOL GetSection( const char* Section, char* Result, INT Size) const;
	BOOL GetString(	const char* Section, const char* Key, string& Str) const;
	BOOL GetStringVector(const char* Section, const char* Key, vector<string>& Str) const;
	BOOL GetInt(const char*	Section, const char* Key, INT& Value) const;
	BOOL GetFloat(const char* Section,const char* Key, FLOAT& Value) const;
	BOOL GetDouble(const char* Section,const char* Key, DOUBLE& Value) const;
	BOOL GetBool(const char* Section, const char* Key, BOOL& Value) const;
	BOOL GetDoubleVector(const char* Section, const char* Key, vector<double>& ValueArray) const;
	BOOL GetVectorXd(const char* Section, const char* Key, Eigen::VectorXd& ValueArray) const;
	BOOL GetIntVector(const char* Section, const char* Key, vector<int>& ValueArray) const;
	const DecoConfigSection* FindSection(const char* Section) const ;
};

#endif
