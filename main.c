
/*rtknav demo without gui*/

#include <stdio.h>
#include "rtklib.h"
#include <vector>
#include <iostream>
#include<process.h>
#include<windows.h>

using namespace std;

#define TRACEFILE   "rtknavi_%Y%m%d%h%M.trace" // debug trace file
#define STATFILE    "rtknavi_%Y%m%d%h%M.stat"  // solution status file
#define DEBUGTRACEF 1   //debug trace file 
#define DEBUGSTATUSF 1  //debug status file

#define GNSS_VECTOR_SIZE 5 /*gnss buffer length*/

vector <GPS_SIG_Data> gnssVector;//gps vector
HANDLE hsem;//Semaphore handle
lock_t lockvec;

/*push gnss data to vector*/
void transmitGNSS(ubx_nav_pvt pvt) {

	GPS_SIG_Data gpsData = { 0 };

	gpsData.nTimeTag = pvt.nTimeTag;
	gpsData.year = pvt.year;
	gpsData.month = pvt.month;
	gpsData.day = pvt.day;
	gpsData.hour = pvt.hour;
	gpsData.min = pvt.hour;
	gpsData.sec = pvt.sec;
	gpsData.dLat = pvt.lat;
	gpsData.dLon = pvt.lon;
	gpsData.dHeight = pvt.height;
	gpsData.dHeading = pvt.headMot;
	gpsData.dSpeed = pvt.gSpeed;
	gpsData.dHAccuarcy = pvt.hAcc;
	gpsData.dHDop = pvt.pDop;
	gpsData.dVDop = pvt.pDop;
	gpsData.dPDop = pvt.pDop;
	gpsData.nSatsNum = pvt.numSV;
	gpsData.gpsStatus = pvt.gpsStatus;

	lock(&lockvec);
	gnssVector.push_back(gpsData);
	printf("gnss data coming in status:%d  timeofweek:%f\n", gpsData.gpsStatus, gpsData.sec);
	while (gnssVector.size() > GNSS_VECTOR_SIZE) {
		vector<GPS_SIG_Data> ::iterator it = gnssVector.begin();
		gnssVector.erase(it);
	}
	ReleaseSemaphore(hsem, 1, NULL);
	unlock(&lockvec);
}
/*
*return : status (1:ok 0:error)
*/
int startRtkSvr(rtksvr_t *rtksvr,const char *strFile,const char *optFile) 
{
	int i=0;
	int SvrCycle;
	int SvrBuffSize;
	int strs[MAXSTRRTK] = { 0 };
	char *paths[MAXSTRRTK];
	int Format[MAXSTRRTK] = { 0 };
	int NavSelect;
	char * cmds[3] = { NULL,NULL,NULL };
	char *cmds_periodic[3] = { NULL,NULL,NULL };
	char *rcvopts[3] = { NULL,NULL,NULL };
	int NmeaCycle;
	int NmeaReq;
	double nmeapos[3] = { 0 };

	/* load stream settings from configuration file */
	stropt_t StrOpt;
	resetstropts();
	if (!loadopts(strFile, stropts)) return 0;
	getstropts(&StrOpt);

	SvrCycle = StrOpt.SvrCycle;
	SvrBuffSize = StrOpt.SvrBuffSize;

	for (i = 0; i < MAXSTRRTK; i++) {
		strs[i] = StrOpt.strs[i];
		paths[i] = StrOpt.paths[i];
		Format[i] = StrOpt.Format[i];
	}

	NavSelect = StrOpt.NavSelect;

	for (i = 0; i < 3; i++) {
		cmds[i] = StrOpt.cmds_s[i];
		cmds_periodic[i] = StrOpt.cmds_periodic[i];
		rcvopts[i] = StrOpt.rcvopts[i];
		nmeapos[i] = StrOpt.nmeapos[i];
	}

	NmeaCycle = StrOpt.NmeaCycle;
	NmeaReq = StrOpt.NmeaReq;

	prcopt_t PrcOpt;
	solopt_t solopt[2];
	stream_t monistr;  // monitor stream
	char errmsg[2048];

	/* load options from configuration file */
	resetsysopts();
	if (!loadopts(optFile, sysopts)) return 0;
	getsysopts(&PrcOpt, NULL, NULL);

	solopt[0] = solopt_default;
	solopt[1] = solopt_default;

	//rtk server initial 
	rtksvrinit(rtksvr);
	strinit(&monistr);

	//create Semaphore
	hsem=CreateSemaphore(NULL, 0, 1, NULL);
	initlock(&lockvec);

	// start rtk server
	if (!rtksvrstart(rtksvr, SvrCycle, SvrBuffSize, strs, paths, Format, NavSelect,
		cmds, cmds_periodic, rcvopts, NmeaCycle, NmeaReq, nmeapos, &PrcOpt, solopt,
		&monistr, errmsg)) {
		traceclose();
		return 0;
	}
	else
	{
		if (DEBUGTRACEF > 0) {
			traceopen(TRACEFILE);
			tracelevel(DEBUGTRACEF);
		}
		if (DEBUGSTATUSF > 0) {
			rtkopenstat(STATFILE, DEBUGSTATUSF);
		}

		return 1;
	}

	return 0;
}

/*stop rtkserver thread*/
void stopRtkSvr(rtksvr_t *svr)
{
	//close rtk server
	char cmdss[3][256] = { 0 };
	char *cmds[3] = { 0 };
	cmds[0] = cmdss[0];
	cmds[1] = cmdss[1];
	cmds[2] = cmdss[2];

	rtksvrstop(svr, cmds);
	CloseHandle(hsem);

	if (DEBUGTRACEF > 0) traceclose();
	if (DEBUGSTATUSF > 0) rtkclosestat();
}


void main()
{
	int count = 0;
	rtksvr_t rtksvr = {0}; // rtk server struct

	char strsettingFile[] = "DATA\\strset.conf";
	char optFile[] = "DATA\\rtk.conf";

	if (!startRtkSvr(&rtksvr, strsettingFile, optFile)) {
		printf("start fail\n");
		return;
	}

	//get result from rtk server
	while (TRUE){

		Sleep(1);

		WaitForSingleObject(hsem, INFINITE);

		lock(&lockvec);
		printf("size:%d\n", gnssVector.size());
		unlock(&lockvec);
	}

	return;
	
	
}
