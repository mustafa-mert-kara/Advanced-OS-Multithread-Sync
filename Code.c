#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <mqueue.h> 
#include <string.h>
#include <sys/stat.h> 
#include <sys/time.h>
#include <math.h>
#include <semaphore.h>
#include <time.h>
#include <errno.h>
#define PRODMEAN 500
#define PRODSTD 100
#define ACTIVEADVANTAGE 2
#define GROUPADVANTAGE 5
#define CARPASSINGTIME 1000
#define LANECHECKTIME 1000

/*
Wait: 
	No car present
	Lane no longer has prio(prio+activeLaneCost)
	Other lane in the laneGroup has dropped lock -- Maybe
Run:
	There are cars in lane
	Lane has prio
	can currently run in the crosssection -- Check all locks
Wakeup:
	car has arrived
	signaled by other laneGroup dropping lock
	after time

*/
char laneDirs[11]={' ','W','W','W','S','S','S','S','E','E','E'};
int crossedLanes[11][11]={{},{5, 6, 8, 9, 10},{5,6,7,8},{},{8},{1, 2, 8, 9},{1, 2, 8, 9, 10},{2},{1, 2, 4, 5, 6 },{1, 5, 6 },{1,6}};
int laneGroups[3][2]={{1,2},{5,6},{8,9}};
int crossSections[4][3]={{1,2,3},{-4,-8},{-7,-2},{-10,-1,-6}};
static int laneNumber=10;
static int laneGroupNumber=3;
static int crossSectionNumber=4;

int64_t start_time;

typedef struct THREADINPUT
{
	struct LANE* lane;
	struct LANEGROUP* group;
	struct CROSSSECTION* section[3];
	int sectionCount;
}t_input;

typedef struct CAR
{
	int name;
	int64_t creationTime;
	int64_t passageTime;
	int isItFinal;
}car;


typedef struct CROSSSECTION{
	int id;	
	pthread_mutex_t dataMutex;
	struct LANEGROUP** crossingLaneGroups;
	int totalLaneGroups;
	struct LANE** crossingLanes;
	int totalLanes; 
	struct LANE** currentlyCrossing;
	int currPrioHolder;
}crossSection;

typedef struct LANE{
	int id;
	struct LANE** crossingLanes;
	int crossingCount;
	mqd_t msgq_id;
	int lanePrio;	
	int64_t lastTimeLock;
	int64_t carCreationTimes[10];
	pthread_cond_t condList;	
	pthread_mutex_t prioMutex;
	int laneFinished;
}lane;

typedef struct LANEGROUP{
	int id;
	struct LANE** lanes;
	int laneCount;
	int groupPrio;
	int groupFinished;
}laneGroup;

void createWaittime(struct timespec *ts){
	struct timeval tv;
	gettimeofday(&tv,NULL);
	ts->tv_sec=tv.tv_sec;	

// wake up at most every 1000ms
	ts->tv_nsec =tv.tv_usec*1000+ 1000 * 1000000;
	if (ts->tv_nsec >= 1000000000) {
		ts->tv_sec++;
		ts->tv_nsec -= 1000000000;
	}
}

double gaussian() {
	double z0;
	do{
	double u1 = (rand() + 1.0) / (RAND_MAX + 2.0);
    double u2 = (rand() + 1.0) / (RAND_MAX + 2.0);

    z0= sqrt(-2.0 * log(u1)) * cos(2.0 * 3.14 * u2);
	} while(z0<0);
    
    return (z0 * PRODSTD + PRODMEAN);
}

void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

long gettime(struct timeval tv){
	int64_t s1 = (int64_t)(tv.tv_sec) * 1000 + (tv.tv_usec / 1000);
  	return s1-start_time;
}

void calcPrio(lane* Lane,int active){
	int i;
	struct timeval tv;
	gettimeofday(&tv,NULL);
	int64_t currTime=gettime(tv);
	double totalWaitTime=0;
	int carsWaiting=0;
	for(i=0;i<10;i++){
		if(Lane->carCreationTimes[i]!=-1){
			totalWaitTime+=abs(currTime-Lane->carCreationTimes[i]);
			carsWaiting++;
		}
			
	}
	if(carsWaiting==0){
		Lane->lanePrio=0;
		return;
	}
	int activeEffect=1;
	if(active){
		activeEffect=ACTIVEADVANTAGE;
		// printf("Lane %d is Active\n",Lane->id);
	}
	int newPrio=activeEffect*abs(currTime- Lane->lastTimeLock)*(totalWaitTime)/(currTime*carsWaiting);
	if(newPrio<0){
		newPrio=__INT16_MAX__;
	}
	Lane->lanePrio=newPrio;
}

void calcGroupPrio(laneGroup* group){
	int i;
	int tmp=0;
	if(group==NULL)
		return;
	for(i=0;i<group->laneCount;i++){
		if(group->lanes[i]->laneFinished==0)
			tmp+=group->lanes[i]->lanePrio;
	}
	group->groupPrio=tmp;
}

int doIHavePrio(crossSection* section,laneGroup* group, lane* Lane){
		
	if(section->totalLaneGroups>0){
		if((section->currPrioHolder==-1)||(group->id==section->currPrioHolder)){
			section->currPrioHolder=group->id;
			return 1;
		}
			
	}else{
		if((section->currPrioHolder==-1)||(Lane->id==section->currPrioHolder)){
			section->currPrioHolder=Lane->id;
			return 1;
		}
			
	}

	return 0;
}

void clearPrio(t_input* laneInfo){
	if(laneInfo->sectionCount==0)
		return;
	int i;
	for(i=0;i<laneInfo->sectionCount;i++){
		pthread_mutex_lock(&laneInfo->section[i]->dataMutex);
		int comp;
		if(laneInfo->section[i]->totalLaneGroups>0)
			comp=laneInfo->group->id;
		else
			comp=laneInfo->lane->id;
		if(laneInfo->section[i]->currPrioHolder==comp)
			laneInfo->section[i]->currPrioHolder=-1;
		pthread_mutex_unlock(&laneInfo->section[i]->dataMutex);
	}
}




int checkPrio(crossSection* section,laneGroup* group, lane* Lane, int advantage ){
	int i;
	int maxPrio=-1000;
	int maxPrioIdx=-1;
	if(section==NULL)
		return 1;
	int amIAllowedToChange=doIHavePrio(section,group,Lane);
	if(amIAllowedToChange){
		if(section->totalLaneGroups>0){
			for(i=0;i<section->totalLaneGroups;i++){
				if((section->crossingLaneGroups[i]->groupFinished==0)&&(section->crossingLaneGroups[i]->groupPrio>maxPrio)){
					maxPrio=section->crossingLaneGroups[i]->groupPrio;
					maxPrioIdx=i;
				}
			}
			pthread_mutex_lock(&section->dataMutex);
			if(maxPrioIdx==-1){
				section->currPrioHolder=-1;	
			}else{
				section->currPrioHolder=section->crossingLaneGroups[maxPrioIdx]->id;
			}			
			
			pthread_mutex_unlock(&section->dataMutex);
			if(section->currPrioHolder==group->id)
				return 1;
			else
				return 0;
		}else{
			for(i=0;i<section->totalLanes;i++){
				int lanePrio=section->crossingLanes[i]->lanePrio;				
				if((section->crossingLanes[i]->laneFinished==0)&&(lanePrio>maxPrio)){
					maxPrio=section->crossingLanes[i]->lanePrio;
					maxPrioIdx=i;
				}
			}
			pthread_mutex_lock(&section->dataMutex);
			if(maxPrioIdx==-1){
				section->currPrioHolder=-1;
			}else{
				section->currPrioHolder=section->crossingLanes[maxPrioIdx]->id;
			}			
			pthread_mutex_unlock(&section->dataMutex);
			if(section->currPrioHolder==Lane->id)
				return 1;
			else
				return 0;
		}
	}
	return 0;
}

int checkSectionUsage(crossSection* section,lane* Lane){
int i;
for(i=0;i<Lane->crossingCount;i++){
	int laneidx=Lane->crossingLanes[i]->id;	
	if(section->currentlyCrossing[laneidx]!=NULL){
		pthread_mutex_unlock(&section->dataMutex);
		return 0;
	}
}

return 1;
}

int checkAllPrios(t_input* laneInfo){
	if(laneInfo->sectionCount==0)
		return 1;
	int i;
	for(i=0;i<laneInfo->sectionCount;i++){
		pthread_mutex_lock(&laneInfo->section[i]->dataMutex);
		int res=doIHavePrio(laneInfo->section[i],laneInfo->group,laneInfo->lane);
		pthread_mutex_unlock(&laneInfo->section[i]->dataMutex);
		if(res==0){
			return 0;
		}
			
	}
	return 1;
}

int updateAllPrios(t_input* laneInfo){
	if(laneInfo->sectionCount==0)
		return 1;
	int i;
	int groupAdvantage=0;
	for(i=0;i<laneInfo->sectionCount;i++){
		if(laneInfo->section[i]->totalLaneGroups>0){
			groupAdvantage=1;
			if(checkPrio(laneInfo->section[i],laneInfo->group,laneInfo->lane,0)==0)
				return 0;
		}
		
	}
	for(i=0;i<laneInfo->sectionCount;i++){
		if(laneInfo->section[i]->totalLanes>0){
			if(checkPrio(laneInfo->section[i],laneInfo->group,laneInfo->lane,groupAdvantage)==0)
				return 0;
		}
		
	}
	return 1;
}

 void getAllKeys(crossSection** sections,int sectionsCount,lane* Lane){
	int i;
	for(i=0;i<sectionsCount;i++){
		
		pthread_mutex_lock(&Lane->prioMutex);
		while(checkSectionUsage(sections[i],Lane)==0){
			struct timespec ts;
			createWaittime(&ts);
			int rc =pthread_cond_timedwait(&Lane->condList,&Lane->prioMutex,&ts);
		}
		sections[i]->currentlyCrossing[Lane->id]=Lane;
		pthread_mutex_unlock(&Lane->prioMutex);
		
	}
 }
void releaseAllKeys(crossSection** sections,int sectionsCount,lane* Lane){
	int i;
	struct timeval tv;
	gettimeofday(&tv,NULL);
	int64_t currTime=gettime(tv);
	Lane->lastTimeLock=currTime;
	for(i=0;i<sectionsCount;i++){
		sections[i]->currentlyCrossing[Lane->id]=NULL;		
	}
	for(i=0;i<Lane->crossingCount;i++){
		pthread_mutex_lock(&Lane->crossingLanes[i]->prioMutex);
		pthread_cond_signal(&Lane->crossingLanes[i]->condList);
		pthread_mutex_unlock(&Lane->crossingLanes[i]->prioMutex);
	}
 }
void* carProducer(void* arg){
	t_input* laneInfo=(t_input*)arg;
	lane* Lane=laneInfo->lane;
	laneGroup* group=laneInfo->group;
	crossSection** section=laneInfo->section;
	int i;

	for(i=0;i<10;i++){
		struct timeval tv;
		gettimeofday(&tv,NULL);
		int64_t currTime=gettime(tv);
		car* newCar=(car*) calloc(1,sizeof(car));
		newCar->name=i;
		newCar->creationTime=currTime;
		Lane->carCreationTimes[i]=currTime;
		newCar->passageTime=0;
		newCar->isItFinal=0;
		if(i==9){
			newCar->isItFinal=1;
		}			
		mq_send(Lane->msgq_id, (char*) newCar, sizeof(car), 0);
		pthread_mutex_lock(&Lane->prioMutex);
		pthread_cond_signal(&Lane->condList);
		pthread_mutex_unlock(&Lane->prioMutex);
		sleep_ms((long)gaussian());
	}
	return NULL;
}

void* laneControl(void* arg){
	t_input* laneInfo=(t_input*)arg;
	lane* Lane=laneInfo->lane;
	laneGroup* group=laneInfo->group;
	crossSection** section=laneInfo->section;	
	calcPrio(Lane,0);
	calcGroupPrio(group);
	car* carPassing;
	long totalTimeToFinish=0;
	do{
		pthread_mutex_lock(&Lane->prioMutex);
		while(checkAllPrios(laneInfo)==0){
			clearPrio(laneInfo);
			
			struct timespec ts;
			createWaittime(&ts);
			int rc =pthread_cond_timedwait(&Lane->condList, &Lane->prioMutex,&ts);
			calcPrio(Lane,0);
			calcGroupPrio(group);	
			if(rc == ETIMEDOUT){
				
				updateAllPrios(laneInfo);
			}
			
					
		}
		pthread_mutex_unlock(&Lane->prioMutex);
		getAllKeys(section,laneInfo->sectionCount,Lane);


		carPassing= malloc(sizeof(car));;
		mq_receive(Lane->msgq_id, carPassing, sizeof(car), NULL);
		
		
		struct timeval tv;
		gettimeofday(&tv,NULL);
		int64_t currTime=gettime(tv);
		printf("Lane %d - Car %d Passed %d\n",Lane->id,carPassing->name,currTime);
		totalTimeToFinish+=abs(currTime-Lane->carCreationTimes[carPassing->name]);
		Lane->carCreationTimes[carPassing->name]=-1;
		
		calcPrio(Lane,1);
		calcGroupPrio(group);		
		sleep_ms(CARPASSINGTIME);
		updateAllPrios(laneInfo);
		
		releaseAllKeys(section,laneInfo->sectionCount,Lane);
	}while(carPassing->isItFinal==0);
	laneInfo->lane->lanePrio=0;
	laneInfo->lane->lastTimeLock=0;
	laneInfo->lane->laneFinished=1;
	int i,groupFinished=1;
	if(group!=NULL){
		for(i=0;i<group->laneCount;i++){
			if(group->lanes[i]->laneFinished==0)
				groupFinished=0;
		}
		group->groupFinished=groupFinished;
	}
		
	calcGroupPrio(group);
	clearPrio(laneInfo);
	updateAllPrios(laneInfo);
	
	releaseAllKeys(section,laneInfo->sectionCount,Lane);
	printf("!!Average Time Passed For Lane %d: %ld\n",Lane->id,totalTimeToFinish/10);
	return NULL;
}



void addLaneGroupToSection(crossSection* CS,laneGroup* LG ){
	CS->crossingLaneGroups[CS->totalLaneGroups++]=LG;
}

void addLaneToSection(crossSection* CS,lane* laneToAdd ){
	CS->crossingLanes[CS->totalLanes++]=laneToAdd;
}

crossSection* createCrossSection(int id){
	crossSection* newCrossSection=(crossSection*) malloc (sizeof(crossSection));

	newCrossSection->id=id;
	newCrossSection->crossingLaneGroups=(laneGroup**) malloc(3*sizeof(laneGroup*));
	newCrossSection->totalLaneGroups=0;

	newCrossSection->crossingLanes=(lane**) malloc(3*sizeof(lane*));
	newCrossSection->totalLanes=0;

	newCrossSection->currentlyCrossing=(lane**) malloc(11*sizeof(lane*));

	newCrossSection->currPrioHolder=-1;

	pthread_mutex_init(&newCrossSection->dataMutex, NULL);	

	return newCrossSection;
}

void destroyCrossSection(crossSection* cSToDestroy){	
	free(cSToDestroy->crossingLaneGroups);
	free(cSToDestroy->crossingLanes);
	free(cSToDestroy->currentlyCrossing);
	free(cSToDestroy);
}

void addLaneToGroup(laneGroup* group, lane* laneToAdd){
	group->lanes[group->laneCount++]=laneToAdd;	
}

laneGroup* createLaneGroup(int id){
	laneGroup* newGroup=(laneGroup*) malloc(sizeof(laneGroup));
	newGroup->id=id;
	newGroup->lanes=(lane**)malloc(3*sizeof(lane*));
	newGroup->laneCount=0;
	newGroup->groupPrio=0;
	return newGroup;
}

void destroyLaneGroup(laneGroup* groupToDestroy){
	free(groupToDestroy->lanes);
	free(groupToDestroy);
}

void addCrossingLane(lane* src, lane* dst){
	src->crossingLanes[src->crossingCount++]=dst;
}

lane* createLane(int id){
	int i;
	lane *newLane=(lane*)calloc(1,sizeof(lane));
	newLane->id=id;
	newLane->lanePrio=0;	
	newLane->lastTimeLock=0;
	newLane->laneFinished=0;
	for(i=0;i<10;i++){
		newLane->carCreationTimes[i]=-1;
	}


	
  	pthread_cond_init (&newLane->condList, NULL);	
	pthread_mutex_init(&newLane->prioMutex, NULL);

	struct mq_attr attr;
	attr.mq_flags   = 1;          // blocking mode - Non-blocking
	attr.mq_maxmsg  = 10;         // <-- max number of cars
	attr.mq_msgsize = sizeof(car);       // message is just the car structure	
	char MSGQOBJ_NAME[100]="/";
	sprintf(MSGQOBJ_NAME, "/%d_msq", id);
	mqd_t msgq_id = mq_open(MSGQOBJ_NAME, O_RDWR | O_CREAT | O_EXCL, S_IRWXU | S_IRWXG, &attr);
	if (msgq_id == (mqd_t)-1) {  // If mq_open fails, print an error and exit
        perror("In mq_open()");
        exit(1);
    }
	newLane->msgq_id=msgq_id;
	newLane->crossingLanes=(lane**)malloc(10*sizeof(lane*));
	newLane->crossingCount=0;
	return newLane;
}

void destroyLane(lane* laneToDestroy){
	free(laneToDestroy->crossingLanes);

	mq_close(laneToDestroy->msgq_id);
	char MSGQOBJ_NAME[100]="/";
	sprintf(MSGQOBJ_NAME, "/%d_msq", laneToDestroy->id);
	mq_unlink(MSGQOBJ_NAME);
	free(laneToDestroy);
}

void ResetProgram(){
	int i;
	for(i=0;i<10;i++){
		char MSGQOBJ_NAME[100]="/";
		sprintf(MSGQOBJ_NAME, "/%d_msq",i+1);
		mq_unlink(MSGQOBJ_NAME);
	}
	for(i=0;i<10;i++){
		char SEM_NAME[100];
		sprintf(SEM_NAME, "/%d_sem", i);
		sem_unlink(SEM_NAME);
	}
}

int main(){
	int i,j;
	lane** lanes=(lane**) malloc(11*sizeof(lane*));
	laneGroup** groups=(laneGroup**)malloc(laneGroupNumber*sizeof(laneGroup*));
	crossSection** sections=(crossSection**)malloc(crossSectionNumber*sizeof(crossSection*));
	t_input** laneInfos=(t_input**)malloc(11*sizeof(t_input*));
	srand(time(NULL));
	struct timeval tv;
	gettimeofday(&tv,NULL);
	start_time=(int64_t)(tv.tv_sec) * 1000 + (tv.tv_usec / 1000);
	//srand(1);
	ResetProgram();

	for(i=1;i<11;i++){
		lanes[i]=createLane(i);
		laneInfos[i]=(t_input*)calloc(1,sizeof(t_input));
		laneInfos[i]->lane=lanes[i];
		laneInfos[i]->group=NULL;
		laneInfos[i]->sectionCount=0;
	}
	for(i=1;i<11;i++){
		j=0;
		while(crossedLanes[i][j]!=0){
			addCrossingLane(lanes[i],lanes[crossedLanes[i][j]]);
			j++;
		}
	}
	for(i=0;i<laneGroupNumber;i++){
		groups[i]=createLaneGroup(i+1);
		j=0;
		while((j<2)&&(laneGroups[i][j]!=0)){
			addLaneToGroup(groups[i],lanes[laneGroups[i][j]]);
			laneInfos[laneGroups[i][j]]->group=groups[i];
			j++;
		}
	}
	for(i=0;i<crossSectionNumber;i++){
		sections[i]=createCrossSection(i);
		j=0;
		while((j<3)&&(crossSections[i][j]!=0)){
			if(crossSections[i][j]>0){
				addLaneGroupToSection(sections[i],groups[crossSections[i][j]-1]);
				int k;
				for(k=0;k<groups[crossSections[i][j]-1]->laneCount;k++)
					laneInfos[groups[crossSections[i][j]-1]->lanes[k]->id]->section[laneInfos[groups[crossSections[i][j]-1]->lanes[k]->id]->sectionCount++]=
					sections[i];
			}else{
				addLaneToSection(sections[i],lanes[-1*crossSections[i][j]]);
				laneInfos[-1*crossSections[i][j]]->section[laneInfos[-1*crossSections[i][j]]->sectionCount++]=sections[i];
			}			
			j++;
		}
	}

	pthread_t threads[20];
  	pthread_attr_t attr;
	pthread_attr_init(&attr);
  	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

	for(i=1;i<11;i++){
		pthread_create(&threads[i-1], &attr, carProducer, laneInfos[i]);
		pthread_create(&threads[i+9], &attr, laneControl, laneInfos[i]);
	}

	for(i=0;i<20;i++)
		pthread_join(threads[i], NULL);

	// pthread_create(&threads[0], &attr, carProducer, laneInfos[1]);
	// pthread_create(&threads[1], &attr, carProducer, laneInfos[2]);
	// pthread_create(&threads[4], &attr, carProducer, laneInfos[5]);
	// pthread_create(&threads[5], &attr, carProducer, laneInfos[6]);
	// pthread_create(&threads[9], &attr, carProducer, laneInfos[10]);

  	// pthread_create(&threads[10], &attr, laneControl, laneInfos[1]);
	// pthread_create(&threads[11], &attr, laneControl, laneInfos[2]);
	// pthread_create(&threads[14], &attr, laneControl, laneInfos[5]);
	// pthread_create(&threads[15], &attr, laneControl, laneInfos[6]);
	// pthread_create(&threads[19], &attr, laneControl, laneInfos[10]);
	

	// pthread_join(threads[0], NULL);
	// pthread_join(threads[1], NULL);
	// pthread_join(threads[4], NULL);
	// pthread_join(threads[5], NULL);
	// pthread_join(threads[9], NULL);
	// pthread_join(threads[10], NULL);
	// pthread_join(threads[11], NULL);
	// pthread_join(threads[14], NULL);
	// pthread_join(threads[15], NULL);
	// pthread_join(threads[19], NULL);

	for(i=0;i<crossSectionNumber;i++){
		destroyCrossSection(sections[i]);
	}

	for(i=0;i<laneGroupNumber;i++){
		destroyLaneGroup(groups[i]);		
	}
	
	for(i=1;i<11;i++){
		destroyLane(lanes[i]);
	}
	pthread_exit(0);
}
