#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <iostream>

using namespace std;

typedef struct{
	double **PT;
	double **PT_new;
	double  *Ysec;
	double	*Yslp;
	double   angle;
	double   det_cw;
	double   deter[2];
	int	     SIZE;
	int		 cw_ccw;
}FIELD;

typedef struct{
	double **PT;
	double **PT_new;
	double  *Ysec;
	double	*Yslp;
	double	 angle;
	double **cntct;
	double   det_cw;
	double   deter[2];
	double   dst[2];
	int		 SIZE;
	int      cw_ccw;
	int      cntct_cnt;
}OBSTACLE;

typedef struct{
	double	RANGE;		// m
	double	ACC;		// m/s^2
	double	VEL;			// m/s
	double	STOP_TIME;	// sec
	int		TOT_NUM;   // # of helicopters
}HELISPEC;

typedef struct{
	double **WAY_PTS;
	int		 cnt;
	int		 num;	// 몇 번째 RUAV인가? 전체 경지에 대한 경로를 구할 경우 0.
	double	 cost_time;
}WAYPOINT;


// SUB-FUNCTION :: DETERMINATION FUNCTIONS
static int determine_inside_line(double, double, double, double, double, double);
static FIELD determine_cw_ccw(FIELD);
static OBSTACLE determine_cw_ccw(OBSTACLE);

// SUB-FUNCTION :: CALCULATE FUNCTIONS.
static double calculate_distance(double, double, double, double);
static FIELD calculate_new_BP(FIELD,HELISPEC);
static OBSTACLE calculate_new_BP(OBSTACLE,HELISPEC);
static WAYPOINT calculate_WP(FIELD, WAYPOINT, HELISPEC);
static double calculate_costtime(double, double, double, double, double, HELISPEC);

// SUB_FUNCTION :: ASSIGN FUNCTIONS.
static double** assign_array(int, int);
static double* assign_array(int);
static WAYPOINT* assign_WAYPOINT(int);
static double** extend_array(double***, int, int, int);

// SUB-FUNCTION :: DELETE FUNCTIONS.
static void delete_array(double **, int);
static void delete_array(double *);

// MAIN FUNCTION.
void main()
{
	// VALUABLES
	FIELD			  FLD;
	OBSTACLE		  OBS_1;
	WAYPOINT		 *WPTS;
	HELISPEC		  HLI;

	HLI.ACC = 1.0;
	HLI.RANGE = 7.5;
	HLI.STOP_TIME = 0.0;
	HLI.VEL = 5.0;
	HLI.TOT_NUM = 8;

	WPTS = assign_WAYPOINT(HLI.TOT_NUM+1);
	for (int i=0; i<HLI.TOT_NUM+1; i++)
		WPTS[i].num = i;

	//// FIELD SETTING //////

	printf("How many boundary points do you need? : ");
	cin >> FLD.SIZE;

	FLD.PT = assign_array(FLD.SIZE+1,2);

	printf("Boundary points are being set...\n");

	// TEST FIELD BOUNDARY POINTS.

	FLD.PT[0][0] = -105.5300; FLD.PT[0][1] = -131.5789;
	FLD.PT[1][0] = -110.1382; FLD.PT[1][1] = 84.7953;
	FLD.PT[2][0] = 37.3272;   FLD.PT[2][1] = 81.2865;
	FLD.PT[3][0] = 56.6820;   FLD.PT[3][1] = -124.5614;
	FLD.PT[FLD.SIZE][0] = FLD.PT[0][0]; FLD.PT[FLD.SIZE][1] = FLD.PT[0][1];
	printf("\n");
	printf("Boundary points are set!!\n");

	///// OBSTACLE SETTING //////

	printf("How many boundary points of obstacles do you need? : ");
	cin >> OBS_1.SIZE;

	OBS_1.PT = assign_array(OBS_1.SIZE+1,2);

	printf("Boundary points are being set...\n");

	// TEST OBSTACLE BOUNDARY POINTS.

	OBS_1.PT[0][0] = -52.9954; OBS_1.PT[0][1] = -42.6901;
	OBS_1.PT[1][0] = -40.0922; OBS_1.PT[1][1] = -26.3158;
	OBS_1.PT[2][0] = -22.5806; OBS_1.PT[2][1] = -29.8246;
	OBS_1.PT[3][0] = -26.2673; OBS_1.PT[3][1] = -71.9298;
	OBS_1.PT[OBS_1.SIZE][0] = OBS_1.PT[0][0]; OBS_1.PT[OBS_1.SIZE][1] = OBS_1.PT[0][1];
	printf("\n");
	printf("Boundary points are set!!\n");

	///// DETERMINE CCW / CW //////

	FLD = determine_cw_ccw(FLD);
	OBS_1 = determine_cw_ccw(OBS_1);

	printf("Direction is determined!!\n");

    ////// FIND NEW BOUNDARY POINT ////

	FLD = calculate_new_BP(FLD,HLI);
	OBS_1 = calculate_new_BP(OBS_1,HLI);
	printf("New Boundary Point is setted!!\n");
	printf("New Obstacle Boundary Point is setted!!\n");

	////////// CREATE MAIN PATH ///////////
	
	WPTS[0] = calculate_WP(FLD, WPTS[0], HLI);
	printf("Whole Waypoints are set!!!\n");

	////////// CREATE SUB PATH ////////////

	for (int i=1; i< HLI.TOT_NUM+1; i++)
	{
		WPTS[i].cost_time = WPTS[0].cost_time;
		WPTS[i] = calculate_WP(FLD,WPTS[i],HLI);
	}

	////////// EXPORT WAYPOINTS ///////////

	FILE* outxt_1;
	fopen_s(&outxt_1,"result_wp_1.dat","wt");
	for (int i=0; i<WPTS[1].cnt; i++)
		fprintf(outxt_1,"%f\t%f\n",WPTS[1].WAY_PTS[i][0], WPTS[1].WAY_PTS[i][1]);
	fclose(outxt_1);

	FILE* outxt_2;
	fopen_s(&outxt_2,"result_wp_2.dat","wt");
	for (int i=0; i<WPTS[2].cnt; i++)
		fprintf(outxt_2,"%f\t%f\n",WPTS[2].WAY_PTS[i][0], WPTS[2].WAY_PTS[i][1]);
	fclose(outxt_2);

	for (int i=0; i< HLI.TOT_NUM+1; i++)
	printf("%f\n",WPTS[i].cost_time);

	////////// DELETE VALUVALES ///////////
	delete_array(FLD.PT,FLD.SIZE+1);
	delete_array(FLD.PT_new,FLD.SIZE+1);
	delete_array(FLD.Ysec);
	delete_array(FLD.Yslp);
	for (int i=0; i<HLI.TOT_NUM+1;i++)
		delete_array(WPTS[i].WAY_PTS,WPTS[i].cnt);
}


// DETERMINATION FUNCTIONS.
FIELD determine_cw_ccw(FIELD FLD)
{
	FLD.angle = atan2(FLD.PT[1][1]-FLD.PT[0][1],FLD.PT[1][0]-FLD.PT[0][0]);
	FLD.deter[0] = FLD.PT[2][0]-FLD.PT[0][0]; FLD.deter[1] = FLD.PT[2][1]-FLD.PT[0][1];
	FLD.det_cw = FLD.deter[0]*sin(-FLD.angle)+FLD.deter[1]*cos(-FLD.angle);
	if (FLD.det_cw <= 0)
		FLD.cw_ccw = 1; // CW
	else
		FLD.cw_ccw = 0; // CCW
	return FLD;
}
OBSTACLE determine_cw_ccw(OBSTACLE OBS)
{
	OBS.angle = atan2(OBS.PT[1][1]-OBS.PT[0][1],OBS.PT[1][0]-OBS.PT[0][0]);
	OBS.deter[0] = OBS.PT[2][0]-OBS.PT[0][0]; OBS.deter[1] = OBS.PT[2][1]-OBS.PT[0][1];
	OBS.det_cw = OBS.deter[0]*sin(-OBS.angle)+OBS.deter[1]*cos(-OBS.angle);
	if (OBS.det_cw <= 0)
		OBS.cw_ccw = 1; // CW
	else
		OBS.cw_ccw = 0; // CCW
	return OBS;
}
int determine_inside_line(double pt_x,double pt_y, double BPT1_x, double BPT1_y, double BPT2_x, double BPT2_y)
{
	int det = 0;
	if ((pt_x >= min(BPT1_x,BPT2_x)) && (pt_x <= max(BPT1_x,BPT2_x)))
		det++;
	if ((pt_y >= min(BPT1_y,BPT2_y)) && (pt_y <= max(BPT1_y,BPT2_y)))
		det++;
	if (det == 2)
		return 1;
	else
		return 0;
}

// CALCULATE FUNCTIONS.
FIELD calculate_new_BP(FIELD FLD, HELISPEC HLI)
{
	FLD.Ysec=assign_array(FLD.SIZE+1);
	FLD.Yslp=assign_array(FLD.SIZE+1);

	for (int i = 0; i<FLD.SIZE; i++)
	{
		if (abs(FLD.PT[i+1][0] - FLD.PT[i][0]) < 0.1)
			FLD.PT[i+1][0] = FLD.PT[i+1][0] + 0.01;
		FLD.Yslp[i] = (FLD.PT[i+1][1]-FLD.PT[i][1])/(FLD.PT[i+1][0]-FLD.PT[i][0]);
		FLD.Ysec[i] = -FLD.Yslp[i]*FLD.PT[i][0]+FLD.PT[i][1];
		if (FLD.cw_ccw == 0)
			FLD.Ysec[i] = FLD.Ysec[i] + HLI.RANGE/(2*cos(atan2(FLD.PT[i+1][1]-FLD.PT[i][1],FLD.PT[i+1][0]-FLD.PT[i][0])));
		else if (FLD.cw_ccw == 1)	
			FLD.Ysec[i] = FLD.Ysec[i] - HLI.RANGE/(2*cos(atan2(FLD.PT[i+1][1]-FLD.PT[i][1],FLD.PT[i+1][0]-FLD.PT[i][0])));
	}

	FLD.Yslp[FLD.SIZE] = FLD.Yslp[0];
	FLD.Ysec[FLD.SIZE] = FLD.Ysec[0];

	FLD.PT_new = assign_array(FLD.SIZE+1,2);

	for (int i=0; i<FLD.SIZE; i++)
	{
		FLD.PT_new[i][0] = (FLD.Ysec[i]-FLD.Ysec[i+1])/(-FLD.Yslp[i]+FLD.Yslp[i+1]);
		FLD.PT_new[i][1] = (FLD.Yslp[i+1]*FLD.Ysec[i]-FLD.Yslp[i]*FLD.Ysec[i+1])/(-FLD.Yslp[i]+FLD.Yslp[i+1]);
	}
	FLD.PT_new[FLD.SIZE][0] = FLD.PT_new[0][0]; FLD.PT_new[FLD.SIZE][1] = FLD.PT_new[0][1];

	return FLD;
}
OBSTACLE calculate_new_BP(OBSTACLE OBS, HELISPEC HLI)
{
	OBS.Ysec=assign_array(OBS.SIZE+1);
	OBS.Yslp=assign_array(OBS.SIZE+1);

	for (int i = 0; i<OBS.SIZE; i++)
	{
		if (abs(OBS.PT[i+1][0] - OBS.PT[i][0]) < 0.1)
			OBS.PT[i+1][0] = OBS.PT[i+1][0] + 0.01;
		OBS.Yslp[i] = (OBS.PT[i+1][1]-OBS.PT[i][1])/(OBS.PT[i+1][0]-OBS.PT[i][0]);
		OBS.Ysec[i] = -OBS.Yslp[i]*OBS.PT[i][0]+OBS.PT[i][1];
		if (OBS.cw_ccw == 0)
			OBS.Ysec[i] = OBS.Ysec[i] - HLI.RANGE/(2*cos(atan2(OBS.PT[i+1][1]-OBS.PT[i][1],OBS.PT[i+1][0]-OBS.PT[i][0])));
		else if (OBS.cw_ccw == 1)	
			OBS.Ysec[i] = OBS.Ysec[i] + HLI.RANGE/(2*cos(atan2(OBS.PT[i+1][1]-OBS.PT[i][1],OBS.PT[i+1][0]-OBS.PT[i][0])));
	}

	OBS.Yslp[OBS.SIZE] = OBS.Yslp[0];
	OBS.Ysec[OBS.SIZE] = OBS.Ysec[0];

	OBS.PT_new = assign_array(OBS.SIZE+1,2);

	for (int i=0; i<OBS.SIZE; i++)
	{
		OBS.PT_new[i][0] = (OBS.Ysec[i]-OBS.Ysec[i+1])/(-OBS.Yslp[i]+OBS.Yslp[i+1]);
		OBS.PT_new[i][1] = (OBS.Yslp[i+1]*OBS.Ysec[i]-OBS.Yslp[i]*OBS.Ysec[i+1])/(-OBS.Yslp[i]+OBS.Yslp[i+1]);
	}
	OBS.PT_new[OBS.SIZE][0] = OBS.PT_new[0][0]; OBS.PT_new[OBS.SIZE][1] = OBS.PT_new[0][1];

	return OBS;
}
double calculate_distance(double pt1_x, double pt1_y, double pt2_x, double pt2_y)
{
	return sqrt(pow(pt1_x-pt2_x,2)+pow(pt1_y-pt2_y,2));
}
WAYPOINT calculate_WP(FIELD FLD, WAYPOINT WPT, HELISPEC HLI)
{
	WPT.cnt = 1;
	int tmp_cnt = 0;
	int flag = 0;
	double tmp[2];
	double tot_time = 0;
	double tmp_Ysec = FLD.Ysec[0];
	WPT.WAY_PTS = assign_array(2,2);
	if (WPT.num <= 1)
	{
		WPT.cnt = 3;
		WPT.WAY_PTS[0][0] = FLD.PT_new[FLD.SIZE-1][0]; WPT.WAY_PTS[0][1] = FLD.PT_new[FLD.SIZE-1][1];
		WPT.WAY_PTS[1][0] = FLD.PT_new[0][0];		  WPT.WAY_PTS[1][1] = FLD.PT_new[0][1];
		tot_time = calculate_costtime(WPT.WAY_PTS[0][0],WPT.WAY_PTS[0][1],WPT.WAY_PTS[1][0],WPT.WAY_PTS[1][1],tot_time,HLI);
	}

	while (1)
	{
		if (FLD.cw_ccw == 0)
			FLD.Ysec[0] = FLD.Ysec[0] + HLI.RANGE/cos(atan2(FLD.PT[1][1]-FLD.PT[0][1],FLD.PT[1][0]-FLD.PT[0][0]));
		else if (FLD.cw_ccw == 1)
			FLD.Ysec[0] = FLD.Ysec[0] - HLI.RANGE/cos(atan2(FLD.PT[1][1]-FLD.PT[0][1],FLD.PT[1][0]-FLD.PT[0][0]));
		for (int j=0; j< FLD.SIZE-1; j++)
		{
			tmp[0] = (FLD.Ysec[0]-FLD.Ysec[j+1])/(-FLD.Yslp[0]+FLD.Yslp[j+1]);
			tmp[1] = (FLD.Yslp[j+1]*FLD.Ysec[0]-FLD.Yslp[0]*FLD.Ysec[j+1])/(-FLD.Yslp[0]+FLD.Yslp[j+1]);
			if (determine_inside_line(tmp[0],tmp[1],FLD.PT_new[j][0],FLD.PT_new[j][1],FLD.PT_new[j+1][0],FLD.PT_new[j+1][1]))
			{
				WPT.WAY_PTS=extend_array(&(WPT.WAY_PTS),WPT.cnt-1,WPT.cnt+1,2);
				WPT.WAY_PTS[WPT.cnt-1][0] = tmp[0];
				WPT.WAY_PTS[WPT.cnt-1][1] = tmp[1];
				WPT.cnt++;
				tmp_cnt++;
				flag = 1;
			}
			if ((tmp_cnt == 2) && (WPT.cnt > 3))
			{
				if ((calculate_distance(WPT.WAY_PTS[WPT.cnt-4][0],WPT.WAY_PTS[WPT.cnt-4][1],WPT.WAY_PTS[WPT.cnt-3][0], WPT.WAY_PTS[WPT.cnt-3][1])) >= (calculate_distance(WPT.WAY_PTS[WPT.cnt-4][0],WPT.WAY_PTS[WPT.cnt-4][1],WPT.WAY_PTS[WPT.cnt-2][0], WPT.WAY_PTS[WPT.cnt-2][1])))
				{
					tmp[0] = WPT.WAY_PTS[WPT.cnt-2][0];
					tmp[1] = WPT.WAY_PTS[WPT.cnt-2][1];
					WPT.WAY_PTS[WPT.cnt-2][0] = WPT.WAY_PTS[WPT.cnt-3][0];
					WPT.WAY_PTS[WPT.cnt-2][1] = WPT.WAY_PTS[WPT.cnt-3][1];
					WPT.WAY_PTS[WPT.cnt-3][0] = tmp[0];
					WPT.WAY_PTS[WPT.cnt-3][1] = tmp[1];
				}
			}
			else if ((tmp_cnt == 2) && (WPT.cnt == 3))
			{
				if (abs(FLD.PT_new[0][1]-FLD.PT_new[FLD.SIZE-1][1])/(FLD.PT_new[0][0]-FLD.PT_new[FLD.SIZE-1][0])+(WPT.WAY_PTS[1][1]-WPT.WAY_PTS[0][1])/(WPT.WAY_PTS[1][0]-WPT.WAY_PTS[0][0]) < 0.1)
				{
					tmp[0] = WPT.WAY_PTS[1][0];
					tmp[1] = WPT.WAY_PTS[1][1];
					WPT.WAY_PTS[1][0] = WPT.WAY_PTS[0][0];
					WPT.WAY_PTS[1][1] = WPT.WAY_PTS[0][1];
					WPT.WAY_PTS[0][0] = tmp[0];
					WPT.WAY_PTS[0][1] = tmp[1];
				}
			}
		}

		if (flag == 0)
		{
			WPT.cost_time = tot_time;
			break;
		}
		if (WPT.cnt > 3)
		{
			for (int i=4; i>2; i--)
				tot_time = calculate_costtime(WPT.WAY_PTS[WPT.cnt-i][0],WPT.WAY_PTS[WPT.cnt-i][1],WPT.WAY_PTS[WPT.cnt-i+1][0],WPT.WAY_PTS[WPT.cnt-i+1][1],tot_time,HLI);
		}
		else if (WPT.cnt == 3)
			tot_time = calculate_costtime(WPT.WAY_PTS[0][0],WPT.WAY_PTS[0][1],WPT.WAY_PTS[1][0],WPT.WAY_PTS[1][1],tot_time,HLI);
	
		if ((WPT.num != 0) && (WPT.cost_time < tot_time))
		{
			WPT.cost_time = tot_time;
			break;
		}
		flag = 0;
		tmp_cnt = 0;
	}
	if (WPT.num == 0)
	{
		FLD.Ysec[0] = tmp_Ysec;
		WPT.cost_time = (double)(tot_time / HLI.TOT_NUM);
	}
	WPT.cnt--;
	return WPT;
}
double calculate_costtime(double a_1, double a_2, double b_1, double b_2, double tot_time, HELISPEC HLI)
{
	if (calculate_distance(a_1,a_2,b_1,b_2) < pow(HLI.VEL,2)/HLI.ACC)
		tot_time = tot_time + sqrt(calculate_distance(a_1,a_2,b_1,b_2)/HLI.ACC);
	else
		tot_time = tot_time + sqrt(pow(HLI.VEL,2)/HLI.ACC)+(calculate_distance(a_1,a_2,b_1,b_2)-pow(HLI.VEL,2)/HLI.ACC)/HLI.VEL;
	return tot_time;
}

// ASSIGN FUNCTIONS. 
double** assign_array(int raw, int col)
{
	double **second_array = (double**)malloc(raw*sizeof(double));
	for (int i=0; i<raw; i++)
		second_array[i] = (double*)malloc(col*sizeof(double));
	return second_array;
}
double* assign_array(int col)
{
	double *first_array = (double*)malloc(col*sizeof(double));
	return first_array;
}
WAYPOINT* assign_WAYPOINT(int col)
{
	WAYPOINT *WPTS = (WAYPOINT*)malloc(col*sizeof(WAYPOINT));
	return WPTS;
}
double** extend_array(double*** array_orgin, int raw_old, int raw_new, int col_new)
{
	*array_orgin = (double**)realloc(*array_orgin, raw_new*sizeof(double*));
	for (int i=0; i<raw_new;i++)
	{
		if (raw_old > i)
			(*array_orgin)[i] = (double*)realloc((*array_orgin)[i],col_new*sizeof(double));
		else
			(*array_orgin)[i] = (double*)malloc(col_new*sizeof(double));
	}
	return *array_orgin;
}

// DELETE FUNCTIONS.
void delete_array(double **second_array,int raw)
{
	for (int i=0; i< raw; i++)
		delete[] second_array[i];
	delete[] second_array;
}
void delete_array(double *first_array)
{
	delete[] first_array;
}
