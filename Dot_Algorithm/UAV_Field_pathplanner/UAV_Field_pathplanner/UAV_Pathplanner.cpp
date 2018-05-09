#include <math.h>
#include <stdio.h>
#include <iostream>

using namespace std;

static double MAX_DATA(double**, int, int);
static double MIN_DATA(double**, int, int);
static double MAX(double, double);
static double dist(double, double, double,double);
static double MAX_DIST(double*, int);
static double MAX_APT(double**, int);
static double dist_line(double, double, double, double, double);
static double ave_pt(double**, int, int);
static void delete_mem(double**, int);
static void delete_mem(double*);

void main()
{
const double	range = 7.5;		// meter
const double	div_range = 10;
const double	PI = 3.141592;
double	**P;

double	X;
double	Y;
double	**PT;	// Grid Points
double	**PT_TMP;
int		size;

double	alpha;
double	*theta;
double	**PT_BY_BP;  // transformed points by Boundary Points
double	**ABS_BY_BP; // Abstracted points by Boundaary Points
double	**ABS_BY_BP_TMP;
double	**WP_TMP;	// waypoints which are temperary values.
double	**WP;		// waypoints that we need.

	printf("How many boundary points do you need? : ");
	scanf("%d",&size);
	
	P = new double *[size+1];
	for (int i=0; i<size+1;i++)
	{
		P[i] = new double[2];
	}
	
	 printf("Boundary points are being set...\n");

	// Boudary Point Definition

	for (int i=0; i<size;i++)
	{
		for (int j=0; j<2; j++)
		{
			cout << "P[" << i << "][" << j << "] = ";
			cin >> P[i][j];
		}
	}

	printf("\n"); // insert the line-space to be noticed quickly.

	P[size][0] = P[0][0]; P[size][1] = P[0][1];

	printf("Boundary points are set!!\n");

	printf("Basis of field is being set...\n");

	// Define the main BASIS.
	alpha = atan2(P[1][1]-P[0][1],P[1][0]-P[0][0]);

	// Determine whether clockwise or not.
	double cw_agl = atan2(P[1][1]-P[0][1],P[1][0]-P[0][0]);
	double det_cw[2];
	det_cw[0] = P[2][0]-P[0][0]; det_cw[1] = P[2][1]-P[0][1];
	double jdg_cw = det_cw[0]*sin(-cw_agl)+det_cw[1]*cos(-cw_agl);
	int cw_ccw;
	if (jdg_cw <= 0)
		cw_ccw = 1; // CW
	else
		cw_ccw = 0; // CCW

	printf("Basis of field is set!!\n");

	printf("Grid points are being set...\n");

	// Grid Generation
	double range_gp = range/div_range;
	int GRID = (int)(MAX_APT(P,size)*2/range_gp);
	
	PT = new double *[(int)(GRID*GRID)];
	PT_TMP = new double *[(int)(GRID*GRID)];
	for (int i=0; i<(int)(GRID*GRID); i++)
	{
		PT[i] = new double[2];
		PT_TMP[i] = new double[2];
	}

	double *P_AVE = new double[2];
	P_AVE[0] = ave_pt(P,size,1);
	P_AVE[1] = ave_pt(P,size,2);

	for (int j=0; j< GRID; j++)
	{
		for (int k=0; k< GRID; k++)
		{
			PT_TMP[k+j*GRID][0] = range_gp*(j-GRID/2);
			PT_TMP[k+j*GRID][1] = range_gp*(k-GRID/2);
			PT[k+j*GRID][0] = sqrt(pow(range_gp*(j-GRID/2),2)+pow(range_gp*(k-GRID/2),2))*cos(alpha+atan2(range_gp*(k-GRID/2),range_gp*(j-GRID/2)));
			PT[k+j*GRID][1] = sqrt(pow(range_gp*(j-GRID/2),2)+pow(range_gp*(k-GRID/2),2))*sin(alpha+atan2(range_gp*(k-GRID/2),range_gp*(j-GRID/2)));
		}
	}

	printf("Grid points are set!!\n");

	// Check GRID : GRID-FIELD MATCHING.
	double *chk_dist = new double[2];
	chk_dist[0] = ave_pt(PT,(int)(GRID*GRID),1)-P_AVE[0];
	chk_dist[1] = ave_pt(PT,(int)(GRID*GRID),2)-P_AVE[1];
	for (int j=0; j<(int)(GRID*GRID); j++)
	{
		PT[j][0] = PT[j][0]-chk_dist[0];
		PT[j][1] = PT[j][1]-chk_dist[1];
	}

	printf("Matching the grid points is complecated!!\n");

	printf("Boundary determinent is in progress..\n");

	// Boundary Determinent Making
	theta = new double[size];
	PT_BY_BP = new double*[(int)(GRID*GRID)];
	for (int i=0; i<(int)(GRID*GRID); i++)
	{
		PT_BY_BP[i] = new double[2*size];
	}
	for (int iter=0; iter<size; iter++)
	{
		for (int i=0; i<(int)(GRID*GRID);i++)
		{
			theta[iter] = atan2(P[iter+1][1]-P[iter][1],P[iter+1][0]-P[iter][0]);
			PT_BY_BP[i][iter*2] = PT[i][0]-P[iter][0];
			PT_BY_BP[i][iter*2+1] = PT[i][1]-P[iter][1];
			PT_BY_BP[i][iter*2+1] = PT_BY_BP[i][iter*2]*sin(-theta[iter])+PT_BY_BP[i][iter*2+1]*cos(-theta[iter]);
		}
	}

	delete_mem(P, size+1);

	// Boundary Determinent
	int cnt = 0;
	if (cw_ccw == 1) // CW
	{
		for (int i=0; i<(int)(GRID*GRID); i++)
		{
			int det = 0;
			for (int iter=0; iter<size; iter++)
			{
				if (PT_BY_BP[i][iter*2+1] > -range/2)
				{
					det = 1;
				}
			}
			if (det == 0)
			{
				cnt++;
			}
		}
	}
	else if (cw_ccw == 0) // CCW
	{
		for (int i=0; i<(int)(GRID*GRID); i++)
		{
			int det = 0;
			for (int iter=0; iter<size; iter++)
			{
				if (PT_BY_BP[i][iter*2+1] < range/2)
				{
					det = 1;
				}
			}
			if (det == 0)
			{
				cnt++;
			}
		}
	}

	printf("Real points are being declared...\n");

	//Declare REAL POINTS.
	ABS_BY_BP = new double*[cnt];
	ABS_BY_BP_TMP = new double*[cnt];
	for (int i=0; i<cnt; i++)
	{
		ABS_BY_BP[i] = new double[2];
		ABS_BY_BP_TMP[i] = new double[2];
	}

	int index = 0;

	if (cw_ccw == 1) // CW
	{
		for (int i=0; i<(int)(GRID*GRID); i++)
		{
			int det = 0;
			for (int iter=0; iter<size; iter++)
			{
				if (PT_BY_BP[i][iter*2+1] > -range/2)
				{
					det = 1;
				}
			}
			if (det == 0)
				{
				ABS_BY_BP[index][0] = PT[i][0];
				ABS_BY_BP_TMP[index][0] = PT_TMP[i][0];
				ABS_BY_BP[index][1] = PT[i][1];
				ABS_BY_BP_TMP[index][1] = PT_TMP[i][1];
				index++;
			}
		}
	}
	else if (cw_ccw == 0) // CCW
	{
		for (int i=0; i<(int)(GRID*GRID); i++)
		{
			int det = 0;
			for (int iter=0; iter<size; iter++)
			{
				if (PT_BY_BP[i][iter*2+1] < range/2)
				{
					det = 1;
				}
			}
			if (det == 0)
				{
				ABS_BY_BP[index][0] = PT[i][0];
				ABS_BY_BP_TMP[index][0] = PT_TMP[i][0];
				ABS_BY_BP[index][1] = PT[i][1];
				ABS_BY_BP_TMP[index][1] = PT_TMP[i][1];
				index++;
			}
		}
	}

	// Delete Valuables
	delete_mem(PT, (int)(GRID*GRID));
	delete_mem(PT_TMP, (int)(GRID*GRID));
	delete_mem(PT_BY_BP, (int)(GRID*GRID));
	delete_mem(theta);

	printf("Real points are specified!!\n");

	printf("Waypoints are being set...\n");
	// Waypoints Setting : Path Generation
	double ABS_BY_BP_TMP_min_y = MIN_DATA(ABS_BY_BP_TMP,cnt,2);
	double ABS_BY_BP_TMP_max_y = MAX_DATA(ABS_BY_BP_TMP,cnt,2);
	double REF;
	int	flag = 1;
	int det = 0;
	int num = 0;
	int cnt_wp = 0; // count(size) of waypoint valuable for the dynamic assignment.

	printf(" - Find the size of waypoint valuable.\n");

	// Find the size of waypoint valuable

	while (1)
	{
		// Determine the Starting Point.
		if (flag == 1)
		{
			if (cw_ccw == 0)
				REF = ABS_BY_BP_TMP_min_y;
			else if (cw_ccw == 1)
				REF = ABS_BY_BP_TMP_max_y;
		}

		if (flag > 1)
		{
			if (cw_ccw == 0)
				REF = REF + range;
			else if (cw_ccw == 1)
				REF = REF - range;
		}

		int index = 0;
		for (int i=0; i < cnt; i++)
		{
			if (ABS_BY_BP_TMP[i][1] == REF)
			{
				det = 1;
			}
		}

		if (det == 0)
			break;
		
		if (index == 1)
		{
			cnt_wp++;
			flag++;
			det = 0;
		}
		else
		{
			cnt_wp = cnt_wp+2;
			flag++;
			det = 0;
		}
	}
	
	printf(" - Declare the waypoint valuable.\n");

	//Declare the waypoint valuable
	WP = new double *[cnt_wp];
	for (int i=0; i<cnt_wp; i++)
	{
		WP[i] = new double[2];
	}


	flag = 1;
	det = 0;
	ABS_BY_BP_TMP_min_y = MIN_DATA(ABS_BY_BP_TMP,cnt,2);
	ABS_BY_BP_TMP_max_y = MAX_DATA(ABS_BY_BP_TMP,cnt,2);

	while (1)
	{
		// Determine the Starting Point.
		if (flag == 1)
		{
			if (cw_ccw == 0)
				REF = ABS_BY_BP_TMP_min_y;
			else if (cw_ccw == 1)
				REF = ABS_BY_BP_TMP_max_y;
		}

		if (flag > 1)
		{
			if (cw_ccw == 0)
				REF = REF + range;
			else if (cw_ccw == 1)
				REF = REF - range;
		}

		int index = 0;
		for (int i=0; i < cnt; i++)
		{
			if (ABS_BY_BP_TMP[i][1] == REF)
			{
				index++;
				det = 1;
			}
		}
		if (det == 0)
			break;

		WP_TMP = new double *[index];
		for (int i=0; i<index; i++)
		{
			WP_TMP[i] = new double[2];
		}

		index = 0;
		for (int i=0; i < cnt; i++)
		{
			if (ABS_BY_BP_TMP[i][1] == REF)
			{
				WP_TMP[index][0] = ABS_BY_BP[i][0];
				WP_TMP[index][1] = ABS_BY_BP[i][1];
				index++;
			}
		}

		double *temp = new double[2];

		if ((flag%2 == 1) && (index > 1))
		{
			for (int i=0; i<index-1; i++)
			{
				int m = i;
				for (int j=i+1; j<index; j++)
				{
					if (WP_TMP[i][0] > WP_TMP[j][0])
					{
						m = j;
					}
				}
				temp[0] = WP_TMP[i][0];
				temp[1] = WP_TMP[i][1];
				WP_TMP[i][0] = WP_TMP[m][0];
				WP_TMP[i][1] = WP_TMP[m][1];
				WP_TMP[m][0] = temp[0];
				WP_TMP[m][1] = temp[1];
			}

			if ((alpha <= PI/2) && (alpha >= -PI/2))
			{
				WP[num][0] = WP_TMP[0][0];
				WP[num][1] = WP_TMP[0][1];
				num++;
				WP[num][0] = WP_TMP[index-1][0];
				WP[num][1] = WP_TMP[index-1][1];
				num++;
			}
			else
			{
				WP[num][0] = WP_TMP[index-1][0];
				WP[num][1] = WP_TMP[index-1][1];
				num++;
				WP[num][0] = WP_TMP[0][0];
				WP[num][1] = WP_TMP[0][1];
				num++;
			}
		}
		else if ((flag%2 == 0) && (index > 1))
		{
			for (int i=0; i<index-1; i++)
			{
				int m = i;
				for (int j=i+1; j<index; j++)
				{
					if (WP_TMP[i][0] < WP_TMP[j][0])
					{
						m = j;
					}
				}
				temp[0] = WP_TMP[i][0];
				temp[1] = WP_TMP[i][1];
				WP_TMP[i][0] = WP_TMP[m][0];
				WP_TMP[i][1] = WP_TMP[m][1];
				WP_TMP[m][0] = temp[0];
				WP_TMP[m][1] = temp[1];
			}

			if ((alpha <= PI/2) && (alpha >= -PI/2))
			{
				WP[num][0] = WP_TMP[0][0];
				WP[num][1] = WP_TMP[0][1];
				num++;
				WP[num][0] = WP_TMP[index-1][0];
				WP[num][1] = WP_TMP[index-1][1];
				num++;
			}
			else
			{
				WP[num][0] = WP_TMP[index-1][0];
				WP[num][1] = WP_TMP[index-1][1];
				num++;
				WP[num][0] = WP_TMP[0][0];
				WP[num][1] = WP_TMP[0][1];
				num++;
			}
		}
		else
		{
			WP[num][0] = WP_TMP[0][0];
			WP[num][1] = WP_TMP[0][1];
			num++;
		}
		flag++;
		delete_mem(WP_TMP, index);
		det = 0;
	}

	printf("Waypoint Setting is complecated!!\n");
	
	delete_mem(ABS_BY_BP,cnt);
	delete_mem(ABS_BY_BP_TMP,cnt);

	// Save the file.
	
//	char pad
	FILE *outxt = fopen("test.dat","wt");
	for (int i = 0; i < cnt_wp; i++)
	{
//		fprintf(outxt,"FlyTo (%3.1f,%3.1f,*)abs vel=%1.1fm/s heading=%3.1f;\r",WP[i][0],WP[i][1],3.0,180.0);
		fprintf(outxt, "%f\t%f\n", WP[i][0],WP[i][1]);
	}
	fclose(outxt);
	// printf("Waypoints are saved in the name of 'result.dat'!!\n");

	delete_mem(WP,cnt_wp);
}	


double MAX_DATA(double **DATA, int size, int col)
{
	double MAX = DATA[0][col-1];
	for (int i = 1; i<size; i++)
	{
		if (MAX < DATA[i][col-1])
		{
			MAX = DATA[i][col-1];
		}
	}
	return MAX;
}
double MIN_DATA(double **DATA, int size, int col)
{
	double MIN = DATA[0][col-1];
	for (int i = 1; i<size; i++)
	{
		if (MIN > DATA[i][col-1])
		{
			MIN = DATA[i][col-1];
		}
	}
	return MIN;
}
double MAX(double x, double y)
{
	if (x >= y)
		return x;
	else
		return y;
}
double dist(double x_1, double y_1, double x_2, double y_2)
{
	return sqrt(pow(x_1-x_2,2)+pow(y_1-y_2,2));
}
double MAX_DIST(double *DATA, int size)
{
	double MAX = DATA[0];
	for (int i = 1; i < size; i++)
	{
		if (MAX < DATA[i])
		{
			MAX = DATA[i];
		}
	}
	return MAX;
}
double dist_line(double x_1, double y_1, double x_p, double y_p, double angle)
{
	return (tan(angle)*x_p-y_p-tan(angle)*x_1+y_1)/sqrt(1+pow(tan(angle),2));
}
double ave_pt(double **DATA, int size, int col)
{
	double AVE_BP=0.0;
	for (int i=0; i<size; i++)
	{
		AVE_BP=AVE_BP+DATA[i][col-1];
	}
	return AVE_BP/size;
}
double MAX_APT(double **DATA, int size)
{
	double MAX = dist(DATA[0][0],DATA[0][1],DATA[1][0],DATA[1][1]); 
	for (int i=0; i<size-1; i++)
	{
		for (int j=i+1; j<size; j++)
		{
			if (MAX < dist(DATA[i][0],DATA[i][1],DATA[j][0],DATA[j][1]))
			{
				MAX = dist(DATA[i][0],DATA[i][1],DATA[j][0],DATA[j][1]);
			}
		}
	}
	return MAX;
}
void delete_mem(double **DATA, int size)
{
	for (int i=0; i<size; i++)
		delete[] DATA[i];
	delete[] DATA;
}
void delete_mem(double *DATA)
{
	delete[] DATA;
}