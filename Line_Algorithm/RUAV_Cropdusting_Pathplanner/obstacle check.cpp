		//////////////// OBSTACLE CHECK //////////////////

		OBS_cntct_cnt = 0;
		for (int j=0; j<OBS_SIZE; j++)
		{
			tmp[0] = (FLD_Ysec[0]-OBS_Ysec[j+1])/(-FLD_Yslp[0]+OBS_Yslp[j+1]);
			tmp[1] = (OBS_Yslp[j+1]*FLD_Ysec[0]-FLD_Yslp[0]*OBS_Ysec[j+1])/(-FLD_Yslp[0]+OBS_Yslp[j+1]);
			if (determine_inside_line(tmp[0],tmp[1],OBS_PT_new[j][0],OBS_PT_new[j][1],OBS_PT_new[j+1][0],OBS_PT_new[j+1][1]) == 1)
			{
				OBS_cntct_cnt++;
			}
		}

		OBS_cntct = new double*[OBS_cntct_cnt];
		for (int k=0; k<OBS_cntct_cnt; k++)
			OBS_cntct[k] = new double[3];

		OBS_cntct_cnt = 0;
		for (int j=0; j<OBS_SIZE; j++)
		{
			tmp[0] = (FLD_Ysec[0]-OBS_Ysec[j+1])/(-FLD_Yslp[0]+OBS_Yslp[j+1]);
			tmp[1] = (OBS_Yslp[j+1]*FLD_Ysec[0]-FLD_Yslp[0]*OBS_Ysec[j+1])/(-FLD_Yslp[0]+OBS_Yslp[j+1]);
			if (determine_inside_line(tmp[0],tmp[1],OBS_PT_new[j][0],OBS_PT_new[j][1],OBS_PT_new[j+1][0],OBS_PT_new[j+1][1]) == 1)
			{
				OBS_cntct_cnt++;
				OBS_cntct[OBS_cntct_cnt-1][0] = tmp[0];
				OBS_cntct[OBS_cntct_cnt-1][1] = tmp[1];
				OBS_cntct[OBS_cntct_cnt-1][2] = j;		// where this contacted point is...
			}
		}

		if (OBS_cntct_cnt == 2)
		{
			if (calculate_distance(temp_wp[1][0],temp_wp[1][1],OBS_cntct[0][0],OBS_cntct[0][1]) >= calculate_distance(temp_wp[1][0],temp_wp[1][1],OBS_cntct[1][0],OBS_cntct[1][1]))
			{
				temp[0] = OBS_cntct[1][0];
				temp[1] = OBS_cntct[1][1];
				temp[2] = OBS_cntct[1][2];
				OBS_cntct[1][0] = OBS_cntct[0][0];
				OBS_cntct[1][1] = OBS_cntct[0][1];
				OBS_cntct[1][2] = OBS_cntct[0][2];
				OBS_cntct[0][0] = temp[0];
				OBS_cntct[0][1] = temp[1];
				OBS_cntct[0][2] = temp[2];
			}
			OBS_dst[0]=OBS_dst[1]=0;
			int pt[2]; pt[0]=pt[1]=0;
			if (OBS_cntct[0][2] == OBS_SIZE-1)
			{
				pt[0] = 0;
				pt[1] = (int)OBS_cntct[0][2];
			}
			else
			{
				pt[0] = (int)(OBS_cntct[0][2])+1;
				pt[1] = (int)OBS_cntct[0][2];
			}
			dst_cnt[0]=dst_cnt[1]=1;
			OBS_dst[0] = calculate_distance(OBS_cntct[0][0],OBS_cntct[0][1],OBS_PT_new[pt[0]][0],OBS_PT_new[pt[0]][1]);
			OBS_dst[1] = calculate_distance(OBS_cntct[0][0],OBS_cntct[0][1],OBS_PT_new[pt[1]][0],OBS_PT_new[pt[1]][1]);
	//여기까지 디버깅 했음!!!!!!!!!!!!!!!!!!!!!!!!!
			while (pt[0] != (int)OBS_cntct[1][2])
			{
				if (pt[0] == OBS_SIZE-1)
				{
					OBS_dst[0] = OBS_dst[0] + calculate_distance(OBS_PT_new[OBS_SIZE-1][0],OBS_PT_new[OBS_SIZE-1][1],OBS_PT_new[0][0],OBS_PT_new[0][1]);
					pt[0] = 1;
					dst_cnt[0]++;
				}
				else
				{
					OBS_dst[0] = OBS_dst[0] + calculate_distance(OBS_PT_new[pt[0]][0],OBS_PT_new[pt[0]][1],OBS_PT_new[pt[0]+1][0],OBS_PT_new[pt[0]+1][1]);
					pt[0]++;
					dst_cnt[0]++;
				}
			}
			if (pt[0] == 0)
			{
				OBS_dst[0] = OBS_dst[0]-calculate_distance(OBS_PT_new[pt[0]][0],OBS_PT_new[pt[0]][1],OBS_PT_new[OBS_SIZE-1][0],OBS_PT_new[OBS_SIZE-1][1])+calculate_distance(OBS_PT_new[pt[0]][0],OBS_PT_new[pt[0]][1],OBS_cntct[1][0],OBS_cntct[1][1]);
				dst_cnt[0]++;
			}
			else
			{
				OBS_dst[0] = OBS_dst[0]-calculate_distance(OBS_PT_new[pt[0]-1][0],OBS_PT_new[pt[0]-1][1],OBS_PT_new[pt[0]][0],OBS_PT_new[pt[0]][1])+calculate_distance(OBS_PT_new[pt[0]][0],OBS_PT_new[pt[0]][1],OBS_cntct[1][0],OBS_cntct[1][1]);
				dst_cnt[0]++;
			}


			while (pt[1] != (int)OBS_cntct[1][2])
			{
				if (pt[1] == 0)
				{
					OBS_dst[1] = OBS_dst[1] + calculate_distance(OBS_PT_new[OBS_SIZE-1][0],OBS_PT_new[OBS_SIZE-1][1],OBS_PT_new[0][0],OBS_PT_new[0][1]);
					pt[1] = 1;
					dst_cnt[1]++;
				}
				else
				{
					OBS_dst[0] = OBS_dst[0] + calculate_distance(OBS_PT_new[pt[1]][0],OBS_PT_new[pt[1]][1],OBS_PT_new[pt[0]-1][0],OBS_PT_new[pt[0]-1][1]);
					pt[1]++;
					dst_cnt[1]++;
				}
			}
			if (pt[1] == OBS_SIZE-1)
			{
				OBS_dst[1] = OBS_dst[1]-calculate_distance(OBS_PT_new[0][0],OBS_PT_new[0][1],OBS_PT_new[OBS_SIZE-1][0],OBS_PT_new[OBS_SIZE-1][1])+calculate_distance(OBS_PT_new[pt[1]][0],OBS_PT_new[pt[1]][1],OBS_cntct[1][0],OBS_cntct[1][1]);
				dst_cnt[1]++;
			}
			else
			{
				OBS_dst[1] = OBS_dst[1]-calculate_distance(OBS_PT_new[pt[1]+1][0],OBS_PT_new[pt[1]+1][1],OBS_PT_new[pt[1]][0],OBS_PT_new[pt[1]][1])+calculate_distance(OBS_PT_new[pt[1]][0],OBS_PT_new[pt[1]][1],OBS_cntct[1][0],OBS_cntct[1][1]);
				dst_cnt[1]++;
			}

			if (OBS_dst[0] >= OBS_dst[1])
				cnt = cnt + dst_cnt[1];
			else
				cnt = cnt + dst_cnt[0];
		}