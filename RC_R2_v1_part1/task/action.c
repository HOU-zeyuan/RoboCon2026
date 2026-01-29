#include "action.h"
//进入之前使nav.nav_state=NAV_PERMUTATION_PATH;flag_permutation_path=1;
uint8_t flag_iinit=1;
void Part0_INIT(void)
{
	if(part_over_flag==0)
  {
		if(flag_iinit)
    {
			nav.nav_state=CHASSIS_INIT;
			foot.foot_state = FOOT_INIT;
			flag_iinit=0;
		}
		if(nav.nav_state==CHASSIS_INIT_DONE)
    {
			//nav.nav_state=NAV_PERMUTATION_PATH;
			if(leftup_turn_motor.ControlLoop_State==MULTIPLE_LOOP&&
				rightup_turn_motor.ControlLoop_State == MULTIPLE_LOOP&&
        leftdown_turn_motor.ControlLoop_State == MULTIPLE_LOOP&&
        rightdown_turn_motor.ControlLoop_State == MULTIPLE_LOOP)
			part_over_flag=1;
			flag_path_1=1;
		}
	}
}

void Part1_action(void)   //一区
{
	if(part_over_flag==1)
  {
	   switch(flag_path_1)
    {
	    case 1:
				if(flag_choose)
        {
					up_s_state=S_INIT;
					nav.auto_path.number_permutation=0;
			    nav.nav_state=NAV_PERMUTATION_PATH;					
					flag_choose=0;
				}
		    if(nav.nav_state==NAV_LOCK)    //路径完成
        {
			    flag_path_1=2;
		    }
		  break;		
		  case 2:
				up_d_state=D_READY_GET_HEAD;   //准备取武器头  2
			  if(flag_D_ready_get_head==10)//准备完成
        {
					up_d_state=D_GET_LEFT;    //取武器头3
				  	flag_path_1=3;
			  }
		  break;
			case 3:
				if(flag_D_ready_get_head==10)//取完
				{
					//part_over_flag=2;  //进入二区前置任务	
            	judge_123=0;	//改！！！			
				}
				break;
	  }
	}
}
void Part2_123_action(void)//上台阶前取123方块
{
	if(part_over_flag==2)
  {
		switch(judge_123)
   {
			case 0:  //123无方块，直接到二
				if(flag_choose)
			 {
         nav.auto_path.number_permutation=1; 
			   nav.nav_state=NAV_PERMUTATION_PATH;
				 flag_choose=0;
			  }
			  if(nav.nav_state==NAV_LOCK)  
        {					
				   judge_123=4;
		     }
				break;
			case 1:  //1有方块，到1 
			     if(flag_choose)
			    {
             nav.auto_path.number_permutation=4; 
			       nav.nav_state=NAV_PERMUTATION_PATH;
				     flag_choose=0;
			    }
			    if(nav.nav_state==NAV_LOCK)
         {
			      judge_123=4;
		      }
				break;
			case 2:  //2有方块，到2		     
					if(flag_choose)
					{
					 nav.auto_path.number_permutation=1; 
					 nav.nav_state=NAV_PERMUTATION_PATH;
						flag_choose=0;
					}				  
					if(nav.nav_state==NAV_LOCK)
					{
					 judge_123=4;
					}
				break;
			case 3:  //3有方块，到3				
				if(flag_choose)
			  {
					 nav.auto_path.number_permutation=5; 
					 nav.nav_state=NAV_PERMUTATION_PATH;
					 flag_choose=0;
			  }
				if(nav.nav_state==NAV_LOCK)
				{
					 judge_123=4;
				}
				break;
			case 4:	
				if(flag_choose)
			   {
						 nav.auto_path.number_permutation=3;//逆时针 
						 nav.nav_state=NAV_PERMUTATION_PATH;
						 flag_choose=0;
						 judge_123=5;
			  }
        break;
			case 5:
				if(nav.nav_state==NAV_LOCK)  //取块或进二区
        {
             if(block_state.vision_judge_123==0)
						 {
							  flag_part_2=3;
								part_over_flag=3;
						 }
						 else if(block_state.vision_judge_123==2)
						 {
							  up_d_flag=D_GET_BLOCK;
							  up_s_flag=S_GET_BLOCK_UP;
						 }
						 else if(block_state.vision_judge_123)
						 {
							  up_d_flag=D_GET_BLOCK;
							  up_s_flag=S_GET_BLOCK_TOP;
						 }		 
					  if((flag_S_get_block_up==10||flag_S_get_block_top==10)&&flag_D_get_block==10)
            {
					     judge_123=6;
		        }
		     }				
				break;
			case 6:
			   if(block_state.vision_judge_123==2)
         {   flag_part_2=3;	
						part_over_flag=3;  //进入二区任务									  
				 }
				else if(block_state.vision_judge_123==1)
        {
						if(flag_choose)
						{
							 nav.auto_path.number_permutation=6; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}
						if(nav.nav_state==NAV_LOCK)
						{ 
							flag_part_2=3;
							 part_over_flag=3;  //进入二区任务										  
						}						
				}				
				else if(block_state.vision_judge_123==3)
        {					
						if(flag_choose)
						{
							 nav.auto_path.number_permutation=7; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}
						if(nav.nav_state==NAV_LOCK)
						{  
							 flag_part_2=3;
							 part_over_flag=3;  //进入二区任务										  
						 }						  
				}
        break;
	   }
   }
}
uint8_t up_stairs_flag=1;
void Part2_action(void)  //二区
{
	int i=0;
	if(i<5&&part_over_flag==3)
  {
		switch(flag_part_2)
   {
			case 0: //判断转弯
				if((vision_data_recieve.x2-vision_data_recieve.x1)<600&&(vision_data_recieve.x2-vision_data_recieve.x1)>-600)//水平走
		    {
			    if((vision_data_recieve.y2-vision_data_recieve.y1)>800)  //左转
			    {
						if(flag_choose)
						{
							 nav.auto_path.number_permutation=3; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}
				    turn_action=2;
			     }
			     else if((vision_data_recieve.y2-vision_data_recieve.y1)<-800)  //右转
					 {
							if(flag_choose)
							{
								 nav.auto_path.number_permutation=2; 
								 nav.nav_state=NAV_PERMUTATION_PATH;
								 flag_choose=0;
							}
						 turn_action=1;
					}				 
		    }
	     else if((vision_data_recieve.x2-vision_data_recieve.x1)>800)//向前走
			 {
				  turn_action=0;
				  flag_part_2=1;
			 }		 
				break;
			case 1:   //面前是否有方块,并判断是否取
				if(nav.nav_state==NAV_LOCK)
        {
					  if(!vision_data_recieve.judge)//接收反馈
						{
							flag_part_2=2; 
						}
						else if(vision_data_recieve.judge)
						{
								 if((vision_data_recieve.z2-vision_data_recieve.z1)>150)
								 {
									 up_s_flag=S_GET_BLOCK_UP;
								 }
								 else if((vision_data_recieve.z2-vision_data_recieve.z1)<-150)
								 {
									 up_s_flag=S_GET_BLOCK_DOWN;
								 }	
								 if(flag_S_get_block_up==10||flag_S_get_block_down==10)
								{
									flag_part_2=8;
								}							 
						}
			   } 
				 break;
			case 2: //微调
				if(flag_choose)
				{
					 nav.auto_path.number_point=(vision_data_recieve.id*10+vision_data_recieve.next_id%10); 
					 nav.nav_state=NAV_POINT_TO_POINT;
					 flag_choose=0;
				}
			  if(nav.nav_state==NAV_LOCK)
        {
				   flag_part_2=3;
			  }
				break;
			case 3:  //判断上台阶还是下台阶			
				if((vision_data_recieve.z2-vision_data_recieve.z1)>150)//上台阶
			 { 
					if (up_stairs_flag)
					{
						foot.foot_state = FOOT_UP_PREPARE; // 2
						up_stairs_flag = 0;
					}
					if (foot.foot_state == FOOT_CLEAR) // 1
					{
						up_stairs_flag = 1;
						flag_part_2=4;
					}
			 }
			 else if((vision_data_recieve.z2-vision_data_recieve.z1)<150)//下台阶
			 {
          if (up_stairs_flag)
					{
						foot.foot_state=FOOT_DOWN_PREPARE; // 2
						up_stairs_flag = 0;
					}
					if (foot.foot_state == FOOT_CLEAR) // 1
					{
						up_stairs_flag = 1;
						flag_part_2=4;				
					}				          			 
			 }
				break;			
			case 4:  //上台阶后转回去
			if(turn_action==0)  //判断转向完成
       {
			   flag_part_2=5;
				 i++;
		   }
				else if(turn_action==1)
        {			      				   
						if(flag_choose)
						{
							 nav.auto_path.number_permutation=3; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}		
		    }
		     else if(turn_action==2)
		    {    
						if(flag_choose)
						{
							 nav.auto_path.number_permutation=2; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}
						 flag_choose=0;			
		    }
			  if(nav.nav_state==NAV_LOCK)
        {
				  flag_part_2=5;
					i++;
			  }
				break;
			case 5:  //是否取左右方块,取则转弯,并取块
				if(block_state.vision_block_num==2||block_state.vision_block_num==3||block_state.vision_block_num==4)//路上总块数>=2,不取
        {
					flag_vision_update++;
					flag_part_2=0;
				}
				else if((block_state.vision_block_num==0&&block_state.num==0)||block_state.vision_block_num==1)//路上总块数<2,取第一个块
       {
				 if((vision_data_recieve.x3-vision_data_recieve.x1)<400&&(vision_data_recieve.x3-vision_data_recieve.x1)>-400)//此刻取
				 {
					 if((vision_data_recieve.y3-vision_data_recieve.y1)>800)  //左转
					 {			 
							if(flag_choose)
							{
								 nav.auto_path.number_permutation=3; 
								 nav.nav_state=NAV_PERMUTATION_PATH;
								 flag_choose=0;
							}
							 turn_action=2;
					 }
					 else if((vision_data_recieve.y3-vision_data_recieve.y1)<-800)  //右转
			     {
							if(flag_choose)
							{
								 nav.auto_path.number_permutation=2; 
								 nav.nav_state=NAV_PERMUTATION_PATH;
								 flag_choose=0;
							}
							 turn_action=1;
			     }
				   if(nav.nav_state==NAV_LOCK)
           {
							 if((vision_data_recieve.z3-vision_data_recieve.z1)>150)
							 {
								 up_s_flag=S_GET_BLOCK_UP;
							 } 
							 else if((vision_data_recieve.z3-vision_data_recieve.z1)<-150)	
							 {
								 up_s_flag=S_GET_BLOCK_DOWN;
							 } 
							 flag_part_2=6;
				   }
				 }
				 else
				 {
					 flag_vision_update++;
					 flag_part_2=0;
				 }				 
			 }
			 else if(block_state.vision_block_num==0&&block_state.num==1)
       {
				 if((vision_data_recieve.x4-vision_data_recieve.x1)<400&&(vision_data_recieve.x4-vision_data_recieve.x1)>-400)//此刻取
				 {
					 if((vision_data_recieve.y4-vision_data_recieve.y1)>800)  //左转
					 {						 
							if(flag_choose)
							{
								 nav.auto_path.number_permutation=3; 
								 nav.nav_state=NAV_PERMUTATION_PATH;
								 flag_choose=0;
							}
							 turn_action=2;
					 }
					 else if((vision_data_recieve.y4-vision_data_recieve.y1)<-800)  //右转
			     {
							if(flag_choose)
							{
								 nav.auto_path.number_permutation=2; 
								 nav.nav_state=NAV_PERMUTATION_PATH;
								 flag_choose=0;
							}
							 turn_action=1;
			     }
				   if(nav.nav_state==NAV_LOCK)
           {
							 if((vision_data_recieve.z4-vision_data_recieve.z1)>150)
							 {
								 up_s_flag=S_GET_BLOCK_UP;
							 }
							 else if((vision_data_recieve.z4-vision_data_recieve.z1)<-150)	
							 {
								 up_s_flag=S_GET_BLOCK_DOWN;
							 }	
							 flag_part_2=6;
				   }
				 }
				 else
				 {
					 flag_vision_update++;
					 flag_part_2=0;
				 }				
			 }
				break;
			case 6:  //	左右取如何存方块				
			  if(flag_S_get_block_up==10||flag_S_get_block_down==10)
        {
						if(block_state.num==1)//第二个块单臂存
						{					
							 flag_part_2=7;
						}
						else if(block_state.num==0)//第一个块双臂存
						{
							up_d_flag=D_GET_BLOCK;
						}
				}
				else if(flag_D_get_block==10)
        {
						flag_vision_update++;
						block_state.num++;
						//block_state.pass_block++;
						flag_part_2=7;
				}				
				break;
			case 7:  //向左右取块后再转回去
				if(turn_action==1)
        { 
				   if(flag_choose)
						{
							 nav.auto_path.number_permutation=3; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}
		    }
		     else if(turn_action==2)
		    {			     
				  if(flag_choose)
						{
							 nav.auto_path.number_permutation=2; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}
		    }
					if(nav.nav_state==NAV_LOCK)  //判断转向完成
				 {
					 flag_part_2=0;
				 }
				break;
			case 8:  //向前取如何存块
				if(block_state.vision_block_num==1||block_state.vision_block_num==2)//总路上块数1或2
         {
					  if(block_state.num==0)//第一个块双臂存
           {
						  up_d_flag=D_GET_BLOCK;  
					  }
					  else if(block_state.num==1)//第二个块单臂存
           {
					   flag_part_2=3;  
				   }				
				}
				 else if(block_state.vision_block_num==3)//总路上块数为3
        {
						if(block_state.vision_judge_123==0)//123无块
						{
								if(block_state.pass_block==0)  //第一个块直接扔
								{
									up_s_flag=S_THROW_BLOCK_BACK;    
								}
								else if(block_state.pass_block==1)  //第二个块双臂存
								{
									up_s_flag=D_GET_BLOCK; 
								}
								else if(block_state.pass_block==2)  //第三个块单臂存
								{
									flag_part_2=3;
								}
						}
						else if(block_state.vision_judge_123!=0)	//手中一定有1个块				
						{
								if(block_state.pass_block==1)  //第二个块转身扔
								{
										if(flag_choose)
									  {
											 nav.auto_path.number_permutation=8; 
											 nav.nav_state=NAV_PERMUTATION_PATH;
											 flag_choose=0;
									  }
										if(nav.nav_state==NAV_LOCK)
										{
											 flag_part_2=9;
										}
								}
								else if(block_state.pass_block==2) //第三个块单臂存
								{
									   flag_part_2=3;
								}
					}
				}
				else if(block_state.vision_block_num==4)//总路上块数为4
        {
						if(block_state.vision_judge_123==0)//123无块
						{
								if(block_state.pass_block<2)//12个块直接扔
								{
									  up_s_flag=S_THROW_BLOCK_BACK;
								}
								else if(block_state.pass_block==2)//第3个双臂存
							  {
								    up_s_flag=D_GET_BLOCK;
							  }
							  else if(block_state.pass_block==3)//第4个单臂存
							  {
								   flag_part_2=3; 
							  }
						}
						else if(block_state.vision_judge_123!=0)	//手中一定有1个块		
						{
								if(block_state.pass_block==1||block_state.pass_block==2)//23个块转身扔
								{
										if(flag_choose)
									  {
											 nav.auto_path.number_permutation=8; 
											 nav.nav_state=NAV_PERMUTATION_PATH;
											 flag_choose=0;
									  }
										if(nav.nav_state==NAV_LOCK)
										{
											 flag_part_2=9;
										}
								}
								else if(block_state.pass_block==3)//第4个块单臂存
								{
									 flag_part_2=3;
								}
						}
				}
				if(flag_D_get_block==10)
        {
						block_state.pass_block++;
					  block_state.num++;
					  flag_part_2=3;
				  }
        	else if(flag_S_throw_block_back==10)
         {
					  block_state.pass_block++;
					  flag_part_2=3;
				 }				
				break;
			case 9://转身扔块
				up_s_flag=S_THROW_BLOCK_FRONT;
			  if(flag_S_throw_block_front==10)
        {					
				    if(flag_choose)
						{
							 nav.auto_path.number_permutation=8; 
							 nav.nav_state=NAV_PERMUTATION_PATH;
							 flag_choose=0;
						}
						if(nav.nav_state==NAV_LOCK)
						{
							 block_state.pass_block++;
							 flag_part_2=3;
						}
				}
				break;
	 }	   
	}
	if(i==5)
  {
		  if (up_stairs_flag)
			{
					foot.foot_state=FOOT_DOWN_PREPARE; // 2
					up_stairs_flag = 0;
			}
			if (foot.foot_state == FOOT_CLEAR) // 1
			{
					up_stairs_flag = 1;
					flag_part_3=vision_data_recieve.id;
		      part_over_flag=4;
			}				 		
	}
}
	void Part3_action(void)//3区状态机
{
	if((nav.nav_state==NAV_PERMUTATION_PATH||nav.nav_state==NAV_LOCK)&&part_over_flag==4)
  {
		switch(flag_part_3)
   {
			case 10://到9宫格前
				if(flag_choose)
				{
					 nav.auto_path.number_permutation=9; 
					 nav.nav_state=NAV_PERMUTATION_PATH;
					 flag_choose=0;
				}
			  if(nav.nav_state==NAV_LOCK)
        {
				   flag_part_3=1;
				}
				break;
			case 12://到9宫格前
				if(flag_choose)
				{
					 nav.auto_path.number_permutation=10; 
					 nav.nav_state=NAV_PERMUTATION_PATH;
					 flag_choose=0;
				}
			  if(nav.nav_state==NAV_LOCK)
        {
				   flag_part_3=1;
				}
				break;
			case 1://逆时针转90度，单臂面向9宫格
				if(flag_choose)
				{
					 nav.auto_path.number_permutation=2; 
					 nav.nav_state=NAV_PERMUTATION_PATH;
					 flag_choose=0;
				}
			  if(nav.nav_state==NAV_LOCK)
        {
				   flag_part_3=2;
				}
				break;
			case 2://放中层方块
				up_s_flag=S_PUT_BLOCK_MIDDLE;
			  if(flag_S_put_block_middle==10)
        {
					 flag_part_3=3;
				}
				break;
			case 3://上R1
				
				break;
			case 4://放上层方块
				
				break;
	 }
	}
}



