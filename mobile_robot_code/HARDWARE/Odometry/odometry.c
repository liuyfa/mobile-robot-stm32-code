#include "odometry.h"
#include <stdio.h>
/***********************************************  ���  *****************************************************************/

float position_x=0,position_y=0,oriention=0,velocity_linear=0,velocity_angular=0;

/***********************************************  ����  *****************************************************************/

extern float odometry_right,odometry_left;//���ڵõ����������ٶ�


/***********************************************  ����  *****************************************************************/

float wheel_interval= 405.0f;//    272.0f;        //  1.0146
//float wheel_interval=276.089f;    //���У��ֵ=ԭ���/0.987

float multiplier=4.0f;           //��Ƶ��
float deceleration_ratio=16.0f;  //���ٱ�
float wheel_diameter=100.0f;     //����ֱ������λmm
float pi_1_2=1.570796f;			 //��/2
float pi=3.141593f;              //��
float pi_3_2=4.712389f;			 //��*3/2
float pi_2_1=6.283186f;			 //��*2
float dt=0.005f;                 //����ʱ����5ms
float line_number=500.0f;       //��������
float oriention_interval=0;  //dtʱ���ڷ���仯ֵ

float sin_=0;        //�Ƕȼ���ֵ
float cos_=0;

float delta_distance=0,delta_oriention=0;   //����ʱ�������˶��ľ���

float const_frame=0,const_angle=0,distance_sum=0,distance_diff=0;

float oriention_1=0;
	int a=0;

u8 once=1;

/****************************************************************************************************************/
float deit_theta = 0, theta = 0;
int v_r, v_l;
//��̼Ƽ��㺯��
void odometry(float right,float left)
{	
//	printf("b");
	if(once)  //����������һ��
	{
		const_frame=wheel_diameter*pi/(line_number*multiplier*deceleration_ratio); //
		const_angle=const_frame/wheel_interval; //
		once=0;
	}
    
	distance_sum = 0.5f*(right+left);//�ں̵ܶ�ʱ���ڣ�С����ʻ��·��Ϊ�����ٶȺ�
	distance_diff = right-left;//�ں̵ܶ�ʱ���ڣ�С����ʻ�ĽǶ�Ϊ�����ٶȲ�

    //���������ֵķ��򣬾�����ʱ���ڣ�С����ʻ��·�̺ͽǶ���������
	if((odometry_right>0)&&(odometry_left>0))            //���Ҿ���
	{
		delta_distance = distance_sum;
		delta_oriention = distance_diff;
	}
	else if((odometry_right<0)&&(odometry_left<0))       //���Ҿ���
	{
		delta_distance = -distance_sum;
		delta_oriention = -distance_diff;
		right = -right;
		left = -left;
	}
	else if((odometry_right<0)&&(odometry_left>0))       //�����Ҹ�
	{
		delta_distance = -distance_diff;
		delta_oriention = -2.0f*distance_sum;	
		right = -right;		
	}
	else if((odometry_right>0)&&(odometry_left<0))       //������
	{
		delta_distance = distance_diff;
		delta_oriention = 2.0f*distance_sum;
		left = -left;
	}
	else
	{
		delta_distance=0;
		delta_oriention=0;
	}
	
	deit_theta = (right-left)*0.00981/wheel_interval;
	v_r += right;
	v_l += left;
	theta += deit_theta;
    
	oriention_interval = delta_oriention * const_angle;//����ʱ�����ߵĽǶ�	
	oriention = oriention + oriention_interval;//�������̼Ʒ����
	oriention_1 = oriention + 0.5f * oriention_interval;//��̼Ʒ��������λ���仯���������Ǻ�������
  sin_ = sin(oriention_1);//���������ʱ����y����
	cos_ = cos(oriention_1);//���������ʱ����x����
	
  position_x = position_x + delta_distance * cos_ * const_frame;//�������̼�x����
	position_y = position_y + delta_distance * sin_ * const_frame;//�������̼�y����
	
    
	velocity_linear = delta_distance*const_frame / dt;//�������̼����ٶ�
	velocity_angular = oriention_interval / dt;//�������̼ƽ��ٶ�
	
    //����ǽǶȾ���
	if(oriention > pi)
	{
		oriention -= pi_2_1;
	}
	else
	{
		if(oriention < -pi)
		{
			oriention += pi_2_1;
		}
	}
}
