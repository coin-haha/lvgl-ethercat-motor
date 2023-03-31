#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>
#include "ethercat.h"


#include "lvgl/lvgl.h"
#include "lvgl/demos/lv_demos.h"
#include "lv_drivers/display/fbdev.h"
#include "lv_drivers/wayland/wayland.h"
#include "lv_drivers/indev/evdev.h"
#include <pthread.h>
#include <time.h>
#include <sys/time.h>



#define EC_TIMEOUTMON 500

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
OSAL_THREAD_HANDLE thread2;
int expectedWKC;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

#pragma pack(2)
typedef struct
{
    uint16 wControlWord;
    uint8 bModeOperation;
    uint8 bComplement;
    int iTargetPosition;
    int16 siVITargetVelocity;
    int16 siVITargetTorque;
    int iFstSetPoint;
    int iTargetVelocity;
    uint16 wDOStatesCommSet;
} RPDO_Datas;

typedef struct
{
    uint16 wErrorCode;
    uint16 wStatusWord;
    uint8 bModeOperationDisplay;
    uint8 bComplement;
    int iActualPosition;
    int iActualPosition_Sensor;
    int iActualVelocity_Sensor;
    int iActualVelocity;
    int16 siActualTorque;
    int16 siActualCurrent;
    uint16 wMonitoredDIStates;
} TPDO_Datas;

typedef struct
{
    RPDO_Datas rpdo;
    TPDO_Datas tpdo;
    int reset_delay;
} SERVO_PDO_Datas;

#pragma pack()

volatile SERVO_PDO_Datas ServoList[16];



char help_str[][256]={
	"-----------------------------------------------------------------------------------------------------------------------\n",
	"| 1 : set mode | 2 : start | 3: shutdown | 4: quick stop | 5: autorun | 6: invert pol |  7: show mode | 8: fault reset\n",
	"------------------------------------------------------------------------------------------------------------------------\n",


};

char mode_str[][64]={
	"--------------------------------\n"
	"| 1 :  pp mode | 3 : pv mode \n",
	"--------------------------------\n"

};


int autorun_flag=0;
int polarity = 1;
int mode=1;
int enable_debug = 0;
int abs_rel=0;
/* pp mode */
int target_position = 10000;
int profile_velocity = 450000;
int Profile_acceleration = 500000;
int Profile_deceleration = 500000;
int quick_stop_Deceleration = 100000;


OSAL_THREAD_FUNC PDOThread( void *ptr )
{
    while(inOP)
    {
        //prepare rpdo datas
        for (int slave = 1; slave <= ec_slavecount; slave++)
        {
            memcpy(ec_slave[slave].outputs, &ServoList[slave-1].rpdo, sizeof(RPDO_Datas));
        }
        //send rpdo datas
        ec_send_processdata();
        //receive tpdo datas
        wkc = ec_receive_processdata(EC_TIMEOUTRET);
        //parse tpdo datas
        if (wkc == expectedWKC)
        {
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                memcpy(&ServoList[slave-1].tpdo, ec_slave[slave].inputs, sizeof(TPDO_Datas));
            }
        }
        osal_usleep(5000);
    }
}

OSAL_THREAD_FUNC ecatcheck( void *ptr )
{
    int slave;
    (void)ptr;                  /* Not used */

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            /*if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }*/
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     /* re-check state */
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTMON))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}


OSAL_THREAD_FUNC autorun (void *ptr){
	(void)ptr;                  /* Not used */

	while(1){
		if(autorun_flag == 1){
			if(mode == 1){
				start(1);
				ServoList[0].rpdo.iTargetPosition = 10000000*polarity;
				ServoList[0].rpdo.wControlWord = 0x6F;
				usleep(10000);
				ServoList[0].rpdo.wControlWord = 0x7F;


				
			}else if(mode == 3){
				start(1);

			}

		}


		sleep(1);
	}

	
	
}


boolean PowerOn(int slaveId)
{
    ServoList[slaveId-1].rpdo.bModeOperation = 1;
	/*  ready to switch on */
	printf("11111111111 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);
    ServoList[slaveId-1].rpdo.wControlWord = 6;
	/* switch on */
	
	printf("22222222222 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0001) != 0)
        ServoList[slaveId-1].rpdo.wControlWord = 7;
	printf("333333333333 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);
	/* switch on or operation enable */
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0006) != 0)
        ServoList[slaveId-1].rpdo.wControlWord = 15;
	printf("44444444444 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);

	/* fault*/
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0008) != 0
            && ServoList[slaveId-1].reset_delay <= 0)
        ServoList[slaveId-1].reset_delay = 100;
	printf("5555555555 reset_delay[%d] [%x] \n",ServoList[slaveId-1].reset_delay,ServoList[slaveId-1].tpdo.wStatusWord);

    if (ServoList[slaveId-1].reset_delay > 0)
    {
        ServoList[slaveId-1].reset_delay--;
        ServoList[slaveId-1].rpdo.wControlWord = 128;
    }
	printf("666666666666666 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);

    return ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0004) != 0);
}


int check_is_read_to_switch_on_stat(int slaveId)
{
	return (ServoList[slaveId-1].tpdo.wStatusWord & 0x0001) == 0x0001;

}

int check_wait_read_to_switch_on_stat(int slaveId)
{
	int time =0;
	do
	{
		usleep(100);
		time++;
		if(time > 200){
			return -1;
		}
	}
	while((ServoList[slaveId-1].tpdo.wStatusWord & 0x0001) != 0x0001);
	return 0;

}


int check_is_switch_on_stat(int slaveId)
{
	return (ServoList[slaveId-1].tpdo.wStatusWord & 0x0002) == 0x0002;

}

int check_wait_switch_on_stat(int slaveId)
{
	int time =0;
	do
	{
		usleep(100);
		time++;
		if(time > 200){
			return -1;
		}
	}
	while((ServoList[slaveId-1].tpdo.wStatusWord & 0x0002) != 0x0002);
	return 0;

}

int check_is_op_enabled_stat(int slaveId)
{
	return (ServoList[slaveId-1].tpdo.wStatusWord & 0x0004) == 0x0004;

}

int check_wait_op_enabled_stat(int slaveId)
{
	int time =0;
	do
	{
		usleep(100);
		time++;
		if(time > 200){
			return -1;
		}
	}
	while((ServoList[slaveId-1].tpdo.wStatusWord & 0x0004) != 0x0004);
	return 0;

}


int start(int slaveId)
{
	int ret = 0;
	ServoList[slaveId-1].rpdo.bModeOperation = mode;

	if(enable_debug)
		printf("11111111111 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);
	
	if(check_is_read_to_switch_on_stat(slaveId)  == 0){
		ServoList[slaveId-1].rpdo.wControlWord = 6;
		ret = check_wait_read_to_switch_on_stat(slaveId);
		if(ret == 0){
			printf("read to switch on !\n");
		}
	}
	if(enable_debug)
		printf("22222222 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);

	if(check_is_switch_on_stat(slaveId) == 0){
		ServoList[slaveId-1].rpdo.wControlWord = 7;
		ret = check_wait_switch_on_stat( slaveId);
		if(ret == 0){
			printf("is switch on !\n");
		}

	}

	if(enable_debug)
		printf("333333333 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);


	/* fault*/
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0008) != 0
            && ServoList[slaveId-1].reset_delay <= 0)
        ServoList[slaveId-1].reset_delay = 100;

    if (ServoList[slaveId-1].reset_delay > 0)
    {
    	printf("need reset\n");
        ServoList[slaveId-1].reset_delay--;
        ServoList[slaveId-1].rpdo.wControlWord = 128;
    }
	if(enable_debug)
		printf("55555555555555 [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);

	if(check_is_op_enabled_stat(slaveId) == 0){
		ServoList[slaveId-1].rpdo.wControlWord = 15;
		ret = check_wait_op_enabled_stat( slaveId);
		if(ret == 0){
			printf("is op enable !\n");
		}

	}	


	
#if 0
	/*  ready to switch on */
    ServoList[slaveId-1].rpdo.wControlWord = 6;
	/* switch on */
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0001) != 0)
        ServoList[slaveId-1].rpdo.wControlWord = 7;
	/* switch on or operation enable */
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0006) != 0)
        ServoList[slaveId-1].rpdo.wControlWord = 15;

	/* fault*/
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0008) != 0
            && ServoList[slaveId-1].reset_delay <= 0)
        ServoList[slaveId-1].reset_delay = 100;

    if (ServoList[slaveId-1].reset_delay > 0)
    {
        ServoList[slaveId-1].reset_delay--;
        ServoList[slaveId-1].rpdo.wControlWord = 128;
    }

    return ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0004) != 0);
#endif
	return 0;
}


/* ready to switch on */
int shutdown(int slaveId)
{
	ServoList[slaveId-1].rpdo.wControlWord = 6;
	if((ServoList[slaveId-1].tpdo.wStatusWord & 0x0001) !=0 ){
		printf("read to switch on \n");
	}else{
		printf("not read to switch on [%x] \n",ServoList[slaveId-1].tpdo.wStatusWord);
	}

	return 0;
}

/*switched on */
int switch_on(int slaveId)
{

	if((ServoList[slaveId-1].tpdo.wStatusWord & 0x0002) !=0){
		/* has switch on state */
		return 0;
	}

	if((ServoList[slaveId-1].tpdo.wStatusWord & 0x0001) !=0){
		printf("no on ready to switch on stat\n");
		shutdown(slaveId);
	}

	return 0;
}

/* switch on disabled */
int disable_voltage(int slaveId)
{
	ServoList[slaveId-1].rpdo.wControlWord = 0;
	return 0;
}

/* switch on disabled*/
int quick_stop(int slaveId)
{
	ServoList[slaveId-1].rpdo.wControlWord = 2;

	return 0;
}

/* switched on */
int disabled_operation(int slaveId)
{
	
	return 0;
}

/* operation enabled  */
int enable_operation(int slaveId)
{
	
	return 0;
}

/* switch on disabled */
int fault_reset(int slaveId)
{
	/* fault*/
    if ((ServoList[slaveId-1].tpdo.wStatusWord & 0x0008) != 0
            && ServoList[slaveId-1].reset_delay <= 0)
        ServoList[slaveId-1].reset_delay = 100;

    if (ServoList[slaveId-1].reset_delay > 0)
    {
    	printf("need reset\n");
        ServoList[slaveId-1].reset_delay--;
        ServoList[slaveId-1].rpdo.wControlWord = 128;
    }

	return 0;
}
int show_pv_mode_param(int slaveId)
{
	uint32 nValue ;
	uint32 l = sizeof(nValue);

	ec_SDOread(slaveId, 0X6060, 0, FALSE, &l, &nValue, 250000);
	printf("[MODE ]-[%d]\n",nValue);


	ec_SDOread(slaveId, 0x60FF, 0, FALSE, &l, &nValue, 250000);
	printf("[Target velocity]-[%d]\n",nValue);
	

	ec_SDOread(slaveId, 0x6083, 0, FALSE, &l, &nValue, 250000);
	printf("[Profile Acceleration]-[%d]\n",nValue);

	ec_SDOread(slaveId, 0x6084, 0, FALSE, &l, &nValue, 250000);
	printf("[Profile Deceleration]-[%d]\n",nValue);


	ec_SDOread(slaveId, 0x6085, 0, FALSE, &l, &nValue, 250000);
	printf("[Quick Stop Deceleration]-[%d]\n",nValue);


	ec_SDOread(slaveId, 0x60C5, 0, FALSE, &l, &nValue, 250000);
	printf("[Max Acceleration]-[%d]\n",nValue);

	ec_SDOread(slaveId, 0x60C6, 0, FALSE, &l, &nValue, 250000);
	printf("[Max Deceleration]-[%d]\n",nValue);

}

int configure_pv_mode(int slaveId)
{
	 uint32 nValue ;
	
	 nValue = 4000;
	 ec_SDOwrite(slaveId, 0x60FF, 0, FALSE, 4, &nValue, 250000);
	
	
	
	 nValue = 400000;
	 ec_SDOwrite(slaveId, 0x6083, 0, FALSE, 4, &nValue, 250000);
	 
	
	 nValue = 400000;
	 ec_SDOwrite(slaveId, 0x6084, 0, FALSE, 4, &nValue, 250000);
	
	
	 nValue=10000;
	 ec_SDOwrite(slaveId, 0x6085, 0, FALSE, 4, &nValue, 250000);
	
	 show_pv_mode_param(slaveId);


}

int show_pp_mode_param(int slaveId)
{
	uint32 nValue ;
	uint32 l = sizeof(nValue);


	//ec_SDOread(slaveId, 0X6060, 0, FALSE, &l, &nValue, 250000);
	//printf("[MODE ]-[%d]\n",nValue);


	ec_SDOread(slaveId, 0x607F, 0, FALSE, &l, &nValue, 250000);
	printf("[Max Profile Velocity]-[%d]\n",nValue);
	


	ec_SDOread(slaveId, 0x6081, 0, FALSE, &l, &nValue, 250000);
	printf("[Profile Velocity]-[%d]\n",nValue);

	ec_SDOread(slaveId, 0x6082, 0, FALSE, &l, &nValue, 250000);
	printf("[End Velocity]-[%d]\n",nValue);


	ec_SDOread(slaveId, 0x6083, 0, FALSE, &l, &nValue, 250000);
	printf("[Profile Acceleration]-[%d]\n",nValue);

	ec_SDOread(slaveId, 0x6084, 0, FALSE, &l, &nValue, 250000);
	printf("[Profile Deceleration]-[%d]\n",nValue);


	ec_SDOread(slaveId, 0x6085, 0, FALSE, &l, &nValue, 250000);
	printf("[Quick Stop Deceleration]-[%d]\n",nValue);


	ec_SDOread(slaveId, 0x60C5, 0, FALSE, &l, &nValue, 250000);
	printf("[Max Acceleration]-[%d]\n",nValue);

	ec_SDOread(slaveId, 0x60C6, 0, FALSE, &l, &nValue, 250000);
	printf("[Max Deceleration]-[%d]\n",nValue);

	return 0;
}


int configure_pp_profile_velocity(int slaveId,int value)
{
	uint32 nValue ;
	
    nValue = value;
    ec_SDOwrite(slaveId, 0x6081, 0, FALSE, 4, &nValue, 250000);


}


int configure_pp_Profile_acceleration(int slaveId,int value)
{
	uint32 nValue ;
	
    nValue = value;
    ec_SDOwrite(slaveId, 0x6083, 0, FALSE, 4, &nValue, 250000);


}


int configure_pp_Profile_deceleration(int slaveId,int value)
{
	uint32 nValue ;
	
    nValue = value;
    ec_SDOwrite(slaveId, 0x6084, 0, FALSE, 4, &nValue, 250000);


}

int configure_pp_quick_stop_Deceleration(int slaveId,int value)
{
	uint32 nValue ;
	
    nValue = value;
    ec_SDOwrite(slaveId, 0x6085, 0, FALSE, 4, &nValue, 250000);


}

int configure_pp_mode(int slaveId)
{
    


	configure_pp_profile_velocity(slaveId,profile_velocity);



	configure_pp_Profile_acceleration(slaveId,Profile_acceleration);


	
	configure_pp_Profile_deceleration(slaveId,Profile_deceleration);



	configure_pp_quick_stop_Deceleration(slaveId,quick_stop_Deceleration);



	show_pp_mode_param(slaveId);

	return 0;

}

int set_workmode(int slaveId, int mode)
{



	
	ServoList[slaveId-1].rpdo.bModeOperation = mode;

	switch(mode){
		case 1:
			/* pp mode */
			configure_pp_mode(slaveId);
			break;

		case 3:
			/* pv mode */
			configure_pv_mode(slaveId);
			break;

		defualt:
			printf("can not recoginze mode [%d]\n",mode);


	}
	return 0;
}



int ethercat_init(int argc, char * argv[])
{
	//scan network adapter
    int flag = 1;
	int i,input;
	int m_mode;
	ec_adaptert *adapter = NULL;
	int slc;
	int ret = -1;

    while (adapter != NULL)
    {
        printf("Description : %s, Name: %s\r\n", adapter->desc, adapter->name);
        adapter = adapter->next;
    }

    if (argc < 2)
    {
        return ret;
    }

	// initialise SOEM, bind socket to ifname
    if (0 >= ec_init(argv[1]))
    {
        printf("SOEM init faild\r\n");
        return ret;
    }

    // scan slave device
    if (0 >= ec_config_init(FALSE))
    {
        printf("No slave device found\r\n");
        return ret;
    }
	ec_statecheck(0, EC_STATE_PRE_OP, 500000);

	for ( slc = 1; slc <= ec_slavecount; slc++)
	{
		ec_statecheck(slc, EC_STATE_PRE_OP, EC_TIMEOUTRET);
		printf("%d: configAddr = 0x%04x, eep_man = 0x%08x, eep_id = 0x%04x, name = '%s', state = %x\r\n",
			   slc, ec_slave[slc].configadr, ec_slave[slc].eep_man,
			   ec_slave[slc].eep_id, ec_slave[slc].name, ec_slave[slc].state);
	}


	ec_config_map(&IOmap);
	ec_configdc();
	// wait for all slaves to reach SAFE_OP state
	ec_statecheck(0, EC_STATE_SAFE_OP,	EC_TIMEOUTSTATE * 4);
	if (ec_slave[0].state != EC_STATE_SAFE_OP)
	{
		//not all slaves changed to safe_operate state
		return ret;
	}

	
	printf("ouputswkc[%d] inputwkc[%d]\n",ec_group[0].outputsWKC,ec_group[0].inputsWKC);
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;

	    /* send one valid process data to make outputs in slaves happy*/
    memset(&IOmap[0], 0, 4096);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);
    /* request OP state for all slaves */
    ec_slave[0].state = EC_STATE_OPERATIONAL;
    ec_writestate(0);
    int chk = 200;
    /* wait for all slaves to reach OP state */
    do
    {
       ec_send_processdata();
       ec_receive_processdata(EC_TIMEOUTRET);
       ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    }
    while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

	if (ec_slave[0].state != EC_STATE_OPERATIONAL )
	{
		// request INIT state for all slaves
		ec_slave[0].state = EC_STATE_PRE_OP;
		ec_writestate(0);
		return ret;
	}

	inOP = TRUE;
	osal_thread_create(&thread1, 128000, &PDOThread, (void*) &ctime);
	osal_thread_create(&thread2, 128000, &ecatcheck, (void*) &ctime);
	osal_thread_create(&thread2, 128000, &autorun, (void*) &mode);


	return 0;

	
}

int reload_position()
{
	printf("reload_new_position [%d]\n",target_position*polarity);
	ServoList[0].rpdo.iTargetPosition = target_position*polarity;
	ServoList[0].rpdo.wControlWord = 0x2F|(abs_rel<<6);
	usleep(10000);
	ServoList[0].rpdo.wControlWord = 0x3F|(abs_rel<<6);


}

static void mode_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
    
    		switch(lv_dropdown_get_selected(obj)){
    				case 0:
    					set_workmode(1,1);
    				break;
    				case 1:
    					set_workmode(1,3);
    				break;
    				
    				default:
    					LV_LOG_USER("invalid !\n");
    		
    		}
    
       
    }
}

static void direction_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
        LV_LOG_USER("Option: %s", buf);
		if(strcasecmp("FWD",buf) == 0){
			polarity=1;
			printf("FWD\n");
		}else if(strcasecmp("REV",buf) == 0){
			polarity=-1;
			printf("REV\n");

		}
		reload_position();
    }

}




static void enable_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        start(1);
		if(mode == 1){
			reload_position();

			
		}else if(mode == 3){

		}
    }
}



static void qucistop_event_handler(lv_event_t * e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t * obj = lv_event_get_target(e);
	if(code == LV_EVENT_CLICKED) {
		quick_stop(1);
	}
}


static void reset_event_handler(lv_event_t * e)
{
	lv_event_code_t code = lv_event_get_code(e);
	lv_obj_t * obj = lv_event_get_target(e);
	if(code == LV_EVENT_CLICKED) {
		fault_reset(1);(1);
	}
}

static void stop_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
        shutdown(1);
    }
}
static void target_position_slider_event_cb(lv_event_t * e)
{
    lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
    char buf[128];
	target_position = 500000*(int)lv_slider_get_value(slider);
    lv_snprintf(buf, sizeof(buf), "%s :%d","target pos", target_position);
    lv_label_set_text(lable, buf);
    lv_obj_align_to(lable, slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
	reload_position();
}


static void abs_rel_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_VALUE_CHANGED) {
        char buf[32];
        lv_dropdown_get_selected_str(obj, buf, sizeof(buf));
        LV_LOG_USER("Option: %s", buf);
		if(strcasecmp("ABS",buf) == 0){
			abs_rel = 0;
		}else if(strcasecmp("REAL",buf) == 0){
			abs_rel=1;

		}
    }

}


static void profile_velocity_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	char buf[128];
	profile_velocity= 4500*(int)lv_slider_get_value(slider);
	lv_snprintf(buf, sizeof(buf), "%s :%d","profile speed", profile_velocity);
	lv_label_set_text(lable, buf);
	lv_obj_align_to(lable, slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
	configure_pp_profile_velocity(1,profile_velocity);
}

static void Profile_acceleration_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	char buf[128];
	Profile_acceleration= 5000*(int)lv_slider_get_value(slider);
	lv_snprintf(buf, sizeof(buf), "%s :%d","profile acceleration", Profile_acceleration);
	lv_label_set_text(lable, buf);
	lv_obj_align_to(lable, slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
	configure_pp_Profile_acceleration(1,Profile_acceleration);
}

static void Profile_deceleration_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	char buf[128];
	Profile_deceleration= 5000*(int)lv_slider_get_value(slider);
	lv_snprintf(buf, sizeof(buf), "%s :%d","profile deceleration", Profile_deceleration);
	lv_label_set_text(lable, buf);
	lv_obj_align_to(lable, slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
	configure_pp_Profile_deceleration(1,Profile_deceleration);
}

static void quick_stop_Deceleration_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	char buf[128];
	quick_stop_Deceleration= 5000*(int)lv_slider_get_value(slider);
	lv_snprintf(buf, sizeof(buf), "%s :%d","quick stop Deceleration", quick_stop_Deceleration);
	lv_label_set_text(lable, buf);
	lv_obj_align_to(lable, slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);
	configure_pp_quick_stop_Deceleration(1,quick_stop_Deceleration);
}


void* pp_thread(void *param)
{
	uint32 nValue ;
	uint32 l = sizeof(nValue);

	lv_obj_t *current_text = (lv_obj_t *)param;
	char buf[12]={0};
	
	while(1){

		if(ServoList[0].rpdo.bModeOperation == 1){
			sleep(1);
			ec_SDOread(1, 0x6064, 0, FALSE, &l, &nValue, 250000);
			lv_snprintf(buf,12,"%d",nValue);
			lv_textarea_set_text(current_text,buf);
		}
	}

}

int lvgl_screen_init()
{
	char buf[128];
	int ret=0;
    /*Properties to transition*/
    static lv_style_prop_t props[] = {
        LV_STYLE_TRANSFORM_WIDTH, LV_STYLE_TRANSFORM_HEIGHT, LV_STYLE_TEXT_LETTER_SPACE, 0
    };



	pthread_t tid;

	lv_init();



	lv_wayland_init();
	lv_disp_t * disp;
	disp = lv_wayland_create_window(LV_HOR_RES_MAX,LV_VER_RES_MAX,"lvgl ethercat demo",lv_wayland_close_window);


	lv_obj_t * logo = lv_img_create(lv_scr_act());
    LV_IMG_DECLARE(myir);
    lv_img_set_src(logo, &myir);
    lv_obj_align(logo, LV_ALIGN_LEFT_MID, 0, 0);

#if 0
	lv_obj_t * scr = lv_obj_create(NULL);
	lv_scr_load(scr);
    /*Create a normal drop down list*/
    lv_obj_t * mode_select = lv_dropdown_create(scr);
    lv_dropdown_set_options(mode_select, "(PP)\n(PV)");
    //lv_obj_set_pos(mode_select,0,0);
    lv_obj_add_event_cb(mode_select, mode_event_handler, LV_EVENT_ALL, NULL);





	/*Create a slider for target position*/
	lv_obj_t * target_position_slider = lv_slider_create(scr);
	lv_obj_align(target_position_slider,LV_ALIGN_LEFT_MID,0,-200);
	lv_slider_set_value(target_position_slider,target_position/500000,LV_ANIM_OFF);
	lv_snprintf(buf, sizeof(buf), "%s :%d","target pos",target_position);

	/*Create a label below the slider*/
	lv_obj_t *target_position_slider_label = lv_label_create(scr);
	lv_label_set_text(target_position_slider_label, buf);

	lv_obj_align_to(target_position_slider_label, target_position_slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);



	lv_obj_add_event_cb(target_position_slider, target_position_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void*)target_position_slider_label);


    /*Create a normal abs / rel list*/
    lv_obj_t * abs_rel_select = lv_dropdown_create(scr);
    lv_dropdown_set_options(abs_rel_select, "ABS\nREAL");
	lv_obj_align_to(abs_rel_select, target_position_slider, LV_ALIGN_OUT_TOP_LEFT, 0, -50);
    lv_dropdown_set_dir(abs_rel_select, LV_DIR_RIGHT);
    lv_dropdown_set_symbol(abs_rel_select, LV_SYMBOL_RIGHT);

    lv_obj_add_event_cb(abs_rel_select, abs_rel_event_handler, LV_EVENT_ALL, NULL);

    /*Create a normal fwd / rev list*/
    lv_obj_t * direction_select = lv_dropdown_create(scr);
    lv_dropdown_set_options(direction_select, "FWD\nREV");
	lv_obj_align_to(direction_select, target_position_slider, LV_ALIGN_OUT_RIGHT_MID, 50, 0);
    lv_dropdown_set_dir(direction_select, LV_DIR_RIGHT);
    lv_dropdown_set_symbol(direction_select, LV_SYMBOL_RIGHT);

    lv_obj_add_event_cb(direction_select, direction_event_handler, LV_EVENT_ALL, NULL);


	/*Create a slider for profile_velocity*/
	lv_obj_t * profile_velocity_slider = lv_slider_create(scr);
	lv_obj_align(profile_velocity_slider,LV_ALIGN_LEFT_MID,0,-100);
	lv_slider_set_value(profile_velocity_slider,profile_velocity/4500,LV_ANIM_OFF);
	lv_snprintf(buf, sizeof(buf), "%s :%d","profile speed",profile_velocity);

	lv_obj_t * profile_velocity_label = lv_label_create(scr);
	lv_label_set_text(profile_velocity_label, buf);
	lv_obj_align_to(profile_velocity_label, profile_velocity_slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

	lv_obj_add_event_cb(profile_velocity_slider, profile_velocity_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void*)profile_velocity_label);

	/*Create a slider for Profile_acceleration*/
	lv_obj_t * Profile_acceleration_slider = lv_slider_create(scr);
	lv_obj_align(Profile_acceleration_slider,LV_ALIGN_LEFT_MID,0,-50);
	lv_slider_set_value(Profile_acceleration_slider,Profile_acceleration/5000,LV_ANIM_OFF);
	lv_snprintf(buf, sizeof(buf), "%s :%d","profile acceleration",Profile_acceleration);

	lv_obj_t * Profile_acceleration_label = lv_label_create(scr);
	lv_label_set_text(Profile_acceleration_label, buf);
	lv_obj_align_to(Profile_acceleration_label, Profile_acceleration_slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

	lv_obj_add_event_cb(Profile_acceleration_slider, Profile_acceleration_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void*)Profile_acceleration_label);

	
	/*Create a slider for Profile_deceleration*/
	lv_obj_t * Profile_deceleration_slider = lv_slider_create(scr);
	lv_obj_align_to(Profile_deceleration_slider, Profile_acceleration_slider, LV_ALIGN_OUT_RIGHT_MID, 50, 0);
	//lv_obj_align(Profile_deceleration_slider,LV_ALIGN_LEFT_MID,300,-50);
	lv_slider_set_value(Profile_deceleration_slider,Profile_deceleration/5000,LV_ANIM_OFF);
	lv_snprintf(buf, sizeof(buf), "%s :%d","Profile deceleration",Profile_acceleration);

	lv_obj_t * Profile_deceleration_label = lv_label_create(scr);
	lv_label_set_text(Profile_deceleration_label, buf);
	lv_obj_align_to(Profile_deceleration_label, Profile_deceleration_slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

	lv_obj_add_event_cb(Profile_deceleration_slider, Profile_deceleration_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void*)Profile_deceleration_label);


	/*Create a slider for quick_stop_Deceleration*/
	lv_obj_t * quick_stop_Deceleration_slider = lv_slider_create(scr);
	lv_obj_align_to(quick_stop_Deceleration_slider, Profile_deceleration_slider, LV_ALIGN_OUT_RIGHT_MID, 50, 0);
	//lv_obj_align(Profile_deceleration_slider,LV_ALIGN_LEFT_MID,300,-50);
	lv_slider_set_value(quick_stop_Deceleration_slider,quick_stop_Deceleration/5000,LV_ANIM_OFF);
	lv_snprintf(buf, sizeof(buf), "%s :%d","quick stop Deceleration",quick_stop_Deceleration);

	lv_obj_t * quick_stop_Deceleration_label = lv_label_create(scr);
	lv_label_set_text(quick_stop_Deceleration_label, buf);
	lv_obj_align_to(quick_stop_Deceleration_label, quick_stop_Deceleration_slider, LV_ALIGN_OUT_TOP_LEFT, 0, 0);

	lv_obj_add_event_cb(quick_stop_Deceleration_slider, quick_stop_Deceleration_slider_event_cb, LV_EVENT_VALUE_CHANGED, (void*)quick_stop_Deceleration_label);



    /*Transition descriptor when going back to the default state.
     *Add some delay to be sure the press transition is visible even if the press was very short*/
    static lv_style_transition_dsc_t transition_dsc_def;
    lv_style_transition_dsc_init(&transition_dsc_def, props, lv_anim_path_overshoot, 250, 100, NULL);

    /*Transition descriptor when going to pressed state.
     *No delay, go to presses state immediately*/
    static lv_style_transition_dsc_t transition_dsc_pr;
    lv_style_transition_dsc_init(&transition_dsc_pr, props, lv_anim_path_ease_in_out, 250, 0, NULL);

    /*Add only the new transition to he default state*/
    static lv_style_t style_def;
    lv_style_init(&style_def);
    lv_style_set_transition(&style_def, &transition_dsc_def);

    /*Add the transition and some transformation to the presses state.*/
    static lv_style_t style_pr;
    lv_style_init(&style_pr);
    lv_style_set_transform_width(&style_pr, 10);
    lv_style_set_transform_height(&style_pr, -10);
    lv_style_set_text_letter_space(&style_pr, 10);
    lv_style_set_transition(&style_pr, &transition_dsc_pr);


	/* enable btn */
	lv_obj_t * enable_btn = lv_btn_create(scr);
	lv_obj_t * enable_label = lv_label_create(enable_btn);

	lv_label_set_text(enable_label,"enable");
	lv_obj_center(enable_label);
	lv_obj_add_style(enable_btn, &style_pr, LV_STATE_PRESSED);
	 lv_obj_add_style(enable_btn, &style_def, 0);

	lv_obj_align(enable_btn, LV_ALIGN_LEFT_MID, 0, 40);
	lv_obj_add_event_cb(enable_btn,enable_event_handler,LV_EVENT_CLICKED,NULL);


	/* stop btn */
	lv_obj_t * stop_btn = lv_btn_create(scr);
	lv_obj_t * stop_label = lv_label_create(stop_btn);

	lv_label_set_text(stop_label,"stop");
	lv_obj_center(stop_label);

	lv_obj_align_to(stop_btn,enable_btn,LV_ALIGN_OUT_RIGHT_MID,50,0);
	lv_obj_add_event_cb(stop_btn,stop_event_handler,LV_EVENT_CLICKED,NULL);



	/* quick btn */
	lv_obj_t * quick_btn = lv_btn_create(scr);
	lv_obj_t * quick_stop_label = lv_label_create(quick_btn);

	lv_label_set_text(quick_stop_label,"quick stop");
	lv_obj_center(quick_stop_label);

	lv_obj_align_to(quick_btn,stop_btn,LV_ALIGN_OUT_RIGHT_MID,50,0);
	lv_obj_add_event_cb(quick_btn,qucistop_event_handler,LV_EVENT_CLICKED,NULL);

	/* reset btn */
	lv_obj_t * reset_btn = lv_btn_create(scr);
	lv_obj_t * reset_label = lv_label_create(reset_btn);

	lv_label_set_text(reset_label,"reset");
	lv_obj_center(reset_label);

	lv_obj_align_to(reset_btn,quick_btn,LV_ALIGN_OUT_RIGHT_MID,50,0);
	lv_obj_add_event_cb(reset_btn,reset_event_handler,LV_EVENT_CLICKED,NULL);


    lv_obj_t * current_position_label = lv_label_create(scr);
    lv_label_set_text(current_position_label, "current_position");
	lv_obj_align(current_position_label, LV_ALIGN_LEFT_MID, 0, 80);

    lv_obj_t * current_position_text = lv_textarea_create(scr);
    lv_textarea_set_one_line(current_position_text, true);
    lv_obj_align_to(current_position_text,current_position_label,LV_ALIGN_LEFT_MID,50,0);
    
	pthread_create(&tid,NULL,pp_thread,(void *)current_position_text);

	LV_FONT_DECLARE(lv_font_myfont_28);

/*
	lv_obj_t * test_bt = lv_btn_create(scr);

	lv_obj_t * test = lv_label_create(test_bt);
	 lv_obj_set_style_text_font(test, &lv_font_myfont_28,0);
	lv_label_set_text(test,"ţºb);
	lv_obj_align(test_bt,LV_ALIGN_BOTTOM_MID,0,-200);
*/

#endif






	return 0;
}

int main(int argc, char* argv[])
{

	int ret = -1;
	ret = ethercat_init(argc, argv);
	if(ret != 0){
		printf("ether init fail!\n");
		exit(1);
	}
	

	ret = lvgl_screen_init();
	if(ret != 0){
		printf("lvgl_screen_init init fail!\n");
		exit(1);
	}




	while(1){

		lv_task_handler();
  		usleep(5000);
		lv_tick_inc(5);

	}

    return 0;
}


