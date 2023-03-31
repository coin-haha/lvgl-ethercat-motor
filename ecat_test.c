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


typedef enum {
    DISP_SMALL,
    DISP_MEDIUM,
    DISP_LARGE,
} disp_size_t;
enum {
    LV_MENU_ITEM_BUILDER_VARIANT_1,
    LV_MENU_ITEM_BUILDER_VARIANT_2
};
typedef uint8_t lv_menu_builder_variant_t;


static disp_size_t disp_size;

static const lv_font_t * font_large;
static const lv_font_t * font_normal;

static lv_obj_t * tv;
static lv_obj_t * calendar;
static lv_style_t style_text_muted;
static lv_style_t style_title;
static lv_style_t style_icon;
static lv_style_t style_bullet;
static lv_style_t style_roller;

static lv_obj_t *root_page;
static lv_obj_t * kb;
static lv_obj_t *current_position_text;
static lv_obj_t *current_speed_text;
static lv_obj_t * meter;


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
int profile_velocity = 150000;
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



static void direction_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);


	if(code == LV_EVENT_VALUE_CHANGED) {
		 LV_LOG_USER("State: %s\n", lv_obj_has_state(obj, LV_STATE_CHECKED) ? "On" : "Off");
		 if(lv_obj_has_state(obj, LV_STATE_CHECKED)){
			polarity=-1;
		 }else{
			polarity=1;
		 }
		 reload_position();
	 }



}

static void rev_enable_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
		abs_rel = 1;
		configure_pp_mode(1);
        start(1);
		reload_position();


    }
}



static void abs_enable_event_handler(lv_event_t * e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * obj = lv_event_get_target(e);
    if(code == LV_EVENT_CLICKED) {
		abs_rel = 0;
		configure_pp_mode(1);
        start(1);
		reload_position();


    }
}



static void quickstop_event_handler(lv_event_t * e)
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
		fault_reset(1);
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





static void profile_velocity_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	profile_velocity= (int)lv_slider_get_value(slider);
	
	lv_label_set_text_fmt(lable,"%d",profile_velocity);
	configure_pp_profile_velocity(1,profile_velocity);
}

static void Profile_acceleration_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	Profile_acceleration= (int)lv_slider_get_value(slider);
	
	lv_label_set_text_fmt(lable,"%d" ,Profile_acceleration);
	configure_pp_Profile_acceleration(1,Profile_acceleration);
}

static void Profile_deceleration_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	Profile_deceleration= (int)lv_slider_get_value(slider);
	
	lv_label_set_text_fmt(lable,"%d" ,Profile_deceleration);
	configure_pp_Profile_deceleration(1,Profile_deceleration);
}

static void quick_stop_Deceleration_slider_event_cb(lv_event_t * e)
{
	lv_obj_t * slider = lv_event_get_target(e);
	lv_obj_t * lable = (lv_obj_t *)lv_event_get_user_data(e);
	quick_stop_Deceleration= (int)lv_slider_get_value(slider);
	
	lv_label_set_text_fmt(lable,"%d", quick_stop_Deceleration);
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
			
			ec_SDOread(1, 0x6064, 0, FALSE, &l, &nValue, 250000);
			lv_snprintf(buf,12,"%d",nValue);
			lv_textarea_set_text(current_position_text,buf);

		
			ec_SDOread(1, 0x606c, 0, FALSE, &l, &nValue, 250000);
			lv_snprintf(buf,12,"%d",nValue);
			lv_textarea_set_text(current_speed_text,buf);


			sleep(1);
		}
	}

}
static lv_obj_t * create_text(lv_obj_t * parent, const char * icon, const char * txt,
                                        int *val,lv_menu_builder_variant_t builder_variant)
{
    lv_obj_t * obj = lv_menu_cont_create(parent);

    lv_obj_t * img = NULL;
    lv_obj_t * label = NULL;

    if(icon) {
        img = lv_img_create(obj);
        lv_img_set_src(img, icon);
    }

    if(txt) {
        label = lv_label_create(obj);
		if(val)
			lv_label_set_text_fmt(label,"%s:%d",txt,*val);
		else
        	lv_label_set_text(label, txt);
        lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
        lv_obj_set_flex_grow(label, 1);
    }

    if(builder_variant == LV_MENU_ITEM_BUILDER_VARIANT_2 && icon && txt) {
        lv_obj_add_flag(img, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
        lv_obj_swap(img, label);
    }

    return obj;
}

static void set_btn_style(lv_obj_t * btn)
{
    /*Init the style for the default state*/
    static lv_style_t style;
    lv_style_init(&style);

    lv_style_set_radius(&style, 3);

    lv_style_set_bg_opa(&style, LV_OPA_100);
    lv_style_set_bg_color(&style, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_bg_grad_color(&style, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_dir(&style, LV_GRAD_DIR_VER);

    lv_style_set_border_opa(&style, LV_OPA_40);
    lv_style_set_border_width(&style, 2);
    lv_style_set_border_color(&style, lv_palette_main(LV_PALETTE_GREY));

    lv_style_set_shadow_width(&style, 8);
    lv_style_set_shadow_color(&style, lv_palette_main(LV_PALETTE_GREY));
    lv_style_set_shadow_ofs_y(&style, 8);

    lv_style_set_outline_opa(&style, LV_OPA_COVER);
    lv_style_set_outline_color(&style, lv_palette_main(LV_PALETTE_BLUE));

    lv_style_set_text_color(&style, lv_color_white());
    lv_style_set_pad_all(&style, 10);

    /*Init the pressed style*/
    static lv_style_t style_pr;
    lv_style_init(&style_pr);

    /*Add a large outline when pressed*/
    lv_style_set_outline_width(&style_pr, 30);
    lv_style_set_outline_opa(&style_pr, LV_OPA_TRANSP);

    lv_style_set_translate_y(&style_pr, 5);
    lv_style_set_shadow_ofs_y(&style_pr, 3);
    lv_style_set_bg_color(&style_pr, lv_palette_darken(LV_PALETTE_BLUE, 2));
    lv_style_set_bg_grad_color(&style_pr, lv_palette_darken(LV_PALETTE_BLUE, 4));

    /*Add a transition to the outline*/
    static lv_style_transition_dsc_t trans;
    static lv_style_prop_t props[] = {LV_STYLE_OUTLINE_WIDTH, LV_STYLE_OUTLINE_OPA, 0};
    lv_style_transition_dsc_init(&trans, props, lv_anim_path_linear, 300, 0, NULL);

    lv_style_set_transition(&style_pr, &trans);

    lv_obj_remove_style_all(btn);                          /*Remove the style coming from the theme*/
    lv_obj_add_style(btn, &style, 0);
    lv_obj_add_style(btn, &style_pr, LV_STATE_PRESSED);
    lv_obj_set_size(btn, LV_SIZE_CONTENT, LV_SIZE_CONTENT);



}


static void set_slider_style(lv_obj_t * slider)
{
    /*Create a transition*/
    static const lv_style_prop_t props[] = {LV_STYLE_BG_COLOR, 0};
    static lv_style_transition_dsc_t transition_dsc;
    lv_style_transition_dsc_init(&transition_dsc, props, lv_anim_path_linear, 300, 0, NULL);

    static lv_style_t style_main;
    static lv_style_t style_indicator;
    static lv_style_t style_knob;
    static lv_style_t style_pressed_color;
    lv_style_init(&style_main);
    lv_style_set_bg_opa(&style_main, LV_OPA_COVER);
    lv_style_set_bg_color(&style_main, lv_color_hex3(0xbbb));
    lv_style_set_radius(&style_main, LV_RADIUS_CIRCLE);
    lv_style_set_pad_ver(&style_main, -2); /*Makes the indicator larger*/

    lv_style_init(&style_indicator);
    lv_style_set_bg_opa(&style_indicator, LV_OPA_COVER);
    lv_style_set_bg_color(&style_indicator, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_radius(&style_indicator, LV_RADIUS_CIRCLE);
    lv_style_set_transition(&style_indicator, &transition_dsc);

    lv_style_init(&style_knob);
    lv_style_set_bg_opa(&style_knob, LV_OPA_COVER);
    lv_style_set_bg_color(&style_knob, lv_palette_main(LV_PALETTE_LIGHT_GREEN));
    lv_style_set_border_color(&style_knob, lv_palette_darken(LV_PALETTE_RED, 3));
    lv_style_set_border_width(&style_knob, 2);
    lv_style_set_radius(&style_knob, LV_RADIUS_CIRCLE);
    lv_style_set_pad_all(&style_knob, 6); /*Makes the knob larger*/
    lv_style_set_transition(&style_knob, &transition_dsc);

    lv_style_init(&style_pressed_color);
    lv_style_set_bg_color(&style_pressed_color, lv_palette_darken(LV_PALETTE_CYAN, 2));

    /*Create a slider and add the style*/
    lv_obj_remove_style_all(slider);        /*Remove the styles coming from the theme*/

    lv_obj_add_style(slider, &style_main, LV_PART_MAIN);
    lv_obj_add_style(slider, &style_indicator, LV_PART_INDICATOR);
    lv_obj_add_style(slider, &style_pressed_color, LV_PART_INDICATOR | LV_STATE_PRESSED);
    lv_obj_add_style(slider, &style_knob, LV_PART_KNOB);
    lv_obj_add_style(slider, &style_pressed_color, LV_PART_KNOB | LV_STATE_PRESSED);



}

static lv_obj_t * create_slider(lv_obj_t * parent,  const char * txt, int32_t min, int32_t max, int32_t* val, const char *unit,void (*fun )(lv_obj_t *) )
{

    lv_obj_t * obj = lv_menu_cont_create(parent);

    lv_obj_t * label = NULL;
	lv_obj_t * label_val = NULL;
	lv_obj_t * label_unit = NULL;



    label = lv_label_create(obj);

	lv_label_set_text(label, txt);

	label_val = lv_label_create(obj);
	lv_label_set_text_fmt(label_val,":%d",*val);

	label_unit = lv_label_create(obj);
	lv_label_set_text(label_unit, unit);

	# if 0
	if(val)
		lv_label_set_text_fmt(label,"%s:%d",txt,*val);
	else
    	lv_label_set_text(label, txt);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_flex_grow(label, 1);
	#endif
    

    
	obj = lv_menu_cont_create(parent);



    lv_obj_t * slider = lv_slider_create(obj);
    lv_obj_set_flex_grow(slider, 1);
    lv_slider_set_range(slider, min, max);
    lv_slider_set_value(slider, *val, LV_ANIM_OFF);
	set_slider_style(slider);
	lv_obj_add_event_cb(slider,fun,LV_EVENT_VALUE_CHANGED,label_val);

//    if(icon == NULL) {
 //       lv_obj_add_flag(slider, LV_OBJ_FLAG_FLEX_IN_NEW_TRACK);
  //  }

    return obj;
}

static void textarea_event_handler(lv_event_t * e)
{

    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t * ta = lv_event_get_target(e);
	lv_obj_t * kb = lv_event_get_user_data(e);

	if(code == LV_EVENT_CLICKED || code == LV_EVENT_FOCUSED) {
        /*Focus on the clicked text area*/
		lv_obj_clear_flag(kb,LV_OBJ_FLAG_HIDDEN);
        if(kb != NULL) lv_keyboard_set_textarea(kb, ta);
    }else if(code == LV_EVENT_READY){
		lv_obj_add_flag(kb,LV_OBJ_FLAG_HIDDEN);
		LV_LOG_USER("Ready, current text: %s", lv_textarea_get_text(ta));

		target_position = atoi(lv_textarea_get_text(ta));
		reload_position();

		
    }


}




void meter_create(lv_obj_t * parent)
{
    meter = lv_meter_create(parent);
    lv_obj_set_size(meter, 200, 200);
    /*Add a scale first*/
    lv_meter_scale_t * scale = lv_meter_add_scale(meter);
    lv_meter_set_scale_ticks(meter, scale, 41, 2, 10, lv_palette_main(LV_PALETTE_GREY));
    lv_meter_set_scale_major_ticks(meter, scale, 8, 4, 15, lv_color_black(), 10);

	lv_meter_indicator_t * indic;

	/*Add a blue arc to the start*/
	indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_BLUE), 0);
	lv_meter_set_indicator_start_value(meter, indic, 0);
	lv_meter_set_indicator_end_value(meter, indic, 20);

	/*Make the tick lines blue at the start of the scale*/
	indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_BLUE), false, 0);
	lv_meter_set_indicator_start_value(meter, indic, 0);
	lv_meter_set_indicator_end_value(meter, indic, 20);

	/*Add a red arc to the end*/
	indic = lv_meter_add_arc(meter, scale, 3, lv_palette_main(LV_PALETTE_RED), 0);
	lv_meter_set_indicator_start_value(meter, indic, 80);
	lv_meter_set_indicator_end_value(meter, indic, 100);

	/*Make the tick lines red at the end of the scale*/
	indic = lv_meter_add_scale_lines(meter, scale, lv_palette_main(LV_PALETTE_RED), lv_palette_main(LV_PALETTE_RED), false, 0);
	lv_meter_set_indicator_start_value(meter, indic, 80);
	lv_meter_set_indicator_end_value(meter, indic, 100);

	/*Add a needle line indicator*/
	indic = lv_meter_add_needle_line(meter, scale, 4, lv_palette_main(LV_PALETTE_GREY), -10);




}


static void pp_create(lv_obj_t * parent)
{

    

	

	lv_obj_t *panel1 = lv_obj_create(parent);
	//lv_obj_set_height(panel1,LV_SIZE_CONTENT);
	//lv_obj_set_width(panel1,LV_HOR_RES);
	lv_obj_set_width(panel1,lv_pct(100));
	lv_obj_set_height(panel1,lv_pct(70));

	lv_obj_add_style(panel1,&style_bullet,0);

    lv_obj_t * menu = lv_menu_create(panel1);
	lv_obj_set_height(menu,LV_VER_RES);
	lv_obj_set_width(menu,LV_HOR_RES);
#if 1
    lv_color_t bg_color = lv_obj_get_style_bg_color(menu, 0);
    if(lv_color_brightness(bg_color) > 127) {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 10), 0);
    }
    else {
        lv_obj_set_style_bg_color(menu, lv_color_darken(lv_obj_get_style_bg_color(menu, 0), 50), 0);
    }
#endif
	//lv_obj_center(menu);

	lv_obj_t * cont;
	lv_obj_t * section;
	lv_obj_t * obj_t;
	lv_obj_t * text_l;
	lv_obj_t * btn_enable;
	lv_obj_t * label_enable;
	lv_obj_t * btn_stop;
	lv_obj_t * label_stop;
	lv_obj_t * btn_quickstop;
	lv_obj_t * label_quickstop;
	lv_obj_t * btn_reset;
	lv_obj_t * label_reset;
	lv_obj_t * sw;

	/*Create pp rev pages*/
	lv_obj_t * sub_pp_rev_page = lv_menu_page_create(menu, NULL);
	lv_obj_set_style_pad_hor(sub_pp_rev_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
	lv_menu_separator_create(sub_pp_rev_page);
	section = lv_menu_section_create(sub_pp_rev_page);
	
	/* create 距离 label */
	obj_t = lv_menu_cont_create(section);
	text_l = lv_label_create(obj_t);
	lv_label_set_text(text_l,"距离");
	lv_obj_add_style(text_l,font_normal,0);

	/*创建 输入 距离框 */
	obj_t = lv_menu_cont_create(section);
	lv_obj_t * ta = lv_textarea_create(obj_t);
    lv_textarea_set_one_line(ta, true);
    lv_obj_align_to(ta, text_l,LV_ALIGN_OUT_RIGHT_MID, 10, 0);
  

    /*Create a keyboard*/
    kb = lv_keyboard_create(obj_t);
    lv_obj_set_size(kb,  200, 150);
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(kb, ta);
	lv_obj_add_style(kb,&style_title,LV_PART_ITEMS );
	//lv_obj_align(kb,LV_ALIGN_RIGHT_MID,-50,0);
	lv_obj_add_flag(kb,LV_OBJ_FLAG_HIDDEN);

	lv_obj_add_event_cb(ta, textarea_event_handler, LV_EVENT_ALL, kb);
	lv_obj_add_state(ta, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/

		/* 正转/反转 */
	obj_t = lv_menu_cont_create(section);
	text_l = lv_label_create(obj_t);
	lv_label_set_text(text_l,"正转");
	lv_obj_add_style(text_l,font_normal,0);

	sw = lv_switch_create(obj_t);
	lv_obj_add_state(sw, polarity ? 0 : LV_STATE_CHECKED );
	lv_obj_add_event_cb(sw,direction_event_handler,LV_EVENT_ALL,sw);
	
	text_l = lv_label_create(obj_t);
	lv_label_set_text(text_l,"反转");
	lv_obj_add_style(text_l,font_normal,0);

	create_slider(section, "轮廓速度:", 0, 450000, &profile_velocity,"u/s",profile_velocity_slider_event_cb);
	create_slider(section, "轮廓加速度:", 0, 500000, &Profile_acceleration,"u/^s",Profile_acceleration_slider_event_cb);
	create_slider(section, "轮廓减速度:", 0, 500000, &Profile_deceleration,"u/^s",Profile_deceleration_slider_event_cb);
	create_slider(section, "快速停止加速度:", 0 ,500000,&quick_stop_Deceleration,"u/^s",quick_stop_Deceleration_slider_event_cb);

	/* add enable button */
	obj_t = lv_menu_cont_create(section);
	btn_enable = lv_btn_create(obj_t);
	set_btn_style(btn_enable);
	label_enable = lv_label_create(btn_enable);
	lv_label_set_text(label_enable,"使能");
	lv_obj_add_event_cb(btn_enable,rev_enable_event_handler,LV_EVENT_CLICKED,NULL);


	/* add stop button */
	btn_stop = lv_btn_create(obj_t);
	set_btn_style(btn_stop);
	label_stop = lv_label_create(btn_stop);
	lv_label_set_text(label_stop,"暂停");
	lv_obj_add_event_cb(btn_stop,stop_event_handler,LV_EVENT_CLICKED,NULL);

	/* add quick stop button */
	btn_quickstop = lv_btn_create(obj_t);
	set_btn_style(btn_quickstop);
	label_quickstop = lv_label_create(btn_quickstop);
	lv_label_set_text(label_quickstop,"急停");
	lv_obj_add_event_cb(btn_quickstop,quickstop_event_handler,LV_EVENT_CLICKED,NULL);

	/* add reset button */
	btn_reset = lv_btn_create(obj_t);
	set_btn_style(btn_reset);
	label_reset = lv_label_create(btn_reset);
	lv_label_set_text(label_reset,"复位");
	lv_obj_add_event_cb(btn_reset,reset_event_handler,LV_EVENT_CLICKED,NULL);


	/* pp abs page */
	lv_obj_t * sub_pp_abs_page =  lv_menu_page_create(menu, NULL);
	lv_obj_set_style_pad_hor(sub_pp_abs_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
	lv_menu_separator_create(sub_pp_abs_page);
	section = lv_menu_section_create(sub_pp_abs_page);

		/* create 位置 label */
	obj_t = lv_menu_cont_create(section);
	text_l = lv_label_create(obj_t);
	lv_label_set_text(text_l,"位置");
	lv_obj_add_style(text_l,font_normal,0);




	/*创建 输入 位置框 */
	obj_t = lv_menu_cont_create(section);
	lv_obj_t * abs_ta = lv_textarea_create(obj_t);
    lv_textarea_set_one_line(abs_ta, true);
    lv_obj_align_to(abs_ta, text_l,LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    lv_obj_add_state(abs_ta, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/

	lv_obj_add_event_cb(abs_ta, textarea_event_handler, LV_EVENT_ALL, kb);
	lv_obj_add_state(abs_ta, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/



    /*Create a keyboard*/
    kb = lv_keyboard_create(obj_t);
    lv_obj_set_size(kb,  200, 150);
    lv_keyboard_set_mode(kb, LV_KEYBOARD_MODE_NUMBER);
    lv_keyboard_set_textarea(kb, abs_ta);
	lv_obj_add_style(kb,&style_title,LV_PART_ITEMS );
	lv_obj_add_flag(kb,LV_OBJ_FLAG_HIDDEN);

	lv_obj_add_event_cb(abs_ta, textarea_event_handler, LV_EVENT_ALL, kb);
	lv_obj_add_state(abs_ta, LV_STATE_FOCUSED); /*To be sure the cursor is visible*/


	create_slider(section, "轮廓速度:", 0, 450000, &profile_velocity,"u/s",profile_velocity_slider_event_cb);
	create_slider(section, "轮廓加速度:", 0, 500000, &Profile_acceleration,"u/^s",Profile_acceleration_slider_event_cb);
	create_slider(section, "轮廓减速度:", 0, 500000, &Profile_deceleration,"u/^s",Profile_deceleration_slider_event_cb);
	create_slider(section, "快速停止加速度:", 0 ,500000,&quick_stop_Deceleration,"u/^s",quick_stop_Deceleration_slider_event_cb);



	/* add enable button */
	obj_t = lv_menu_cont_create(section);
	btn_enable = lv_btn_create(obj_t);
	set_btn_style(btn_enable);
	label_enable = lv_label_create(btn_enable);
	lv_label_set_text(label_enable,"使能");
	lv_obj_add_event_cb(btn_enable,abs_enable_event_handler,LV_EVENT_CLICKED,NULL);


	/* add stop button */
	btn_stop = lv_btn_create(obj_t);
	set_btn_style(btn_stop);
	label_stop = lv_label_create(btn_stop);
	lv_label_set_text(label_stop,"暂停");
	lv_obj_add_event_cb(btn_stop,stop_event_handler,LV_EVENT_CLICKED,NULL);

	/* add quick stop button */
	btn_quickstop = lv_btn_create(obj_t);
	set_btn_style(btn_quickstop);
	label_quickstop = lv_label_create(btn_quickstop);
	lv_label_set_text(label_quickstop,"急停");
	lv_obj_add_event_cb(btn_quickstop,quickstop_event_handler,LV_EVENT_CLICKED,NULL);

	/* add reset button */
	btn_reset = lv_btn_create(obj_t);
	set_btn_style(btn_reset);
	label_reset = lv_label_create(btn_reset);
	lv_label_set_text(label_reset,"复位");
	lv_obj_add_event_cb(btn_reset,reset_event_handler,LV_EVENT_CLICKED,NULL);


    /*Create a root page*/
    root_page = lv_menu_page_create(menu,  "轮廓运动模式");
    lv_obj_set_style_pad_hor(root_page, lv_obj_get_style_pad_left(lv_menu_get_main_header(menu), 0), 0);
    section = lv_menu_section_create(root_page);
    cont = create_text(section, LV_SYMBOL_SETTINGS, "相对给定", NULL,LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_pp_rev_page);
    cont = create_text(section, LV_SYMBOL_AUDIO, "绝对给定", NULL,LV_MENU_ITEM_BUILDER_VARIANT_1);
    lv_menu_set_load_page_event(menu, cont, sub_pp_abs_page);


   lv_obj_t *panel2 = lv_obj_create(parent);
   lv_obj_add_style(panel2,&style_bullet,0);
   lv_obj_align_to(panel2,panel1,LV_ALIGN_OUT_BOTTOM_LEFT,0,0);
   lv_obj_set_width(panel2,lv_pct(100));
   lv_obj_set_height(panel2,lv_pct(30));
	
	lv_obj_t * current_position_label = lv_label_create(panel2);
	lv_label_set_text(current_position_label,"当前位置");
	lv_obj_align(current_position_label,LV_ALIGN_LEFT_MID,0,0);


   current_position_text = lv_textarea_create(panel2);
   lv_textarea_set_one_line(current_position_text, true);
   lv_obj_align_to(current_position_text,current_position_label,LV_ALIGN_OUT_RIGHT_MID,50,0);


   	lv_obj_t * current_speed_label = lv_label_create(panel2);
	lv_label_set_text(current_speed_label,"当前速度");
	lv_obj_align(current_speed_label,LV_ALIGN_CENTER,0,0);


   current_speed_text = lv_textarea_create(panel2);
   lv_textarea_set_one_line(current_speed_text, true);
   lv_obj_align_to(current_speed_text,current_speed_label,LV_ALIGN_OUT_RIGHT_MID,50,0);

	//meter_create(panel2);
  	// lv_obj_align_to(meter,current_speed_text,LV_ALIGN_OUT_RIGHT_MID,50,0);


   lv_menu_set_sidebar_page(menu, root_page);
   
	lv_event_send(lv_obj_get_child(lv_obj_get_child(lv_menu_get_cur_sidebar_page(menu), 0), 0), LV_EVENT_CLICKED, NULL);
	

}
static void pv_create(lv_obj_t * parent)
{
	lv_obj_t *panel1 = lv_obj_create(parent);
	lv_obj_set_height(panel1,LV_SIZE_CONTENT);

	lv_obj_t * name = lv_label_create(panel1);
    lv_label_set_text(name, "设计中ing");
    lv_obj_add_style(name, &style_text_muted, 0);


}

LV_FONT_DECLARE(lv_font_msyh_16)

void lv_ethercat_demo_widgets()
{

    if(LV_HOR_RES <= 320) disp_size = DISP_SMALL;
    else if(LV_HOR_RES < 720) disp_size = DISP_MEDIUM;
    else disp_size = DISP_LARGE;

	font_large=LV_FONT_DEFAULT;
	
	font_normal=&lv_font_msyh_16;

	
	lv_theme_default_init(NULL, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), LV_THEME_DEFAULT_DARK,font_normal);

	lv_style_init(&style_text_muted);
	lv_style_set_text_opa(&style_text_muted, LV_OPA_80);
	lv_style_set_text_font(&style_text_muted, font_normal);

	lv_style_init(&style_title);
	lv_style_set_text_font(&style_title, font_large);

	lv_style_init(&style_icon);
	lv_style_set_text_color(&style_icon, lv_theme_get_color_primary(NULL));
	lv_style_set_text_font(&style_icon, font_large);

	lv_style_init(&style_bullet);
	lv_style_set_border_width(&style_bullet, 0);
	lv_style_set_radius(&style_bullet, LV_RADIUS_CIRCLE);

	lv_style_init(&style_roller);
    lv_style_set_bg_color(&style_roller, lv_color_white());
    lv_style_set_text_color(&style_roller, lv_palette_main(LV_PALETTE_BLUE));
    lv_style_set_border_width(&style_roller, 3);
    lv_style_set_pad_all(&style_roller, 0);


	tv = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, 70);

	lv_obj_set_style_text_font(lv_scr_act(), font_normal, 0);


	if(disp_size == DISP_LARGE) {
		lv_obj_t * tab_btns = lv_tabview_get_tab_btns(tv);
		lv_obj_set_style_pad_left(tab_btns, LV_HOR_RES / 2, 0);
		lv_obj_t * logo = lv_img_create(tab_btns);
		LV_IMG_DECLARE(img_myir_logo);
		lv_img_set_src(logo, &img_myir_logo);
		lv_obj_align(logo, LV_ALIGN_LEFT_MID, -LV_HOR_RES / 2 + 25, 0);

		lv_obj_t * label = lv_label_create(tab_btns);
		lv_obj_add_style(label, &style_title, 0);
		lv_label_set_text(label, "LVGL v8");
		lv_obj_align_to(label, logo, LV_ALIGN_OUT_RIGHT_TOP, 10, 0);

		label = lv_label_create(tab_btns);
		lv_label_set_text(label, "ethercat demo");
		lv_obj_add_style(label, &style_text_muted, 0);
		lv_obj_align_to(label, logo, LV_ALIGN_OUT_RIGHT_BOTTOM, 10, 0);
	}
    lv_obj_t * tab_pp = lv_tabview_add_tab(tv, "轮廓位置模式(PP)");
    lv_obj_t * tab_pv = lv_tabview_add_tab(tv, "轮廓速度模式(PV)");
    
	pp_create(tab_pp);
	pv_create(tab_pv);


}

int lvgl_screen_init()
{
	int ret=0;
    /*Properties to transition*/
    static lv_style_prop_t props[] = {
        LV_STYLE_TRANSFORM_WIDTH, LV_STYLE_TRANSFORM_HEIGHT, LV_STYLE_TEXT_LETTER_SPACE, 0
    };



	pthread_t tid;

	lv_init();

#if USE_WAYLAND == 1
	lv_wayland_init();
	lv_disp_t * disp;
	disp = lv_wayland_create_window(LV_HOR_RES_MAX,LV_VER_RES_MAX,"lvgl ethercat demo",lv_wayland_close_window);

#endif


#if USE_FBDEV == 1
	#define DISP_BUF_SIZE (LV_HOR_RES_MAX*LV_VER_RES_MAX/10)
	/*Linux frame buffer device init*/
    fbdev_init();

    /*A small buffer for LittlevGL to draw the screen's content*/
    static lv_color_t buf[DISP_BUF_SIZE];

    /*Initialize a descriptor for the buffer*/
    static lv_disp_draw_buf_t disp_buf;
    lv_disp_draw_buf_init(&disp_buf, buf, NULL, DISP_BUF_SIZE);

    /*Initialize and register a display driver*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.draw_buf   = &disp_buf;
    disp_drv.flush_cb   = fbdev_flush;
    disp_drv.hor_res    = LV_HOR_RES_MAX;
    disp_drv.ver_res    = LV_VER_RES_MAX;
    lv_disp_drv_register(&disp_drv);

#endif
	
	lv_ethercat_demo_widgets();

	//lv_obj_t * logo = lv_img_create(lv_scr_act());
    //LV_IMG_DECLARE(img_myir_logo);
   // lv_img_set_src(logo, &img_myir_logo);
    //lv_obj_align(logo, LV_ALIGN_LEFT_MID, 0, 0);







	return 0;
}

int main(int argc, char* argv[])
{

	int ret = -1;
	pthread_t tid;
	#if 1
	ret = ethercat_init(argc, argv);
	if(ret != 0){
		printf("ether init fail!\n");
		exit(1);
	}
	#endif

	ret = lvgl_screen_init();
	if(ret != 0){
		printf("lvgl_screen_init init fail!\n");
		exit(1);
	}
	pthread_create(&tid,NULL,pp_thread,(void *)current_position_text);




	while(1){

		lv_task_handler();
  		usleep(5000);
		lv_tick_inc(5);
		//lv_event_send(current_position_text,LV_EVENT_VALUE_CHANGED,NULL);
		//lv_event_send(current_speed_text,LV_EVENT_VALUE_CHANGED,NULL);

	}

    return 0;
}



