#ifndef __SYS_INFO_H__
#define __SYS_INFO_H__


#include <stdint.h>
#include "main.h"

typedef struct
{
    int16_t     flag;
    char        file[FILE_MAX_BYTE];
}str_sysfile_ram;

extern str_sysfile_ram file_ram_bsif;
extern str_sysfile_ram file_ram_pmng;

extern char dev_name[16];
extern char sw_ver[26];
extern char hw_ver[20];


typedef struct
{
    char desc[16];
    char mac[18];
    char ip[16];
    char netmask[16];
    char gateway[16];
    char sw_ver[16];
    char hw_ver[16];
}str_basic_info;


typedef struct
{
    str_basic_info basic_info;
}str_sys_info;


extern uint8_t sys_info_config(uint8_t state);
extern int8_t binfo_file_update(void);
extern int8_t binfo_file_update_from_rambuf(char *buf,uint16_t len);
extern int8_t binfo_data_update(uint32_t size);
extern int8_t portmng_parameter_update(uint32_t size);
extern int8_t portmng_file_update(void);
extern int8_t portmng_file_update_from_rambuf(char *buf,uint16_t len);
extern int8_t strIP4_to_array(uint8_t* dec,const char* src);
extern int8_t checkIPformat(const char* src);
extern int8_t strMAC_to_array(uint8_t* dec,const char* src);
extern int8_t checkMACformat(const char* src);

#endif /*__SYS_INFO_H__*/


