#ifndef __SPCP_H__
#define __SPCP_H__

#include "stdio.h"
#include "stdint.h"

#define CONFIG_SPCP_PARAMS_MAX          16
#define CONFIG_SPCP_PARAMS_NAME_MAX     16
#define CONFIG_SPCP_PARAMS_DESC_MAX     32
#define CONFIG_SPCP_BUFFER_SIZE         32

//#define CONFIG_SPCP_HEXDUMP
//#define CONFIG_SPCP_DEBUG

#define SPCP_CMD_ACK_FLAG               (1<<7)
#define SPCP_CMD_REQUEST                (0<<7)
#define SPCP_CMD_ACK                    (1<<7)

typedef enum {
    SPCP_ENUM_CMD_GET_COUNT =   0x10,
    SPCP_ENUM_CMD_GET_PARAM =   0x11,
    SPCP_ENUM_CMD_SET_PARAM =   0x12,
    SPCP_ENUM_CMD_AUTH      =   0x20,
    SPCP_ENUM_CMD_AUTH_PASS =   0x21,
    SPCP_ENUM_CMD_AUTH_FAIL =   0x2E,
    SPCP_ENUM_CMD_NO_AUTH   =   0x2F,
    SPCP_ENUM_CMD_ERROR     =   0xFF,
} spcp_enum_cmd;


typedef enum {
    SPCP_ENUM_TYPE_UINT8=0,
    SPCP_ENUM_TYPE_INTEGER,
    SPCP_ENUM_TYPE_FLOAT,
    SPCP_ENUM_TYPE_UINT8_ARRAY,
    SPCP_ENUM_TYPE_STR,
    SPCP_ENUM_TYPE_MAX
} spcp_enum_type;

typedef enum{
    SPCP_ENUM_ERR_SUCCESS = 0,
    SPCP_ENUM_ERR_ILLEGAL_CMD = -1,
    SPCP_ENUM_ERR_ILLEGAL_TYPE = -2,
    SPCP_ENUM_ERR_DATA_TO_LITTLE = -3,
    SPCP_ENUM_ERR_OUT_OF_RANGE = -4,
    SPCP_ENUM_ERR_ACK_CMD = -5,
    SPCP_ENUM_ERR_AUTH_FAILURE = -6,
    SPCP_ENUM_ERR_READ_ONLY = -7,
    SPCP_ENUM_ERR_NO_TRANSMIT_FUNC = -20,
    SPCP_ENUM_ERR_UNKNOW = -50,
} spcp_enum_err;

typedef enum{
    SPCP_ENUM_PRIVILEGE_RO = 0,
    SPCP_ENUM_PRIVILEGE_RW = 1,
} spcp_enum_privilege;

typedef struct{
    const char name[CONFIG_SPCP_PARAMS_DESC_MAX];
    const char desc[CONFIG_SPCP_PARAMS_DESC_MAX];
    spcp_enum_privilege priv;
    spcp_enum_type type;
    uint8_t size;
    void* data;
} spcp_param,*spcp_param_t;

typedef struct{
    int idx;
    char name[CONFIG_SPCP_PARAMS_NAME_MAX];
    char desc[CONFIG_SPCP_PARAMS_DESC_MAX];
    spcp_enum_type type;
    
    union{
        int i32;
        float f32;
        uint8_t u8;
        struct{
            uint8_t* raw;
            size_t size;
        }array;
    }data;
} spcp_param_info,*spcp_param_info_t;

typedef struct{
    char auth_key[16];
    uint8_t is_auth;
    int (*edit_cb)(spcp_param_t);
    int count;
    spcp_param_t params[CONFIG_SPCP_PARAMS_MAX];
} spcp_param_handle,*spcp_param_handle_t;

typedef struct{
    char auth_key[16];
    int (*spcpc_transmit_func)(uint8_t*,size_t,uint8_t*,size_t*);
} spcpc_param_handle,*spcpc_param_handle_t;

int spcp_init(spcp_param_handle_t* handle);
int spcp_set_key(spcp_param_handle_t handle,const char* key,size_t len);
int spcp_register(spcp_param_handle_t handle,spcp_param_t param);
int spcp_set_edit_cb(spcp_param_handle_t handle,int (*edit_cb)(spcp_param_t param));
int spcp_process(spcp_param_handle_t handle,uint8_t* in_buffer,size_t in_len,uint8_t* out_buffer,size_t* out_len);

int spcpc_init(spcpc_param_handle_t* handle);
int spcpc_set_transmit_func(spcpc_param_handle_t handle,int (*spcpc_transmit_func)(uint8_t*,size_t,uint8_t*,size_t*));
int spcpc_set_key(spcpc_param_handle_t handle,const char* key,size_t len);
int spcpc_auth(spcpc_param_handle_t handle);
int spcpc_get_count(spcpc_param_handle_t handle);
int spcpc_get_param(spcpc_param_handle_t handle,int idx,spcp_param_info_t* param);
int spcpc_set_param(spcpc_param_handle_t handle,spcp_param_info_t param);

int spcp_dump(spcp_param_handle_t handle);
void spcp_param_info_display(spcp_param_info_t param);
void spcp_hexdump(const char*tag,const void *pdata, int len);

#endif
