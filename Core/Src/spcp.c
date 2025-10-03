#include "spcp.h"

#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "ctype.h"


int spcp_init(spcp_param_handle_t* handle){
    *handle = (spcp_param_handle_t)malloc(sizeof(spcp_param_handle));
    (*handle)->count = 0;
    (*handle)->auth_key[0] = 0;
    (*handle)->is_auth = 1;
    (*handle)->edit_cb = NULL;
    memset((*handle)->params,0,CONFIG_SPCP_PARAMS_MAX * sizeof(spcp_param_t ));
    return SPCP_ENUM_ERR_SUCCESS;
}

int spcp_set_key(spcp_param_handle_t handle,const char* key,size_t len){
    if(key == NULL){
			handle->is_auth = 1;
			handle->auth_key[0] = 0;
    }else{
			memset(handle->auth_key,0,16);
			memcpy(handle->auth_key,key,len>15?15:len);
			handle->is_auth = 0;
    }
    return SPCP_ENUM_ERR_SUCCESS;
}

int spcp_register(spcp_param_handle_t handle,spcp_param_t param){
    if(param->type >= SPCP_ENUM_TYPE_MAX) return -1;
    handle->params[handle->count] = param;
    handle->count++;
    return SPCP_ENUM_ERR_SUCCESS;
}

int spcp_set_edit_cb(spcp_param_handle_t handle,int (*edit_cb)(spcp_param_t param)){
    handle->edit_cb = edit_cb;
    return 0;
}

int spcp_process(spcp_param_handle_t handle,uint8_t* in_buffer,size_t in_len,uint8_t* out_buffer,size_t* out_len) {
    int in_buffer_offset = 0;
    int out_buffer_offset = 0;
    if(in_len == 0) return SPCP_ENUM_ERR_DATA_TO_LITTLE;
    uint8_t cmd = in_buffer[in_buffer_offset++];
    if(cmd & SPCP_CMD_ACK_FLAG) return SPCP_ENUM_ERR_ILLEGAL_CMD;
    switch(cmd){
        case SPCP_ENUM_CMD_GET_COUNT:
        {
            if(!handle->is_auth){
                out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_NO_AUTH | SPCP_CMD_ACK;
                break;
            }
            out_buffer[out_buffer_offset++] = cmd | SPCP_CMD_ACK;
            out_buffer[out_buffer_offset++] = 0xFF&(handle->count>>24);
            out_buffer[out_buffer_offset++] = 0xFF&(handle->count>>16);
            out_buffer[out_buffer_offset++] = 0xFF&(handle->count>>8);
            out_buffer[out_buffer_offset++] = 0xFF&(handle->count>>0);
        }
        break;
        case SPCP_ENUM_CMD_GET_PARAM:
        {
            if(!handle->is_auth){
                out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_NO_AUTH | SPCP_CMD_ACK;
                break;
            }
            out_buffer[out_buffer_offset++] = cmd | SPCP_CMD_ACK;
            if(in_len < 2) return SPCP_ENUM_ERR_DATA_TO_LITTLE;
            uint8_t idx = in_buffer[in_buffer_offset++];
            if(idx >= handle->count) return SPCP_ENUM_ERR_OUT_OF_RANGE;
            spcp_param_t param = handle->params[idx];
            out_buffer[out_buffer_offset++] = idx;
            // name
            out_buffer[out_buffer_offset++] = 0xFF&strlen(param->name);
            memcpy(&out_buffer[out_buffer_offset],param->name,strlen(param->name));
            out_buffer_offset += strlen(param->name);

            // desc
            out_buffer[out_buffer_offset++] = 0xFF&strlen(param->desc);
            memcpy(&out_buffer[out_buffer_offset],param->desc,strlen(param->desc));
            out_buffer_offset += strlen(param->desc);

            // privilege
            out_buffer[out_buffer_offset++] = param->priv;

            // type
            out_buffer[out_buffer_offset++] = param->type;

            //data
            switch(param->type){
                case SPCP_ENUM_TYPE_UINT8:
                {
                    out_buffer[out_buffer_offset++] = *((uint8_t*)param->data);
                }
                break;
                case SPCP_ENUM_TYPE_INTEGER:
                {
                    out_buffer[out_buffer_offset+0] = *((int*)param->data) >> 24;
                    out_buffer[out_buffer_offset+1] = *((int*)param->data) >> 16;
                    out_buffer[out_buffer_offset+2] = *((int*)param->data) >> 8;
                    out_buffer[out_buffer_offset+3] = *((int*)param->data) >> 0;
                    out_buffer_offset += sizeof(int);
                }
                break;
                case SPCP_ENUM_TYPE_FLOAT:
                {
                    memcpy(&out_buffer[out_buffer_offset],param->data,sizeof(float));
                    out_buffer_offset += sizeof(float);
                }
                break;
                case SPCP_ENUM_TYPE_UINT8_ARRAY:
                case SPCP_ENUM_TYPE_STR:
                {
                    out_buffer[out_buffer_offset++] = param->size;
                    memset(&out_buffer[out_buffer_offset],0,param->size);
                    memcpy(&out_buffer[out_buffer_offset],param->data,param->size);
                    out_buffer_offset += param->size;
                }
                break;
                case SPCP_ENUM_TYPE_MAX:
                default:
                break;
            }
        }
        break;
        case SPCP_ENUM_CMD_SET_PARAM:
        {
            if(!handle->is_auth){
                out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_NO_AUTH | SPCP_CMD_ACK;
                break;
            }
            out_buffer[out_buffer_offset++] = cmd | SPCP_CMD_ACK;
            if(in_len < 2) return SPCP_ENUM_ERR_DATA_TO_LITTLE;
            uint8_t idx = in_buffer[in_buffer_offset++];
            if(idx >= handle->count) return SPCP_ENUM_ERR_OUT_OF_RANGE;
            spcp_param_t param = handle->params[idx];
            out_buffer[out_buffer_offset++] = idx;

            if(param->priv == SPCP_ENUM_PRIVILEGE_RO){
                return SPCP_ENUM_ERR_READ_ONLY;
            }
            switch(param->type){
                case SPCP_ENUM_TYPE_UINT8:
                {
                    memcpy(param->data,&in_buffer[in_buffer_offset],1);
                }
                break;
                case SPCP_ENUM_TYPE_INTEGER:
                {
                    *((int*)param->data) = in_buffer[in_buffer_offset] << 24 | in_buffer[in_buffer_offset+1] << 16 | in_buffer[in_buffer_offset+2] << 8 | in_buffer[in_buffer_offset+3] << 0;
                    // memcpy(param->data,&in_buffer[in_buffer_offset],4);
                }
                break;
                case SPCP_ENUM_TYPE_FLOAT:
                {
                    memcpy(param->data,&in_buffer[in_buffer_offset],4);
                }
                break;
                case SPCP_ENUM_TYPE_UINT8_ARRAY:
                case SPCP_ENUM_TYPE_STR:
                {
                    memset(param->data,0,param->size);
                    memcpy(param->data,&in_buffer[in_buffer_offset],param->size);
                }
                break;
                case SPCP_ENUM_TYPE_MAX:
                default:
                return SPCP_ENUM_ERR_ILLEGAL_TYPE;
            }
            if(handle->edit_cb != NULL ){
                handle->edit_cb(param);
            }
        }
        break;
        case SPCP_ENUM_CMD_AUTH:
        {
            if(handle->auth_key == NULL){
                out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_AUTH_PASS | SPCP_CMD_ACK;
                break;
            }
            if(in_len != 2 + strlen(handle->auth_key)){
                out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_AUTH_FAIL | SPCP_CMD_ACK;
								for(int i=0;i<strlen(handle->auth_key);i++){
									 out_buffer[out_buffer_offset++] = handle->auth_key[i];
								}
                break;
            }
            uint8_t auth_key_len = in_buffer[in_buffer_offset++];

            if(auth_key_len != strlen(handle->auth_key)){
                out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_AUTH_FAIL | SPCP_CMD_ACK;
								for(int i=0;i<auth_key_len;i++){
									 out_buffer[out_buffer_offset++] = in_buffer[in_buffer_offset+i];
								}
                break;
            }

            if(memcmp(&in_buffer[in_buffer_offset],handle->auth_key,auth_key_len)){
                out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_AUTH_FAIL | SPCP_CMD_ACK;
								for(int i=0;i<auth_key_len;i++){
									 out_buffer[out_buffer_offset++] = in_buffer[in_buffer_offset+i];
								}
                break;
            }

            out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_AUTH_PASS | SPCP_CMD_ACK;
            handle->is_auth = 1;
        }
        break;
        default:
            out_buffer[out_buffer_offset++] = SPCP_ENUM_CMD_ERROR;
            *out_len = out_buffer_offset;
            return SPCP_ENUM_ERR_ILLEGAL_CMD;
    }
    *out_len = out_buffer_offset;
    return SPCP_ENUM_ERR_SUCCESS;
}

int spcpc_init(spcpc_param_handle_t* handle){
    *handle = (spcpc_param_handle_t)malloc(sizeof(spcpc_param_handle));
    (*handle)->auth_key[0] = 0;
    (*handle)->spcpc_transmit_func = NULL;
    return SPCP_ENUM_ERR_SUCCESS;
}

int spcpc_set_transmit_func(spcpc_param_handle_t handle,int (*func)(uint8_t*,size_t,uint8_t*,size_t*)){
    handle->spcpc_transmit_func = func;
    return SPCP_ENUM_ERR_SUCCESS;
}

int spcpc_set_key(spcpc_param_handle_t handle,const char* key,size_t len){
    if(key == NULL){
			handle->auth_key[0] = 0;
    }else{
			memset(handle->auth_key,0,16);
			memcpy(handle->auth_key,key,len>15?15:len);
    }
    return SPCP_ENUM_ERR_SUCCESS;
}

int spcpc_auth(spcpc_param_handle_t handle){
    int ret = SPCP_ENUM_ERR_SUCCESS;
    uint8_t request_buffer[CONFIG_SPCP_BUFFER_SIZE] = {0};
    uint8_t response_buffer[CONFIG_SPCP_BUFFER_SIZE] = {0};
    size_t request_len = 0,response_len = CONFIG_SPCP_BUFFER_SIZE;
    if(handle->spcpc_transmit_func == NULL) {
        return SPCP_ENUM_ERR_NO_TRANSMIT_FUNC;
    }

    request_buffer[request_len++] = SPCP_ENUM_CMD_AUTH;
    request_buffer[request_len++] = handle->auth_key != NULL?strlen(handle->auth_key):0;
    if(handle->auth_key != NULL){
        memcpy(request_buffer+request_len,handle->auth_key,strlen(handle->auth_key));
        request_len += strlen(handle->auth_key);
    }
    spcp_hexdump("spcpc_auth-req",request_buffer,request_len);
    ret = handle->spcpc_transmit_func(request_buffer,request_len,response_buffer,&response_len);
    if(ret != SPCP_ENUM_ERR_SUCCESS){
        return ret;
    }
    spcp_hexdump("spcpc_auth-resp",response_buffer,response_len);
    return (response_buffer[0] == (SPCP_ENUM_CMD_AUTH_PASS | SPCP_CMD_ACK ))?SPCP_ENUM_ERR_SUCCESS:SPCP_ENUM_ERR_AUTH_FAILURE;
}

int spcpc_get_count(spcpc_param_handle_t handle){
    int ret = SPCP_ENUM_ERR_SUCCESS;
    uint8_t request_buffer[CONFIG_SPCP_BUFFER_SIZE] = {0};
    uint8_t response_buffer[CONFIG_SPCP_BUFFER_SIZE] = {0};
    size_t request_len = 0,response_len = CONFIG_SPCP_BUFFER_SIZE;
    if(handle->spcpc_transmit_func == NULL) {
        return SPCP_ENUM_ERR_NO_TRANSMIT_FUNC;
    }

    request_buffer[request_len++] = SPCP_ENUM_CMD_GET_COUNT;
    spcp_hexdump("spcpc_get_count-req",request_buffer,request_len);
    ret = handle->spcpc_transmit_func(request_buffer,request_len,response_buffer,&response_len);
    if(ret != SPCP_ENUM_ERR_SUCCESS){
        return ret;
    }
    spcp_hexdump("spcpc_get_count-resp",response_buffer,response_len);
    if(response_buffer[0] != (SPCP_ENUM_CMD_GET_COUNT | SPCP_CMD_ACK )){
        return SPCP_ENUM_ERR_UNKNOW;
    }
    ret = response_buffer[1] << 24 | response_buffer[2] << 16 | response_buffer[3] << 8 | response_buffer[4] << 0;
    return ret;
}

int spcpc_get_param(spcpc_param_handle_t handle,int idx,spcp_param_info_t* param){
    int ret = SPCP_ENUM_ERR_SUCCESS;
    uint8_t request_buffer[CONFIG_SPCP_BUFFER_SIZE] = {0};
    uint8_t response_buffer[CONFIG_SPCP_BUFFER_SIZE] = {0};
    size_t request_len = 0,response_len = CONFIG_SPCP_BUFFER_SIZE,response_offset = 0;
    if(handle->spcpc_transmit_func == NULL) {
        return SPCP_ENUM_ERR_NO_TRANSMIT_FUNC;
    }

    request_buffer[request_len++] = SPCP_ENUM_CMD_GET_PARAM;
    request_buffer[request_len++] = idx&0xFF;
    spcp_hexdump("spcpc_get_param-req",request_buffer,request_len);
    ret = handle->spcpc_transmit_func(request_buffer,request_len,response_buffer,&response_len);
    if(ret != SPCP_ENUM_ERR_SUCCESS){
        return ret;
    }
    spcp_hexdump("spcpc_get_param-resp",response_buffer,response_len);
    if(response_buffer[response_offset++] != (SPCP_ENUM_CMD_GET_PARAM | SPCP_CMD_ACK )){
        return SPCP_ENUM_ERR_UNKNOW;
    }
    *param = (spcp_param_info_t)malloc(sizeof(spcp_param_info));
    memset(*param,0,sizeof(spcp_param_info));

    (*param)->idx = response_buffer[response_offset++];

    int name_len = response_buffer[response_offset++];
    memset((*param)->name,0,CONFIG_SPCP_PARAMS_NAME_MAX);
    memcpy((*param)->name,response_buffer+response_offset,name_len);
    response_offset += name_len;

    int desc_len = response_buffer[response_offset++];
    memset((*param)->desc,0,CONFIG_SPCP_PARAMS_DESC_MAX);
    memcpy((*param)->desc,response_buffer+response_offset,desc_len);
    response_offset += desc_len;


    (*param)->type = response_buffer[response_offset++];

    switch((*param)->type){
        case SPCP_ENUM_TYPE_INTEGER:
            memcpy(&((*param)->data.i32),response_buffer+response_offset,sizeof(int));
            response_offset += sizeof(int);
            break;
        case SPCP_ENUM_TYPE_FLOAT:
            memcpy(&((*param)->data.f32),response_buffer+response_offset,sizeof(float));
            response_offset += sizeof(float);
            break;
        case SPCP_ENUM_TYPE_UINT8:
            memcpy(&((*param)->data.u8),response_buffer+response_offset,sizeof(uint8_t));
            response_offset += sizeof(uint8_t);
            break;
        case SPCP_ENUM_TYPE_UINT8_ARRAY:
            {
                int raw_size = response_buffer[response_offset++];
                uint8_t* raw_addr = (uint8_t*)malloc(sizeof(uint8_t)*(raw_size + 1));
                memset(raw_addr,0,raw_size + 1);
                memcpy(raw_addr,response_buffer+response_offset,raw_size);
                (*param)->data.array.raw = raw_addr;
                (*param)->data.array.size = raw_size;
                response_offset += raw_size;
            }
            break;
        case SPCP_ENUM_TYPE_MAX:
        default:
            break;
    }

    return SPCP_ENUM_ERR_SUCCESS;
}

int spcpc_set_param(spcpc_param_handle_t handle,spcp_param_info_t param){
    int ret = SPCP_ENUM_ERR_SUCCESS;
    uint8_t request_buffer[CONFIG_SPCP_BUFFER_SIZE]= {0};
    uint8_t response_buffer[CONFIG_SPCP_BUFFER_SIZE]= {0};
    size_t request_len = 0,response_len = CONFIG_SPCP_BUFFER_SIZE,response_offset = 0;
    if(handle->spcpc_transmit_func == NULL) {
        return SPCP_ENUM_ERR_NO_TRANSMIT_FUNC;
    }

    request_buffer[request_len++] = SPCP_ENUM_CMD_SET_PARAM;
    request_buffer[request_len++] = param->idx;

    switch(param->type){
        case SPCP_ENUM_TYPE_INTEGER:
            // memcpy(request_buffer+request_len,&param->data.i32,sizeof(int));
            *(request_buffer+request_len+0) = (param->data.i32 >> 24)&0xFF;
            *(request_buffer+request_len+1) = (param->data.i32 >> 16)&0xFF;
            *(request_buffer+request_len+2) = (param->data.i32 >> 8)&0xFF;
            *(request_buffer+request_len+3) = (param->data.i32 >> 0)&0xFF;
            request_len += sizeof(int);
            break;
        case SPCP_ENUM_TYPE_FLOAT:
            memcpy(request_buffer+request_len,&param->data.f32,sizeof(float));
            request_len += sizeof(float);
            break;
        case SPCP_ENUM_TYPE_UINT8:
            memcpy(request_buffer+request_len,&param->data.u8,sizeof(uint8_t));
            request_len += sizeof(uint8_t);
            break;
        case SPCP_ENUM_TYPE_UINT8_ARRAY:
            {
                memcpy(request_buffer+request_len,param->data.array.raw,param->data.array.size);
                request_len += param->data.array.size;
            }
            break;
        case SPCP_ENUM_TYPE_MAX:
        default:
            break;
    }

    spcp_hexdump("spcpc_set_param-req",request_buffer,request_len);
    ret = handle->spcpc_transmit_func(request_buffer,request_len,response_buffer,&response_len);
    if(ret != SPCP_ENUM_ERR_SUCCESS){
        return ret;
    }
    spcp_hexdump("spcpc_set_param-resp",response_buffer,response_len);
    if(response_buffer[response_offset++] != (SPCP_ENUM_CMD_SET_PARAM | SPCP_CMD_ACK )){
        return SPCP_ENUM_ERR_UNKNOW;
    }

    return SPCP_ENUM_ERR_SUCCESS;
}

int spcp_dump(spcp_param_handle_t handle){
#ifdef CONFIG_SPCP_DEBUG
    for(int i=0;i<handle->count;i++){
        spcp_param_t param = handle->params[i];
        printf("index: %d\n",i);
        printf("name: %s\n",param->name);
        printf("desc: %s\n",param->desc);
        printf("type: %d\n",param->type);
        printf("data: %p\n",param->data);
    }
#endif
    return SPCP_ENUM_ERR_SUCCESS;
}

void spcp_param_info_display(spcp_param_info_t param){
    printf("------------------------------------------\n");
    printf("idx: %d\n",param->idx);
    printf("name: %s\n",param->name);
    printf("desc: %s\n",param->desc);
    switch(param->type){
        case SPCP_ENUM_TYPE_INTEGER:
            printf("type: SPCP_ENUM_TYPE_INTEGER\n");
            printf("data: %d\n",param->data.i32);
            break;
        case SPCP_ENUM_TYPE_FLOAT:
            printf("type: SPCP_ENUM_TYPE_FLOAT\n");
            printf("data: %f\n",param->data.f32);
            break;
        case SPCP_ENUM_TYPE_UINT8:
            printf("type: SPCP_ENUM_TYPE_UINT8\n");
            printf("data: %02x\n",param->data.u8);
            break;
        case SPCP_ENUM_TYPE_UINT8_ARRAY:
            printf("type: SPCP_ENUM_TYPE_UINT8_ARRAY\n");
            printf("size: %zu\n",param->data.array.size);
            printf("data: ");
            for(int i=0;i<param->data.array.size;i++){
                printf("%02X ",param->data.array.raw[i]);
            }
            printf("\n");
            break;
        case SPCP_ENUM_TYPE_MAX:
        default:
            break;
    }
}


void spcp_hexdump(const char*tag,const void *pdata, int len) {
#ifdef CONFIG_SPCP_HEXDUMP
    int i, j, k, l;
    const char *data = (const char*)pdata;
    char buf[256], str[64], t[] = "0123456789ABCDEF";
    for (i = j = k = 0; i < len; i++) {
        if (0 == i % 16) 
            j += sprintf(buf + j, "[ %s ] %04xh: ",tag, i); 
        buf[j++] = t[0x0f & (data[i] >> 4)];
        buf[j++] = t[0x0f & data[i]];
        buf[j++] = ' ';
        str[k++] = isprint(data[i]) ? data[i] : '.';
        if (0 == (i + 1) % 16) {
            str[k] = 0;
            j += sprintf(buf + j, "| %s\n", str);
            printf("%s", buf);
            j = k = buf[0] = str[0] = 0;
        }
    }
    str[k] = 0;
    if (k) {
        for (l = 0; l < 3 * (16 - k); l++)
            buf[j++] = ' ';
        j += sprintf(buf + j, "| %s\n", str);
    }   
    if (buf[0]) printf("%s", buf);
#endif
}

