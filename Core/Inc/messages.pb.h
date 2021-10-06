/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_MESSAGES_PB_H_INCLUDED
#define PB_MESSAGES_PB_H_INCLUDED
#include <pb.h>

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _SensorType { 
    SensorType_WATER = 0, 
    SensorType_ENERGY = 1 
} SensorType;

typedef enum _DataType { 
    DataType_ANY_DATA = 0, 
    DataType_POWER_ACTIVE = 1, 
    DataType_POWER_APPARENT = 2, 
    DataType_CURRENT = 3, 
    DataType_VOLTAGE = 4, 
    DataType_FLOW = 5 
} DataType;

/* Struct definitions */
/* *
 Mensagem com dado de medição de um sensor 

 Rota associada `sensor/{serial}/data/report`
 @param datetime {number} timestamp da hora da medição em formato UNIX
 @param data {number}  dado da medição */
typedef struct _DataReport { 
    uint64_t datetime; 
    double data; 
    DataType type; 
} DataReport;

/* *
 Mensagem para registrar um Hub

 @param secret {string} segredo para registrar um módulo
 @param mac_address {string} endereço MAC do Hub */
typedef struct _HubRegister { 
    char secret[128]; 
    char mac_address[128]; 
} HubRegister;

/* *
 Mensagem para registrar um módulo de Sensor

 Rota associada `hub/{serial}/sensor/register`
 @param type {SensorType} tipo do sensor
 @param number {number} número associado ao sensor
 @param serial {string} serial identificador */
typedef struct _SensorRegister { 
    SensorType type; 
    int32_t number; 
    char serial[128]; 
} SensorRegister;


/* Helper constants for enums */
#define _SensorType_MIN SensorType_WATER
#define _SensorType_MAX SensorType_ENERGY
#define _SensorType_ARRAYSIZE ((SensorType)(SensorType_ENERGY+1))

#define _DataType_MIN DataType_ANY_DATA
#define _DataType_MAX DataType_FLOW
#define _DataType_ARRAYSIZE ((DataType)(DataType_FLOW+1))


#ifdef __cplusplus
extern "C" {
#endif

/* Initializer values for message structs */
#define DataReport_init_default                  {0, 0, _DataType_MIN}
#define SensorRegister_init_default              {_SensorType_MIN, 0, ""}
#define HubRegister_init_default                 {"", ""}
#define DataReport_init_zero                     {0, 0, _DataType_MIN}
#define SensorRegister_init_zero                 {_SensorType_MIN, 0, ""}
#define HubRegister_init_zero                    {"", ""}

/* Field tags (for use in manual encoding/decoding) */
#define DataReport_datetime_tag                  1
#define DataReport_data_tag                      2
#define DataReport_type_tag                      3
#define HubRegister_secret_tag                   1
#define HubRegister_mac_address_tag              2
#define SensorRegister_type_tag                  1
#define SensorRegister_number_tag                2
#define SensorRegister_serial_tag                3

/* Struct field encoding specification for nanopb */
#define DataReport_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UINT64,   datetime,          1) \
X(a, STATIC,   SINGULAR, DOUBLE,   data,              2) \
X(a, STATIC,   SINGULAR, UENUM,    type,              3)
#define DataReport_CALLBACK NULL
#define DataReport_DEFAULT NULL

#define SensorRegister_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, UENUM,    type,              1) \
X(a, STATIC,   SINGULAR, INT32,    number,            2) \
X(a, STATIC,   SINGULAR, STRING,   serial,            3)
#define SensorRegister_CALLBACK NULL
#define SensorRegister_DEFAULT NULL

#define HubRegister_FIELDLIST(X, a) \
X(a, STATIC,   SINGULAR, STRING,   secret,            1) \
X(a, STATIC,   SINGULAR, STRING,   mac_address,       2)
#define HubRegister_CALLBACK NULL
#define HubRegister_DEFAULT NULL

extern const pb_msgdesc_t DataReport_msg;
extern const pb_msgdesc_t SensorRegister_msg;
extern const pb_msgdesc_t HubRegister_msg;

/* Defines for backwards compatibility with code written before nanopb-0.4.0 */
#define DataReport_fields &DataReport_msg
#define SensorRegister_fields &SensorRegister_msg
#define HubRegister_fields &HubRegister_msg

/* Maximum encoded size of messages (where known) */
#define DataReport_size                          22
#define HubRegister_size                         260
#define SensorRegister_size                      143

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
