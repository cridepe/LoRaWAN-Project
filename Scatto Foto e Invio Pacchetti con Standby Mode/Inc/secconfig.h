/*
 * Copyright (c) 2021 Hangmao Liu <liuhangmao@hotmail.com>
 *                    Mohammad Hamad <mohammad.hamad@tum.de>
 *
 * This library is free software; you can redistribute it and/or modify
 * it under the terms of the MIT license. See COPYING for details.
 *
 */

#ifndef _SECCONFIG_H
#define _SECCONFIG_H

/* DEBUG FLAGS  */
#define DBG_MSG  /* enable high-level debug message */
#define TIME_MSG /* enable debug messages about consumed time */
//#define DDBG /*more detailed debug messages that includes crypto-related operations*/

/* KeyStore */
#define KSN_NUM 3
#define MAX_SK_USAGE 50 // a session key can only be used in MAX_SK_USAGE times communication, then needs to be updated

/* Shamir Secret Sharing Params */
#define PRIME 257
#define SSS_T 2 // SSS threshold (<= KSN_NUM)
#define SSS_SIZE 39 // 6 + 2*BLOCK_SIZE + 1

/* Crypto Params */
#define BLOCK_SIZE 16 // topic key (128 bits)
#define C_SIZE 128 // rsa key size (1024 bits)
#define PK_LEN 273
#define HASH_LEN 32 // 256 bits
#define ELEMENT_LEN 128

#define TYPEA_PARAMS \
"type a\n" \
"q 87807107996633125224377819847540498158068831994142082" \
"1102865339926647563088022295707862517942266222142315585" \
"8769582317459277713367317481324925129998224791\n" \
"h 12016012264891146079388821366740534204802954401251311" \
"822919615131047207289359704531102844802183906537786776\n" \
"r 730750818665451621361119245571504901405976559617\n" \
"exp2 159\n" \
"exp1 107\n" \
"sign1 1\n" \
"sign0 1\n"

#define TOPIC_SIZE 50 // maximum size of a topic
#define NONCE_SIZE 16 // size of nonce

#endif /* End of _SECCONFIG_H */
