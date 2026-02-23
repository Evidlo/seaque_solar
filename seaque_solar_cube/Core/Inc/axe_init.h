#include <axe.h>

#include <stdbool.h>

#ifndef AXE_TABLE_GEN_H
#define AXE_TABLE_GEN_H
#define TABLE_SIZE 1


extern struct TNODE root_table[TABLE_SIZE];

extern volatile bool telemetry_en;


void table_init(void);

#endif