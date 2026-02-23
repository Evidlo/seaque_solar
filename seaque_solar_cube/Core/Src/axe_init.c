#include "axe_init.h"


volatile bool telemetry_en = false;


struct TNODE root_table[TABLE_SIZE];

void table_init()
{
	struct TNODE* node_ptr;


	node_ptr = add_node(root_table, TABLE_SIZE, &telemetry_en, 1, "telemetry_en");
	node_ptr->data_type = T_BOOLEAN;
	node_ptr->is_array = false;
	node_ptr->shape[0] = 1;
	node_ptr->shape[1] = 1;
	node_ptr->shape[2] = 1;
	node_ptr->readable = true;
	node_ptr->writeable = true;
	node_ptr->executable = false;
	node_ptr->read_cb = NULL;
	node_ptr->write_cb = NULL;
	node_ptr->exec_func = NULL;

}