#include "app.h"
#include "stdbool.h"
#include "bg_types.h"
#include "native_gecko.h"
#include "log.h"
static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);

void AppHandler(void){

	INIT_LOG();

	while (1) {
		struct gecko_cmd_packet *evt = gecko_wait_event();
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	}
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
	switch (evt_id) {
	case gecko_evt_dfu_boot_id:
		//gecko_cmd_le_gap_set_advertising_timing(0, 1000*adv_interval_ms/625, 1000*adv_interval_ms/625, 0, 0);
		gecko_cmd_le_gap_set_mode(2, 2);
		break;
	case gecko_evt_system_boot_id:
		// Initialize Mesh stack in Node operation mode, wait for initialized event
		gecko_cmd_mesh_node_init();
		break;
	case gecko_evt_mesh_node_initialized_id:
		// The Node is now initialized, start unprovisioned Beaconing using PB-Adv Bearer
		gecko_cmd_mesh_node_start_unprov_beaconing(0x1);
		break;

	default:
		break;
	}
}
