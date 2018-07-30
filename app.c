#include "app.h"
#include "stdbool.h"
#include "bg_types.h"
#include "native_gecko.h"
#include "log.h"
#include "gatt_db.h"
#include "em_gpio.h"
#include "em_rtcc.h"
#include "em_cmu.h"

#include "my_model_def.h"

extern uint8_t boot_to_dfu;
/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/
#define TIMER_ID_FACTORY_RESET  		77
#define EX_PB0_PRESS					((1) << 5)
#define EX_PB0_LONG_PRESS				((1) << 6)
#define EX_PB1_PRESS					((1) << 7)
#define EX_PB1_LONG_PRESS				((1) << 8)

#define RES_100_MILLI_TICKS				3277
#define RES_1_SEC_TICKS					(32768)
#define RES_10_SEC_TICKS				((32768)*(10))
#define RES_10_MIN_TICKS				((32768)*(60)*(10))

#define RES_100_MILLI				0
#define RES_1_SEC					((1) << 6)
#define RES_10_SEC					((2) << 6)
#define RES_10_MIN					((3) << 6)
#define RES_BIT_MASK				0xC0

uint32_t real_time_ticks = 0;
uint8_t update_interval = 0;
static unit_t unit = celsius;
uint8_t conn_handle = 0xFF;
static uint8_t pb0_state = 0;
static uint8_t pb1_state = 0;
static uint32_t pb1_counter = 0;
static uint32_t pb0_counter = 0;
// Max x is 63
#define SET_100_MILLI(x)					(uint8_t)(RES_100_MILLI | ((x) & (0x3F)))
#define SET_1_SEC(x)						(uint8_t)(RES_1_SEC | ((x) & (0x3F)))
#define SET_10_SEC(x)						(uint8_t)(RES_10_SEC | ((x) & (0x3F)))
#define SET_10_MIN(x)						(uint8_t)(RES_10_MIN | ((x) & (0x3F)))

static uint8_t index = 0;

uint8_t periods[] = {
		SET_100_MILLI(3),			// 300ms
		0,
		SET_100_MILLI(20),			// 2s
		0,
		SET_1_SEC(10),				// 10s
		0,
		SET_10_SEC(12),				// 2min
		0,
		SET_10_MIN(1),				// 10min
		0 };

my_model_t my_model = {
		.elem_index = PRIMARY_ELEMENT,
		.vendor_id = MY_VENDOR_ID,
		.model_id = MY_MODEL_CLIENT_ID,
		.publish = 1,
		.opcodes_len = NUMBER_OF_OPCODES,
		.opcodes_data[0] = temperature_get,
		.opcodes_data[1] = temperature_status,
		.opcodes_data[2] = unit_get,
		.opcodes_data[3] = unit_set,
		.opcodes_data[4] = unit_set_unack,
		.opcodes_data[5] = unit_status,
		.opcodes_data[6] = update_interval_get,
		.opcodes_data[7] = update_interval_set,
		.opcodes_data[8] = update_interval_set_unack,
		.opcodes_data[9] = update_interval_status };


static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt);
bool mesh_bgapi_listener(struct gecko_cmd_packet *evt);


void key_interrupt_handler(void) {
	uint32_t int_flag = GPIO_IntGet();
	GPIO_IntClear(int_flag);
	if (int_flag & (1 << BSP_BUTTON0_PIN)) {
		pb0_state = !GPIO_PinInGet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN);
		if (pb0_state == 1) {
			// press, start counting
			pb0_counter = RTCC_CounterGet();
		} else {
			pb0_counter = RTCC_CounterGet() - pb0_counter;
			if (pb0_counter > 32768) {
				gecko_external_signal(EX_PB0_LONG_PRESS);
			} else {
				gecko_external_signal(EX_PB0_PRESS);
			}
		}
	}
	if (int_flag & (1 << BSP_BUTTON1_PIN)) {
		pb1_state = !GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN);
		if (pb1_state == 1) {
			// press, start counting
			pb1_counter = RTCC_CounterGet();
		} else {
			pb1_counter = RTCC_CounterGet() - pb1_counter;
			if (pb1_counter > 32768) {
				gecko_external_signal(EX_PB1_LONG_PRESS);
			} else {
				gecko_external_signal(EX_PB1_PRESS);
			}
		}
	}
}

void GPIO_EVEN_IRQHandler(void) {
	key_interrupt_handler();
}
void GPIO_ODD_IRQHandler(void) {
	key_interrupt_handler();
}
void initiate_factory_reset(void) {

	/* if connection is open then close it before rebooting */
	if (conn_handle != 0xFF) {
		gecko_cmd_le_connection_close(conn_handle);
	}

	/* perform a factory reset by erasing PS storage. This removes all the keys and other settings
	 that have been configured for this node */
	gecko_cmd_flash_ps_erase_all();
	// reboot after a small delay
	gecko_cmd_hardware_set_soft_timer(2 * 32768, TIMER_ID_FACTORY_RESET, 1);
}

static void parse_period(uint8_t interval) {
	switch (interval & RES_BIT_MASK) {
	case RES_100_MILLI:
		real_time_ticks = RES_100_MILLI_TICKS * (interval & (~RES_BIT_MASK));
		break;
	case RES_1_SEC:
		real_time_ticks = RES_1_SEC_TICKS * (interval & (~RES_BIT_MASK));
		break;
	case RES_10_SEC:
		real_time_ticks = RES_10_SEC_TICKS * (interval & (~RES_BIT_MASK));
		break;
	case RES_10_MIN:
		real_time_ticks = RES_10_MIN_TICKS * (interval & (~RES_BIT_MASK));
		break;
	default:
		break;
	}
	if(real_time_ticks){
		LOGI("Update Period [hh:mm:ss:ms]= %02d:%02d:%02d:%04d\r\n", real_time_ticks / (32768 * 60 * 60), (real_time_ticks % (32768 * 60 * 60)) / (32768 * 60), (real_time_ticks % (32768 * 60)) / 32768, ((real_time_ticks % (32768)) / 3277) * 100);
	}else{
		LOGI("Periodical update off.\r\n");
	}

}


void AppHandler(void){
	INIT_LOG();
	GPIO_PinModeSet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, gpioModeInputPull, 1);
	GPIO_PinModeSet(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, gpioModeInputPull, 1);

	NVIC_EnableIRQ(GPIO_EVEN_IRQn);
	NVIC_EnableIRQ(GPIO_ODD_IRQn);

	GPIO_IntConfig(BSP_BUTTON0_PORT, BSP_BUTTON0_PIN, true, true, true);
	GPIO_IntConfig(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN, true, true, true);

	CMU_ClockEnable(cmuClock_RTCC, true);

	while (1) {
		struct gecko_cmd_packet *evt = gecko_wait_event();
		bool pass = mesh_bgapi_listener(evt);
		if (pass) {
			handle_gecko_event(BGLIB_MSG_ID(evt->header), evt);
		}
	}
}

static void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt) {
	switch (evt_id) {
	case gecko_evt_hardware_soft_timer_id:
		switch (evt->data.evt_hardware_soft_timer.handle) {
		case TIMER_ID_FACTORY_RESET:
			LOGW("Software reset.\r\n");
			gecko_cmd_system_reset(0);
			break;
		}
		break;

		case gecko_evt_le_connection_opened_id:
			LOGI("Connected.\r\n");
			conn_handle = evt->data.evt_le_connection_opened.connection;
			break;

		case gecko_evt_system_boot_id:
			if (GPIO_PinInGet(BSP_BUTTON1_PORT, BSP_BUTTON1_PIN) == 0) {
				LOGW("Factory Reset Soon.\r\n");
				initiate_factory_reset();
			} else {
				LOGI("Normal start.\r\n");
				// Initialize Mesh stack in Node operation mode, wait for initialized event
				gecko_cmd_mesh_node_init();
			}

			break;

		case gecko_evt_mesh_vendor_model_receive_id: {
			uint32_t temperature = 0;
			struct gecko_msg_mesh_vendor_model_receive_evt_t *recv_evt = (struct gecko_msg_mesh_vendor_model_receive_evt_t *) &evt->data;
			LOGD("Vendor model data received.\r\n\tElement index = %d\r\n\tVendor id = 0x%04X\r\n\tModel id = 0x%04X\r\n\tSource address = 0x%04X\r\n\tDestination address = 0x%04X\r\n\tDestination label UUID index = 0x%02X\r\n\tApp key index = 0x%04X\r\n\tNon-relayed = 0x%02X\r\n\tOpcode = 0x%02X\r\n\tFinal = 0x%04X\r\n\tPayload: ", recv_evt->elem_index, recv_evt->vendor_id, recv_evt->model_id, recv_evt->source_address, recv_evt->destination_address, recv_evt->va_index, recv_evt->appkey_index, recv_evt->nonrelayed, recv_evt->opcode, recv_evt->final);
			UINT8_ARRAY_DUMP(recv_evt->payload.data, recv_evt->payload.len);
			LOGN();

			switch (recv_evt->opcode) {
			case temperature_status:
				temperature = *(uint32_t *) recv_evt->payload.data;
				LOGI("Temperature = %d.%d %s\r\n", temperature / 1000, temperature % 1000, unit == celsius ? (char * )"Celsius" : (char * )"Fahrenheit");
				break;
			case unit_status:
				unit = (unit_t) recv_evt->payload.data[0];
				LOGI("Unit = %s\r\n", unit == celsius ? (char * )"Celsius" : (char * )"Fahrenheit");
				break;
			case update_interval_status:
				update_interval = recv_evt->payload.data[0];
				//					LOGW("Period received = %d\r\n", update_interval);
				parse_period(update_interval);
				break;
				// Client should not response to other opcodes
			default:
				break;
			}
		}
		break;
		case gecko_evt_system_external_signal_id: {
			uint8_t opcode = 0, length = 0, data = 0;
			struct gecko_msg_mesh_vendor_model_set_publication_rsp_t *set_pub_ret;
			struct gecko_msg_mesh_vendor_model_publish_rsp_t *pub_ret;
			if (evt->data.evt_system_external_signal.extsignals & EX_PB0_PRESS) {
				opcode = temperature_get;
				LOGD("PB0 Pressed.\r\n");
			}
			if (evt->data.evt_system_external_signal.extsignals & EX_PB0_LONG_PRESS) {
				opcode = update_interval_set_unack;
				length = 1;
				data = periods[index];
				//				LOGW("Period to set = %d\r\n", data);
				if (index == sizeof(periods) - 1) {
					index = 0;
				} else {
					index++;
				}
				LOGD("PB0 Long Pressed.\r\n");
			}
			if (evt->data.evt_system_external_signal.extsignals & EX_PB1_PRESS) {
				opcode = unit_get;
				LOGD("PB1 Pressed.\r\n");

			}
			if (evt->data.evt_system_external_signal.extsignals & EX_PB1_LONG_PRESS) {
				if (unit == celsius) {
					opcode = unit_set_unack;
					length = 1;
					data = fahrenheit;
				} else {
					opcode = unit_set;
					length = 1;
					data = celsius;
				}
				LOGD("PB1 Long Pressed.\r\n");
			}
			set_pub_ret = gecko_cmd_mesh_vendor_model_set_publication(my_model.elem_index, my_model.vendor_id, my_model.model_id, opcode, 1, length, &data);
			if (set_pub_ret->result) {
				LOGE("Set publication error = 0x%04X\r\n", set_pub_ret->result);
				ERROR_ADDRESSING();
			} else {
				LOGD("Set publication done. Publishing...\r\n");
				pub_ret = gecko_cmd_mesh_vendor_model_publish(my_model.elem_index, my_model.vendor_id, my_model.model_id);
				if (pub_ret->result) {
					LOGE("Publish error = 0x%04X\r\n", pub_ret->result);
					ERROR_ADDRESSING();
				} else {
					LOGD("Publish done.\r\n");
				}
			}
		}
		break;

		case gecko_evt_mesh_node_initialized_id: {
			struct gecko_msg_mesh_node_initialized_evt_t *node_init_ret = (struct gecko_msg_mesh_node_initialized_evt_t *) &evt->data;
			struct gecko_msg_mesh_vendor_model_init_rsp_t *vm_init_ret;
			vm_init_ret = gecko_cmd_mesh_vendor_model_init(my_model.elem_index, my_model.vendor_id, my_model.model_id, my_model.publish, my_model.opcodes_len, my_model.opcodes_data);
			if (vm_init_ret->result) {
				LOGE("Vendor model init error = 0x%04X\r\n", vm_init_ret->result);
				ERROR_ADDRESSING();
			} else {
				LOGI("Vendor model init done. --- ");
				LOGW("Client. \r\n");
			}

			if (node_init_ret->provisioned) {
				// provisioned already
				LOGI("Provisioned already.\r\n");
			} else {
				// The Node is now initialized, start unprovisioned Beaconing using PB-Adv Bearer
				gecko_cmd_mesh_node_start_unprov_beaconing(0x3);
				LOGI("Unprovisioned beaconing.\r\n");
			}
		}
		break;

		case gecko_evt_mesh_node_provisioned_id: {
			LOGI("Provisioning done.\r\n");
		}
		break;

		case gecko_evt_mesh_node_provisioning_failed_id:
			LOGW("Provisioning failed. Result = 0x%04x\r\n", evt->data.evt_mesh_node_provisioning_failed.result);
			break;

		case gecko_evt_mesh_node_provisioning_started_id:
			LOGI("Provisioning started.\r\n");
			break;

		case gecko_evt_mesh_node_key_added_id:
			LOGI("got new %s key with index %x\r\n", evt->data.evt_mesh_node_key_added.type == 0 ? "network" : "application", evt->data.evt_mesh_node_key_added.index);
			break;

		case gecko_evt_mesh_node_config_set_id: {
			LOGD("gecko_evt_mesh_node_config_set_id\r\n\t");
			struct gecko_msg_mesh_node_config_set_evt_t *set_evt = (struct gecko_msg_mesh_node_config_set_evt_t *) &evt->data;
			UINT8_ARRAY_DUMP(set_evt->value.data, set_evt->value.len);
		}
		break;
		case gecko_evt_mesh_node_model_config_changed_id:
			LOGI("model config changed\r\n");
			break;

		case gecko_evt_le_connection_closed_id:
			LOGI("Disconnected.\r\n");
			conn_handle = 0xFF;
			/* Check if need to boot to dfu mode */
			if (boot_to_dfu) {
				/* Enter to DFU OTA mode */
				gecko_cmd_system_reset(2);
			}
			break;

		case gecko_evt_gatt_server_user_write_request_id:
			if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
				/* Set flag to enter to OTA mode */
				boot_to_dfu = 1;
				/* Send response to Write Request */
				gecko_cmd_gatt_server_send_user_write_response(evt->data.evt_gatt_server_user_write_request.connection,
						gattdb_ota_control, bg_err_success);

				/* Close connection to enter to DFU OTA mode */
				gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
			}
			break;
		default:
			break;
	}
}
