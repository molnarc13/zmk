/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/types.h>
#include <zephyr/init.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/settings/settings.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/ble.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>
#include <zmk/split/bluetooth/uuid.h>
#include <zmk/split/bluetooth/service.h>
#include <zmk/split/bluetooth/central.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>
#include <zmk/pointing/input_split.h>

static int start_scanning(void);

#define POSITION_STATE_DATA_LEN 16

enum peripheral_slot_state {
    PERIPHERAL_SLOT_STATE_OPEN,
    PERIPHERAL_SLOT_STATE_CONNECTING,
    PERIPHERAL_SLOT_STATE_CONNECTED,
};

struct peripheral_slot {
    enum peripheral_slot_state state;
    struct bt_conn *conn;
    struct bt_gatt_discover_params discover_params;
    struct bt_gatt_subscribe_params subscribe_params;
    struct bt_gatt_subscribe_params sensor_subscribe_params;
    struct bt_gatt_discover_params sub_discover_params;
    uint16_t run_behavior_handle;
    uint8_t position_state[POSITION_STATE_DATA_LEN];
    uint8_t changed_positions[POSITION_STATE_DATA_LEN];
};

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static const struct bt_uuid *gatt_ccc_uuid = BT_UUID_GATT_CCC;
static const struct bt_uuid *gatt_cpf_uuid = BT_UUID_GATT_CPF;

struct peripheral_input_slot {
    struct bt_conn *conn;
    struct bt_gatt_subscribe_params sub;
    uint8_t reg;
};

#define COUNT_INPUT_SPLIT(n) +1

static struct peripheral_input_slot
    peripheral_input_slots[(0 DT_FOREACH_STATUS_OKAY(zmk_input_split, COUNT_INPUT_SPLIT))];

static bool input_slot_is_open(size_t i) {
    return i < ARRAY_SIZE(peripheral_input_slots) && peripheral_input_slots[i].conn == NULL;
}

static bool input_slot_is_pending(size_t i) {
    return i < ARRAY_SIZE(peripheral_input_slots) && peripheral_input_slots[i].conn != NULL &&
           (!peripheral_input_slots[i].sub.value_handle ||
            !peripheral_input_slots[i].sub.ccc_handle || !peripheral_input_slots[i].reg);
}

static int reserve_next_open_input_slot(struct peripheral_input_slot **slot, struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (input_slot_is_open(i)) {
            peripheral_input_slots[i].conn = conn;
            peripheral_input_slots[i].sub.value_handle = 0;
            peripheral_input_slots[i].sub.ccc_handle = 0;
            peripheral_input_slots[i].reg = 0;
            *slot = &peripheral_input_slots[i];
            return i;
        }
    }
    return -ENOMEM;
}

static int find_pending_input_slot(struct peripheral_input_slot **slot, struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (peripheral_input_slots[i].conn == conn && input_slot_is_pending(i)) {
            *slot = &peripheral_input_slots[i];
            return i;
        }
    }
    return -ENODEV;
}

void release_peripheral_input_subs(struct bt_conn *conn) {
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (peripheral_input_slots[i].conn == conn) {
            peripheral_input_slots[i].conn = NULL;
        }
    }
}

#endif // IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)

static struct peripheral_slot peripherals[ZMK_SPLIT_BLE_PERIPHERAL_COUNT];
static bool is_scanning = false;
static const struct bt_uuid_128 split_service_uuid = BT_UUID_INIT_128(ZMK_SPLIT_BT_SERVICE_UUID);

int peripheral_slot_index_for_conn(struct bt_conn *conn) {
    for (int i = 0; i < ZMK_SPLIT_BLE_PERIPHERAL_COUNT; i++) {
        if (peripherals[i].conn == conn) {
            return i;
        }
    }
    return -EINVAL;
}

struct peripheral_slot *peripheral_slot_for_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return NULL;
    }
    return &peripherals[idx];
}

int release_peripheral_slot(int index) {
    if (index < 0 || index >= ZMK_SPLIT_BLE_PERIPHERAL_COUNT) {
        return -EINVAL;
    }
    struct peripheral_slot *slot = &peripherals[index];
    if (slot->state == PERIPHERAL_SLOT_STATE_OPEN) {
        return -EINVAL;
    }
    LOG_DBG("Releasing peripheral slot at %d", index);
    if (slot->conn != NULL) {
        bt_conn_unref(slot->conn);
        slot->conn = NULL;
    }
    slot->state = PERIPHERAL_SLOT_STATE_OPEN;
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (slot->position_state[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                struct zmk_position_state_changed ev = {.position = position,
                                                        .state = false,
                                                        .timestamp = k_uptime_get()};
                ZMK_EVENT_RAISE_AT(ev, split_central);
            }
        }
    }
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        slot->position_state[i] = 0U;
        slot->changed_positions[i] = 0U;
    }
    slot->subscribe_params.value_handle = 0;
    slot->run_behavior_handle = 0;
    return 0;
}

int reserve_peripheral_slot(const bt_addr_le_t *addr) {
    int i = zmk_ble_put_peripheral_addr(addr);
    if (i >= 0) {
        if (peripherals[i].state == PERIPHERAL_SLOT_STATE_OPEN) {
            release_peripheral_slot(i);
            peripherals[i].state = PERIPHERAL_SLOT_STATE_CONNECTING;
            return i;
        }
    }
    return -ENOMEM;
}

int release_peripheral_slot_for_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return idx;
    }
    return release_peripheral_slot(idx);
}

int confirm_peripheral_slot_conn(struct bt_conn *conn) {
    int idx = peripheral_slot_index_for_conn(conn);
    if (idx < 0) {
        return idx;
    }
    peripherals[idx].state = PERIPHERAL_SLOT_STATE_CONNECTED;
    return 0;
}

#if ZMK_KEYMAP_HAS_SENSORS
static uint8_t split_central_sensor_notify_func(struct bt_conn *conn,
                                                struct bt_gatt_subscribe_params *params,
                                                const void *data, uint16_t length) {
    if (!data) {
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }
    if (length < offsetof(struct sensor_event, channel_data)) {
        return BT_GATT_ITER_STOP;
    }
    struct sensor_event sensor_event;
    memcpy(&sensor_event, data, MIN(length, sizeof(sensor_event)));
    if (sensor_event.channel_data_size != 1) {
        return BT_GATT_ITER_STOP;
    }
    struct zmk_sensor_event ev = {
        .sensor_number = sensor_event.sensor_index,
        .channel_data = sensor_event.channel_data[0],
        .timestamp = k_uptime_get()};
    ZMK_EVENT_RAISE_AT(ev, split_central);
    return BT_GATT_ITER_CONTINUE;
}
#endif /* ZMK_KEYMAP_HAS_SENSORS */

#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
static uint8_t peripheral_input_event_notify_cb(struct bt_conn *conn,
                                                struct bt_gatt_subscribe_params *params,
                                                const void *data, uint16_t length) {
    if (!data) {
        params->value_handle = 0U;
        return BT_GATT_ITER_STOP;
    }
    if (length != sizeof(struct zmk_split_input_event_payload)) {
        return BT_GATT_ITER_STOP;
    }
    struct zmk_split_input_event_payload payload;
    memcpy(&payload, data, MIN(length, sizeof(struct zmk_split_input_event_payload)));
    for (size_t i = 0; i < ARRAY_SIZE(peripheral_input_slots); i++) {
        if (&peripheral_input_slots[i].sub == params) {
            zmk_input_split_report_event(peripheral_input_slots[i].reg, payload.type,
                                         payload.code, payload.value, payload.sync);
            break;
        }
    }
    return BT_GATT_ITER_CONTINUE;
}
#endif

static uint8_t split_central_notify_func(struct bt_conn *conn,
                                         struct bt_gatt_subscribe_params *params, const void *data,
                                         uint16_t length) {
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL || !data) {
        if (params) {
            params->value_handle = 0U;
        }
        return BT_GATT_ITER_STOP;
    }
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        slot->changed_positions[i] = ((uint8_t *)data)[i] ^ slot->position_state[i];
        slot->position_state[i] = ((uint8_t *)data)[i];
    }
    for (int i = 0; i < POSITION_STATE_DATA_LEN; i++) {
        for (int j = 0; j < 8; j++) {
            if (slot->changed_positions[i] & BIT(j)) {
                uint32_t position = (i * 8) + j;
                bool pressed = slot->position_state[i] & BIT(j);
                struct zmk_position_state_changed ev = {.position = position,
                                                        .state = pressed,
                                                        .timestamp = k_uptime_get()};
                ZMK_EVENT_RAISE_AT(ev, split_central);
            }
        }
    }
    return BT_GATT_ITER_CONTINUE;
}

static int split_central_subscribe(struct bt_conn *conn, struct bt_gatt_subscribe_params *params) {
    atomic_set(params->flags, BT_GATT_SUBSCRIBE_FLAG_NO_RESUB);
    return bt_gatt_subscribe(conn, params);
}

static uint8_t split_central_chrc_discovery_func(struct bt_conn *conn,
                                                 const struct bt_gatt_attr *attr,
                                                 struct bt_gatt_discover_params *params) {
    if (!attr || !attr->user_data) {
        return BT_GATT_ITER_STOP;
    }
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        return BT_GATT_ITER_STOP;
    }
    switch (params->type) {
    case BT_GATT_DISCOVER_CHARACTERISTIC: {
        const struct bt_uuid *chrc_uuid = ((struct bt_gatt_chrc *)attr->user_data)->uuid;
        if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_POSITION_STATE_UUID)) == 0) {
            slot->subscribe_params.disc_params = &slot->sub_discover_params;
            slot->subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->subscribe_params.notify = split_central_notify_func;
            slot->subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->subscribe_params);
#if ZMK_KEYMAP_HAS_SENSORS
        } else if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_SENSOR_STATE_UUID)) == 0) {
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 2;
            slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
            slot->sensor_subscribe_params.disc_params = &slot->sub_discover_params;
            slot->sensor_subscribe_params.end_handle = slot->discover_params.end_handle;
            slot->sensor_subscribe_params.value_handle = bt_gatt_attr_value_handle(attr);
            slot->sensor_subscribe_params.notify = split_central_sensor_notify_func;
            slot->sensor_subscribe_params.value = BT_GATT_CCC_NOTIFY;
            split_central_subscribe(conn, &slot->sensor_subscribe_params);
#endif
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        } else if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_INPUT_EVENT_UUID)) == 0) {
            struct peripheral_input_slot *input_slot;
            int ret = reserve_next_open_input_slot(&input_slot, conn);
            if (ret >= 0) {
                input_slot->sub.value_handle = bt_gatt_attr_value_handle(attr);
                slot->discover_params.uuid = gatt_ccc_uuid;
                slot->discover_params.start_handle = attr->handle;
                slot->discover_params.type = BT_GATT_DISCOVER_STD_CHAR_DESC;
            }
#endif
        } else if (bt_uuid_cmp(chrc_uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_CHAR_RUN_BEHAVIOR_UUID)) == 0) {
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 2;
            slot->run_behavior_handle = bt_gatt_attr_value_handle(attr);
        }
        break;
    }
    case BT_GATT_DISCOVER_STD_CHAR_DESC:
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
        if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_GATT_CCC) == 0) {
            struct peripheral_input_slot *input_slot;
            int ret = find_pending_input_slot(&input_slot, conn);
            if (ret >= 0) {
                input_slot->sub.ccc_handle = attr->handle;
                slot->discover_params.uuid = gatt_cpf_uuid;
                slot->discover_params.start_handle = attr->handle + 1;
                slot->discover_params.type = BT_GATT_DISCOVER_STD_CHAR_DESC;
            }
        } else if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_GATT_CPF) == 0) {
            struct bt_gatt_cpf *cpf = attr->user_data;
            struct peripheral_input_slot *input_slot;
            int ret = find_pending_input_slot(&input_slot, conn);
            if (ret >= 0) {
                input_slot->reg = cpf->description;
                input_slot->sub.notify = peripheral_input_event_notify_cb;
                input_slot->sub.value = BT_GATT_CCC_NOTIFY;
                split_central_subscribe(conn, &input_slot->sub);
            }
            slot->discover_params.uuid = NULL;
            slot->discover_params.start_handle = attr->handle + 1;
            slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
        }
#endif
        break;
    }
    bool subscribed = slot->run_behavior_handle && slot->subscribe_params.value_handle;
#if ZMK_KEYMAP_HAS_SENSORS
    subscribed = subscribed && slot->sensor_subscribe_params.value_handle;
#endif
    return subscribed ? BT_GATT_ITER_STOP : BT_GATT_ITER_CONTINUE;
}

static uint8_t split_central_service_discovery_func(struct bt_conn *conn,
                                                    const struct bt_gatt_attr *attr,
                                                    struct bt_gatt_discover_params *params) {
    if (!attr) {
        return BT_GATT_ITER_STOP;
    }
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        return BT_GATT_ITER_STOP;
    }
    if (bt_uuid_cmp(slot->discover_params.uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)) != 0) {
        return BT_GATT_ITER_CONTINUE;
    }
    slot->discover_params.uuid = NULL;
    slot->discover_params.func = split_central_chrc_discovery_func;
    slot->discover_params.type = BT_GATT_DISCOVER_CHARACTERISTIC;
    bt_gatt_discover(conn, &slot->discover_params);
    return BT_GATT_ITER_STOP;
}

static void split_central_process_connection(struct bt_conn *conn) {
    struct peripheral_slot *slot = peripheral_slot_for_conn(conn);
    if (slot == NULL) {
        return;
    }
    if (!slot->subscribe_params.value_handle) {
        slot->discover_params.uuid = &split_service_uuid.uuid;
        slot->discover_params.func = split_central_service_discovery_func;
        slot->discover_params.start_handle = 0x0001;
        slot->discover_params.end_handle = 0xffff;
        slot->discover_params.type = BT_GATT_DISCOVER_PRIMARY;
        bt_gatt_discover(slot->conn, &slot->discover_params);
    }
    start_scanning();
}

static int stop_scanning(void) {
    is_scanning = false;
    return bt_le_scan_stop();
}

static bool split_central_eir_found(const bt_addr_le_t *addr) {
    int slot_idx = reserve_peripheral_slot(addr);
    if (slot_idx < 0) {
        return false;
    }
    struct peripheral_slot *slot = &peripherals[slot_idx];
    if (stop_scanning() < 0) {
        return false;
    }
    struct bt_le_conn_param *param =
        BT_LE_CONN_PARAM(6, 6, 0, 800);
    int err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &slot->conn);
    if (err < 0) {
        release_peripheral_slot(slot_idx);
        start_scanning();
    }
    return false;
}

static bool split_central_eir_parse(struct bt_data *data, void *user_data) {
    bt_addr_le_t *addr = user_data;
    if (data->type == BT_DATA_UUID128_SOME || data->type == BT_DATA_UUID128_ALL) {
        for (int i = 0; i < data->data_len; i += 16) {
            struct bt_uuid_128 uuid;
            if (!bt_uuid_create(&uuid.uuid, &data->data[i], 16)) {
                continue;
            }
            if (bt_uuid_cmp(&uuid.uuid, BT_UUID_DECLARE_128(ZMK_SPLIT_BT_SERVICE_UUID)) == 0) {
                return split_central_eir_found(addr);
            }
        }
    }
    return true;
}

static void split_central_device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
                                       struct net_buf_simple *ad) {
    if (type == BT_GAP_ADV_TYPE_ADV_IND) {
        bt_data_parse(ad, split_central_eir_parse, (void *)addr);
    } else if (type == BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
        split_central_eir_found(addr);
    }
}

static int start_scanning(void) {
    if (is_scanning) {
        return 0;
    }
    for (int i = 0; i < CONFIG_ZMK_SPLIT_BLE_CENTRAL_PERIPHERALS; i++) {
        if (peripherals[i].conn == NULL) {
            is_scanning = true;
            return bt_le_scan_start(BT_LE_SCAN_PASSIVE, split_central_device_found);
        }
    }
    return 0;
}

static void split_central_connected(struct bt_conn *conn, uint8_t conn_err) {
    struct bt_conn_info info;
    bt_conn_get_info(conn, &info);
    if (info.role != BT_CONN_ROLE_CENTRAL) {
        return;
    }
    if (conn_err) {
        release_peripheral_slot_for_conn(conn);
        start_scanning();
        return;
    }
    confirm_peripheral_slot_conn(conn);
    split_central_process_connection(conn);
}

static void split_central_disconnected(struct bt_conn *conn, uint8_t reason) {
#if IS_ENABLED(CONFIG_ZMK_INPUT_SPLIT)
    release_peripheral_input_subs(conn);
#endif
    release_peripheral_slot_for_conn(conn);
    start_scanning();
}

static struct bt_conn_cb conn_callbacks = {
    .connected = split_central_connected,
    .disconnected = split_central_disconnected,
};

K_THREAD_STACK_DEFINE(split_central_split_run_q_stack,
                      CONFIG_ZMK_SPLIT_BLE_CENTRAL_SPLIT_RUN_STACK_SIZE);
struct k_work_q split_central_split_run_q;

struct central_cmd_wrapper {
    uint8_t source;
    struct zmk_split_run_behavior_payload payload;
};

K_MSGQ_DEFINE(zmk_split_central_split_run_msgq, sizeof(struct central_cmd_wrapper),
              CONFIG_ZMK_SPLIT_BLE_CENTRAL_SPLIT_RUN_QUEUE_SIZE, 4);

void split_central_split_run_callback(struct k_work *work) {
    struct central_cmd_wrapper payload_wrapper;
    while (k_msgq_get(&zmk_split_central_split_run_msgq, &payload_wrapper, K_NO_WAIT) == 0) {
        if (peripherals[payload_wrapper.source].state != PERIPHERAL_SLOT_STATE_CONNECTED) {
            continue;
        }
        if (!peripherals[payload_wrapper.source].run_behavior_handle) {
            continue;
        }
        bt_gatt_write_without_response(
            peripherals[payload_wrapper.source].conn,
            peripherals[payload_wrapper.source].run_behavior_handle, &payload_wrapper.payload,
            sizeof(struct zmk_split_run_behavior_payload), true);
    }
}

K_WORK_DEFINE(split_central_split_run_work, split_central_split_run_callback);

static int zmk_split_bt_central_init(void) {
    k_work_queue_start(&split_central_split_run_q, split_central_split_run_q_stack,
                       K_THREAD_STACK_SIZEOF(split_central_split_run_q_stack),
                       CONFIG_ZMK_BLE_THREAD_PRIORITY, NULL);
    bt_conn_cb_register(&conn_callbacks);
    start_scanning();
    return 0;
}

SYS_INIT(zmk_split_bt_central_init, APPLICATION, CONFIG_ZMK_BLE_INIT_PRIORITY);

int zmk_split_bt_central_run_behavior(uint8_t source, struct zmk_split_run_behavior_payload payload) {
    if (source >= ARRAY_SIZE(peripherals)) {
        return -EINVAL;
    }
    struct central_cmd_wrapper wrapper = {.source = source, .payload = payload};
    int err = k_msgq_put(&zmk_split_central_split_run_msgq, &wrapper, K_MSEC(100));
    if (err) {
        return err;
    }
    k_work_submit_to_queue(&split_central_split_run_q, &split_central_split_run_work);
    return 0;
}