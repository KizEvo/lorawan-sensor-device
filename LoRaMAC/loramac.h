#ifndef LORAMAC_H
#define LORAMAC_H

#include <stdint.h>

#define LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_UP 0x40
#define LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_DOWN 0x60
#define LORAMAC_PHYS_PAYLOAD_JOIN_ACCEPT 0x20

enum loramac_data_dir {UPLINK, DOWNLINK};
enum loramac_byte_offset {LRMAC_BYTE_OFFSET_MHDR, LRMAC_BYTE_OFFSET_DEVADDR, LRMAC_BYTE_OFFSET_FCTRL = 5, LRMAC_BYTE_OFFSET_FCNT, LRMAC_BYTE_OFFSET_FPORT = 8, LRMAC_BYTE_OFFSET_FRMPAYLOAD};

struct loramac_f_hdr {
	uint32_t dev_addr;
	uint8_t f_ctrl;
	uint16_t f_cnt;
	uint8_t *f_opts;
} __attribute__ ((packed));

struct loramac_mac_payload {
	struct loramac_f_hdr f_hdr;
	uint8_t f_port;
	uint8_t *frm_payload;
} __attribute__ ((packed));

// The user should init this data structure properly in order to use the API
// e.g struct loramac_phys_payload payload = {0};
// or you could use the loramac_init(void)
struct loramac_phys_payload {
	uint8_t m_hdr;
	struct loramac_mac_payload mac_payload;
	uint32_t mic;
} __attribute__ ((packed));

struct loramac_phys_payload_join_request {
	uint8_t m_hdr;
	uint8_t app_eui[8];
	uint8_t dev_eui[8];
	uint8_t dev_nonce[2];
	uint8_t mic[4];
};

struct loramac_phys_payload_join_accept {
	uint8_t m_hdr;
	uint8_t app_nonce[3];
	uint8_t net_id[3];
	uint8_t dev_addr[4];
	uint8_t dl_settings;
	uint8_t rx_delay;
	// CFList unused. Device freq configuration
	uint8_t mic[4];
};

struct join_accept_xskey_input {
	uint8_t byte1;
	uint8_t app_nonce[3];
	uint8_t net_id[3];
	uint8_t dev_nonce[2];
	uint8_t pad[7];
};

struct loramac_phys_payload *loramac_init(void);

int32_t loramac_fill_fhdr(struct loramac_phys_payload *payload, uint32_t dev_addr, uint8_t f_ctrl, uint16_t f_cnt, uint8_t *f_opts);

// Expect the user to fill the FHDR with the loramac_fill_fhdr function
int32_t loramac_fill_mac_payload(struct loramac_phys_payload *payload, uint8_t f_port, uint8_t *frm_payload);

// Expect the user to fill the MACPayload with the loramac_fill_mac_payload function
int32_t loramac_fill_phys_payload(struct loramac_phys_payload *payload, uint8_t m_hdr, uint32_t mic);

// Calculate MIC
int32_t loramac_calculate_mic(struct loramac_phys_payload *payload, uint8_t frm_payload_size, uint8_t *key, uint8_t algo_option, uint32_t *mic);

// Before calling this, fill all struct loramac_phys_payload and other data structures
// Support <= 16 bytes FRM_PAYLOAD_SIZE
// After using this function, the frm_payload field is encrypted
int32_t loramac_frm_payload_encryption(struct loramac_phys_payload *payload, uint8_t frm_payload_size, uint8_t *key);

int32_t loramac_serialize_data(struct loramac_phys_payload *payload, uint8_t *out_data, uint8_t frm_payload_size);

// join_request_le_msg - little endian format
int32_t loramac_pack_join_request(struct loramac_phys_payload_join_request **jr_frame, uint8_t *app_eui, uint8_t *dev_eui, uint8_t *dev_nonce, uint8_t *appkey);

// join_request_le_msg - little endian format
// Should be used after loramac_pack_join_request
int32_t loramac_update_join_request_nonce(struct loramac_phys_payload_join_request *jr_frame, uint8_t *new_dev_nonce);

int32_t loramac_pack_join_accept(struct loramac_phys_payload_join_accept **ja_frame, uint8_t *app_nonce, uint8_t *net_id, uint8_t *dev_addr, uint8_t *dl_settings, uint8_t *rx_delay, uint8_t *appkey);

#endif /* LORAMAC_H */
