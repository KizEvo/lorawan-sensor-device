#ifndef LORAMAC_H
#define LORAMAC_H

#include <stdint.h>

#define LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_UP 0x40
#define LORAMAC_PHYS_PAYLOAD_MHDR_UNCONFIRM_DATA_DOWN 0x60

enum loramac_data_dir {UPLINK, DOWNLINK};

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

#endif /* LORAMAC_H */
