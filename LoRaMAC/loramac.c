#include <stdlib.h>
#include <string.h>

#include "api.h"
#include "crypto_auth.h"

#include "loramac.h"
#include "aes.h"

struct loramac_phys_payload *loramac_init(void)
{
	static struct loramac_phys_payload payload = {0};
	payload.mac_payload.frm_payload = NULL;
	payload.mac_payload.f_hdr.f_opts = NULL;
	
	return &payload;
}

int32_t loramac_fill_fhdr(struct loramac_phys_payload *payload, uint32_t dev_addr, uint8_t f_ctrl, uint16_t f_cnt, uint8_t *f_opts)
{
	payload->mac_payload.f_hdr.dev_addr = dev_addr;
	payload->mac_payload.f_hdr.f_ctrl = f_ctrl;
	payload->mac_payload.f_hdr.f_cnt = f_cnt;
	payload->mac_payload.f_hdr.f_opts = f_opts;

	return 0;
}

int32_t loramac_fill_mac_payload(struct loramac_phys_payload *payload, uint8_t f_port, uint8_t *frm_payload)
{
	payload->mac_payload.f_port = f_port;
	payload->mac_payload.frm_payload = frm_payload;

	return 0;
}

int32_t loramac_fill_phys_payload(struct loramac_phys_payload *payload, uint8_t m_hdr, uint32_t mic)
{
	payload->m_hdr = m_hdr;
	payload->mic = mic;
	
	return 0;
}

static int32_t loramac_aes_byte_array_xor(uint8_t *byte_a, uint8_t *byte_b, uint8_t *out, uint32_t size)
{
	for (int i = 0, j = size - 1; i < size; i++, j--) {
		out[i] = byte_a[i] ^ byte_b[j];
	}
	return 0;
}

// TODO: support FOpts in calculation, currently it's skipped as FCTRL will always be 0x00
int32_t loramac_calculate_mic(struct loramac_phys_payload *payload, uint8_t frm_payload_size, uint8_t *key, uint8_t algo_option, uint32_t *mic)
{
	unsigned long long inlen = 9 + frm_payload_size + 16;
	uint8_t in[inlen]; // 9 bytes = MHDR + DEV_ADDR + FCTRL + FCNT + FPORT

	uint8_t out[16] = {0};
	uint8_t *B_dev_addr;
	uint8_t *B_f_cnt;

	memset(in, 0, inlen);

	uint8_t B[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	B[0] = 0x49;

	B_dev_addr = (uint8_t *)&payload->mac_payload.f_hdr.dev_addr;
	memcpy(&B[6], B_dev_addr, 4);
	B_f_cnt = (uint8_t *)&payload->mac_payload.f_hdr.f_cnt;
	memcpy(&B[10], B_f_cnt, 2);

	B[15] = frm_payload_size + 9; // FRM_PAYLOAD + MHDR + FHDR + FPORT
	
	memcpy(in, B, 16);
	in[16] = payload->m_hdr;
	memcpy(&in[17], (uint8_t *)&payload->mac_payload.f_hdr, 7);
	in[17 + 7] = payload->mac_payload.f_port;
	// Little endian payload
	for (uint8_t i = 17 + 7 + 1, j = frm_payload_size - 1; i < frm_payload_size + 17 + 7 + 1; i++, j--) {
		in[i] = payload->mac_payload.frm_payload[j]; // FRM_PAYLOAD starts from byte offset 9
	}
	// ASCON MAC
	if (algo_option) {
		int rc = crypto_auth(out, in, inlen, key);
		if (rc != 0) {
			return -1;
		}
	} else {
		return -2;
	}
	*mic = out[0];
	*mic |= out[1] << 8;
	*mic |= out[2] << (8 * 2);
	*mic |= out[3] << (8 * 3);
	return 0;
}

// TODO: support frm_payload_size greater than 16 bytes
// TODO: support other MType other than Unconfirmed up/down
int32_t loramac_frm_payload_encryption(struct loramac_phys_payload *payload, uint8_t frm_payload_size, uint8_t *key)
{
	uint8_t *Ai_dev_addr;
	uint8_t *Ai_f_cnt;

	uint8_t Ai[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t S[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	Ai[0] = 0x01;
	Ai[5] = payload->m_hdr & 0x20 ? DOWNLINK : UPLINK;

	Ai_dev_addr = (uint8_t *)&payload->mac_payload.f_hdr.dev_addr; // transform to little endian
	memcpy(&Ai[6], Ai_dev_addr, 4);
	Ai_f_cnt = (uint8_t *)&payload->mac_payload.f_hdr.f_cnt;
	memcpy(&Ai[10], Ai_f_cnt, 2);

	aes_context ctx = {0};
	aes_set_key(key, 16, &ctx);

	for (uint8_t i = 1; i <= (uint8_t)(frm_payload_size / 16) + (frm_payload_size % 16 ? 1 : 0); i++) {
		Ai[15] = i;
		aes_encrypt(Ai, S, &ctx);
	}

	loramac_aes_byte_array_xor(payload->mac_payload.frm_payload, S, payload->mac_payload.frm_payload, frm_payload_size);

	return 0;
}

// TODO: support FOpts unknown length, skip it for now
int32_t loramac_serialize_data(struct loramac_phys_payload *payload, uint8_t *out_data, uint8_t frm_payload_size)
{
	memcpy(out_data, (uint8_t *)payload, 8); // MHDR -> FCNT
	out_data[8] = payload->mac_payload.f_port; // FPORT
	// Little endian payload
	for (uint8_t i = 9, j = frm_payload_size - 1; i < frm_payload_size + 9; i++, j--) {
		out_data[i] = payload->mac_payload.frm_payload[j]; // FRM_PAYLOAD starts from byte offset 9
	}
	memcpy(&out_data[9 + frm_payload_size], (uint8_t *)&payload->mic, 4); // MIC
	
	return 0;
}
