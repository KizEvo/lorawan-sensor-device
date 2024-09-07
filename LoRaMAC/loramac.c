#include "loramac.h"
#include "aes.h"

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

static int32_t byte_array_xor(uint8_t *byte_a, uint8_t *byte_b, uint8_t *out, uint32_t size)
{
	for (int i = 0; i < size; i++) {
		out[i] = byte_a[i] ^ byte_b[i];
	}
	return 0;
}

// TODO: support frm_payload_size greater than 16 bytes
// TODO: support other MType other than Unconfirmed up/down
int32_t loramac_frm_payload_encryption(struct loramac_phys_payload *payload, uint8_t frm_payload_size, uint8_t *key)
{
	if (!payload) {
		return -1;
	}
	uint8_t *Ai_dev_addr;
	uint8_t *Ai_f_cnt;

	uint8_t Ai[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t S[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	Ai[0] = 0x01;
	Ai[5] = payload->m_hdr & 0x20 ? DOWNLINK : UPLINK;
	Ai_dev_addr = &Ai[6];
	Ai_dev_addr = (uint8_t *)&payload->mac_payload.f_hdr.dev_addr; // transform to little endian
	Ai_f_cnt = &Ai[10];
	Ai_f_cnt = (uint8_t *)&payload->mac_payload.f_hdr.f_cnt;

	aes_context ctx = {0};
	aes_set_key(key, 16, &ctx);

	for (uint8_t i = 1; i <= (uint8_t)(frm_payload_size / 16) + (frm_payload_size % 16 ? 1 : 0); i++) {
		Ai[15] = i;
		aes_encrypt(Ai, S, &ctx);
	}

	byte_array_xor(payload->mac_payload.frm_payload, S, payload->mac_payload.frm_payload, frm_payload_size);

	return 0;
}
