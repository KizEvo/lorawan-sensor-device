#include <stdlib.h>
#include <string.h>

#include "api.h"
#include "crypto_auth.h"
#include "crypto_aead.h"

#include "loramac.h"

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

int32_t loramac_fill_phys_payload(struct loramac_phys_payload *payload, uint8_t m_hdr, uint8_t *mic)
{
	payload->m_hdr = m_hdr;
	if (mic){
		memcpy(payload->mic, mic, 16);
	}
	return 0;
}

int32_t loramac_enc_aead(struct loramac_phys_payload *payload, uint8_t *out, uint8_t *out_size, uint8_t frm_payload_size, uint8_t *key)
{
	uint8_t cipher[frm_payload_size + 16]; // ciphertext + 16 tags
	uint8_t plaintext[frm_payload_size];

	uint64_t cipher_length_out = 0;
	uint8_t nonce[16] = {0};
	uint8_t ad[9] = {0}; // MHDR 1 + DevAddr 4 + FCtrl 1 + FCnt 2 + FPort 1
	uint8_t i;

	for (i = 0; i < frm_payload_size; i++){
		plaintext[i] = payload->mac_payload.frm_payload[frm_payload_size - 1 - i];
	}
	ad[0] = payload->m_hdr;

	ad[1] = payload->mac_payload.f_hdr.dev_addr & 0xFF;
	ad[2] = (payload->mac_payload.f_hdr.dev_addr >> 8) & 0xFF;
	ad[3] = (payload->mac_payload.f_hdr.dev_addr >> 16) & 0xFF;
	ad[4] = (payload->mac_payload.f_hdr.dev_addr >> 24) & 0xFF;
	ad[5] = payload->mac_payload.f_hdr.f_ctrl;

	ad[6] = payload->mac_payload.f_hdr.f_cnt & 0xFF;
	ad[7] = payload->mac_payload.f_hdr.f_cnt >> 8;
	ad[8] = payload->mac_payload.f_port;

	memcpy(nonce, ad, 5);
	nonce[5] = ad[6];
	nonce[6] = ad[7];

	int32_t ret = crypto_aead_encrypt(cipher, &cipher_length_out, plaintext, frm_payload_size, ad, 9, 0, nonce, key);

	// Sanity check
	if (((uint8_t)cipher_length_out) != (frm_payload_size + 16))
	{
		return -1;
	}

	memcpy(out, ad, 9);
	memcpy(&out[9], cipher, (uint8_t)cipher_length_out);

	*out_size = (uint8_t)cipher_length_out + 9;

	return ret;
}

int32_t loramac_dec_aead(struct loramac_phys_payload *out, uint8_t *out_frm_payload, uint8_t *out_frm_payload_size, uint8_t *loramac_in, uint8_t loramac_in_size, uint8_t *key)
{
	uint8_t cipher[loramac_in_size - 9]; // Ciphertext + tag = in - (MHDR 1 + DevAddr 4 + FCtrl 1 + FCnt 2 + FPort 1)
	uint8_t plaintext[loramac_in_size - 9 - 16]; // plaintext
	memset(cipher, 0, loramac_in_size - 9);
	memset(plaintext, 0, loramac_in_size - 9 - 16);

	uint8_t nonce[16] = {0};
	uint8_t ad[9] = {0}; // MHDR 1 + DevAddr 4 + FCtrl 1 + FCnt 2 + FPort 1

	uint64_t plaintext_length_out = 0;

	memcpy(ad, loramac_in, 9);
	memcpy(cipher, &loramac_in[9], loramac_in_size - 9);
	memcpy(nonce, ad, 5);
	nonce[5] = ad[6];
	nonce[6] = ad[7];
	
	int32_t ret = crypto_aead_decrypt(plaintext, &plaintext_length_out, NULL, cipher, loramac_in_size - 9, ad, sizeof(ad), nonce, key);
	
	// Sanity check
	if (((uint8_t)plaintext_length_out) != (loramac_in_size - 9 - 16))
	{
		return -1;
	}

	if (ret) {
		return ret;
	}

	out->m_hdr = ad[0];
	out->mac_payload.f_hdr.dev_addr = (ad[4] << 24) | (ad[3] << 16) | (ad[2] << 8) | ad[1];
	out->mac_payload.f_hdr.f_ctrl = ad[5];
	out->mac_payload.f_hdr.f_cnt = (ad[7] << 8) | ad[6];
	out->mac_payload.f_port = ad[8];

	for(uint8_t i = 0; i < (uint8_t)plaintext_length_out; i++){
		out_frm_payload[i] = plaintext[(uint8_t)plaintext_length_out - 1 - i];
	}
	
	*out_frm_payload_size = (uint8_t)plaintext_length_out;

	return 0;
}

int32_t loramac_pack_join_request(struct loramac_phys_payload_join_request **jr_frame, uint8_t *app_eui, uint8_t *dev_eui, uint8_t *dev_nonce, uint8_t *appkey)
{
	static struct loramac_phys_payload_join_request frame = {0};
	uint8_t out[16] = {0};
	uint8_t i;

	frame.m_hdr = 0;

	for (i = 0; i < 8; i++){
		frame.app_eui[i] = app_eui[7 - i];
	}
	for (i = 0; i < 8; i++){
		frame.dev_eui[i] = dev_eui[7 - i];
	}
	for (i = 0; i < 2; i++){
		frame.dev_nonce[i] = dev_nonce[1 - i];
	}

	int rc = crypto_auth(out, &frame.m_hdr, sizeof(struct loramac_phys_payload_join_request) - sizeof(frame.mic), appkey);
	if (rc != 0) {
		return -2;
	}
	for (i = 0; i < sizeof(frame.mic); i++){
		frame.mic[i] = out[sizeof(frame.mic) - 1 - i];
	}

	*jr_frame = &frame;

	return 0;
}

int32_t loramac_update_join_request_nonce(struct loramac_phys_payload_join_request *jr_frame, uint8_t *new_dev_nonce)
{
	jr_frame->dev_nonce[0] = new_dev_nonce[1];
	jr_frame->dev_nonce[1] = new_dev_nonce[0];
	return 0;
}

int32_t loramac_pack_join_accept(struct loramac_phys_payload_join_accept **ja_frame, uint8_t *app_nonce, uint8_t *net_id, uint8_t *dev_addr, uint8_t *dl_settings, uint8_t *rx_delay, uint8_t *appkey)
{
	static struct loramac_phys_payload_join_accept frame = {0};
	uint64_t cipher_length_out = 0;
	uint8_t cipher[sizeof(struct loramac_phys_payload_join_accept)];
	memset(cipher, 0, sizeof(struct loramac_phys_payload_join_accept));
	uint8_t nonce[16] = {0};
	uint8_t ad[9] = {0}; // MHDR 1 + DevAddr 4 + FCtrl 1 + FCnt 2 + FPort 1
	uint8_t i, j;
	j = 0;
	frame.m_hdr = LORAMAC_PHYS_PAYLOAD_JOIN_ACCEPT;
	nonce[j++] = LORAMAC_PHYS_PAYLOAD_JOIN_ACCEPT;
	for (i = 0; i < 3; i++){
		frame.app_nonce[i] = app_nonce[2 - i];
		nonce[j++] = app_nonce[2 - i];
	}
	for (i = 0; i < 4; i++){
		frame.dev_addr[i] = dev_addr[3 - i];
	}
	for (i = 0; i < 3; i++){
		frame.net_id[i] = net_id[2 - i];
	}
	frame.dl_settings = dl_settings[0];
	frame.rx_delay = rx_delay[0];
	// Don't count MHDR + MIC
	uint64_t frame_len = sizeof(struct loramac_phys_payload_join_accept) - 1 - 16;
	// cipher[1] cause cipher[0] is ad (MHDR)
	int rc = crypto_aead_encrypt(&cipher[1], &cipher_length_out, frame.app_nonce, frame_len, &frame.m_hdr, 1, NULL, nonce, appkey);
	if (rc != 0) {
		return -2;
	}
	// Assign ciphertext back to struct loramac_phys_payload_join_accept
	uint8_t *frame_ptr = &frame.app_nonce[0];

	frame.m_hdr = LORAMAC_PHYS_PAYLOAD_JOIN_ACCEPT;
	for (i = 0; i < 12; i++) {
		frame_ptr[i] = cipher[i + 1];
	}
	for (i = 0; i < sizeof(frame.mic); i++){
		frame.mic[i] = cipher[i + 13];
	}
	*ja_frame = &frame;

	return 0;
}
