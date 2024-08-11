#ifndef CC20_P1305_H
#define CC20_P1305_H

void chaCha20AEADEncrypt(unsigned char *aad, // additional authenticated data (AAD)
                         unsigned int aadSize,
                         unsigned int *key,
                         unsigned int keySize,
                         unsigned int *iv, // if nonce is 64 bits then the first 32 bits of nonce is set to constants
                         unsigned int ivSize,
                         unsigned int *constant, // this is used when nonce is 64 bits
                         unsigned int constantSize,
                         unsigned char *plaintext,
                         int plaintextSize,
                         unsigned char *ciphertext,
                         unsigned int ciphertextSize,
                         unsigned char *tag);

void chaCha20AEADDecrypt(unsigned char *aad, // additional authenticated data (AAD)
                         unsigned int aadSize,
                         unsigned int *key,
                         unsigned int keySize,
                         unsigned int *iv, // if nonce is 64 bits then the first 32 bits of nonce is set to constants
                         unsigned int ivSize,
                         unsigned int *constant, // this is used when nonce is 64 bits
                         unsigned int constantSize,
                         unsigned char *plaintext,
                         int plaintextSize,
                         unsigned char *ciphertext,
                         unsigned int ciphertextSize,
                         unsigned char *received_tag,
                         unsigned char *calculated_tag);

#endif
												 