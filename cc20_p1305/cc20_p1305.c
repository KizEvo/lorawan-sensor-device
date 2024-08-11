#include <stdint.h>
#include <string.h>
#include "secrets.h"

#define INT_BITS (32U)

// https://datatracker.ietf.org/doc/html/rfc8439#section-2.3
typedef union ChaChaState
{
    struct
    {
        unsigned int constant[4];
        unsigned int key[8];
        unsigned int blockcount;
        unsigned int nonce[3];
    };
    unsigned int words[16];
} ChaChaState;

static int compare(const unsigned char *num1, const unsigned char *num2, size_t length1, size_t length2)
{
    int i;

    // Compare lengths
    if (length1 < length2)
        return -1;
    if (length1 > length2)
        return 1;

    // Compare byte by byte
    for (i = length1 - 1; i >= 0; i--)
    {
        if (num1[i] < num2[i])
            return -1;
        if (num1[i] > num2[i])
            return 1;
    }
    return 0; // Numbers are equal
}

static int subtract(unsigned char *result, const unsigned char *num1, const unsigned char *num2, size_t length1, size_t length2)
{
    if (compare(num1, num2, length1, length2) < 0)
    {
        return 1; // Subtraction would result in a negative number
    }

    size_t i = 0;
    int borrow = 0;

    // Iterate through each byte of the numbers
    for (i = 0; i < length1; i++)
    {
        // Perform subtraction on each byte, considering borrow
        int sub = num1[i] - (i < length2 ? num2[i] : 0) - borrow;
        borrow = sub < 0 ? 1 : 0;              // Update borrow
        result[i] = sub < 0 ? sub + 256 : sub; // Ensure result is in the range [0, 255]
    }

    // Handle any remaining borrow
    while (i > 0 && borrow && --i)
    {
        int sub = num1[i] - borrow;
        borrow = sub < 0 ? 1 : 0;              // Update borrow
        result[i] = sub < 0 ? sub + 256 : sub; // Ensure result is in the range [0, 255]
    }

    return 0; // Subtraction successful
}

static unsigned int leftRoll(unsigned int n, unsigned int shift)
{
    return (n << shift) | (n >> (INT_BITS - shift));
}

// Based on RFC 8439, ChaCha20 and Poly1305 for IETF Protocols
// https://datatracker.ietf.org/doc/html/rfc8439#section-2.1
static void doChaChaQuarter(unsigned int *a, unsigned int *b, unsigned int *c, unsigned *d)
{
    *a = (*a + *b) % UINT32_MAX;
    *d ^= *a;
    *d = leftRoll(*d, 16);
    *c = (*c + *d) % UINT32_MAX;
    *b ^= *c;
    *b = leftRoll(*b, 12);
    *a = (*a + *b) % UINT32_MAX;
    *d ^= *a;
    *d = leftRoll(*d, 8);
    *c = (*c + *d) % UINT32_MAX;
    *b ^= *c;
    *b = leftRoll(*b, 7);
}

static void innerBlock(ChaChaState *state)
{
    // Column rounds
    doChaChaQuarter(&state->words[0], &state->words[4], &state->words[8], &state->words[12]);
    doChaChaQuarter(&state->words[1], &state->words[5], &state->words[9], &state->words[13]);
    doChaChaQuarter(&state->words[2], &state->words[6], &state->words[10], &state->words[14]);
    doChaChaQuarter(&state->words[3], &state->words[7], &state->words[11], &state->words[15]);
    // Diagonal rounds
    doChaChaQuarter(&state->words[0], &state->words[5], &state->words[10], &state->words[15]);
    doChaChaQuarter(&state->words[1], &state->words[6], &state->words[11], &state->words[12]);
    doChaChaQuarter(&state->words[2], &state->words[7], &state->words[8], &state->words[13]);
    doChaChaQuarter(&state->words[3], &state->words[4], &state->words[9], &state->words[14]);
}

static void matrixAddChaChaState(ChaChaState *result, ChaChaState *b)
{
    for (unsigned char i = 0; i < 16; i++)
    {
        result->words[i] = result->words[i] + b->words[i];
    }
}

static unsigned char *serialize(ChaChaState *state)
{
    static unsigned char serializedData[64]; // a ChaChaState consist of 16 data words
                                             // each words contains 4 blocks of 8 bits (unsigned char) => 16 * 4 = 64 bytes
    for (unsigned char i = 0, wIdx = 0; i < 64; i += 4)
    {
        serializedData[i] = state->words[wIdx] & 0xFFU;
        serializedData[i + 1] = (state->words[wIdx] & 0xFF00U) >> 8U;
        serializedData[i + 2] = (state->words[wIdx] & 0xFF0000U) >> 16U;
        serializedData[i + 3] = (state->words[wIdx] & 0xFF000000U) >> 24U;
        wIdx++;
    }
    return serializedData;
}

// When you use this function, you need to free the serialized data that got returned
static unsigned char *chaCha20Block(unsigned int *keyArr, unsigned int blockcounter, unsigned int *nonceArr)
{
    ChaChaState state = {0};
    // Constants
    state.words[0] = 0x61707865UL;
    state.words[1] = 0x3320646eUL;
    state.words[2] = 0x79622d32UL;
    state.words[3] = 0x6b206574UL;
    // Initial state
    state.words[12] = blockcounter;
		int j = 0;
    for (unsigned char i = 4; i < 8 + 4; i++)
        state.words[i] = keyArr[j++];
		j = 0;
    for (unsigned char i = 13; i < 3 + 13; i++)
        state.words[i] = nonceArr[j++];
    // Save a copy of the initial state to do matrix addition later
    ChaChaState initState = state;
    for (unsigned char i = 0; i < 10; i++) // Run innerBlock 10 times => 20 ChaCha "rounds"
    {
        innerBlock(&state);
    }
    // Add two states
    matrixAddChaChaState(&state, &initState);
    // Serialize state and return it (little endian)
    return serialize(&state);
}

static void chaCha20XOR(unsigned char *encryptedMessage, int blockStart, int blockMax, int absBlockMax, unsigned char *block, unsigned char *keyStream)
{
    int sIdx = 0;
    for (int i = blockStart; i <= blockMax; i++)
    {
        encryptedMessage[i] = block[sIdx] ^ keyStream[sIdx];
        sIdx++;
    }
}

static unsigned char *chaCha20Encrypt(unsigned int *keyArr, unsigned int blockcounter, unsigned int *nonceArr, unsigned char *message, int mesSize)
{
    static unsigned char encryptedMessage[1000] = {0}; // 1000 bytes static storage
    unsigned char *keyStream;
    unsigned char *block;
    int i;
    for (i = 0; i <= (mesSize / 64) - 1; i++)
    {
        keyStream = chaCha20Block(keyArr, blockcounter + i, nonceArr);
        // Get plaintext/message block
        block = &message[i * 64];
        // XOR block with key
        chaCha20XOR(encryptedMessage, i * 64, i * 64 + 63, 256, block, keyStream);
    }
    if (mesSize % 64 != 0)
    {
        i = mesSize / 64;
        keyStream = chaCha20Block(keyArr, blockcounter + i, nonceArr);
        block = &message[i * 64];
        chaCha20XOR(encryptedMessage, i * 64, mesSize - 1, 256, block, keyStream);
    }
    return encryptedMessage;
}

static void poly1305ClampR(unsigned char *r)
{
    // Top 4 bits cleared
    r[3] &= 15U;
    r[7] &= 15U;
    r[11] &= 15U;
    r[15] &= 15U;
    // Bottom 2 bits cleared
    r[4] &= 252U;
    r[8] &= 252U;
    r[12] &= 252U;
}

static void poly1305KeyToOctets(unsigned int *keyArr, int totalOctet, unsigned char *result)
{
    unsigned int tmpValue = 0;
    unsigned char i = 0;
    unsigned char nextItem = 0;
    while (i < totalOctet)
    {
        for (unsigned int shift = 0; shift < 32; shift += 8)
        {
            tmpValue = ((*(keyArr + nextItem)) >> shift) & 0xFF;
            result[i] = (unsigned char)tmpValue; // result[] is stored in little endian format => [0] = LSByte ... [n] = MSByte
            i++;
        }
        nextItem++;
    }
}

static void poly1305DivideMsgTo16BytesBlock(unsigned char *message, int mesSize, unsigned char *result)
{
    unsigned char i;
    for (i = 0; i < mesSize; i++)
    {
        result[i] = *(message + i);
    }
    if (mesSize == 16)
        result[16] = 0x1; // 2^128
    else
        result[i] = 0x01; // next block 0x01
}

static unsigned char add(unsigned char *result, unsigned char a, unsigned char b, unsigned char carry_in)
{
    unsigned int sum = a + b + carry_in;
    *result = (unsigned char)sum;
    return (sum > 0xFF); // Carry out if sum exceeds 255 (0xFF)
}

static void multiByteAdder(unsigned char *result, unsigned char *a, unsigned char *b, int length)
{
    unsigned char carry = 0;

    for (int i = 0; i < length; i++)
    {
        carry = add(&result[i], a[i], b[i], carry);
    }
    result[length] += carry; // Store the final carry
}

static void multiByteMultiply(unsigned char *result, unsigned char *a, unsigned char *b, int length)
{
    for (int i = 0; i < length; i++)
    {
        unsigned char carry = 0;
        for (int j = 0; j < length; j++)
        {
            unsigned int product = a[i] * b[j] + result[i + j] + carry;
            result[i + j] = product & 0xFF;
            carry = product >> 8;
        }
        result[i + length] += carry;
    }
}

static void multiByteMultiplyByTwo(unsigned char *result, const unsigned char *a, int length)
{
    unsigned char carry = 0;

    // Multiply each byte of 'a' by 2 and add carry
    for (int i = 0; i < length; i++)
    {
        unsigned int product = a[i] * 2 + carry;
        result[i] = product & 0xFF; // Store the least significant byte of the product
        carry = product >> 8;       // Update carry to be added to the next byte
    }

    result[length] += carry;
}

static void multiByteModulo(unsigned char *r, unsigned char *num1, unsigned char *num2, size_t length1, size_t length2)
{
    unsigned char d[length2];
    memcpy(r, num1, length1);
    while (compare(r, num2, length1, length2) >= 1)
    {
        memcpy(d, num2, length2);
        while (compare(r, d, length1, length2) >= 1) // Multiply by 2 until larger than divisor
        {
            multiByteMultiplyByTwo(d, d, length2);
        }
        // Shift right 1 bit
        unsigned char carry = 0;
        for (int i = length2 - 1; i >= 0; i--)
        {
            unsigned int current = d[i] + (carry << 8); // Add carry from the previous byte
            d[i] = current >> 1;                        // Store the result of division by 2
            carry = current & 1;                        // Update carry
        }
        subtract(r, r, d, length1, length2);
    }
    // When dividend is divisible by divisor
    for (unsigned int i = 0; i < length2; i++)
    {
        if (r[i] != num2[i])
            return;
    }
    r[0] = 0;
}

static void poly1305MAC(unsigned char *result, unsigned char *message, int mesSize, unsigned int *keyArr)
{
    unsigned char s[17] = {0}; // 1 extra byte for multiByteAdder
    unsigned char r[18] = {0}; // 2 extra bytes for multiByteMultiply
    unsigned char p[36] = {
        0xfb, 0xff, 0xff, 0xff, // 0xfffffffbU
        0xff, 0xff, 0xff, 0xff, // 0xffffffffU
        0xff, 0xff, 0xff, 0xff, // 0xffffffffU
        0xff, 0xff, 0xff, 0xff, // 0xffffffffU
        0x03, 0x00, 0x00, 0x00  // 3U
    };
    int tmpMsgSize = 16;
    unsigned char acc[36] = {0}; // accumulator 17 bytes, 1 extra byte for carry

    poly1305KeyToOctets(keyArr, 16, r); // first half of the 256bits key
    poly1305ClampR(r);
    poly1305KeyToOctets(&keyArr[4], 16, s); // second half of the 256bits key

    int condition = mesSize % 16 ? (mesSize / 16) + 1 : mesSize / 16;
    for (int i = 1; i <= condition; i++)
    {
        unsigned char n[17] = {0};          // 17 bytes so that we can add one bit beyond the 16 byte block (e.g 2^128) or any power of 2 that is divisible by 8 (e.g 2^120) if the last block does not go all the way to 128 bits
        unsigned char mul_result[36] = {0}; // 2 * total length of (r) or (acc)
        if (i == (mesSize / 16) + 1)
            tmpMsgSize = mesSize - ((i - 1) * 16);

        poly1305DivideMsgTo16BytesBlock(&message[(i - 1) * 16], tmpMsgSize, n);
        multiByteAdder(acc, acc, n, tmpMsgSize + 1);
        multiByteMultiply(mul_result, r, acc, sizeof(r));
        multiByteModulo(acc, mul_result, p, sizeof(mul_result), sizeof(p));
    }
    multiByteAdder(acc, acc, s, sizeof(s));
    for (int i = 0; i < 16; i++)
    {
        result[i] = acc[i];
    }
}

static unsigned char *poly1305KeyGen(unsigned int *keyArr, unsigned int *nonceArr)
{
    unsigned char *block = chaCha20Block(keyArr, 0, nonceArr);
    return block;
}

static void transformOtkToInt(unsigned int *result, unsigned char *key, unsigned int keySize)
{
    unsigned int i_r = 0;
    for (unsigned int i = 0; i < keySize; i += 4)
    {
        result[i_r] = key[i];
        result[i_r] |= key[i + 1] << 8U;
        result[i_r] |= key[i + 2] << 16U;
        result[i_r] |= key[i + 3] << 24U;
        i_r++;
    }
}

static void pad16(unsigned int *size)
{
    while (*size % 16 != 0)
    {
        *size = *size + 1;
    }
}

static void numTo8LittleEnBytes(unsigned char *result, unsigned int num)
{
    int bytesIdx = 0;
    int tmpNum = 0;
    while (bytesIdx < 4)
    {
        tmpNum = (num >> (0x8 * bytesIdx)) & 0xFF;
        *(result + bytesIdx) = tmpNum;
        bytesIdx++;
    }
}

static void formNonce(unsigned int *nonce, unsigned int *iv, unsigned int ivSize, unsigned int *constant)
{
    if (ivSize <= 2) // 64 bits
    {
        nonce[0] = constant[0];
        nonce[1] = iv[0];
        nonce[2] = iv[1];
    }
    else // 96 bits
    {
        nonce[0] = iv[0];
        nonce[1] = iv[1];
        nonce[2] = iv[2];
    }
}

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
                         unsigned char *tag)
{
    unsigned int otk[8] = {0};
    unsigned int nonce[constantSize + ivSize];
    formNonce(nonce, iv, ivSize, constant);                     // nonce = constant | iv
    transformOtkToInt(otk, poly1305KeyGen(key, nonce), 64 / 2); // after poly1305KeyGen run, we have 512 bit state -> take only first 256 bit => 32 bytes
    unsigned char *tmpCipherText;
    tmpCipherText = chaCha20Encrypt(key, 1, nonce, plaintext, plaintextSize);
    for (int i = 0; i < plaintextSize; i++)
    {
        ciphertext[i] = tmpCipherText[i];
    }
    unsigned int mac_size = 0;
    unsigned char mac_data[ciphertextSize + aadSize + 100]; /*variable length array*/
    memset(mac_data, '\0', sizeof(mac_data));
    for (unsigned int i = 0; i < aadSize; i++)
    {
        mac_data[i] = aad[i];
    }
    mac_size += aadSize;
    pad16(&mac_size);
    memcpy(&mac_data[mac_size], ciphertext, ciphertextSize);
    for (unsigned int i = mac_size; i < mac_size + ciphertextSize; i++)
    {
        mac_data[i] = ciphertext[i - mac_size];
    }
    mac_size += ciphertextSize;
    pad16(&mac_size);
    // Append the length of aad in octets to mac_data
    // Do the same for ciphertext
    numTo8LittleEnBytes(&mac_data[mac_size], aadSize);
    mac_size += 8;
    numTo8LittleEnBytes(&mac_data[mac_size], ciphertextSize);
    mac_size += 8;
    // Get the tag
    poly1305MAC(tag, mac_data, mac_size, otk);
}

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
                         unsigned char *calculated_tag)
{
    unsigned int otk[8] = {0};
    unsigned int nonce_decrypt[constantSize + ivSize];
    unsigned int mac_size = 0;
    formNonce(nonce_decrypt, iv, ivSize, constant);
    transformOtkToInt(otk, poly1305KeyGen(key, nonce_decrypt), 64 / 2); // after poly1305KeyGen run, we have 512 bit state -> take only first 256 bit => 32 bytes
    unsigned char mac_data[ciphertextSize + aadSize + 100];             /*variable length array*/
    memset(mac_data, '\0', sizeof(mac_data));
    for (unsigned int i = 0; i < aadSize; i++)
    {
        mac_data[i] = aad[i];
    }
    mac_size += aadSize;
    pad16(&mac_size);
    memcpy(&mac_data[mac_size], ciphertext, ciphertextSize);
    for (unsigned int i = mac_size; i < mac_size + ciphertextSize; i++)
    {
        mac_data[i] = ciphertext[i - mac_size];
    }
    mac_size += ciphertextSize;
    pad16(&mac_size);
    // Append the length of aad in octets to mac_data
    // Do the same for ciphertext
    numTo8LittleEnBytes(&mac_data[mac_size], aadSize);
    mac_size += 8;
    numTo8LittleEnBytes(&mac_data[mac_size], ciphertextSize);
    mac_size += 8;
    // Calculate tag and compare with received tag
    poly1305MAC(calculated_tag, mac_data, mac_size, otk);
    // Compare below here
    // if (...)
    unsigned char *tmpPlaintext;
    tmpPlaintext = chaCha20Encrypt(key, 1, nonce_decrypt, ciphertext, plaintextSize);
    for (int i = 0; i < plaintextSize; i++)
    {
        plaintext[i] = tmpPlaintext[i];
    }
}
