#include "user.h"
#include "simple_random.h"
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define APPROXIM8(x) (((x+7)/8)*8)
#define JPEG_NEW_H APPROXIM8(IMG_HEIGHT)
#define JPEG_NEW_W APPROXIM8(IMG_WIDTH)
#define JPEG_MAX_BITS 512
#define JPEG_MAX_AC   64

#ifdef USER_JPEG_COMPRESS
#include "data/jpeg_image.h"
#else
static uint8_t mat_in[3][IMG_HEIGHT][IMG_WIDTH];
#endif

typedef struct {
    uint16_t bits[JPEG_MAX_BITS];
    uint32_t count;
    uint8_t available_bits;
    uint16_t next;
} BitStream_t;

typedef struct {
    int first;
    int second;
} ac_pair_t;

static uint8_t in_img[3][IMG_HEIGHT][IMG_WIDTH];
static uint8_t in_preprocessed_img[3][IMG_HEIGHT][IMG_WIDTH];
static uint8_t preprocessed_img[3][JPEG_NEW_H][JPEG_NEW_W];
static double dct_img[3][JPEG_NEW_H][JPEG_NEW_W];
static BitStream_t compressed_img;
static int last_dc = 0;

crc crc_jpeg_compress;

MEASURE_GLOBAL_VARIABLES()

static const uint8_t quant_luminance[8][8] = {
    {16,11,10,16,24,40,51,61},
    {12,12,14,19,26,58,60,55},
    {14,13,16,24,40,57,69,56},
    {14,17,22,29,51,87,80,62},
    {18,22,37,56,68,109,103,77},
    {24,35,55,64,81,104,113,92},
    {49,64,78,87,103,121,120,101},
    {72,92,95,98,112,100,130,99}
};

static const uint8_t quant_chrominance[8][8] = {
    {17,18,24,47,99,99,99,99},
    {18,21,26,66,99,99,99,99},
    {24,26,56,99,99,99,99,99},
    {47,66,99,99,99,99,99,99},
    {99,99,99,99,99,99,99,99},
    {99,99,99,99,99,99,99,99},
    {99,99,99,99,99,99,99,99},
    {99,99,99,99,99,99,99,99}
};

static const uint8_t DC_Y_codes_size[12] = {2, 4, 5, 6, 7, 8, 10, 12, 14, 16, 18, 20};
static const uint16_t DC_Y_values[12] = {0x00,0x02,0x03,0x04,0x05,0x6,0x0E,0x1E,0x3E,0x7E,0xFE,0x1FE};

static const uint8_t AC_Y_codes_size[16][11] = {
    {4 ,3 ,4 ,6 ,8 ,10,12,14,18,25,26},
    {0 ,5 ,8 ,10,13,16,22,23,24,25,26},
    {0 ,6 ,10,13,20,21,22,23,24,25,26},
    {0 ,7 ,11,14,20,21,22,23,24,25,26},
    {0 ,7 ,12,19,20,21,22,23,24,25,26},
    {0 ,8 ,12,19,20,21,22,23,24,25,26},
    {0 ,8 ,13,19,20,21,22,23,24,25,26},
    {0 ,9 ,13,19,20,21,22,23,24,25,26},
    {0 ,9 ,17,19,20,21,22,23,24,25,26},
    {0 ,10,18,19,20,21,22,23,24,25,26},
    {0 ,10,18,19,20,21,22,23,24,25,26},
    {0 ,10,18,19,20,21,22,23,24,25,26},
    {0 ,11,18,19,20,21,22,23,24,25,26},
    {0 ,12,18,19,20,21,22,23,24,25,26},
    {0 ,13,18,19,20,21,22,23,24,25,26},
    {12,17,18,19,20,21,22,23,24,25,26}
};

static const uint16_t AC_Y_values[16][11] = {
    {0xa, 0x0, 0x1, 0x4, 0xb, 0x1a, 0x38, 0x78, 0x3f6, 0xff82, 0xff83},
    {0x0, 0xc, 0x39, 0x79, 0x1f6, 0x7f6, 0xff84, 0xff85, 0xff86, 0xff87, 0xff88},
    {0x0, 0x1b, 0xf8, 0x3f7, 0xff89, 0xff8a, 0xff8b, 0xff8c, 0xff8d, 0xff8e, 0xff8f},
    {0x0, 0x3a, 0x1f7, 0x7f7, 0xff90, 0xff91, 0xff92, 0xff93, 0xff94, 0xff95, 0xff96},
    {0x0, 0x3b, 0x3f8, 0xff97, 0xff98, 0xff99, 0xff9a, 0xff9b, 0xff9c, 0xff9d, 0xff9e},
    {0x0,0x7a, 0x3f9, 0xff9f, 0xffa0, 0xffa1, 0xffa2, 0xffa3, 0xffa4, 0xffa5, 0xffa6},
    {0x0, 0x7b, 0x7f8, 0xffa7, 0xffa8, 0xffa9, 0xffaa, 0xffab, 0xffac, 0xffad, 0xffae},
    {0x0, 0xf9, 0x7f9, 0xffaf, 0xffb0, 0xffb1, 0xffb2, 0xffb3, 0xffb4, 0xffb5, 0xffb6},
    {0x0, 0xfa, 0x7fc0, 0xffb7, 0xffb8, 0xffb9, 0xffba, 0xffbb, 0xffbc, 0xffbd, 0xffbe},
    {0x0, 0x1f8, 0xffbf, 0xffc0, 0xffc1, 0xffc2, 0xffc3, 0xffc4, 0xffc5, 0xffc6, 0xffc7},
    {0x0, 0x1f9, 0xffc8, 0xffc9, 0xffca, 0xffcb, 0xffcc, 0xffcd, 0xffce, 0xffcf, 0xffd0},
    {0x0, 0x1fa, 0xffd1, 0xffd2, 0xffd3, 0xffd4, 0xffd5, 0xffd6, 0xffd7, 0xffd8, 0xffd9},
    {0x0, 0x3fa, 0xffda, 0xffdb, 0xffdc, 0xffdd, 0xffde, 0xffdf, 0xffe0, 0xffe1, 0xffe2},
    {0x0, 0x7fa, 0xffe3, 0xffe4, 0xffe5, 0xffe6, 0xffe7, 0xffe8, 0xffe9, 0xffea, 0xffeb},
    {0x0, 0xff6, 0xffec, 0xffed, 0xffee, 0xffef, 0xfff0, 0xfff1, 0xfff2, 0xfff3, 0xfff4},
    {0xff7, 0xfff5, 0xfff6, 0xfff7, 0xfff8, 0xfff9, 0xfffa, 0xfffb, 0xfffc, 0xfffd, 0xfffe}
};

static const uint8_t DC_C_codes_size[12] = {3, 3, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
static const uint16_t DC_C_values[12] = {0x00,0x01,0x02,0x06,0x0E,0x1E,0x3E,0x7E,0xFE,0x1FE,0x3FE,0x7FE};

static const uint8_t AC_C_codes_size[16][11] = {
    {2,2,3,4,5,5,6,7,9,10,12},
    {0,4,6,8,9,11,12,16,16,16,16},
    {0,5,8,10,12,15,16,16,16,16,16},
    {0,5,8,10,12,16,16,16,16,16,16},
    {0,6,9,16,16,16,16,16,16,16,16},
    {0,6,10,16,16,16,16,16,16,16,16},
    {0,7,11,16,16,16,16,16,16,16,16},
    {0,7,11,16,16,16,16,16,16,16,16},
    {0,8,16,16,16,16,16,16,16,16,16},
    {0,9,16,16,16,16,16,16,16,16,16},
    {0,9,16,16,16,16,16,16,16,16,16},
    {0,9,16,16,16,16,16,16,16,16,16},
    {0,9,16,16,16,16,16,16,16,16,16},
    {0,11,16,16,16,16,16,16,16,16,16},
    {0,14,16,16,16,16,16,16,16,16,16},
    {10,15,16,16,16,16,16,16,16,16,16}
};

static const uint16_t AC_C_values[16][11] = {
    {0x0, 0x1, 0x4, 0xa, 0x18, 0x19, 0x38, 0x78, 0x1f4, 0x3f6, 0xff4},
    {0x0, 0xb, 0x39, 0xf6, 0x1f5, 0x7f6, 0xff5, 0xff88, 0xff89, 0xff8a, 0xff8b},
    {0x0, 0x1a, 0xf7, 0x3f7, 0xff6, 0x7fc2, 0xff8c, 0xff8d, 0xff8e, 0xff8f, 0xff90},
    {0x0, 0x1b, 0xf8, 0x3f8, 0xff7, 0xff91, 0xff92, 0xff93, 0xff94, 0xff95, 0xff96},
    {0x0, 0x3a, 0x1f6, 0xff97, 0xff98, 0xff99, 0xff9a, 0xff9b, 0xff9c, 0xff9d, 0xff9e},
    {0x0, 0x3b, 0x3f9, 0xff9f, 0xffa0, 0xffa1, 0xffa2, 0xffa3, 0xffa4, 0xffa5, 0xffa6},
    {0x0, 0x79, 0x7f7, 0xffa7, 0xffa8, 0xffa9, 0xffaa, 0xffab, 0xffac, 0xffad, 0xffae},
    {0x0, 0x7a, 0x7f8, 0xffaf, 0xffb0, 0xffb1, 0xffb2, 0xffb3, 0xffb4, 0xffb5, 0xffb6},
    {0x0, 0xf9, 0xffb7, 0xffb8, 0xffb9, 0xffba, 0xffbb, 0xffbc, 0xffbd, 0xffbe, 0xffbf},
    {0x0, 0x1f7, 0xffc0, 0xffc1, 0xffc2, 0xffc3, 0xffc4, 0xffc5, 0xffc6, 0xffc7, 0xffc8},
    {0x0, 0x1f8, 0xffc9, 0xffca, 0xffcb, 0xffcc, 0xffcd, 0xffce, 0xffcf, 0xffd0, 0xffd1},
    {0x0, 0x1f9, 0xffd2, 0xffd3, 0xffd4, 0xffd5, 0xffd6, 0xffd7, 0xffd8, 0xffd9, 0xffda},
    {0x0, 0x1fa, 0xffdb, 0xffdc, 0xffdd, 0xffde, 0xffdf, 0xffe0, 0xffe1, 0xffe2, 0xffe3},
    {0x0, 0x7f9, 0xffe4, 0xffe5, 0xffe6, 0xffe7, 0xffe8, 0xffe9, 0xffea, 0xffeb, 0xffec},
    {0x0, 0x3fe0, 0xffed, 0xffee, 0xffef, 0xfff0, 0xfff1, 0xfff2, 0xfff3, 0xfff4, 0xfff5},
    {0x3fa, 0x7fc3, 0xfff6, 0xfff7, 0xfff8, 0xfff9, 0xfffa, 0xfffb, 0xfffc, 0xfffd, 0xfffe}
};

static void bitstream_clear(BitStream_t *bs)
{
    bs->count = 0;
    bs->next = 0;
    bs->available_bits = 16;
}

static void bitstream_write_bits(BitStream_t *bs, uint16_t n, uint8_t len)
{
    while (len > 0) {
        if (len > bs->available_bits) {
            uint8_t diff = len - bs->available_bits;
            bs->next = bs->next | (uint16_t)(n >> diff);
            len -= bs->available_bits;
            bs->available_bits = 0;
        } else {
            uint8_t diff = bs->available_bits - len;
            bs->next = bs->next | (uint16_t)(n << diff);
            len = 0;
            bs->available_bits = diff;
        }

        if (bs->available_bits == 0) {
            if (bs->count < JPEG_MAX_BITS) {
                bs->bits[bs->count++] = bs->next;
            }
            bs->next = 0;
            bs->available_bits = 16;
        }
    }
}

static void img_preprocess(void)
{
    for (int i = 0; i < IMG_HEIGHT; i++) {
        for (int j = 0; j < IMG_WIDTH; j++) {
            char r = (char)in_img[0][i][j];
            char g = (char)in_img[1][i][j];
            char b = (char)in_img[2][i][j];
            in_preprocessed_img[0][i][j] = (uint8_t)((0.299 * r + 0.587 * g + 0.114 * b) - 128);
            in_preprocessed_img[1][i][j] = (uint8_t)((-0.16874 * r - 0.33126 * g + 0.5 * b) - 128);
            in_preprocessed_img[2][i][j] = (uint8_t)((0.5 * r - 0.41869 * g - 0.08131 * b) - 128);
        }
    }
}

static void resize_preprocessed(void)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < JPEG_NEW_H; j++) {
            for (int z = 0; z < JPEG_NEW_W; z++) {
                int z_clamped = (z >= IMG_WIDTH) ? (IMG_WIDTH - 1) : z;
                int j_clamped = (j >= IMG_HEIGHT) ? (IMG_HEIGHT - 1) : j;
                preprocessed_img[i][j][z] = in_preprocessed_img[i][j_clamped][z_clamped];
            }
        }
    }
}

static void quantized_DCT(int ch, int id_x, int id_y, const uint8_t quant_table[8][8])
{
    for (int i = 0; i < 8; i++) {
        for (int j = 0; j < 8; j++) {
            double alpha_i = (i == 0) ? (1.0 / sqrt(2.0)) : 1.0;
            double alpha_j = (j == 0) ? (1.0 / sqrt(2.0)) : 1.0;
            double dct = 0.0;
            for (int x = 0; x < 8; x++) {
                for (int y = 0; y < 8; y++) {
                    dct += preprocessed_img[ch][id_x + x][id_y + y] *
                           cos(((2 * x + 1) * i * M_PI) / 16.0) *
                           cos(((2 * y + 1) * j * M_PI) / 16.0);
                }
            }
            dct = 0.25 * alpha_i * alpha_j * dct;
            dct_img[ch][id_x + i][id_y + j] = round(dct / quant_table[i][j]);
        }
    }
}

static int get_cat(int val)
{
    val = abs(val);
    int b = 0;
    while (val) {
        b++;
        val >>= 1;
    }
    return b;
}

static void entropy_encoding(
    int ch,
    int id_x,
    int id_y,
    const uint16_t dc_values[12],
    const uint8_t dc_codes_size[12],
    const uint16_t ac_values[16][11],
    const uint8_t ac_codes_size[16][11]
)
{
    int inv = 0;
    int zeroes = 0;
    int dc, dc_cat;

    if (id_x == 0 && id_y == 0) {
        dc = (int)dct_img[ch][id_x][id_y];
    } else {
        dc = (int)dct_img[ch][id_x][id_y] - last_dc;
    }
    dc_cat = get_cat(dc);

    if (dc < 0) {
        dc += ((1 << dc_cat) - 1);
    }

    bitstream_write_bits(&compressed_img, (uint16_t)((dc_values[dc_cat] << dc_cat) | dc), dc_codes_size[dc_cat]);

    ac_pair_t AC_component[JPEG_MAX_AC];
    int ac_count = 0;

    for (int col = 1; col < 8; col++, inv = !inv) {
        if (inv) {
            for (int x = col, y = 0; x >= 0 && y <= col; x--, y++) {
                int ac = (int)dct_img[ch][x + id_x][y + id_y];
                if (ac || zeroes == 15) {
                    if (ac_count < JPEG_MAX_AC) {
                        AC_component[ac_count].first = zeroes;
                        AC_component[ac_count].second = ac;
                        ac_count++;
                    }
                    zeroes = 0;
                } else {
                    zeroes++;
                }
            }
        } else {
            for (int x = 0, y = col; x < 8 && y >= 0; x++, y--) {
                int ac = (int)dct_img[ch][x + id_x][y + id_y];
                if (ac || zeroes == 15) {
                    if (ac_count < JPEG_MAX_AC) {
                        AC_component[ac_count].first = zeroes;
                        AC_component[ac_count].second = ac;
                        ac_count++;
                    }
                    zeroes = 0;
                } else {
                    zeroes++;
                }
            }
        }
    }

    for (int row = 1; row < 8; row++, inv = !inv) {
        if (inv) {
            for (int x = 7, y = row; x >= 1 && y < 8; x--, y++) {
                int ac = (int)dct_img[ch][x + id_x][y + id_y];
                if (ac || zeroes == 15) {
                    if (ac_count < JPEG_MAX_AC) {
                        AC_component[ac_count].first = zeroes;
                        AC_component[ac_count].second = ac;
                        ac_count++;
                    }
                    zeroes = 0;
                } else {
                    zeroes++;
                }
            }
        } else {
            for (int x = row, y = 7; x < 7 && y >= 0; x++, y--) {
                int ac = (int)dct_img[ch][x + id_x][y + id_y];
                if (ac || zeroes == 15) {
                    if (ac_count < JPEG_MAX_AC) {
                        AC_component[ac_count].first = zeroes;
                        AC_component[ac_count].second = ac;
                        ac_count++;
                    }
                    zeroes = 0;
                } else {
                    zeroes++;
                }
            }
        }
    }

    for (int k = 0; k < ac_count; k++) {
        int nz_cat = get_cat(AC_component[k].first);
        int ac_cat = get_cat(AC_component[k].second);
        bitstream_write_bits(
            &compressed_img,
            (uint16_t)((ac_values[nz_cat][ac_cat] << ac_cat) | dc),
            ac_codes_size[nz_cat][ac_cat]
        );
    }
}

static void compressor_update_in_img(uint8_t src[3][IMG_HEIGHT][IMG_WIDTH])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < IMG_HEIGHT; j++) {
            for (int z = 0; z < IMG_WIDTH; z++) {
                in_img[i][j][z] = src[i][j][z];
            }
        }
    }
    bitstream_clear(&compressed_img);
}

static void compressor_compress(void)
{
    img_preprocess();
    resize_preprocessed();

    for (int i = 0; i < JPEG_NEW_H; i += 8) {
        for (int j = 0; j < JPEG_NEW_W; j += 8) {
            for (int c = 0; c < 3; c++) {
                if (c) {
                    quantized_DCT(c, i, j, quant_chrominance);
                    entropy_encoding(c, i, j, DC_C_values, DC_C_codes_size, AC_C_values, AC_C_codes_size);
                } else {
                    quantized_DCT(c, i, j, quant_luminance);
                    entropy_encoding(c, i, j, DC_Y_values, DC_Y_codes_size, AC_Y_values, AC_Y_codes_size);
                }
            }
        }
    }
}

void init_jpeg_compress(void)
{
#ifndef USER_JPEG_COMPRESS
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < IMG_HEIGHT; j++) {
            for (int z = 0; z < IMG_WIDTH; z++) {
                mat_in[i][j][z] = (uint8_t)(random_get() * 256.f);
            }
        }
    }
#endif
    compressor_update_in_img(mat_in);
}

void jpeg_compress_update_init_conditions(void)
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < IMG_HEIGHT; j++) {
            for (int z = 0; z < IMG_WIDTH; z++) {
                double rnd = random_get();
                mat_in[i][j][z] = (uint8_t)(rnd * (256 - mat_in[i][j][z] * rnd));
            }
        }
    }
    compressor_update_in_img(mat_in);
}

crc jpeg_compress(SemaphoreHandle_t xMutexRandom)
{
    xSemaphoreTake(xMutexRandom, TIMEOUT_QUEUES);
    jpeg_compress_update_init_conditions();
    xSemaphoreGive(xMutexRandom);

    MEASURE_START(jpeg_compress_result);

    compressor_compress();

    MEASURE_STOP(jpeg_compress_result);

    crc_jpeg_compress = 0;
    uint8_t bits[4] = {0};
    for (uint32_t idx = 0; idx < compressed_img.count; idx++) {
        uint16_t e = compressed_img.bits[idx];
        bits[0] = (uint8_t)(e ^ 0xff);
        bits[1] = (uint8_t)(e >> 8);
        crc_jpeg_compress ^= crc32(bits, sizeof(bits));
    }
    return crc_jpeg_compress;
}
