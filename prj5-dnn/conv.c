#include "printf.h"
#include "trap.h"
#include "mul.h"
#include "div.h"
#include "perf_cnt.h"

#define FRAC_BIT 10

#define RD_ADDR 135106448
#define RD_SIZE_D0 1
#define RD_SIZE_D1 1
#define RD_SIZE_D2 28
#define RD_SIZE_D3 28

#define WEIGHT_ADDR 134217728
#define WEIGHT_SIZE_D0 20
#define WEIGHT_SIZE_D1 1
#define WEIGHT_SIZE_D2 5
#define WEIGHT_SIZE_D3 5

#define WR_ADDR 135108240
#define WR_SIZE_D0 1
#define WR_SIZE_D1 20
#define WR_SIZE_D2 12
#define WR_SIZE_D3 12

#define KERN_ATTR_CONV_PAD 0
#define KERN_ATTR_CONV_STRIDE 1
#define KERN_ATTR_POOL_PAD 0
#define KERN_ATTR_POOL_KERN_SIZE 2
#define KERN_ATTR_POOL_STRIDE 2

//MMIO register address of DNN accelerator
#define GPIO_START_ADDR    0x60030000
#define GPIO_DONE_ADDR     0x60030008

struct size_vec4
{
	unsigned d0;
	unsigned d1;
	unsigned d2;
	unsigned d3;
};

struct mem_addr
{
	unsigned rd_addr;
	unsigned weight_addr;
	unsigned wr_addr;
};

int mul(short a, short b)
{
#ifndef USE_MUL
	int ans = mul_ll(a, b);
#else
	int ans = a * b;
#endif
	return ans;
}

struct mem_addr addr = {RD_ADDR, WEIGHT_ADDR, WR_ADDR};
struct size_vec4 rd_size = {RD_SIZE_D0, RD_SIZE_D1, RD_SIZE_D2, RD_SIZE_D3};
struct size_vec4 wr_size = {WR_SIZE_D0, WR_SIZE_D1, WR_SIZE_D2, WR_SIZE_D3};
struct size_vec4 weight_size = {WEIGHT_SIZE_D0, WEIGHT_SIZE_D1, WEIGHT_SIZE_D2, WEIGHT_SIZE_D3};

struct size_vec4 conv_size;

extern char _binary_data_result_bin_start[];
extern char _binary_data_result_bin_size[];

void convolution()
{
	short *in = (short *)addr.rd_addr;
	short *weight = (short *)addr.weight_addr;
	short *out = (short *)addr.wr_addr;

	unsigned output_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = rd_size.d3;
	unsigned input_fm_h = rd_size.d2;

	unsigned pad = KERN_ATTR_CONV_PAD;
	unsigned pad_len = pad << 1;

	unsigned conv_out_w = rd_size.d3 - weight_size.d3 + pad_len;
	unsigned conv_out_h = rd_size.d2 - weight_size.d2 + pad_len;

	unsigned stride = KERN_ATTR_CONV_STRIDE;

	conv_out_w = div(conv_out_w, stride);
	conv_out_h = div(conv_out_h, stride);

	conv_out_w++;
	conv_out_h++;

	conv_size.d0 = wr_size.d0;
	conv_size.d1 = wr_size.d1;
	conv_size.d2 = conv_out_h;
	conv_size.d3 = conv_out_w;

	// TODO: Please add your implementation here
	int input_area = mul(input_fm_w, input_fm_h);
	int weight_area = mul(weight_size.d2, weight_size.d3) + 1;	// including bias

	int no, ni;
	int x, y;
	int kx, ky;
	int iw = 0, ih = 0;

	int conv_out_pos = output_offset;
	int bias_pos = 0;
	for (no = 0; no < conv_size.d1; no++) {
		int input_pos_d1 = 0;
		for (ni = 0; ni < rd_size.d1; ni++) {
			int stride_y = 0;
			int input_pos_d2 = 0;
			for (int i = 0; i < pad; i++) input_pos_d2 -= input_fm_w;
			for (y = 0; y < conv_out_h; y++) {
				int stride_x = 0;
				for (x = 0; x < conv_out_w; x++) {
					if (ni == 0) out[conv_out_pos] = weight[bias_pos];
					
					int product = 0;
					int weight_offset = 0;
					int input_pos_d3 = 0;
					for (ky = 0; ky < weight_size.d2; ky++) {
						for (kx = 0; kx < weight_size.d3; kx++) {
							iw = kx + stride_x;
							ih = ky + stride_y;
							int input_pos = input_pos_d1 + input_pos_d2 + input_pos_d3 + iw - pad + input_offset;
							int weight_pos = bias_pos + weight_offset + 1;
							int input_data;
							if (iw < pad || iw >= pad + input_fm_w || ih < pad || ih >= pad + input_fm_h) {
								input_data = 0;
							} else input_data = in[input_pos];

							product += mul(input_data, weight[weight_pos]);
							weight_offset++;
						}
						input_pos_d3 += input_fm_w;
					}
					out[conv_out_pos] += ((short)(product >> FRAC_BIT) & 0x7fff) | ((short)(product >> 16) & 0x8000);

					conv_out_pos++;
					stride_x += stride;
				}
				stride_y += stride;
				for (int i = 0; i < stride; i++) input_pos_d2 += input_fm_w;
			}
			input_pos_d1 += input_area;
		}
		bias_pos += weight_area;
	}
}

void pooling()
{
	short *out = (short *)addr.wr_addr;

	unsigned output_offset = 0;
	unsigned input_offset = 0;

	unsigned input_fm_w = conv_size.d3;
	unsigned input_fm_h = conv_size.d2;

	unsigned pad = KERN_ATTR_POOL_PAD;
	unsigned pad_len = pad << 1;

	unsigned pad_w_test = conv_size.d3 - KERN_ATTR_POOL_KERN_SIZE;
	unsigned pad_h_test = conv_size.d2 - KERN_ATTR_POOL_KERN_SIZE;

	unsigned pool_out_w = pad_w_test + pad_len;
	unsigned pool_out_h = pad_h_test + pad_len;

	unsigned stride = KERN_ATTR_POOL_STRIDE;

	unsigned pad_w_test_remain = pad_w_test - mul(div(pad_w_test, stride), stride);
	unsigned pad_h_test_remain = pad_h_test - mul(div(pad_h_test, stride), stride);

	pool_out_w = div(pool_out_w, stride);
	pool_out_h = div(pool_out_h, stride);
	pool_out_w++;
	pool_out_h++;

	if ((!pad) && (pad_w_test_remain || pad_h_test_remain))
	{
		pool_out_w++;
		pool_out_h++;
	}

	// TODO: Please add your implementation here
	int no;
	int x, y;
	int kx, ky;

	int input_area = mul(input_fm_w, input_fm_h);
	int pool_pos = input_offset;
	int input_pos_d1 = 0;
	for (no = 0; no < conv_size.d1 + output_offset; no++) {
		int input_pos_d2 = 0;
		for (y = 0; y < pool_out_h; y++) {
			int input_pos_d3 = 0;
			for (x = 0;  x < pool_out_w; x++) {
				int cmp_begin_pos = input_pos_d1 + input_pos_d2 + input_pos_d3;

				int max = out[cmp_begin_pos];
				int cmp_len = 0;
				for (ky = 0; ky < stride; ky++) {
					for (kx = 0; kx < stride; kx++) {
						int cmp_pos = cmp_begin_pos + cmp_len + kx;
						if (out[cmp_pos] > max) max = out[cmp_pos];
					}
					cmp_len += conv_size.d3;
				}
				
				out[pool_pos] = max;
				pool_pos++;
				input_pos_d3 += stride;
			}
			for (int i = 0; i < stride; i++) input_pos_d2 += input_fm_w;
		}
		input_pos_d1 += input_area;
	}
}

#ifdef USE_HW_ACCEL
void launch_hw_accel()
{
	volatile int* gpio_start = (void*)(GPIO_START_ADDR);
	volatile int* gpio_done = (void*)(GPIO_DONE_ADDR);

	//TODO: Please add your implementation here
	*gpio_start = 0x1;
	while (!((*gpio_done) & 0x1));
}
#endif

int comparing()
{
	char *out = (char *)addr.wr_addr;
	char *result = (char *)_binary_data_result_bin_start;

#ifdef USE_HW_ACCEL
	int count = (int)_binary_data_result_bin_size + 
		    (16 - WR_SIZE_D3) * 2 * WR_SIZE_D2 * WR_SIZE_D1;
#else
	int count = (int)_binary_data_result_bin_size;
#endif

	for (int i = 0, j = 0; i < count; i++)
	{
#ifdef USE_HW_ACCEL
		int alignment = i & 0x0000001f;
		if (alignment >= (WR_SIZE_D3 << 1))
			continue;
#endif
		if (*(out + i) != *(result + j))
		{
			printf("Failed! at address %x and %x with data %x and %x\n", out + i, result + j, *(out + i), *(result + j));
			return 1;
		}
		j++;
	}

	printf("Passed!\n");
	return 0;
}

int main()
{
	Result res;
	res.msec = 0;
	bench_prepare(&res);

#ifdef USE_HW_ACCEL
	printf("Launching task...\n");
	launch_hw_accel();
#else
	printf("starting convolution\n");
	convolution();
	printf("starting pooling\n");
	pooling();
#endif
	bench_done(&res);
	printf("The program terminated in %d cycles\n", res.msec);
	printf("benchmark finished\n");

	int result = comparing();

	if (result == 0) {
		hit_good_trap();
	} else {
		nemu_assert(0);
	}

	return 0;
}

