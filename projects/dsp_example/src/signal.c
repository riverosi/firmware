/*
 * signal.c
 *
 *  Created on: 8 sept. 2021
 *      Author: river
 */
#include "arm_math.h"
#include "signal.h"

float32_t testInput_f32[128] = {
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		-0.00002524f,
		-0.00000933f,
		-0.00006341f,
		0.00000814f,
		-0.00011037f,
		0.00007280f,
		-0.00020991f,
		0.00027961f,
		-0.00074136f,
		0.00478519f,
		0.03727223f,
		0.06945804f,
		0.09450610f,
		0.10968079f,
		0.11505409f,
		0.11302166f,
		0.10476543f,
		0.09205794f,
		0.07661060f,
		0.05920377f,
		0.04014657f,
		0.02111493f,
		0.00305703f,
		-0.01205063f,
		-0.02502598f,
		-0.03493538f,
		-0.04119025f,
		-0.04726180f,
		-0.05178793f,
		-0.05486362f,
		-0.05479046f,
		-0.05064977f,
		-0.04384392f,
		-0.03362475f,
		-0.02164624f,
		-0.01344652f,
		-0.00897800f,
		-0.00408731f,
		0.00087062f,
		0.00342401f,
		0.00325894f,
		0.00203733f,
		-0.00064854f,
		-0.00148719f,
		0.00507034f,
		0.01776569f,
		0.02994995f,
		0.03690913f,
		0.03898356f,
		0.03778106f,
		0.03493966f,
		0.03126647f,
		0.02713533f,
		0.02267505f,
		0.01869983f,
		0.01451366f,
		0.00989223f,
		0.00441770f,
		-0.00172453f,
		-0.00767961f,
		-0.01291467f,
		-0.01644143f,
		-0.01791746f,
		-0.01900579f,
		-0.02082183f,
		-0.02306468f,
		-0.02420712f,
		-0.02582171f,
		-0.02687400f,
		-0.02938119f,
		-0.03047896f,
		-0.03247667f,
		-0.03303312f,
		-0.03602198f,
		-0.03546350f,
		-0.04143606f,
		-0.02680322f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f,
		0.00000000f};


