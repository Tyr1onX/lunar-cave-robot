#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ARM_MATH_SUCCESS        =  0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR   = -2,
    ARM_MATH_SIZE_MISMATCH  = -3,
    ARM_MATH_NANINF         = -4,
    ARM_MATH_SINGULAR       = -5,
    ARM_MATH_TEST_FAILURE   = -6
} arm_status;

typedef struct
{
    uint16_t numRows;
    uint16_t numCols;
    float *pData;
} arm_matrix_instance_f32;

static inline void arm_mat_init_f32(
    arm_matrix_instance_f32 *S,
    uint16_t nRows,
    uint16_t nColumns,
    float *pData)
{
    S->numRows = nRows;
    S->numCols = nColumns;
    S->pData = pData;
}

static inline arm_status arm_mat_add_f32(
    const arm_matrix_instance_f32 *A,
    const arm_matrix_instance_f32 *B,
    arm_matrix_instance_f32 *C)
{
    if (A->numRows != B->numRows || A->numCols != B->numCols ||
        C->numRows != A->numRows || C->numCols != A->numCols)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    const uint32_t size = (uint32_t)A->numRows * A->numCols;
    for (uint32_t i = 0; i < size; ++i)
    {
        C->pData[i] = A->pData[i] + B->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_sub_f32(
    const arm_matrix_instance_f32 *A,
    const arm_matrix_instance_f32 *B,
    arm_matrix_instance_f32 *C)
{
    if (A->numRows != B->numRows || A->numCols != B->numCols ||
        C->numRows != A->numRows || C->numCols != A->numCols)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    const uint32_t size = (uint32_t)A->numRows * A->numCols;
    for (uint32_t i = 0; i < size; ++i)
    {
        C->pData[i] = A->pData[i] - B->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_scale_f32(
    const arm_matrix_instance_f32 *A,
    float scale,
    arm_matrix_instance_f32 *C)
{
    if (C->numRows != A->numRows || C->numCols != A->numCols)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    const uint32_t size = (uint32_t)A->numRows * A->numCols;
    for (uint32_t i = 0; i < size; ++i)
    {
        C->pData[i] = A->pData[i] * scale;
    }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_mult_f32(
    const arm_matrix_instance_f32 *A,
    const arm_matrix_instance_f32 *B,
    arm_matrix_instance_f32 *C)
{
    if (A->numCols != B->numRows ||
        C->numRows != A->numRows ||
        C->numCols != B->numCols)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (uint16_t row = 0; row < A->numRows; ++row)
    {
        for (uint16_t col = 0; col < B->numCols; ++col)
        {
            float sum = 0.0f;
            for (uint16_t k = 0; k < A->numCols; ++k)
            {
                sum += A->pData[row * A->numCols + k] * B->pData[k * B->numCols + col];
            }
            C->pData[row * C->numCols + col] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_mat_trans_f32(
    const arm_matrix_instance_f32 *A,
    arm_matrix_instance_f32 *C)
{
    if (C->numRows != A->numCols || C->numCols != A->numRows)
    {
        return ARM_MATH_SIZE_MISMATCH;
    }

    for (uint16_t row = 0; row < A->numRows; ++row)
    {
        for (uint16_t col = 0; col < A->numCols; ++col)
        {
            C->pData[col * C->numCols + row] = A->pData[row * A->numCols + col];
        }
    }
    return ARM_MATH_SUCCESS;
}

static inline arm_status arm_dot_prod_f32(
    const float *pSrcA,
    const float *pSrcB,
    uint32_t blockSize,
    float *result)
{
    float sum = 0.0f;
    for (uint32_t i = 0; i < blockSize; ++i)
    {
        sum += pSrcA[i] * pSrcB[i];
    }
    *result = sum;
    return ARM_MATH_SUCCESS;
}

#ifdef __cplusplus
}
#endif
