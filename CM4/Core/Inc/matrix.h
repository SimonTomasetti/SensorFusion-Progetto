/*
 * matrix.h
 *
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_

#include <stdint.h>
#include "compiler.h"

/*!
* \def EXTERN_INLINE_MATRIX Helper inline to switch from local inline to extern inline
*/
#ifndef EXTERN_INLINE_MATRIX
#define EXTERN_INLINE_MATRIX EXTERN_INLINE
#endif

/**
* Matrix data type definition.
*/
typedef float matrix_data_t;

/**
* \brief Matrix definition
*/
typedef struct {
    /**
    * \brief Number of rows
    */
    uint_fast8_t rows;

    /**
    * \brief Number of columns
    */
    uint_fast8_t cols;

    /**
    * \brief Pointer to the data array of size {\see rows} x {\see cols}.
    */
    matrix_data_t *data;
} matrix_t;

/**
* \brief Initializes a matrix structure.
* \param[in] mat The matrix to initialize
* \param[in] rows The number of rows
* \param[in] cols The number of columns
* \param[in] buffer The data buffer (of size {\see rows} x {\see cols}).
*/
void matrix_init(matrix_t *const  mat, const uint_fast8_t rows, const uint_fast8_t cols, matrix_data_t *const buffer);

/**
* \brief Inverts a lower triangular matrix.
* \param[in] lower The lower triangular matrix to be inverted.
* \param[in] inverse The calculated inverse of the lower triangular matrix.
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
void matrix_invert_lower(const matrix_t *RESTRICT const lower, matrix_t *RESTRICT inverse) HOT;

/*!
* \brief Performs a matrix multiplication such that {\ref c} = {\ref x} * {\ref b}
* \param[in] a Matrix A
* \param[in] x Vector x
* \param[in] c Resulting vector C (will be overwritten)
* \param[in] aux Auxiliary vector that can hold a column of {\ref b}
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
void matrix_mult_rowvector(const matrix_t *RESTRICT const a, const matrix_t *RESTRICT const x, matrix_t *RESTRICT const c) HOT;

/*!
* \brief Performs a matrix multiplication such that {\ref c} = {\ref c} + {\ref x} * {\ref b}
* \param[in] a Matrix A
* \param[in] x Vector x
* \param[in] c Resulting vector C (will be added to)
* \param[in] aux Auxiliary vector that can hold a column of {\ref b}
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
void matrix_multadd_rowvector(const matrix_t *RESTRICT const a, const matrix_t *RESTRICT const x, matrix_t *RESTRICT const c) HOT;

/*!
* \brief Performs a matrix multiplication such that {\ref c} = {\ref a} * {\ref b}
* \param[in] a Matrix A
* \param[in] b Matrix B
* \param[in] c Resulting matrix C (will be overwritten)
* \param[in] aux Auxiliary vector that can hold a column of {\ref b}
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
void matrix_mult(const matrix_t *const a, const matrix_t *const b, const matrix_t *RESTRICT c, matrix_data_t *const baux) HOT;

/*!
* \brief Performs a matrix multiplication with transposed B such that {\ref c} = {\ref a} * {\ref b'}
* \param[in] a Matrix A
* \param[in] b Matrix B
* \param[in] c Resulting matrix C (will be overwritten)
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
void matrix_mult_transb(const matrix_t *const a, const matrix_t *const b, const matrix_t *RESTRICT c) HOT;

/*!
* \brief Performs a matrix multiplication with transposed B and adds the result to {\ref c} such that {\ref c} = {\ref c} + {\ref a} * {\ref b'}
* \param[in] a Matrix A
* \param[in] b Matrix B
* \param[in] c Resulting matrix C (will be added to)
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
void matrix_multadd_transb(const matrix_t *const a, const matrix_t *const b, const matrix_t *RESTRICT c) HOT;

/*!
* \brief Performs a matrix multiplication with transposed B and scales the result such that {\ref c} = {\ref a} * {\ref b'} * {\ref scale}
* \param[in] a Matrix A
* \param[in] b Matrix B
* \param[in] scale Scaling factor
* \param[in] c Resulting matrix C(will be overwritten)
*
* Kudos: https://code.google.com/p/efficient-java-matrix-library
*/
void matrix_multscale_transb(const matrix_t *const a, const matrix_t *const b, register const matrix_data_t scale, const matrix_t *RESTRICT c) HOT;

/*!
* \brief Gets a matrix element
* \param[in] mat The matrix to get from
* \param[in] rows The row
* \param[in] cols The column
* \return The value at the given cell.
*/
void matrix_copy(const matrix_t *src, matrix_t *dst);
void matrix_sub_inplace_b(const matrix_t *a, matrix_t *b);
void matrix_add_inplace(matrix_t *a, const matrix_t *b);
void matrix_sub(matrix_t *a, const matrix_t *b, matrix_t *c);
void matrix_get_column_copy(const matrix_t *mat, uint_fast8_t col, matrix_data_t *out_col);


#undef EXTERN_INLINE_MATRIX

#endif /* INC_MATRIX_H_ */
