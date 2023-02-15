/*
 * matrix.h
 *
 *  Created on: Feb 10, 2023
 *      Author: tima
 */

#ifndef INC_MATRIX_H_
#define INC_MATRIX_H_
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>


typedef struct {
    uint8_t dim;
    double *data;
} vector_t;


/*
Структура матрицы
cols - количество столбцов
rows - количество строк
data - двумерный массив
*/
typedef struct {
    uint8_t cols;
    uint8_t rows;
    double **data;
    uint8_t is_square;
} matrix_t;


// ******************************************
//
// CONSTRUCTING AND DESTROYING A MATRIX STRUCT
//
// ******************************************

vector_t *vector_new(uint8_t dim);
matrix_t *matrix_new(uint8_t rows, uint8_t cols);

vector_t *vector_copy(vector_t *v);
vector_t *vector_eye(uint8_t dim);
matrix_t *matrix_sqr_zero(uint8_t size);
matrix_t *matrix_eye(uint8_t size);
matrix_t *matrix_copy(matrix_t *m);
matrix_t *matrix_from_arr(uint8_t rows, uint8_t cols, uint8_t dim, double *values);
matrix_t *matrix_from_vec(uint8_t rows, uint8_t cols, vector_t *v);
uint8_t *matrix_from_matrix_ip(matrix_t *m, matrix_t *m2);
vector_t *vector_from_arr(uint8_t dim, double *values);
void matrix_free(matrix_t *m);
void vector_free(vector_t *v);

// ******************************************
//
// HELP FUNCTIONS
//
// ******************************************

uint8_t matrix_check_row(matrix_t *m, uint8_t row);
uint8_t matrix_check_col(matrix_t *m, uint8_t col);
uint8_t matrix_check_dim(matrix_t *m, uint8_t row, uint8_t col);
uint8_t vector_check_dim(vector_t *v, uint8_t dim);

// ******************************************
//
// MATRIX EQUALITY
//
// ******************************************
uint8_t vector_eq_dim(vector_t *v1, vector_t *v2);
uint8_t matrix_eq_dim(matrix_t *m1, matrix_t *m2);
uint8_t vector_eq(vector_t *v1, vector_t *v2, double tolerance);
uint8_t matrix_eq(matrix_t *m1, matrix_t *m2, double tolerance);


// ******************************************
//
// MATRIX PRINTING
//
// ******************************************
void matrix_print(matrix_t *m);
void vector_print(vector_t *v);

// *******************************************
//
// Accessing and modifying matrix elements
//
// *******************************************

double vector_get_el(vector_t *v, uint8_t i);
double matrix_get_el(matrix_t *m, uint8_t i, uint8_t j);
vector_t *matrix_get_col(matrix_t *m, uint8_t col);
vector_t *matrix_get_row(matrix_t *m, uint8_t row);
uint8_t vector_set_el(vector_t *v, uint8_t i, double value);
uint8_t matrix_set_el(matrix_t *m, uint8_t i, uint8_t j, double value);
uint8_t matrix_set_all(matrix_t *m, double value);
uint8_t matrix_diag_set(matrix_t *m, double value);
uint8_t matrix_diag_set_vector(matrix_t *m, vector_t *v);
vector_t *matrix_mult_vector(matrix_t *m, vector_t *v);

uint8_t matrix_mult_scalar_ip(matrix_t *m, double value);
matrix_t *matrix_mult_scalar(matrix_t *m, double value);

uint8_t matrix_mult_row_ip(matrix_t *m, uint8_t row, double value);
matrix_t *matrix_mult_row(matrix_t *m, uint8_t row, double value);

uint8_t matrix_mult_col_ip(matrix_t *m, uint8_t col, double value);
matrix_t *matrix_mult_col(matrix_t *m, uint8_t col, double value);

uint8_t matrix_add_row_ip(matrix_t *m, uint8_t where, uint8_t row, double multiplier);
matrix_t *matrix_add_row(matrix_t *m, uint8_t where, uint8_t row, double multiplier);

uint8_t vector_mult_scalar_ip(vector_t *v, double value, double add);
vector_t *vector_mult_scalar(vector_t *v, double value, double add);

uint8_t vector_add_to_el_ip(vector_t *v, uint8_t element, double value);
vector_t *vector_add_to_el(vector_t *v, uint8_t element, double value);

uint8_t vector_add_vector_ip(vector_t *v, vector_t *v2);
uint8_t vector_sub_vector_ip(vector_t *v, vector_t *v2);
uint8_t vector_from_vector_ip(vector_t *v, vector_t *v2);


// *******************************************
//
// Modifying the matrix structure
//
// *******************************************

matrix_t *matrix_rem_col(matrix_t *m, uint8_t col);
matrix_t *matrix_rem_row(matrix_t *m, uint8_t row);

uint8_t matrix_col_swap_ip(matrix_t *m, uint8_t col1, uint8_t col2);
matrix_t *matrix_col_swap(matrix_t *m, uint8_t col1, uint8_t col2);

uint8_t matrix_row_swap_ip(matrix_t *m, uint8_t row1, uint8_t row2);
matrix_t *matrix_row_swap(matrix_t *m, uint8_t row1, uint8_t row2);

uint8_t matrix_add_ip(matrix_t *m1, matrix_t *m2);
matrix_t *matrix_add(matrix_t *m1, matrix_t *m2);

uint8_t matrix_sub_ip(matrix_t *m1, matrix_t *m2);
matrix_t *matrix_sub(matrix_t *m1, matrix_t *m2);

matrix_t *matrix_dot(matrix_t *m1, matrix_t *m2);
matrix_t *matrix_transpose(matrix_t *m);
double matrix_trace(matrix_t *m);

double vector_scalar_dot(vector_t *v1, vector_t *v2);
double vector_norm2(vector_t *v);
double vector_norm(vector_t *v);
uint8_t vector_normalize_ip(vector_t *v);
vector_t *vector_normalize(vector_t *v);


// *******************************************
//
// Matrix invertion
//
// *******************************************

matrix_t *matrix_invert(matrix_t *m1);


#endif /* INC_MATRIX_H_ */
