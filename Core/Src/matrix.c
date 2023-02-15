/*
 * matrix.c
 *
 *  Created on: Feb 10, 2023
 *      Author: tima
 */

#include <math.h>
#include "matrix.h"

vector_t *vector_new(uint8_t dim)
{
    if (dim == 0)
    {
        printf("Wrong number of dimensions\n\r");
        return NULL;
    }

    vector_t *v = calloc(1, sizeof(*v));
    v->dim = dim;
    v->data = calloc(v->dim, sizeof(*v->data));
    return v;
}



matrix_t *matrix_new(uint8_t rows, uint8_t cols)
{
    if (cols == 0 || rows == 0)
    {
        printf("Invalid Parameters od Matrix\n\r");
        return NULL;
    }

    matrix_t *m = calloc(1, sizeof(*m));
    m->cols = cols;
    m->rows = rows;
    m->is_square = (cols == rows) ? 1 : 0;
    m->data = calloc(m->rows, sizeof(*m->data));

    for (uint8_t i = 0; i < m->rows; ++i)
    {
        m->data[i] = calloc(m->cols, sizeof(**m->data));
    }
    return m;
}

vector_t *vector_copy(vector_t *v)
{
    if (v == NULL)
    {
        printf("vector_copy: VECTOR IS NULL");
        return NULL;
    }
    vector_t *new = vector_new(v->dim);
    for (uint8_t i = 0; i < v->dim; i++)
    {
        new->data[i] = v->data[i];
    }
    return new;
}
vector_t *vector_eye(uint8_t dim)
{
    vector_t *new = vector_new(dim);
    for (uint8_t i = 0; i < dim; i++)
    {
        new->data[i] = 1.0;
    }
    return new;
}

matrix_t *matrix_sqr_zero(uint8_t size)
{
    matrix_t *m = matrix_new(size, size);
    return m;
}

matrix_t *matrix_eye(uint8_t size)
{
    matrix_t *m = matrix_new(size, size);
    for (uint8_t i = 0; i < m->cols; i++)
    {
        m->data[i][i] = 1.0;
    }
    return m;
}

matrix_t *matrix_copy(matrix_t *m)
{
    matrix_t *new = matrix_new(m->rows, m->cols);
    for (uint8_t i = 0; i < m->rows; i++)
    {
        for (uint8_t j = 0; j < m->cols; j++)
        {
            new->data[i][j] = m->data[i][j];
        }
    }
    return new;
}

matrix_t *matrix_from_arr(uint8_t rows, uint8_t cols, uint8_t dim, double *values)
{
    if (rows*cols != dim)
    {
        printf("matrix_from_arr: WRONG DIMENSIONS\n");
        return NULL;
    }
    vector_t *v = vector_from_arr(dim, values);
    matrix_t *new = matrix_from_vec(rows, cols, v);
    return new;
}

matrix_t *matrix_from_vec(uint8_t rows, uint8_t cols, vector_t *v)
{
    matrix_t *m = matrix_new(rows, cols);
    uint8_t idx;
    for (uint8_t i = 0; i < m->rows; i++)
    {
        for (uint8_t j = 0; j < m->cols; j++)
        {
            idx = i*m->cols + j;
            m->data[i][j] = (idx < v->dim) ? v->data[idx] : 0.0;
        }
    }
    return m;
}

uint8_t *matrix_from_matrix_ip(matrix_t *m, matrix_t *m2){
	if (m == NULL || m2 == NULL){
		printf("matrix_from_matrix_ip WRONG PARAMS: matrix do not exists \r\n");
		return 0;
	}
	if (m->cols != m2->cols || m->rows != m2->rows){
		printf("matrix_from_matrix_ip WRONG dimentions:\r\n");
		return 0;
	}

	for (uint8_t i = 0; i < m->rows; i++)
	{
	    for (uint8_t j = 0; j < m->cols; j++)
	    {

	        m->data[i][j] = m2->data[i][j];
	    }
	}
	return 1;
}

vector_t *vector_from_arr(uint8_t dim, double *values)
{
    vector_t *v = vector_new(dim);
    for (uint8_t i = 0; i < dim; i++)
    {
        v->data[i] = values[i];
    }
    return v;
}


void matrix_free(matrix_t *m)
{
    if (m == NULL)
    {
        printf("MATRIX NULL\n");
        return;
    }
    for (uint8_t i = 0; i < m->rows; ++i)
    {
        free(m->data[i]);
    }
    free(m->data);
    free(m);
}

void vector_free(vector_t *v)
{
    if (v == NULL)
    {
        printf("VECTOR NULL\n");
        return;
    }
    free(v->data);
    free(v);
}

// ******************************************
//
// HELP FUNCTIONS
//
// ******************************************

uint8_t matrix_check_row(matrix_t *m, uint8_t row)
{
    return (m->rows > row);
}

uint8_t matrix_check_col(matrix_t *m, uint8_t col)
{
    return (m->cols > col);
}

uint8_t matrix_check_dim(matrix_t *m, uint8_t row, uint8_t col)
{
    return matrix_check_col(m, col) && matrix_check_row(m, row);
}

uint8_t vector_check_dim(vector_t *v, uint8_t dim)
{
    return (v->dim > dim);
}

// ******************************************
//
// MATRIX EQUALITY
//
// ******************************************

uint8_t vector_eq_dim(vector_t *v1, vector_t *v2)
{
    return (v1->dim == v2->dim);
}

uint8_t matrix_eq_dim(matrix_t *m1, matrix_t *m2)
{
    return (m1->cols == m2->cols) && (m1->rows == m2->rows);
}

uint8_t vector_eq(vector_t *v1, vector_t *v2, double tolerance)
{
    if (!vector_eq_dim(v1, v2))
    {
        return 0;
    }
    for (uint8_t i = 0; i < v1->dim; i++)
    {
        if (fabs(v1->data[i] - v2->data[i]) > tolerance)
        {
            return 0;
        }
    }
    return 1;
}

uint8_t matrix_eq(matrix_t *m1, matrix_t *m2, double tolerance)
{
    if(!matrix_eq_dim(m1, m2))
    {
        return 0;
    }
    for (uint8_t i = 0; i < m1->rows; i++)
    {
        for (uint8_t j = 0; j < m1->cols; j++)
        {
            if (fabs(m1->data[i][j] - m2->data[i][j]) > tolerance)
            {
                return 0;
            }
        }
    }
    return 1;
}

// ******************************************
//
// MATRIX PRINTING
//
// ******************************************

void matrix_print(matrix_t *m)
{
    printf("Matrix \n");
    if (m == NULL)
    {
        printf("MATRIX NULL\n");
        return;
    }
    for (uint8_t i = 0; i < m->rows; i++)
    {
        for (uint8_t j = 0; j < m->cols; j++)
        {
            printf("%.6f ", m->data[i][j]);
        }
        printf("\n");
    }
}

void vector_print(vector_t *v)
{
    printf("Vector: \n");
    if (v == NULL)
    {
        printf("VECTOR NULL\n");
        return;
    }
    for (uint8_t i = 0; i < v->dim; i++)
    {
        printf("%.2f ", v->data[i]);
    }
}


// *******************************************
//
// Accessing and modifying matrix elements
//
// *******************************************

double vector_get_el(vector_t *v, uint8_t i)
{
    if (!vector_check_dim(v, i))
    {
        printf("vector_get_el %d dimension wrong\n", i);
        return 0.0;
    }
    return v->data[i];
}

double matrix_get_el(matrix_t *m, uint8_t i, uint8_t j)
{
    if(!matrix_check_col(m, j) && !matrix_check_row(m, i))
    {
        printf("matrix_get_el: Dimentions %d %d did not exists", i, j);
        return 0.0;
    }
    return m->data[i][j];
}

vector_t *matrix_get_col(matrix_t *m, uint8_t col)
{
    if (!matrix_check_col(m, col))
    {
        printf("COL %d did not exist \n", col);
        return NULL;
    }
    vector_t *v = vector_new(m->rows);
    for (uint8_t i = 0; i < m->rows; i++)
    {
        v->data[i] = m->data[i][col];
    }
    return v;
}
vector_t *matrix_get_row(matrix_t *m, uint8_t row)
{
    if (!matrix_check_row(m, row))
    {
        printf("ROW %d did not exist \n", row);
        return NULL;
    }
    vector_t *v = vector_new(m->cols);
    for (uint8_t i = 0; i < m->cols; i++)
    {
        v->data[i] = m->data[row][i];
    }
    return v;
}

uint8_t vector_set_el(vector_t *v, uint8_t i, double value)
{
    if (v == NULL || !vector_check_dim(v, i))
    {
        return 0;
    }
    v->data[i] = value;
    return 1;
}
uint8_t matrix_set_el(matrix_t *m, uint8_t i, uint8_t j, double value)
{
    if (m == NULL || !matrix_check_dim(m, i, j))
    {
        printf("matrix_set_el WRONG DIMENTIONS\n");
        return 0;
    }
    m->data[i][j] = value;
    return 1;
}

uint8_t matrix_set_all(matrix_t *m, double value)
{
    if (m == NULL)
    {
        printf("matrix_set_all Matrix is NULL\n");
        return 0;
    }

    for (uint8_t i = 0; i < m->rows; i++)
    {
        for (uint8_t j = 0; j < m->cols; j++)
        {
            m->data[i][j] == value;
        }
    }
    return 1;
}

uint8_t matrix_diag_set(matrix_t *m, double value)
{
    if (m == NULL || !m->is_square)
    {
        printf("matrix_diag_set: INCORRECT MATRIX\n");
        return 0;
    }
    for (uint8_t i = 0; i < m->cols; i++)
    {
        m->data[i][i] = value;
    }
    return 1;
}

uint8_t matrix_diag_set_vector(matrix_t *m, vector_t *v)
{
    if (m == NULL || v == NULL || !m->is_square )
    {
        printf("matrix_diag_set_vector: INCORRECT DATA\n");
        return 0;
    }
    if (m->cols != v->dim)
    {
        printf("matrix_diag_set_vector: WRONG DIMENSIONS");
        return 0;
    }
    for (uint8_t i = 0; i < v->dim; i++)
    {
        m->data[i][i] = v->data[i];
    }
    return 1;
}

vector_t *matrix_mult_vector(matrix_t *m, vector_t *v)
{
    if(m == NULL || v == NULL)
    {
        printf("matrix_mult_by_vector: NULL ELEMENTS\n");
        return NULL;
    }
    if(m->cols != v->dim)
    {
        printf("matrix_mult_by_vector: WRONG DIMENSIONS\n");
        return NULL;
    }

    vector_t *new = vector_new(m->rows);
    for (uint8_t i = 0; i < m->rows; i++)
    {
        double new_val = 0;
        for (uint8_t j = 0; j < m->cols; j++)
        {
            new_val += m->data[i][j]*v->data[j];
        }
        new->data[i] = new_val;
    }
}



uint8_t matrix_mult_scalar_ip(matrix_t *m, double value)
{
    if (m == NULL)
    {
        printf("matrix_mult_scalar_ip: MATRIX IS NULL\n");
        return 0;
    }
    for (uint8_t i = 0; i < m->rows; i++)
    {
        for (uint8_t j = 0; j < m->cols; j++)
        {
            m->data[i][j] *= value;
        }
    }
    return 1;
}

matrix_t *matrix_mult_scalar(matrix_t *m, double value)
{
    matrix_t *new = matrix_copy(m);
    if (!matrix_mult_scalar_ip(m, value))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}


uint8_t matrix_mult_row_ip(matrix_t *m, uint8_t row, double value)
{
    if (m == NULL || m->rows <= row)
    {
        printf("matrix_mult_row_ip: WRONG PARAMETERS\n");
        return 0;
    }
    for (uint8_t i = 0; i < m->cols; i++)
    {
        m->data[row][i] *= value;
    }
    return 1;
}

matrix_t *matrix_mult_row(matrix_t *m, uint8_t row, double value)
{
    matrix_t *new = matrix_copy(m);
    if (!succmatrix_mult_row_ip(m, row, value))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}


uint8_t matrix_mult_col_ip(matrix_t *m, uint8_t col, double value)
{
    if (m == NULL || m->cols <= col)
    {
        printf("matrix_mult_col_ip: WRONG PARAMETERS\n");
        return 0;
    }
    for (uint8_t i = 0; i < m->rows; i++)
    {
        m->data[i][col] *= value;
    }
    return 1;
}


matrix_t *matrix_mult_col(matrix_t *m, uint8_t col, double value)
{
    matrix_t *new = matrix_copy(m);
    if (!matrix_mult_col_ip(m, col, value))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}


uint8_t matrix_add_row_ip(matrix_t *m, uint8_t where, uint8_t row, double multiplier)
{
    if (m == NULL || m->rows <= where || m->rows <= row)
    {
        printf ("matrix_add_row_ip: WRONG PARAMS\n");
        return 0;
    }
    for (uint8_t i = 0; i < m->cols; i++)
    {
        m->data[where][i] += multiplier*m->data[row][i];
    }
    return 1;
}

matrix_t *matrix_add_row(matrix_t *m, uint8_t where, uint8_t row, double multiplier)
{
    matrix_t *new = matrix_copy(m);
    if (!matrix_add_row_ip(new, where, row, multiplier))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}

uint8_t vector_mult_scalar_ip(vector_t *v, double value, double add)
{
    if (v == NULL)
    {
        printf("vector_mult_scalar_ip: VECTOR IS NULL\n");
        return 0;
    }
    for (uint8_t i = 0; i < v->dim; i++)
    {
        v->data[i] = v->data[i]*value + add;
    }
    return 1;
}
vector_t *vector_mult_scalar(vector_t *v, double value, double add)
{
    vector_t *new = vector_copy(v);
    if (!vector_mult_scalar_ip(new, value, add))
    {
        vector_free(new);
        return NULL;
    }

    return new;
}

uint8_t vector_add_to_el_ip(vector_t *v, uint8_t element, double value)
{
    if (v == NULL || !vector_check_dim(v, element))
    {
        printf("vector_add_to_el_ip: WRONG PARAMETERS\n");
        return 0;
    }
    v->data[element] += value;
    return 1;
}

vector_t *vector_add_to_el(vector_t *v, uint8_t element, double value)
{
    vector_t *new = vector_copy(v);
    if (!vector_add_to_el_ip(new, element, value))
    {
        vector_free(new);
        return NULL;
    }

    return new;
}


uint8_t vector_add_vector_ip(vector_t *v, vector_t *v2)
{
	if(v->dim != v2->dim){
		printf("vector_add_vector_ip: WRONG DIMENTIONS \r\n");
		return 0;
	}
	for (uint8_t i = 0; i < v->dim; i++){
		v->data[i] += v2->data[i];
	}
	return 1;
}

uint8_t vector_sub_vector_ip(vector_t *v, vector_t *v2){
	if(v->dim != v2->dim){
		printf("vector_add_vector_ip: WRONG DIMENTIONS \r\n");
		return 0;
	}
	for (uint8_t i = 0; i < v->dim; i++){
		v->data[i] -= v2->data[i];
	}
	return 1;
}


uint8_t vector_from_vector_ip(vector_t *v, vector_t *v2){
	if(v->dim != v2->dim){
		printf("vector_add_vector_ip: WRONG DIMENTIONS \r\n");
		return 0;
	}
	for (uint8_t i = 0; i < v->dim; i++){
		v->data[i] = v2->data[i];
	}
	return 1;
}



// *******************************************
//
// Modifying the matrix structure
//
// *******************************************

matrix_t *matrix_rem_col(matrix_t *m, uint8_t col)
{
    if (m == NULL || m->cols <= col)
    {
        printf("matrix_rem_col: WRONG PARAMETERS\n");
        return NULL;
    }
    matrix_t *new = matrix_new(m->rows, m->cols-1);
    uint8_t i, j, k;
    for (i = 0; i < m->rows; i++)
    {
        for(j = 0, k = 0; j < m->cols; j++)
        {
            if (j != col)
            {
                new->data[i][k] = m->data[i][j];
                k++;
            }

        }
    }
    return new;
}

matrix_t *matrix_rem_row(matrix_t *m, uint8_t row)
{
    if (m == NULL || m->rows <= row)
    {
        printf("matrix_rem_row: WRONG PARAMETERS\n");
        return NULL;
    }
    matrix_t *new = matrix_new(m->rows - 1, m->cols);
    uint8_t i, j, k;
    for (i = 0, k = 0; i < m->rows; i++)
    {
        if (i != row)
        {
            for (j = 0; j < m->cols; j++)
            {
                new->data[k][j] = m->data[i][j];
            }
            k++;
        }
    }
}

uint8_t matrix_col_swap_ip(matrix_t *m, uint8_t col1, uint8_t col2)
{
    if (m == NULL || m->cols <= col1 || m->cols <= col2)
    {
        printf("matrix_col_swap_ip: WRONG PARAMETERS");
        return 0;
    }
    double tmp;
    for (uint8_t i = 0; i < m->rows; i++)
    {
        tmp = m->data[i][col1];
        m->data[i][col1] = m->data[i][col2];
        m->data[i][col2] = tmp;
    }
    return 1;
}

matrix_t *matrix_col_swap(matrix_t *m, uint8_t col1, uint8_t col2)
{
    matrix_t *new = matrix_copy(m);
    if (!matrix_col_swap_ip(new, col1, col2))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}

uint8_t matrix_row_swap_ip(matrix_t *m, uint8_t row1, uint8_t row2)
{
    if (m == NULL || m->rows <= row1 || m->rows <= row2)
    {
        printf("matrix_row_swap_ip: WRONG PARAMETERS\n");
        return 0;
    }
    double *tmp = m->data[row1];
    m->data[row1] = m->data[row2];
    m->data[row2] = tmp;
    return 1;
}
matrix_t *matrix_row_swap(matrix_t *m, uint8_t row1, uint8_t row2)
{
    matrix_t *new = matrix_copy(m);
    if (!matrix_row_swap_ip(new, row1, row2))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}

uint8_t matrix_add_ip(matrix_t *m1, matrix_t *m2)
{
    if(m1 == NULL || m2 == NULL || !matrix_eq_dim(m1, m2))
    {
        printf("matrix_add_ip: NOT EQUAL DIMENSIONS\n");
        return 0;
    }
    uint i, j;
    for (i = 0; i < m1->rows; i++)
    {
        for (j = 0; j < m1->cols; j++)
        {
            m1->data[i][j] += m2->data[i][j];
        }
    }
    return 1;
}

matrix_t *matrix_add(matrix_t *m1, matrix_t *m2)
{
    matrix_t *new = matrix_copy(m1);
    if (!matrix_add_ip(new, m2))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}

uint8_t matrix_sub_ip(matrix_t *m1, matrix_t *m2)
{
    if (m1 == NULL || m2 == NULL || !matrix_eq_dim(m1, m2))
    {
        printf("matrix_sub_ip: NOT EQUAL DIMENSIONS\n");
        return 0;
    }
    uint8_t i, j;
    for (i = 0; i < m1->rows; i++)
    {
        for (j = 0; j < m1->cols; j++)
        {
            m1->data[i][j] -= m2->data[i][j];
        }
        return 1;
    }
}

matrix_t *matrix_sub(matrix_t *m1, matrix_t *m2)
{
    matrix_t *new = matrix_copy(m1);
    if (!matrix_sub_ip(new, m2))
    {
        matrix_free(new);
        return NULL;
    }

    return new;
}

matrix_t *matrix_dot(matrix_t *m1, matrix_t *m2)
{
    if (m1 == NULL || m2 == NULL || m1->cols != m2->rows)
    {
        printf("matrix_dot: MATRICES CAN NOT MULTYPLIED\n");
        return NULL;
    }
    matrix_t *new = matrix_new(m1->rows, m2->cols);
    uint8_t i, j, k;
    for (i = 0; i < new->rows; i++)
    {
        for (j = 0; j < new->cols; j++)
        {
            for (k = 0; k < m1->cols; k++)
            {
                new->data[i][j] += m1->data[i][k]*m2->data[k][j];
            }
        }
    }
    return new;
}

matrix_t *matrix_transpose(matrix_t *m)
{
    if (m == NULL)
    {
        printf("matrix_transpose: MATRIX IS NULL\n");
        return NULL;
    }
    matrix_t *new = matrix_new(m->cols, m->rows);
    uint8_t i, j;
    for (i = 0; i < new->rows; i++)
    {
        for (j = 0; j < new->cols; j++)
        {
            new->data[i][j] = m->data[j][i];
        }
    }
    return new;
}
double matrix_trace(matrix_t *m)
{
    if (m == NULL || !m->is_square)
    {
        printf("matrix_trace: MATRIX IS WRONG\n");
        return 0.0;
    }
    double trace = 0.0;
    for (uint8_t i = 0; i < m->rows; i++)
    {
        trace += m->data[i][i];
    }
    return trace;
}

double vector_scalar_dot(vector_t *v1, vector_t *v2)
{
    if (v1 == NULL || v2 == NULL || v1->dim != v2->dim)
    {
        printf("vector_scalar_dot: WRONG PARAMETERS\n");
        return 0.0;
    }
    double result = 0.0;
    for (uint8_t i = 0; i < v1->dim; i++)
    {
        result += v1->data[i]*v2->data[i];
    }
    return result;
}

double vector_norm2(vector_t *v)
{
    double result = vector_scalar_dot(v, v);
    return result;
}

double vector_norm(vector_t *v)
{
    double result = sqrt(vector_norm2(v));
    return result;
}

uint8_t vector_normalize_ip(vector_t *v)
{
    double norm = vector_norm(v);
    if (norm = 0.0)
    {
        printf("vector_normalize: NORM IS ZERO\n");
        return 0;
    }
    for (uint8_t i = 0; i < v->dim; i++)
    {
        v->data[i] /= norm;
    }
    return 1;
}

vector_t *vector_normalize(vector_t *v)
{
    vector_t *new = vector_copy(v);
    if (!vector_normalize_ip(new))
    {
        vector_free(new);
        return NULL;
    }
    return new;
}

// *******************************************
//
// Matrix invertion
//
// *******************************************

matrix_t *matrix_invert(matrix_t *m1)
{
    if (m1 == NULL || !m1->is_square)
    {
        printf("matrix_invert: WRONG MATRIX\n");
        return NULL;
    }

    matrix_t *m = matrix_copy(m1);
    matrix_t *new = matrix_eye(m->rows);

    for (uint8_t i = 0; i < m->rows; ++i)
    {
        if (m->data[i][i] == 0.0)
        {
            uint8_t r;
            for (r = i + 1; r < m->rows; ++r)
            {
                if (m->data[r][i] != 0.0)
                {
                    break;
                }
            }
            if (r == m->rows)
            {
                printf("matrix_invert: MATRIX CANNOT BE INVERT\n");
                return NULL;
            }
            matrix_row_swap_ip(m, i, r);
            matrix_row_swap_ip(new, i, r);
        }
        double scalar = 1.0/m->data[i][i];
        matrix_mult_row_ip(m, i, scalar);
        matrix_mult_row_ip(new, i, scalar);

        for (uint8_t j = 0; j < m->rows; ++j)
        {
            if (i==j){
                continue;
            }
            double shear_needed = -m->data[j][i];
            matrix_add_row_ip(m, j, i, shear_needed);
            matrix_add_row_ip(new, j, i, shear_needed);
        }
    }
    matrix_free(m);
    return new;
}
