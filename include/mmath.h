#pragma once

#define attempt_faster_mul

template<class T, int rows, int cols = 1>
class matrix {

public:
	
	typedef T mem_t;
	// array<array<mem_t, cols>, rows> mem;
	mem_t mem[rows*cols];
	size_t memsize = rows*cols;

	const static int Rows = rows;
	const static int Cols = cols;
	
	// initialization?? 
	matrix() = default;

	// element access and assignment
	mem_t  operator()(int row, int col = 0) const { return(mem[(row * Cols + col)%memsize]); }
	mem_t &operator()(int row, int col = 0)       { return(mem[(row * Cols + col)%memsize]); }

	bool swap_row(int src, int dest) {
		if ( src < 0 || dest < 0 || src > Rows || dest > Rows ) { return false; } 

		mem_t tmp;
		int destCols = dest*Cols;
		int srcCols = src*Cols;
		for (int c = 0; c < Cols; c++) {
			tmp = mem[destCols+c];
			mem[destCols+c] = mem[src*Cols+c];
			mem[srcCols+c] = tmp;
		}

		return true;
	}

	// assignment
	matrix &operator=(const matrix &other) {
		static_assert(Rows == other.Rows && Cols == other.Cols, "Matrices are not of equal size!");
		memcpy(mem, other.mem, (Rows*Cols)*sizeof(mem_t));
		return *this;
	}

	// negation
	matrix operator-() {
		for ( int i = 0; i < Rows*Cols; i++ ) {
			mem[i] = -mem[i];
		}

		return *this;
	}

	// addition
	matrix &operator+=(const matrix &other) {
		static_assert(Rows == other.Rows && Cols == other.Cols, "Matrices are not of equal size!");
		int rowCols = Rows*Cols;
		for ( int i = 0; i < rowCols; i++ ) {
			mem[i] += other.mem[i];    
		}

		return *this;
	}

	const matrix operator+(const matrix &other) const {
		return matrix(*this) += other;
	}

	// subtraction
	matrix &operator-=(const matrix &other) {
		static_assert(Rows == other.Rows && Cols == other.Cols, "Matrices are not of equal size!");
		int rowCols = Rows*Cols;
		for ( int i = 0; i < rowCols; i++ ) {
			mem[i] -= other.mem[i];
		}
		return *this;
	}

	const matrix operator-(const matrix &other) const {
		return matrix(*this) -= other;
	}
	
	// multiplication
	template<int opRows = 1, int opCols = 1>
	const matrix<T, rows, opRows> operator*(const matrix<T, opCols, opRows> &other) const {
		static_assert(Cols == other.Rows, "Matrices are not of equal size!");
		matrix<mem_t, opCols, opRows> ret;

		for ( int i = 0; i < rows; i++ ) {
			for ( int j = 0; j < opCols; j++ ) {
				
				// slower?
				// mem_t sum = 0;
				// for ( int k = 0; k < Cols; k++ ) {
				//     sum += mem[i*Cols+k] * other(k, j);
				// }
				// ret(i, j) = sum;

				// precompute multiplication
				int iCols = i*Cols;

				mem_t sum = mem[iCols] * other.mem[j];
				for ( int k = 1; k < Cols; k++ ) {

					// skip multiplication if multiplying by zero
					#ifdef attempt_faster_mul
						if ( ((*(int*)&mem[iCols+k]) & 0x7fffffff) != 0 ) {
							sum += mem[iCols+k] * other(k, j);
						}
					#else
						sum += mem[iCols+k] * other(k, j);
					#endif

				}

				ret(i, j) = sum;
		
			}
		}

		return ret;
	}

	matrix &operator*=(const mem_t scalar) {
		for ( int i = 0; i < Rows*Cols; i++ ) {
			mem[i] *= scalar;
		} 
		return *this;
	}

	// transposition
	matrix operator~() {
		matrix<mem_t, Rows, Cols> ret;
		for ( int i = 0; i < Rows; i++ ) {
			int iCols = i*Cols;
			for ( int j = 0; j < Cols; j++ ) {
				ret(j, i) = mem[iCols+j];
			}
		}
		return ret;
	}

	// LU decomposition
	// https://en.wikipedia.org/wiki/LU_decomposition
	matrix invert(bool *ok, mem_t tol) {

		static_assert(Rows == Cols, "Cannot invert a non-square matrix!");

		*ok = Rows == Cols;
		if (!*ok) {
			return *this;
		}

		matrix<mem_t, Rows, Cols> A = *this;
		int N = Rows;

		int P[N+1];
		for ( int i = 0; i < N; i++ ) {
			P[i] = i;
		}

		int i, j, k, imax; 
		mem_t maxA, absA, *ptr;

		for ( i = 0; i < N; i++ ) {

			maxA = 0;
			imax = i;

			for ( k = i; k < N; k++ )  {
				if ( ( absA = abs(A(k, i)) ) > maxA ) {
					maxA = absA;
					imax = k;
				}
			}

			if ( maxA < tol ) {
				*ok = false;
				return *this;
			}

			if ( imax != i ) {
				j = P[i];
				P[i] = P[imax];
				P[imax] = j;

				A.swap_row(i, imax);

				P[N]++;
			}

			for ( j = i + 1; j < N; j++ ) {
				A(j, i) /= A(i, i);
				for ( k = i+1; k < N; k++ ) {
					A(j, k) -= A(j, i) * A(i, k);
				}
			}
		}

		*ok = true;

		matrix<float, Rows, Cols> IA;

		for ( j = 0; j < N; j++ ) {

			for ( i = 0; i < N; i++ ) {

				IA(i, j) = (P[i] == j);

				for ( k = 0; k < i; k++ ) { IA(i, j) -= A(i, k) * IA(k, j); }

			}

			for ( i = N - 1; i >= 0; i-- ) {
				
				for ( k = i + 1; k < N; k++ ) { IA(i, j) -= A(i, k) * IA(k, j); }

				IA(i, j) /= A(i, i);

			}

		}

		return IA;

	}

};