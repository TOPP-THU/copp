//! C-compatible data container types.

use crate::copp::constraints::InputMatrix;
use crate::ffi::c::CoppStatus;
use crate::ffi::c::core::status::clear_last_error;
use nalgebra::{Const, DMatrix, DMatrixView, Dyn};
use std::{mem::ManuallyDrop, ptr, ptr::NonNull, slice};

/// Matrix memory layout used by [`CoppMatrixViewF64`].
///
/// COPP recommends column-major input because it matches nalgebra. Strictly
/// contiguous column-major input (`leading_dim == rows`) can be borrowed as a
/// slice, and padded column-major input (`leading_dim >= rows`) can be borrowed
/// as a strided matrix view. Row-major input is accepted, but COPP copies it
/// into column-major storage before passing it to COPP internals.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CoppMatrixLayout {
    /// Column-major storage: element `(row, col)` is at
    /// `data[row + col * leading_dim]`.
    ColumnMajor = 0,
    /// Row-major storage: element `(row, col)` is at
    /// `data[col + row * leading_dim]`.
    RowMajor = 1,
}

/// Borrowed immutable `f64` matrix passed from C to COPP.
///
/// The matrix has shape `(rows, cols)` and uses the indexing rule selected by
/// `layout`. For contiguous column-major input, use `leading_dim = rows`. For
/// contiguous row-major input, use `leading_dim = cols`.
///
/// Column-major input with `leading_dim >= rows` is the zero-copy path for
/// matrix-view consumers. Strictly contiguous column-major input with
/// `leading_dim == rows` is also accepted by APIs that require a raw
/// contiguous slice. Row-major input is copied once into column-major
/// temporary storage before COPP uses it.
///
/// The pointer must reference enough initialized `double` values for the
/// selected layout unless `rows == 0` or `cols == 0`, in which case `data` may
/// be null. Borrowed matrix inputs must remain valid and must not be mutated
/// concurrently for the duration of the call.
///
/// # Example
/// The example below declares a borrowed column-major matrix view for a
/// `dim x n` buffer.
///
/// ```c
/// enum { DIM = 3, N = 100 };
/// double column_major[DIM * N];
/// struct CoppMatrixViewF64 view =
///     COPP_MATRIX_VIEW_F64_COLUMN_MAJOR(column_major, DIM, N);
/// ```
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppMatrixViewF64 {
    /// Pointer to the first matrix element.
    pub data: *const f64,
    /// Number of matrix rows.
    pub rows: usize,
    /// Number of matrix columns.
    pub cols: usize,
    /// Matrix memory layout.
    pub layout: CoppMatrixLayout,
    /// Leading dimension / physical stride.
    ///
    /// For column-major input this is the distance between consecutive
    /// columns, and must be at least `rows` for non-empty matrices.
    /// For row-major input this is the distance between consecutive rows, and
    /// must be at least `cols` for non-empty matrices.
    pub leading_dim: usize,
}

pub(crate) enum CoppInputMatrix<'a> {
    Borrowed(InputMatrix<'a>),
    Owned(DMatrix<f64>),
}

impl CoppInputMatrix<'_> {
    pub(crate) fn as_view(&self) -> InputMatrix<'_> {
        match self {
            Self::Borrowed(view) => *view,
            Self::Owned(matrix) => matrix.as_view(),
        }
    }
}

impl CoppMatrixViewF64 {
    /// Empty column-major matrix descriptor.
    pub(crate) const fn empty() -> Self {
        Self {
            data: ptr::null(),
            rows: 0,
            cols: 0,
            layout: CoppMatrixLayout::ColumnMajor,
            leading_dim: 0,
        }
    }

    #[inline(always)]
    fn len(&self) -> Result<usize, CoppStatus> {
        self.rows
            .checked_mul(self.cols)
            .ok_or(CoppStatus::InvalidShape)
    }

    #[inline(always)]
    fn is_empty(&self) -> bool {
        self.rows == 0 || self.cols == 0
    }

    #[inline(always)]
    fn is_strict_contiguous_column_major(&self) -> bool {
        self.layout == CoppMatrixLayout::ColumnMajor
            && (self.is_empty() || self.leading_dim == self.rows)
    }

    #[inline(always)]
    fn is_borrowable_column_major(&self) -> bool {
        self.layout == CoppMatrixLayout::ColumnMajor
            && (self.is_empty() || self.leading_dim >= self.rows)
    }

    fn storage_len(&self) -> Result<usize, CoppStatus> {
        if self.is_empty() {
            return Ok(0);
        }

        match self.layout {
            CoppMatrixLayout::ColumnMajor => {
                if self.leading_dim < self.rows {
                    return Err(CoppStatus::InvalidShape);
                }
                self.leading_dim
                    .checked_mul(self.cols - 1)
                    .and_then(|offset| offset.checked_add(self.rows))
                    .ok_or(CoppStatus::InvalidShape)
            }
            CoppMatrixLayout::RowMajor => {
                if self.leading_dim < self.cols {
                    return Err(CoppStatus::InvalidShape);
                }
                self.leading_dim
                    .checked_mul(self.rows - 1)
                    .and_then(|offset| offset.checked_add(self.cols))
                    .ok_or(CoppStatus::InvalidShape)
            }
        }
    }

    /// Convert this C matrix descriptor into a raw contiguous column-major slice.
    ///
    /// This is only available for the recommended fast path:
    /// `layout = COPP_MATRIX_LAYOUT_COLUMN_MAJOR` and `leading_dim = rows`.
    /// Other valid layouts should use [`CoppMatrixViewF64::to_dmatrix`] or
    /// [`CoppMatrixViewF64::as_input_matrix`], which perform a temporary copy
    /// when needed.
    ///
    /// # Safety
    /// For non-empty matrices, `data` must point to enough initialized `f64`
    /// values for the declared layout that remain valid for the returned
    /// lifetime.
    pub(crate) unsafe fn as_slice<'a>(&self) -> Result<&'a [f64], CoppStatus> {
        if !self.is_strict_contiguous_column_major() {
            return Err(CoppStatus::InvalidShape);
        }
        let len = self.len()?;
        if len == 0 {
            return Ok(&[]);
        }
        if self.data.is_null() {
            return Err(CoppStatus::NullPointer);
        }
        // SAFETY: The checked fast-path layout requires `data` to point to at
        // least `rows * cols` contiguous f64 values for the duration of the call.
        Ok(unsafe { slice::from_raw_parts(self.data, len) })
    }

    /// Convert this C matrix descriptor into the raw storage slice implied by
    /// `layout` and `leading_dim`.
    ///
    /// # Safety
    /// For non-empty matrices, `data` must point to enough initialized `f64`
    /// values for the declared layout that remain valid for the returned
    /// lifetime.
    unsafe fn storage_slice<'a>(&self) -> Result<&'a [f64], CoppStatus> {
        let len = self.storage_len()?;
        if len == 0 {
            return Ok(&[]);
        }
        if self.data.is_null() {
            return Err(CoppStatus::NullPointer);
        }
        // SAFETY: `storage_len` validates the declared layout and computes the
        // last reachable element count for this descriptor.
        Ok(unsafe { slice::from_raw_parts(self.data, len) })
    }

    /// Convert this C matrix descriptor into an owned column-major matrix.
    ///
    /// Column-major input can usually be borrowed by callers that accept
    /// [`CoppInputMatrix`]. This function always returns owned storage.
    ///
    /// # Safety
    /// Same pointer validity contract as [`CoppMatrixViewF64::storage_slice`].
    pub(crate) unsafe fn to_dmatrix(self) -> Result<DMatrix<f64>, CoppStatus> {
        let len = self.len()?;
        if len == 0 {
            return Ok(DMatrix::zeros(self.rows, self.cols));
        }

        match self.layout {
            CoppMatrixLayout::ColumnMajor if self.is_strict_contiguous_column_major() => {
                // SAFETY: Same C matrix-view contract as this function.
                let data = unsafe { self.as_slice()? };
                Ok(DMatrix::from_column_slice(self.rows, self.cols, data))
            }
            CoppMatrixLayout::ColumnMajor => {
                // SAFETY: Same C matrix-view contract as this function.
                let data = unsafe { self.storage_slice()? };
                let mut values = Vec::with_capacity(len);
                for col in 0..self.cols {
                    let start = col * self.leading_dim;
                    values.extend_from_slice(&data[start..start + self.rows]);
                }
                Ok(DMatrix::from_vec(self.rows, self.cols, values))
            }
            CoppMatrixLayout::RowMajor if self.leading_dim == self.cols => {
                // SAFETY: Same C matrix-view contract as this function.
                let data = unsafe { self.storage_slice()? };
                Ok(DMatrix::from_row_slice(self.rows, self.cols, &data[..len]))
            }
            CoppMatrixLayout::RowMajor => {
                // SAFETY: Same C matrix-view contract as this function.
                let data = unsafe { self.storage_slice()? };
                let mut values = Vec::with_capacity(len);
                for col in 0..self.cols {
                    for row in 0..self.rows {
                        values.push(data[col + row * self.leading_dim]);
                    }
                }
                Ok(DMatrix::from_vec(self.rows, self.cols, values))
            }
        }
    }

    /// Convert this C matrix descriptor into an input matrix for constraints.
    ///
    /// Column-major input with `leading_dim >= rows` is borrowed. Row-major
    /// input is copied once into owned column-major storage.
    ///
    /// # Safety
    /// Same pointer validity contract as [`CoppMatrixViewF64::storage_slice`].
    pub(crate) unsafe fn as_input_matrix<'a>(&self) -> Result<CoppInputMatrix<'a>, CoppStatus> {
        if self.is_empty() {
            return Ok(CoppInputMatrix::Borrowed(DMatrixView::from_slice(
                &[],
                self.rows,
                self.cols,
            )));
        }

        if self.is_borrowable_column_major() {
            // SAFETY: Same C matrix-view contract as this function.
            let data = unsafe { self.storage_slice()? };
            Ok(CoppInputMatrix::Borrowed(
                DMatrixView::from_slice_with_strides_generic(
                    data,
                    Dyn(self.rows),
                    Dyn(self.cols),
                    Const::<1>,
                    Dyn(self.leading_dim),
                ),
            ))
        } else {
            // SAFETY: Same C matrix-view contract as this function.
            Ok(CoppInputMatrix::Owned(unsafe { (*self).to_dmatrix()? }))
        }
    }
}

/// Borrowed immutable `f64` slice passed from C to COPP.
///
/// The pointer must reference at least `len` contiguous `double` values unless
/// `len == 0`, in which case `data` may be null.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppSliceF64 {
    /// Pointer to the first element.
    pub data: *const f64,
    /// Number of elements.
    pub len: usize,
}

impl CoppSliceF64 {
    /// Convert this C slice descriptor into an internal slice.
    ///
    /// # Safety
    /// For non-empty slices, `data` must point to at least `len` contiguous
    /// initialized `f64` values that remain valid for the returned lifetime.
    pub(crate) unsafe fn as_slice<'a>(&self) -> Result<&'a [f64], CoppStatus> {
        if self.len == 0 {
            return Ok(&[]);
        }
        if self.data.is_null() {
            return Err(CoppStatus::NullPointer);
        }
        // SAFETY: The C ABI contract requires `data` to point to at least
        // `len` contiguous f64 values for the duration of the call.
        Ok(unsafe { slice::from_raw_parts(self.data, self.len) })
    }
}

/// Borrowed mutable `f64` slice passed from C to COPP.
///
/// The pointer must reference at least `len` contiguous initialized `double`
/// values unless `len == 0`, in which case `data` may be null. Functions that
/// accept this type may modify the pointed-to values in place.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct CoppSliceMutF64 {
    /// Pointer to the first mutable element.
    pub data: *mut f64,
    /// Number of elements.
    pub len: usize,
}

impl CoppSliceMutF64 {
    /// Convert this C slice descriptor into a mutable internal slice.
    ///
    /// # Safety
    /// For non-empty slices, `data` must point to at least `len` contiguous
    /// initialized `f64` values that remain valid and uniquely borrowed for the
    /// returned lifetime.
    pub(crate) unsafe fn as_mut_slice<'a>(&self) -> Result<&'a mut [f64], CoppStatus> {
        if self.len == 0 {
            return Ok(&mut []);
        }
        if self.data.is_null() {
            return Err(CoppStatus::NullPointer);
        }
        // SAFETY: The C ABI contract requires `data` to point to at least
        // `len` contiguous f64 values that are uniquely mutable for this call.
        Ok(unsafe { slice::from_raw_parts_mut(self.data, self.len) })
    }
}

/// Library-owned `f64` vector returned to C.
///
/// C callers may read `data[0..len]` and must release the vector exactly once
/// with `copp_vec_f64_free`.
///
/// # Example
/// The example below reads a solver-returned vector and then releases it.
///
/// ```c
/// struct CoppVecF64 a = {0};
/// check(topp2_ra(problem, options, &a));
/// for (size_t k = 0; k < a.len; ++k) {
///     printf("a[%zu] = %.17g\n", k, a.data[k]);
/// }
/// copp_vec_f64_free(a);
/// ```
#[repr(C)]
#[derive(Debug)]
pub struct CoppVecF64 {
    /// Pointer to the first element, or null for an empty vector.
    pub data: *mut f64,
    /// Number of initialized elements.
    pub len: usize,
    /// Allocation capacity, used only by `copp_vec_f64_free`.
    pub capacity: usize,
}

/// Library-owned `usize` vector returned to C.
///
/// C callers may read `data[0..len]` and must release the vector exactly once
/// with `copp_vec_usize_free`.
#[repr(C)]
#[derive(Debug)]
pub struct CoppVecUsize {
    /// Pointer to the first element, or null for an empty vector.
    pub data: *mut usize,
    /// Number of initialized elements.
    pub len: usize,
    /// Allocation capacity, used only by `copp_vec_usize_free`.
    pub capacity: usize,
}

/// Library-owned column-major `f64` matrix returned to C.
///
/// C callers may read `data[0..rows * cols]` using column-major indexing and
/// must release the matrix exactly once with `copp_matrix_f64_free`.
///
/// # Example
/// The example below reads a column-major path-evaluation matrix and releases
/// all returned matrices.
///
/// ```c
/// struct CoppMatrixF64 q = {0};
/// struct CoppMatrixF64 dq = {0};
/// struct CoppMatrixF64 ddq = {0};
/// check(copp_path_evaluate_up_to_2nd(path, s_slice, &q, &dq, &ddq));
///
/// double q_row_i_col_j = q.data[i + j * q.rows];
/// copp_matrix_f64_free(ddq);
/// copp_matrix_f64_free(dq);
/// copp_matrix_f64_free(q);
/// ```
#[repr(C)]
#[derive(Debug)]
pub struct CoppMatrixF64 {
    /// Pointer to the first column-major element, or null for an empty matrix.
    pub data: *mut f64,
    /// Number of matrix rows.
    pub rows: usize,
    /// Number of matrix columns.
    pub cols: usize,
    /// Allocation capacity in number of `double` elements, used only by
    /// `copp_matrix_f64_free`.
    pub capacity: usize,
}

impl CoppMatrixF64 {
    /// Empty matrix descriptor.
    pub const fn empty() -> Self {
        Self {
            data: ptr::null_mut(),
            rows: 0,
            cols: 0,
            capacity: 0,
        }
    }

    /// Transfer ownership of a column-major vector to C as a matrix.
    pub(crate) fn from_vec(rows: usize, cols: usize, vec: Vec<f64>) -> Result<Self, CoppStatus> {
        let len = rows.checked_mul(cols).ok_or(CoppStatus::InvalidShape)?;
        if vec.len() != len {
            return Err(CoppStatus::InvalidShape);
        }
        if len == 0 {
            return Ok(Self {
                data: ptr::null_mut(),
                rows,
                cols,
                capacity: 0,
            });
        }

        let mut vec = ManuallyDrop::new(vec);
        Ok(Self {
            data: vec.as_mut_ptr(),
            rows,
            cols,
            capacity: vec.capacity(),
        })
    }

    /// Transfer ownership of a nalgebra matrix to C.
    pub(crate) fn from_matrix(matrix: DMatrix<f64>) -> Result<Self, CoppStatus> {
        let rows = matrix.nrows();
        let cols = matrix.ncols();
        let vec: Vec<f64> = matrix.data.into();
        Self::from_vec(rows, cols, vec)
    }

    /// Return a checked non-null output pointer.
    pub(crate) fn out_ptr(out: *mut Self) -> Result<NonNull<Self>, CoppStatus> {
        NonNull::new(out).ok_or(CoppStatus::NullPointer)
    }

    /// Write an empty matrix descriptor to a checked output pointer.
    ///
    /// # Safety
    /// `out` must be valid for one `CoppMatrixF64` write.
    pub(crate) unsafe fn write_empty_to(out: NonNull<Self>) {
        // SAFETY: The caller guarantees that `out` is valid for one write.
        unsafe {
            out.as_ptr().write(Self::empty());
        }
    }

    /// Transfer a matrix to C by writing its descriptor to an output pointer.
    ///
    /// # Safety
    /// `out` must be valid for one `CoppMatrixF64` write.
    pub(crate) unsafe fn write_matrix_to(
        out: NonNull<Self>,
        matrix: DMatrix<f64>,
    ) -> Result<(), CoppStatus> {
        let matrix = Self::from_matrix(matrix)?;
        // SAFETY: The caller guarantees that `out` is valid for one write.
        unsafe {
            out.as_ptr().write(matrix);
        }
        Ok(())
    }

    /// Release a matrix previously returned by COPP.
    pub(crate) fn free(self) {
        let Some(len) = self.rows.checked_mul(self.cols) else {
            return;
        };
        if self.data.is_null() || self.capacity == 0 || len > self.capacity {
            return;
        }

        // SAFETY: `CoppMatrixF64` returned by COPP is created from a `Vec<f64>`
        // using the exact pointer, length, and capacity saved above.
        unsafe {
            drop(Vec::from_raw_parts(self.data, len, self.capacity));
        }
    }
}

impl CoppVecF64 {
    /// Empty vector descriptor.
    pub const fn empty() -> Self {
        Self {
            data: ptr::null_mut(),
            len: 0,
            capacity: 0,
        }
    }

    /// Transfer ownership of a vector to C.
    pub(crate) fn from_vec(vec: Vec<f64>) -> Self {
        if vec.is_empty() {
            return Self::empty();
        }

        let mut vec = ManuallyDrop::new(vec);
        Self {
            data: vec.as_mut_ptr(),
            len: vec.len(),
            capacity: vec.capacity(),
        }
    }

    /// Return a checked non-null output pointer.
    pub(crate) fn out_ptr(out: *mut Self) -> Result<NonNull<Self>, CoppStatus> {
        NonNull::new(out).ok_or(CoppStatus::NullPointer)
    }

    /// Write an empty vector descriptor to a checked output pointer.
    ///
    /// # Safety
    /// `out` must be valid for one `CoppVecF64` write.
    pub(crate) unsafe fn write_empty_to(out: NonNull<Self>) {
        // SAFETY: The caller guarantees that `out` is valid for one write.
        unsafe {
            out.as_ptr().write(Self::empty());
        }
    }

    /// Transfer a vector to C by writing its descriptor to an output pointer.
    ///
    /// # Safety
    /// `out` must be valid for one `CoppVecF64` write.
    pub(crate) unsafe fn write_vec_to(out: NonNull<Self>, vec: Vec<f64>) {
        // SAFETY: The caller guarantees that `out` is valid for one write.
        unsafe {
            out.as_ptr().write(Self::from_vec(vec));
        }
    }

    /// Release a vector previously returned by COPP.
    pub(crate) fn free(self) {
        if self.data.is_null() || self.capacity == 0 || self.len > self.capacity {
            return;
        }

        // SAFETY: `CoppVecF64` returned by COPP is created from a `Vec<f64>`
        // using the exact pointer, length, and capacity saved above.
        unsafe {
            drop(Vec::from_raw_parts(self.data, self.len, self.capacity));
        }
    }
}

impl CoppVecUsize {
    /// Empty vector descriptor.
    pub const fn empty() -> Self {
        Self {
            data: ptr::null_mut(),
            len: 0,
            capacity: 0,
        }
    }

    /// Release a vector previously returned by COPP.
    pub(crate) fn free(self) {
        if self.data.is_null() || self.capacity == 0 || self.len > self.capacity {
            return;
        }

        // SAFETY: `CoppVecUsize` returned by COPP is created from a
        // `Vec<usize>` using the exact pointer, length, and capacity saved
        // above.
        unsafe {
            drop(Vec::from_raw_parts(self.data, self.len, self.capacity));
        }
    }
}

/// Release a library-owned `f64` vector returned by COPP.
///
/// # Safety
/// `vec` must either be empty/null or have been returned by a COPP C ABI
/// function. Passing arbitrary pointers or modified capacity fields is invalid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_vec_f64_free(vec: CoppVecF64) {
    vec.free();
    clear_last_error();
}

/// Release a library-owned `usize` vector returned by COPP.
///
/// # Safety
/// `vec` must either be empty/null or have been returned by a COPP C ABI
/// function. Passing arbitrary pointers or modified capacity fields is invalid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_vec_usize_free(vec: CoppVecUsize) {
    vec.free();
    clear_last_error();
}

/// Release a library-owned `f64` matrix returned by COPP.
///
/// # Safety
/// `matrix` must either be empty/null or have been returned by a COPP C ABI
/// function. Passing arbitrary pointers or modified capacity fields is invalid.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn copp_matrix_f64_free(matrix: CoppMatrixF64) {
    matrix.free();
    clear_last_error();
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn padded_column_major_input_is_borrowed() {
        let data = [1.0, 2.0, -1.0, -1.0, 3.0, 4.0, -1.0, -1.0, 5.0, 6.0];
        let view = CoppMatrixViewF64 {
            data: data.as_ptr(),
            rows: 2,
            cols: 3,
            layout: CoppMatrixLayout::ColumnMajor,
            leading_dim: 4,
        };

        let input = unsafe { view.as_input_matrix().unwrap() };
        let CoppInputMatrix::Borrowed(matrix) = input else {
            panic!("padded column-major input should be borrowed");
        };
        assert_eq!(matrix[(0, 0)], 1.0);
        assert_eq!(matrix[(1, 0)], 2.0);
        assert_eq!(matrix[(0, 1)], 3.0);
        assert_eq!(matrix[(1, 2)], 6.0);
        assert!(matches!(
            unsafe { view.as_slice() },
            Err(CoppStatus::InvalidShape)
        ));
    }

    #[test]
    fn row_major_input_is_copied() {
        let data = [1.0, 2.0, 3.0, -1.0, 4.0, 5.0, 6.0, -1.0];
        let view = CoppMatrixViewF64 {
            data: data.as_ptr(),
            rows: 2,
            cols: 3,
            layout: CoppMatrixLayout::RowMajor,
            leading_dim: 4,
        };

        let input = unsafe { view.as_input_matrix().unwrap() };
        let CoppInputMatrix::Owned(matrix) = input else {
            panic!("row-major input should be copied");
        };
        assert_eq!(matrix.as_slice(), &[1.0, 4.0, 2.0, 5.0, 3.0, 6.0]);
    }

    #[test]
    fn to_dmatrix_preserves_padded_layout_values() {
        let data = [1.0, 2.0, -1.0, -1.0, 3.0, 4.0, -1.0, -1.0, 5.0, 6.0];
        let view = CoppMatrixViewF64 {
            data: data.as_ptr(),
            rows: 2,
            cols: 3,
            layout: CoppMatrixLayout::ColumnMajor,
            leading_dim: 4,
        };

        let matrix = unsafe { view.to_dmatrix().unwrap() };
        assert_eq!(matrix.as_slice(), &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn output_matrix_reuses_owned_dmatrix_storage() {
        let matrix = DMatrix::from_column_slice(2, 2, &[1.0, 2.0, 3.0, 4.0]);
        let original_ptr = matrix.as_ptr();

        let out = CoppMatrixF64::from_matrix(matrix).unwrap();
        assert_eq!(out.data as *const f64, original_ptr);
        assert_eq!(out.rows, 2);
        assert_eq!(out.cols, 2);
        assert_eq!(out.capacity, 4);
        assert_eq!(
            unsafe { slice::from_raw_parts(out.data, out.rows * out.cols) },
            &[1.0, 2.0, 3.0, 4.0]
        );
        out.free();
    }
}
