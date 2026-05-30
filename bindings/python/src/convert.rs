use nalgebra::DMatrix;
use ndarray::Array2;
use numpy::{PyArray1, PyArray2, PyReadonlyArray1, PyReadonlyArray2, PyUntypedArrayMethods};
use pyo3::prelude::*;

/// numpy 2-D array → nalgebra DMatrix<f64>
pub fn ndarray2_to_dmatrix(arr: PyReadonlyArray2<'_, f64>) -> DMatrix<f64> {
    let shape = arr.shape();
    let (nrows, ncols) = (shape[0], shape[1]);
    let nd = arr.as_array();
    let mut data = Vec::with_capacity(nrows * ncols);
    for i in 0..nrows {
        for j in 0..ncols {
            data.push(nd[[i, j]]);
        }
    }
    DMatrix::from_row_slice(nrows, ncols, &data)
}

/// nalgebra DMatrix<f64> → numpy 2-D array
pub fn dmatrix_to_ndarray2<'py>(py: Python<'py>, mat: &DMatrix<f64>) -> Bound<'py, PyArray2<f64>> {
    let (nrows, ncols) = mat.shape();
    let mut nd = Array2::<f64>::zeros((nrows, ncols));
    for i in 0..nrows {
        for j in 0..ncols {
            nd[[i, j]] = mat[(i, j)];
        }
    }
    PyArray2::from_owned_array(py, nd)
}

/// 1-D numpy array → Vec<f64>
pub fn ndarray1_to_vec(arr: PyReadonlyArray1<'_, f64>) -> Vec<f64> {
    arr.as_array().iter().copied().collect()
}

/// Vec<f64> → 1-D numpy array
pub fn vec_to_ndarray1<'py>(py: Python<'py>, v: &[f64]) -> Bound<'py, PyArray1<f64>> {
    PyArray1::from_slice(py, v)
}
