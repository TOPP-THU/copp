use pyo3::PyErr;
use std::sync::{Arc, Mutex, MutexGuard};

#[derive(Clone, Default)]
pub struct PyCallbackErrorSlot {
    inner: Arc<Mutex<Option<PyErr>>>,
}

impl PyCallbackErrorSlot {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn set_once(&self, err: PyErr) {
        let mut guard = self.guard();
        if guard.is_none() {
            *guard = Some(err);
        }
    }

    pub fn take(&self) -> Option<PyErr> {
        self.guard().take()
    }

    pub fn has_error(&self) -> bool {
        self.guard().is_some()
    }

    fn guard(&self) -> MutexGuard<'_, Option<PyErr>> {
        match self.inner.lock() {
            Ok(guard) => guard,
            Err(poisoned) => poisoned.into_inner(),
        }
    }
}
