"""IMU live viewer, logger and replay."""

import os

# We only multiply 3x3 matrices; BLAS worker threads just burn CPU spinning.
os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
