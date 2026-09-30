import csv
import json

import mtdata2_decoder as vendor
import numpy as np
import pyarrow.parquet as pq
import pytest

from imuview.calibration import Calibration
from imuview.cli import main
from imuview.protocol_mtdata2 import encode_frame, encode_gpssol_payload, encode_imu_payload


def device_bytes(n=120):
    rng = np.random.default_rng(0)
    frames = []
    for i in range(n):
        acc = tuple(rng.normal(0, 0.1, 3) + [0, 0, 9.8])
        frames.append(
            encode_frame(
                encode_imu_payload(
                    acc=acc, gyro=(0.001 * i, 0, 0), euler=(0, 0, i), x2060=129.0 + i
                )
            )
        )
        if i % 15 == 14:
            frames.append(encode_frame(encode_gpssol_payload()))
    return b"".join(frames)


def write_vendor_csv(path, data, chunk=300, dt=0.027):  # 300 B take 26 ms at 115200
    """What mtdata2_decoder.py writes: rows of one chunk share (almost) the same t."""
    fr = vendor.Framer()
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(vendor.CSV_COLS)
        for k in range(0, len(data), chunk):
            t = (k // chunk + 1) * dt
            for j, p in enumerate(fr.feed(data[k : k + chunk])):
                w.writerow(
                    ["" if v is None else v for v in vendor.to_row(round(t + j * 1e-4, 4), p)]
                )


def only_session(logs):
    (s,) = [p for p in logs.iterdir() if p.is_dir()]
    return s


def columns(session, stream, *cols):
    t = pq.read_table(session / f"{stream}.parquet").to_pydict()
    return np.array([t[c] for c in cols], dtype=float).T


def test_import_csv_round_trip(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    data = device_bytes()
    write_vendor_csv(tmp_path / "still.csv", data)
    main(["import", "csv", "still.csv", "--label", "still"])
    session = only_session(tmp_path / "logs")
    assert session.name.endswith("_still")
    meta = json.loads((session / "meta.json").read_text())
    assert meta["label"] == "still" and meta["source"]["type"] == "import-csv"

    rows = list(csv.DictReader(open(tmp_path / "still.csv")))
    imu_rows = [r for r in rows if r["ax"] != ""]
    raw = columns(session, "imu", "accel_x", "accel_y", "accel_z")
    expected = np.array([[float(r[k]) for k in ("ax", "ay", "az")] for r in imu_rows])
    assert np.array_equal(raw, expected)  # bit-exact, nothing touched
    assert pq.read_table(session / "gnss_raw.parquet").num_rows == len(rows) - len(imu_rows)
    imu = pq.read_table(session / "imu.parquet").to_pydict()
    host = np.array(imu["host_time_ns"])
    assert (np.diff(host) > 0).all()  # frames of one chunk no longer share a time
    assert set(imu["time_source"]) == {"host"}
    assert json.loads(imu["extra"][0]) == {"0x2060": 129.0}


def test_import_raw_synthetic_time_and_calibration_never_touches_raw(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    data = device_bytes()
    (tmp_path / "dump.bin").write_bytes(data)
    cal = Calibration(
        accel_offset=np.array([0.1, -0.2, 0.3]),
        accel_matrix=np.diag([1.01, 0.99, 1.02]),
        gyro_bias=np.array([0.001, 0.002, 0.003]),
    )
    cal.save(tmp_path / "calib.json")
    main(["import", "raw", "dump.bin", "--label", "dump", "--duration", "2.0"])
    plain = only_session(tmp_path / "logs")
    imu = pq.read_table(plain / "imu.parquet").to_pydict()
    assert len(imu["accel_x"]) == 120
    assert set(imu["time_source"]) == {"synthetic"}
    t = (np.array(imu["host_time_ns"]) - 0) * 1e-9
    assert t[-1] == pytest.approx(2.0 * (len(data) - 60) / len(data), abs=1e-6)  # last GpsSol
    assert imu["accel_cal_x"][0] is None

    # the same session again, now with a calibration: raw columns are identical
    main(["reparse", str(plain), "--calib", "calib.json", "--out", "cal"])
    cal_session = tmp_path / "cal"
    a_plain = columns(plain, "imu", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y", "gyro_z")
    a_cal = columns(cal_session, "imu", "accel_x", "accel_y", "accel_z", "gyro_x", "gyro_y",
                    "gyro_z")  # fmt: skip
    assert np.array_equal(a_plain, a_cal)
    got = columns(cal_session, "imu", "accel_cal_x", "accel_cal_y", "accel_cal_z")
    want, _ = cal.apply_arrays(a_plain[:, :3], a_plain[:, 3:])
    assert np.allclose(got, want)
    meta = json.loads((cal_session / "meta.json").read_text())
    assert meta["calibration"]["sha256"] == cal.sha256
    assert meta["reparsed_from"] == str(plain)
    assert (cal_session / "raw.bin").stat().st_size > len(data)  # bytes kept (+ record headers)


def test_reparse_of_a_csv_session_uses_its_messages(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    write_vendor_csv(tmp_path / "s.csv", device_bytes(40))
    main(["import", "csv", "s.csv", "--label", "s"])
    first = only_session(tmp_path / "logs")
    main(["reparse", str(first), "--out", "again"])
    a = columns(first, "imu", "accel_x", "accel_y", "accel_z", "host_time_ns")
    b = columns(tmp_path / "again", "imu", "accel_x", "accel_y", "accel_z", "host_time_ns")
    assert np.array_equal(a, b)
