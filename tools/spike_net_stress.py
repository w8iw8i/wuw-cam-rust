#!/usr/bin/env python3
"""Spike C stress test: N parallel keep-alive /jpg loops for D seconds.

Each thread keeps one HTTP/1.1 connection open and reuses it (reconnecting
after any error), so the server's keep-alive path is exercised. Every body is
compared byte for byte with the expected JPEG, and the Content-Type and
Cache-Control headers are checked.

/stats is read once before (after one warm-up /jpg) and once after the run.
PASS needs at least one good frame, no errors, and heap_free not to drop by
more than HEAP_TOLERANCE bytes.

    python3 tools/spike_net_stress.py --host 192.168.4.1 --conns 6 --secs 600
"""
import argparse
import http.client
import json
import os
import threading
import time

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_EXPECT = os.path.join(
    REPO_ROOT, "rust", "spikes", "spike-net", "src", "test.jpg"
)
HEAP_TOLERANCE = 1024
TIMEOUT = 5
MAX_PRINTED_ERRORS = 3


def fetch(conn, path):
    """GET path on an open connection; return (status, headers, body)."""
    conn.request("GET", path)
    resp = conn.getresponse()
    body = resp.read()
    return resp.status, resp, body


def check_jpg(status, resp, body, expected):
    """Return None if the /jpg response is correct, else a reason string."""
    if status != 200:
        return "status %d" % status
    content_type = resp.getheader("Content-Type")
    if content_type != "image/jpeg":
        return "Content-Type %r" % content_type
    cache_control = resp.getheader("Cache-Control")
    if cache_control != "no-store":
        return "Cache-Control %r" % cache_control
    if body != expected:
        return "body mismatch (%d bytes, expected %d)" % (
            len(body),
            len(expected),
        )
    return None


def get_stats(host, port):
    conn = http.client.HTTPConnection(host, port, timeout=TIMEOUT)
    try:
        status, _, body = fetch(conn, "/stats")
    finally:
        conn.close()
    if status != 200:
        raise RuntimeError("/stats returned status %d" % status)
    return json.loads(body.decode())


def warm_up(host, port, expected):
    conn = http.client.HTTPConnection(host, port, timeout=TIMEOUT)
    try:
        status, resp, body = fetch(conn, "/jpg")
    finally:
        conn.close()
    reason = check_jpg(status, resp, body, expected)
    if reason is not None:
        raise RuntimeError("warm-up /jpg failed: " + reason)


def worker(host, port, expected, stop, results, index):
    ok = 0
    err = 0
    printed = 0
    conn = None
    while not stop.is_set():
        try:
            if conn is None:
                conn = http.client.HTTPConnection(host, port, timeout=TIMEOUT)
            status, resp, body = fetch(conn, "/jpg")
            reason = check_jpg(status, resp, body, expected)
            if reason is not None:
                raise ValueError(reason)
            ok += 1
            if resp.will_close:
                conn.close()
                conn = None
        except Exception as exc:
            err += 1
            if printed < MAX_PRINTED_ERRORS:
                printed += 1
                print("thread %d error: %r" % (index, exc), flush=True)
            if conn is not None:
                conn.close()
                conn = None
    if conn is not None:
        conn.close()
    results[index] = (ok, err)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--host", default="192.168.4.1")
    ap.add_argument("--port", type=int, default=80)
    ap.add_argument("--conns", type=int, default=6)
    ap.add_argument("--secs", type=float, default=600)
    ap.add_argument(
        "--expect",
        default=DEFAULT_EXPECT,
        help="file the /jpg body must equal (default: %(default)s)",
    )
    args = ap.parse_args()

    with open(args.expect, "rb") as f:
        expected = f.read()

    warm_up(args.host, args.port, expected)
    before = get_stats(args.host, args.port)

    stop = threading.Event()
    results = {}
    threads = [
        threading.Thread(
            target=worker,
            args=(args.host, args.port, expected, stop, results, i),
        )
        for i in range(args.conns)
    ]
    t0 = time.time()
    for t in threads:
        t.start()
    time.sleep(args.secs)
    stop.set()
    for t in threads:
        t.join()
    elapsed = time.time() - t0

    after = get_stats(args.host, args.port)

    ok = sum(r[0] for r in results.values())
    err = sum(r[1] for r in results.values())
    heap_drop = before["heap_free"] - after["heap_free"]
    heap_ok = heap_drop <= HEAP_TOLERANCE

    print("frames ok %d, errors %d, %.1f req/s" % (ok, err, ok / elapsed))
    print("stats before:", json.dumps(before))
    print("stats after: ", json.dumps(after))
    print(
        "heap_free delta: %+d bytes (fail if it drops by more than %d)"
        % (after["heap_free"] - before["heap_free"], HEAP_TOLERANCE)
    )
    passed = ok > 0 and err == 0 and heap_ok
    print("PASS" if passed else "FAIL")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
