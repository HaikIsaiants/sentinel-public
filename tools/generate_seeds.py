import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "benchmarks" / "seeds"
BENCHMARK = json.loads((ROOT / "config" / "benchmark.json").read_text())
SCENARIOS = json.loads((ROOT / "config" / "scenario_strata.json").read_text())
BENCHMARK_ID = BENCHMARK["benchmark_id"]
BENCHMARK_VERSION = BENCHMARK["version"]
SEED_NAMESPACE = BENCHMARK["seed_derivation_namespace"]
if SCENARIOS["benchmark_id"] != BENCHMARK_ID or SCENARIOS["benchmark_version"] != BENCHMARK_VERSION:
    raise ValueError("benchmark configs disagree")
STRATA = tuple(item["id"] for item in SCENARIOS["strata"])
CORPORA = BENCHMARK["corpora"]
SOURCES = (
    "config/benchmark.json",
    "config/scenario_generation.json",
    "config/scenario_strata.json",
    "tools/generate_scenarios.py",
    "tools/generate_seeds.py",
)


def digest(data):
    return hashlib.sha256(data).hexdigest()


def seed_value(corpus, stratum, index):
    key = f"{SEED_NAMESPACE}|{corpus}|{stratum}|{index}".encode()
    # high bit stays clear for protobuf int64 interoperability
    return int.from_bytes(hashlib.sha256(key).digest()[:8], "big") & 0x7FFFFFFFFFFFFFFF


def records(corpus, per_stratum):
    rows = []
    for index in range(per_stratum):
        # round-robin order keeps prefixes balanced across strata
        for stratum in STRATA:
            ordinal = len(rows)
            rows.append(
                {
                    "id": f"{corpus}-{ordinal:05d}",
                    "seed": seed_value(corpus, stratum, index),
                    "stratum": stratum,
                    "stratum_index": index,
                }
            )
    return rows


def encode_rows(rows):
    return b"".join(
        (json.dumps(row, sort_keys=True, separators=(",", ":")) + "\n").encode()
        for row in rows
    )


def source_hashes():
    return {path: digest((ROOT / path).read_bytes()) for path in SOURCES}


def build_files():
    corpus_bytes = {
        f"{corpus}.jsonl": encode_rows(records(corpus, values["missions_per_stratum"]))
        for corpus, values in CORPORA.items()
    }
    manifest = {
        "benchmark_id": BENCHMARK_ID,
        "benchmark_version": BENCHMARK_VERSION,
        "corpora": {
            corpus: {
                "count": values["missions"],
                "file": f"benchmarks/seeds/{corpus}.jsonl",
                "missions_per_stratum": values["missions_per_stratum"],
                "sha256": digest(corpus_bytes[f"{corpus}.jsonl"]),
            }
            for corpus, values in CORPORA.items()
        },
        "derivation": f"uint64_be(sha256('{SEED_NAMESPACE}|<corpus>|<stratum>|<stratum_index>')[0:8]) & 0x7fffffffffffffff",
        "generator": "tools/generate_seeds.py",
        "order": "strata round-robin in manifest order",
        "source_sha256": source_hashes(),
        "strata": list(STRATA),
    }
    manifest_bytes = (json.dumps(manifest, indent=2, sort_keys=True) + "\n").encode()
    sums = {
        **corpus_bytes,
        "manifest.json": manifest_bytes,
    }
    checksum_bytes = "".join(
        f"{digest(data)}  {name}\n" for name, data in sorted(sums.items())
    ).encode()
    return {**sums, "SHA256SUMS": checksum_bytes}


def validate(files):
    manifest = json.loads(files["manifest.json"])
    seen = set()
    for corpus, values in CORPORA.items():
        per_stratum = values["missions_per_stratum"]
        rows = [json.loads(line) for line in files[f"{corpus}.jsonl"].splitlines()]
        counts = Counter(row["stratum"] for row in rows)
        expected = per_stratum * len(STRATA)
        if values["missions"] != expected or len(rows) != expected or any(counts[name] != per_stratum for name in STRATA):
            raise ValueError(f"invalid {corpus} stratum counts")
        for row in rows:
            identity = row["seed"]
            if identity in seen or not 0 <= row["seed"] <= 0x7FFFFFFFFFFFFFFF:
                raise ValueError(f"invalid {corpus} seed")
            seen.add(identity)
        if manifest["corpora"][corpus]["sha256"] != digest(files[f"{corpus}.jsonl"]):
            raise ValueError(f"invalid {corpus} hash")
    expected_sums = "".join(
        f"{digest(files[name])}  {name}\n"
        for name in sorted(("development.jsonl", "holdout.jsonl", "manifest.json"))
    ).encode()
    if files["SHA256SUMS"] != expected_sums:
        raise ValueError("invalid SHA256SUMS")


def main():
    files = build_files()
    validate(files)
    if len(sys.argv) == 2 and sys.argv[1] == "--check":
        for name, expected in files.items():
            if not (OUTPUT / name).is_file() or (OUTPUT / name).read_bytes() != expected:
                raise SystemExit(f"out of date: {name}")
        print("seed corpus valid")
        return
    if len(sys.argv) != 1:
        raise SystemExit("usage: generate_seeds.py [--check]")
    OUTPUT.mkdir(parents=True, exist_ok=True)
    for name, data in files.items():
        (OUTPUT / name).write_bytes(data)
    print(f"wrote {sum(values['missions'] for values in CORPORA.values())} deterministic seeds")


if __name__ == "__main__":
    main()
