import argparse
import hashlib
import json
import pathlib

from benchmark_io import canonical_bytes, file_digest, output_record_path, read_output_record, validate_image_digest


SKIPPED = {
    ".git",
    ".pytest_cache",
    "Testing",
    "__pycache__",
    "build",
    "out",
    "vcpkg_installed",
}


def skipped(path, root, excluded):
    relative = path.relative_to(root)
    return path in excluded or any(
        part in SKIPPED or part.startswith(("build-", "cmake-build-"))
        for part in relative.parts
    )


def source_files(root, excluded=()):
    root = root.resolve()
    excluded = {path.resolve() for path in excluded}
    files = []
    for path in sorted(root.rglob("*")):
        if not path.is_file() or skipped(path.resolve(), root, excluded):
            continue
        relative = path.relative_to(root).as_posix()
        files.append({"path": relative, "sha256": file_digest(path), "size": path.stat().st_size})
    return files


def tree_digest(files):
    return hashlib.sha256(canonical_bytes(files)).hexdigest()


def executable(path):
    path = path.resolve()
    if not path.is_file():
        raise RuntimeError(f"missing executable: {path}")
    return {"name": path.name, "sha256": file_digest(path), "size": path.stat().st_size}


def create_manifest(root, simulator, agent, output=None, container_image_digest="", supervisor=None):
    root = root.resolve()
    excluded = [output.resolve()] if output else []
    # Skip transient build and run output when hashing the tree.
    files = source_files(root, excluded)
    benchmark_path = root / "config" / "benchmark.json"
    benchmark = json.loads(benchmark_path.read_text(encoding="utf-8"))
    inputs = {
        item["path"]: item["sha256"]
        for item in files
        if item["path"].startswith(("benchmarks/seeds/", "config/", "scenarios/"))
        or item["path"] in {"tools/generate_scenarios.py", "tools/generate_seeds.py"}
    }
    payload = {
        "benchmark_id": benchmark["benchmark_id"],
        "benchmark_inputs": inputs,
        "container_image_digest": validate_image_digest(container_image_digest) if container_image_digest else "",
        "benchmark_version": benchmark["version"],
        "executables": {
            "agent": executable(agent),
            "simulator": executable(simulator),
        },
        "schema_version": 1,
        "source_files": files,
        "source_digest": tree_digest(files),
    }
    if supervisor:
        payload["executables"]["supervisor"] = executable(supervisor)
    return {**payload, "run_id": hashlib.sha256(canonical_bytes(payload)).hexdigest()}


def write_manifest(path, manifest):
    if path.exists():
        raise RuntimeError(f"benchmark manifest already exists: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def verify_manifest(path, root, simulator, agent, container_image_digest="", supervisor=None):
    stored = json.loads(path.read_text(encoding="utf-8"))
    if container_image_digest and stored.get("container_image_digest") != validate_image_digest(container_image_digest):
        raise RuntimeError("benchmark manifest container image digest does not match")
    current = create_manifest(root, simulator, agent, path, stored.get("container_image_digest", ""), supervisor)
    if stored != current:
        raise RuntimeError("benchmark manifest does not match the source tree and executables")
    return stored


def record_output(path, manifest, output):
    record = output_record_path(path, manifest["run_id"])
    record_data = {
        "container_image_digest": manifest["container_image_digest"],
        "output": str(output.resolve()),
        "run_id": manifest["run_id"],
        "schema_version": 1,
    }
    try:
        with record.open("x", encoding="utf-8", newline="\n") as stream:
            stream.write(json.dumps(record_data, indent=2, sort_keys=True) + "\n")
    except FileExistsError:
        raise RuntimeError("benchmark manifest already has an output")
    return record


def verify_output(path, run_id, container_image_digest):
    return read_output_record(path, run_id, container_image_digest)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=pathlib.Path, default=pathlib.Path(__file__).resolve().parents[1])
    parser.add_argument("--sim", type=pathlib.Path, required=True)
    parser.add_argument("--agent", type=pathlib.Path, required=True)
    parser.add_argument("--supervisor", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--container-image-digest", required=True)
    options = parser.parse_args()
    manifest = create_manifest(
        options.root,
        options.sim,
        options.agent,
        options.output,
        options.container_image_digest,
        options.supervisor,
    )
    write_manifest(options.output, manifest)
    print(json.dumps({"run_id": manifest["run_id"], "source_digest": manifest["source_digest"]}, sort_keys=True))


if __name__ == "__main__":
    main()
