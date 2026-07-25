import hashlib
import json


def canonical_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":")).encode()


def file_digest(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_image_digest(value):
    algorithm, separator, encoded = value.partition(":")
    if (
        algorithm != "sha256"
        or separator != ":"
        or len(encoded) != 64
        or any(character not in "0123456789abcdef" for character in encoded)
    ):
        raise RuntimeError("invalid container image digest")
    return value


def output_record_path(path, run_id, legacy=False):
    suffix = ".consumed.json" if legacy else ".output.json"
    return path.resolve().with_name(run_id + suffix)


def read_output_record(path, run_id, container_image_digest, legacy=False):
    record = output_record_path(path, run_id, legacy)
    data = json.loads(record.read_text(encoding="utf-8"))
    id_field = "release_id" if legacy else "run_id"
    if (
        data.get(id_field) != run_id
        or data.get("container_image_digest") != container_image_digest
        or not isinstance(data.get("output"), str)
        or not data["output"]
    ):
        raise RuntimeError("invalid benchmark output record")
    return record, data
