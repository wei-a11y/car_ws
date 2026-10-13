# Copyright 2026 car_ws contributors
# SPDX-License-Identifier: Apache-2.0
"""Bounded JSONL journal. Write/flush/rotation errors intentionally propagate."""
import json
from pathlib import Path


def json_safe(value):
    # Keep JSON valid when a LaserScan min range is inf or a diagnostic is NaN.
    import math
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(item) for item in value]
    return value


class Journal:
    def __init__(self, directory, max_bytes=10485760, backups=5):
        self.path = Path(directory).resolve() / 'tasks.jsonl'
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.max_bytes, self.backups = max_bytes, backups
        # Probe even if no event is written yet; fail before accepting a task.
        with self.path.open('a', encoding='utf-8'):
            pass

    def write(self, record):
        line = json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False) + '\n'
        size = len(line.encode('utf-8'))
        if size > self.max_bytes:
            raise OSError('A journal record exceeds log_max_bytes')
        if self.path.stat().st_size + size > self.max_bytes:
            oldest = self.path.with_name(self.path.name + f'.{self.backups}')
            oldest.unlink(missing_ok=True)
            for index in range(self.backups - 1, 0, -1):
                previous = self.path.with_name(self.path.name + f'.{index}')
                if previous.exists():
                    previous.rename(self.path.with_name(self.path.name + f'.{index + 1}'))
            self.path.rename(self.path.with_name(self.path.name + '.1'))
        with self.path.open('a', encoding='utf-8') as stream:
            stream.write(line)
            stream.flush()
