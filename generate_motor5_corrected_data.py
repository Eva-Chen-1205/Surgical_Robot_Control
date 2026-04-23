from __future__ import annotations

import csv
from pathlib import Path


SOURCE_CSV = Path(r"C:\Users\horse\Downloads\encoder_log_2026-04-22T14-44-27-386Z.csv")
OUTPUT_CSV = Path(r"D:\Codex\Control\Yuhan_motor\motor5_corrected_experiment.csv")

# Current firmware uses /100 scaling on motor 5 encoder count.
MOTOR5_SCALE_FACTOR = 100.0
MOTOR5_COUNTS_PER_DEGREE_RAW = 15.5307
MOTOR1_TO_4_COUNTS_PER_DEGREE = 1553.0667


def to_float(value: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError):
        return 0.0


def main() -> None:
    with SOURCE_CSV.open("r", newline="", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
        if reader.fieldnames is None:
            raise RuntimeError("Source CSV has no header.")
        fieldnames = list(reader.fieldnames)

    extra_fields = [
        "motor5_target_raw_count",
        "motor5_encoder_raw_count",
        "motor5_target_corrected_x100_count",
        "motor5_encoder_corrected_x100_count",
        "motor5_target_raw_degree",
        "motor5_encoder_raw_degree",
        "motor5_target_corrected_degree_check",
        "motor5_encoder_corrected_degree_check",
        "motor5_scale_factor_used",
    ]

    output_fields = fieldnames + extra_fields

    with OUTPUT_CSV.open("w", newline="", encoding="utf-8-sig") as f:
        writer = csv.DictWriter(f, fieldnames=output_fields)
        writer.writeheader()

        for row in rows:
            target_raw = to_float(row.get("target_4", "0"))
            encoder_raw = to_float(row.get("encoder_4", "0"))

            target_corrected = target_raw * MOTOR5_SCALE_FACTOR
            encoder_corrected = encoder_raw * MOTOR5_SCALE_FACTOR

            row["motor5_target_raw_count"] = f"{target_raw:.6f}"
            row["motor5_encoder_raw_count"] = f"{encoder_raw:.6f}"
            row["motor5_target_corrected_x100_count"] = f"{target_corrected:.6f}"
            row["motor5_encoder_corrected_x100_count"] = f"{encoder_corrected:.6f}"

            # Physical angle from the current logged raw motor-5 scaling.
            row["motor5_target_raw_degree"] = f"{(target_raw / MOTOR5_COUNTS_PER_DEGREE_RAW):.9f}"
            row["motor5_encoder_raw_degree"] = f"{(encoder_raw / MOTOR5_COUNTS_PER_DEGREE_RAW):.9f}"

            # Same physical angle after multiplying count by 100 and using the 1~4 axis count scale.
            row["motor5_target_corrected_degree_check"] = (
                f"{(target_corrected / MOTOR1_TO_4_COUNTS_PER_DEGREE):.9f}"
            )
            row["motor5_encoder_corrected_degree_check"] = (
                f"{(encoder_corrected / MOTOR1_TO_4_COUNTS_PER_DEGREE):.9f}"
            )
            row["motor5_scale_factor_used"] = f"{MOTOR5_SCALE_FACTOR:.1f}"

            writer.writerow(row)

    print(f"Generated: {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
