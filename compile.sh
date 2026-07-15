#!/usr/bin/env bash

set -euo pipefail

usage() {
    echo "Usage: $0 <sn_prefix> <start_sn> <count> <lis2>"
    echo "  sn_prefix: alphanumeric sensor serial number prefix"
    echo "  start_sn:  first numeric sensor serial number suffix"
    echo "  count:     number of consecutive sensor firmwares to build"
    echo "  lis2:      1 when LIS2DH12 is fitted, otherwise 0"
    echo "Example:"
    echo "  $0 26SH00 101 9 0    # builds 26SH00101 through 26SH00109"
    echo "  $0 26SH00 201 5 1    # builds 26SH00201 through 26SH00205"
}

if [[ $# -ne 4 ]]; then
    usage
    exit 2
fi

sn_prefix="$1"
start_sn="$2"
count="$3"
lis2="$4"

if [[ ! "$sn_prefix" =~ ^[[:alnum:]]+$ ]]; then
    echo "Error: sn_prefix must contain letters and digits only: $sn_prefix" >&2
    exit 2
fi

if [[ ! "$start_sn" =~ ^[0-9]+$ ]]; then
    echo "Error: start_sn must contain digits only: $start_sn" >&2
    exit 2
fi

if [[ ! "$count" =~ ^[0-9]+$ ]] || (( 10#$count == 0 )); then
    echo "Error: count must be a positive integer: $count" >&2
    exit 2
fi

if [[ "$lis2" != "0" && "$lis2" != "1" ]]; then
    echo "Error: lis2 must be 0 or 1: $lis2" >&2
    exit 2
fi

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
start_sn_value=$((10#$start_sn))
sn_suffix_width=${#start_sn}
count_value=$((10#$count))
end_sn=$((start_sn_value + count_value - 1))

cd "$project_dir"

printf -v first_full_sn '%s%0*d' "$sn_prefix" "$sn_suffix_width" "$start_sn_value"
printf -v last_full_sn '%s%0*d' "$sn_prefix" "$sn_suffix_width" "$end_sn"

echo "Batch building ${count_value} sensors: ${first_full_sn}..${last_full_sn}, LIS2=${lis2}"

for ((sensor_sn = start_sn_value; sensor_sn <= end_sn; ++sensor_sn)); do
    printf -v sensor_suffix '%0*d' "$sn_suffix_width" "$sensor_sn"
    full_sn="${sn_prefix}${sensor_suffix}"
    build_dir="build-${full_sn}"
    echo "Building sensor SN=${full_sn} in ${build_dir}"

    python3 "$IDF_PATH/tools/idf.py" \
        -B "$build_dir" \
        "-DSN=${full_sn}" \
        "-DLIS2=${lis2}" \
        build

    echo "Firmware: ${project_dir}/${build_dir}/sentinel.bin"
done

echo "Batch build completed: ${first_full_sn}..${last_full_sn}, LIS2=${lis2}"
