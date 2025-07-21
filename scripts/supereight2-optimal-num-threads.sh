#!/bin/sh
# SPDX-FileCopyrightText: 2025 Mobile Robotics Lab, ETH Zurich
# SPDX-FileCopyrightText: 2025 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause

# Run a supereight2 pipeline using different numbers of OpenMP threads and print
# the results in TSV. The results depend on the computer the script is run on.
# Values of OMP_NUM_THREADS larger than 10 or so can cause supereight2 to
# actually be slower.

set -eu

if [ "$#" -lt 2 ]
then
	printf 'Usage: %s PROGAM CONFIG\n' "${0##*/}" >&2
	exit 2
fi

export LC_ALL=C # For consistent time(1) output.

printf 'OMP_NUM_THREADS\tTime (s)\n'
n=1
n_max=$(nproc) # The number of threads available on the system.
while [ "$n" -le "$n_max" ]
do
	export OMP_NUM_THREADS="$n"
	# Time the run, ignore stdout and parse the time(1) output from stderr.
	t=$({ time -p "$@" 2>&1 >/dev/null; } | awk '/^real / && NF == 2 { print $2 }')
	printf '%s\t%s\n' "$n" "$t"
	n=$((n + 1))
done
