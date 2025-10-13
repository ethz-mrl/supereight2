#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Mobile Robotics Lab, ETH Zurich
# SPDX-FileCopyrightText: 2025 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause

import argparse
import os.path
import re
import statistics
import subprocess
import sys

from typing import Any, List, Optional


class TSV:
    def __init__(self, filename=None):
        self.nameline = None
        self.records = []
        if filename:
            self.read(filename)

    def __bool__(self) -> bool:
        return bool(self.nameline)

    def __str__(self) -> str:
        records = [self.nameline] + self.records
        return "\n".join(["\t".join([str(x) for x in r]) for r in records])

    def read(self, filename: str):
        nameline = None
        records = []
        try:
            with open(filename) as f:
                for i, line in enumerate(f):
                    record = line.rstrip("\n").split("\t")
                    if i == 0:
                        nameline = record
                    else:
                        records.append(record)
        except OSError:
            return None
        if nameline:
            self.nameline = nameline
            self.records = records

    def filter(self, func):
        indices = [i for i, x in enumerate(self.nameline) if func(x)]
        self.nameline = [self.nameline[i] for i in indices]
        for i, record in enumerate(self.records):
            self.records[i] = [record[j] for j in indices]

    def convert(self, t):
        def conv(x):
            try:
                return t(x)
            except ValueError:
                return t(0)
        for i, record in enumerate(self.records):
            self.records[i] = [conv(x) for x in record]

    def sum(self) -> List[Any]:
        if not self.records:
            return []
        s = self.records[0].copy()
        for record in self.records[1:]:
            for i, field in enumerate(record):
                s[i] += field
        return s

    def mean_stdev(self) -> List[str]:
        if not self.records:
            return []
        s = []
        for f in range(len(self.records[0])):
            d = []
            for record in self.records:
                d.append(record[f])
            mean = statistics.mean(d)
            stdev = 0 if len(d) < 2 else statistics.stdev(d)
            s.append("{:f}±{:f}".format(mean, stdev))
        return s


def printerr(*args, **kwargs) -> None:
    """Print to stderr prefixed with the program name"""
    error_prefix = os.path.basename(sys.argv[0]) + ": error:"
    print(error_prefix, *args, file=sys.stderr, **kwargs)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="""Run all combinations of
            supereight2 executables and config files and print timing statistics
            for each combination on standard output as TSV. Each combination is
            run multiple times and produces a single TSV line that contains the
            mean and standard deviation of the cumulative per-run timings.""")
    parser.add_argument("-l", "--logfile", metavar="FILE", default="-",
            help="write the output to FILE instead of standard output")
    parser.add_argument("-n", "--num-runs", metavar="N", type=int, default=10,
            help="the number of times to run each executable/config combination")
    parser.add_argument("-e", "--executable", metavar="EXE", action="append",
            required=True,
            help="""a supereight2 executable to benchmark (can be supplied
            multiple times)""")
    parser.add_argument("config", metavar="CONFIG", nargs="+",
            help="a supereight2 config to pass to the executables")
    args = parser.parse_args()
    if args.num_runs < 1:
        printerr("argument --num-runs: value must be greater than 0")
        sys.exit(2)
    return args


def log_path(config: str) -> Optional[str]:
    with open(config) as f:
        for line in f:
            match = re.fullmatch(r'log_file: *"(.+)"', line.strip())
            if match:
                # Compute the log file path in the same way as supereight2.
                log_file = os.path.expanduser(match.group(1))
                if not os.path.isabs(log_file):
                    log_file = os.path.join(os.path.dirname(config), log_file)
                return log_file
    return None


if __name__ == "__main__":
    try:
        args = parse_args()
        stats = TSV()
        for e in args.executable:
            for c in args.config:
                logfile = log_path(c)
                if not logfile:
                    printerr("no log_file key found in config " + c)
                    sys.exit(1)
                combination_stats = TSV()
                for i in range(args.num_runs):
                    result = subprocess.run([e, c], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    if result.returncode != 0:
                        printerr("command failed: " + e + " " +c)
                        sys.exit(1)
                    tsv = TSV(logfile)
                    if not tsv:
                        printerr("couldn't read log file " + logfile)
                        sys.exit(1)
                    tsv.convert(float)
                    if not combination_stats:
                        combination_stats.nameline = tsv.nameline
                    combination_stats.records.append(tsv.sum())
                if not stats:
                    stats.nameline = ["executable", "config"] + combination_stats.nameline
                stats.records.append([e, c] + combination_stats.mean_stdev())
        if args.logfile == '-':
            print(stats)
        else:
            with open(args.logfile, 'w') as f:
                f.write(str(stats) + '\n')
    except (BrokenPipeError, KeyboardInterrupt):
        pass
