#!/bin/sh
# SPDX-FileCopyrightText: 2024 Smart Robotics Lab, Imperial College London, Technical University of Munich
# SPDX-FileCopyrightText: 2024-2025 Sotiris Papatheodorou
# SPDX-License-Identifier: BSD-3-Clause

# Show the supereight2 scale colours by parsing the source code. Assumes a
# reasonable formatting of the code.
#
# Usage:
# scale-colours.sh
#   Show the scale colours on a terminal using ANSI escape sequences.
# scale-colours.sh html
#   Show the scale colours as an HTML table on standard output.

awk -v html="${1:-0}" '
BEGIN {
	if (html)
		print "<table>\n<tr><th>Scale</th><th>Colour</th></tr>"
}

/^namespace colours \{$/, /^}( \/\/ namespace colours)?$/ {
	if ($0 ~ "\\{ *[0-9]+, *[0-9]+, *[0-9]+ *}") {
		sub("^.*\\{", "")
		sub("}.*$", "")
		gsub(", *", " ")
		if (html)
			printf "<tr><td>%d</td><td style=\"color:#%x%x%x\">████</td></tr>\n", scale++, $1, $2, $3
		else
			printf "\033[48;2;%d;%d;%dm        \033[m scale %d\n", $1, $2, $3, scale++
	}
}

END {
	if (html)
		print "</table>"
}
' "$(dirname "$0")/../include/se/common/colour_utils.hpp"
