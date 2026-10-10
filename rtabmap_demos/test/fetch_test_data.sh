#!/usr/bin/env bash
# Fetch the bags the demo playback tests replay, as listed in data/manifest.txt.
# Same manifest format as rtabmap's scripts/fetch_test_data.sh. An entry with a
# fourth column is a .zip, which is extracted into the data directory and then
# removed, its SHA-256 being the one of the file it holds. Skips files that are
# already present and whose SHA-256 matches the manifest.
#
# Usage: fetch_test_data.sh [basename...]
#   With no argument, fetches every entry of the manifest; otherwise only the named
#   ones (e.g. stereo_outdoorA_bag.zip), so a job can fetch just what it replays.
#
# The data goes next to the manifest, unless RTABMAP_DEMOS_TEST_DATA names another
# directory (a CI cache, or a disk with room: the bags are several GB). The tests
# read the same variable.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MANIFEST="${SCRIPT_DIR}/data/manifest.txt"
DEST_DIR="${RTABMAP_DEMOS_TEST_DATA:-${SCRIPT_DIR}/data}"

if [[ ! -f "$MANIFEST" ]]; then
	echo "Error: manifest not found at $MANIFEST" >&2
	exit 1
fi
mkdir -p "$DEST_DIR"

sha256_of() { sha256sum "$1" | awk '{print $1}'; }

# unzip where there is one, python3's zipfile where there is not (slim ROS images).
extract_zip() {
	local archive="$1"
	if command -v unzip >/dev/null 2>&1; then
		unzip -o -q "$archive" -d "$DEST_DIR"
	else
		python3 -m zipfile -e "$archive" "$DEST_DIR"
	fi
}

verify_sha() {
	local file="$1" expected="$2"
	if [[ "$expected" == "TODO_FILL_SHA256" || -z "$expected" ]]; then
		echo "  (no SHA in manifest yet for $file; skipping integrity check)" >&2
		echo "  sha256: $(sha256_of "$file")" >&2
		return 0
	fi
	local actual
	actual="$(sha256_of "$file")"
	if [[ "$actual" != "$expected" ]]; then
		echo "  SHA mismatch for $file: expected $expected, got $actual" >&2
		return 1
	fi
}

wanted() {
	local name="$1"
	[[ $# -eq 0 || ${#REQUESTED[@]} -eq 0 ]] && return 0
	for r in "${REQUESTED[@]}"; do
		[[ "$r" == "$name" ]] && return 0
	done
	return 1
}

REQUESTED=("$@")
found=()

while IFS=$'\t' read -r name url expected_sha extracted; do
	name="${name%$'\r'}"
	url="${url%$'\r'}"
	expected_sha="${expected_sha%$'\r'}"
	extracted="${extracted-}"
	extracted="${extracted%$'\r'}"
	# Skip comments and blank lines.
	[[ -z "${name// }" || "$name" =~ ^# ]] && continue
	wanted "$name" || continue
	found+=("$name")

	target="$DEST_DIR/$name"
	# An archive is not kept once it is extracted, so what has to be there, and
	# what the sha is of, is the file it held.
	kept="${extracted:-$name}"
	if [[ -f "$DEST_DIR/$kept" ]] && verify_sha "$DEST_DIR/$kept" "$expected_sha" 2>/dev/null; then
		echo "Already up-to-date: $kept"
		continue
	fi

	echo "Fetching $name <- $url"
	# -L follows the redirect to the release storage, -f fails on HTTP errors, -S
	# shows errors on stderr. Retries cover the odd dropped connection on a GB file,
	# and -C - resumes a .partial an interrupted run left behind; the SHA check
	# below catches a partial that does not belong to this file.
	curl -fsSL --retry 3 --retry-delay 5 -C - "$url" -o "$target.partial"

	if [[ -z "$extracted" ]]; then
		if ! verify_sha "$target.partial" "$expected_sha"; then
			rm -f "$target.partial"
			exit 1
		fi
		mv -f "$target.partial" "$target"
	else
		mv -f "$target.partial" "$target"
		echo "  Extracting $name"
		if ! extract_zip "$target"; then
			rm -f "$target"
			exit 1
		fi
		# The archive has served its purpose, and these are large files.
		rm -f "$target"
		if [[ ! -f "$DEST_DIR/$extracted" ]]; then
			echo "  $name does not hold $extracted" >&2
			exit 1
		fi
		if ! verify_sha "$DEST_DIR/$extracted" "$expected_sha"; then
			rm -f "$DEST_DIR/$extracted"
			exit 1
		fi
		echo "  Extracted $extracted ($(du -h "$DEST_DIR/$extracted" | cut -f1))"
	fi
done < "$MANIFEST"

for r in "${REQUESTED[@]}"; do
	if [[ ! " ${found[*]} " == *" $r "* ]]; then
		echo "Error: $r is not in $MANIFEST" >&2
		exit 1
	fi
done

echo "Test data ready under $DEST_DIR"
