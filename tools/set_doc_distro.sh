#!/usr/bin/env bash
#
# Rewrites the ROS distro in every docs.ros.org link of the documentation pages.
#
# The distro cannot be resolved at documentation build time: rosdoc2 has no notion of one
# and injects nothing a Markdown page could read, and docs.ros.org has no distro-agnostic
# URL for message types. So the distro is baked into the links, and this script is how it
# gets flipped on a per-distro branch.
#
# Usage:
#   tools/set_doc_distro.sh jazzy [path ...]
#
# Rewrites every *.md under the given paths, defaulting to the whole repository. Note that
# it cannot tell a link meant to track the branch from one that deliberately names a
# distro -- the root README's "Humble minimum required" note, for instance -- so pass
# explicit paths when that matters.
#
# On a distro branch, treat the documentation as *derived* rather than hand-edited, and
# the merge from the development branch never conflicts:
#
#   git merge ros2
#   git checkout ros2 -- '*/doc' '*/README.md'   # always take the upstream pages
#   tools/set_doc_distro.sh humble               # then re-stamp the distro
#   git commit -a
#
set -euo pipefail

distro=${1:-}
if [ -z "$distro" ]; then
	echo "usage: $(basename "$0") <distro>   e.g. $(basename "$0") jazzy" >&2
	exit 1
fi

# An explicit list rather than a wildcard: docs.ros.org also serves /en/api/, which is the
# legacy ROS 1 message documentation and must not be rewritten into a distro.
known='humble|iron|jazzy|kilted|lyrical|rolling'

root=$(cd "$(dirname "$0")/.." && pwd)
shift || true
paths=("$@")
[ ${#paths[@]} -eq 0 ] && paths=("$root")

mapfile -t files < <(grep -rlE "docs\.ros\.org/en/($known)/" --include='*.md' "${paths[@]}")

if [ ${#files[@]} -eq 0 ]; then
	echo "no documentation pages with a distro link found" >&2
	exit 0
fi

sed -i -E "s#docs\.ros\.org/en/($known)/#docs.ros.org/en/$distro/#g" "${files[@]}"
echo "set distro to '$distro' in ${#files[@]} file(s):"
printf '  %s\n' "${files[@]#$root/}"
